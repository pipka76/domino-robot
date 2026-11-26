using System;
using System.Text.Json.Nodes;
using System.Threading.Tasks;
using Godot;

namespace DominoSim;

/// <summary>
/// Executes one program step (mission.json format) on the sim robot. Like the real robot with
/// drive.absolute_heading, turn/curve aim at a heading counted from the start (RefHeading), so
/// turn errors do not add up and every drive holds that heading.
/// Supported: drive, drive_time, turn, curve, wait, intake (sim only), unload, log. Others are skipped.
/// </summary>
public sealed class StepExecutor
{
    readonly Robot _r;
    readonly Motion _m;
    readonly Func<double> _now;
    public Action<string> Log = _ => { };
    /// <summary>Target heading in sim radians (+ = clockwise).</summary>
    public float RefHeading;

    public StepExecutor(Robot r, Motion m, Func<double> now)
    {
        _r = r;
        _m = m;
        _now = now;
        RefHeading = r.Theta;
    }

    static float F(JsonObject st, string k, float def)
    {
        if (st[k] is not JsonValue v) return def;
        if (v.TryGetValue(out double d)) return (float)d;
        if (v.TryGetValue(out int i)) return i;
        if (v.TryGetValue(out float f)) return f;
        return def;
    }

    static string S(JsonObject st, string k, string def) =>
        st[k] is JsonValue v && v.TryGetValue(out string x) ? x : def;

    public static string Describe(JsonObject st)
    {
        string cmd = S(st, "cmd", "?");
        return cmd switch
        {
            "drive" => $"drive {F(st, "mm", 0):+0;-0} mm" + (st["speed"] != null ? $" @{F(st, "speed", 0):0}" : ""),
            "turn" => $"turn {F(st, "deg", 0):+0;-0}°" + (st["speed"] != null ? $" @{F(st, "speed", 0):0}" : ""),
            "curve" => $"curve {F(st, "deg", 0):+0;-0}°" + (st["inner"] != null ? $" ({F(st, "inner", 0):0}/{F(st, "outer", 0):0})" : ""),
            "drive_time" => $"drive_time {F(st, "s", 0):0.0#} s",
            "wait" => $"wait {F(st, "s", 0):0.0#} s",
            "intake" => $"intake {S(st, "arm", "NONE")}",
            _ => cmd,
        };
    }

    /// <summary>Returns false when the step was skipped (not supported in the sim).</summary>
    public async Task<bool> Exec(JsonObject st)
    {
        string cmd = S(st, "cmd", null);
        switch (cmd)
        {
            case "drive":
            {
                float mm = F(st, "mm", 0);
                float v = DriveConfig.Ms(Mathf.Abs(F(st, "speed", DriveConfig.Speed)));
                float vmin = Mathf.Min(DriveConfig.Ms(DriveConfig.MinSpeed), v);
                bool hold = st["hold"] is not JsonValue h || !h.TryGetValue(out bool hb) || hb;
                float heading = hold ? RefHeading : _r.Theta;
                if (st["until"] != null) Log("    (until se v simulaci ignoruje – jede celou vzdálenost)");
                await _m.DriveLine(_r.Pos2, heading, mm * 0.001f, v, vmin: vmin);
                float tail = F(st, "tail_s", 0);
                if (tail > 0) await DriveFor(tail, Mathf.Sign(mm) * DriveConfig.Ms(Mathf.Abs(F(st, "tail_speed", DriveConfig.MinSpeed))));
                return true;
            }
            case "drive_time":
                await DriveFor(F(st, "s", 0), DriveConfig.Ms(F(st, "speed", DriveConfig.Speed)));
                return true;
            case "turn":
            {
                RefHeading = Geo.Wrap(RefHeading - Mathf.DegToRad(F(st, "deg", 0)));
                float wmax = DriveConfig.Ms(F(st, "speed", DriveConfig.TurnSpeed)) / Robot.HalfTrack;
                await _m.TurnTo(RefHeading, 3 + Mathf.Abs(F(st, "deg", 0)) / Mathf.RadToDeg(wmax) * 2, wmax);
                return true;
            }
            case "curve":
                await Curve(F(st, "deg", 0), F(st, "inner", DriveConfig.CurveInner), F(st, "outer", DriveConfig.CurveOuter),
                    F(st, "s_per_90", DriveConfig.CurveSPer90));
                return true;
            case "wait":
                await _m.Wait(F(st, "s", 0));
                return true;
            case "intake":
                _r.Intake.SetArm(S(st, "arm", "NONE").ToUpperInvariant() switch
                {
                    "LEFT" => Intake.ArmMode.Left,
                    "RIGHT" => Intake.ArmMode.Right,
                    _ => Intake.ArmMode.Retracted,
                });
                return true;
            case "unload":
                _r.Intake.Release();
                return true;
            case "log":
                Log("    log: " + S(st, "msg", ""));
                return true;
            default:
                return false;
        }
    }

    async Task DriveFor(float seconds, float v)
    {
        double end = _now() + seconds;
        while (_now() < end)
        {
            _r.SetTwist(v, 0);
            await _m.Tick();
        }
        await _m.StopAndSettle();
    }

    /// <summary>Arc: inner wheel slow, outer fast (deg/s), until the heading reaches RefHeading + deg (+ = left).</summary>
    async Task Curve(float deg, float inner, float outer, float sPer90)
    {
        float left = deg > 0 ? inner : outer, right = deg > 0 ? outer : inner;
        // Remaining angle measured from the reference heading, unwrapped while turning.
        float remaining = Mathf.DegToRad(Mathf.Abs(deg)) - Mathf.Sign(deg) * -Geo.Wrap(_r.Theta - RefHeading);
        RefHeading = Geo.Wrap(RefHeading - Mathf.DegToRad(deg));
        float prev = _r.Theta;
        double end = _now() + sPer90 * Mathf.Abs(deg) / 90f * 1.5f + 1;
        while (remaining > 0)
        {
            if (_now() > end) { Log("    curve: timeout"); break; }
            _r.SetWheelSpeeds(DriveConfig.Ms(left), DriveConfig.Ms(right));
            await _m.Tick();
            float d = Geo.Wrap(_r.Theta - prev);   // + = clockwise
            prev = _r.Theta;
            remaining -= deg > 0 ? -d : d;
        }
        await _m.StopAndSettle();
    }
}
