using System;
using System.Collections.Generic;
using System.Text.Json.Nodes;
using System.Threading.Tasks;
using Godot;

namespace DominoSim;

/// <summary>
/// Executes one program step (mission.json format) on the sim robot. Like the real robot with
/// drive.absolute_heading, turn/curve aim at a heading counted from the start (RefHeading), so
/// turn errors do not add up and every drive holds that heading.
/// Supported: drive, drive_time, turn, curve, wait, arm (rot / slot / nudge, our mechanism), macro, unload (= macro unload), log.
/// Others (pick_row, …) are skipped.
/// </summary>
public sealed class StepExecutor
{
    readonly Robot _r;
    readonly Motion _m;
    readonly Func<double> _now;
    public Action<string> Log = _ => { };
    /// <summary>Target heading in sim radians (+ = clockwise).</summary>
    public float RefHeading;
    /// <summary>Macros for "macro" / "unload" steps (mission.json macros, overridden by the program's own).</summary>
    public JsonObject Macros = DriveConfig.Macros;

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
            "arm" => "arm" + (st["rot"] != null ? $" {S(st, "rot", "")}" : "") + (st["slot"] != null ? $" slot {S(st, "slot", "")}" : "")
                     + (st["nudge"] != null ? " nudge" : ""),
            "macro" => $"macro {S(st, "name", "?")}",
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
            case "arm":
            {
                var mech = _r.Mech;
                if (mech == null) return false;
                double t = 0;
                string rot = S(st, "rot", null)?.ToUpperInvariant(), slot = S(st, "slot", null)?.ToUpperInvariant();
                if (slot == "TOGGLE") slot = mech.Slot == "LEFT" ? "RIGHT" : "LEFT";
                if (slot != null && slot != mech.Slot)
                {
                    // Shift: motor C turns past the end rotation of the slot (as mechanism.py, robot.arm.transitions).
                    string end = slot == "LEFT" ? "CW" : "CCW";
                    float deg = DriveConfig.Transitions.GetValueOrDefault($"{mech.Rot},{mech.Slot}>{end},{slot}", 5000f);
                    deg = Mathf.Abs(deg) + DriveConfig.ShiftExtra.GetValueOrDefault(slot, 0f) + F(st, "extra", 0);
                    t += deg / DriveConfig.ShiftSpeed;
                    mech.SetState(end, slot);
                }
                if (rot != null && rot != mech.Rot)
                {
                    t += Mathf.Abs(Pos(rot) - Pos(mech.Rot)) / DriveConfig.ArmSpeed;
                    mech.SetState(rot, null);
                }
                if (st["nudge"] is JsonValue nv && !(nv.TryGetValue(out bool nb) && !nb))
                {
                    string end = mech.Slot == "LEFT" ? "CW" : "CCW";
                    float extra = nv.TryGetValue(out double nd) ? (float)nd : DriveConfig.NudgeDeg;
                    t += (Mathf.Abs(Pos(end) - Pos(mech.Rot)) + extra) / DriveConfig.ShiftSpeed + 0.3;
                    mech.SetState(end, null);
                }
                await _m.Wait(t);
                return true;
            }
            case "unload":
                return await RunMacro(S(st, "macro", "unload"));
            case "macro":
                return await RunMacro(S(st, "name", ""));
            case "log":
                Log("    log: " + S(st, "msg", ""));
                return true;
            default:
                return false;
        }
    }

    static float Pos(string rot) => DriveConfig.RotPositions.GetValueOrDefault(rot ?? "NONE", 0f);

    int _depth;

    async Task<bool> RunMacro(string name)
    {
        if (Macros?[name] is not JsonArray steps || _depth > 8) { Log($"    makro '{name}' není"); return false; }
        _depth++;
        foreach (var o in steps)
            if (o is JsonObject st && st["cmd"] != null)
                if (!await Exec(st)) Log($"    ({StepExecutor.Describe(st)} v makru přeskočeno)");
        _depth--;
        return true;
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
