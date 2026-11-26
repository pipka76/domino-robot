using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json.Nodes;
using System.Threading.Tasks;
using Godot;

namespace DominoSim;

/// <summary>
/// Mode 2: the user drives with the keyboard, every movement is recorded as one step
/// (drive / turn / curve in the mission.json format). Pauses between movements are not recorded,
/// consecutive movements of the same kind are merged into one step.
/// </summary>
public partial class ManualDriver : SimDriver
{
    public Main MainNode;
    public readonly List<JsonObject> Steps = new();
    public string Status = "";
    public string SavedPath;
    /// <summary>Time the robot was actually moving (= what the recorded program takes, roughly).</summary>
    public double MoveTime { get; private set; }

    enum Kind { None, Drive, Turn, Curve }
    readonly record struct Seg(Kind Kind, int Sign, bool Slow);

    Seg _seg;                    // active segment (Kind.None = idle)
    bool _settling;
    int _still;
    double _segStart, _settleStart;
    Vector2 _pos0;
    float _turned, _prevTheta;
    Task _busy;                  // replay or exact turn in progress
    bool _replaying;

    public override void _Ready()
    {
        Name = "ManualDriver";
        base._Ready();
        Exec = new StepExecutor(Robot, M, () => SimTime) { Log = Log };
    }

    public override void _PhysicsProcess(double delta)
    {
        if (SettleTicks > 0)
        {
            if (--SettleTicks == 0)
            {
                Exec.RefHeading = Robot.Theta;
                if (Session.PendingSteps is { Count: > 0 } pending)
                {
                    Session.PendingSteps = null;
                    RunNoContext(() => _busy = Replay(pending));
                }
                Log("Ruční jízda: šipky/WASD, Shift = pomalu. Každý pohyb se zapíše jako krok.");
            }
            return;
        }
        SimTime += delta;
        RunNoContext(Ticker.Pump);
        if (_busy != null)
        {
            if (!_busy.IsCompleted) return;
            if (_busy.IsFaulted) GD.PushError(_busy.Exception?.ToString());
            _busy = null;
        }
        ManualTick();
    }

    async Task Replay(List<JsonObject> steps)
    {
        _replaying = true;
        MainNode?.SetSpeed(8);
        Status = $"přehrávám {steps.Count} kroků…";
        foreach (var st in steps)
        {
            await Exec.Exec(st);
            Steps.Add(st);
        }
        MainNode?.SetSpeed(1);
        _replaying = false;
        MoveTime = SimTime;
        Status = "záznam pokračuje";
    }

    // ---------------------------------------------------------------- keyboard driving

    static bool K(Key a, Key b = Key.None) => Input.IsPhysicalKeyPressed(a) || (b != Key.None && Input.IsPhysicalKeyPressed(b));

    Seg ReadInput()
    {
        bool fwd = K(Key.W, Key.Up), back = K(Key.S, Key.Down), left = K(Key.A, Key.Left), right = K(Key.D, Key.Right);
        bool slow = K(Key.Shift);
        if (fwd && !back) return left != right ? new Seg(Kind.Curve, left ? 1 : -1, slow) : new Seg(Kind.Drive, 1, slow);
        if (back && !fwd) return new Seg(Kind.Drive, -1, slow);
        if (!fwd && !back && left != right) return new Seg(Kind.Turn, left ? 1 : -1, slow);
        return new Seg(Kind.None, 0, false);
    }

    void ManualTick()
    {
        float th = Robot.Theta;
        if (_seg.Kind != Kind.None)
        {
            _turned += Geo.Wrap(th - _prevTheta);
            _prevTheta = th;
        }
        if (_settling)
        {
            _still = Mathf.Abs(Robot.Speed) < 0.003f && Mathf.Abs(Robot.YawRate) < 0.03f ? _still + 1 : 0;
            if (_still >= 3 || SimTime - _settleStart > 1.0) FinishSegment();
            return;
        }
        var inp = ReadInput();
        if (_seg.Kind == Kind.None)
        {
            if (inp.Kind == Kind.None) return;
            _seg = inp;
            _segStart = SimTime;
            _pos0 = Robot.Pos2;
            _turned = 0;
            _prevTheta = th;
        }
        else if (inp != _seg)
        {
            Robot.Stop();
            _settling = true;
            _still = 0;
            _settleStart = SimTime;
            return;
        }
        Command(_seg, th);
    }

    void Command(Seg s, float th)
    {
        switch (s.Kind)
        {
            case Kind.Drive:
            {
                float v = s.Sign * DriveConfig.Ms(s.Slow ? DriveConfig.SlowSpeed : DriveConfig.Speed);
                float w = Mathf.Clamp(6f * Geo.Wrap(Exec.RefHeading - th), -1.5f, 1.5f);
                Robot.SetTwist(v, w);
                break;
            }
            case Kind.Turn:
            {
                float w = DriveConfig.Ms(s.Slow ? DriveConfig.SlowTurnSpeed : DriveConfig.TurnSpeed) / Robot.HalfTrack;
                Robot.SetTwist(0, -s.Sign * w);   // + sign = left = counter-clockwise
                break;
            }
            case Kind.Curve:
            {
                float inner = s.Slow ? DriveConfig.PivotInner : DriveConfig.CurveInner;
                float outer = s.Slow ? DriveConfig.PivotOuter : DriveConfig.CurveOuter;
                float l = s.Sign > 0 ? inner : outer, r = s.Sign > 0 ? outer : inner;
                Robot.SetWheelSpeeds(DriveConfig.Ms(l), DriveConfig.Ms(r));
                break;
            }
        }
    }

    void FinishSegment()
    {
        var s = _seg;
        _seg = default;
        _settling = false;
        MoveTime += SimTime - _segStart;
        switch (s.Kind)
        {
            case Kind.Drive:
            {
                int mm = Mathf.RoundToInt((Robot.Pos2 - _pos0).Dot(Geo.Dir(Exec.RefHeading)) * 1000f);
                if (Mathf.Abs(mm) < 2) return;
                var st = new JsonObject { ["cmd"] = "drive", ["mm"] = mm };
                if (s.Slow) st["speed"] = (int)DriveConfig.SlowSpeed;
                var prev = Steps.LastOrDefault();
                if (prev != null && Cmd(prev) == "drive" && Math.Sign(prev["mm"].GetValue<int>()) == Math.Sign(mm)
                    && prev["speed"]?.GetValue<int>() == st["speed"]?.GetValue<int>() && prev.Count == st.Count)
                    prev["mm"] = prev["mm"].GetValue<int>() + mm;
                else Add(st);
                break;
            }
            case Kind.Turn:
            case Kind.Curve:
            {
                // Angle from the reference heading (absolute_heading on the real robot), + = left.
                float startOff = Geo.Wrap(_prevTheta - _turned - Exec.RefHeading);
                int deg = Mathf.RoundToInt(-Mathf.RadToDeg(startOff + _turned));
                if (deg == 0) return;
                Exec.RefHeading = Geo.Wrap(Exec.RefHeading - Mathf.DegToRad(deg));
                string cmd = s.Kind == Kind.Turn ? "turn" : "curve";
                var st = new JsonObject { ["cmd"] = cmd, ["deg"] = deg };
                if (s.Slow && s.Kind == Kind.Turn) st["speed"] = (int)DriveConfig.SlowTurnSpeed;
                if (s.Slow && s.Kind == Kind.Curve) { st["inner"] = (int)DriveConfig.PivotInner; st["outer"] = (int)DriveConfig.PivotOuter; }
                AddOrMergeAngle(st);
                break;
            }
        }
    }

    static string Cmd(JsonObject st) => st["cmd"]?.GetValue<string>();

    void AddOrMergeAngle(JsonObject st)
    {
        var prev = Steps.LastOrDefault();
        bool same = prev != null && Cmd(prev) == Cmd(st) && prev.Count == st.Count
                    && st.Where(kv => kv.Key != "deg").All(kv => prev[kv.Key]?.ToJsonString() == kv.Value?.ToJsonString());
        int deg = st["deg"].GetValue<int>();
        // Merge only same direction for curves (a curve back is a different path); turns may cancel out.
        if (same && (Cmd(st) == "turn" || Math.Sign(prev["deg"].GetValue<int>()) == Math.Sign(deg)))
        {
            int sum = prev["deg"].GetValue<int>() + deg;
            if (sum == 0) Steps.RemoveAt(Steps.Count - 1);
            else prev["deg"] = sum;
        }
        else Add(st);
    }

    void Add(JsonObject st)
    {
        Steps.Add(st);
        Log($"  + {StepExecutor.Describe(st)}");
    }

    bool Idle => _busy == null && _seg.Kind == Kind.None && !_settling && SettleTicks == 0;

    // ---------------------------------------------------------------- discrete commands (keys)

    public void ExactTurn(int deg)
    {
        if (!Idle) return;
        var st = new JsonObject { ["cmd"] = "turn", ["deg"] = deg };
        double t0 = SimTime;
        RunNoContext(() => _busy = Do(st, t0));
    }

    async Task Do(JsonObject st, double t0)
    {
        await Exec.Exec(st);
        MoveTime += SimTime - t0;
        AddOrMergeAngle((JsonObject)st.DeepClone());
    }

    public void SetArm(string arm)
    {
        if (!Idle) return;
        var st = new JsonObject { ["cmd"] = "intake", ["arm"] = arm };
        _ = Exec.Exec(st);
        if (Steps.LastOrDefault() is { } prev && Cmd(prev) == "intake") Steps.RemoveAt(Steps.Count - 1);
        Add(st);
    }

    public void Unload()
    {
        if (!Idle) return;
        var st = new JsonObject { ["cmd"] = "unload" };
        _ = Exec.Exec(st);
        Add(st);
    }

    /// <summary>Undo = restart and replay all steps but the last one (fast).</summary>
    public void Undo()
    {
        if (!Idle || _replaying || Steps.Count == 0) return;
        Session.PendingSteps = Steps.Take(Steps.Count - 1).Select(s => (JsonObject)s.DeepClone()).ToList();
        GetTree().ReloadCurrentScene();
    }

    public ProgramFile ToProgram()
    {
        float startHeading = Mathf.RadToDeg(Layout.RobotStartHeading);
        var p = new ProgramFile { Steps = Steps.Select(s => (JsonObject)s.DeepClone()).ToList() };
        var start = Session.Program?.Start;
        p.Start = start ?? (Layout.RobotStart.X * 1000f, Layout.RobotStart.Y * 1000f, startHeading);
        return p;
    }

    public string Save()
    {
        if (Steps.Count == 0) { Status = "není co uložit"; return null; }
        var p = ToProgram();
        string path = SavedPath ?? Path.Combine(ProgramFile.ProgramsDir, $"zaznam_{DateTime.Now:yyyyMMdd_HHmmss}.json");
        SavedPath = p.Save(path);
        string json = ProgramFile.StepsToJson(p.Steps, "");
        DisplayServer.ClipboardSet(json);
        GD.Print("Kroky (zkopírováno do schránky):\n" + json);
        Status = $"uloženo: {SavedPath} (kroky i ve schránce)";
        Log("Uloženo do " + SavedPath);
        return SavedPath;
    }

    public string StatusText()
    {
        string seg = _replaying ? "přehrávám záznam…" : _busy != null ? "otáčím…" : _settling ? "zastavuji…" : _seg.Kind != Kind.None ? $"{_seg.Kind} {(_seg.Sign > 0 ? "+" : "-")}{(_seg.Slow ? " pomalu" : "")}" : "stojí";
        float hdg = -Mathf.RadToDeg(Geo.Wrap(Robot.Theta - Layout.RobotStartHeading));
        return $"MÓD 2 – ruční jízda se záznamem   kroků: {Steps.Count}   čas jízdy (bez čekání) ≈ {MoveTime:0.0} s\n" +
               $"Robot: {seg}   x = {Robot.Pos2.X * 1000:0} mm, z = {Robot.Pos2.Y * 1000:0} mm, směr od startu {hdg:+0;-0}° (+ = doleva)\n" +
               $"Zásobník: {CarriedText()}   rameno: {Robot.Intake.Arm}\n" +
               Scorer.Summary(Dominos, Layout) + (Status != "" ? $"\n{Status}" : "");
    }

    public string SideText()
    {
        var sb = new StringBuilder("Zaznamenané kroky:\n");
        int from = Math.Max(0, Steps.Count - 30);
        for (int i = from; i < Steps.Count; i++) sb.Append($"{i + 1,3}. {StepExecutor.Describe(Steps[i])}\n");
        if (Steps.Count == 0) sb.Append("  (zatím nic)\n");
        return sb.ToString();
    }
}
