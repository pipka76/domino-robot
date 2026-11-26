using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Nodes;
using System.Threading;
using System.Threading.Tasks;
using Godot;

namespace DominoSim;

/// <summary>Base for nodes that run async robot code in lock-step with physics.</summary>
public abstract partial class SimDriver : Node
{
    public Robot Robot;
    public List<Domino> Dominos;
    public FieldLayout Layout;
    public double SimTime { get; protected set; }
    protected readonly PhysicsTicker Ticker = new();
    protected Motion M;
    protected StepExecutor Exec;
    protected int SettleTicks = 30;
    public readonly List<string> LogLines = new();

    public override void _Ready()
    {
        M = new Motion(Robot, Ticker, () => SimTime) { Log = Log };
        Robot.Intake.Candidates = () => Dominos;
        Robot.Intake.CaptureStarted += (d, arm) => Log($"    nabrána kostka #{d.Id} [{PColors.Letter(d.C0)}{PColors.Letter(d.C1)}] ramenem {arm}");
    }

    protected void Log(string s)
    {
        GD.Print($"[{SimTime,6:0.00}] {s}");
        LogLines.Add(s);
        if (LogLines.Count > 200) LogLines.RemoveAt(0);
    }

    // Run continuations without Godot's SynchronizationContext so awaits resume inline, in the same physics tick.
    protected static void RunNoContext(Action a)
    {
        var prev = SynchronizationContext.Current;
        SynchronizationContext.SetSynchronizationContext(null);
        try { a(); }
        finally { SynchronizationContext.SetSynchronizationContext(prev); }
    }

    public string CarriedText()
    {
        var sb = new StringBuilder();
        foreach (var d in Robot.Intake.Stored) sb.Append($"#{d.Id}[{PColors.Letter(d.C0)}{PColors.Letter(d.C1)}] ");
        return sb.Length == 0 ? "-" : sb.ToString();
    }
}

/// <summary>Mode 1: runs a program (list of steps) and measures its time.</summary>
public partial class ProgramRunner : SimDriver
{
    public ProgramFile Program;
    public List<JsonObject> StepsOverride;   // used by the drive mode replay
    public bool AutoQuit;
    public bool Quiet;
    public event Action Done;

    public List<JsonObject> Steps { get; private set; }
    public int Index { get; private set; } = -1;
    public bool Finished { get; private set; }
    public readonly List<double> StepTimes = new();
    public StepExecutor Executor => Exec;
    public int Skipped { get; private set; }
    Task _root;

    public override void _Ready()
    {
        Name = "ProgramRunner";
        base._Ready();
        Steps = StepsOverride ?? Program.Flatten();
        Exec = new StepExecutor(Robot, M, () => SimTime) { Log = Log };
    }

    public override void _PhysicsProcess(double delta)
    {
        if (SettleTicks > 0)
        {
            if (--SettleTicks == 0)
            {
                Exec.RefHeading = Robot.Theta;
                RunNoContext(() => _root = RunAsync());
            }
            return;
        }
        if (Finished) return;
        SimTime += delta;
        RunNoContext(Ticker.Pump);
        if (_root != null && _root.IsCompleted)
        {
            if (_root.IsFaulted) GD.PushError(_root.Exception?.ToString());
            Finished = true;
            Robot.Stop();
            if (!Quiet)
            {
                Log($"=== Celkový čas: {SimTime:0.00} s {(SimTime <= MissionController.MatchTime ? "(v limitu 90 s)" : $"(o {SimTime - MissionController.MatchTime:0.0} s přes 90 s)")}");
                Log("=== " + Scorer.Summary(Dominos, Layout));
                Log($"=== Konečná poloha x = {Robot.Pos2.X * 1000:0} mm, z = {Robot.Pos2.Y * 1000:0} mm, směr {Mathf.RadToDeg(Robot.Theta):0.0}° (sim, + = po směru hodin)");
            }
            Done?.Invoke();
            if (AutoQuit) GetTree().Quit();
        }
    }

    async Task RunAsync()
    {
        if (!Quiet) Log($"Program '{Program?.Name ?? "záznam"}': {Steps.Count} kroků");
        for (Index = 0; Index < Steps.Count; Index++)
        {
            var st = Steps[Index];
            double t0 = SimTime;
            bool done = await Exec.Exec(st);
            StepTimes.Add(SimTime - t0);
            if (!done) Skipped++;
            if (!Quiet)
                Log($"{Index + 1,3}. {StepExecutor.Describe(st),-28} {(done ? $"{SimTime - t0,5:0.00} s  (t = {SimTime:0.00})" : "přeskočeno (v simulaci není)")}");
        }
        await M.StopAndSettle();
    }

    public string StatusText()
    {
        double t = SimTime;
        string timer = t <= MissionController.MatchTime ? $"zbývá {MissionController.MatchTime - t:0.0} s" : $"+{t - MissionController.MatchTime:0.0} s přes 90 s";
        string cur = Finished ? "hotovo" : Index >= 0 && Index < Steps.Count ? $"{Index + 1}/{Steps.Count}: {StepExecutor.Describe(Steps[Index])}" : "start";
        return $"MÓD 1 – přehrání programu '{Program?.Name}'   t = {t:0.00} s ({timer})\n" +
               $"Krok {cur}" + (Skipped > 0 ? $"   (přeskočeno {Skipped})" : "") + "\n" +
               $"Zásobník: {CarriedText()}\n" +
               Scorer.Summary(Dominos, Layout);
    }

    public string SideText()
    {
        var sb = new StringBuilder("Kroky:\n");
        int from = Math.Max(0, Math.Min(Index - 12, Steps.Count - 30));
        for (int i = from; i < Math.Min(Steps.Count, from + 30); i++)
        {
            string mark = i == Index && !Finished ? "▶" : " ";
            string time = i < StepTimes.Count ? $"{StepTimes[i],5:0.00} s" : "";
            sb.Append($"{mark}{i + 1,3}. {StepExecutor.Describe(Steps[i]),-24} {time}\n");
        }
        return sb.ToString();
    }
}
