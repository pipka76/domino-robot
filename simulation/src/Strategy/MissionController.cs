using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Godot;

namespace DominoSim;

/// <summary>
/// Autonomous run: plan the chain, then for each pair capture two pieces, drive to the slot,
/// release and back off. Navigation uses the true robot pose (no localisation model yet);
/// piece colours/orientation are taken from the simulated colour sensors at capture time.
/// </summary>
public partial class MissionController : Node
{
    public const double MatchTime = 90.0;

    public Robot Robot;
    public List<Domino> Dominos;
    public FieldLayout Layout;
    public bool AutoQuit;
    public double HardTimeLimit = 400;

    public double SimTime { get; private set; }
    public string State { get; private set; } = "starting";
    public bool Finished { get; private set; }
    public readonly List<string> JointLog = new();

    readonly PhysicsTicker _ticker = new();
    Motion _m;
    Navigator _nav;
    Task _root;
    int _settleTicks = 30;
    bool _reported90;

    List<Slot> _slots;
    List<PairPlan> _plan = new();
    readonly Dictionary<int, (PColor, PColor)> _known = new();
    readonly PlannerState _st = new();
    Domino[][] _bySlot;                     // placed pieces per slot, in slot (low -> high) order
    readonly List<Domino> _loose = new();    // pieces released without forming a full pair
    readonly Dictionary<int, int> _failures = new();

    public IReadOnlyList<Slot> Slots => _slots;
    public IReadOnlyList<PairPlan> Plan => _plan;

    public override void _Ready()
    {
        Name = "Mission";
        _nav = new Navigator(Layout.Width, Layout.Depth)
        {
            WallInflate = Mathf.Max(0.12f, RobotSpec.Width / 2f + 0.03f),
            // Wheel/caster clearance from pieces. A full turning clearance (RobotSpec.FloorPartsRadius)
            // would close off the corner pockets at the row ends for a 280 mm robot.
            ObstacleInflate = Mathf.Max(0.115f, RobotSpec.HalfTrack + 0.022f),
        };
        _m = new Motion(Robot, _ticker, () => SimTime) { Log = Log };
        Robot.Intake.Candidates = () => Dominos;
        Robot.Intake.CaptureStarted += (d, arm) => Log($"  captured #{d.Id} with {arm} arm");
        foreach (var p in Layout.Pieces) _known[p.Id] = (p.C0, p.C1);
    }

    public override void _PhysicsProcess(double delta)
    {
        if (_settleTicks > 0)
        {
            if (--_settleTicks == 0) RunNoContext(() => _root = RunAsync());
            return;
        }
        if (Finished) return;
        SimTime += delta;
        RunNoContext(_ticker.Pump);

        if (!_reported90 && SimTime >= MatchTime)
        {
            _reported90 = true;
            Log($"=== 90 s mark: {Chain().Count} pieces placed in the chain; {Summary()}");
        }
        if (_root != null && _root.IsCompleted)
        {
            if (_root.IsFaulted) GD.PushError(_root.Exception?.ToString());
            Finish();
        }
        else if (SimTime > HardTimeLimit)
        {
            Log("Hard time limit reached");
            Finish();
        }
    }

    // Run continuations without Godot's SynchronizationContext so awaits resume inline, in the same physics tick.
    static void RunNoContext(Action a)
    {
        var prev = SynchronizationContext.Current;
        SynchronizationContext.SetSynchronizationContext(null);
        try { a(); }
        finally { SynchronizationContext.SetSynchronizationContext(prev); }
    }

    void Finish()
    {
        Finished = true;
        Robot.Stop();
        State = "finished";
        Log($"=== Finished at {SimTime:0.0} s: {Chain().Count} pieces placed; {Summary()}");
        if (AutoQuit) GetTree().Quit();
    }

    void Log(string s)
    {
        GD.Print($"[{SimTime,6:0.00}] {s}");
    }

    void Replan()
    {
        _st.RobotPos = Robot.Pos2;
        _st.RobotHeading = Robot.Theta;
        _plan = ChainPlanner.Plan(Layout, _known, _slots, _st);
        Log(_st.Placed == 0
            ? $"Plan ({_plan.Count} pairs):"
            : $"Plan ({_plan.Count} pairs; placed slots {_st.SlotLo + 1}-{_st.SlotHi + 1}, free ends {PColors.Letter(_st.ColLo)} / {PColors.Letter(_st.ColHi)}):");
        foreach (var p in _plan) Log("   " + p);
    }

    async Task RunAsync()
    {
        _slots = FormationPlanner.BuildU(Layout);
        _bySlot = new Domino[_slots.Count][];
        // If the start pose is already lined up on a piece (e.g. corner start), take that one first.
        foreach (var line in Layout.Lines)
            for (int side = 0; side < 2; side++)
            {
                var step = ChainPlanner.Step(line, side == 0 ? line.Pieces[0] : line.Pieces[^1], side);
                if (step.Approach.DistanceTo(Robot.Pos2) < 0.03f && Mathf.Abs(Geo.Wrap(step.Heading - Robot.Theta)) < 0.2f)
                {
                    _st.ForcedFirst = step.PieceId;
                    // ...and keep going straight: the next piece of the same line.
                    int idx = line.Pieces.FindIndex(p => p.Id == step.PieceId) + (side == 0 ? 1 : -1);
                    if (idx >= 0 && idx < line.Pieces.Count) _st.ForcedSecond = line.Pieces[idx].Id;
                    Log($"Start pose is lined up on #{step.PieceId}: capturing it first, then #{_st.ForcedSecond} straight ahead");
                }
            }
        Replan();
        _st.ForcedFirst = _st.ForcedSecond = -1;
        int i = 0;
        while (i < _plan.Count)
        {
            var pp = _plan[i];
            State = $"pair {pp.SlotIndex + 1}/{_slots.Count}: capture #{pp.First.PieceId}";
            var r1 = await Capture(pp.First);
            if (r1 == CaptureResult.Mismatch && Robot.Intake.Count == 0)
            {
                Replan();
                i = 0;
                continue;
            }
            State = $"pair {pp.SlotIndex + 1}/{_slots.Count}: capture #{pp.Second.PieceId}";
            await Capture(pp.Second);

            State = $"pair {pp.SlotIndex + 1}/{_slots.Count}: place";
            await Place(_slots[pp.SlotIndex], pp.Reversed);
            _st.Apply(pp);
            i++;
        }
        State = "done";
        await _m.StopAndSettle();
    }

    enum CaptureResult { Ok, Mismatch, Failed }

    List<Obb> Obstacles() =>
        Dominos.Where(d => !d.Carried).Select(d => d.Footprint).ToList();

    async Task NavigateTo(Vector2 goal, float heading)
    {
        // Goal straight ahead on the line the robot is already driving along: just keep going.
        var d = goal - Robot.Pos2;
        float along = d.Dot(Geo.Dir(heading));
        if (Mathf.Abs(d.Dot(Geo.Right(heading))) < 0.01f && Mathf.Abs(Geo.Wrap(heading - Robot.Theta)) < 0.09f && along > -0.01f && along < 0.5f)
        {
            await _m.DriveLine(goal, heading, 0f, 0.2f);
            return;
        }
        if (Robot.Pos2.DistanceTo(goal) > 0.025f)
        {
            // Plan to a point just behind the goal, then finish along the goal heading so the
            // robot arrives (almost) aligned with the straight run that follows.
            var obs = Obstacles();
            var pre = goal - Geo.Dir(heading) * 0.08f;
            bool usePre = _nav.WallDistance(pre) >= _nav.WallInflate && _nav.WallDistance(goal) >= _nav.EscapeWall;
            var path = _nav.Plan(Robot.Pos2, usePre ? pre : goal, obs, out bool ok);
            if (!ok && usePre)
            {
                usePre = false;
                path = _nav.Plan(Robot.Pos2, goal, obs, out ok);
            }
            if (!ok) Log($"  WARNING A* found no path to {goal}, driving direct");
            if (usePre) path.Add(goal);
            await _m.FollowPath(path, 0.38f);
        }
        await _m.TurnTo(heading);
    }

    async Task<CaptureResult> Capture(CaptureStep step)
    {
        var piece = Dominos[step.PieceId];
        var def = Layout.Pieces.First(p => p.Id == step.PieceId);
        float moved = piece.Pos2.DistanceTo(def.Center);
        if (moved > 0.003f) Log($"  WARNING piece #{step.PieceId} displaced {moved * 1000:0} mm from its start position");
        await NavigateTo(step.Approach, step.Heading);
        await _m.GoToPrecise(step.Approach, step.Heading, Robot.Intake.Count == 0);

        var (k0, k1) = _known[step.PieceId];
        var (cl, cr) = ReadHalves(step);
        // The piece's two colours are known (only its orientation is not). A reading that does not
        // match that pair means the robot is too far off line (e.g. shoved by a wall): realign, re-read.
        for (int tries = 0; tries < 2 && !((cl == k0 && cr == k1) || (cl == k1 && cr == k0)); tries++)
        {
            Log($"  #{step.PieceId}: reading {PColors.Letter(cl)}|{PColors.Letter(cr)} does not match {PColors.Letter(k0)}{PColors.Letter(k1)}, realigning");
            await _m.GoToPrecise(step.Approach, step.Heading, Robot.Intake.Count == 0);
            await _m.DriveLine(step.Approach, step.Heading, 0f, 0.08f);
            (cl, cr) = ReadHalves(step, allowInference: tries == 1);
        }
        bool consistent = (cl == k0 && cr == k1) || (cl == k1 && cr == k0);
        Intake.ArmMode arm;
        if (consistent && cl == step.RequiredFront) arm = Intake.ArmMode.Left;
        else if (consistent && cr == step.RequiredFront) arm = Intake.ArmMode.Right;
        else
        {
            Log($"  colour mismatch at #{step.PieceId}: sensed {PColors.Letter(cl)}|{PColors.Letter(cr)}, need {PColors.Letter(step.RequiredFront)} in front");
            int n = _failures[step.PieceId] = _failures.GetValueOrDefault(step.PieceId) + 1;
            if (n >= 2)
            {
                Log($"  giving up on #{step.PieceId} (probably knocked out of place)");
                _st.Taken.Add(step.PieceId);
            }
            if (Robot.Intake.Count == 0) return CaptureResult.Mismatch;
            // Already carrying the first piece of the pair: take this one anyway (joint may be wrong).
            arm = cr == step.RequiredFront ? Intake.ArmMode.Right : Intake.ArmMode.Left;
        }
        Log($"  #{step.PieceId}: sensed L={PColors.Letter(cl)} R={PColors.Letter(cr)}, front must be {PColors.Letter(step.RequiredFront)} -> {arm} arm");
        Robot.Intake.SetArm(arm);
        await _m.Wait(0.15);

        int before = Robot.Intake.Stored.Count;
        await _m.DriveLine(step.Approach, step.Heading, 0.32f, 0.15f,
            stop: () => Robot.Intake.Stored.Count > before && !Robot.Intake.Busy, vmin: 0.05f);
        Robot.Intake.SetArm(Intake.ArmMode.Retracted);
        if (Robot.Intake.Stored.Count <= before)
        {
            Log($"  capture of #{step.PieceId} FAILED");
            return CaptureResult.Failed;
        }
        _st.Taken.Add(step.PieceId);
        return CaptureResult.Ok;
    }

    /// <summary>
    /// Colours of the left and right half of the piece ahead. Uses the robot's sideways offset from
    /// the capture line to tell which half each sensor is really over; a half seen by neither
    /// sensor is filled in from the piece's known colour pair.
    /// </summary>
    (PColor left, PColor right) ReadHalves(CaptureStep step, bool allowInference = false)
    {
        var (cl, cr) = ColorSensor.Read(Robot, Dominos);
        float lat = (Robot.Pos2 - step.Approach).Dot(Geo.Right(step.Heading));
        PColor left = PColor.None, right = PColor.None;
        void Put(PColor c, float y)
        {
            if (c == PColor.None || c == PColor.Black) return;
            if (y < 0) left = c; else right = c;
        }
        Put(cl, lat - ColorSensor.SensorZ);
        Put(cr, lat + ColorSensor.SensorZ);
        var (k0, k1) = _known[step.PieceId];
        PColor Other(PColor c) => c == k0 ? k1 : c == k1 ? k0 : PColor.None;
        // Only one sensor on the piece: the inference assumes the piece is still on its line, which is
        // wrong if it was pushed. Only trust it when allowed (after realigning has not helped).
        if (allowInference)
        {
            if (left == PColor.None && right != PColor.None) left = Other(right);
            if (right == PColor.None && left != PColor.None) right = Other(left);
        }
        if (Mathf.Abs(lat) > 0.005f) Log($"  #{step.PieceId}: robot {lat * 1000:0} mm off line, raw sensors {PColors.Letter(cl)}|{PColors.Letter(cr)}");
        return (left, right);
    }

    async Task Place(Slot slot, bool reversed)
    {
        var s = slot.Placement(reversed);
        await NavigateTo(s.ApproachStart, s.Heading);
        float run = s.ApproachStart.DistanceTo(s.ReleaseCenter);
        await _m.GoToPrecise(s.ApproachStart, s.Heading, false);
        await _m.DriveLine(s.ApproachStart, s.Heading, run, 0.15f, vmin: 0.012f);
        var e = Robot.Pos2 - s.ReleaseCenter;
        Log($"  release pose error: along={e.Dot(Geo.Dir(s.Heading)) * 1000:0.0}mm lat={e.Dot(Geo.Right(s.Heading)) * 1000:0.0}mm heading={Mathf.RadToDeg(Geo.Wrap(Robot.Theta - s.Heading)):0.00}°");
        var placed = Robot.Intake.Release();
        await _m.Wait(0.1);
        await _m.DriveLine(s.ApproachStart, s.Heading, run - 0.17f, 0.2f);

        if (placed.Count != 2)
        {
            Log($"  released {placed.Count} piece(s) - pair incomplete");
            _loose.AddRange(placed);
            return;
        }
        // Stored [0] = rear, [1] = front. Store in slot order (low index side first).
        int j = slot.Index;
        _bySlot[j] = reversed ? new[] { placed[0], placed[1] } : new[] { placed[1], placed[0] };
        void Report(Domino a, Domino b)
        {
            var r = JointEvaluator.Evaluate(a.Pose, b.Pose);
            string line = $"  joint #{a.Id}-#{b.Id} (slot {j + 1}): {r}";
            JointLog.Add(line);
            Log(line);
        }
        if (j > 0 && _bySlot[j - 1] != null) Report(_bySlot[j - 1][1], _bySlot[j][0]);
        Report(_bySlot[j][0], _bySlot[j][1]);
        if (j < _bySlot.Length - 1 && _bySlot[j + 1] != null) Report(_bySlot[j][1], _bySlot[j + 1][0]);
    }

    /// <summary>Placed pieces in chain order.</summary>
    List<Domino> Chain()
    {
        var list = new List<Domino>();
        if (_bySlot == null) return list;
        foreach (var p in _bySlot)
            if (p != null) list.AddRange(p);
        return list;
    }

    /// <summary>Indicative evaluation of the placed chain (not a full referee).</summary>
    public string Summary()
    {
        var inner = Layout.YellowInner;
        int inYellow = Dominos.Count(d => !d.Carried && d.Footprint.Corners().Any(c => Layout.YellowRect.HasPoint(c)));
        bool Inside(Domino d) => d.Upright && d.Footprint.Corners().All(c => inner.HasPoint(c));
        var _chain = Chain();
        int bestRun = _chain.Count > 0 && Inside(_chain[0]) ? 1 : 0, run = bestRun, perp = 0, bestPerp = 0;
        for (int k = 1; k < _chain.Count; k++)
        {
            var j = JointEvaluator.Evaluate(_chain[k - 1].Pose, _chain[k].Pose);
            if (j.Valid && Inside(_chain[k - 1]) && Inside(_chain[k]))
            {
                run = run == 0 ? 2 : run + 1;
                if (j.Perpendicular) perp++;
            }
            else
            {
                run = Inside(_chain[k]) ? 1 : 0;
                perp = 0;
            }
            if (run > bestRun) { bestRun = run; bestPerp = perp; }
        }
        int chainPts = bestRun >= 2 ? bestRun * bestRun + bestPerp : 0;
        return $"pieces touching yellow={inYellow}, longest valid chain={bestRun} (≈{chainPts} pts incl. {bestPerp} perp bonus)";
    }

    public string CarriedText()
    {
        var sb = new StringBuilder();
        foreach (var d in Robot.Intake.Stored) sb.Append($"#{d.Id}[{PColors.Letter(d.C0)}{PColors.Letter(d.C1)}] ");
        return sb.Length == 0 ? "-" : sb.ToString();
    }

    public int PlacedCount => Chain().Count + _loose.Count;
}
