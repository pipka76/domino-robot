using System.Collections.Generic;
using System.Linq;
using Godot;

namespace DominoSim;

/// <summary>Headless checks of layout, formation geometry, chain planning and navigation.</summary>
public static class SelfTest
{
    static int _fail;

    static void Check(bool cond, string what)
    {
        GD.Print($"{(cond ? "PASS" : "FAIL")}  {what}");
        if (!cond) _fail++;
    }

    public static bool Run(FieldLayout L)
    {
        _fail = 0;
        var pieces = L.Pieces.ToList();
        Check(pieces.Count == 16, $"layout has 16 pieces ({pieces.Count})");
        var deg = new Dictionary<PColor, int>();
        foreach (var p in pieces) { deg[p.C0] = deg.GetValueOrDefault(p.C0) + 1; deg[p.C1] = deg.GetValueOrDefault(p.C1) + 1; }
        Check(deg.Values.All(v => v % 2 == 0), "every colour has even degree: " + string.Join(" ", deg.Select(kv => $"{PColors.Letter(kv.Key)}={kv.Value}")));

        // Formation.
        var slots = FormationPlanner.BuildU(L);
        Check(slots.Count == 8, "U formation has 8 pair slots");
        Check(FormationPlanner.InsideYellow(L, slots, 0.003f, out var why), "formation inside yellow area (3 mm margin) " + why);
        foreach (var s in slots)
            foreach (bool rev in new[] { false, true })
            {
                var a = s.Placement(rev).ApproachStart;
                bool inField = new Rect2(0.1f, 0.1f, L.Width - 0.2f, L.Depth - 0.2f).HasPoint(a);
                Check(inField, $"slot {s.Index + 1}{(rev ? " reversed" : "")} approach start {a} is inside the field");
            }

        // Chain plan.
        var known = pieces.ToDictionary(p => p.Id, p => (p.C0, p.C1));
        var st = new PlannerState { RobotPos = L.RobotStart, RobotHeading = L.RobotStartHeading, ForcedFirst = 0, ForcedSecond = 1 };
        var plan = ChainPlanner.Plan(L, known, slots, st);
        foreach (var p in plan) GD.Print("      " + p);
        Check(plan.Count == 8, $"chain planner fills all 8 slots ({plan.Count})");
        var ids = plan.SelectMany(p => new[] { p.First.PieceId, p.Second.PieceId }).ToList();
        Check(ids.Distinct().Count() == ids.Count, "no piece used twice");
        // Replay the plan: each pair's front colour must match the chain end it is pushed against.
        var st2 = new PlannerState();
        bool colourChain = true;
        var bySlot = new PiecePose[slots.Count][];
        foreach (var p in plan)
        {
            var a = known[p.Second.PieceId];
            var b = known[p.First.PieceId];
            colourChain &= Has(a, p.Front) && Has(a, p.Joint) && Has(b, p.Joint) && Has(b, p.Rear);
            if (st2.Placed > 0)
            {
                bool lowSide = p.SlotIndex == st2.SlotLo - 1 && p.Reversed && p.Front == st2.ColLo;
                bool highSide = p.SlotIndex == st2.SlotHi + 1 && !p.Reversed && p.Front == st2.ColHi;
                colourChain &= lowSide || highSide;
            }
            st2.Apply(p);
            // Virtual pieces: axis = placement heading, so C1 (+X end) faces the front.
            var pose = slots[p.SlotIndex].Placement(p.Reversed);
            var front = new PiecePose(pose.FrontPieceCenter, pose.Heading, true, p.Joint, p.Front);
            var rear = new PiecePose(pose.RearPieceCenter, pose.Heading, true, p.Rear, p.Joint);
            bySlot[p.SlotIndex] = p.Reversed ? new[] { rear, front } : new[] { front, rear };
        }
        Check(colourChain, "planned pairs form one colour-consistent chain growing from its ends");

        var chain = bySlot.Where(x => x != null).SelectMany(x => x).ToList();
        int perp = 0;
        bool joints = true;
        for (int k = 1; k < chain.Count; k++)
        {
            var j = JointEvaluator.Evaluate(chain[k - 1], chain[k]);
            joints &= j.Valid;
            if (j.Perpendicular) perp++;
            if (!j.Valid) GD.Print($"      joint {k}: {j}");
        }
        Check(joints, $"all {chain.Count - 1} virtual joints valid ({perp} perpendicular)");

        // Navigation reachability from the start to each capture approach (initial obstacles).
        var nav = new Navigator(L.Width, L.Depth);
        var obs = pieces.Select(p => new Obb(p.Center, p.Axis, Domino.L / 2f, Domino.W / 2f)).ToList();
        foreach (var line in L.Lines)
            for (int side = 0; side < 2; side++)
            {
                var piece = side == 0 ? line.Pieces[0] : line.Pieces[^1];
                var step = ChainPlanner.Step(line, piece, side);
                nav.Plan(L.RobotStart, step.Approach, obs, out bool ok);
                Check(ok, $"A* reaches {line.Name} {(side == 0 ? "start" : "end")} approach {step.Approach}");
            }

        GD.Print(_fail == 0 ? "SELFTEST OK" : $"SELFTEST FAILED ({_fail})");
        return _fail == 0;
    }

    static bool Has((PColor, PColor) c, PColor x) => c.Item1 == x || c.Item2 == x;
}
