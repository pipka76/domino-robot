using System.Collections.Generic;
using System.Linq;
using Godot;

namespace DominoSim;

public sealed class CaptureStep
{
    public int PieceId;
    public Vector2 Approach;   // robot origin before driving through the piece
    public float Heading;      // perpendicular to the piece's long axis
    public PColor RequiredFront;
    public override string ToString() => $"#{PieceId} front={PColors.Letter(RequiredFront)}";
}

public sealed class PairPlan
{
    public int SlotIndex;
    /// <summary>
    /// false: robot drives toward the low slot indices, the stored line's front is the pair's low end.
    /// true: robot drives toward the high indices, front is the high end.
    /// The front always touches the existing chain (or nothing for the very first pair).
    /// </summary>
    public bool Reversed;
    public CaptureStep First;    // captured first -> rear slot
    public CaptureStep Second;   // captured second -> front slot -> touches the existing chain end
    public PColor Front, Joint, Rear;
    public override string ToString() =>
        $"slot {SlotIndex + 1}{(Reversed ? " (from low side)" : "")}: front {PColors.Letter(Front)}|{PColors.Letter(Joint)}  {PColors.Letter(Joint)}|{PColors.Letter(Rear)} rear  (1st #{First.PieceId}, 2nd #{Second.PieceId})";
}

/// <summary>Placed part of the chain: a contiguous slot range [SlotLo, SlotHi] with the free colour at each end.</summary>
public sealed class PlannerState
{
    public HashSet<int> Taken = new();
    public int SlotLo = -1, SlotHi = -1;      // -1 = nothing placed yet
    public PColor ColLo = PColor.None, ColHi = PColor.None;
    public Vector2 RobotPos;
    public float RobotHeading;
    /// <summary>Piece that must be captured first (the one the robot is already lined up on), or -1.</summary>
    public int ForcedFirst = -1;
    /// <summary>Piece that should be captured second in the first pair (e.g. the next one straight ahead), or -1. Ignored if it would shorten the chain.</summary>
    public int ForcedSecond = -1;
    public int Placed => SlotLo < 0 ? 0 : SlotHi - SlotLo + 1;

    /// <summary>Record a placed pair.</summary>
    public void Apply(PairPlan p)
    {
        if (SlotLo < 0)
        {
            SlotLo = SlotHi = p.SlotIndex;
            ColLo = p.Reversed ? p.Rear : p.Front;
            ColHi = p.Reversed ? p.Front : p.Rear;
        }
        else if (p.SlotIndex < SlotLo) { SlotLo = p.SlotIndex; ColLo = p.Rear; }
        else { SlotHi = p.SlotIndex; ColHi = p.Rear; }
    }
}

/// <summary>
/// Chooses which pieces to carry together and in which order so that the placed pairs form
/// one colour-correct chain. Pieces are edges between two colours; a pair = two edges sharing the
/// joint colour. The chain is a row of slots; it starts anywhere and grows at both ends, each new
/// pair pushed front-first against one free end (so its front colour must equal that end's colour).
/// Only the outermost piece of each line can be reached (robot drives in from the line end).
/// Depth-first branch &amp; bound: maximise number of pairs, then minimise estimated travel + turning.
/// </summary>
public static class ChainPlanner
{
    /// <summary>Robot stops with the colour sensors over the piece centre.</summary>
    public static float ApproachDist => ColorSensor.SensorX;
    /// <summary>Cost of turning, in metres of travel per radian (turns near walls are slow and shove the robot).</summary>
    public const float TurnCost = 0.25f;

    static float Turn(float a, float b) => TurnCost * Mathf.Abs(Geo.Wrap(a - b));

    readonly struct Cand
    {
        public readonly CaptureStep A, B;
        public readonly PColor F, J, R;
        public readonly int LineA, LineB, SideA, SideB, Slot;
        public readonly bool Rev;
        public readonly float Cost;
        public Cand(CaptureStep a, CaptureStep b, PColor f, PColor j, PColor r, int la, int sa, int lb, int sb, int slot, bool rev, float cost)
        { A = a; B = b; F = f; J = j; R = r; LineA = la; SideA = sa; LineB = lb; SideB = sb; Slot = slot; Rev = rev; Cost = cost; }
    }

    public static List<PairPlan> Plan(FieldLayout L, IReadOnlyDictionary<int, (PColor, PColor)> colors,
        List<Slot> slots, PlannerState st, int nodeBudget = 300000)
    {
        var lines = L.Lines;
        int n = lines.Count;
        var lo = new int[n];
        var hi = new int[n];
        for (int i = 0; i < n; i++)
        {
            var ps = lines[i].Pieces;
            lo[i] = 0;
            hi[i] = ps.Count - 1;
            while (lo[i] <= hi[i] && st.Taken.Contains(ps[lo[i]].Id)) lo[i]++;
            while (hi[i] >= lo[i] && st.Taken.Contains(ps[hi[i]].Id)) hi[i]--;
        }

        int maxPairs = slots.Count - st.Placed;
        int goal = 0;
        bool forceSecond = false;
        var memo = new Dictionary<long, int>();

        // Upper bound on pairs still achievable from a state (colours/accessibility only, no cost).
        int Reach(int sLo, int sHi, PColor cLo, PColor cHi)
        {
            if (sLo >= 0 && sHi - sLo + 1 >= slots.Count) return 0;
            long key = 0;
            for (int i = 0; i < n; i++) key = key * 64 + lo[i] * 8 + (hi[i] + 1);
            key = (((key * 16 + (sLo + 1)) * 16 + (sHi + 1)) * 8 + (int)cLo) * 8 + (int)cHi;
            if (memo.TryGetValue(key, out int v)) return v;
            int best = 0;
            foreach (var c in Enumerate(sLo, sHi, cLo, cHi, Vector2.Zero, 0f, ignoreForced: true, withCost: false))
            {
                Take(c);
                var (nLo, nHi, nCLo, nCHi) = Next(sLo, sHi, cLo, cHi, c);
                best = System.Math.Max(best, 1 + Reach(nLo, nHi, nCLo, nCHi));
                Untake(c);
                if (best >= slots.Count - (sLo < 0 ? 0 : sHi - sLo + 1)) break;
            }
            memo[key] = best;
            return best;
        }

        void Take(Cand c)
        {
            if (c.SideA == 0) lo[c.LineA]++; else hi[c.LineA]--;
            if (c.SideB == 0) lo[c.LineB]++; else hi[c.LineB]--;
        }
        void Untake(Cand c)
        {
            if (c.SideB == 0) lo[c.LineB]--; else hi[c.LineB]++;
            if (c.SideA == 0) lo[c.LineA]--; else hi[c.LineA]++;
        }
        static (int, int, PColor, PColor) Next(int sLo, int sHi, PColor cLo, PColor cHi, Cand c)
        {
            if (sLo < 0) return (c.Slot, c.Slot, c.Rev ? c.R : c.F, c.Rev ? c.F : c.R);
            if (c.Slot < sLo) return (c.Slot, sHi, c.R, cHi);
            return (sLo, c.Slot, cLo, c.R);
        }

        List<Cand> Enumerate(int sLo, int sHi, PColor cLo, PColor cHi, Vector2 pos, float heading, bool ignoreForced, bool withCost)
        {
            // Where can the next pair go?  (slot, reversed, colour its front must have)
            var targets = new List<(int slot, bool rev, PColor need)>();
            if (sLo < 0)
            {
                for (int k = 0; k < slots.Count; k++)
                {
                    // Orientation matters: it decides which colour faces the side with fewer free slots.
                    targets.Add((k, false, PColor.None));
                    targets.Add((k, true, PColor.None));
                }
            }
            else
            {
                if (sLo > 0) targets.Add((sLo - 1, true, cLo));
                if (sHi < slots.Count - 1) targets.Add((sHi + 1, false, cHi));
            }

            var cands = new List<Cand>();
            for (int la = 0; la < n; la++)
            {
                if (lo[la] > hi[la]) continue;
                for (int sa = 0; sa < 2; sa++)
                {
                    if (!withCost && sa == 1 && lo[la] == hi[la]) continue;
                    var pa = lines[la].Pieces[sa == 0 ? lo[la] : hi[la]];
                    if (!ignoreForced && st.ForcedFirst >= 0 && pa.Id != st.ForcedFirst) continue;
                    var stepA = Step(lines[la], pa, sa);
                    if (sa == 0) lo[la]++; else hi[la]--;
                    for (int lb = 0; lb < n; lb++)
                    {
                        if (lo[lb] > hi[lb]) continue;
                        for (int sb = 0; sb < 2; sb++)
                        {
                            if (!withCost && sb == 1 && lo[lb] == hi[lb]) continue;
                            var pb = lines[lb].Pieces[sb == 0 ? lo[lb] : hi[lb]];
                            if (!ignoreForced && forceSecond && pb.Id != st.ForcedSecond) continue;
                            var stepB = Step(lines[lb], pb, sb);
                            var (a0, a1) = colors[pa.Id];
                            var (b0, b1) = colors[pb.Id];
                            float pickCost = !withCost ? 0f : pos.DistanceTo(stepA.Approach) + Turn(heading, stepA.Heading)
                                             + stepA.Approach.DistanceTo(stepB.Approach) + Turn(stepA.Heading, stepB.Heading);
                            foreach (var j in new[] { a0, a1 }.Distinct())
                            {
                                if (b0 != j && b1 != j) continue;
                                var r = a0 == j ? a1 : a0;
                                var f = b0 == j ? b1 : b0;
                                foreach (var (slot, rev, need) in targets)
                                {
                                    if (need != PColor.None && f != need) continue;
                                    float c = pickCost;
                                    if (withCost)
                                    {
                                        var pl = slots[slot].Placement(rev);
                                        c += stepB.Approach.DistanceTo(pl.ApproachStart) + Turn(stepB.Heading, pl.Heading);
                                    }
                                    cands.Add(new Cand(stepA, stepB, f, j, r, la, sa, lb, sb, slot, rev, c));
                                }
                            }
                        }
                    }
                    if (sa == 0) lo[la]--; else hi[la]++;
                }
            }
            return cands;
        }
        List<PairPlan> best = new();
        float bestCost = float.MaxValue;
        var cur = new List<PairPlan>();
        int nodes = 0;

        void Recurse(int sLo, int sHi, PColor cLo, PColor cHi, Vector2 pos, float heading, float cost)
        {
            if (cur.Count > best.Count || (cur.Count == best.Count && cost < bestCost))
            {
                best = new List<PairPlan>(cur);
                bestCost = cost;
            }
            if (cur.Count >= goal) return;
            if (++nodes > nodeBudget) return;
            if (best.Count == goal && cost >= bestCost) return;

            var cands = Enumerate(sLo, sHi, cLo, cHi, pos, heading, ignoreForced: cur.Count > 0, withCost: true);
            int need = goal - cur.Count;   // pairs still required for the best achievable chain
            foreach (var c in cands.OrderBy(c => c.Cost))
            {
                Take(c);
                var nx = Next(sLo, sHi, cLo, cHi, c);
                if (1 + Reach(nx.Item1, nx.Item2, nx.Item3, nx.Item4) < need)
                {
                    Untake(c);
                    continue; // cannot reach the best possible chain length any more
                }
                var pp = new PairPlan
                {
                    SlotIndex = c.Slot, Reversed = c.Rev,
                    First = Clone(c.A, c.J), Second = Clone(c.B, c.F),
                    Front = c.F, Joint = c.J, Rear = c.R,
                };
                cur.Add(pp);
                var pl = slots[c.Slot].Placement(c.Rev);
                Recurse(nx.Item1, nx.Item2, nx.Item3, nx.Item4, pl.ApproachStart, pl.Heading + Mathf.Pi, cost + c.Cost);
                cur.RemoveAt(cur.Count - 1);
                Untake(c);
                if (nodes > nodeBudget) return;
            }
        }

        // Best achievable number of pairs (respecting the forced pieces).
        int GoalNow()
        {
            int g = 0;
            var first = Enumerate(st.SlotLo, st.SlotHi, st.ColLo, st.ColHi, Vector2.Zero, 0f, ignoreForced: false, withCost: false);
            foreach (var c in first)
            {
                Take(c);
                var nx = Next(st.SlotLo, st.SlotHi, st.ColLo, st.ColHi, c);
                g = System.Math.Max(g, 1 + Reach(nx.Item1, nx.Item2, nx.Item3, nx.Item4));
                Untake(c);
                if (g >= maxPairs) break;
            }
            return g;
        }
        goal = GoalNow();
        if (st.ForcedSecond >= 0)
        {
            forceSecond = true;
            int g2 = GoalNow();
            if (g2 < goal) forceSecond = false; // would cost pieces: let the planner choose
            else goal = g2;
        }
        Recurse(st.SlotLo, st.SlotHi, st.ColLo, st.ColHi, st.RobotPos, st.RobotHeading, 0f);
        return best;
    }

    static CaptureStep Clone(CaptureStep s, PColor front) =>
        new() { PieceId = s.PieceId, Approach = s.Approach, Heading = s.Heading, RequiredFront = front };

    /// <summary>Capture step for a piece entered from the line start (side 0) or end (side 1).</summary>
    public static CaptureStep Step(LineDef line, PieceDef p, int side)
    {
        var dir = side == 0 ? line.Dir : -line.Dir;
        return new CaptureStep { PieceId = p.Id, Heading = Geo.Angle(dir), Approach = p.Center - dir * ApproachDist };
    }
}
