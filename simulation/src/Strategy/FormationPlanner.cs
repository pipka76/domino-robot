using System.Collections.Generic;
using Godot;

namespace DominoSim;

/// <summary>
/// One pair position in the formation. The formation is a row of slots (index order = chain order);
/// the pair occupies the segment from Front back (opposite to Heading) for two piece lengths.
/// Heading points toward the lower slot indices. A pair can be unloaded either way round:
/// Placement(false) drives along Heading (stored line's front = low end), Placement(true) drives
/// the opposite way (front = high end). The robot stops with the front of its stored line at the
/// placement front, releases, then reverses toward ApproachStart.
/// </summary>
public sealed class Slot
{
    public int Index;
    public Vector2 Front;
    public float Heading;

    public Vector2 Back => Front - Geo.Dir(Heading) * (2 * Domino.L);

    public readonly record struct Pose(Vector2 Front, float Heading, Vector2 ReleaseCenter, Vector2 ApproachStart)
    {
        public Vector2 FrontPieceCenter => Front - Geo.Dir(Heading) * (Domino.L / 2f);
        public Vector2 RearPieceCenter => Front - Geo.Dir(Heading) * (1.5f * Domino.L);
    }

    public Pose Placement(bool reversed)
    {
        var f = reversed ? Back : Front;
        float h = reversed ? Geo.Wrap(Heading + Mathf.Pi) : Heading;
        var d = Geo.Dir(h);
        var c = f - d * Robot.BodyFront;
        return new Pose(f, h, c, c - d * FormationPlanner.ApproachRun);
    }
}

/// <summary>
/// Turns the chain into pair slots inside the yellow area. Default shape is a "U":
/// left column of 4 pairs (top -> bottom), 1 pair base (left -> right, perpendicular joint
/// at both corners) and right column of 3 pairs (bottom -> top) = 16 pieces, 2 right-angle joints.
/// Chain order = slot order; the chain may start in any slot and grow both ways. Each new pair is
/// approached from the open side so the robot never drives over already placed pieces.
/// </summary>
public static class FormationPlanner
{
    public const float PairGap = 0.002f;
    public const float ApproachRun = 0.20f;
    const float PL = 2 * Domino.L;        // pair length 0.192
    const float HW = Domino.W / 2f;       // 0.016

    public static List<Slot> BuildU(FieldLayout L)
    {
        var inner = L.YellowInner;
        float width = Domino.W + PairGap + PL;            // left column outer side .. base right end
        float height = 4 * PL + 3 * PairGap;              // left column top .. base bottom side
        float xl = inner.Position.X + (inner.Size.X - width) / 2f + HW;
        float zTop = inner.Position.Y + (inner.Size.Y - height) / 2f;
        float zb = zTop + height;

        var slots = new List<Slot>();
        // Left column, chain runs downward, robot approaches from below heading up (-z).
        for (int k = 0; k < 4; k++)
            slots.Add(Make(new Vector2(xl, zTop + k * (PL + PairGap)), -Mathf.Pi / 2f));
        // Base: front end touches the side of the last left-column piece; robot heads -x.
        float zh = zb - HW;
        var f5 = new Vector2(xl + HW + PairGap, zh);
        slots.Add(Make(f5, Mathf.Pi));
        // Right column: chain runs upward, robot approaches from above heading down (+z);
        // first piece's end face touches the top side of the base's right end.
        float xr = f5.X + PL - HW;
        float z6 = zh - HW - PairGap;
        for (int k = 0; k < 3; k++)
            slots.Add(Make(new Vector2(xr, z6 - k * (PL + PairGap)), Mathf.Pi / 2f));

        for (int i = 0; i < slots.Count; i++) slots[i].Index = i;
        return slots;
    }

    static Slot Make(Vector2 front, float heading) => new() { Front = front, Heading = heading };

    /// <summary>All piece footprints of the formation fit inside the yellow area (with margin).</summary>
    public static bool InsideYellow(FieldLayout L, List<Slot> slots, float margin, out string why)
    {
        var inner = L.YellowInner.Grow(-margin);
        foreach (var s in slots)
            foreach (var c in new[] { s.Placement(false).FrontPieceCenter, s.Placement(false).RearPieceCenter })
                foreach (var p in new Obb(c, s.Heading, Domino.L / 2f, HW).Corners())
                    if (!inner.HasPoint(p))
                    {
                        why = $"slot {s.Index} corner {p} outside {inner}";
                        return false;
                    }
        why = "";
        return true;
    }
}
