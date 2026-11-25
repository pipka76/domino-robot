using Godot;

namespace DominoSim;

public readonly record struct JointResult(bool Perpendicular, float AngleDevDeg, float Gap, float Offset, PColor Ca, PColor Cb, bool Valid)
{
    public override string ToString() =>
        $"{(Perpendicular ? "perp" : "straight")} ang={AngleDevDeg:0.0}° gap={Gap * 1000:0.0}mm off={Offset * 1000:0.0}mm " +
        $"{PColors.Letter(Ca)}-{PColors.Letter(Cb)} {(Valid ? "OK" : "INVALID")}";
}

/// <summary>Checks one connection against the rules: angle ≤10°, gap ≤5 mm, lateral offset ≤5 mm, colours equal.</summary>
public static class JointEvaluator
{
    public const float MaxAngleDeg = 10f, MaxGap = 0.005f, MaxOffset = 0.005f, MaxOverlap = 0.003f;
    const float L = Domino.L, HW = Domino.W / 2f;

    public static JointResult Evaluate(PiecePose a, PiecePose b)
    {
        var ax = Geo.Dir(a.Axis);
        var bx = Geo.Dir(b.Axis);
        float ang = Mathf.Acos(Mathf.Clamp(Mathf.Abs(ax.Dot(bx)), 0f, 1f));
        bool perp = ang > Mathf.Pi / 4f;
        float dev = Mathf.RadToDeg(perp ? Mathf.Pi / 2f - ang : ang);

        float gap, off;
        PColor ca, cb;
        if (!perp)
        {
            int bi = 0, bj = 0;
            float bd = float.MaxValue;
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                {
                    float d = a.End(i).DistanceTo(b.End(j));
                    if (d < bd) { bd = d; bi = i; bj = j; }
                }
            var outward = (a.End(bi) - a.C).Normalized();
            var dv = b.End(bj) - a.End(bi);
            gap = dv.Dot(outward);
            off = Mathf.Abs(outward.Cross(dv));
            ca = a.ColorAtEnd(bi);
            cb = b.ColorAtEnd(bj);
        }
        else
        {
            var r1 = EndToSide(a, b);   // end of a against side of b
            var r2 = EndToSide(b, a);   // end of b against side of a
            bool use1 = Mathf.Abs(r1.gap) + r1.off <= Mathf.Abs(r2.gap) + r2.off;
            gap = use1 ? r1.gap : r2.gap;
            off = use1 ? r1.off : r2.off;
            ca = use1 ? r1.cEnd : r2.cSide;
            cb = use1 ? r1.cSide : r2.cEnd;
        }
        bool valid = dev <= MaxAngleDeg && gap <= MaxGap && gap >= -MaxOverlap && off <= MaxOffset
                     && ca == cb && ca != PColor.None && ca != PColor.Black && a.Upright && b.Upright;
        return new JointResult(perp, dev, gap, off, ca, cb, valid);
    }

    static (float gap, float off, PColor cEnd, PColor cSide) EndToSide(PiecePose e, PiecePose s)
    {
        (float gap, float off, PColor, PColor) best = (float.MaxValue, float.MaxValue, PColor.None, PColor.None);
        for (int i = 0; i < 2; i++)
        {
            var d = e.End(i) - s.C;
            float lx = d.Dot(Geo.Dir(s.Axis));
            float lz = d.Dot(Geo.Right(s.Axis));
            float gap = Mathf.Abs(lz) - HW;
            float sq = Mathf.Sign(lx) * (L / 2f - HW);
            float off = Mathf.Abs(lx - sq);
            if (Mathf.Abs(gap) + off < Mathf.Abs(best.gap) + best.off)
                best = (gap, off, e.ColorAtEnd(i), s.ColorAtEnd(lx < 0 ? 0 : 1));
        }
        return best;
    }
}
