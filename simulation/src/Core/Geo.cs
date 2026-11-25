using Godot;

namespace DominoSim;

/// <summary>
/// 2D field helpers. Field plane is Godot XZ; a 2D point is Vector2(x, z).
/// Heading theta: forward = (cos θ, sin θ) in (x, z). With z pointing "down" on the
/// top-down view, increasing θ is a clockwise (right) turn.
/// </summary>
public static class Geo
{
    public static Vector2 XZ(Vector3 v) => new(v.X, v.Z);
    public static Vector3 V3(Vector2 p, float y = 0f) => new(p.X, y, p.Y);
    public static Vector2 Dir(float heading) => new(Mathf.Cos(heading), Mathf.Sin(heading));
    /// <summary>Right-hand side unit vector for a heading.</summary>
    public static Vector2 Right(float heading) => new(-Mathf.Sin(heading), Mathf.Cos(heading));
    public static float Angle(Vector2 v) => Mathf.Atan2(v.Y, v.X);

    public static float Wrap(float a)
    {
        while (a > Mathf.Pi) a -= Mathf.Tau;
        while (a < -Mathf.Pi) a += Mathf.Tau;
        return a;
    }

    /// <summary>Heading of a basis' local +X axis in the field plane.</summary>
    public static float HeadingOf(Basis b) => Mathf.Atan2(b.X.Z, b.X.X);

    /// <summary>Yaw-only basis whose local +X points along the heading.</summary>
    public static Basis YawBasis(float heading) => new(Vector3.Up, -heading);

    /// <summary>Rotate a robot-local 2D vector (x fwd, z right) into the field frame.</summary>
    public static Vector2 Rotate(Vector2 v, float heading)
    {
        float c = Mathf.Cos(heading), s = Mathf.Sin(heading);
        return new Vector2(v.X * c - v.Y * s, v.X * s + v.Y * c);
    }

    public static Vector2 Unrotate(Vector2 v, float heading) => Rotate(v, -heading);
}

/// <summary>Oriented rectangle in the field plane.</summary>
public struct Obb
{
    public Vector2 C;
    public float Angle;
    public float Hx, Hz;

    public Obb(Vector2 c, float angle, float hx, float hz) { C = c; Angle = angle; Hx = hx; Hz = hz; }

    public float Distance(Vector2 p)
    {
        var l = Geo.Unrotate(p - C, Angle);
        float dx = Mathf.Max(Mathf.Abs(l.X) - Hx, 0f);
        float dz = Mathf.Max(Mathf.Abs(l.Y) - Hz, 0f);
        return Mathf.Sqrt(dx * dx + dz * dz);
    }

    public Vector2[] Corners()
    {
        var ax = Geo.Dir(Angle) * Hx;
        var az = Geo.Right(Angle) * Hz;
        return new[] { C - ax - az, C + ax - az, C + ax + az, C - ax + az };
    }
}
