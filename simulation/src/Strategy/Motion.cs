using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Godot;

namespace DominoSim;

/// <summary>Closed-loop motion primitives (run one control step per physics tick).</summary>
public sealed class Motion
{
    readonly Robot _r;
    readonly PhysicsTicker _t;
    readonly Func<double> _now;
    public Action<string> Log = _ => { };

    public float Accel = 0.5f;      // planning decel for stopping at a target

    public Motion(Robot r, PhysicsTicker t, Func<double> now)
    {
        _r = r;
        _t = t;
        _now = now;
    }

    public PhysicsTicker.Awaitable Tick() => _t.Next();

    public async Task Wait(double seconds)
    {
        double end = _now() + seconds;
        while (_now() < end) await Tick();
    }

    public async Task StopAndSettle()
    {
        _r.Stop();
        double end = _now() + 1.0;
        int still = 0;
        while (_now() < end && still < 3)
        {
            await Tick();
            still = (Mathf.Abs(_r.Speed) < 0.003f && Mathf.Abs(_r.YawRate) < 0.03f) ? still + 1 : 0;
        }
    }

    public async Task<bool> TurnTo(float heading, double timeout = 4)
    {
        double end = _now() + timeout;
        while (true)
        {
            float err = Geo.Wrap(heading - _r.Theta);
            if (Mathf.Abs(err) < 0.006f && Mathf.Abs(_r.YawRate) < 0.15f) break;
            if (_now() > end) { Log($"TurnTo timeout, err={Mathf.RadToDeg(err):0.0}°"); break; }
            float w = Mathf.Clamp(5f * err, -3f, 3f);
            if (Mathf.Abs(w) < 0.25f) w = 0.25f * Mathf.Sign(err);
            _r.SetTwist(0, w);
            await Tick();
        }
        await StopAndSettle();
        return Mathf.Abs(Geo.Wrap(heading - _r.Theta)) < 0.02f;
    }

    /// <summary>
    /// Track the line through origin with the given heading until the along-line coordinate
    /// reaches targetS (negative progress = reversing). Optional stop predicate ends early.
    /// </summary>
    public async Task<bool> DriveLine(Vector2 origin, float heading, float targetS, float vmax,
        Func<bool> stop = null, double timeout = -1, float vmin = 0.02f)
    {
        var dir = Geo.Dir(heading);
        var right = Geo.Right(heading);
        if (timeout < 0)
        {
            float dist = Mathf.Abs(targetS - (_r.Pos2 - origin).Dot(dir));
            timeout = 3 + dist / Mathf.Max(0.5f * vmax, vmin);
        }
        double end = _now() + timeout;
        bool reached = false;
        while (true)
        {
            if (stop != null && stop()) break;
            var d = _r.Pos2 - origin;
            float s = d.Dot(dir);
            float lat = d.Dot(right);
            float rem = targetS - s;
            if (Mathf.Abs(rem) < 0.001f) { reached = true; break; }
            if (_now() > end) { Log($"DriveLine timeout rem={rem * 1000:0}mm"); break; }
            float v = Mathf.Sign(rem) * Mathf.Clamp(Mathf.Sqrt(2f * Accel * Mathf.Abs(rem)), vmin, vmax);
            float herr = Geo.Wrap(heading - _r.Theta);
            float w = Mathf.Clamp(6f * herr - 50f * lat * Mathf.Sign(v), -1.5f, 1.5f);
            _r.SetTwist(v, w);
            await Tick();
        }
        await StopAndSettle();
        return reached;
    }

    /// <summary>
    /// Small precise repositioning: turn toward the target point, drive straight onto it, then
    /// turn to the final heading. Removes the lateral error left by path following.
    /// </summary>
    public async Task GoToPrecise(Vector2 target, float heading, bool allowReverse)
    {
        for (int iter = 0; iter < 2; iter++)
        {
            var d = target - _r.Pos2;
            float lat = d.Dot(Geo.Right(heading));
            if (Mathf.Abs(lat) < 0.0025f && d.Dot(Geo.Dir(heading)) > -0.05f) break; // on the line, not past it
            float dist = d.Length();
            if (dist < 0.002f) break;
            float face = Geo.Angle(d);
            float sign = 1f;
            if (allowReverse && Mathf.Abs(Geo.Wrap(face - _r.Theta)) > Mathf.Pi / 2f)
            {
                face = Geo.Wrap(face + Mathf.Pi);
                sign = -1f;
            }
            await TurnTo(face);
            var origin = _r.Pos2;
            await DriveLine(origin, face, sign * dist, 0.1f);
        }
        await TurnTo(heading);
    }

    /// <summary>Pure-pursuit, forward only. Turns on the spot first when facing away from the path.</summary>
    public async Task<bool> FollowPath(List<Vector2> path, float vmax, double timeout = -1)
    {
        if (path.Count < 2) return true;
        if (timeout < 0)
        {
            float len = 0;
            for (int i = 1; i < path.Count; i++) len += path[i - 1].DistanceTo(path[i]);
            timeout = 4 + len / (0.4f * vmax);
        }
        var goal = path[^1];
        double end = _now() + timeout;
        int seg = 0;
        const float look = 0.12f;
        bool ok = false;
        while (true)
        {
            var p = _r.Pos2;
            float dg = p.DistanceTo(goal);
            if (dg < 0.012f) { ok = true; break; }
            if (_now() > end) { Log($"FollowPath timeout, {dg * 1000:0}mm from goal"); break; }

            // Advance segment index to the closest segment.
            while (seg < path.Count - 2 && Project(p, path[seg + 1], path[seg + 2]).dist <= Project(p, path[seg], path[seg + 1]).dist)
                seg++;
            var target = Lookahead(path, seg, p, look);
            float alpha = Geo.Wrap(Geo.Angle(target - p) - _r.Theta);
            if (dg < 0.06f && Mathf.Abs(alpha) > 1.2f) { ok = true; break; } // beside the goal: let the caller align
            if (Mathf.Abs(alpha) > 0.9f)
            {
                _r.SetTwist(0, Mathf.Clamp(4f * alpha, -3f, 3f));
                await Tick();
                continue;
            }
            float ld = Mathf.Max(p.DistanceTo(target), 0.02f);
            float curv = 2f * Mathf.Sin(alpha) / ld;
            float v = vmax * (1f - 0.6f * Mathf.Abs(alpha));
            v = Mathf.Min(v, 0.04f + 1.5f * dg);
            _r.SetTwist(v, Mathf.Clamp(v * curv, -3f, 3f));
            await Tick();
        }
        await StopAndSettle();
        return ok;
    }

    static (float t, float dist) Project(Vector2 p, Vector2 a, Vector2 b)
    {
        var ab = b - a;
        float l2 = ab.LengthSquared();
        float t = l2 < 1e-9f ? 0 : Mathf.Clamp((p - a).Dot(ab) / l2, 0f, 1f);
        return (t, p.DistanceTo(a + ab * t));
    }

    static Vector2 Lookahead(List<Vector2> path, int seg, Vector2 p, float look)
    {
        var (t, _) = Project(p, path[seg], path[seg + 1]);
        var q = path[seg].Lerp(path[seg + 1], t);
        float remaining = look;
        for (int i = seg; i < path.Count - 1; i++)
        {
            var a = i == seg ? q : path[i];
            var b = path[i + 1];
            float l = a.DistanceTo(b);
            if (l >= remaining) return a + (b - a) * (remaining / Mathf.Max(l, 1e-6f));
            remaining -= l;
        }
        return path[^1];
    }
}
