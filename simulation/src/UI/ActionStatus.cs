using System;

namespace DominoSim;

/// <summary>
/// The action the robot is busy with while standing still (motor C: hands, magazine shift, nudge; waits).
/// Set by whoever starts the action, shown by the HUD as a countdown next to the robot.
/// </summary>
public static class ActionStatus
{
    public static string Label { get; private set; }
    static double _start, _duration;
    static Func<double> _now = () => 0;

    public static void Begin(string label, double seconds, Func<double> now)
    {
        if (seconds <= 0.15) return;   // too short to be worth showing
        Label = label;
        _now = now;
        _start = now();
        _duration = seconds;
    }

    public static void Clear() => Label = null;

    /// <summary>(label, elapsed, total) of the running action, or null.</summary>
    public static (string label, double elapsed, double total)? Current
    {
        get
        {
            if (Label == null) return null;
            double e = _now() - _start;
            if (e >= _duration) { Label = null; return null; }
            return (Label, Math.Max(e, 0), _duration);
        }
    }
}
