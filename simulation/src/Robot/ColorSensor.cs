using System.Collections.Generic;
using Godot;

namespace DominoSim;

/// <summary>
/// Two downward-looking colour sensors on a bracket ahead of the robot, at ±24 mm from the
/// centre line: when the robot stands at a capture approach point they look at the centres
/// of the left and right colour halves of the crosswise piece in front of it.
/// </summary>
public static class ColorSensor
{
    public static float SensorX => Robot.BodyFront + RobotSpec.SensorAhead;
    public const float SensorZ = 0.024f;

    public static (PColor left, PColor right) Read(Robot r, IEnumerable<Domino> pieces)
    {
        var pl = r.LocalToField(new Vector2(SensorX, -SensorZ));
        var pr = r.LocalToField(new Vector2(SensorX, SensorZ));
        PColor cl = PColor.None, cr = PColor.None;
        foreach (var d in pieces)
        {
            if (d.Carried) continue;
            var a = d.ColorAt(pl);
            if (a != PColor.None) cl = a;
            var b = d.ColorAt(pr);
            if (b != PColor.None) cr = b;
        }
        return (cl, cr);
    }
}
