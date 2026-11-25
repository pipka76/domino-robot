using System.Text.Json;
using Godot;

namespace DominoSim;

/// <summary>Robot geometry (metres), loaded from data/robot*.json. Robot frame: +X forward, +Z right, origin on the axle centre at floor level.</summary>
public static class RobotSpec
{
    public static float Length = 0.28f, Width = 0.20f, BodyWidth = 0.20f, AxleFromFront = 0.14f;
    public static float CornerRadius = 0f, SensorAhead = 0.05f;
    public static float WheelRadius = 0.028f, WheelWidth = 0.016f, GroundClearance = 0.03f, LaneHalfWidth = 0.05f;
    public static string Source = "defaults";

    public static float BodyFront => AxleFromFront;
    public static float BodyRear => AxleFromFront - Length;
    public static float BodyHalfWidth => BodyWidth / 2f;
    public static float HalfTrack => Width / 2f - WheelWidth / 2f;
    public static float BodyBottom => GroundClearance;
    public static float BodyTop => GroundClearance + 0.08f;
    public static float CasterRadius => 0.012f;
    public static float CasterX => BodyRear + 0.025f;
    public static float CasterZ => LaneHalfWidth - 0.005f;
    /// <summary>Radius swept when turning on the spot.</summary>
    public static float TurnRadius
    {
        get
        {
            float fx = Mathf.Max(BodyFront, -BodyRear);
            float r = Mathf.Min(CornerRadius, Mathf.Min(fx, BodyHalfWidth));
            float body = new Vector2(fx - r, BodyHalfWidth - r).Length() + r;
            float wheel = new Vector2(WheelRadius, Width / 2f).Length();
            return Mathf.Max(body, wheel);
        }
    }

    /// <summary>Largest distance from the axle centre of any part that touches the floor (wheels, casters).</summary>
    public static float FloorPartsRadius =>
        Mathf.Max(new Vector2(WheelRadius, Width / 2f).Length(), new Vector2(CasterX, CasterZ).Length() + CasterRadius);

    public static void Load(string resPath)
    {
        string text = FileAccess.GetFileAsString(resPath);
        if (string.IsNullOrEmpty(text)) { GD.PushError($"Cannot read robot spec {resPath}"); return; }
        using var doc = JsonDocument.Parse(text);
        var r = doc.RootElement;
        float Mm(string k, float def) => r.TryGetProperty(k, out var v) ? v.GetSingle() * 0.001f : def;
        Length = Mm("length", Length);
        Width = Mm("width", Width);
        BodyWidth = Mm("bodyWidth", Width);
        AxleFromFront = Mm("axleFromFront", Length / 2f);
        WheelRadius = Mm("wheelDiameter", WheelRadius * 2000f * 0.001f) / 2f;
        WheelWidth = Mm("wheelWidth", WheelWidth);
        GroundClearance = Mm("groundClearance", GroundClearance);
        LaneHalfWidth = Mm("laneHalfWidth", LaneHalfWidth);
        CornerRadius = Mm("cornerRadius", 0f);
        SensorAhead = Mm("sensorAhead", 0.05f);
        Source = resPath;
    }

    public static string Describe() =>
        $"robot {Length * 1000:0}x{Width * 1000:0} mm (body width {BodyWidth * 1000:0}, corner r {CornerRadius * 1000:0}), axle {AxleFromFront * 1000:0} mm from front, " +
        $"wheel centres {2 * HalfTrack * 1000:0} mm apart, turn-on-spot radius {TurnRadius * 1000:0} mm  [{Source}]";
}
