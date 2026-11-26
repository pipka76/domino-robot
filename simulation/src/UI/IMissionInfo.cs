namespace DominoSim;

/// <summary>What the HUD shows; implemented by MissionController and OurRunner.</summary>
public interface IMissionInfo
{
    double SimTime { get; }
    string State { get; }
    string LastLine { get; }
    int PlacedCount { get; }
    string CarriedText();
    string Summary();
}
