using Godot;

namespace DominoSim;

public partial class Hud : CanvasLayer
{
    public MissionController Mission;
    public Main MainNode;
    Label _label;

    public override void _Ready()
    {
        _label = new Label { Position = new Vector2(12, 10) };
        _label.AddThemeFontSizeOverride("font_size", 17);
        _label.AddThemeColorOverride("font_color", Colors.White);
        _label.AddThemeColorOverride("font_outline_color", Colors.Black);
        _label.AddThemeConstantOverride("outline_size", 5);
        AddChild(_label);
    }

    public override void _Process(double delta)
    {
        if (Mission == null) return;
        double t = Mission.SimTime;
        string timer = t <= MissionController.MatchTime ? $"{MissionController.MatchTime - t:0.0} s left" : $"+{t - MissionController.MatchTime:0.0} s over 90 s";
        string last = Mission.JointLog.Count > 0 ? Mission.JointLog[^1].Trim() : "-";
        _label.Text =
            $"Domino sim   t = {t:0.0} s   ({timer})   speed x{MainNode?.SpeedFactor ?? 1}\n" +
            $"State: {Mission.State}\n" +
            $"Carried: {Mission.CarriedText()}   Placed: {Mission.PlacedCount}/16\n" +
            $"Last joint: {last}\n" +
            $"{Mission.Summary()}\n" +
            "[1/2/3/4] speed x1/x2/x4/x8   [T] top/orbit view   [F] follow robot   [R] restart   RMB drag orbit, wheel zoom";
    }
}
