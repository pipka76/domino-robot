using System;
using Godot;

namespace DominoSim;

/// <summary>Status text top-left, optional step list on the right, key help at the bottom.</summary>
public partial class Hud : CanvasLayer
{
    public Func<string> TextFn;
    public Func<string> SideFn;
    public string Help = "";
    Label _label, _side, _help;

    static Label MakeLabel(int size)
    {
        var l = new Label();
        l.AddThemeFontSizeOverride("font_size", size);
        l.AddThemeColorOverride("font_color", Colors.White);
        l.AddThemeColorOverride("font_outline_color", Colors.Black);
        l.AddThemeConstantOverride("outline_size", 5);
        return l;
    }

    public override void _Ready()
    {
        _label = MakeLabel(17);
        _label.Position = new Vector2(12, 10);
        AddChild(_label);

        _side = MakeLabel(15);
        _side.AddThemeFontOverride("font", new SystemFont { FontNames = new[] { "Menlo", "Consolas", "monospace" } });
        _side.SetAnchorsPreset(Control.LayoutPreset.TopRight);
        _side.GrowHorizontal = Control.GrowDirection.Begin;
        _side.Position = new Vector2(-12, 10);
        AddChild(_side);

        _help = MakeLabel(15);
        _help.SetAnchorsPreset(Control.LayoutPreset.BottomLeft);
        _help.GrowVertical = Control.GrowDirection.Begin;
        _help.Position = new Vector2(12, -10);
        AddChild(_help);
    }

    public override void _Process(double delta)
    {
        _label.Text = TextFn?.Invoke() ?? "";
        _side.Text = SideFn?.Invoke() ?? "";
        _help.Text = Help;
        // Keep the side panel glued to the right edge as its width changes.
        var vp = GetViewport().GetVisibleRect().Size;
        _side.Position = new Vector2(vp.X - _side.Size.X - 12, 10);
        _help.Position = new Vector2(12, vp.Y - _help.Size.Y - 10);
    }
}
