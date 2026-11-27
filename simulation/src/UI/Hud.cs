using System;
using System.Collections.Generic;
using Godot;

namespace DominoSim;

/// <summary>
/// HUD: a compact panel top-left (big match timer, score, short status), an optional step list on the right,
/// a countdown bubble next to the robot while it is busy (motor C, waits) and the key help on F1.
/// </summary>
public partial class Hud : CanvasLayer
{
    public string Title = "";
    public Func<double> TimeFn = () => 0;
    public Func<int> SpeedFn = () => 1;
    public Func<string> InfoFn;
    public Func<string> MechFn;
    public Func<string> SideFn;
    public string Help = "";
    public List<Domino> Dominos;
    public FieldLayout Layout;
    public Robot Robot;
    public Camera3D Camera;

    static readonly Color PanelBg = new(0.08f, 0.09f, 0.11f, 0.82f);
    static readonly Color Muted = new(0.62f, 0.66f, 0.72f);
    static readonly Color Accent = new(0.98f, 0.62f, 0.16f);
    static readonly Color Good = new(0.35f, 0.82f, 0.45f);
    static readonly Color Bad = new(0.95f, 0.35f, 0.3f);

    Label _title, _time, _limit, _score, _scoreSub, _info, _mech, _side, _helpHint, _help, _actLabel;
    ProgressBar _matchBar, _actBar;
    StyleBoxFlat _matchFill;
    PanelContainer _sidePanel, _helpPanel, _act;
    Scorer.Result _sc;
    double _scoreAge = 99;

    static StyleBoxFlat Box(Color bg, int radius = 10, int pad = 14) => new()
    {
        BgColor = bg,
        CornerRadiusTopLeft = radius, CornerRadiusTopRight = radius, CornerRadiusBottomLeft = radius, CornerRadiusBottomRight = radius,
        ContentMarginLeft = pad, ContentMarginRight = pad, ContentMarginTop = pad - 2, ContentMarginBottom = pad - 2,
    };

    static Label L(Control parent, int size, Color? color = null, Font font = null)
    {
        var l = new Label();
        l.AddThemeFontSizeOverride("font_size", size);
        l.AddThemeColorOverride("font_color", color ?? Colors.White);
        if (font != null) l.AddThemeFontOverride("font", font);
        parent.AddChild(l);
        return l;
    }

    static ProgressBar Bar(Control parent, Color fill, float height, out StyleBoxFlat fillBox)
    {
        var b = new ProgressBar { ShowPercentage = false, MaxValue = 1, CustomMinimumSize = new Vector2(0, height) };
        b.AddThemeStyleboxOverride("background", Box(new Color(1, 1, 1, 0.12f), 3, 0));
        fillBox = Box(fill, 3, 0);
        b.AddThemeStyleboxOverride("fill", fillBox);
        parent.AddChild(b);
        return b;
    }

    public override void _Ready()
    {
        ActionStatus.Clear();
        var mono = new SystemFont { FontNames = new[] { "Menlo", "SF Mono", "Consolas", "monospace" } };

        // Main panel.
        var panel = new PanelContainer { Position = new Vector2(16, 16), CustomMinimumSize = new Vector2(300, 0) };
        panel.AddThemeStyleboxOverride("panel", Box(PanelBg));
        AddChild(panel);
        var v = new VBoxContainer();
        v.AddThemeConstantOverride("separation", 4);
        panel.AddChild(v);
        _title = L(v, 14, Muted);
        var timeRow = new HBoxContainer();
        timeRow.AddThemeConstantOverride("separation", 8);
        v.AddChild(timeRow);
        _time = L(timeRow, 44);
        _limit = L(timeRow, 16, Muted);
        _limit.SizeFlagsVertical = Control.SizeFlags.ShrinkEnd;
        _limit.Text = "/ 90 s";
        _matchBar = Bar(v, Good, 6, out _matchFill);
        v.AddChild(new Control { CustomMinimumSize = new Vector2(0, 6) });
        _score = L(v, 22);
        _scoreSub = L(v, 14, Muted);
        _info = L(v, 15);
        _mech = L(v, 14, Muted, mono);

        // Step list on the right.
        _sidePanel = new PanelContainer();
        _sidePanel.AddThemeStyleboxOverride("panel", Box(PanelBg));
        AddChild(_sidePanel);
        _side = L(_sidePanel, 14, null, mono);

        // Countdown next to the robot.
        _act = new PanelContainer { Visible = false };
        _act.AddThemeStyleboxOverride("panel", Box(new Color(0.08f, 0.09f, 0.11f, 0.9f), 8, 10));
        AddChild(_act);
        var av = new VBoxContainer();
        av.AddThemeConstantOverride("separation", 4);
        _act.AddChild(av);
        _actLabel = L(av, 15, Accent);
        _actBar = Bar(av, Accent, 5, out _);
        _actBar.CustomMinimumSize = new Vector2(170, 5);

        // Help.
        var hintBox = new Control();
        AddChild(hintBox);
        _helpHint = L(hintBox, 13, Muted);
        _helpHint.Text = "F1 nápověda";
        _helpPanel = new PanelContainer { Visible = false };
        _helpPanel.AddThemeStyleboxOverride("panel", Box(PanelBg));
        AddChild(_helpPanel);
        _help = L(_helpPanel, 15, null, mono);
    }

    public override void _UnhandledInput(InputEvent e)
    {
        if (e is InputEventKey { Pressed: true, Echo: false } k && k.PhysicalKeycode == Key.F1)
        {
            _helpPanel.Visible = !_helpPanel.Visible;
            GetViewport().SetInputAsHandled();
        }
    }

    public override void _Process(double delta)
    {
        var vp = GetViewport().GetVisibleRect().Size;
        double t = TimeFn();
        int speed = SpeedFn();
        _title.Text = Title.ToUpperInvariant() + (speed > 1 ? $"   ×{speed}" : "");
        _time.Text = $"{t:0.0} s";
        bool over = t > MissionController.MatchTime;
        _time.AddThemeColorOverride("font_color", over ? Bad : Colors.White);
        _matchBar.Value = Math.Min(t / MissionController.MatchTime, 1);
        _matchFill.BgColor = over ? Bad : t > MissionController.MatchTime - 10 ? Accent : Good;

        // Score: the chain search is not free, refresh a few times a second.
        _scoreAge += delta;
        if (_scoreAge > 0.25 && Dominos != null)
        {
            _sc = Scorer.Score(Dominos, Layout);
            _scoreAge = 0;
        }
        _score.Text = _sc.Chain >= 2 ? $"Řetěz {_sc.Chain}  ·  {_sc.Points} b." : "Řetěz –";
        _scoreSub.Text = $"ve žlutém {_sc.InYellow} kostek" + (_sc.Perp > 0 ? $", {_sc.Perp}× kolmý spoj" : "");
        Set(_info, InfoFn?.Invoke());
        Set(_mech, MechFn?.Invoke());

        string side = SideFn?.Invoke();
        _sidePanel.Visible = !string.IsNullOrEmpty(side);
        if (_sidePanel.Visible)
        {
            _side.Text = side;
            _sidePanel.ResetSize();
            _sidePanel.Position = new Vector2(vp.X - _sidePanel.Size.X - 16, 16);
        }

        _helpHint.Position = new Vector2(16, vp.Y - _helpHint.Size.Y - 12);
        _help.Text = Help;
        _helpPanel.ResetSize();
        _helpPanel.Position = new Vector2(16, vp.Y - _helpPanel.Size.Y - 40);

        // Countdown bubble above the robot.
        var cur = ActionStatus.Current;
        _act.Visible = cur != null && Robot != null && Camera != null;
        if (_act.Visible)
        {
            var (label, elapsed, total) = cur.Value;
            _actLabel.Text = $"{label}   {total - elapsed:0.0} s";
            _actBar.Value = elapsed / total;
            var p = Camera.UnprojectPosition(Robot.GlobalPosition + new Vector3(0, 0.15f, 0));
            _act.ResetSize();
            _act.Position = (p - new Vector2(_act.Size.X / 2f, _act.Size.Y + 70)).Clamp(Vector2.Zero, vp - _act.Size);
        }
    }

    static void Set(Label l, string text)
    {
        l.Visible = !string.IsNullOrEmpty(text);
        l.Text = text ?? "";
    }
}
