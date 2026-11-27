using System;
using System.Collections.Generic;
using System.IO;
using Godot;

namespace DominoSim;

/// <summary>Start screen: mode 1 (play a program) or mode 2 (drive manually and record).</summary>
public partial class ModeMenu : CanvasLayer
{
    /// <summary>(mode, program for Mode.Play, mission for Mode.Ours)</summary>
    public Action<Session.Mode, ProgramFile, string> Start;

    OptionButton _programs;
    Label _error;
    readonly List<Action> _choices = new();
    FileDialog _dialog;

    public override void _Ready()
    {
        var dim = new ColorRect { Color = new Color(0, 0, 0, 0.45f) };
        dim.SetAnchorsPreset(Control.LayoutPreset.FullRect);
        AddChild(dim);

        var center = new CenterContainer();
        center.SetAnchorsPreset(Control.LayoutPreset.FullRect);
        AddChild(center);
        var panel = new PanelContainer();
        panel.AddThemeStyleboxOverride("panel", new StyleBoxFlat
        {
            BgColor = new Color(0.13f, 0.14f, 0.17f, 0.97f),
            CornerRadiusTopLeft = 8, CornerRadiusTopRight = 8, CornerRadiusBottomLeft = 8, CornerRadiusBottomRight = 8,
        });
        center.AddChild(panel);
        var margin = new MarginContainer();
        foreach (var side in new[] { "left", "right", "top", "bottom" }) margin.AddThemeConstantOverride("margin_" + side, 24);
        panel.AddChild(margin);
        var box = new VBoxContainer { CustomMinimumSize = new Vector2(560, 0) };
        box.AddThemeConstantOverride("separation", 12);
        margin.AddChild(box);

        Label L(string t, int size = 16)
        {
            var l = new Label { Text = t, AutowrapMode = TextServer.AutowrapMode.WordSmart };
            l.AddThemeFontSizeOverride("font_size", size);
            box.AddChild(l);
            return l;
        }

        L("Domino – simulace", 26);
        L("1) Jízda podle programu", 20);
        L("Náš Python kód (main.py s mission.json), nebo uložený záznam kroků z módu 2. Simulace změří čas.");
        _programs = new OptionButton();
        box.AddChild(_programs);
        FillPrograms();
        var row = new HBoxContainer();
        box.AddChild(row);
        var play = new Button { Text = "▶  Spustit program", SizeFlagsHorizontal = Control.SizeFlags.ExpandFill };
        play.Pressed += OnPlay;
        row.AddChild(play);
        var open = new Button { Text = "Otevřít soubor…" };
        open.Pressed += () => _dialog.PopupCentered(new Vector2I(900, 600));
        row.AddChild(open);

        box.AddChild(new HSeparator());
        L("2) Ruční jízda se záznamem", 20);
        L("Řídíš robota klávesnicí, každý pohyb se zapíše jako krok (bez čekání). Záznam jde uložit a přehrát v módu 1.");
        var drive = new Button { Text = "🎮  Ruční jízda" };
        drive.Pressed += () => Start?.Invoke(Session.Mode.Drive, null, null);
        box.AddChild(drive);

        _error = L("");
        _error.AddThemeColorOverride("font_color", new Color(1, 0.45f, 0.4f));

        _dialog = new FileDialog
        {
            FileMode = FileDialog.FileModeEnum.OpenFile,
            Access = FileDialog.AccessEnum.Filesystem,
            Filters = new[] { "*.json ; Program (JSON)" },
            CurrentDir = ProgramFile.ProgramsDir,
            UseNativeDialog = true,
        };
        _dialog.FileSelected += path => TryStart(() => ProgramFile.Load(path));
        AddChild(_dialog);
    }

    void FillPrograms()
    {
        var missions = ProgramFile.MissionNames();
        string active = ProgramFile.ActiveMission();
        if (active != null && missions.Remove(active)) missions.Insert(0, active);
        if (missions.Remove("sim_demo")) missions.Insert(0, "sim_demo");   // ukázková jízda pro simulaci
        foreach (var m in missions)
        {
            string name = m;
            _programs.AddItem($"Náš Python kód – mise {m}" + (m == active ? "   (active_mission)" : m == "sim_demo" ? "   (ukázka pro simulaci)" : ""));
            _choices.Add(() => Start?.Invoke(Session.Mode.Ours, null, name));
        }
        foreach (var f in ProgramFile.SavedPrograms())
        {
            string path = f;
            _programs.AddItem("Záznam kroků: programs/" + Path.GetFileName(f));
            _choices.Add(() => TryStart(() => ProgramFile.Load(path)));
        }
        _programs.AddItem("Vestavěná autonomní strategie (původní simulace, jiný robot)");
        _choices.Add(() => Start?.Invoke(Session.Mode.Auto, null, null));
        _programs.Selected = 0;
    }

    void OnPlay() => _choices[_programs.Selected]();

    void TryStart(Func<ProgramFile> loader)
    {
        try { Start?.Invoke(Session.Mode.Play, loader(), null); }
        catch (Exception e) { _error.Text = "Chyba: " + e.Message; }
    }
}
