using System;
using System.Linq;
using Godot;

namespace DominoSim;

/// <summary>
/// Entry point. User args (after "--"):
///   --selftest         run planner/geometry checks headless and quit (exit code 1 on failure)
///   --mission          run one autonomous match and quit when finished (use with --headless)
///   --round2 [seed]    rotate random pieces by 180° (allowed in round 2)
///   --robot RES_PATH   robot geometry file (default res://data/robot.json)
///   --speed N          initial simulation speed factor
///   --screenshot PATH  save a screenshot at --shot-time seconds of sim time (default 12) and quit
/// </summary>
public partial class Main : Node3D
{
    public const string LayoutPath = "res://data/layout_varianta_final.json";
    public int SpeedFactor { get; private set; } = 1;

    OrbitCamera _cam;
    MissionController _mission;
    string _shotPath;
    double _shotTime;

    public override void _Ready()
    {
        var args = OS.GetCmdlineUserArgs().ToList();
        var layout = FieldLayout.Load(LayoutPath);
        int ri = args.IndexOf("--robot");
        RobotSpec.Load(ri >= 0 && ri + 1 < args.Count ? args[ri + 1] : "res://data/robot.json");
        GD.Print(RobotSpec.Describe());

        if (args.Contains("--selftest"))
        {
            bool ok = SelfTest.Run(layout);
            GetTree().Quit(ok ? 0 : 1);
            return;
        }

        bool round2 = args.Contains("--round2");
        int seed = ArgInt(args, "--round2", 1);
        SetSpeed(ArgInt(args, "--speed", 1));

        // Environment and light.
        var env = new Godot.Environment
        {
            BackgroundMode = Godot.Environment.BGMode.Color,
            BackgroundColor = new Color(0.22f, 0.24f, 0.27f),
            AmbientLightSource = Godot.Environment.AmbientSource.Color,
            AmbientLightColor = Colors.White,
            AmbientLightEnergy = 0.55f,
        };
        AddChild(new WorldEnvironment { Environment = env });
        var sun = new DirectionalLight3D { ShadowEnabled = true, LightEnergy = 0.9f };
        AddChild(sun);
        sun.LookAtFromPosition(new Vector3(0.3f, 2f, 1.4f), new Vector3(0.625f, 0, 0.84f), Vector3.Up);

        var field = new FieldBuilder();
        AddChild(field);
        var dominos = field.Build(layout, round2, seed);

        var robot = new Robot();
        AddChild(robot);
        robot.GlobalTransform = new Transform3D(Geo.YawBasis(layout.RobotStartHeading), Geo.V3(layout.RobotStart, 0.002f));

        var mission = new MissionController { Robot = robot, Dominos = dominos, Layout = layout, AutoQuit = args.Contains("--mission") };
        AddChild(mission);
        _mission = mission;
        int si = args.IndexOf("--screenshot");
        if (si >= 0 && si + 1 < args.Count) _shotPath = args[si + 1];
        _shotTime = ArgInt(args, "--shot-time", 12);

        _cam = new OrbitCamera { Follow = robot };
        AddChild(_cam);
        _cam.MakeCurrent();

        AddChild(new Hud { Mission = mission, MainNode = this });
        GD.Print($"Domino sim started. round2={round2} seed={seed}");
    }

    static int ArgInt(System.Collections.Generic.List<string> args, string key, int def)
    {
        int i = args.IndexOf(key);
        if (i >= 0 && i + 1 < args.Count && int.TryParse(args[i + 1], out int v)) return v;
        return def;
    }

    void SetSpeed(int k)
    {
        SpeedFactor = Math.Clamp(k, 1, 16);
        // Keep the physics step at 1/120 s regardless of speed.
        Engine.TimeScale = SpeedFactor;
        Engine.PhysicsTicksPerSecond = 120 * SpeedFactor;
        Engine.MaxPhysicsStepsPerFrame = 16 * SpeedFactor;
    }

    public override void _Process(double delta)
    {
        if (_shotPath != null && _mission.SimTime >= _shotTime)
        {
            GetViewport().GetTexture().GetImage().SavePng(_shotPath);
            GD.Print($"Screenshot saved to {_shotPath}");
            _shotPath = null;
            GetTree().Quit();
        }
    }

    public override void _UnhandledInput(InputEvent e)
    {
        if (e is not InputEventKey { Pressed: true, Echo: false } k) return;
        switch (k.Keycode)
        {
            case Key.Key1: SetSpeed(1); break;
            case Key.Key2: SetSpeed(2); break;
            case Key.Key3: SetSpeed(4); break;
            case Key.Key4: SetSpeed(8); break;
            case Key.T: _cam?.ToggleTop(); break;
            case Key.F: _cam?.ToggleFollow(); break;
            case Key.R: GetTree().ReloadCurrentScene(); break;
        }
    }
}
