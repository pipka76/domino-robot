using System;
using System.Linq;
using Godot;

namespace DominoSim;

/// <summary>
/// Entry point. Without arguments a start menu offers the two modes:
///   1) play a program (list of steps in the mission.json format) and measure its time,
///   2) drive manually with the keyboard and record the movements as steps.
/// Mode 1 can also run OUR Python code (../main.py --sim MISSION through hw_sim.py, see OurRunner).
/// User args (after "--"):
///   --selftest           run planner/geometry checks headless and quit (exit code 1 on failure)
///   --ours [MISSION]     mode 1 with our Python code (default mission main)
///   --theirs             mode 1 with the built-in autonomous strategy (MissionController, robot.json)
///   --mission            quit when the run ends (use with --headless); alone = --ours main
///   --no-python          with --ours: do not launch Python (run `python3 main.py --sim MISSION` yourself)
///   --start X Z DEG      start pose for our robot (axle x, z in mm; heading 90 = facing down)
///   --program PATH       mode 1 with this program (array of steps, {"steps": [...]}, or mission.json)
///   --mission-name NAME  with --program mission.json: which mission (default active_mission)
///   --drive              mode 2 (manual driving + recording)
///   --quit               with --program: quit when the program ends (use with --headless)
///   --round2 [seed]      rotate random pieces by 180° (allowed in round 2)
///   --robot RES_PATH     robot geometry file (default res://data/robot.json)
///   --speed N            initial simulation speed factor
///   --screenshot PATH    save a screenshot at --shot-time seconds of sim time (default 12) and quit
/// </summary>
public partial class Main : Node3D
{
    public const string LayoutPath = "res://data/layout_varianta_final.json";
    public int SpeedFactor { get; private set; } = 1;
    public static Vector2 StartPos;
    public static float StartHeading;

    OrbitCamera _cam;
    Func<double> _simTime = () => 0;
    ManualDriver _driver;
    string _shotPath;
    double _shotTime;

    const string PlayHelp = "[1/2/3/4] rychlost x1/x2/x4/x8   [R] znovu   [Esc] menu   [T] pohled shora/orbit   [F] sledovat robota   pravé tl. myši = orbit, kolečko = zoom";
    const string DriveHelp =
        "[↑↓←→ / WASD] jízda, otáčení, ↑+←/→ oblouk   [Shift] pomalu (oblouk kolem kola)   [Q/E] otočit přesně o 90° vlevo/vpravo\n" +
        "[J/L] spustit levé/pravé rameno  [K] zvednout   [Mezerník] vyložit   [Backspace] zpět (smaže poslední krok)\n" +
        "[Ctrl+S] uložit   [Enter] uložit a přehrát v módu 1   [R] začít znovu   [Esc] menu   [T/F] kamera";

    public override void _Ready()
    {
        var args = OS.GetCmdlineUserArgs().ToList();
        var layout = FieldLayout.Load(LayoutPath);
        if (!Session.ArgsApplied && !args.Contains("--selftest"))
        {
            Session.ArgsApplied = true;
            SetSpeed(ArgInt(args, "--speed", 1));
            int pi = args.IndexOf("--program");
            int oi = args.IndexOf("--ours");
            if (args.Contains("--theirs")) Session.Current = Session.Mode.Auto;
            else if (args.Contains("--drive")) Session.Current = Session.Mode.Drive;
            else if (pi >= 0 && pi + 1 < args.Count)
            {
                int mi = args.IndexOf("--mission-name");
                Session.Program = ProgramFile.Load(System.IO.Path.GetFullPath(args[pi + 1]), mi >= 0 && mi + 1 < args.Count ? args[mi + 1] : null);
                Session.Current = Session.Mode.Play;
            }
            else if (oi >= 0 || args.Contains("--mission"))
            {
                Session.Current = Session.Mode.Ours;
                Session.OursMission = oi >= 0 && oi + 1 < args.Count && !args[oi + 1].StartsWith("--") ? args[oi + 1] : "main";
            }
        }
        // Our robot everywhere except the original autonomous strategy (built for its own robot concept).
        bool oursRobot = Session.Current != Session.Mode.Auto;
        int ri = args.IndexOf("--robot");
        RobotSpec.Load(ri >= 0 && ri + 1 < args.Count ? args[ri + 1] : oursRobot ? "res://data/robot_ours.json" : "res://data/robot.json");
        DriveConfig.Load();
        GD.Print(RobotSpec.Describe());
        GD.Print(DriveConfig.Describe());

        if (args.Contains("--selftest"))
        {
            bool ok = SelfTest.Run(layout);
            GetTree().Quit(ok ? 0 : 1);
            return;
        }
        if (Session.Current == Session.Mode.Drive) SetSpeed(1);

        bool round2 = args.Contains("--round2");
        int seed = ArgInt(args, "--round2", 1);

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
        var startPos = layout.RobotStart;
        float startHeading = layout.RobotStartHeading;
        if (oursRobot)
        {
            // Our start: magazine (rear, 190 mm behind the axle) against the top wall, above the left row, facing down.
            startPos = new Vector2(0.132f, RobotSpec.Length - RobotSpec.AxleFromFront + 0.002f);
            startHeading = Mathf.Pi / 2f;
            int st = args.IndexOf("--start");
            if (st >= 0 && st + 3 < args.Count)
            {
                startPos = new Vector2(float.Parse(args[st + 1], System.Globalization.CultureInfo.InvariantCulture) * 0.001f,
                                       float.Parse(args[st + 2], System.Globalization.CultureInfo.InvariantCulture) * 0.001f);
                startHeading = Mathf.DegToRad(float.Parse(args[st + 3], System.Globalization.CultureInfo.InvariantCulture));
            }
        }
        if (Session.Current == Session.Mode.Play && Session.Program?.Start is { } s)
        {
            startPos = new Vector2(s.x, s.z) * 0.001f;
            startHeading = Mathf.DegToRad(s.heading);
        }
        StartPos = startPos;
        StartHeading = startHeading;
        robot.GlobalTransform = new Transform3D(Geo.YawBasis(startHeading), Geo.V3(startPos, 0.002f));

        int si = args.IndexOf("--screenshot");
        if (si >= 0 && si + 1 < args.Count) _shotPath = args[si + 1];
        _shotTime = ArgInt(args, "--shot-time", 12);

        _cam = new OrbitCamera { Follow = robot };
        AddChild(_cam);
        _cam.MakeCurrent();

        switch (Session.Current)
        {
            case Session.Mode.Menu:
                AddChild(new ModeMenu
                {
                    Start = (mode, program, mission) =>
                    {
                        Session.Current = mode;
                        Session.Program = program;
                        Session.OursMission = mission ?? Session.OursMission;
                        Session.PendingSteps = null;
                        GetTree().ReloadCurrentScene();
                    },
                });
                break;
            case Session.Mode.Auto:
            {
                var mission = new MissionController { Robot = robot, Dominos = dominos, Layout = layout, AutoQuit = args.Contains("--mission") };
                AddChild(mission);
                _simTime = () => mission.SimTime;
                AddChild(new Hud { TextFn = () => InfoText("MÓD 1 – vestavěná autonomní strategie", mission), Help = PlayHelp });
                break;
            }
            case Session.Mode.Ours:
            {
                var runner = new OurRunner
                {
                    Robot = robot, Dominos = dominos, Layout = layout, MissionName = Session.OursMission,
                    RepoDir = ProjectSettings.GlobalizePath("res://").TrimEnd('/') + "/..",
                    LaunchPython = !args.Contains("--no-python"), AutoQuit = args.Contains("--mission"),
                };
                AddChild(runner);
                _simTime = () => runner.SimTime;
                AddChild(new Hud { TextFn = () => InfoText($"MÓD 1 – náš Python kód, mise '{Session.OursMission}'", runner), Help = PlayHelp });
                break;
            }
            case Session.Mode.Play:
            {
                var runner = new ProgramRunner { Robot = robot, Dominos = dominos, Layout = layout, Program = Session.Program, AutoQuit = args.Contains("--quit") };
                AddChild(runner);
                _simTime = () => runner.SimTime;
                AddChild(new Hud { TextFn = () => runner.StatusText() + $"   rychlost x{SpeedFactor}", SideFn = runner.SideText, Help = PlayHelp });
                break;
            }
            case Session.Mode.Drive:
            {
                _driver = new ManualDriver { Robot = robot, Dominos = dominos, Layout = layout, MainNode = this };
                AddChild(_driver);
                _simTime = () => _driver.SimTime;
                AddChild(new Hud { TextFn = _driver.StatusText, SideFn = _driver.SideText, Help = DriveHelp });
                break;
            }
        }
        GD.Print($"Domino sim started. mode={Session.Current} round2={round2} seed={seed}");
    }

    string InfoText(string title, IMissionInfo m)
    {
        double t = m.SimTime;
        string timer = t <= MissionController.MatchTime ? $"zbývá {MissionController.MatchTime - t:0.0} s" : $"+{t - MissionController.MatchTime:0.0} s přes 90 s";
        return $"{title}   t = {t:0.0} s   ({timer})   rychlost x{SpeedFactor}\n" +
               $"Stav: {m.State}\n" +
               $"Zásobník: {m.CarriedText()}   Položeno: {m.PlacedCount}/16\n" +
               $"Poslední: {m.LastLine}\n" +
               m.Summary();
    }

    static int ArgInt(System.Collections.Generic.List<string> args, string key, int def)
    {
        int i = args.IndexOf(key);
        if (i >= 0 && i + 1 < args.Count && int.TryParse(args[i + 1], out int v)) return v;
        return def;
    }

    public void SetSpeed(int k)
    {
        SpeedFactor = Math.Clamp(k, 1, 16);
        // Keep the physics step at 1/120 s regardless of speed.
        Engine.TimeScale = SpeedFactor;
        Engine.PhysicsTicksPerSecond = 120 * SpeedFactor;
        Engine.MaxPhysicsStepsPerFrame = 16 * SpeedFactor;
    }

    public override void _Process(double delta)
    {
        if (_shotPath != null && _simTime() >= _shotTime && Engine.GetProcessFrames() > 10)
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
        var key = k.PhysicalKeycode;
        switch (key)
        {
            case Key.Escape:
                Session.Current = Session.Mode.Menu;
                Session.PendingSteps = null;
                SetSpeed(1);
                GetTree().ReloadCurrentScene();
                return;
            case Key.R:
                Session.PendingSteps = null;
                GetTree().ReloadCurrentScene();
                return;
            case Key.T: _cam?.ToggleTop(); return;
            case Key.F: _cam?.ToggleFollow(); return;
        }
        if (_driver != null)
        {
            switch (key)
            {
                case Key.Q: _driver.ExactTurn(90); break;
                case Key.E: _driver.ExactTurn(-90); break;
                case Key.J: _driver.SetArm("LEFT"); break;
                case Key.L: _driver.SetArm("RIGHT"); break;
                case Key.K: _driver.SetArm("NONE"); break;
                case Key.Space: _driver.Unload(); break;
                case Key.Backspace: _driver.Undo(); break;
                case Key.S when k.CtrlPressed || k.MetaPressed: _driver.Save(); break;
                case Key.Enter or Key.KpEnter:
                    if (_driver.Save() is { } path)
                    {
                        Session.Program = ProgramFile.Load(path);
                        Session.Current = Session.Mode.Play;
                        GetTree().ReloadCurrentScene();
                    }
                    break;
            }
            return;
        }
        switch (key)
        {
            case Key.Key1: SetSpeed(1); break;
            case Key.Key2: SetSpeed(2); break;
            case Key.Key3: SetSpeed(4); break;
            case Key.Key4: SetSpeed(8); break;
        }
    }
}
