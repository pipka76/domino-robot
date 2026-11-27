using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Godot;

namespace DominoSim;

/// <summary>
/// Runs OUR Python robot code (main.py --sim, see hw_sim.py in the repo root) against this simulator.
/// Python is the brain (actions.py, mechanism.py, mission.json); this node is the hardware:
/// wheels (deg/s), encoders, gyro (deg, CCW+), colour sensor, motor C state (hands + magazine).
///
/// Lock-step: physics only advances while Python is in hw.sleep(); every other request is answered
/// at the same sim instant, so Python's own compute time costs no sim time.
///
/// Mechanism model (our robot):
///  * Hands (rot CW/CCW) lowered = a crosswise piece reaching the pick-up line (175 mm ahead of the
///    axle) is captured and turned 90° while the robot drives on. NONE = no capture.
///    Turn direction: slot RIGHT + CCW or slot LEFT + CW = left end (sensor side) goes in front.
///  * Magazine: two lanes (LEFT/RIGHT) side by side, 2 pieces each. The active lane (= current slot)
///    is under the intake. A shift swaps the active lane immediately, so a piece still being
///    captured ends up in the new lane (as on the real robot).
///  * Reversing ≥ ReleaseMm with pieces in the active lane leaves them on the floor (unload).
/// </summary>
public partial class OurRunner : Node, IMissionInfo
{
    public Robot Robot;
    public List<Domino> Dominos;
    public FieldLayout Layout;
    public string MissionName = "main";
    public string RepoDir;
    public bool LaunchPython = true;
    public bool AutoQuit;
    public int Port = 5577;

    // Sensor / mechanism geometry (robot frame: x forward, z right; metres).
    static Vector2 SensorLocal => OurMechanism.SensorLocal;

    public double SimTime { get; private set; }
    public string State { get; private set; } = "čekám na Python…";
    public string LastLine { get; private set; } = "-";

    TcpListener _srv;
    TcpClient _cl;
    StreamReader _rd;
    StreamWriter _wr;
    double _wake = -1;          // sim time at which the pending sleep ends; <0 = Python is not sleeping
    bool _done;
    float _unwrapped, _lastTheta;

    public override void _Ready()
    {
        Name = "OurRunner";
        Robot.Mech.Candidates = () => Dominos;
        Robot.Mech.Log = m => GD.Print($"[{SimTime,6:0.00}] {m}");
        var init = DriveConfig.ArmInitial;
        Robot.Mech.SetState(init.rot, init.slot);
        _lastTheta = Robot.Theta;
        _srv = new TcpListener(IPAddress.Loopback, Port);
        _srv.Start();
        GD.Print($"[ours] čekám na Python na 127.0.0.1:{Port} (mise {MissionName})");
        if (LaunchPython)
        {
            string log = Path.Combine(RepoDir, "simulation", "ours_python.log");
            string cmd = $"cd '{RepoDir}' && python3 main.py --sim {MissionName} > '{log}' 2>&1";
            int pid = OS.CreateProcess("/bin/sh", new[] { "-c", cmd });
            GD.Print($"[ours] spuštěn Python (pid {pid}), chyby Pythonu: {log}");
        }
    }

    public override void _PhysicsProcess(double delta)
    {
        UpdateGyro();
        if (_done) return;
        if (_cl == null)
        {
            if (!_srv.Pending()) return;
            _cl = _srv.AcceptTcpClient();
            _cl.NoDelay = true;
            _cl.ReceiveTimeout = 30000;
            var ns = _cl.GetStream();
            _rd = new StreamReader(ns, Encoding.UTF8);
            _wr = new StreamWriter(ns, new UTF8Encoding(false)) { AutoFlush = true, NewLine = "\n" };
            State = "běží";
            Serve();
            return;
        }
        if (_wake < 0) return;           // should not happen: Serve() blocks until a sleep
        SimTime += delta;
        if (SimTime + 1e-6 >= _wake)
        {
            _wake = -1;
            Reply("ok");
            Serve();
        }
    }

    /// <summary>Answer Python requests until it asks to sleep (or finishes).</summary>
    void Serve()
    {
        while (!_done)
        {
            string line;
            try { line = _rd.ReadLine(); }
            catch (Exception e) { Finish("spojení: " + e.Message); return; }
            if (line == null) { Finish("Python skončil"); return; }
            var a = line.Split(' ', 2);
            string cmd = a[0], arg = a.Length > 1 ? a[1] : "";
            switch (cmd)
            {
                case "sleep":
                    _wake = SimTime + Math.Max(0.0, double.Parse(arg, System.Globalization.CultureInfo.InvariantCulture));
                    return;
                case "time": Reply($"ok {F(SimTime)}"); break;
                case "drive":
                {
                    var v = arg.Split(' ');
                    float k = Mathf.Pi / 180f * RobotSpec.WheelRadius; // wheel deg/s -> m/s
                    Robot.SetWheelSpeeds(P(v[0]) * k, P(v[1]) * k);
                    Reply("ok");
                    break;
                }
                case "enc": Reply($"ok {F(Mathf.RadToDeg((float)Robot.EncoderL))} {F(Mathf.RadToDeg((float)Robot.EncoderR))}"); break;
                case "gyro": Reply($"ok {F(-Mathf.RadToDeg(_unwrapped))}"); break;
                case "colour": Reply($"ok {SensorColour().name}"); break;
                case "reflected": Reply($"ok {SensorColour().refl}"); break;
                case "arm":
                {
                    var v = arg.Split(' ');
                    Robot.Mech.SetState(v[0], v[1]);
                    Reply("ok");
                    break;
                }
                case "log":
                    LastLine = arg.Trim();
                    GD.Print($"[{SimTime,6:0.00}] {arg}");
                    Reply("ok");
                    break;
                case "done":
                    Reply("ok");
                    Finish("hotovo");
                    return;
                default: Reply("err neznámý příkaz " + cmd); break;
            }
        }
    }

    void Reply(string s)
    {
        try { _wr.WriteLine(s); }
        catch (Exception e) { Finish("spojení: " + e.Message); }
    }

    void Finish(string why)
    {
        if (_done) return;
        _done = true;
        Robot.Stop();
        State = why;
        GD.Print($"=== [ours] {why} v {SimTime:0.0} s; {Summary()}");
        try { _cl?.Close(); _srv?.Stop(); } catch { }
        if (AutoQuit) GetTree().Quit();
    }

    public override void _ExitTree()
    {
        try { _cl?.Close(); _srv?.Stop(); } catch { }
    }

    static string F(double v) => v.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);
    static float P(string s) => float.Parse(s, System.Globalization.CultureInfo.InvariantCulture);

    void UpdateGyro()
    {
        float th = Robot.Theta;
        _unwrapped += Geo.Wrap(th - _lastTheta);
        _lastTheta = th;
    }

    // ------------------------------------------------------------------ sensor

    (string name, int refl) SensorColour()
    {
        var p = Robot.LocalToField(SensorLocal);
        foreach (var d in Dominos)
        {
            if (d.Carried) continue;
            var c = d.ColorAt(p);
            if (c == PColor.None) continue;
            return c switch
            {
                PColor.Red => ("RED", 40),
                PColor.Yellow => ("YELLOW", 58),
                PColor.Green => ("GREEN", 8),
                PColor.Blue => ("BLUE", 8),
                _ => ("BLACK", 5),
            };
        }
        return ("WHITE", 17); // podlaha (reálný senzor ji čte jako GREEN, ale odražené světlo ~17)
    }

    // ------------------------------------------------------------------ HUD

    public string CarriedText() => Robot.Mech.Text();

    public int PlacedCount => Robot.Mech.Placed.Count;

    public string Summary()
    {
        int inYellow = Dominos.Count(d => !d.Carried && d.Footprint.Corners().Any(c => Layout.YellowRect.HasPoint(c)));
        int joints = 0;
        var free = Dominos.Where(d => !d.Carried && d.Upright).ToList();
        for (int i = 0; i < free.Count; i++)
            for (int j = i + 1; j < free.Count; j++)
                if (free[i].Pos2.DistanceTo(free[j].Pos2) < 0.13f && JointEvaluator.Evaluate(free[i].Pose, free[j].Pose).Valid)
                    joints++;
        return $"kostek ve žlutém={inYellow}, platných spojů={joints}";
    }
}
