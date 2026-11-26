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
    static readonly Vector2 SensorLocal = new(0.210f, -0.024f); // left side, sees one half of a crosswise piece
    const float PickX = 0.175f;            // front edge of the pick-up
    const float LaneOffset = 0.045f;       // inactive lane sideways offset (drawing only)
    const float ReleaseMm = 50f;           // reversing this far drops the active lane

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

    // Mechanism state.
    string _rot = "CCW", _slot = "RIGHT";
    readonly Dictionary<string, List<(Domino d, float psi)>> _lanes = new() { ["LEFT"] = new(), ["RIGHT"] = new() };
    Domino _cap;
    Vector2 _capStart;
    float _capPsi0, _capSign, _capTravel0, _capNeeded;
    string _capLane;
    float _travel, _reverse;
    Vector2 _lastPos;
    bool _hasLast;
    readonly List<Domino> _released = new();
    readonly List<Domino> _placed = new();

    public override void _Ready()
    {
        Name = "OurRunner";
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
        UpdateMechanism();
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
                    SetArm(v[0], v[1]);
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

    // ------------------------------------------------------------------ mechanism

    void SetArm(string rot, string slot)
    {
        if (slot != _slot)
        {
            GD.Print($"[{SimTime,6:0.00}] [sim] posun zásobníku {_slot} -> {slot}" + (_cap != null ? " (kostka ještě nedojela -> jde do nového slotu!)" : ""));
            if (_cap != null) _capLane = slot;
            _reverse = 0f; // nový slot vyjede až při couvání po posunu
        }
        _rot = rot;
        _slot = slot;
    }

    static float LaneX(int index) => PickX - 0.01f - Domino.L / 2f - (1 - index) * Domino.L; // 0 = rear, 1 = front

    float LaneZ(string lane) => lane == _slot ? 0f : (lane == "LEFT" ? -LaneOffset : LaneOffset);

    void UpdateMechanism()
    {
        var pos = Robot.Pos2;
        float th = Robot.Theta;
        float step = _hasLast ? (pos - _lastPos).Dot(Geo.Dir(th)) : 0f;
        _travel += step;
        _lastPos = pos;
        _hasLast = true;

        // Capture in progress: driven by forward travel.
        if (_cap != null)
        {
            var lane = _lanes[_capLane];
            int idx = Math.Min(lane.Count, 1);
            float tx = LaneX(idx);
            float t = Mathf.Clamp((_travel - _capTravel0) / _capNeeded, 0f, 1f);
            var local = new Vector2(Mathf.Lerp(_capStart.X, tx, t), Mathf.Lerp(_capStart.Y, LaneZ(_capLane), t));
            float psi = _capPsi0 + _capSign * Mathf.Pi / 2f * t;
            if (t >= 1f)
            {
                psi = Mathf.Abs(Geo.Wrap(psi)) < Mathf.Pi / 2f ? 0f : Mathf.Pi;
                lane.Add((_cap, psi));
                GD.Print($"[{SimTime,6:0.00}] [sim] kostka #{_cap.Id} v zásobníku {_capLane} ({lane.Count}/2)");
                _cap = null;
            }
            else SetPiece(_cap, local, psi);
        }
        else if (_rot != "NONE" && _lanes[_slot].Count < 2)
        {
            TryCapture(pos, th);
        }

        foreach (var kv in _lanes)
            for (int i = 0; i < kv.Value.Count; i++)
                SetPiece(kv.Value[i].d, new Vector2(LaneX(kv.Value.Count == 1 ? 0 : i), LaneZ(kv.Key)), kv.Value[i].psi);

        // Unload: reversing leaves the active lane on the floor.
        if (step < -1e-5) _reverse += -step * 1000f;
        else if (step > 1e-5) _reverse = 0f;
        if (_reverse >= ReleaseMm && _lanes[_slot].Count > 0)
        {
            foreach (var (d, _) in _lanes[_slot])
            {
                d.SetCarried(false);
                _released.Add(d);
                if (!_placed.Contains(d)) _placed.Add(d);
            }
            GD.Print($"[{SimTime,6:0.00}] [sim] couvání: vyjely kostky ze zásobníku {_slot}: " + string.Join(" ", _lanes[_slot].Select(x => "#" + x.d.Id)));
            _lanes[_slot].Clear();
            _reverse = 0f; // další slot vyjede až při dalším couvání
        }
        for (int i = _released.Count - 1; i >= 0; i--)
            if (_released[i].Pos2.DistanceTo(pos) > 0.30f)
            {
                Robot.RemoveCollisionExceptionWith(_released[i]);
                _released.RemoveAt(i);
            }
    }

    void TryCapture(Vector2 pos, float th)
    {
        var lateral = Geo.Right(th);
        foreach (var d in Dominos)
        {
            if (d.Carried || !d.Upright || _placed.Contains(d)) continue;
            var local = Geo.Unrotate(d.Pos2 - pos, th);
            if (local.X > PickX || local.X < PickX - 0.05f) continue;
            if (Mathf.Abs(local.Y) > 0.035f) continue;
            if (Mathf.Abs(Geo.Dir(d.Axis).Dot(lateral)) < 0.85f) continue; // must lie crosswise
            bool leftEndFront = (_slot == "RIGHT" && _rot == "CCW") || (_slot == "LEFT" && _rot == "CW");
            _cap = d;
            d.SetCarried(true);
            Robot.AddCollisionExceptionWith(d);
            _capStart = local;
            _capPsi0 = Geo.Wrap(d.Axis - th);
            _capSign = leftEndFront ? 1f : -1f; // +θ = clockwise from above
            _capLane = _slot;
            _capTravel0 = _travel;
            _capNeeded = Mathf.Max(local.X - LaneX(Math.Min(_lanes[_slot].Count, 1)), 0.03f);
            GD.Print($"[{SimTime,6:0.00}] [sim] nabírám kostku #{d.Id} ({_rot}, slot {_slot})");
            return;
        }
    }

    void SetPiece(Domino d, Vector2 local, float psi)
    {
        float th = Robot.Theta;
        d.GlobalTransform = new Transform3D(Geo.YawBasis(th + psi), Geo.V3(Robot.LocalToField(local), Domino.H / 2f));
    }

    // ------------------------------------------------------------------ HUD

    public string CarriedText()
    {
        string Lane(string k) => k + ":" + (_lanes[k].Count == 0 ? "-" : string.Join(",", _lanes[k].Select(x => $"#{x.d.Id}")));
        return $"ruce {_rot}, slot {_slot}   {Lane("LEFT")}  {Lane("RIGHT")}" + (_cap != null ? $"   nabírá #{_cap.Id}" : "");
    }

    public int PlacedCount => _placed.Count;

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
