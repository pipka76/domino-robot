using System;
using System.Collections.Generic;
using System.Linq;
using Godot;

namespace DominoSim;

/// <summary>
/// Our robot's pick-up mechanism (motor C), robot frame: x forward from the wheel axle, z right; metres.
///
///  * HANDS at the very front (pick line x = PickX), one left, one right, 90° out of phase: rot CW or CCW
///    lowers one of them, NONE = both up. A crosswise piece reaching the pick line is caught and turned by
///    90° while the robot drives on (slot RIGHT + CCW or slot LEFT + CW = the left end, seen by the sensor, goes in front).
///  * TURNING ZONE between the hands and the neck (what is left of the length: 365 mm − 3 pieces).
///  * NECK behind it: room for 2 pieces in a row. The pieces in the neck stand on the floor – when the
///    robot drives forward they stay put, i.e. they move backwards relative to the robot, until they hit the
///    piece behind or the rear wall of the active magazine slot.
///  * MAGAZINE behind the neck, at the very rear of the robot: 2 slots (LEFT / RIGHT) side by side, one piece each. The active slot lines up
///    with the neck; a shift moves the magazine sideways (a piece in the slot moves with it).
///    So at most 4 pieces: 2 in the neck + 1 in each slot. Order of a row: 1 → slot R, 2 → slot L, 3 + 4 neck.
///  * Reversing: the pieces in the line (neck + active slot) stay on the floor and leave through the front;
///    the piece in the other slot stays in its pocket until a shift. Driving forward onto a piece lying
///    lengthwise in the lane takes it back into the neck (it is pushed along).
/// </summary>
public partial class OurMechanism : Node3D
{
    // Geometry from data/robot_ours.json: the body runs from the magazine end (rear) to the hands (front).
    public static float PickX => RobotSpec.BodyFront;                        // front edge = hands
    public static float SlotRearX => RobotSpec.BodyRear + 0.004f;            // rear wall of a slot = end of the robot
    public static float NeckRearX => SlotRearX + Domino.L;                   // end of the neck = start of the magazine
    public static float NeckFrontX => NeckRearX + 2 * Domino.L;              // neck entry; between it and the hands the piece turns
    public const float SlotPitch = 0.040f;                                   // sideways distance of the two slots
    public static float SlotX => (NeckRearX + SlotRearX) / 2f;
    /// <summary>Colour sensor: at the LEFT hand (sensorAhead in robot_ours.json), sees the left half of a crosswise piece.</summary>
    public static Vector2 SensorLocal => new(PickX + RobotSpec.SensorAhead, -0.024f);

    public Func<IEnumerable<Domino>> Candidates = () => Enumerable.Empty<Domino>();
    public Action<string> Log = _ => { };

    public string Rot { get; private set; } = "CW";
    public string Slot { get; private set; } = "RIGHT";

    /// <summary>Pieces in the line (neck + active slot): relative x of the centre, heading relative to the robot.</summary>
    readonly List<(Domino d, float x, float psi)> _line = new();
    /// <summary>Piece held in each slot while that slot is not active.</summary>
    readonly Dictionary<string, (Domino d, float psi)?> _parked = new() { ["LEFT"] = null, ["RIGHT"] = null };

    Domino _cap;
    Vector2 _capStart;
    float _capPsi0, _capSign, _capTurn, _capTravel0, _capNeeded, _travel;
    Vector2 _lastPos;
    bool _hasLast;
    readonly List<Domino> _released = new();
    public readonly List<Domino> Placed = new();

    Robot _robot;
    MeshInstance3D _handL, _handR;
    Node3D _magazine;

    public int Count => _line.Count + _parked.Values.Count(p => p != null) + (_cap != null ? 1 : 0);
    public bool Busy => _cap != null;

    public override void _Ready()
    {
        _robot = GetParent<Robot>();
        Name = "OurMechanism";
        var orange = new StandardMaterial3D { AlbedoColor = new Color(0.95f, 0.55f, 0.1f) };
        var wall = new StandardMaterial3D { AlbedoColor = new Color(0.15f, 0.15f, 0.18f) };
        float wy = 0.012f, hw = Domino.W / 2f + 0.003f;
        MeshInstance3D Box(Node3D parent, Vector3 size, Vector3 pos, Material m)
        {
            var mi = new MeshInstance3D { Mesh = new BoxMesh { Size = size, Material = m }, Position = pos };
            parent.AddChild(mi);
            return mi;
        }
        // Neck: two side walls.
        foreach (int s in new[] { -1, 1 })
            Box(this, new Vector3(NeckFrontX - NeckRearX, 0.024f, 0.003f), new Vector3((NeckFrontX + NeckRearX) / 2f, wy + 0.022f, s * hw), wall);
        // Magazine: two slots (3 walls + rear wall), moved sideways by Slot.
        _magazine = new Node3D();
        AddChild(_magazine);
        foreach (float z in new[] { -SlotPitch / 2f - hw, -SlotPitch / 2f + hw, SlotPitch / 2f - hw, SlotPitch / 2f + hw })
            Box(_magazine, new Vector3(Domino.L, 0.024f, 0.003f), new Vector3(SlotX, wy + 0.022f, z), wall);
        Box(_magazine, new Vector3(0.004f, 0.024f, SlotPitch + 2 * hw), new Vector3(SlotRearX - 0.002f, wy + 0.022f, 0), wall);
        // Colour sensor at the left hand (black body, white lens pointing down).
        var sensorPos = new Vector3(SensorLocal.X, 0.032f, SensorLocal.Y);
        Box(this, new Vector3(0.024f, 0.016f, 0.016f), sensorPos, new StandardMaterial3D { AlbedoColor = new Color(0.08f, 0.08f, 0.1f) });
        Box(this, new Vector3(0.008f, 0.002f, 0.008f), sensorPos + new Vector3(0, 0.009f, 0), new StandardMaterial3D { AlbedoColor = new Color(0.9f, 0.9f, 1f), EmissionEnabled = true, Emission = new Color(0.6f, 0.6f, 1f) });
        // Hands.
        _handL = Box(this, new Vector3(0.012f, 0.03f, 0.03f), Vector3.Zero, orange);
        _handR = Box(this, new Vector3(0.012f, 0.03f, 0.03f), Vector3.Zero, orange);
        UpdateVisuals();
    }

    // Which end of a caught piece ends up in front: slot RIGHT + CCW or slot LEFT + CW = the LEFT end, i.e. the half
    // seen by the colour sensor (as verified on the robot, mission.json pick.seen_to_front).
    bool LeftEndFront => (Slot == "RIGHT" && Rot == "CCW") || (Slot == "LEFT" && Rot == "CW");

    void UpdateVisuals()
    {
        if (_handL == null) return;
        bool down = Rot != "NONE";
        bool leftDown = down && LeftEndFront, rightDown = down && !LeftEndFront;
        _handL.Position = leftDown ? new Vector3(PickX, 0.018f, -0.03f) : new Vector3(PickX, 0.07f, -0.06f);
        _handR.Position = rightDown ? new Vector3(PickX, 0.018f, 0.03f) : new Vector3(PickX, 0.07f, 0.06f);
        // Active slot lines up with the neck: RIGHT active → magazine shifted left.
        _magazine.Position = new Vector3(0, 0, Slot == "RIGHT" ? -SlotPitch / 2f : SlotPitch / 2f);
    }

    float SlotZ(string slot) => slot == Slot ? 0f : (slot == "LEFT" ? -SlotPitch : SlotPitch);

    public void SetState(string rot, string slot)
    {
        rot = rot?.ToUpperInvariant() ?? Rot;
        slot = slot?.ToUpperInvariant() ?? Slot;
        if (slot == "TOGGLE") slot = Slot == "LEFT" ? "RIGHT" : "LEFT";
        if (slot != Slot)
        {
            // The piece sitting in the active slot moves away with the magazine...
            int i = _line.FindIndex(p => p.x < NeckRearX);
            if (i >= 0)
            {
                _parked[Slot] = (_line[i].d, _line[i].psi);
                _line.RemoveAt(i);
            }
            // ...and the new slot's piece lines up behind the neck.
            if (_parked[slot] is { } q)
            {
                _line.Add((q.d, SlotX, q.psi));
                _parked[slot] = null;
            }
            Log($"[sim] posun zásobníku {Slot} -> {slot}" + (_cap != null ? " (kostka se ještě nabírá)" : ""));
        }
        Rot = rot;
        Slot = slot;
        UpdateVisuals();
    }

    public override void _PhysicsProcess(double delta)
    {
        var pos = _robot.Pos2;
        float th = _robot.Theta;
        float step = _hasLast ? (pos - _lastPos).Dot(Geo.Dir(th)) : 0f;
        _lastPos = pos;
        _hasLast = true;
        _travel += step;

        // Pieces in the line stand on the floor: relative to the robot they move against its travel.
        for (int i = 0; i < _line.Count; i++) _line[i] = (_line[i].d, _line[i].x - step, _line[i].psi);
        if (step > 0) TryIngest(pos, th);

        if (_cap != null)
        {
            float t = Mathf.Clamp((_travel - _capTravel0) / _capNeeded, 0f, 1f);
            var local = new Vector2(Mathf.Lerp(_capStart.X, NeckFrontX - Domino.L / 2f, t), Mathf.Lerp(_capStart.Y, 0f, t));
            float psi = _capPsi0 + _capSign * _capTurn * t;
            if (t >= 1f)
            {
                psi = Mathf.Abs(Geo.Wrap(psi)) < Mathf.Pi / 2f ? 0f : Mathf.Pi;
                _line.Add((_cap, local.X, psi));
                Log($"[sim] kostka #{_cap.Id} otočená v krku");
                _cap = null;
            }
            else SetPiece(_cap, local, psi);
        }
        else if (Rot != "NONE" && _line.Count < 3)
        {
            TryCapture(pos, th);
        }

        // Rear wall of the active slot and the pieces behind push the line forward.
        _line.Sort((a, b) => a.x.CompareTo(b.x));
        float min = SlotRearX + Domino.L / 2f;
        for (int i = 0; i < _line.Count; i++)
        {
            float x = Mathf.Max(_line[i].x, min);
            _line[i] = (_line[i].d, x, _line[i].psi);
            min = x + Domino.L;
        }
        // Reversing: a piece fully out of the front is left on the floor.
        for (int i = _line.Count - 1; i >= 0; i--)
        {
            if (_line[i].x - Domino.L / 2f <= PickX + 0.002f) continue;
            var d = _line[i].d;
            SetPiece(d, new Vector2(_line[i].x, 0f), _line[i].psi);
            d.SetCarried(false);
            _released.Add(d);
            if (!Placed.Contains(d)) Placed.Add(d);
            Log($"[sim] kostka #{d.Id} vyjela z krku");
            _line.RemoveAt(i);
        }
        foreach (var p in _line) SetPiece(p.d, new Vector2(p.x, 0f), p.psi);
        foreach (var kv in _parked)
            if (kv.Value is { } q) SetPiece(q.d, new Vector2(SlotX, SlotZ(kv.Key)), q.psi);

        for (int i = _released.Count - 1; i >= 0; i--)
            if (_released[i].Pos2.DistanceTo(pos) > 0.40f)
            {
                _robot.RemoveCollisionExceptionWith(_released[i]);
                _released.RemoveAt(i);
            }
    }

    bool Free(Domino d) => !d.Carried && d.Upright && d != _cap;

    /// <summary>
    /// A piece that gets under the front (between the hands and the neck entry) while a hand is down is caught and
    /// turned in the hand's direction until it lies lengthwise: 90° for a crosswise piece on a straight run, less
    /// for a piece the robot drives over at an angle (e.g. the first piece of a row taken by an arc).
    /// </summary>
    void TryCapture(Vector2 pos, float th)
    {
        foreach (var d in Candidates())
        {
            if (!Free(d)) continue;
            var local = Geo.Unrotate(d.Pos2 - pos, th);
            if (local.X > PickX || local.X < NeckFrontX - 0.02f) continue;
            if (Mathf.Abs(local.Y) > 0.045f) continue;
            float psi0 = Geo.Wrap(d.Axis - th);
            float sign = LeftEndFront ? 1f : -1f; // +θ = clockwise from above
            // Turn in the hand's direction until lengthwise (psi = 0 or π).
            float turn = Mathf.PosMod(-sign * psi0, Mathf.Pi);
            if (turn > Mathf.Pi - 0.05f) turn = 0f;
            if (turn < 0.05f && Mathf.Abs(local.Y) < 0.012f) continue; // lengthwise in the lane: TryIngest takes it
            _cap = d;
            d.SetCarried(true);
            _robot.AddCollisionExceptionWith(d);
            _capStart = local;
            _capPsi0 = psi0;
            _capSign = sign;
            _capTurn = turn;
            _capTravel0 = _travel;
            _capNeeded = Mathf.Max(local.X - (NeckFrontX - Domino.L / 2f), 0.03f);
            Log($"[sim] nabírám kostku #{d.Id} ({Rot}, slot {Slot}, otočí o {Mathf.RadToDeg(turn):0}°)");
            return;
        }
    }

    /// <summary>Driving forward onto a piece lying lengthwise in the lane takes it into the neck.</summary>
    void TryIngest(Vector2 pos, float th)
    {
        var fwd = Geo.Dir(th);
        foreach (var d in Candidates())
        {
            if (!Free(d)) continue;
            var local = Geo.Unrotate(d.Pos2 - pos, th);
            if (Mathf.Abs(local.Y) > 0.012f) continue;
            if (Mathf.Abs(Geo.Dir(d.Axis).Dot(fwd)) < 0.97f) continue; // must lie lengthwise
            if (local.X - Domino.L / 2f > PickX || local.X < SlotRearX) continue;
            d.SetCarried(true);
            _robot.AddCollisionExceptionWith(d);
            float psi = Mathf.Abs(Geo.Wrap(d.Axis - th)) < Mathf.Pi / 2f ? 0f : Mathf.Pi;
            _line.Add((d, local.X, psi));
            Placed.Remove(d);
            Log($"[sim] kostka #{d.Id} zpět v krku (najetí dopředu)");
        }
    }

    void SetPiece(Domino d, Vector2 local, float psi)
    {
        float th = _robot.Theta;
        d.GlobalTransform = new Transform3D(Geo.YawBasis(th + psi), Geo.V3(_robot.LocalToField(local), Domino.H / 2f));
    }

    /// <summary>One line for the HUD: hands, neck (2 places) and the two magazine slots (active one in brackets).</summary>
    public string ShortText()
    {
        int neck = _line.Count(p => p.x >= NeckRearX) + (_cap != null ? 1 : 0);
        bool activeFull = _line.Any(p => p.x < NeckRearX);
        string other = Slot == "LEFT" ? "RIGHT" : "LEFT";
        bool otherFull = _parked[other] != null;
        string Sq(bool b) => b ? "■" : "□";
        string Name(string s) => s == "LEFT" ? "L" : "P";
        string neckS = new string('■', Math.Min(neck, 2)) + new string('□', Math.Max(0, 2 - neck));
        string a = $"[{Name(Slot)}{Sq(activeFull)}]", b = $" {Name(other)}{Sq(otherFull)} ";
        string slots = Slot == "LEFT" ? a + b : b + a;
        return $"ruce {Rot,-4}  krk {neckS}  zásobník {slots.Trim()}";
    }

    public string Text()
    {
        string P(Domino d) => $"#{d.Id}[{PColors.Letter(d.C0)}{PColors.Letter(d.C1)}]";
        string slot = _line.Where(p => p.x < NeckRearX).Select(p => P(p.d)).FirstOrDefault() ?? "-";
        string neck = string.Join(" ", _line.Where(p => p.x >= NeckRearX).OrderBy(p => p.x).Select(p => P(p.d)));
        string other = Slot == "LEFT" ? "RIGHT" : "LEFT";
        string parked = _parked[other] is { } q ? P(q.d) : "-";
        return $"ruce {Rot}, krk [{(neck == "" ? "-" : neck)}], slot {Slot} (u krku) {slot}, slot {other} {parked}" + (_cap != null ? $", nabírá #{_cap.Id}" : "");
    }
}
