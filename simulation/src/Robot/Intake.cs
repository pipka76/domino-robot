using System;
using System.Collections.Generic;
using System.Linq;
using Godot;

namespace DominoSim;

/// <summary>
/// Front arms + belly storage. Scripted ("hybrid") mechanism:
///  * With one arm lowered, a crosswise piece whose centre reaches the arm line is captured.
///    Left arm holds the left end -> piece turns clockwise 90° (left end ends up in front);
///    right arm -> counter-clockwise (right end in front).
///  * The rotation is driven by the robot's forward travel, then the piece slides to the
///    rear slot (first piece, against the rear stop) or the front slot (second piece).
///  * Release() leaves the stored pieces on the floor; the robot then reverses away.
/// </summary>
public partial class Intake : Node3D
{
    public enum ArmMode { Retracted, Left, Right }

    public static float ArmLineX => Robot.BodyFront - 0.01f;
    public static float FrontSlotX => Robot.BodyFront - Domino.L / 2f;
    public static float RearSlotX => FrontSlotX - Domino.L;
    public const int Capacity = 2;

    public ArmMode Arm { get; private set; } = ArmMode.Retracted;
    public readonly List<Domino> Stored = new();               // [0] = rear slot, [1] = front slot
    readonly List<float> _storedPsi = new();                     // piece axis relative to robot heading
    public Func<IEnumerable<Domino>> Candidates = () => Enumerable.Empty<Domino>();
    public event Action<Domino, ArmMode> CaptureStarted;

    // Capture in progress.
    public Domino Capturing { get; private set; }
    Vector2 _capStart;
    float _capPsi0, _capSign, _capTravel0, _capNeeded, _capTargetX;

    public float Travel { get; private set; }
    Vector2 _lastPos;
    bool _hasLast;

    readonly List<Domino> _released = new();
    Robot _robot;
    MeshInstance3D _armL, _armR;

    public bool Busy => Capturing != null;
    public int Count => Stored.Count + (Capturing != null ? 1 : 0);

    public override void _Ready()
    {
        _robot = GetParent<Robot>();
        var mat = new StandardMaterial3D { AlbedoColor = new Color(0.95f, 0.55f, 0.1f) };
        _armL = new MeshInstance3D { Mesh = new BoxMesh { Size = new Vector3(0.012f, 0.03f, 0.03f), Material = mat } };
        _armR = new MeshInstance3D { Mesh = new BoxMesh { Size = new Vector3(0.012f, 0.03f, 0.03f), Material = mat } };
        AddChild(_armL);
        AddChild(_armR);
        UpdateArmVisuals();
    }

    public void SetArm(ArmMode m)
    {
        Arm = m;
        UpdateArmVisuals();
    }

    void UpdateArmVisuals()
    {
        // Lowered arm sits in the lane at the front; raised arm is lifted and swung outward.
        _armL.Position = Arm == ArmMode.Left ? new Vector3(ArmLineX + 0.015f, 0.018f, -0.04f) : new Vector3(Robot.BodyFront + 0.006f, 0.06f, -0.055f);
        _armR.Position = Arm == ArmMode.Right ? new Vector3(ArmLineX + 0.015f, 0.018f, 0.04f) : new Vector3(Robot.BodyFront + 0.006f, 0.06f, 0.055f);
    }

    public override void _PhysicsProcess(double delta)
    {
        var pos = _robot.Pos2;
        float th = _robot.Theta;
        if (_hasLast) Travel += (pos - _lastPos).Dot(Geo.Dir(th));
        _lastPos = pos;
        _hasLast = true;

        if (Capturing != null)
        {
            float t = Mathf.Clamp((Travel - _capTravel0) / _capNeeded, 0f, 1f);
            var local = new Vector2(Mathf.Lerp(_capStart.X, _capTargetX, t), Mathf.Lerp(_capStart.Y, 0f, t));
            float psi = _capPsi0 + _capSign * Mathf.Pi / 2f * t;
            if (t >= 1f)
            {
                // Snap to exactly lengthwise.
                psi = Mathf.Abs(Geo.Wrap(psi)) < Mathf.Pi / 2f ? 0f : Mathf.Pi;
                Stored.Add(Capturing);
                _storedPsi.Add(psi);
                Capturing = null;
            }
            else
            {
                SetPiece(Capturing, local, psi);
            }
        }
        else if (Arm != ArmMode.Retracted && Stored.Count < Capacity)
        {
            TryStartCapture(pos, th);
        }

        for (int i = 0; i < Stored.Count; i++)
            SetPiece(Stored[i], new Vector2(i == 0 ? RearSlotX : FrontSlotX, 0f), _storedPsi[i]);

        // Re-enable robot <-> piece collisions once released pieces are left behind.
        for (int i = _released.Count - 1; i >= 0; i--)
        {
            if (_released[i].Pos2.DistanceTo(pos) > 0.26f)
            {
                _robot.RemoveCollisionExceptionWith(_released[i]);
                _released.RemoveAt(i);
            }
        }
    }

    void TryStartCapture(Vector2 pos, float th)
    {
        var lateral = Geo.Right(th);
        foreach (var d in Candidates())
        {
            if (d.Carried || !d.Upright) continue;
            var local = Geo.Unrotate(d.Pos2 - pos, th);
            if (local.X > ArmLineX || local.X < ArmLineX - 0.05f) continue;
            if (Mathf.Abs(local.Y) > 0.035f) continue;
            if (Mathf.Abs(Geo.Dir(d.Axis).Dot(lateral)) < 0.85f) continue; // must lie crosswise

            Capturing = d;
            d.SetCarried(true);
            _robot.AddCollisionExceptionWith(d);
            _capStart = local;
            _capPsi0 = Geo.Wrap(d.Axis - th);
            _capSign = Arm == ArmMode.Left ? 1f : -1f; // +θ = clockwise from above
            _capTargetX = Stored.Count == 0 ? RearSlotX : FrontSlotX;
            _capTravel0 = Travel;
            _capNeeded = Mathf.Max(local.X - _capTargetX, 0.03f);
            CaptureStarted?.Invoke(d, Arm);
            return;
        }
    }

    void SetPiece(Domino d, Vector2 local, float psi)
    {
        float th = _robot.Theta;
        d.GlobalTransform = new Transform3D(Geo.YawBasis(th + psi), Geo.V3(_robot.LocalToField(local), Domino.H / 2f));
    }

    /// <summary>Leave all stored pieces where they are (robot reverses afterwards).</summary>
    public List<Domino> Release()
    {
        var list = new List<Domino>(Stored);
        foreach (var d in Stored)
        {
            d.SetCarried(false);
            _released.Add(d);
        }
        Stored.Clear();
        _storedPsi.Clear();
        return list;
    }
}
