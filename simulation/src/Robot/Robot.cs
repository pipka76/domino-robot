using Godot;

namespace DominoSim;

/// <summary>
/// Two-wheel differential robot. Local +X = forward, +Z = right, origin at floor level
/// on the wheel axle centre. The chassis rides on two frictionless rear casters and two
/// frictionless wheel colliders; traction is applied explicitly at the wheel contact points
/// from a simple motor + friction model, so wheel slip and pushing are physical.
/// The space under the body (|z| &lt; BodyHalfWidth, y &lt; BodyBottom) is the storage lane.
/// </summary>
public partial class Robot : RigidBody3D
{
    // Geometry comes from RobotSpec (data/robot.json).
    static float WheelRadius => RobotSpec.WheelRadius;
    static float WheelWidth => RobotSpec.WheelWidth;
    public static float HalfTrack => RobotSpec.HalfTrack;
    public static float BodyFront => RobotSpec.BodyFront;
    static float BodyRear => RobotSpec.BodyRear;
    static float BodyHalfWidth => RobotSpec.BodyHalfWidth;
    static float BodyBottom => RobotSpec.BodyBottom;
    static float BodyTop => RobotSpec.BodyTop;
    static float CasterRadius => RobotSpec.CasterRadius;
    static float CasterX => RobotSpec.CasterX;
    static float CasterZ => RobotSpec.CasterZ;

    // Drive model.
    public const float MaxWheelSpeed = 0.45f;   // m/s at the tyre (≈ EV3 large motor)
    public const float MotorAccel = 2.0f;       // m/s² slew of the commanded tyre speed
    public const float Mu = 0.9f;               // tyre-floor friction
    public const float WheelLoad = 3.6f;        // N normal load per driven wheel
    const float SlipGain = 30f;                 // N per (m/s) of slip

    float _tgtL, _tgtR, _vL, _vR;
    public double EncoderL, EncoderR;           // wheel angle, rad

    public Intake Intake { get; private set; }

    public Vector2 Pos2 => Geo.XZ(GlobalPosition);
    public float Theta => Geo.HeadingOf(GlobalBasis);
    public float YawRate => -AngularVelocity.Y;
    public float Speed => LinearVelocity.Dot(GlobalBasis.X);
    public Vector2 LocalToField(Vector2 local) => Pos2 + Geo.Rotate(local, Theta);

    public override void _Ready()
    {
        Name = "Robot";
        Mass = 1.0f;
        CanSleep = false;
        CenterOfMassMode = CenterOfMassModeEnum.Custom;
        CenterOfMass = new Vector3(Mathf.Max(BodyRear * 0.25f, CasterX * 0.3f), 0.05f, 0f);
        PhysicsMaterialOverride = new PhysicsMaterial { Friction = 0f, Bounce = 0f };
        AngularDamp = 0.5f;

        // Semi-transparent so the stored pieces stay visible.
        var bodyMat = new StandardMaterial3D { AlbedoColor = new Color(0.25f, 0.32f, 0.45f, 0.5f), Roughness = 0.8f, Transparency = BaseMaterial3D.TransparencyEnum.Alpha };
        var darkMat = new StandardMaterial3D { AlbedoColor = new Color(0.12f, 0.12f, 0.12f) };
        var accentMat = new StandardMaterial3D { AlbedoColor = new Color(0.95f, 0.55f, 0.1f) };

        // Chassis above the storage lane.
        var bodySize = new Vector3(BodyFront - BodyRear, BodyTop - BodyBottom, 2 * BodyHalfWidth);
        var bodyPos = new Vector3((BodyFront + BodyRear) / 2f, (BodyTop + BodyBottom) / 2f, 0);
        // Rounded-rectangle outline: two crossing boxes + a cylinder in each corner.
        float r = Mathf.Min(RobotSpec.CornerRadius, Mathf.Min(bodySize.X, bodySize.Z) / 2f - 0.001f);
        if (r < 0.002f)
        {
            AddChild(new CollisionShape3D { Shape = new BoxShape3D { Size = bodySize }, Position = bodyPos });
            AddChild(new MeshInstance3D { Mesh = new BoxMesh { Size = bodySize, Material = bodyMat }, Position = bodyPos });
        }
        else
        {
            foreach (var sz in new[] { new Vector3(bodySize.X - 2 * r, bodySize.Y, bodySize.Z), new Vector3(bodySize.X, bodySize.Y, bodySize.Z - 2 * r) })
            {
                AddChild(new CollisionShape3D { Shape = new BoxShape3D { Size = sz }, Position = bodyPos });
                AddChild(new MeshInstance3D { Mesh = new BoxMesh { Size = sz, Material = bodyMat }, Position = bodyPos });
            }
            foreach (int sx in new[] { -1, 1 })
                foreach (int sz in new[] { -1, 1 })
                {
                    var cp = bodyPos + new Vector3(sx * (bodySize.X / 2f - r), 0, sz * (bodySize.Z / 2f - r));
                    AddChild(new CollisionShape3D { Shape = new CylinderShape3D { Radius = r, Height = bodySize.Y }, Position = cp });
                    AddChild(new MeshInstance3D { Mesh = new CylinderMesh { TopRadius = r, BottomRadius = r, Height = bodySize.Y, Material = bodyMat }, Position = cp });
                }
        }
        // Heading marker.
        AddChild(new MeshInstance3D
        {
            Mesh = new BoxMesh { Size = new Vector3(0.06f, 0.004f, 0.012f), Material = accentMat },
            Position = new Vector3(BodyFront - 0.04f, BodyTop + 0.002f, 0),
        });

        // Wheels.
        foreach (int s in new[] { -1, 1 })
        {
            var pos = new Vector3(0, WheelRadius, s * HalfTrack);
            var rot = new Basis(Vector3.Right, Mathf.Pi / 2f);
            AddChild(new CollisionShape3D
            {
                Shape = new CylinderShape3D { Radius = WheelRadius, Height = WheelWidth },
                Transform = new Transform3D(rot, pos),
            });
            AddChild(new MeshInstance3D
            {
                Mesh = new CylinderMesh { TopRadius = WheelRadius, BottomRadius = WheelRadius, Height = WheelWidth, Material = darkMat },
                Transform = new Transform3D(rot, pos),
            });
            // Side pod connecting wheel to body (only for a body narrower than the wheel track).
            if (HalfTrack - WheelWidth / 2f > BodyHalfWidth + 0.002f)
            AddChild(new MeshInstance3D
            {
                Mesh = new BoxMesh { Size = new Vector3(0.05f, 0.012f, HalfTrack - BodyHalfWidth - WheelWidth / 2f), Material = bodyMat },
                Position = new Vector3(0, WheelRadius + 0.01f, s * (BodyHalfWidth + (HalfTrack - BodyHalfWidth - WheelWidth / 2f) / 2f)),
            });
            // Rear caster.
            var cpos = new Vector3(CasterX, CasterRadius, s * CasterZ);
            AddChild(new CollisionShape3D { Shape = new SphereShape3D { Radius = CasterRadius }, Position = cpos });
            AddChild(new MeshInstance3D { Mesh = new SphereMesh { Radius = CasterRadius, Height = 2 * CasterRadius, Material = darkMat }, Position = cpos });
        }

        Intake = new Intake();
        AddChild(Intake);
    }

    public void SetWheelSpeeds(float left, float right)
    {
        float m = Mathf.Max(Mathf.Abs(left), Mathf.Abs(right));
        if (m > MaxWheelSpeed) { left *= MaxWheelSpeed / m; right *= MaxWheelSpeed / m; }
        _tgtL = left;
        _tgtR = right;
    }

    /// <summary>v forward speed (m/s); w yaw rate (rad/s), positive = turn right (clockwise from above).</summary>
    public void SetTwist(float v, float w) => SetWheelSpeeds(v + w * HalfTrack, v - w * HalfTrack);

    public void Stop() => SetWheelSpeeds(0, 0);

    public override void _IntegrateForces(PhysicsDirectBodyState3D state)
    {
        float dt = (float)state.Step;
        float dv = MotorAccel * dt;
        _vL = Mathf.MoveToward(_vL, _tgtL, dv);
        _vR = Mathf.MoveToward(_vR, _tgtR, dv);
        EncoderL += _vL / WheelRadius * dt;
        EncoderR += _vR / WheelRadius * dt;

        var t = state.Transform;
        if (t.Basis.Y.Y < 0.9f) return; // tipped over: no traction
        var fwd = t.Basis.X.Normalized();
        var lat = t.Basis.Z.Normalized();
        var com = t.Origin + state.CenterOfMass;
        float fmax = Mu * WheelLoad;

        foreach (int s in new[] { -1, 1 })
        {
            var pw = t * new Vector3(0, 0, s * HalfTrack);
            if (pw.Y > 0.006f) continue; // wheel not on the floor
            var v = state.LinearVelocity + state.AngularVelocity.Cross(pw - com);
            float vl = v.Dot(fwd), vt = v.Dot(lat);
            float vw = s < 0 ? _vL : _vR;
            var f = fwd * (SlipGain * (vw - vl)) + lat * (-SlipGain * vt);
            if (f.Length() > fmax) f = f.Normalized() * fmax;
            state.ApplyForce(f, pw - t.Origin);
        }
    }
}
