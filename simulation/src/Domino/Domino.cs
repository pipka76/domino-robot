using Godot;

namespace DominoSim;

/// <summary>
/// One competition piece: 96 x 32 x 21 mm, black body with two coloured top halves.
/// Local +X is the long axis; C0 is the colour of the -X half, C1 of the +X half.
/// </summary>
public partial class Domino : RigidBody3D
{
    public const float L = 0.096f, W = 0.032f, H = 0.021f;

    public int Id;
    public PColor C0, C1;
    public bool Carried { get; private set; }

    public Vector2 Pos2 => Geo.XZ(GlobalPosition);
    public float Axis => Geo.HeadingOf(GlobalBasis);
    public bool Upright => GlobalBasis.Y.Y > 0.8f;
    public Obb Footprint => new(Pos2, Axis, L / 2f, W / 2f);

    public void Setup(int id, PColor c0, PColor c1)
    {
        Id = id;
        C0 = c0;
        C1 = c1;
        Name = $"Domino{id}";
        Mass = 0.030f;
        CanSleep = true;
        ContinuousCd = true;
        PhysicsMaterialOverride = new PhysicsMaterial { Friction = 0.6f, Bounce = 0f };

        AddChild(new CollisionShape3D { Shape = new BoxShape3D { Size = new Vector3(L, H, W) } });

        var black = new StandardMaterial3D { AlbedoColor = PColors.ToColor(PColor.Black), Roughness = 0.6f };
        AddChild(new MeshInstance3D
        {
            Mesh = new BoxMesh { Size = new Vector3(L, H - 0.003f, W), Material = black },
            Position = new Vector3(0, -0.0015f, 0),
        });
        for (int i = 0; i < 2; i++)
        {
            var mat = new StandardMaterial3D { AlbedoColor = PColors.ToColor(i == 0 ? c0 : c1), Roughness = 0.95f, MetallicSpecular = 0.1f };
            AddChild(new MeshInstance3D
            {
                Mesh = new BoxMesh { Size = new Vector3(L / 2f - 0.001f, 0.003f, W), Material = mat },
                Position = new Vector3((i == 0 ? -1 : 1) * L / 4f, H / 2f - 0.0015f, 0),
            });
        }
    }

    public void PlaceAt(Vector2 center, float axis)
    {
        GlobalTransform = new Transform3D(Geo.YawBasis(axis), Geo.V3(center, H / 2f));
    }

    public void SetCarried(bool carried)
    {
        Carried = carried;
        if (carried)
        {
            FreezeMode = FreezeModeEnum.Kinematic;
            Freeze = true;
            // A carried piece is moved by the mechanism, not by physics: it must not shove pieces on the floor.
            CollisionLayer = 0;
            CollisionMask = 0;
        }
        else
        {
            CollisionLayer = 1;
            CollisionMask = 1;
            Freeze = false;
            LinearVelocity = Vector3.Zero;
            AngularVelocity = Vector3.Zero;
            Sleeping = false;
        }
    }

    /// <summary>Colour visible from above at a field point, or None if the point is not on this piece.</summary>
    public PColor ColorAt(Vector2 p)
    {
        var l = Geo.Unrotate(p - Pos2, Axis);
        if (Mathf.Abs(l.X) > L / 2f || Mathf.Abs(l.Y) > W / 2f) return PColor.None;
        if (!Upright) return PColor.Black;
        return l.X < 0 ? C0 : C1;
    }

    public PiecePose Pose => new(Pos2, Axis, Upright, C0, C1);
}

/// <summary>Lightweight piece description for geometry/colour evaluation.</summary>
public readonly record struct PiecePose(Vector2 C, float Axis, bool Upright, PColor C0, PColor C1)
{
    public Vector2 End(int i) => C + Geo.Dir(Axis) * (i == 0 ? -Domino.L / 2f : Domino.L / 2f);
    public PColor ColorAtEnd(int i) => !Upright ? PColor.Black : (i == 0 ? C0 : C1);
}
