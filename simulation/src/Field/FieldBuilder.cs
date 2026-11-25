using System;
using System.Collections.Generic;
using Godot;

namespace DominoSim;

/// <summary>Builds one half of the playing field (floor, walls, printed lines) and spawns the pieces.</summary>
public partial class FieldBuilder : Node3D
{
    public List<Domino> Build(FieldLayout L, bool round2Rotate, int seed)
    {
        Name = "Field";
        float W = L.Width, D = L.Depth;

        // Floor.
        var floor = new StaticBody3D { Name = "Floor", PhysicsMaterialOverride = new PhysicsMaterial { Friction = 0.6f } };
        floor.AddChild(new CollisionShape3D
        {
            Shape = new BoxShape3D { Size = new Vector3(W + 0.4f, 0.04f, D + 0.4f) },
            Position = new Vector3(W / 2f, -0.02f, D / 2f),
        });
        floor.AddChild(new MeshInstance3D
        {
            Mesh = new BoxMesh { Size = new Vector3(W, 0.01f, D), Material = Mat(new Color(0.96f, 0.96f, 0.95f)) },
            Position = new Vector3(W / 2f, -0.005f, D / 2f),
        });
        AddChild(floor);

        // Walls (outside the interior).
        const float t = 0.02f;
        var wallMat = Mat(new Color(0.55f, 0.55f, 0.58f));
        AddWall(new Vector3(W / 2f, 0, -t / 2f), new Vector3(W + 2 * t, L.WallHeight, t), wallMat);
        AddWall(new Vector3(W / 2f, 0, D + t / 2f), new Vector3(W + 2 * t, L.WallHeight, t), wallMat);
        AddWall(new Vector3(-t / 2f, 0, D / 2f), new Vector3(t, L.WallHeight, D), wallMat);
        AddWall(new Vector3(W + t / 2f, 0, D / 2f), new Vector3(t, L.WallHeight, D), wallMat);

        // Black lines.
        var black = Mat(new Color(0.05f, 0.05f, 0.05f));
        float lw = L.LineWidth;
        var r = L.BlackRect;
        AddRectOutline(r, lw, black, 0.0004f);
        AddLine(new Vector2(L.BlackCrossX, r.Position.Y), new Vector2(L.BlackCrossX, r.End.Y), lw, black, 0.0004f);
        AddLine(new Vector2(r.Position.X, L.BlackCrossZ), new Vector2(r.End.X, L.BlackCrossZ), lw, black, 0.0004f);

        // Stubs: from the piece end nearest to the black rectangle, StubLength long.
        var rc = r.GetCenter();
        foreach (var p in L.Pieces)
        {
            var ax = Geo.Dir(p.Axis);
            var e0 = p.Center - ax * Domino.L / 2f;
            var e1 = p.Center + ax * Domino.L / 2f;
            var near = e0.DistanceTo(rc) < e1.DistanceTo(rc) ? e0 : e1;
            var dir = (near - p.Center).Normalized();
            AddLine(near, near + dir * L.StubLength, lw, black, 0.0004f);
        }

        // Yellow scoring rectangle.
        AddRectOutline(L.YellowRect, L.YellowLineWidth, Mat(new Color(1f, 0.9f, 0.05f)), 0.0007f);

        // Pieces.
        var rng = new Random(seed);
        var list = new List<Domino>();
        foreach (var p in L.Pieces)
        {
            var d = new Domino();
            d.Setup(p.Id, p.C0, p.C1);
            AddChild(d);
            float axis = p.Axis;
            if (round2Rotate && rng.NextDouble() < 0.5) axis += Mathf.Pi;
            d.PlaceAt(p.Center, axis);
            list.Add(d);
        }
        return list;
    }

    static StandardMaterial3D Mat(Color c) => new() { AlbedoColor = c, Roughness = 0.9f };

    void AddWall(Vector3 basePos, Vector3 size, Material m)
    {
        var b = new StaticBody3D { Name = "Wall", Position = basePos + new Vector3(0, size.Y / 2f, 0) };
        b.AddChild(new CollisionShape3D { Shape = new BoxShape3D { Size = size } });
        b.AddChild(new MeshInstance3D { Mesh = new BoxMesh { Size = size, Material = m } });
        AddChild(b);
    }

    void AddRectOutline(Rect2 r, float w, Material m, float y)
    {
        var a = r.Position;
        var b = new Vector2(r.End.X, r.Position.Y);
        var c = r.End;
        var d = new Vector2(r.Position.X, r.End.Y);
        AddLine(a - new Vector2(w / 2, 0), b + new Vector2(w / 2, 0), w, m, y);
        AddLine(d - new Vector2(w / 2, 0), c + new Vector2(w / 2, 0), w, m, y);
        AddLine(a, d, w, m, y);
        AddLine(b, c, w, m, y);
    }

    void AddLine(Vector2 a, Vector2 b, float w, Material m, float y)
    {
        float len = a.DistanceTo(b);
        var mid = (a + b) / 2f;
        AddChild(new MeshInstance3D
        {
            Mesh = new BoxMesh { Size = new Vector3(len, 0.0006f, w), Material = m },
            Transform = new Transform3D(Geo.YawBasis(Geo.Angle(b - a)), Geo.V3(mid, y)),
        });
    }
}
