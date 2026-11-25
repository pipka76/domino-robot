using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using Godot;

namespace DominoSim;

public sealed class PieceDef
{
    public int Id;
    public int Line;
    public int Index;
    public Vector2 Center;
    /// <summary>Heading of the piece's long axis (local +X). C0 is at the -X end.</summary>
    public float Axis;
    public PColor C0, C1;
}

public sealed class LineDef
{
    public string Name;
    /// <summary>Unit vector from the first piece to the last piece.</summary>
    public Vector2 Dir;
    public List<PieceDef> Pieces = new();
}

/// <summary>Field geometry in metres. Loaded from a JSON layout given in millimetres.</summary>
public sealed class FieldLayout
{
    public float Width, Depth, WallHeight, LineWidth, StubLength;
    public Rect2 BlackRect;
    public float BlackCrossX, BlackCrossZ;
    public Rect2 YellowRect;
    public float YellowLineWidth;
    public Vector2 RobotStart;
    public float RobotStartHeading;
    public List<LineDef> Lines = new();

    public IEnumerable<PieceDef> Pieces => Lines.SelectMany(l => l.Pieces);
    /// <summary>Area strictly inside the yellow line (pieces must not touch the line).</summary>
    public Rect2 YellowInner => YellowRect.Grow(-YellowLineWidth / 2f);

    const float Mm = 0.001f;

    public static FieldLayout Load(string resPath)
    {
        string text = FileAccess.GetFileAsString(resPath);
        if (string.IsNullOrEmpty(text)) throw new Exception($"Cannot read layout {resPath}");
        return Parse(text);
    }

    public static FieldLayout Parse(string json)
    {
        using var doc = JsonDocument.Parse(json);
        var root = doc.RootElement;
        var f = root.GetProperty("field");
        var L = new FieldLayout
        {
            Width = f.GetProperty("width").GetSingle() * Mm,
            Depth = f.GetProperty("depth").GetSingle() * Mm,
            WallHeight = f.GetProperty("wallHeight").GetSingle() * Mm,
            LineWidth = f.GetProperty("lineWidth").GetSingle() * Mm,
            StubLength = f.GetProperty("stubLength").GetSingle() * Mm,
            BlackRect = RectFrom(f.GetProperty("blackRect")),
            BlackCrossX = f.GetProperty("blackCrossX").GetSingle() * Mm,
            BlackCrossZ = f.GetProperty("blackCrossZ").GetSingle() * Mm,
            YellowRect = RectFrom(f.GetProperty("yellowRect")),
            YellowLineWidth = f.GetProperty("yellowLineWidth").GetSingle() * Mm,
        };
        var rs = root.GetProperty("robotStart");
        L.RobotStart = new Vector2(rs.GetProperty("x").GetSingle(), rs.GetProperty("z").GetSingle()) * Mm;
        L.RobotStartHeading = Mathf.DegToRad(rs.GetProperty("heading").GetSingle());

        int id = 0;
        foreach (var le in root.GetProperty("lines").EnumerateArray())
        {
            var line = new LineDef { Name = le.GetProperty("name").GetString() };
            int idx = 0;
            foreach (var pe in le.GetProperty("pieces").EnumerateArray())
            {
                string colors = pe.GetProperty("colors").GetString();
                string axis = pe.GetProperty("axis").GetString();
                line.Pieces.Add(new PieceDef
                {
                    Id = id++,
                    Line = L.Lines.Count,
                    Index = idx++,
                    Center = new Vector2(pe.GetProperty("x").GetSingle(), pe.GetProperty("z").GetSingle()) * Mm,
                    Axis = axis == "z" ? Mathf.Pi / 2f : 0f,
                    C0 = PColors.Parse(colors[0]),
                    C1 = PColors.Parse(colors[1]),
                });
            }
            line.Dir = (line.Pieces[^1].Center - line.Pieces[0].Center).Normalized();
            L.Lines.Add(line);
        }
        return L;
    }

    static Rect2 RectFrom(JsonElement a)
    {
        float x0 = a[0].GetSingle() * Mm, z0 = a[1].GetSingle() * Mm;
        float x1 = a[2].GetSingle() * Mm, z1 = a[3].GetSingle() * Mm;
        return new Rect2(x0, z0, x1 - x0, z1 - z0);
    }
}
