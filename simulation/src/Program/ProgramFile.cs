using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using Godot;

namespace DominoSim;

/// <summary>
/// Driving parameters in the units of the real robot (mission.json → robot.drive): wheel speeds in
/// deg/s of the wheel, deg_per_mm converts them to mm/s. Loaded from ../mission.json when it exists,
/// so the sim drives at the same speeds as the real program.
/// </summary>
public static class DriveConfig
{
    public static float DegPerMm = 2.05f;
    public static float Speed = 400, MinSpeed = 100, TurnSpeed = 300;
    public static float CurveInner = 100, CurveOuter = 500, CurveSPer90 = 1.34f;
    // Manual mode (Shift = slow / pivot curve).
    public static float SlowSpeed = 150, SlowTurnSpeed = 80, PivotInner = 0, PivotOuter = 300;
    public static string Source = "defaults";
    // Motor C (robot.arm): times of the mechanism moves, initial state, macros of mission.json.
    public static (string rot, string slot) ArmInitial = ("CW", "RIGHT");
    public static float ArmSpeed = 1000, ShiftSpeed = 1400, NudgeDeg = 800;
    public static Dictionary<string, float> RotPositions = new() { ["CW"] = -1000, ["NONE"] = 0, ["CCW"] = 1000 };
    public static Dictionary<string, float> Transitions = new();
    public static Dictionary<string, float> ShiftExtra = new();
    public static JsonObject Macros = new();

    /// <summary>Wheel speed in deg/s → tyre speed in m/s.</summary>
    public static float Ms(float degPerS) => degPerS / DegPerMm * 0.001f;

    public static void Load()
    {
        string path = ProgramFile.MissionJsonPath;
        if (!File.Exists(path)) return;
        try
        {
            var d = JsonNode.Parse(File.ReadAllText(path))?["robot"]?["drive"];
            if (d == null) return;
            float F(string k, float def) => d[k] is JsonValue v && v.TryGetValue(out double x) ? (float)x : def;
            DegPerMm = F("deg_per_mm", DegPerMm);
            Speed = F("speed", Speed);
            MinSpeed = F("min_speed", MinSpeed);
            TurnSpeed = F("turn_speed", TurnSpeed);
            CurveInner = F("curve_inner_speed", CurveInner);
            CurveOuter = F("curve_outer_speed", CurveOuter);
            CurveSPer90 = F("curve_s_per_90", CurveSPer90);
            Source = path;
            var root = JsonNode.Parse(File.ReadAllText(path));
            if (root?["macros"] is JsonObject m) Macros = (JsonObject)m.DeepClone();
            var a = root?["robot"]?["arm"];
            if (a != null)
            {
                float A(string k, float def) => a[k] is JsonValue v && v.TryGetValue(out double x) ? (float)x : def;
                ArmSpeed = A("speed", ArmSpeed);
                ShiftSpeed = A("shift_speed", ShiftSpeed);
                NudgeDeg = A("nudge_deg", NudgeDeg);
                if (a["initial"] is JsonObject ini)
                    ArmInitial = (ini["rot"]?.GetValue<string>() ?? "CW", ini["slot"]?.GetValue<string>() ?? "RIGHT");
                void Dict(string key, Dictionary<string, float> into)
                {
                    if (a[key] is not JsonObject o) return;
                    into.Clear();
                    foreach (var kv in o)
                        if (!kv.Key.StartsWith("_") && kv.Value is JsonValue v && v.TryGetValue(out double x)) into[kv.Key.Replace(" ", "").ToUpperInvariant()] = (float)x;
                }
                Dict("rot_positions", RotPositions);
                Dict("transitions", Transitions);
                Dict("shift_extra", ShiftExtra);
            }
        }
        catch (Exception e) { GD.PushWarning($"mission.json: {e.Message}"); }
    }

    public static string Describe() =>
        $"drive: speed {Speed:0} deg/s ({Ms(Speed) * 1000:0} mm/s), turn {TurnSpeed:0}, curve {CurveInner:0}/{CurveOuter:0}, deg_per_mm {DegPerMm}  [{Source}]";
}

/// <summary>
/// A program = list of steps in the mission.json format ({"cmd": "drive", "mm": 300}, ...).
/// Accepted files: a plain array of steps, {"steps": [...], "macros": {...}, "start": {...}},
/// or mission.json itself (one of its "missions" + its "macros").
/// </summary>
public sealed class ProgramFile
{
    public string Name = "program";
    public string Path;
    public List<JsonObject> Steps = new();
    public JsonObject Macros = new();
    /// <summary>Start pose in layout units (mm, heading in degrees of the sim: 0 = +x, clockwise); null = layout start.</summary>
    public (float x, float z, float heading)? Start;

    public static string ProgramsDir => ProjectSettings.GlobalizePath("res://programs");
    public static string MissionJsonPath => System.IO.Path.GetFullPath(System.IO.Path.Combine(ProjectSettings.GlobalizePath("res://"), "..", "mission.json"));

    public static ProgramFile Load(string path, string mission = null)
    {
        var root = JsonNode.Parse(File.ReadAllText(path), documentOptions: new JsonDocumentOptions { CommentHandling = JsonCommentHandling.Skip, AllowTrailingCommas = true });
        var p = new ProgramFile { Path = path, Name = System.IO.Path.GetFileNameWithoutExtension(path) };
        JsonArray steps;
        if (root is JsonArray arr) steps = arr;
        else if (root?["missions"] is JsonObject missions)
        {
            mission ??= root["active_mission"]?.GetValue<string>() ?? missions.First().Key;
            steps = missions[mission] as JsonArray ?? throw new Exception($"mise '{mission}' není v {path}");
            p.Name = $"mission.json: {mission}";
            if (root["macros"] is JsonObject m) p.Macros = (JsonObject)m.DeepClone();
        }
        else
        {
            steps = root?["steps"] as JsonArray ?? throw new Exception($"{path}: chybí \"steps\"");
            if (root["macros"] is JsonObject m) p.Macros = (JsonObject)m.DeepClone();
            if (root["start"] is JsonObject s)
                p.Start = (s["x"].GetValue<float>(), s["z"].GetValue<float>(), s["heading"]?.GetValue<float>() ?? 0f);
        }
        foreach (var st in steps)
            if (st is JsonObject o) p.Steps.Add((JsonObject)o.DeepClone());
        return p;
    }

    /// <summary>Mission names in ../mission.json (empty when it does not exist).</summary>
    public static List<string> MissionNames()
    {
        try
        {
            if (!File.Exists(MissionJsonPath)) return new();
            var root = JsonNode.Parse(File.ReadAllText(MissionJsonPath));
            return (root?["missions"] as JsonObject)?.Select(kv => kv.Key).ToList() ?? new();
        }
        catch { return new(); }
    }

    public static string ActiveMission()
    {
        try { return JsonNode.Parse(File.ReadAllText(MissionJsonPath))?["active_mission"]?.GetValue<string>(); }
        catch { return null; }
    }

    public static List<string> SavedPrograms()
    {
        if (!Directory.Exists(ProgramsDir)) return new();
        return Directory.GetFiles(ProgramsDir, "*.json").OrderByDescending(f => File.GetLastWriteTimeUtc(f)).ToList();
    }

    /// <summary>One step per line, so the array can be pasted straight into mission.json.</summary>
    public static string StepsToJson(IEnumerable<JsonObject> steps, string indent = "  ")
    {
        var opts = new JsonSerializerOptions { Encoder = System.Text.Encodings.Web.JavaScriptEncoder.UnsafeRelaxedJsonEscaping };
        string One(JsonObject s) => "{" + string.Join(", ", s.Select(kv =>
            JsonSerializer.Serialize(kv.Key, opts) + ": " + (kv.Value?.ToJsonString(opts) ?? "null"))) + "}";
        var list = steps.Select(s => indent + "  " + One(s)).ToList();
        return "[\n" + string.Join(",\n", list) + "\n" + indent + "]";
    }

    public string Save(string path)
    {
        Directory.CreateDirectory(System.IO.Path.GetDirectoryName(path));
        var sb = new StringBuilder();
        sb.Append("{\n");
        sb.Append($"  \"_\": \"Záznam ruční jízdy ze simulace ({DateTime.Now:yyyy-MM-dd HH:mm}). Kroky jsou ve formátu mission.json; turn/curve kladně = doleva. intake = jen simulace.\",\n");
        if (Start is { } s)
            sb.Append(FormattableString.Invariant($"  \"start\": {{\"x\": {s.x:0}, \"z\": {s.z:0}, \"heading\": {s.heading:0.#}}},\n"));
        sb.Append("  \"steps\": ").Append(StepsToJson(Steps)).Append("\n}\n");
        File.WriteAllText(path, sb.ToString());
        Path = path;
        Name = System.IO.Path.GetFileNameWithoutExtension(path);
        return path;
    }

    /// <summary>Steps with macros expanded and comment-only entries removed.</summary>
    public List<JsonObject> Flatten()
    {
        var outp = new List<JsonObject>();
        void Add(IEnumerable<JsonObject> steps, int depth)
        {
            foreach (var st in steps)
            {
                string cmd = st["cmd"]?.GetValue<string>();
                if (cmd == null) continue;
                if (cmd == "macro" && depth < 8)
                {
                    string name = st["name"]?.GetValue<string>();
                    if (name != null && Macros[name] is JsonArray m)
                    {
                        Add(m.OfType<JsonObject>(), depth + 1);
                        continue;
                    }
                }
                outp.Add(st);
            }
        }
        Add(Steps, 0);
        return outp;
    }
}

/// <summary>What to do after the scene (re)loads. Survives ReloadCurrentScene.</summary>
public static class Session
{
    public enum Mode { Menu, Auto, Ours, Play, Drive }
    public static Mode Current = Mode.Menu;
    public static bool ArgsApplied;
    public static ProgramFile Program;
    /// <summary>Mission of mission.json run by our Python code (Mode.Ours).</summary>
    public static string OursMission = "main";
    /// <summary>Drive mode: steps replayed (fast) before manual control continues (used by undo).</summary>
    public static List<JsonObject> PendingSteps;
}
