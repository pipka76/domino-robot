using System;
using System.Collections.Generic;
using Godot;

namespace DominoSim;

/// <summary>
/// Grid A* over the field half. Free space = centre at least WallInflate from the walls and
/// ObstacleInflate from every lying piece. Cells near the start/goal are always passable so the
/// robot can leave/enter tight spots (row ends next to pieces). Result is string-pulled.
/// </summary>
public sealed class Navigator
{
    public float Cell = 0.01f;
    public float ObstacleInflate = 0.115f;
    public float WallInflate = 0.12f;
    public float EscapeRadius = 0.17f;
    public float EscapeWall = 0.10f;

    readonly float _w, _d;
    readonly int _nx, _nz;

    public Navigator(float width, float depth)
    {
        _w = width;
        _d = depth;
        _nx = (int)Mathf.Ceil(width / Cell);
        _nz = (int)Mathf.Ceil(depth / Cell);
    }

    public List<Vector2> Plan(Vector2 start, Vector2 goal, List<Obb> obstacles, out bool ok)
    {
        var free = new bool[_nx * _nz];
        for (int j = 0; j < _nz; j++)
            for (int i = 0; i < _nx; i++)
                free[j * _nx + i] = IsFree(CellCenter(i, j), start, goal, obstacles);

        int s = Index(start), g = Index(goal);
        free[s] = free[g] = true;
        var gScore = new float[_nx * _nz];
        var came = new int[_nx * _nz];
        var closed = new bool[_nx * _nz];
        Array.Fill(gScore, float.MaxValue);
        Array.Fill(came, -1);
        var open = new PriorityQueue<int, float>();
        gScore[s] = 0;
        open.Enqueue(s, 0);
        ok = false;
        while (open.Count > 0)
        {
            int c = open.Dequeue();
            if (closed[c]) continue;
            closed[c] = true;
            if (c == g) { ok = true; break; }
            int ci = c % _nx, cj = c / _nx;
            for (int dj = -1; dj <= 1; dj++)
                for (int di = -1; di <= 1; di++)
                {
                    if (di == 0 && dj == 0) continue;
                    int ni = ci + di, nj = cj + dj;
                    if (ni < 0 || nj < 0 || ni >= _nx || nj >= _nz) continue;
                    int nIdx = nj * _nx + ni;
                    if (!free[nIdx] || closed[nIdx]) continue;
                    float ng = gScore[c] + ((di != 0 && dj != 0) ? 1.41421f : 1f) * Cell;
                    if (ng < gScore[nIdx])
                    {
                        gScore[nIdx] = ng;
                        came[nIdx] = c;
                        open.Enqueue(nIdx, ng + CellCenter(ni, nj).DistanceTo(goal));
                    }
                }
        }
        if (!ok) return new List<Vector2> { start, goal };

        var raw = new List<Vector2>();
        for (int c = g; c != -1; c = came[c]) raw.Add(CellCenter(c % _nx, c / _nx));
        raw.Reverse();
        raw[0] = start;
        raw[^1] = goal;

        // String pulling.
        var path = new List<Vector2> { raw[0] };
        int k = 0;
        while (k < raw.Count - 1)
        {
            int far = k + 1;
            for (int m = raw.Count - 1; m > k + 1; m--)
                if (LineFree(raw[k], raw[m], start, goal, obstacles)) { far = m; break; }
            path.Add(raw[far]);
            k = far;
        }
        return path;
    }

    bool LineFree(Vector2 a, Vector2 b, Vector2 start, Vector2 goal, List<Obb> obs)
    {
        float len = a.DistanceTo(b);
        int n = Math.Max(1, (int)(len / 0.005f));
        for (int i = 0; i <= n; i++)
            if (!IsFree(a.Lerp(b, i / (float)n), start, goal, obs)) return false;
        return true;
    }

    public float WallDistance(Vector2 p) => Mathf.Min(Mathf.Min(p.X, _w - p.X), Mathf.Min(p.Y, _d - p.Y));

    public bool IsFree(Vector2 p, Vector2 start, Vector2 goal, List<Obb> obs)
    {
        float wall = Mathf.Min(Mathf.Min(p.X, _w - p.X), Mathf.Min(p.Y, _d - p.Y));
        bool escape = p.DistanceTo(start) < EscapeRadius || p.DistanceTo(goal) < EscapeRadius;
        if (wall < (escape ? EscapeWall : WallInflate)) return false;
        if (escape) return true;
        foreach (var o in obs)
            if (o.Distance(p) < ObstacleInflate) return false;
        return true;
    }

    Vector2 CellCenter(int i, int j) => new((i + 0.5f) * Cell, (j + 0.5f) * Cell);

    int Index(Vector2 p)
    {
        int i = Math.Clamp((int)(p.X / Cell), 0, _nx - 1);
        int j = Math.Clamp((int)(p.Y / Cell), 0, _nz - 1);
        return j * _nx + i;
    }
}
