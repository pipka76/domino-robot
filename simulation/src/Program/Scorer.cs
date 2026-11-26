using System.Collections.Generic;
using System.Linq;

namespace DominoSim;

/// <summary>Indicative score of whatever lies in the yellow area: longest chain of valid joints (n² + perpendicular joints).</summary>
public static class Scorer
{
    public static string Summary(IEnumerable<Domino> dominos, FieldLayout layout)
    {
        var inner = layout.YellowInner;
        var all = dominos.Where(d => !d.Carried).ToList();
        int touching = all.Count(d => d.Footprint.Corners().Any(c => layout.YellowRect.HasPoint(c)));
        var pcs = all.Where(d => d.Upright && d.Footprint.Corners().All(c => inner.HasPoint(c))).ToList();
        int n = pcs.Count;
        var ok = new bool[n, n];
        var perp = new bool[n, n];
        for (int i = 0; i < n; i++)
            for (int j = i + 1; j < n; j++)
            {
                var r = JointEvaluator.Evaluate(pcs[i].Pose, pcs[j].Pose);
                ok[i, j] = ok[j, i] = r.Valid;
                perp[i, j] = perp[j, i] = r.Perpendicular;
            }
        int bestLen = n > 0 ? 1 : 0, bestPerp = 0;
        var used = new bool[n];
        void Dfs(int at, int len, int p)
        {
            if (len > bestLen || (len == bestLen && p > bestPerp)) { bestLen = len; bestPerp = p; }
            for (int k = 0; k < n; k++)
                if (!used[k] && ok[at, k])
                {
                    used[k] = true;
                    Dfs(k, len + 1, p + (perp[at, k] ? 1 : 0));
                    used[k] = false;
                }
        }
        for (int i = 0; i < n; i++) { used[i] = true; Dfs(i, 1, 0); used[i] = false; }
        int pts = bestLen >= 2 ? bestLen * bestLen + bestPerp : 0;
        return $"ve žlutém {touching} kostek, nejdelší platný řetěz {bestLen} (≈{pts} b. vč. {bestPerp} kolmých)";
    }
}
