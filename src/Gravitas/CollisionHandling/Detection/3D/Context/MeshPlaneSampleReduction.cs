//=======================================================================
// MeshPlaneSampleReduction.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using System;
using System.Collections.Generic;

namespace Gravitas.CollisionHandling;

internal interface IPlaneContactSample
{
    Vector3d Point { get; }
}

/// <summary>Ranks finite plane samples after their exact maximum has seeded the pool.</summary>
internal static class MeshPlaneSampleReduction
{
    internal static int ChooseNext<T>(ReadOnlySpan<T> pool, ReadOnlySpan<int> chosen,
        in WideDirection3d normal, IComparer<T> pointComparer) where T : struct, IPlaneContactSample
    {
        const int words = 32;
        Span<ulong> values = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        var bestN = ContactQuadratic.At(values, signs, 0, words);
        var bestD = ContactQuadratic.At(values, signs, 1, words);
        var n = ContactQuadratic.At(values, signs, 2, words);
        var d = ContactQuadratic.At(values, signs, 3, words);
        int best = -1;
        for (int candidate = 1; candidate < pool.Length; candidate++)
        {
            if (chosen.IndexOf(candidate) >= 0) continue;
            Vector3d p = pool[candidate].Point;
            d.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.One)));
            if (chosen.Length == 1) WidePlaneMetrics.GetSpanSquared(normal, pool[chosen[0]].Point, p, n);
            else if (chosen.Length == 2) WidePlaneMetrics.GetAreaSquared(normal, pool[chosen[0]].Point, pool[chosen[1]].Point, p, n);
            else WidePlaneMetrics.GetTriangleDistanceSquared(normal, p, pool[chosen[0]].Point,
                pool[chosen[1]].Point, pool[chosen[2]].Point, n, d);
            int comparison = best < 0 ? 1 : ContactQuadratic.CompareRatios(n, d, ReadOnlySpan<ulong>.Empty,
                bestN, bestD, ReadOnlySpan<ulong>.Empty);
            if (comparison > 0 || comparison == 0 && pointComparer.Compare(pool[candidate], pool[best]) < 0)
            { best = candidate; n.CopyTo(bestN); d.CopyTo(bestD); }
        }
        return best;
    }

    internal static int ChooseNext<T>(ReadOnlySpan<T> pool, ReadOnlySpan<int> chosen,
        WidePlaneMetrics metrics, IComparer<T> pointComparer) where T : struct, IPlaneContactSample
    {
        int best = -1;
        Signed832 bestN = default, bestD = default;
        for (int candidate = 1; candidate < pool.Length; candidate++)
        {
            if (chosen.IndexOf(candidate) >= 0) continue;
            Vector3d p = pool[candidate].Point;
            Signed832 n, d = Signed832.ExtendValue(Signed192.Signed(1));
            if (chosen.Length == 1) n = metrics.GetSpanSquared(pool[chosen[0]].Point, p);
            else if (chosen.Length == 2) n = metrics.GetAreaSquared(pool[chosen[0]].Point, pool[chosen[1]].Point, p);
            else metrics.GetTriangleDistanceSquared(p, pool[chosen[0]].Point,
                pool[chosen[1]].Point, pool[chosen[2]].Point, out n, out d);
            int comparison = best < 0 ? 1 : WidePlaneMetrics.CompareRatios(n, d, bestN, bestD);
            // Rounded metrics rank coverage only. Equal metrics retain the
            // owner's exact point/provenance order before any solver reduction.
            if (comparison > 0 || comparison == 0 && pointComparer.Compare(pool[candidate], pool[best]) < 0)
            { best = candidate; bestN = n; bestD = d; }
        }
        return best;
    }
}
