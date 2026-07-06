//=======================================================================
// CollisionDetection.Sat.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool CheckProjectedAxis(
        FixedRange projectionA,
        FixedRange projectionB,
        Vector3d axis,
        Vector3d displacementAtoB,
        ref AxisPenetration penetration)
    {
        if (!projectionA.Overlaps(projectionB))
            return false;

        Fixed64 depth = ComputeMinimumProjectionOverlap(projectionA, projectionB);
        if (!penetration.HasValue || depth < penetration.Depth)
        {
            Vector3d orientedAxis = Vector3d.Dot(axis, displacementAtoB) < Fixed64.Zero ? -axis : axis;
            penetration = new AxisPenetration(orientedAxis, depth);
        }

        return true;
    }

    private static bool CheckVertexProjectionAxis(
        Vector3d[] verticesA,
        Vector3d[] verticesB,
        Vector3d axis,
        Vector3d displacementAtoB,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        FixedRange projectionA = FixedRange.MinRange;
        FixedRange projectionB = FixedRange.MinRange;
        AxisProjectionHelper.ProjectPolygonOntoAxis(normalizedAxis, verticesA, ref projectionA);
        AxisProjectionHelper.ProjectPolygonOntoAxis(normalizedAxis, verticesB, ref projectionB);
        return CheckProjectedAxis(projectionA, projectionB, normalizedAxis, displacementAtoB, ref penetration);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeMinimumProjectionOverlap(FixedRange projectionA, FixedRange projectionB)
    {
        Fixed64 pushALeft = projectionA.Max - projectionB.Min;
        Fixed64 pushARight = projectionB.Max - projectionA.Min;
        Fixed64 overlap = FixedMath.Min(pushALeft, pushARight);
        return overlap > Fixed64.Zero ? overlap : Fixed64.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryNormalizeAxis(Vector3d axis, out Vector3d normalizedAxis)
    {
        Fixed64 magnitudeSqr = axis.MagnitudeSquared;
        if (magnitudeSqr <= Fixed64.Epsilon)
        {
            normalizedAxis = Vector3d.Zero;
            return false;
        }

        normalizedAxis = axis / FixedMath.Sqrt(magnitudeSqr);
        return true;
    }
}
