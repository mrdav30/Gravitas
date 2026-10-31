//=======================================================================
// ConvexSweepQueryWorker.ConvexPatch.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using System;

namespace Gravitas.Queries;

/// <content>Sweeps a certified whole-mesh convex patch when a finite center crossing proves an ordinary impact.</content>
internal sealed partial class ConvexSweepQueryWorker
{
    private bool TrySweepCertifiedConvexPatch(in ConvexShape source, LSMeshCollider target,
        out Physics3DHit hit, out Fixed64 numerator)
    {
        hit = default;
        numerator = default;
        PhysicsMesh mesh = target.Mesh;
        if (_admitTriangle != null || !source.ContainsCenter
            || _triangleCandidates.Count != mesh.TriangleCount || _triangleCandidates.Count < 2)
            return false;

        source.GetSourceBounds(out Vector3d min, out Vector3d max);
        Vector3d padding = Vector3d.One * ContactTolerance;
        if (!Vector3d.TrySubtract(min, padding, out Vector3d paddedMin)
            || !Vector3d.TryAdd(max, padding, out Vector3d paddedMax)
            || SweepBoundsUtility.OverlapsInclusive(paddedMin, paddedMax, target.Bounds.Min, target.Bounds.Max)
            || !mesh.HasSingleConvexCoplanarPatch)
            return false;

        ReadOnlySpan<int> corners = mesh.GetConvexCoplanarPatchCornerVertexIndices(0);
        ReadOnlySpan<Vector3d> vertices = mesh.GeometryVertices;
        var first = new FixedTriangle(vertices[corners[0]], vertices[corners[1]], vertices[corners[2]]);
        WideDirection3d plane = WideTriangleRelations.GetScaledTriangleLocalNormal(first, mesh.GeometryFrame)
            .Transform(mesh.GeometryFrame.GeometryBasis);
        var motion = new WideAxis3(Signed320.ExtendValue(Signed192.Raw(_displacement.X)),
            Signed320.ExtendValue(Signed192.Raw(_displacement.Y)), Signed320.ExtendValue(Signed192.Raw(_displacement.Z)));
        if (WideDirection3d.Dot(plane, motion).IsZero)
            return false;

        // Prepare has already admitted both center endpoints. A finite center
        // crossing avoids an extra GJK pass for perimeter-only and miss cases.
        _ = source.GetCenterAnchor().TryGetPoint(out Vector3d start);
        _ = Vector3d.TryAdd(start, _displacement, out Vector3d end);
        var segment = new FixedSegment(start, end);
        for (int corner = 1; corner < corners.Length - 1; corner++)
        {
            var triangle = new FixedTriangle(vertices[corners[0]], vertices[corners[corner]], vertices[corners[corner + 1]]);
            if (!WideTriangleRelations.TryGetSegmentIntersection(triangle, mesh.GeometryFrame, segment, out _, out _))
                continue;
            if (TrySweepConvexTarget(source, new ConvexShape(target, Vector3d.Zero, convexPatch: true),
                target, out hit, out numerator) && numerator > Fixed64.Zero)
                return true;
            hit = default;
            numerator = default;
            return false;
        }
        return false;
    }
}
