//=======================================================================
// PhysicsMesh.SurfaceTrust.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using SwiftCollections;
using System;

namespace Gravitas.Colliders;

/// <content>Certifies embedded planar owners before manifold seam suppression and connectivity.</content>
public partial class PhysicsMesh
{
    private int[] _manifoldPatchIds = Array.Empty<int>();
    private int[] _preparedManifoldPatchIds = Array.Empty<int>();


    /// <summary>Gets an embedded planar patch owner, or the individual declined triangle.</summary>
    internal int GetManifoldSurfaceOwner(int triangleIndex)
    {
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(triangleIndex, _triangleCount, nameof(triangleIndex));
        int patch = _manifoldPatchIds[triangleIndex];
        return patch < 0 ? triangleIndex : patch;
    }

    /// <summary>Gets the true perimeter of a certified manifold owner; empty means use the triangle's three edges.</summary>
    internal ReadOnlySpan<int> GetManifoldSurfaceBoundaryVertexPairs(int triangleIndex)
    {
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(triangleIndex, _triangleCount, nameof(triangleIndex));
        int patch = _manifoldPatchIds[triangleIndex];
        return patch < 0 ? ReadOnlySpan<int>.Empty
            : _coplanarPatchBoundaryVertexPairs.AsSpan(_coplanarPatchBoundaryOffsets[patch],
                _coplanarPatchBoundaryOffsets[patch + 1] - _coplanarPatchBoundaryOffsets[patch]);
    }

    private void PrepareManifoldPatchTrust(Vector3d[] vertices, int[] triangles, int[] legacyIds,
        int[] boundaryOffsets, int[] boundaries)
    {
        EnsureTopologyBuffer(ref _preparedManifoldPatchIds, _triangleCount);
        for (int patch = 0; patch < _triangleCount; patch++)
        {
            if (legacyIds[patch] != patch) continue;
            // A strict convex ring already proves unit winding. Other patches
            // use the same exact chain proof after reducing their full boundary.
            bool trusted = _preparedConvexCoplanarPatchCornerOffsets[patch + 1]
                > _preparedConvexCoplanarPatchCornerOffsets[patch];
            if (!trusted)
            {
                int first = patch * 3;
                int axis = GetPatchProjectionAxis(vertices[triangles[first]], vertices[triangles[first + 1]], vertices[triangles[first + 2]]);
                int orientation = Vector2d.OrientationSign(ProjectToBoundaryPlane(vertices[triangles[first]], axis),
                    ProjectToBoundaryPlane(vertices[triangles[first + 1]], axis), ProjectToBoundaryPlane(vertices[triangles[first + 2]], axis));
                int cycles = PrepareReducedSurfaceCycles(vertices, patch, boundaryOffsets, boundaries, axis);
                trusted = IsEmbeddedSurfaceBoundary(vertices,
                    _surfaceCycleVertices.AsSpan(0, _surfaceCycleOffsets[cycles]),
                    _surfaceCycleOffsets.AsSpan(0, cycles + 1), axis, orientation);
            }
            _surfaceWriteOffsets[patch] = trusted ? patch : -1;
        }
        for (int triangle = 0; triangle < _triangleCount; triangle++)
            _preparedManifoldPatchIds[triangle] = legacyIds[triangle] < 0 ? -1 : _surfaceWriteOffsets[legacyIds[triangle]];
    }

    /// <summary>Certifies reduced directed planar cycles as the boundary of a domain with multiplicity zero or one.</summary>
    /// <remarks>
    /// Offsets delimit complete cycles of vertex indices, with monotone collinear
    /// subdivisions removed. Projection is injective on the common plane and
    /// orientation is the nonzero projected triangle orientation. Indices and
    /// offsets are prepared storage; geometric simplicity is not a precondition.
    /// </remarks>
    internal static bool IsEmbeddedSurfaceBoundary(ReadOnlySpan<Vector3d> vertices, ReadOnlySpan<int> corners,
        ReadOnlySpan<int> offsets, int axis, int orientation)
    {
        // Full opposite welded seams cancel in the oriented triangle chain.
        // Every remaining open cell's positive triangle multiplicity equals
        // boundary winding. Simple disjoint cycles with winding 0/1 therefore
        // exclude interior overlaps. An unshared touch on a canceled seam would
        // overlap its two incident half-neighborhoods; an uncanceled touch is
        // rejected below. Exact welding handles genuine shared vertices.
        // ponytail: O(B^2) in reduced boundary corners; use a deterministic
        // sweep only if detailed perimeters, rather than subdivision, dominate.
        int cycles = offsets.Length - 1;
        if (cycles == 0) return false;
        for (int cycle = 0; cycle < cycles; cycle++)
        {
            int begin = offsets[cycle], end = offsets[cycle + 1];
            if (end - begin < 3) return false;
            for (int edge = begin; edge < end; edge++)
            {
                Vector2d a = ProjectToBoundaryPlane(vertices[corners[edge]], axis);
                Vector2d b = ProjectToBoundaryPlane(vertices[corners[edge + 1 == end ? begin : edge + 1]], axis);
                Vector2d previous = ProjectToBoundaryPlane(vertices[corners[edge == begin ? end - 1 : edge - 1]], axis);
                // Monotone straight corners are already gone. A remaining
                // zero turn is a reversal or collapsed edge, never a simple rim.
                if (Vector2d.OrientationSign(previous, a, b) == 0) return false;
                for (int otherCycle = cycle; otherCycle < cycles; otherCycle++)
                {
                    int otherBegin = offsets[otherCycle], otherEnd = offsets[otherCycle + 1];
                    for (int other = otherCycle == cycle ? edge + 1 : otherBegin; other < otherEnd; other++)
                    {
                        if (otherCycle == cycle && (other == edge + 1 || edge == begin && other == end - 1)) continue;
                        Vector2d c = ProjectToBoundaryPlane(vertices[corners[other]], axis);
                        Vector2d d = ProjectToBoundaryPlane(vertices[corners[other + 1 == otherEnd ? otherBegin : other + 1]], axis);
                        if (SurfaceBoundarySegmentsIntersect(a, b, c, d)) return false;
                    }
                }
            }
        }
        for (int cycle = 0; cycle < cycles; cycle++)
        {
            Vector2d point = ProjectToBoundaryPlane(vertices[corners[offsets[cycle]]], axis);
            int winding = SurfaceCycleOrientation(vertices, corners, offsets[cycle], offsets[cycle + 1], axis) * orientation;
            for (int other = 0; other < cycles; other++)
                if (other != cycle && SurfaceCycleContains(vertices, corners, offsets[other], offsets[other + 1], axis, point))
                    winding += SurfaceCycleOrientation(vertices, corners, offsets[other], offsets[other + 1], axis) * orientation;
            // Each cycle bounds one new nested cell. Its first vertex lies
            // strictly inside every ancestor and outside all its descendants.
            if (winding < 0 || winding > 1) return false;
        }
        return true;
    }

    private static int SurfaceCycleOrientation(ReadOnlySpan<Vector3d> vertices, ReadOnlySpan<int> corners,
        int begin, int end, int axis)
    {
        int minimum = begin;
        Vector2d point = ProjectToBoundaryPlane(vertices[corners[minimum]], axis);
        for (int i = begin + 1; i < end; i++)
        {
            Vector2d candidate = ProjectToBoundaryPlane(vertices[corners[i]], axis);
            if (candidate.X < point.X || candidate.X == point.X && candidate.Y < point.Y)
            {
                minimum = i;
                point = candidate;
            }
        }
        return Vector2d.OrientationSign(
            ProjectToBoundaryPlane(vertices[corners[minimum == begin ? end - 1 : minimum - 1]], axis), point,
            ProjectToBoundaryPlane(vertices[corners[minimum + 1 == end ? begin : minimum + 1]], axis));
    }

    private static bool SurfaceCycleContains(ReadOnlySpan<Vector3d> vertices, ReadOnlySpan<int> corners,
        int begin, int end, int axis, Vector2d point)
    {
        int winding = 0;
        for (int i = begin; i < end; i++)
        {
            Vector2d a = ProjectToBoundaryPlane(vertices[corners[i]], axis);
            Vector2d b = ProjectToBoundaryPlane(vertices[corners[i + 1 == end ? begin : i + 1]], axis);
            // Half-open crossings count vertices once, without a division or
            // rounded ray intersection. Disjointness excludes boundary points.
            if (a.Y <= point.Y)
            {
                if (b.Y > point.Y && Vector2d.OrientationSign(a, b, point) > 0) winding++;
            }
            else if (b.Y <= point.Y && Vector2d.OrientationSign(a, b, point) < 0) winding--;
        }
        return winding != 0;
    }

    private static bool SurfaceBoundarySegmentsIntersect(Vector2d a, Vector2d b, Vector2d c, Vector2d d)
    {
        if (FixedMath.Max(FixedMath.Min(a.X, b.X), FixedMath.Min(c.X, d.X)) > FixedMath.Min(FixedMath.Max(a.X, b.X), FixedMath.Max(c.X, d.X))
            || FixedMath.Max(FixedMath.Min(a.Y, b.Y), FixedMath.Min(c.Y, d.Y)) > FixedMath.Min(FixedMath.Max(a.Y, b.Y), FixedMath.Max(c.Y, d.Y))) return false;
        return Vector2d.OrientationSign(a, b, c) * Vector2d.OrientationSign(a, b, d) <= 0
            && Vector2d.OrientationSign(c, d, a) * Vector2d.OrientationSign(c, d, b) <= 0;
    }
}
