//=======================================================================
// MeshConeFaceRegions.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using System;

namespace Gravitas.CollisionHandling;

/// <summary>Chooses the smaller exact face-ray maximum independently for each admitted surface component.</summary>
internal sealed class MeshConeFaceRegions
{
    private int[] _offsets = Array.Empty<int>();
    private int[] _cursors = Array.Empty<int>();
    private int[] _triangles = Array.Empty<int>();
    private Region[] _regions = Array.Empty<Region>();
    private ConePlaneRayFrame _frame;

    internal int RegionCount { get; private set; }

    /// <remarks>
    /// Connectivity must have been built from this same mesh, surface, cone
    /// frame and complete admitted list. Descriptors and grouped indices are
    /// valid until the next Build; only the prepared surface owner is consumed.
    /// This selects an orientation and its extremal ray, before sample coverage
    /// or rigid-body response policy is applied.
    /// </remarks>
    internal void Build(PhysicsMesh mesh, int surfaceTriangle, Vector3d coneCenter,
        FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius,
        MeshConeSurfaceConnectivity connectivity, ReadOnlySpan<int> admittedTriangles)
    {
        RegionCount = connectivity.RegionCount;
        if (RegionCount == 0) return;
        if (_offsets.Length < RegionCount + 1)
        {
            int capacity = Math.Max(RegionCount + 1, _offsets.Length * 2);
            Array.Resize(ref _offsets, capacity);
            Array.Resize(ref _cursors, capacity - 1);
            Array.Resize(ref _regions, capacity - 1);
        }
        if (_triangles.Length < admittedTriangles.Length)
            Array.Resize(ref _triangles, Math.Max(admittedTriangles.Length, _triangles.Length * 2));
        GroupTriangles(connectivity, admittedTriangles);
        mesh.GetLocalTriangleVertices(surfaceTriangle, out Vector3d a, out Vector3d b, out Vector3d c);
        _frame = new ConePlaneRayFrame(new FixedTriangle(a, b, c), mesh.Origin, mesh.Rotation,
            coneCenter, coneRotation, height, radius);
        for (int region = 0; region < RegionCount; region++) ReduceRegion(mesh, region);
    }

    private void GroupTriangles(MeshConeSurfaceConnectivity connectivity, ReadOnlySpan<int> admittedTriangles)
    {
        // Counting and prefix offsets form retained CSR groups in O(T+R).
        // Discovery order may change within a group; exact geometric event
        // ties, rather than that order, select the same physical winner.
        Array.Clear(_offsets, 0, RegionCount + 1);
        foreach (int triangle in admittedTriangles)
        {
            int region = connectivity.GetRegionOrdinal(triangle);
            if (region >= 0) _offsets[region + 1]++;
        }
        for (int region = 0; region < RegionCount; region++)
        {
            _offsets[region + 1] += _offsets[region];
            _cursors[region] = _offsets[region];
        }
        foreach (int triangle in admittedTriangles)
        {
            int region = connectivity.GetRegionOrdinal(triangle);
            if (region >= 0) _triangles[_cursors[region]++] = triangle;
        }
    }

    private void ReduceRegion(PhysicsMesh mesh, int region)
    {
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);
        for (int index = _offsets[region]; index < _offsets[region + 1]; index++)
        {
            mesh.GetLocalTriangleVertices(_triangles[index], out Vector3d a, out Vector3d b, out Vector3d c);
            ConePlaneRayEvents.Accumulate(
                new FixedTriangle(a, b, c), _frame, ref positive, ref negative);
        }
        // Every connectivity component contains an admitted triangle. Both
        // finite maxima share the same positive normal metric and frame scale,
        // so comparing their retained ratios exactly compares physical depths.
        System.Diagnostics.Debug.Assert(positive.HasValue && negative.HasValue);
        int comparison = ContactQuadratic.CompareRatios(
            ContactQuadratic.At(positive.Values, positive.Signs, 0, ConePlaneRaySelection.FieldWords),
            ContactQuadratic.At(positive.Values, positive.Signs, 1, ConePlaneRaySelection.FieldWords), positive.Root,
            ContactQuadratic.At(negative.Values, negative.Signs, 0, ConePlaneRaySelection.FieldWords),
            ContactQuadratic.At(negative.Values, negative.Signs, 1, ConePlaneRaySelection.FieldWords), negative.Root);
        int orientation = comparison <= 0 ? 1 : -1;
        ConePlaneRaySelection selected = orientation > 0 ? positive : negative;
        _regions[region] = new Region(selected.MaximumTriangle, selected.MaximumEvent, orientation);
    }

    internal int GetOrientation(int region) => _regions[ValidateRegion(region)].Orientation;

    internal ReadOnlySpan<int> GetTriangleIndices(int region)
    {
        ValidateRegion(region);
        return _triangles.AsSpan(_offsets[region], _offsets[region + 1] - _offsets[region]);
    }

    internal bool TryGetSelectedRay(int region, out Vector3d meshPoint, out Vector3d conePoint, out Fixed64 depth)
    {
        meshPoint = conePoint = default; depth = default;
        if ((uint)region >= (uint)RegionCount) return false;
        Region selected = _regions[region];
        return ConePlaneRayEvents.TryMaterializeEvent(selected.Triangle, _frame, selected.Event,
            selected.Orientation, out meshPoint, out conePoint, out depth);
    }

    private int ValidateRegion(int region)
    {
        if ((uint)region >= (uint)RegionCount) throw new ArgumentOutOfRangeException(nameof(region));
        return region;
    }

    private readonly struct Region
    {
        internal readonly FixedTriangle Triangle;
        internal readonly ConePlaneRayEvent Event;
        internal readonly int Orientation;

        internal Region(FixedTriangle triangle, ConePlaneRayEvent descriptor, int orientation)
        {
            Triangle = triangle; Event = descriptor; Orientation = orientation;
        }
    }
}
