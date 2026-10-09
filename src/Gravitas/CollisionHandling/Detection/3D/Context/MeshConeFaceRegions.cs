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
/// <content>Exact component orientation and retained triangle grouping.</content>
internal sealed partial class MeshConeFaceRegions
{
    private int[] _offsets = Array.Empty<int>();
    private int[] _cursors = Array.Empty<int>();
    private int[] _triangles = Array.Empty<int>();
    private Region[] _regions = Array.Empty<Region>();
    private ConePlaneRayFrame _frame;
    private ulong[] _selectionValues = Array.Empty<ulong>();
    private int[] _selectionSigns = Array.Empty<int>();
    private Winner[] _winners = Array.Empty<Winner>();

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
            Array.Resize(ref _sampleCounts, capacity - 1);
            Array.Resize(ref _selectionValues, 2 * (capacity - 1) * ConePlaneRaySelection.StorageWords);
            Array.Resize(ref _selectionSigns, 2 * (capacity - 1) * ConePlaneRaySelection.SignCount);
            Array.Resize(ref _winners, 2 * (capacity - 1));
            Array.Resize(ref _samples, (capacity - 1) * ContactManifold.MaxContactsPerGroup);
        }
        if (_triangles.Length < admittedTriangles.Length)
            Array.Resize(ref _triangles, Math.Max(admittedTriangles.Length, _triangles.Length * 2));
        GroupTriangles(connectivity, admittedTriangles);
        mesh.GetLocalTriangleVertices(surfaceTriangle, out Vector3d a, out Vector3d b, out Vector3d c);
        _frame = new ConePlaneRayFrame(new FixedTriangle(a, b, c), mesh.Origin, mesh.Rotation,
            coneCenter, coneRotation, height, radius);
        var metrics = new WidePlaneMetrics(new FixedTriangle(a, b, c), mesh.Rotation);
        Array.Clear(_winners, 0, 2 * RegionCount);
        _poolEvents.FastClear();
        GatherEvents(mesh, surfaceTriangle, connectivity);
        for (int region = 0; region < RegionCount; region++)
        {
            ReduceRegion(region);
            ReduceSamples(region, metrics);
        }
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

    private void GatherEvents(PhysicsMesh mesh, int surfaceTriangle, MeshConeSurfaceConnectivity connectivity)
    {
        ReadOnlySpan<int> corners = GetCompleteConvexFan(mesh, surfaceTriangle, connectivity);
        Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.IntrinsicCapacity];
        int count = ConePlaneRayEvents.GetIntrinsicEvents(_frame, events);
        foreach (ConePlaneRayEvent descriptor in events[..count])
            AdmitEvent(mesh, connectivity, corners, ConePlaneRayEventSource.Plane, descriptor);
        ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
        count = ConePlaneRayEvents.GetBoundaryEvents(events);
        for (int edge = 0; edge < boundary.Length; edge += 2)
        {
            var segment = new FixedSegment(vertices[boundary[edge]], vertices[boundary[edge + 1]]);
            if (!_frame.IntersectsSegment(segment)) continue;
            var source = new ConePlaneRayEventSource(segment);
            foreach (ConePlaneRayEvent descriptor in events[..count])
                AdmitEvent(mesh, connectivity, corners, source, descriptor);
        }
    }

    private void AdmitEvent(PhysicsMesh mesh, MeshConeSurfaceConnectivity connectivity, ReadOnlySpan<int> corners,
        ConePlaneRayEventSource source, ConePlaneRayEvent descriptor)
    {
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps);
        var negative = new ConePlaneRaySelection(nv, ns);
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointValues, pointSigns);
        Span<ulong> root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        if (!ConePlaneRayEvents.TryEvaluateEvent(source, _frame, descriptor, point, root, ref positive, ref negative)) return;
        int region = FindPointRegion(mesh, connectivity, corners, point, root);
        if (region < 0) return;
        KeepWinner(2 * region, positive);
        KeepWinner(2 * region + 1, negative);
        bool pointRepresentable = ConePlaneRayPointMaterialization.TryGetWorldPoint(_frame, point, root, out Vector3d meshPoint);
        // Distinct constructions can certify the same exact point. Merge
        // their available directions before duplicate range checks; each
        // direction retains its own certificate for final materialization.
        for (int index = 0; index < _poolEvents.Count; index++)
        {
            PoolEvent retained = _poolEvents[index];
            if (retained.Region != region || retained.MeshPoint != meshPoint || ConePlaneRayEvents.CompareEventAnchors(
                source, descriptor, retained.Source, retained.Event, _frame, includeCoincidentProvenance: false) != 0) continue;
            _poolEvents[index] = new PoolEvent(retained.Source, retained.Event, region, meshPoint,
                retained.Positive.HasValue ? retained.Positive : CheckRay(positive, 1, pointRepresentable),
                retained.Negative.HasValue ? retained.Negative : CheckRay(negative, -1, pointRepresentable));
            return;
        }
        _poolEvents.Add(new PoolEvent(source, descriptor, region, meshPoint,
            CheckRay(positive, 1, pointRepresentable), CheckRay(negative, -1, pointRepresentable)));
    }

    private ReadOnlySpan<int> GetCompleteConvexFan(PhysicsMesh mesh, int surfaceTriangle, MeshConeSurfaceConnectivity connectivity)
    {
        if (RegionCount != 1) return ReadOnlySpan<int>.Empty;
        ReadOnlySpan<int> corners = mesh.GetConvexCoplanarPatchCornerVertexIndices(surfaceTriangle);
        if (corners.IsEmpty || corners.Length - 2 >= _offsets[1]) return ReadOnlySpan<int>.Empty;
        // Every prepared patch union also records its shared-edge neighbors.
        // A nonempty subset closed under those neighbors is the whole connected
        // owner. Check that once before replacing its authored triangles with
        // the strict convex ring's fan, and only when that fan is smaller.
        foreach (int triangle in _triangles.AsSpan(0, _offsets[1]))
            foreach (int neighbor in mesh.GetCoplanarTriangleNeighbors(triangle))
                if (connectivity.GetRegionOrdinal(neighbor) != 0) return ReadOnlySpan<int>.Empty;
        return corners;
    }

    private int FindPointRegion(PhysicsMesh mesh, MeshConeSurfaceConnectivity connectivity, ReadOnlySpan<int> corners,
        scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root)
    {
        if (!corners.IsEmpty)
        {
            ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
            for (int corner = 1; corner < corners.Length - 1; corner++)
                if (ConePlaneRayPointExits.ContainsTrianglePoint(new FixedTriangle(vertices[corners[0]],
                    vertices[corners[corner]], vertices[corners[corner + 1]]), _frame, point, root)) return 0;
            return -1;
        }
        // A shared admitted point joins its incident triangles, so its first
        // closed-triangle match identifies the unique connectivity component.
        // A complete convex owner uses O(T + K*(B-2)) assignment.
        // ponytail: O(K*T) for incomplete, nonconvex and multiple-region owners;
        // use prepared incidence if measured large patches need it.
        foreach (int triangle in _triangles.AsSpan(0, _offsets[RegionCount]))
        {
            mesh.GetLocalTriangleVertices(triangle, out Vector3d a, out Vector3d b, out Vector3d c);
            if (ConePlaneRayPointExits.ContainsTrianglePoint(new FixedTriangle(a, b, c), _frame, point, root))
                return connectivity.GetRegionOrdinal(triangle);
        }
        return -1;
    }

    private ConePlaneRaySelection BorrowWinner(int index)
    {
        var selection = new ConePlaneRaySelection(
            _selectionValues.AsSpan(index * ConePlaneRaySelection.StorageWords, ConePlaneRaySelection.StorageWords),
            _selectionSigns.AsSpan(index * ConePlaneRaySelection.SignCount, ConePlaneRaySelection.SignCount));
        Winner winner = _winners[index];
        if (winner.HasValue) selection.Restore(_frame, winner.Source, winner.Event);
        return selection;
    }

    private void KeepWinner(int index, scoped in ConePlaneRaySelection candidate)
    {
        var selection = BorrowWinner(index);
        if (selection.KeepEvaluated(candidate, _frame))
            _winners[index] = new Winner(selection.MaximumSource, selection.MaximumEvent);
    }

    private void ReduceRegion(int region)
    {
        var positive = BorrowWinner(2 * region);
        var negative = BorrowWinner(2 * region + 1);
        // Each closed component's maximum is intrinsic to its common plane or
        // lies on its true boundary; authored internal seams add no extrema.
        // Shared normal metric and frame scale cancel in the exact comparison.
        System.Diagnostics.Debug.Assert(positive.HasValue && negative.HasValue);
        int comparison = ContactQuadratic.CompareRatios(
            ContactQuadratic.At(positive.Values, positive.Signs, 0, ConePlaneRaySelection.FieldWords),
            ContactQuadratic.At(positive.Values, positive.Signs, 1, ConePlaneRaySelection.FieldWords), positive.Root,
            ContactQuadratic.At(negative.Values, negative.Signs, 0, ConePlaneRaySelection.FieldWords),
            ContactQuadratic.At(negative.Values, negative.Signs, 1, ConePlaneRaySelection.FieldWords), negative.Root);
        int orientation = comparison <= 0 ? 1 : -1;
        _regions[region] = new Region(orientation);
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
        if (_sampleCounts[region] > 0)
        {
            Sample maximum = _samples[region * ContactManifold.MaxContactsPerGroup];
            meshPoint = maximum.MeshPoint; conePoint = maximum.ConePoint; depth = maximum.Depth;
            return true;
        }
        int orientation = _regions[region].Orientation;
        var selected = BorrowWinner(2 * region + (orientation > 0 ? 0 : 1));
        return selected.TryMaterialize(_frame, orientation, out meshPoint, out conePoint, out depth);
    }

    private int ValidateRegion(int region)
    {
        if ((uint)region >= (uint)RegionCount) throw new ArgumentOutOfRangeException(nameof(region));
        return region;
    }

    private readonly struct Region
    {
        internal readonly int Orientation;
        internal Region(int orientation) => Orientation = orientation;
    }

    private readonly struct Winner
    {
        internal readonly ConePlaneRayEventSource Source;
        internal readonly ConePlaneRayEvent Event;
        internal readonly bool HasValue;
        internal Winner(ConePlaneRayEventSource source, ConePlaneRayEvent descriptor)
        { Source = source; Event = descriptor; HasValue = true; }
    }
}
