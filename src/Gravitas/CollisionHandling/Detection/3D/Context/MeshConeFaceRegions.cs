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
    private Vector3d _worldNormal;
    private Vector3d _samplingOrigin;
    private ulong[] _selectionValues = Array.Empty<ulong>();
    private int[] _selectionSigns = Array.Empty<int>();
    private Winner[] _winners = Array.Empty<Winner>();
    private readonly ConePlaneRayEventVisitor _eventVisitor;
    private PhysicsMesh? _gatherMesh;
    private MeshConeSurfaceConnectivity? _gatherConnectivity;
    private int _gatherSurfaceTriangle;
    private bool _gatherUsesConvexFan;
    private bool _gatherContainsSection;
    private bool _wholeConeSamplingRangeRepresentable;

    internal MeshConeFaceRegions() => _eventVisitor = AdmitEvaluatedPoint;

    internal int RegionCount { get; private set; }
    internal bool HasBoundaryIntersection { get; private set; }
    internal ref readonly ConePlaneRayFrame Frame => ref _frame;
    internal Vector3d SamplingOrigin => _samplingOrigin;

    internal void Build(PhysicsMesh mesh, int surfaceTriangle, Vector3d coneCenter,
        FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius,
        MeshConeSurfaceConnectivity connectivity, ReadOnlySpan<int> admittedTriangles, Vector3d samplingOrigin = default)
    {
        mesh.GetLocalTriangleVertices(surfaceTriangle, out Vector3d a, out Vector3d b, out Vector3d c);
        var frame = new ConePlaneRayFrame(new FixedTriangle(a, b, c), mesh.Origin, mesh.Rotation,
            coneCenter, coneRotation, height, radius);
        Build(mesh, surfaceTriangle, frame, connectivity, admittedTriangles, samplingOrigin);
    }

    /// <remarks>
    /// Connectivity must have been built from this same mesh, surface, cone
    /// frame and complete admitted list. Descriptors and grouped indices are
    /// valid until the next Build; only the prepared surface owner is consumed.
    /// This selects an orientation and its extremal ray, before sample coverage
    /// or rigid-body response policy is applied.
    /// </remarks>
    internal void Build(PhysicsMesh mesh, int surfaceTriangle, in ConePlaneRayFrame frame,
        MeshConeSurfaceConnectivity connectivity, ReadOnlySpan<int> admittedTriangles, Vector3d samplingOrigin = default)
    {
        RegionCount = connectivity.RegionCount;
        HasBoundaryIntersection = false;
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
        _frame = frame;
        _samplingOrigin = samplingOrigin;
        _wholeConeSamplingRangeRepresentable = ConePlaneRayPointMaterialization.IsRangeRepresentable(_frame, samplingOrigin);
        _worldNormal = _frame.GetWorldNormal(mesh.Rotation);
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
        _gatherSurfaceTriangle = surfaceTriangle;
        _gatherMesh = mesh;
        _gatherConnectivity = connectivity;
        try
        {
            ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
            ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
            int firstIntersectingEdge = 0;
            for (; firstIntersectingEdge < boundary.Length; firstIntersectingEdge += 2)
                if (_frame.IntersectsSegment(new FixedSegment(vertices[boundary[firstIntersectingEdge]],
                    vertices[boundary[firstIntersectingEdge + 1]]))) break;
            HasBoundaryIntersection = firstIntersectingEdge < boundary.Length;
            // The finite cone section is connected and at least one triangle
            // is admitted. A complete owner with a disjoint true boundary
            // contains that whole section. Neighbor closure also preserves
            // callers' deliberately partial admitted domains; hole boundaries
            // and closed touch prevent the containment certificate.
            if (RegionCount == 1 && firstIntersectingEdge == boundary.Length)
            {
                _gatherContainsSection = IsCompleteSurface(mesh, connectivity);
                // A complete section needs no fan, and an incomplete owner
                // cannot use one. Do not repeat the same closure check.
                _gatherUsesConvexFan = false;
            }
            else
            {
                _gatherContainsSection = false;
                _gatherUsesConvexFan = !GetCompleteConvexFan(mesh, surfaceTriangle, connectivity).IsEmpty;
            }
            // Synchronous visitation borrows each exact construction directly,
            // avoiding chart reconstruction per descriptor. Only compact pool
            // metadata and the two owned winner banks survive the callback.
            ConePlaneRayEvents.VisitEvents(ConePlaneRayEventSource.Plane, _frame, _eventVisitor);
            for (int edge = firstIntersectingEdge; edge < boundary.Length; edge += 2)
            {
                var segment = new FixedSegment(vertices[boundary[edge]], vertices[boundary[edge + 1]]);
                if (edge != firstIntersectingEdge && !_frame.IntersectsSegment(segment)) continue;
                var source = new ConePlaneRayEventSource(segment);
                ConePlaneRayEvents.VisitEvents(source, _frame, _eventVisitor);
            }
        }
        finally
        {
            // Context scratch must not retain the last queried mesh, including
            // when exact geometry or materialization rejects the operation.
            _gatherMesh = null;
            _gatherConnectivity = null;
            _gatherUsesConvexFan = false;
            _gatherContainsSection = false;
        }
    }

    private void AdmitEvaluatedPoint(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        ConePlaneRayEvent descriptor, scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
        scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative)
    {
        PhysicsMesh mesh = _gatherMesh!;
        ReadOnlySpan<int> corners = _gatherUsesConvexFan
            ? mesh.GetConvexCoplanarPatchCornerVertexIndices(_gatherSurfaceTriangle) : ReadOnlySpan<int>.Empty;
        int region = FindPointRegion(mesh, _gatherConnectivity!, corners, point, root);
        if (region < 0) return;
        KeepWinner(2 * region, positive);
        KeepWinner(2 * region + 1, negative);
        bool pointRepresentable = ConePlaneRayPointMaterialization.TryGetPointInFrame(frame, point, root, _samplingOrigin, out Vector3d meshPoint);
        // Distinct constructions can certify the same exact point. Merge
        // their available directions before duplicate range checks; each
        // direction retains its own certificate for final materialization.
        // The callback already owns the incoming exact point; reconstruct
        // only the retained event when checking coordinate equality.
        for (int index = 0; index < _poolEvents.Count; index++)
        {
            PoolEvent retained = _poolEvents[index];
            if (retained.Region != region || retained.MeshPoint != meshPoint || ConePlaneRayEvents.CompareEventAnchors(
                point, root, retained.Source, retained.Event, frame) != 0) continue;
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
        return IsCompleteSurface(mesh, connectivity) ? corners : ReadOnlySpan<int>.Empty;
    }

    private bool IsCompleteSurface(PhysicsMesh mesh, MeshConeSurfaceConnectivity connectivity)
    {
        // Every prepared patch union also records its shared-edge neighbors.
        // A nonempty subset closed under those neighbors is the whole connected
        // owner. A singleton owner has no such neighbors and is complete.
        foreach (int triangle in _triangles.AsSpan(0, _offsets[1]))
            foreach (int neighbor in mesh.GetCoplanarTriangleNeighbors(triangle))
                if (connectivity.GetRegionOrdinal(neighbor) != 0) return false;
        return true;
    }

    private int FindPointRegion(PhysicsMesh mesh, MeshConeSurfaceConnectivity connectivity, ReadOnlySpan<int> corners,
        scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root)
    {
        if (_gatherContainsSection) return 0;
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

    // The cone witness lies along the exit ray from the mesh. Response moves
    // the cone in the opposite direction, including at exact zero-depth touch.
    internal Vector3d GetNormal(int region) => -_worldNormal * GetOrientation(region);

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
        return selected.TryMaterialize(_frame, orientation, _samplingOrigin, out meshPoint, out conePoint, out depth);
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
