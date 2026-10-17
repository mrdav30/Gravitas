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
    private Region[] _regions = Array.Empty<Region>();
    private ConePlaneRayFrame _frame;
    private Vector3d _worldNormal;
    private Vector3d _samplingOrigin;
    private ulong[] _selectionValues = Array.Empty<ulong>();
    private int[] _selectionSigns = Array.Empty<int>();
    private Winner[] _winners = Array.Empty<Winner>();
    private readonly ConePlaneRayEventVisitor _eventVisitor;
    private PhysicsMesh? _gatherMesh;
    private MeshSurfaceConnectivity? _gatherConnectivity;
    private int _gatherSurfaceTriangle;
    private bool _gatherUsesConvexFan;
    private bool _gatherContainsSection;
    private bool _wholeConeSamplingRangeRepresentable;

    internal MeshConeFaceRegions()
    {
        _eventVisitor = AdmitEvaluatedPoint;
        _sampleComparer = System.Collections.Generic.Comparer<Sample>.Create(ComparePoints);
    }

    internal int RegionCount { get; private set; }
    internal bool HasBoundaryIntersection { get; private set; }
    internal ref readonly ConePlaneRayFrame Frame => ref _frame;
    internal Vector3d SamplingOrigin => _samplingOrigin;

    /// <summary>Finds a finite-boundary ray maximum from certified coplanar members in one cone chart.</summary>
    /// <remarks>Orientation follows the first member's canonical plane. Retained spans belong to their supplied member;
    /// sources preserve canonical boundary-span order within each member, which resolves same-member ties.
    /// Geometry must be committed and admitted. This does not gather intrinsic face events or construct a complete face sample pool.</remarks>
    internal static bool TryGetBoundaryWitness(
        ReadOnlySpan<(LSMeshCollider Collider, int SurfaceTriangle, MeshSurfaceBoundaryDomain.BoundarySpan Span)> sources,
        LSConeCollider cone, int orientation, out FixedContactAnchors contact, out int supplier)
    {
        contact = default; supplier = -1;
        if (orientation != -1 && orientation != 1) throw new ArgumentOutOfRangeException(nameof(orientation));
        Span<ulong> bestValues = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> bestSigns = stackalloc int[ConePlaneRaySelection.SignCount];
        var best = new ConePlaneRaySelection(bestValues, bestSigns);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointValues, pointSigns);
        Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.BoundaryCapacity];
        int count = ConePlaneRayEvents.GetBoundaryEvents(events), chosen = -1, chosenOrientation = 0;
        ConePlaneRayFrame bestFrame = default; WideAxis3 commonNormal = default;
        for (int index = 0; index < sources.Length; index++)
        {
            ref readonly var input = ref sources[index]; PhysicsMesh mesh = input.Collider.Mesh;
            mesh.GetLocalTriangleVertices(input.SurfaceTriangle, out Vector3d a, out Vector3d b, out Vector3d c);
            var frame = new ConePlaneRayFrame(new FixedTriangle(a, b, c), mesh.Origin, mesh.Rotation,
                cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius);
            if (index == 0) commonNormal = frame.Normal;
            if (input.Span.Kind == MeshSurfaceBoundaryDomain.SpanKind.Joined) continue;
            int nativeOrientation = orientation * WideAxis3.Dot(frame.Normal, commonNormal).Sign;
            var source = new ConePlaneRayEventSource(new FixedSegment(input.Span.Start, input.Span.End));
            foreach (ConePlaneRayEvent descriptor in events[..count])
            {
                if (!ConePlaneRayEvents.TryEvaluateEvent(source, frame, descriptor, input.Span.Interval,
                    point, root, ref positive, ref negative, nativeOrientation)) continue;
                scoped ConePlaneRaySelection candidate = nativeOrientation > 0 ? positive : negative;
                if (!candidate.HasValue) continue;
                if (best.HasValue)
                {
                    int order = candidate.CompareDepth(best);
                    if (order < 0 || order == 0 && input.Collider.Id >= sources[chosen].Collider.Id) continue;
                }
                candidate.Values.CopyTo(bestValues); candidate.Signs.CopyTo(bestSigns);
                best.Restore(frame, candidate.MaximumSource, candidate.MaximumEvent);
                bestFrame = frame; chosen = index; chosenOrientation = nativeOrientation;
            }
        }
        // Clipped descriptors cannot enter the ordinary replay pool: retain
        // the exact native point and exit fields through final materialization.
        if (!best.HasValue) return false;
        bool represented = best.TryGetRoundedMaximumDepth(out Fixed64 depth)
            & ConePlaneRayPointMaterialization.TryGetAuthoredPoint(bestFrame, best.Point, best.Root, out Vector3d authored)
            & ConePlaneRayPointMaterialization.TryGetExitConeLocalPoint(bestFrame, best.Point, best.Root,
                ContactQuadratic.At(best.Values, best.Signs, 0, ConePlaneRaySelection.FieldWords),
                ContactQuadratic.At(best.Values, best.Signs, 1, ConePlaneRaySelection.FieldWords), chosenOrientation, out Vector3d exit);
        PhysicsMesh supplierMesh = sources[chosen].Collider.Mesh;
        contact = new FixedContactAnchors(new FixedPointAnchor(supplierMesh.Origin, supplierMesh.Rotation, authored),
            new FixedPointAnchor(cone.Center, cone.Rotation, exit),
            -bestFrame.GetWorldNormal(supplierMesh.Rotation) * chosenOrientation, depth, false);
        supplier = chosen; return represented;
    }

    internal void Build(PhysicsMesh mesh, int surfaceTriangle, Vector3d coneCenter,
        FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius,
        MeshSurfaceConnectivity connectivity, ReadOnlySpan<int> admittedTriangles, Vector3d samplingOrigin = default)
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
        MeshSurfaceConnectivity connectivity, ReadOnlySpan<int> admittedTriangles, Vector3d samplingOrigin = default)
    {
        RegionCount = connectivity.RegionCount;
        HasBoundaryIntersection = false;
        if (RegionCount == 0) return;
        if (_regions.Length < RegionCount)
        {
            int capacity = Math.Max(RegionCount, _regions.Length * 2);
            Array.Resize(ref _regions, capacity);
            Array.Resize(ref _sampleCounts, capacity);
            Array.Resize(ref _selectionValues, 2 * capacity * ConePlaneRaySelection.StorageWords);
            Array.Resize(ref _selectionSigns, 2 * capacity * ConePlaneRaySelection.SignCount);
            Array.Resize(ref _winners, 2 * capacity);
            Array.Resize(ref _samples, capacity * ContactManifold.MaxContactsPerGroup);
        }
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

    private void GatherEvents(PhysicsMesh mesh, int surfaceTriangle, MeshSurfaceConnectivity connectivity)
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
                _gatherContainsSection = connectivity.IsCompleteSingleRegion(mesh);
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

    private ReadOnlySpan<int> GetCompleteConvexFan(PhysicsMesh mesh, int surfaceTriangle, MeshSurfaceConnectivity connectivity)
    {
        if (RegionCount != 1) return ReadOnlySpan<int>.Empty;
        ReadOnlySpan<int> corners = mesh.GetConvexCoplanarPatchCornerVertexIndices(surfaceTriangle);
        if (corners.IsEmpty || corners.Length - 2 >= connectivity.TriangleIndices.Length) return ReadOnlySpan<int>.Empty;
        return connectivity.IsCompleteSingleRegion(mesh) ? corners : ReadOnlySpan<int>.Empty;
    }

    private int FindPointRegion(PhysicsMesh mesh, MeshSurfaceConnectivity connectivity, ReadOnlySpan<int> corners,
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
        foreach (int triangle in connectivity.TriangleIndices)
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
