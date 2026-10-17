//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections;
using System;
using System.Collections.Generic;

namespace Gravitas.CollisionHandling;

/// <summary>Clips physical slab supports to the filled finite mesh owner.</summary>
internal sealed class MeshSlabFaceRegions
{
    private readonly ulong[] _values = new ulong[2 * FiniteSlabRaySelection.StorageWords];
    private readonly sbyte[] _signs = new sbyte[2 * FiniteSlabRaySelection.SignCount];
    private readonly FiniteSlabRaySelectionMetadata[] _peaks = new FiniteSlabRaySelectionMetadata[2];
    private readonly SwiftList<Sample> _events = new(), _pool = new();
    private readonly SwiftList<int> _ownerTriangles = new(), _admitted = new();
    private readonly SwiftHashSet<int> _visitedTriangles = new();
    private readonly FiniteSlabRayCandidateVisitor _visitor;
    private static readonly IComparer<Sample> PointComparer = Comparer<Sample>.Create(ComparePoints);
    private FiniteSlabRayFrame _frame;
    private int _region, _orientation, _supportDimension;
    private bool _intrinsicOnly, _usesSupportGraph;

    internal MeshSlabFaceRegions() => _visitor = AdmitPoint;
    internal int RegionCount { get; private set; }
    internal int Orientation => _orientation;
    internal ref readonly FiniteSlabRayFrame Frame => ref _frame;

    /// <summary>Finds a finite-boundary ray maximum from certified coplanar members in one slab chart.</summary>
    /// <remarks>Orientation follows the first member's canonical plane. Retained spans belong to their supplied member;
    /// sources preserve canonical boundary-span order within each member, which resolves same-member ties.
    /// Geometry must be committed and admitted. This does not gather intrinsic face events or construct a complete face sample pool.</remarks>
    internal static bool TryGetBoundaryWitness(
        ReadOnlySpan<(LSMeshCollider Collider, int SurfaceTriangle, MeshSurfaceBoundaryDomain.BoundarySpan Span)> sources,
        Vector3d center, FixedQuaternion rotation, Fixed64 length, Fixed64 radius, Fixed64 halfThickness,
        ReadOnlySpan<Vector2d> prismOffsets, int orientation, out FixedContactAnchors contact, out int supplier)
    {
        contact = default; supplier = -1;
        if (orientation != -1 && orientation != 1) throw new ArgumentOutOfRangeException(nameof(orientation));
        Span<ulong> bestValues = stackalloc ulong[FiniteSlabRaySelection.StorageWords], values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> bestSigns = stackalloc sbyte[FiniteSlabRaySelection.SignCount], signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        var best = new FiniteSlabRaySelection(bestValues, bestSigns);
        var candidate = new FiniteSlabRaySelection(values, signs);
        FiniteSlabRayFrame bestFrame = default; WideAxis3 commonNormal = default;
        int chosen = -1;
        for (int index = 0; index < sources.Length; index++)
        {
            ref readonly var input = ref sources[index]; PhysicsMesh mesh = input.Collider.Mesh;
            mesh.GetLocalTriangleVertices(input.SurfaceTriangle, out Vector3d a, out Vector3d b, out Vector3d c);
            var frame = new FiniteSlabRayFrame(new FixedTriangle(a, b, c), mesh.Origin, mesh.Rotation,
                center, rotation, length, radius, halfThickness);
            if (index == 0) commonNormal = frame.Plane.Normal;
            if (input.Span.Kind == MeshSurfaceBoundaryDomain.SpanKind.Joined) continue;
            int nativeOrientation = orientation * WideAxis3.Dot(frame.Plane.Normal, commonNormal).Sign;
            candidate.Restore(default);
            FiniteSlabRayExtrema.AccumulateSegment(frame, new FixedSegment(input.Span.Start, input.Span.End),
                input.Span.Interval, prismOffsets, nativeOrientation, ref candidate);
            if (!candidate.HasValue) continue;
            if (best.HasValue)
            {
                int order = candidate.CompareDepthAcrossFrames(best, frame, bestFrame);
                if (order < 0 || order == 0 && input.Collider.Id >= sources[chosen].Collider.Id) continue;
            }
            values.CopyTo(bestValues); signs.CopyTo(bestSigns); best.Restore(candidate.Metadata);
            bestFrame = frame; chosen = index;
        }
        if (!best.HasValue) return false;
        bool represented = best.TryGetContactAnchor(bestFrame, out Fixed64 depth, out bool clamped,
            out Vector3d authored, out FixedPointAnchor exit);
        PhysicsMesh supplierMesh = sources[chosen].Collider.Mesh;
        contact = new FixedContactAnchors(new FixedPointAnchor(supplierMesh.Origin, supplierMesh.Rotation, authored),
            exit, bestFrame.GetNormal(best.Metadata.Orientation), depth, clamped);
        supplier = chosen; return represented;
    }

    internal void Prepare(in FiniteSlabRayFrame frame, ReadOnlySpan<Vector2d> prismOffsets)
    {
        _frame = frame;
        for (int orientation = -1; orientation <= 1; orientation += 2)
        {
            var maximum = BorrowPeak(orientation);
            maximum.Restore(default);
            FiniteSlabRayExtrema.AccumulatePlane(frame, prismOffsets, orientation, ref maximum);
            _peaks[orientation > 0 ? 0 : 1] = maximum.Metadata;
        }
        var positive = BorrowPeak(1); var negative = BorrowPeak(-1);
        System.Diagnostics.Debug.Assert(positive.HasValue && negative.HasValue);
        _orientation = positive.CompareDepth(negative) <= 0 ? 1 : -1;
        _supportDimension = frame.GetSupportDimension(prismOffsets, _orientation);
    }

    internal void Build(PhysicsMesh mesh, int surfaceTriangle, MeshSurfaceConnectivity connectivity,
        ReadOnlySpan<Vector2d> prismOffsets, LSCollider2D? embedded)
    {
        _events.FastClear(); _admitted.FastClear();
        _usesSupportGraph = false; RegionCount = 0; _region = 0;
        _intrinsicOnly = _supportDimension == 2;
        ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
        bool containsSection = connectivity.RegionCount == 1 && _frame.RayNormal.X.IsZero && _frame.RayNormal.Z.IsZero;
        for (int edge = 0; edge < boundary.Length && containsSection; edge += 2)
            containsSection = !_frame.Section.IntersectsSegment(new FixedSegment(vertices[boundary[edge]], vertices[boundary[edge + 1]]), prismOffsets);
        if (containsSection)
        {
            // An axial cap projects to the original footprint; no true-boundary
            // crossing certifies that its complete support is inside the fill.
            VisitPlane(prismOffsets);
            RegionCount = _events.Count == 0 ? 0 : 1;
            return;
        }
        ReadOnlySpan<int> corners = mesh.GetConvexCoplanarPatchCornerVertexIndices(surfaceTriangle);
        bool hasSupport = false;
        if (!corners.IsEmpty)
        {
            for (int corner = 1; corner < corners.Length - 1; corner++)
                hasSupport |= VisitTriangle(new FixedTriangle(vertices[corners[0]], vertices[corners[corner]], vertices[corners[corner + 1]]), prismOffsets);
        }
        else
        {
            _ownerTriangles.FastClear(); _visitedTriangles.Clear();
            _ownerTriangles.Add(surfaceTriangle); _visitedTriangles.Add(surfaceTriangle);
            for (int index = 0; index < _ownerTriangles.Count; index++)
            {
                int triangle = _ownerTriangles[index];
                _region = triangle;
                mesh.GetLocalTriangleVertices(triangle, out Vector3d a, out Vector3d b, out Vector3d c);
                if (VisitTriangle(new FixedTriangle(a, b, c), prismOffsets)) { _admitted.Add(triangle); hasSupport = true; }
                foreach (int neighbor in mesh.GetCoplanarTriangleNeighbors(triangle))
                    if (_visitedTriangles.Add(neighbor)) _ownerTriangles.Add(neighbor);
            }
            if (_supportDimension == 2 && hasSupport)
            {
                // Perimeters have already consumed initial-section connectivity.
                // Reuse its graph for the actual projected support footprint.
                connectivity.BuildSlabSupport(mesh, surfaceTriangle, this, embedded, _admitted.AsReadOnlySpan());
                _usesSupportGraph = true;
            }
        }
        if (!hasSupport) { _events.FastClear(); return; }
        RegionCount = _usesSupportGraph ? connectivity.RegionCount : 1;
        for (int index = 0; index < _events.Count; index++)
        {
            Sample sample = _events[index];
            _events[index] = sample.WithRegion(_usesSupportGraph ? connectivity.GetRegionOrdinal(sample.Region) : 0);
        }
        // Only canonical real edges can add clipped support extrema; authored
        // diagonals must not change a flat support family's retained levers.
        _intrinsicOnly = false;
        int wholeOwner = mesh.GetManifoldSurfaceOwner(surfaceTriangle);
        for (int edge = 0; edge < boundary.Length; edge += 2)
        {
            var segment = new FixedSegment(vertices[boundary[edge]], vertices[boundary[edge + 1]]);
            if (!IntersectsSupport(segment, prismOffsets)) continue;
            _region = _usesSupportGraph ? MeshSurfaceBoundaryTopology.FindSupportEdgeRegion(mesh, wholeOwner,
                boundary[edge], boundary[edge + 1], connectivity, this, embedded) : 0;
            // A support-intersecting canonical span contains an admitted real subedge.
            System.Diagnostics.Debug.Assert(_region >= 0);
            VisitTriangle(new FixedTriangle(segment.Start, segment.End, segment.End), prismOffsets, boundary: true);
        }
    }

    internal bool IntersectsSupport(FixedSegment segment, ReadOnlySpan<Vector2d> prismOffsets)
    {
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        var maximum = new FiniteSlabRaySelection(values, signs);
        FiniteSlabRayExtrema.AccumulateTriangle(_frame, new FixedTriangle(segment.Start, segment.End, segment.End),
            prismOffsets, _orientation, ref maximum);
        return maximum.HasValue && maximum.CompareDepth(BorrowPeak(_orientation)) == 0;
    }

    private bool VisitTriangle(FixedTriangle triangle, ReadOnlySpan<Vector2d> prismOffsets, bool boundary = false)
    {
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        var maximum = new FiniteSlabRaySelection(values, signs);
        if (!boundary)
            FiniteSlabRayExtrema.AccumulatePhysicalSupportTriangle(_frame, triangle, prismOffsets, _orientation, ref maximum, _visitor);
        else
            FiniteSlabRayExtrema.AccumulateTriangle(_frame, triangle, prismOffsets, _orientation, ref maximum, _visitor);
        return maximum.HasValue && maximum.CompareDepth(BorrowPeak(_orientation)) == 0;
    }

    private void VisitPlane(ReadOnlySpan<Vector2d> prismOffsets)
    {
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        var maximum = new FiniteSlabRaySelection(values, signs);
        FiniteSlabRayExtrema.AccumulatePlane(_frame, prismOffsets, _orientation, ref maximum, _visitor);
    }

    private void AdmitPoint(scoped in FiniteSlabRaySelection candidate)
    {
        // A finite-domain ray maximum below the intrinsic peak is controlled by
        // a rim, whose real normal belongs to perimeter response, not this face.
        if (_intrinsicOnly && candidate.Metadata.DomainEdgeMask != 0
            || candidate.CompareDepth(BorrowPeak(_orientation)) != 0) return;
        bool represented = candidate.TryGetContactAnchor(_frame, out Fixed64 depth, out bool clamped,
            out Vector3d point, out FixedPointAnchor exit);
        System.Diagnostics.Debug.Assert(represented);
        var sample = new Sample(_region, point, exit, depth, clamped);
        foreach (Sample retained in _events.AsReadOnlySpan())
            if (retained.Region == _region && ComparePoints(sample, retained) == 0) return;
        _events.Add(sample);
    }

    internal void AppendContacts(LSMeshCollider collider, LSCollider2D? embedded, int surfaceTriangle,
        ContactManifold manifold, int namespace3D = 0, int namespace2D = 0)
    {
        collider.Mesh.GetLocalTriangleVertices(surfaceTriangle, out Vector3d a, out Vector3d b, out Vector3d c);
        var metrics = new WidePlaneMetrics(new FixedTriangle(a, b, c), FixedQuaternion.Identity);
        int surface = collider.Mesh.GetCanonicalSurfaceOrdinal(surfaceTriangle);
        Span<int> chosen = stackalloc int[ContactManifold.MaxContactsPerGroup];
        for (int region = 0; region < RegionCount; region++)
        {
            _pool.FastClear();
            foreach (Sample sample in _events.AsReadOnlySpan())
            {
                if (sample.Region != region) continue;
                bool duplicate = false;
                foreach (Sample retained in _pool.AsReadOnlySpan()) if (ComparePoints(sample, retained) == 0) { duplicate = true; break; }
                if (!duplicate) _pool.Add(sample);
            }
            // Every nonempty support region retains an intrinsic pole or a real boundary endpoint.
            System.Diagnostics.Debug.Assert(_pool.Count > 0);
            _pool.SortInPlace(PointComparer);
            int count = _supportDimension == 0 ? 1 : _supportDimension == 1 ? Math.Min(2, _pool.Count)
                : Math.Min(ContactManifold.MaxContactsPerGroup, _pool.Count);
            // Along a genuine support line normal velocity and wrench are affine:
            // its two extreme levers span every interior constraint, even across holes.
            if (_supportDimension <= 1) { chosen[0] = 0; chosen[1] = _pool.Count - 1; }
            else
            {
                chosen[0] = 0;
                for (int slot = 1; slot < count; slot++) chosen[slot] = MeshPlaneSampleReduction.ChooseNext(_pool.AsReadOnlySpan(), chosen[..slot], metrics, PointComparer);
            }
            var group = new ContactGroupKey(namespace3D, namespace2D, surface, 0, region);
            for (int slot = 0; slot < count; slot++)
            {
                Sample sample = _pool[chosen[slot]];
                manifold.AddContact(new ContactAnchor(new FixedPointAnchor(_frame.Origin, _frame.Rotation, sample.Point)),
                    new ContactAnchor(sample.Exit), sample.Depth, _frame.GetNormal(_orientation),
                    collider.Material, embedded?.Material ?? collider.Material, sample.Clamped,
                    featureNamespaceA: namespace3D, featureNamespaceB: namespace2D, group: group);
            }
        }
    }

    private FiniteSlabRaySelection BorrowPeak(int orientation)
    {
        int index = orientation > 0 ? 0 : 1;
        var result = new FiniteSlabRaySelection(_values.AsSpan(index * FiniteSlabRaySelection.StorageWords, FiniteSlabRaySelection.StorageWords),
            _signs.AsSpan(index * FiniteSlabRaySelection.SignCount, FiniteSlabRaySelection.SignCount));
        result.Restore(_peaks[index]); return result;
    }

    private static int ComparePoints(Sample first, Sample second)
    {
        int order = first.Point.X.CompareTo(second.Point.X);
        if (order == 0) order = first.Point.Y.CompareTo(second.Point.Y);
        if (order == 0) order = first.Point.Z.CompareTo(second.Point.Z);
        return order != 0 ? order : first.Exit.CompareLocalFeature(second.Exit);
    }

    private readonly struct Sample : IPlaneContactSample
    {
        internal readonly int Region;
        internal readonly FixedPointAnchor Exit;
        internal readonly Fixed64 Depth;
        internal readonly bool Clamped;
        public Vector3d Point { get; }
        internal Sample(int region, Vector3d point, FixedPointAnchor exit, Fixed64 depth, bool clamped)
        { Region = region; Point = point; Exit = exit; Depth = depth; Clamped = clamped; }
        internal Sample WithRegion(int region) => new(region, Point, Exit, Depth, Clamped);
    }
}
