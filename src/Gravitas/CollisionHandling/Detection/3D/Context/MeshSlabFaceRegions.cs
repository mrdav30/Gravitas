//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.Materials;
using SwiftCollections;
using System;
using System.Collections.Generic;

namespace Gravitas.CollisionHandling;

/// <summary>Clips physical slab supports to the filled finite mesh owner.</summary>
/// <content>Native support charts and the shared finite sample reduction.</content>
internal sealed partial class MeshSlabFaceRegions
{
    private ulong[] _values = new ulong[FiniteSlabRaySelection.StorageWords];
    private sbyte[] _signs = new sbyte[FiniteSlabRaySelection.SignCount];
    private FiniteSlabRaySelectionMetadata[] _peaks = new FiniteSlabRaySelectionMetadata[1];
    private readonly SwiftList<Sample> _events = new(), _pool = new();
    private readonly SwiftList<int> _ownerTriangles = new(), _admitted = new();
    private readonly SwiftHashSet<int> _visitedTriangles = new();
    private readonly FiniteSlabRayCandidateVisitor _visitor;
    private static readonly IComparer<Sample> PointComparer = Comparer<Sample>.Create(ComparePoints);
    private FiniteSlabRayFrame _frame;
    private int _fieldWords = FiniteSlabPolynomial.Words;
    private int SelectionWords => FiniteSlabRaySelection.GetStorageWords(_fieldWords);
    private int _region, _orientation, _supportDimension;
    private bool _intrinsicOnly, _usesSupportGraph, _hasPlaneSamples;

    internal MeshSlabFaceRegions() => _visitor = AdmitPoint;
    internal int RegionCount { get; private set; }
    internal int Orientation => _orientation;
    internal ref readonly FiniteSlabRayFrame Frame => ref _frame;

    internal void Prepare(in FiniteSlabRayFrame frame, ReadOnlySpan<Vector2d> prismOffsets, bool coveredSection)
    {
        _member = -1;
        PrismOffsets = default;
        PrepareFrame(frame, prismOffsets, coveredSection);
    }

    private void PrepareFrame(in FiniteSlabRayFrame frame, ReadOnlySpan<Vector2d> prismOffsets, bool coveredSection = false)
    {
        _frame = frame;
        _fieldWords = frame.FieldWords;
        // Slot zero serves an isolated owner. Joined members retain their chosen
        // peak only for this build, so switching suppliers does not solve the
        // same intrinsic plane again. Every new build overwrites active slots.
        int slots = _member + 2;
        if (_values.Length < slots * SelectionWords) Array.Resize(ref _values, slots * SelectionWords);
        if (_signs.Length < slots * FiniteSlabRaySelection.SignCount) Array.Resize(ref _signs, slots * FiniteSlabRaySelection.SignCount);
        if (_peaks.Length < slots) Array.Resize(ref _peaks, slots);
        _hasPlaneSamples = _member < 0 && coveredSection;
        if (_member < 0) _events.FastClear();
        // The admitted section intersects this plane. Exact support projections
        // choose the nearer exit before either expensive witness construction.
        int order = frame.ComparePlaneExitDepths(prismOffsets);
        _orientation = order <= 0 ? 1 : -1;
        if (_member >= 0 && order == 0)
        {
            WideDirection3d normal = frame.Plane.Normal;
            int sign = !normal.X.IsZero ? normal.X.Sign : !normal.Y.IsZero ? normal.Y.Sign : normal.Z.Sign;
            _orientation = -sign;
        }
        if (_hasPlaneSamples) { _region = 0; _intrinsicOnly = false; }
        var maximum = BorrowPeak();
        maximum.Restore(default);
        FiniteSlabRayExtrema.AccumulatePlane(frame, prismOffsets, _orientation, ref maximum,
            _hasPlaneSamples ? _visitor : null, includeInteriorSample: !_hasPlaneSamples);
        _peaks[_member + 1] = maximum.Metadata;
        System.Diagnostics.Debug.Assert(maximum.HasValue);
        _supportDimension = frame.GetSupportDimension(prismOffsets, _orientation);
    }

    // With at least one admitted triangle, no boundary crossing certifies the
    // entire connected axial footprint. Without admission it can lie in a hole.
    internal static bool ContainsSection(PhysicsMesh mesh, int surfaceTriangle, in FiniteSlabRayFrame frame,
        ReadOnlySpan<Vector2d> prismOffsets)
    {
        ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.GeometryVertices;
        bool containsSection = frame.RayNormal.X.IsZero && frame.RayNormal.Z.IsZero;
        for (int edge = 0; edge < boundary.Length && containsSection; edge += 2)
            containsSection = !frame.Section.IntersectsSegment(new FixedSegment(vertices[boundary[edge]], vertices[boundary[edge + 1]]), prismOffsets);
        return containsSection;
    }

    internal void Build(PhysicsMesh mesh, int surfaceTriangle, MeshSurfaceConnectivity connectivity,
        ReadOnlySpan<Vector2d> prismOffsets, LSCollider2D? embedded, bool containsSection)
    {
        if (!_hasPlaneSamples) _events.FastClear();
        _admitted.FastClear();
        _usesSupportGraph = false; RegionCount = 0; _region = 0;
        _intrinsicOnly = _supportDimension == 2;
        if (containsSection)
        {
            // An axial cap projects to the original footprint; no true-boundary
            // crossing certifies that its complete support is inside the fill.
            if (!_hasPlaneSamples) VisitPlane(prismOffsets);
            RegionCount = _events.Count == 0 ? 0 : 1;
            return;
        }
        _events.FastClear(); _hasPlaneSamples = false;
        ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.GeometryVertices;
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
                mesh.GetGeometryTriangleVertices(triangle, out Vector3d a, out Vector3d b, out Vector3d c);
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
        if (_member >= 0)
        {
            GatherJoinedBoundary(mesh, surfaceTriangle, connectivity);
            return;
        }
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
        Span<ulong> values = stackalloc ulong[SelectionWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        var maximum = new FiniteSlabRaySelection(values, signs, _fieldWords);
        FiniteSlabRayExtrema.AccumulateTriangle(_frame, new FixedTriangle(segment.Start, segment.End, segment.End),
            prismOffsets, _orientation, ref maximum);
        return maximum.HasValue && maximum.CompareDepth(BorrowPeak()) == 0;
    }

    private bool VisitTriangle(FixedTriangle triangle, ReadOnlySpan<Vector2d> prismOffsets, bool boundary = false)
    {
        Span<ulong> values = stackalloc ulong[SelectionWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        var maximum = new FiniteSlabRaySelection(values, signs, _fieldWords);
        if (!boundary)
            FiniteSlabRayExtrema.AccumulatePhysicalSupportTriangle(_frame, triangle, prismOffsets, _orientation, ref maximum, _visitor);
        else
            FiniteSlabRayExtrema.AccumulateTriangle(_frame, triangle, prismOffsets, _orientation, ref maximum, _visitor);
        return maximum.HasValue && maximum.CompareDepth(BorrowPeak()) == 0;
    }

    private void VisitPlane(ReadOnlySpan<Vector2d> prismOffsets, FiniteSlabRayCandidateVisitor? visitor = null)
    {
        Span<ulong> values = stackalloc ulong[SelectionWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        var maximum = new FiniteSlabRaySelection(values, signs, _fieldWords);
        FiniteSlabRayExtrema.AccumulatePlane(_frame, prismOffsets, _orientation, ref maximum, visitor ?? _visitor);
    }

    private void AdmitPoint(scoped in FiniteSlabRaySelection candidate)
    {
        // A finite-domain ray maximum below the intrinsic peak is controlled by
        // a rim, whose real normal belongs to perimeter response, not this face.
        if (_intrinsicOnly && candidate.Metadata.DomainEdgeMask != 0
            || !_hasPlaneSamples && candidate.CompareDepth(BorrowPeak()) != 0) return;
        if (_member >= 0 && _completeSupports[_joined.FindResponseComponent(_member)] && !ContainsSupportPoint(candidate)) return;
        bool represented = candidate.TryGetContactAnchor(_frame, out Fixed64 depth, out bool clamped,
            out Vector3d point, out FixedPointAnchor exit);
        System.Diagnostics.Debug.Assert(represented);
        Vector3d authored = point;
        if (_member >= 0 || _frame.Section.RadialFrame.ShapeFrame.HasExactScale)
        {
            bool rounded = candidate.TryGetSourcePointInSamplingFrame(_frame, out point);
            System.Diagnostics.Debug.Assert(rounded);
        }
        var sample = new Sample(_region, point, exit, depth, clamped, _member, authored);
        foreach (Sample retained in _events.AsReadOnlySpan())
            if (retained.Region == _region && ComparePoints(sample, retained) == 0) return;
        _events.Add(sample);
    }

    internal void AppendContacts(LSMeshCollider collider, int surfaceTriangle,
        ContactManifold manifold, int namespace3D, int namespace2D, PhysicsMaterial targetMaterial)
    {
        collider.Mesh.GetGeometryTriangleVertices(surfaceTriangle, out Vector3d a, out Vector3d b, out Vector3d c);
        var metrics = _frame.Section.RadialFrame.ShapeFrame.HasExactScale
            ? new WidePlaneMetrics(new FixedTriangle(a, b, c), collider.Mesh.GeometryFrame)
            : new WidePlaneMetrics(new FixedTriangle(a, b, c), FixedQuaternion.Identity);
        int surface = collider.Mesh.GetCanonicalSurfaceOrdinal(surfaceTriangle);
        Span<int> chosen = stackalloc int[ContactManifold.MaxContactsPerGroup];
        for (int region = 0; region < RegionCount; region++)
        {
            int count = ReduceSamples(region, _events.AsReadOnlySpan(), metrics, _supportDimension, chosen);
            var group = new ContactGroupKey(namespace3D, namespace2D, surface, 0, region);
            for (int slot = 0; slot < count; slot++)
            {
                Sample sample = _pool[chosen[slot]];
                FixedPointAnchor source = _frame.Section.RadialFrame.ShapeFrame.HasExactScale
                    ? new FixedPointAnchor(_frame.Center, FixedQuaternion.Identity, sample.Point)
                    : new FixedPointAnchor(_frame.Origin, _frame.Rotation, sample.Point);
                manifold.AddContact(new ContactAnchor(source),
                    new ContactAnchor(sample.Exit), sample.Depth, _frame.GetNormal(_orientation),
                    collider.Material, targetMaterial, sample.Clamped,
                    featureNamespaceA: namespace3D, featureNamespaceB: namespace2D, group: group);
            }
        }
    }

    private int ReduceSamples(int region, ReadOnlySpan<Sample> events, WidePlaneMetrics metrics, int dimension, Span<int> chosen)
    {
        _pool.FastClear();
        foreach (Sample sample in events)
        {
            if (sample.Region != region) continue;
            bool duplicate = false;
            foreach (Sample retained in _pool.AsReadOnlySpan()) if (ComparePoints(sample, retained) == 0) { duplicate = true; break; }
            if (!duplicate) _pool.Add(sample);
        }
        // Every nonempty support region retains an intrinsic pole or a real boundary endpoint.
        System.Diagnostics.Debug.Assert(_pool.Count > 0);
        _pool.SortInPlace(PointComparer);
        int count = dimension == 0 ? 1 : dimension == 1 ? Math.Min(2, _pool.Count)
            : Math.Min(ContactManifold.MaxContactsPerGroup, _pool.Count);
        // Retaining every point needs no geometric ranking; the manifold
        // publishes the same set in stable anchor order regardless of insertion.
        if (count == _pool.Count)
        {
            for (int slot = 0; slot < count; slot++) chosen[slot] = slot;
            return count;
        }
        // Normal velocity and wrench are affine along a genuine support line.
        if (dimension <= 1) { chosen[0] = 0; chosen[1] = _pool.Count - 1; }
        else
        {
            chosen[0] = 0;
            for (int slot = 1; slot < count; slot++) chosen[slot] = MeshPlaneSampleReduction.ChooseNext(_pool.AsReadOnlySpan(), chosen[..slot], metrics, PointComparer);
        }
        return count;
    }

    private FiniteSlabRaySelection BorrowPeak()
    {
        int index = _member + 1;
        var result = new FiniteSlabRaySelection(_values.AsSpan(index * SelectionWords, SelectionWords),
            _signs.AsSpan(index * FiniteSlabRaySelection.SignCount, FiniteSlabRaySelection.SignCount), _fieldWords);
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
        internal readonly int Member;
        internal readonly Vector3d Authored;
        public Vector3d Point { get; }
        internal Sample(int region, Vector3d point, FixedPointAnchor exit, Fixed64 depth, bool clamped, int member = -1, Vector3d authored = default)
        { Region = region; Point = point; Exit = exit; Depth = depth; Clamped = clamped; Member = member; Authored = authored; }
        internal Sample WithRegion(int region) => new(region, Point, Exit, Depth, Clamped, Member, Authored);
    }
}
