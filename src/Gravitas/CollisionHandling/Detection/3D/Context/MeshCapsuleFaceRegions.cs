//=======================================================================
// MeshCapsuleFaceRegions.cs
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

/// <summary>Retains each admitted capsule section's exact face direction and finite sample coverage.</summary>
/// <content>Single-member gathering, exact point storage and final sample access.</content>
internal sealed partial class MeshCapsuleFaceRegions
{
    private ulong[] _values = Array.Empty<ulong>();
    private int[] _signs = Array.Empty<int>();
    private sbyte[] _valueSigns = Array.Empty<sbyte>();
    private CapsulePlaneRaySelectionMetadata[] _winners = Array.Empty<CapsulePlaneRaySelectionMetadata>();
    private int[] _orientations = Array.Empty<int>(), _counts = Array.Empty<int>();
    private Sample[] _samples = Array.Empty<Sample>();
    private readonly SwiftList<Sample> _events = new(), _pool = new();
    private ulong[] _pointValues = Array.Empty<ulong>();
    private int[] _pointSigns = Array.Empty<int>();
    private readonly SwiftList<PointStorage> _points = new();
    private int _pointWordCount;
    private readonly CapsulePlaneRayEventVisitor _visitor;
    private readonly IComparer<Sample> _pointComparer;
    private CapsulePlaneRayFrame _frame;
    private int _fieldWords = CapsulePlaneRaySelection.FieldWords, _rootWords = CapsulePlaneRaySelection.RootWords,
        _valueWords = CapsulePlaneRayValue.PolynomialWords;
    private int SelectionWords => CapsulePlaneRaySelection.GetStorageWords(_fieldWords, _rootWords, _valueWords);

    private void SelectBanks(in CapsulePlaneRayFrame frame)
    { _fieldWords = frame.FieldWords; _rootWords = frame.RootWords; _valueWords = frame.ValueWords; }
    private Vector3d _normal, _meshOrigin, _center;
    private FixedQuaternion _meshRotation, _rotation;
    private PhysicsMesh? _gatherMesh;
    private MeshSurfaceConnectivity? _gatherConnectivity;
    private int _gatherSurface;
    private int _sourceRegion;
    private int _fixedOrientation;
    private bool _containsSection, _usesConvexFan;

    internal MeshCapsuleFaceRegions()
    {
        _visitor = AdmitPoint;
        _pointComparer = Comparer<Sample>.Create(ComparePoints);
    }

    internal int RegionCount { get; private set; }
    internal bool HasBoundaryIntersection { get; private set; }

    internal void Build(PhysicsMesh mesh, int surfaceTriangle, Vector3d center,
        FixedQuaternion rotation, Fixed64 coreLength, Fixed64 radius, MeshSurfaceConnectivity connectivity)
    {
        var finite = new CapsuleFiniteSectionFrame(mesh.GeometryFrame, center, rotation, coreLength, radius);
        Build(mesh, surfaceTriangle, finite, center, rotation, connectivity);
    }

    internal void Build(PhysicsMesh mesh, int surfaceTriangle, in CapsuleFiniteSectionFrame finite,
        Vector3d center, FixedQuaternion rotation, MeshSurfaceConnectivity connectivity)
    {
        if (_joined != null) _joined.Active = false;
        RegionCount = connectivity.RegionCount;
        HasBoundaryIntersection = false;
        if (RegionCount == 0) return;
        mesh.GetGeometryTriangleVertices(surfaceTriangle, out Vector3d a, out Vector3d b, out Vector3d c);
        _frame = new CapsulePlaneRayFrame(new FixedTriangle(a, b, c), mesh.GeometryFrame, finite);
        SelectBanks(_frame); Reserve(RegionCount);
        // Sphere and axial rays share one radial height H(q): their signed
        // exits are H(q) +/- the plane offset, so every region chooses alike.
        _fixedOrientation = finite.Length == Fixed64.Zero || _frame.Normal.X.IsZero && _frame.Normal.Z.IsZero
            ? (_frame.PlaneConstant.Sign >= 0 ? 1 : -1) : 0;
        _normal = _frame.GetWorldNormal(mesh.Rotation);
        _meshOrigin = _frame.Finite.ShapeFrame.HasExactScale ? center : mesh.Origin;
        _meshRotation = _frame.Finite.ShapeFrame.HasExactScale ? rotation : mesh.Rotation;
        _center = center; _rotation = rotation;
        Array.Clear(_winners, 0, 2 * RegionCount);
        _events.FastClear(); _points.FastClear(); _pointWordCount = 0;
        Gather(mesh, surfaceTriangle, connectivity);
        // Exact rigid transforms preserve the physical metric. Rounding once
        // in authored coordinates also keeps finite mesh anchors representable.
        var metrics = new WidePlaneMetrics(new FixedTriangle(a, b, c), FixedQuaternion.Identity);
        for (int region = 0; region < RegionCount; region++)
        {
            if (_fixedOrientation != 0) _orientations[region] = _fixedOrientation;
            else
            {
                var positive = BorrowWinner(2 * region);
                var negative = BorrowWinner(2 * region + 1);
                System.Diagnostics.Debug.Assert(positive.HasValue && negative.HasValue);
                _orientations[region] = positive.CompareDepth(negative) <= 0 ? 1 : -1;
            }
            ReduceSamples(region, metrics);
        }
    }

    private void Reserve(int count)
    {
        if (_counts.Length < count)
        {
            int capacity = Math.Max(count, 2 * _counts.Length);
            Array.Resize(ref _counts, capacity); Array.Resize(ref _orientations, capacity);
            Array.Resize(ref _samples, capacity * ContactManifold.MaxContactsPerGroup);
            Array.Resize(ref _winners, 2 * capacity);
            Array.Resize(ref _signs, 2 * capacity * CapsulePlaneRaySelection.SignCount);
            Array.Resize(ref _valueSigns, 2 * capacity * CapsulePlaneRaySelection.ValueSignCount);
        }
        if (_values.Length < 2 * _counts.Length * SelectionWords)
            Array.Resize(ref _values, 2 * _counts.Length * SelectionWords);
    }

    private void Gather(PhysicsMesh mesh, int surfaceTriangle, MeshSurfaceConnectivity connectivity)
    {
        _gatherMesh = mesh; _gatherConnectivity = connectivity; _gatherSurface = surfaceTriangle;
        try
        {
            ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
            ReadOnlySpan<Vector3d> vertices = mesh.GeometryVertices;
            int first = 0;
            for (; first < boundary.Length; first += 2)
                if (_frame.Finite.IntersectsSegment(new FixedSegment(vertices[boundary[first]],
                    vertices[boundary[first + 1]]))) break;
            HasBoundaryIntersection = first < boundary.Length;
            ReadOnlySpan<int> corners = mesh.GetConvexCoplanarPatchCornerVertexIndices(surfaceTriangle);
            // A connected convex section with an admitted point cannot leave
            // the filled domain without meeting its complete true boundary.
            _containsSection = RegionCount == 1 && !HasBoundaryIntersection;
            _usesConvexFan = !_containsSection && RegionCount == 1 && !corners.IsEmpty
                && corners.Length - 2 < connectivity.TriangleIndices.Length
                && connectivity.IsCompleteSingleRegion(mesh);
            if (_containsSection && _fixedOrientation != 0 && _frame.Finite.ShapeFrame.Radius.Sign > 0)
            {
                // A strictly curved contained section has one physical support;
                // off-peak rim rows would add artificial normal-impulse levers.
                int endpoint = _frame.Finite.Length == Fixed64.Zero ? 0
                    : _fixedOrientation * _frame.Normal.Y.Sign > 0 ? 1 : 0;
                var descriptor = new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Support, endpoint);
                Span<ulong> values = stackalloc ulong[SelectionWords];
                Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
                Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
                var support = new CapsulePlaneRaySelection(values, signs, valueSigns, _fieldWords, _rootWords, _valueWords);
                bool found = CapsulePlaneRayEvents.Reconstruct(CapsulePlaneRayEventSource.Plane,
                    _frame, descriptor, _fixedOrientation, ref support);
                System.Diagnostics.Debug.Assert(found);
                AdmitPoint(CapsulePlaneRayEventSource.Plane, _frame, descriptor, _fixedOrientation, support);
                return;
            }
            VisitSource(CapsulePlaneRayEventSource.Plane);
            for (int edge = first; edge < boundary.Length; edge += 2)
            {
                var segment = new FixedSegment(vertices[boundary[edge]], vertices[boundary[edge + 1]]);
                if (edge != first && !_frame.Finite.IntersectsSegment(segment)) continue;
                // A capsule intersects a straight canonical edge in one closed
                // interval; its admitted subedges therefore share one region.
                _sourceRegion = RegionCount == 1 ? 0 : MeshSurfaceBoundaryTopology.FindEdgeRegion(mesh,
                    mesh.GetManifoldSurfaceOwner(surfaceTriangle), boundary[edge], boundary[edge + 1],
                    connectivity, _frame.Finite);
                if (_sourceRegion < 0) continue;
                var source = new CapsulePlaneRayEventSource(segment);
                VisitSource(source);
            }
        }
        finally
        {
            _gatherMesh = null; _gatherConnectivity = null; _containsSection = false; _usesConvexFan = false;
        }
    }

    private void VisitSource(in CapsulePlaneRayEventSource source)
    {
        CapsulePlaneRayEvents.VisitEvents(source, _frame, _fixedOrientation == 0 ? 1 : _fixedOrientation, _visitor);
        if (_fixedOrientation == 0) CapsulePlaneRayEvents.VisitEvents(source, _frame, -1, _visitor);
    }

    private int FindRegion(scoped in CapsulePlaneRaySelection point)
    {
        if (_containsSection) return 0;
        PhysicsMesh mesh = _gatherMesh!;
        MeshSurfaceConnectivity connectivity = _gatherConnectivity!;
        ReadOnlySpan<int> corners = _usesConvexFan
            ? mesh.GetConvexCoplanarPatchCornerVertexIndices(_gatherSurface) : ReadOnlySpan<int>.Empty;
        if (!corners.IsEmpty)
        {
            ReadOnlySpan<Vector3d> vertices = mesh.GeometryVertices;
            for (int corner = 1; corner < corners.Length - 1; corner++)
                if (point.ContainsTrianglePoint(_frame, new FixedTriangle(vertices[corners[0]],
                    vertices[corners[corner]], vertices[corners[corner + 1]]))) return 0;
            return -1;
        }
        // Closed admitted intersections uniquely identify their connected region.
        foreach (int triangle in connectivity.TriangleIndices)
        {
            mesh.GetGeometryTriangleVertices(triangle, out Vector3d a, out Vector3d b, out Vector3d c);
            if (point.ContainsTrianglePoint(_frame, new FixedTriangle(a, b, c)))
                return connectivity.GetRegionOrdinal(triangle);
        }
        return -1;
    }

    private void AdmitPoint(in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame frame,
        CapsulePlaneRayEvent descriptor, int orientation, scoped in CapsulePlaneRaySelection candidate)
    {
        int region = source.IsSegment ? _sourceRegion : FindRegion(candidate);
        if (region < 0) return;
        int index = 2 * region + (orientation > 0 ? 0 : 1);
        var winner = BorrowWinner(index);
        winner.KeepEvaluated(candidate, frame); _winners[index] = winner.Metadata;
        // Opposite rays share the same exact base event. Retain both depth
        // winners, but materialize their common coverage point only once.
        foreach (Sample retained in _events.AsReadOnlySpan())
            if (retained.Region == region && retained.Source.IsSegment == source.IsSegment
                && retained.Source.A == source.A && retained.Source.B == source.B
                && retained.Event.Matches(descriptor)) return;
        bool represented = TryGetSamplingPoint(candidate, frame, out Vector3d p);
        // Admission makes q a convex combination of finite authored vertices.
        System.Diagnostics.Debug.Assert(represented);
        foreach (Sample retained in _events.AsReadOnlySpan())
            if (retained.Region == region && retained.Point == p && CompareToPoint(retained, candidate) == 0) return;
        _events.Add(new Sample(source, descriptor, region, p, RetainPoint(candidate)));
    }

    private bool TryGetSamplingPoint(scoped in CapsulePlaneRaySelection selection, in CapsulePlaneRayFrame frame, out Vector3d point) =>
        frame.Finite.ShapeFrame.HasExactScale
            ? selection.TryGetPointInRigidFrame(frame, _meshOrigin, _meshRotation, _center, _rotation, out point)
            : selection.TryGetAuthoredPoint(frame, _meshOrigin, _meshRotation, _center, _rotation, out point);

    private CapsulePlaneRaySelection BorrowWinner(int index)
    {
        var selection = new CapsulePlaneRaySelection(
            _values.AsSpan(index * SelectionWords, SelectionWords),
            _signs.AsSpan(index * CapsulePlaneRaySelection.SignCount, CapsulePlaneRaySelection.SignCount),
            _valueSigns.AsSpan(index * CapsulePlaneRaySelection.ValueSignCount, CapsulePlaneRaySelection.ValueSignCount), _fieldWords, _rootWords, _valueWords);
        selection.Restore(_winners[index]); return selection;
    }

    private void ReduceSamples(int region, WidePlaneMetrics metrics)
    {
        _pool.FastClear();
        var maximum = BorrowWinner(2 * region + (_orientations[region] > 0 ? 0 : 1));
        Sample seed = GetSeed(region, maximum);
        Vector3d p = seed.Point;
        _pool.Add(seed);
        foreach (Sample sample in _events.AsReadOnlySpan())
        {
            if (sample.Region != region || sample.Point == p && CompareToPoint(sample, maximum) == 0) continue;
            _pool.Add(sample);
        }
        int count = Math.Min(ContactManifold.MaxContactsPerGroup, _pool.Count);
        int offset = region * ContactManifold.MaxContactsPerGroup;
        _samples[offset] = seed;
        Span<int> chosen = stackalloc int[ContactManifold.MaxContactsPerGroup]; chosen[0] = 0;
        for (int slot = 1; slot < count; slot++)
        {
            int best = _frame.Finite.ShapeFrame.HasExactScale
                ? MeshPlaneSampleReduction.ChooseNext(_pool.AsReadOnlySpan(), chosen[..slot], _frame.Normal, _pointComparer)
                : MeshPlaneSampleReduction.ChooseNext(_pool.AsReadOnlySpan(), chosen[..slot], metrics, _pointComparer);
            chosen[slot] = best; _samples[offset + slot] = _pool[best];
        }
        _counts[region] = count;
    }

    private Sample GetSeed(int region, scoped in CapsulePlaneRaySelection maximum)
    {
        foreach (Sample sample in _events.AsReadOnlySpan())
            if (sample.Region == region && sample.Source.IsSegment == maximum.MaximumSource.IsSegment
                && sample.Source.A == maximum.MaximumSource.A && sample.Source.B == maximum.MaximumSource.B
                && sample.Event.Matches(maximum.MaximumEvent)) return sample;
        // Exact-point deduplication can retain an equivalent event descriptor.
        bool represented = TryGetSamplingPoint(maximum, _frame, out Vector3d p);
        System.Diagnostics.Debug.Assert(represented);
        return new Sample(maximum.MaximumSource, maximum.MaximumEvent, region, p, RetainPoint(maximum));
    }

    private int RetainPoint(scoped in CapsulePlaneRaySelection point)
    {
        // Retain q once; exact coverage ties must not replay its event algebra.
        int index = _points.Count;
        int words = Math.Max(Math.Max(ContactQuadratic.ActiveWords(point.X), ContactQuadratic.ActiveWords(point.Y)),
            Math.Max(ContactQuadratic.ActiveWords(point.Z), ContactQuadratic.ActiveWords(point.Denominator)));
        int rootWords = WideArithmetic.GetActiveMagnitudeLength(point.Root);
        int needed = _pointWordCount + 8 * words + rootWords;
        if (_pointValues.Length < needed) Array.Resize(ref _pointValues, Math.Max(needed, 2 * _pointValues.Length));
        if (_pointSigns.Length < (index + 1) * FiniteSectionPoint.SignCount)
        {
            Array.Resize(ref _pointSigns, Math.Max((index + 1) * FiniteSectionPoint.SignCount,
                Math.Max(8 * FiniteSectionPoint.SignCount, 2 * _pointSigns.Length)));
        }
        _points.Add(new PointStorage(_pointWordCount, words, rootWords)); _pointWordCount = needed;
        var retained = BorrowPoint(index);
        point.X.CopyTo(retained.Coordinate(0)); point.Y.CopyTo(retained.Coordinate(1));
        point.Z.CopyTo(retained.Coordinate(2)); point.Denominator.CopyTo(retained.Denominator);
        point.Root[..rootWords].CopyTo(retained.Root);
        return index;
    }

    private FiniteSectionPoint BorrowPoint(int index)
    {
        PointStorage storage = _points[index];
        return new(_pointValues.AsSpan(storage.Offset, 8 * storage.Words + storage.RootWords),
            _pointSigns.AsSpan(index * FiniteSectionPoint.SignCount, FiniteSectionPoint.SignCount),
            storage.Words, storage.RootWords);
    }

    private readonly struct PointStorage
    {
        internal readonly int Offset, Words, RootWords;
        internal PointStorage(int offset, int words, int rootWords)
        { Offset = offset; Words = words; RootWords = rootWords; }
    }

    private int CompareToPoint(Sample sample, scoped in CapsulePlaneRaySelection point)
    {
        var retained = BorrowPoint(sample.ExactPoint);
        return _frame.ComparePoints(retained.Coordinate(0), retained.Coordinate(1), retained.Coordinate(2),
            retained.Denominator, retained.Root, point.X, point.Y, point.Z, point.Denominator, point.Root);
    }

    private int ComparePoints(Sample first, Sample second)
    {
        int order = first.Point.X.CompareTo(second.Point.X);
        if (order != 0) return order;
        var a = BorrowPoint(first.ExactPoint);
        var b = BorrowPoint(second.ExactPoint);
        return _frame.ComparePoints(a.Coordinate(0), a.Coordinate(1), a.Coordinate(2), a.Denominator, a.Root,
            b.Coordinate(0), b.Coordinate(1), b.Coordinate(2), b.Denominator, b.Root);
    }

    internal Vector3d GetNormal(int region) => -_normal * GetOrientation(region);
    internal int GetOrientation(int region) => _orientations[ValidateRegion(region)];
    internal int GetSampleCount(int region) => _counts[ValidateRegion(region)];

    internal bool TryGetSample(int region, int index, out FixedContactAnchors contact)
    {
        contact = default;
        if ((uint)region >= (uint)RegionCount || (uint)index >= (uint)_counts[region]) return false;
        Sample sample = _samples[region * ContactManifold.MaxContactsPerGroup + index];
        if (index == 0)
        {
            var maximum = BorrowWinner(2 * region + (_orientations[region] > 0 ? 0 : 1));
            return TryCreateContact(region, sample.Point, maximum, out contact);
        }
        Span<ulong> values = stackalloc ulong[SelectionWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns, _fieldWords, _rootWords, _valueWords);
        bool found = CapsulePlaneRayEvents.Reconstruct(sample.Source, _frame, sample.Event, _orientations[region], ref selection);
        System.Diagnostics.Debug.Assert(found);
        return TryCreateContact(region, sample.Point, selection, out contact);
    }

    private bool TryCreateContact(int region, Vector3d point, scoped CapsulePlaneRaySelection selection,
        out FixedContactAnchors contact)
    {
        bool represented = selection.TryGetExitAnchor(_frame, _orientations[region], _center, _rotation,
            out FixedPointAnchor exit, out Fixed64 depth, out bool clamped);
        // Admission bounds the radial exit and finite core separately; the
        // anchor remains valid even when its absolute world point cannot fit.
        System.Diagnostics.Debug.Assert(represented);
        contact = new FixedContactAnchors(new FixedPointAnchor(_meshOrigin, _meshRotation, point),
            exit, GetNormal(region), depth, clamped);
        return represented;
    }

    private int ValidateRegion(int region)
    {
        if ((uint)region >= (uint)RegionCount) throw new ArgumentOutOfRangeException(nameof(region));
        return region;
    }

    private readonly struct Sample : IPlaneContactSample
    {
        internal readonly CapsulePlaneRayEventSource Source;
        internal readonly CapsulePlaneRayEvent Event;
        internal readonly int Region;
        internal readonly int ExactPoint;
        public Vector3d Point { get; }
        internal Sample(CapsulePlaneRayEventSource source, CapsulePlaneRayEvent descriptor, int region, Vector3d point, int exactPoint)
        { Source = source; Event = descriptor; Region = region; Point = point; ExactPoint = exactPoint; }
    }
}
