// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using SwiftCollections;
using System;

namespace Gravitas;

/// <content>Local filled-boundary contacts; member overlap remains independent of response ownership.</content>
internal sealed partial class StaticSurface2D
{
    private readonly SwiftList<LSCollider2D> _candidates = new(), _local = new(), _prepared = new();
    private readonly SwiftList<MeshSurfaceBoundaryDomain2D> _domains = new();
    private readonly SwiftList<(int Member, int Span)> _admitted = new();
    private readonly SwiftList<int> _fanMembers = new();
    private readonly SwiftList<int> _components = new();
    private readonly SwiftList<FixedPointAnchor2d> _fan = new();
    private readonly ContactManifold2D _contacts = new();
    private LSCollider2D? _moving, _carrier;
    private Vector2d _movingCenter;
    private Fixed64 _movingRotation;
    private uint _movingShape, _movingDependency;
    private bool _current;

    partial void InvalidateGeometry()
    {
        _current = false;
        _mixedCurrent = false;
        _mixedMoving = null;
        Array.Clear(_mixedLocal.InnerArray, 0, _mixedLocal.InnerArray.Length);
        _mixedLocal.FastClear();
        _moving = _carrier = null;
        // FastClear leaves reference tails beyond Count; ended member lifetimes
        // must be released even when this declaration becomes empty or is retained after disposal.
        Array.Clear(_candidates.InnerArray, 0, _candidates.InnerArray.Length);
        Array.Clear(_local.InnerArray, 0, _local.InnerArray.Length);
        Array.Clear(_prepared.InnerArray, 0, _prepared.InnerArray.Length);
        _candidates.FastClear();
        _local.FastClear();
        _prepared.FastClear();
    }

    internal void Reduce(LSCollider2D mover, LSCollider2D overlapMember, ContactManifold2D destination, bool reverse)
    {
        Gather(mover);
        bool same = _local.Count == _prepared.Count;
        for (int i = 0; same && i < _local.Count; i++) same = _local[i] == _prepared[i];
        if (!same) Prepare();
        if (!_current || _moving != mover || _movingCenter != mover.Center || _movingRotation != mover.Rotation
            || _movingShape != mover.RuntimeShapeVersion || _movingDependency != (mover.CompoundOwner ?? mover).ContactDependencyVersion)
        {
            Build(mover, overlapMember);
            _moving = mover; _movingCenter = mover.Center; _movingRotation = mover.Rotation;
            _movingShape = mover.RuntimeShapeVersion; _movingDependency = (mover.CompoundOwner ?? mover).ContactDependencyVersion;
            _current = true;
        }
        if (_carrier != overlapMember) return;
        if (!_contacts.HasContact) { destination.NeedsSurfaceRecovery = true; return; }
        if (!reverse) { destination.CopyFrom(_contacts, destination.LastUpdatedFrame); return; }
        for (int g = 0; g < _contacts.GroupCount; g++)
        {
            ref ContactGroup2D group = ref _contacts.GetGroup(g);
            for (int p = 0; p < group.Count; p++)
            {
                ManifoldContact2D row = group[p];
                destination.AddContact(row.AnchorB, row.AnchorA, row.Depth, -row.Normal,
                    row.MaterialB, row.MaterialA, row.DepthIsClamped, group: group.Key.Remap(0, 0, true));
            }
        }
    }

    private void Gather(LSCollider2D mover)
    {
        Fixed64 reach = mover.CanonicalCenteredProxyRadius * Fixed64.Two;
        FixedBoundArea bounds = FixedBoundArea.FromCenterAndScopeClippedToDomain(mover.Center, Vector2d.One * reach);
        _context.Collisions2D.CollectBoundsCandidates(bounds.Min, bounds.Max, PhysicsLayerMask.All,
            _context.Query2D.NextOverlapQueryVersion(), false, _candidates, staticStyleOnly: true);
        _local.FastClear();
        foreach (LSCollider2D candidate in _candidates.AsReadOnlySpan())
            if (candidate.StaticSurface == this && !candidate.IsTrigger
                && _context.Physics2D.RequireCollisionPair(mover.CompoundOwner ?? mover, candidate)) _local.Add(candidate);
    }

    private void Prepare()
    {
        _current = false;
        _prepared.FastClear();
        foreach (LSCollider2D member in _local.AsReadOnlySpan()) _prepared.Add(member);
        while (_domains.Count < _local.Count) _domains.Add(new MeshSurfaceBoundaryDomain2D());
        for (int i = 0; i < _local.Count; i++) _domains[i].Build(_local.AsReadOnlySpan(), i);
        _components.FastClear();
        for (int i = 0; i < _local.Count; i++) _components.Add(i);
        for (int i = 0; i < _local.Count; i++)
            foreach (var span in _domains[i].Spans)
                if (span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined)
                {
                    int from = _components[span.OtherMember], to = _components[i];
                    for (int j = 0; j < _components.Count; j++) if (_components[j] == from) _components[j] = to;
                }
    }

    private void Build(LSCollider2D mover, LSCollider2D knownOverlap)
    {
        _contacts.Reset(); _carrier = null; _admitted.FastClear();
        Span<FixedContactAnchors2d> rows = stackalloc FixedContactAnchors2d[2];
        Span<Vector2d> box = stackalloc Vector2d[4];
        for (int member = 0; member < _local.Count; member++)
        {
            LSCollider2D supplier = _local[member];
            if (_carrier == null && (supplier == knownOverlap || CollisionDetection2D.HasSurfaceOverlap(mover, supplier))) _carrier = supplier;
            ReadOnlySpan<MeshSurfaceBoundaryDomain2D.BoundarySpan> spans = _domains[member].Spans;
            for (int s = 0; s < spans.Length; s++)
            {
                if (spans[s].Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined) continue;
                int count = FaceContacts(mover, member, s, rows);
                if (count == 0) continue;
                _admitted.Add((member, s));
                for (int p = 0; p < count; p++) Add(mover, member, s, rows[p]);
            }
        }
        int previous;
        do
        {
            previous = _admitted.Count;
            for (int member = 0; member < _local.Count; member++)
                for (int s = 0; s < _domains[member].Spans.Length; s++)
                {
                    var face = _domains[member].Spans[s];
                    if (face.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined || _admitted.Contains((member, s)) || !PlaneAdmitted(member, s)) continue;
                    _admitted.Add((member, s));
                    int count = FaceContacts(mover, member, s, rows, planeAdmitted: true);
                    for (int p = 0; p < count; p++) Add(mover, member, s, rows[p]);
                }
        } while (_admitted.Count != previous);
        for (int member = 0; member < _local.Count; member++)
        {
            LSCollider2D supplier = _local[member];
            for (int v = 0; v < supplier.VertexCount; v++)
            {
                if (!BoundsOverlap(mover, _domains[member].VertexBounds[v])) continue;
                var vertex = new FixedPointAnchor2d(supplier.Center, supplier.ConvexRotation, supplier.GetScaledLocalVertexUnchecked(v));
                BuildFan(member, vertex);
                bool hit = mover is LSCircleCollider2D circle
                    ? WideCenteredCapsule2dRelations.TryGetBoundaryVertexContact(mover.Center, mover.Rotation, Fixed64.Zero,
                        circle.ScaledRadius, vertex, _fan.AsReadOnlySpan(), out rows[0])
                    : mover is LSCapsuleCollider2D capsule
                        ? WideCenteredCapsule2dRelations.TryGetBoundaryVertexContact(mover.Center, mover.Rotation, capsule.AxisLength,
                            capsule.ScaledRadius, vertex, _fan.AsReadOnlySpan(), out rows[0])
                        : WideConvex2dRelations.TryGetBoundaryVertexContact(mover.Center, mover.ConvexRotation,
                            Offsets(mover, box), vertex, _fan.AsReadOnlySpan(), out rows[0]);
                if (hit) Add(mover, member, v, rows[0], vertex: true);
            }
        }
    }

    private bool PlaneAdmitted(int member, int span)
    {
        LSCollider2D supplier = _local[member];
        var face = _domains[member].Spans[span];
        foreach (var admitted in _admitted.AsReadOnlySpan())
        {
            if (_components[member] != _components[admitted.Member]) continue;
            LSCollider2D other = _local[admitted.Member];
            var otherFace = _domains[admitted.Member].Spans[admitted.Span];
            // Meeting retained spans must also meet in their outward-rounded
            // whole-edge bounds; distant coplanar faces need no exact comparison.
            if (!face.Bounds.Intersects(otherFace.Bounds)) continue;
            if (WideConvex2dRelations.SameBoundaryPlane(supplier.Center, supplier.ConvexRotation,
                new FixedSegment2d(face.Start, face.End), supplier.GetScaledLocalVertexUnchecked((face.Second + 1) % supplier.VertexCount),
                other.Center, other.ConvexRotation, new FixedSegment2d(otherFace.Start, otherFace.End),
                other.GetScaledLocalVertexUnchecked((otherFace.Second + 1) % other.VertexCount))
                && SpansMeet(supplier, face, other, otherFace)) return true;
        }
        return false;
    }

    private static bool SpansMeet(LSCollider2D first, in MeshSurfaceBoundaryDomain2D.BoundarySpan a,
        LSCollider2D second, in MeshSurfaceBoundaryDomain2D.BoundarySpan b)
    {
        // Solid connectivity off this plane must not bridge a genuine boundary gap.
        Span<FixedPointAnchor2d> endpoints = stackalloc FixedPointAnchor2d[4]
        {
            new(first.Center, first.ConvexRotation, a.Start), new(first.Center, first.ConvexRotation, a.End),
            new(second.Center, second.ConvexRotation, b.Start), new(second.Center, second.ConvexRotation, b.End)
        };
        foreach (var point in endpoints)
            if (WideConvex2dRelations.LocateBoundaryPoint(point, first.Center, first.ConvexRotation,
                    new FixedSegment2d(a.Start, a.End), a.Interval) >= 0
                && WideConvex2dRelations.LocateBoundaryPoint(point, second.Center, second.ConvexRotation,
                    new FixedSegment2d(b.Start, b.End), b.Interval) >= 0) return true;
        return false;
    }

    private int FaceContacts(LSCollider2D mover, int member, int span, Span<FixedContactAnchors2d> rows, bool planeAdmitted = false)
    {
        LSCollider2D supplier = _local[member];
        var face = _domains[member].Spans[span];
        if (!planeAdmitted && !BoundsOverlap(mover, face.Bounds)) return 0;
        Vector2d inward = supplier.GetScaledLocalVertexUnchecked((face.Second + 1) % supplier.VertexCount);
        var edge = new FixedSegment2d(face.Start, face.End);
        Span<Vector2d> box = stackalloc Vector2d[4];
        return mover is LSCircleCollider2D circle
            ? WideConvex2dRelations.GetCapsuleBoundaryFaceContacts(mover.Center, mover.Rotation, Fixed64.Zero, circle.ScaledRadius,
                supplier.Center, supplier.ConvexRotation, edge, inward, face.Interval, rows, planeAdmitted)
            : mover is LSCapsuleCollider2D capsule
                ? WideConvex2dRelations.GetCapsuleBoundaryFaceContacts(mover.Center, mover.Rotation, capsule.AxisLength, capsule.ScaledRadius,
                    supplier.Center, supplier.ConvexRotation, edge, inward, face.Interval, rows, planeAdmitted)
                : WideConvex2dRelations.GetBoundaryFaceContacts(mover.Center, mover.ConvexRotation, Offsets(mover, box),
                    supplier.Center, supplier.ConvexRotation, edge, inward, face.Interval, rows, planeAdmitted);
    }

    private void BuildFan(int source, in FixedPointAnchor2d vertex)
    {
        _fan.FastClear(); _fanMembers.FastClear(); _fanMembers.Add(source);
        for (int cursor = 0; cursor < _fanMembers.Count; cursor++)
        {
            int member = _fanMembers[cursor];
            LSCollider2D supplier = _local[member];
            for (int v = 0; v < supplier.VertexCount; v++)
                _fan.Add(new FixedPointAnchor2d(supplier.Center, supplier.ConvexRotation, supplier.GetScaledLocalVertexUnchecked(v)));
            foreach (var span in _domains[member].Spans)
                if (span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined && !_fanMembers.Contains(span.OtherMember)
                    && WideConvex2dRelations.LocateBoundaryPoint(vertex, supplier.Center, supplier.ConvexRotation,
                        new FixedSegment2d(span.Start, span.End), span.Interval) >= 0) _fanMembers.Add(span.OtherMember);
        }
    }

    private void Add(LSCollider2D mover, int member, int feature, in FixedContactAnchors2d row, bool vertex = false)
    {
        LSCollider2D supplier = _local[member];
        _contacts.AddContact(new ContactAnchor2D(row.FirstAnchor), new ContactAnchor2D(row.SecondAnchor), row.Depth, row.Normal,
            mover.Material, supplier.Material, row.DepthIsClamped,
            group: new ContactGroupKey(0, 0, surfaceB: feature, region: vertex ? 1 : 0, sourceIdB: supplier.Id + 1));
    }

    private static ReadOnlySpan<Vector2d> Offsets(LSCollider2D collider, Span<Vector2d> box)
    {
        if (collider is LSPolygonCollider2D polygon) return polygon.ScaledLocalVertices;
        for (int i = 0; i < 4; i++) box[i] = collider.GetScaledLocalVertexUnchecked(i);
        return box;
    }

    // Outward-rounded native bounds reject only impossible finite crossings;
    // borrowed plane support deliberately bypasses this individual-face gate.
    private static bool BoundsOverlap(LSCollider2D mover, FixedBoundArea bounds) =>
        mover.MinX <= bounds.Max.X && mover.MaxX >= bounds.Min.X
        && mover.MinY <= bounds.Max.Y && mover.MaxY >= bounds.Min.Y;
}
