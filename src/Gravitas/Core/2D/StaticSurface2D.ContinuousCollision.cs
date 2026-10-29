// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;

namespace Gravitas;

/// <content>Shares exact boundary and declared-surface ownership with continuous collision.</content>
internal sealed partial class StaticSurface2D
{
    private readonly SwiftList<LSCollider2D> _continuousMixedMembers = new();
    private ContactManifold? _continuousMixedContacts;
    internal int ContinuousOrder => ((IPhysicsColliderRegistryItem)_members[0]).ReplayOrder;

    private static RationalPoint3d GetMixedContinuousSupport(LSCollider2D member, Vector3d outward)
    {
        // Handoff needs the exact global extremum in the native yaw chart;
        // a rounded inverse direction can select a lower support vertex.
        var frame = new NativePrismYawFrame3d(MixedEmbedded2DGeometry.GetCenter3D(member), member.ConvexRotation);
        Fixed64 cap = outward.Y < Fixed64.Zero ? -member.MixedHalfThickness : member.MixedHalfThickness;
        Vector2d vertex = member.GetScaledLocalVertexUnchecked(0);
        RationalPoint3d support = frame.GetPoint(new(vertex.X, cap, vertex.Y));
        for (int index = 1; index < member.VertexCount; index++)
        {
            vertex = member.GetScaledLocalVertexUnchecked(index);
            RationalPoint3d candidate = frame.GetPoint(new(vertex.X, cap, vertex.Y));
            if (RationalPoint3d.CompareProjection(candidate, support, outward) > 0) support = candidate;
        }
        return support;
    }

    internal bool IntersectsMixedContinuous(LSCollider mover, Fixed64 padding = default,
        Vector3d? separationOutward = null, RationalPoint3d separationPlane = default)
    {
        try
        {
            if (!GatherMixedContinuous(mover, padding)) return true;
            foreach (LSCollider2D member in _continuousMixedMembers.AsReadOnlySpan())
            {
                // Only a nonnegative full-member gap qualifies. A previously
                // handled penetration cannot license skipping another edge.
                if (separationOutward is { } axis && RationalPoint3d.CompareProjection(
                        GetMixedContinuousSupport(member, axis), separationPlane, axis) <= 0) continue;
                if (padding > Fixed64.Zero && ContinuousCollisionBounds.AreSeparated(mover, member, padding)) continue;
                if (CollisionDetectionMixed.IntersectsForContinuousCollision(mover, member, padding)) return true;
            }
            return false;
        }
        finally { _continuousMixedMembers.Clear(); }
    }

    internal void AppendMixedContinuousContacts(LSCollider mover, ContactManifold destination,
        Vector3d? separationOutward = null, RationalPoint3d separationPlane = default)
    {
        try
        {
            GatherMixedContinuous(mover, Fixed64.Zero);
            ContactManifold emitted = destination;
            if (separationOutward.HasValue)
            {
                emitted = _continuousMixedContacts ??= new();
                emitted.BeginUpdate(_context.FrameCount);
            }
            // Native emission regathers projected suppliers; keep the physical
            // carrier inventory separate while those existing owners run.
            foreach (LSCollider2D member in _continuousMixedMembers.AsReadOnlySpan())
                CollisionDetectionMixed.AppendManifold(mover, member, emitted, 0, 0, surfaceResponse: true);
            if (separationOutward is { } axis)
            {
                for (int groupIndex = 0; groupIndex < emitted.GroupCount; groupIndex++)
                {
                    ref ContactGroup group = ref emitted.GetGroup(groupIndex);
                    // A carrier may borrow an unsolved supplier's wall row.
                    // Filter by actual provenance only after native emission.
                    if (_context.Physics2D.TryGetColliderById(group.Key.SourceIdB - 1, out var supplier)
                        && supplier!.StaticSurface == this && RationalPoint3d.CompareProjection(
                            GetMixedContinuousSupport(supplier, axis), separationPlane, axis) <= 0) continue;
                    for (int row = 0; row < group.Count; row++) destination.AddContact(group.Key, group[row]);
                }
            }
        }
        finally { _continuousMixedMembers.Clear(); _continuousMixedContacts?.Reset(); }
    }

    private bool GatherMixedContinuous(LSCollider mover, Fixed64 padding)
    {
        _continuousMixedMembers.Clear();
        FixedBoundBox bounds = mover.Bounds;
        if (padding > Fixed64.Zero)
        {
            Vector3d expansion = Vector3d.One * padding;
            if (!Vector3d.TrySubtract(bounds.Min, expansion, out Vector3d minimum)
                || !Vector3d.TryAdd(bounds.Max, expansion, out Vector3d maximum)) return false;
            bounds = FixedBoundBox.FromMinMax(minimum, maximum);
        }
        GatherMixedMembers(mover, bounds, _continuousMixedMembers);
        return true;
    }

    internal bool AdmitContinuousWitness(LSCollider2D mover, LSCollider2D target,
        in ContactAnchor2D witness)
    {
        Gather(mover);
        bool same = _local.Count == _prepared.Count;
        for (int i = 0; same && i < _local.Count; i++) same = _local[i] == _prepared[i];
        if (!same) Prepare();
        for (int member = 0; member < _local.Count; member++)
        {
            if (_local[member] != target) continue;
            foreach (var span in _domains[member].Spans)
                if (WideConvex2dRelations.LocateBoundaryPoint(witness.GeometryPoint, target.Center,
                    target.ConvexRotation, new FixedSegment2d(span.Start, span.End), span.Interval) >= 0)
                {
                    BuildFan(member, witness.GeometryPoint);
                    if (mover is not (LSCircleCollider2D or LSCapsuleCollider2D)) return true;
                    WideDirection3d axis = WideCenteredCapsule2dRelations.GetPointToCoreDirection(mover.Center, mover.Rotation,
                        mover is LSCapsuleCollider2D capsule ? capsule.AxisLength : Fixed64.Zero, witness.GeometryPoint);
                    // A rounded witness must still support its own finite polygon
                    // before its radial direction can exclude an incident neighbor.
                    if (!WideConvex2dRelations.SupportsBoundaryFan(axis, witness.GeometryPoint,
                            _fan.AsReadOnlySpan()[..target.VertexCount])) return true;
                    return WideConvex2dRelations.SupportsBoundaryFan(axis, witness.GeometryPoint, _fan.AsReadOnlySpan());
                }
        }
        return true;
    }

    internal bool AdmitMixedContinuousWitness(LSCollider mover, Vector3d proxyCenter, LSCollider2D target,
        in ContactAnchor witness)
    {
        // The caller proves an exact sphere projection of a sphere/capsule;
        // these native rounded sources always use the scalar-yaw prism chart.
        System.Diagnostics.Debug.Assert(mover is LSSphereCollider or LSCapsuleCollider);
        GatherMixed(mover, mover.CanonicalCenteredProxyRadius * Fixed64.Two);
        RationalPoint3d point = RationalPoint3d.FromAnchor(witness.GeometryPoint);
        WideDirection3d axis = WideTriangleRelations.GetScaledPointDifference(RationalPoint3d.FromScalarPoint(proxyCenter),
            point, WideArithmetic.TruncateSigned320By64(point.Denominator));
        RationalPoint3d footprint = new(point.X, default, point.Z, point.Denominator);
        int source = -1;
        for (int member = 0; member < _mixedLocal.Count; member++)
            if (_mixedLocal[member] == target) source = member;
        if (source < 0) return true;
        if (RationalPoint3d.CompareProjection(point, RationalPoint3d.FromScalarPoint(target.MixedBounds3D.Min), Vector3d.Up) < 0
            || RationalPoint3d.CompareProjection(point, RationalPoint3d.FromScalarPoint(target.MixedBounds3D.Max), Vector3d.Up) > 0) return true;
        bool onBoundary = false;
        foreach (var span in _mixedPrisms[source].Bands[0].Boundary.Spans)
            onBoundary |= Locate(source, span) >= 0;
        if (!onBoundary) return true;
        _mixedFan.FastClear(); _mixedFan.Add(source);
        for (int cursor = 0; cursor < _mixedFan.Count; cursor++)
        {
            int member = _mixedFan[cursor]; MixedPrism prism = _mixedPrisms[member];
            // Own-prism failure leaves the rounded witness unproved; only a
            // neighbor can exclude a certified radial support at the join.
            foreach (Vector3d vertex in prism.Points)
            {
                RationalPoint3d incident = prism.NativeFrame.GetPoint(vertex);
                if (WideDirection3d.Dot(axis, WideTriangleRelations.GetScaledPointDifference(incident, point,
                    WideArithmetic.TruncateSigned320By64(point.Denominator))).Sign > 0) return cursor == 0;
            }
            for (int bandIndex = 0; bandIndex < prism.BandCount; bandIndex++)
            {
                MixedBand band = prism.Bands[bandIndex];
                Vector3d lower = new(Fixed64.Zero, band.Lower, Fixed64.Zero), upper = new(Fixed64.Zero, band.Upper, Fixed64.Zero);
                RationalPoint3d a = prism.NativeFrame.GetPoint(lower), b = prism.NativeFrame.GetPoint(upper);
                if (RationalPoint3d.CompareProjection(point, a, Vector3d.Up) < 0
                    || RationalPoint3d.CompareProjection(point, b, Vector3d.Up) > 0) continue;
                foreach (var span in band.Boundary.Spans)
                    if (span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined && !_mixedFan.Contains(span.OtherMember)
                        && Locate(member, span) >= 0) _mixedFan.Add(span.OtherMember);
            }
        }
        return true;

        int Locate(int member, MeshSurfaceBoundaryDomain2D.BoundarySpan span)
        {
            MixedPrism prism = _mixedPrisms[member];
            FixedSegment edge = new(new(span.Start.X, Fixed64.Zero, span.Start.Y), new(span.End.X, Fixed64.Zero, span.End.Y));
            return WideTriangleRelations.LocatePointOnFrameEdge(footprint, edge, prism.NativeFootFrame, span.Interval);
        }
    }
}
