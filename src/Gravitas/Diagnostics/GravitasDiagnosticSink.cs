//=======================================================================
// GravitasDiagnosticSink.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using SwiftCollections;
using System;

namespace Gravitas.Diagnostics;

/// <summary>
/// Context-owned deterministic diagnostics buffer for physics events and debug draw commands.
/// </summary>
public sealed class GravitasDiagnosticSink
{
    private readonly GravitasWorldContext _context;
    private readonly SwiftList<GravitasDiagnosticEvent> _events = new();
    private readonly SwiftList<GravitasDebugDrawCommand> _drawCommands = new();
    private int _eventSequence;
    private int _drawSequence;

    internal GravitasDiagnosticSink(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    /// <summary>
    /// Gets whether this sink records events and draw commands.
    /// </summary>
    public bool Enabled { get; private set; }

    /// <summary>
    /// Gets captured diagnostic events for this context.
    /// </summary>
    public ReadOnlySpan<GravitasDiagnosticEvent> Events => _events.AsReadOnlySpan();

    /// <summary>
    /// Gets captured engine-agnostic draw commands for this context.
    /// </summary>
    public ReadOnlySpan<GravitasDebugDrawCommand> DrawCommands => _drawCommands.AsReadOnlySpan();

    /// <summary>
    /// Dispatches captured events in buffer order to a typed diagnostic visitor.
    /// </summary>
    public void DispatchEventsTo(GravitasDiagnosticEventVisitor visitor)
    {
        if (visitor == null)
            throw new ArgumentNullException(nameof(visitor));

        ReadOnlySpan<GravitasDiagnosticEvent> events = Events;
        for (int i = 0; i < events.Length; i++)
            events[i].DispatchTo(visitor);
    }

    /// <summary>
    /// Dispatches captured draw commands in buffer order to a typed debug draw visitor.
    /// </summary>
    public void DispatchDrawCommandsTo(GravitasDebugDrawCommandVisitor visitor)
    {
        if (visitor == null)
            throw new ArgumentNullException(nameof(visitor));

        ReadOnlySpan<GravitasDebugDrawCommand> commands = DrawCommands;
        for (int i = 0; i < commands.Length; i++)
            commands[i].DispatchTo(visitor);
    }

    public int EventCount => _events.Count;

    public int DrawCommandCount => _drawCommands.Count;

    internal int EventCapacity => _events.Capacity;

    internal int DrawCommandCapacity => _drawCommands.Capacity;

    /// <summary>
    /// Enables diagnostics and optionally reserves buffer capacity for hot-path capture.
    /// </summary>
    public void Enable(int eventCapacity = 128, int drawCommandCapacity = 128)
    {
        Enabled = true;
        if (eventCapacity > 0)
            _events.EnsureCapacity(eventCapacity);
        if (drawCommandCapacity > 0)
            _drawCommands.EnsureCapacity(drawCommandCapacity);
    }

    /// <summary>
    /// Disables diagnostics and clears captured data.
    /// </summary>
    public void Disable()
    {
        Enabled = false;
        Clear();
    }

    /// <summary>
    /// Clears captured data while retaining allocated buffer capacity.
    /// </summary>
    public void Clear()
    {
        _events.FastClear();
        _drawCommands.FastClear();
        _eventSequence = 0;
        _drawSequence = 0;
    }

    /// <summary>
    /// Emits an engine-agnostic draw command for the supplied collider shape.
    /// </summary>
    public void CaptureCollider(LSCollider collider, GravitasDiagnosticColor color)
    {
        if (!Enabled)
            return;

        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        switch (collider)
        {
            case LSSphereCollider sphere:
                AddDrawCommand(
                    GravitasDebugDrawKind.WireSphere,
                    sphere.Id,
                    sphere.Shape,
                    center: sphere.Center,
                    radius: sphere.ScaledRadius,
                    color: color);
                break;
            case LSCapsuleCollider capsule:
                AddDrawCommand(
                    GravitasDebugDrawKind.WireCapsule,
                    capsule.Id,
                    capsule.Shape,
                    center: capsule.Center,
                    rotation: capsule.Rotation,
                    radius: capsule.ScaledRadius,
                    height: capsule.ScaledSize.Y,
                    color: color);
                break;
            case LSCuboidCollider cuboid:
                AddDrawCommand(
                    GravitasDebugDrawKind.WireBox,
                    cuboid.Id,
                    cuboid.Shape,
                    center: cuboid.Center,
                    size: cuboid.ScaledSize,
                    rotation: cuboid.Rotation,
                    color: color);
                break;
            case LSCylinderCollider cylinder:
                AddDrawCommand(
                    GravitasDebugDrawKind.WireCylinder,
                    cylinder.Id,
                    cylinder.Shape,
                    center: cylinder.Center,
                    rotation: cylinder.Rotation,
                    radius: cylinder.ScaledRadius,
                    height: cylinder.Height,
                    color: color);
                break;
            case LSCompoundCollider compound:
                CaptureCompoundParts(compound, color);
                break;
            case LSMeshCollider mesh:
                CaptureMeshTriangles(mesh, color);
                break;
        }
    }

    /// <summary>
    /// Emits engine-agnostic draw commands for the finite 2D slab used by mixed 2D/3D collision.
    /// </summary>
    public void CaptureMixedCollider(LSCollider2D collider, GravitasDiagnosticColor color)
    {
        if (!Enabled)
            return;

        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        Vector3d center = new(collider.Center.X, collider.MixedSlabCenterY, collider.Center.Y);
        Fixed64 height = collider.MixedHalfThickness * 2;
        switch (collider)
        {
            case LSCircleCollider2D circle:
                AddDrawCommand(
                    GravitasDebugDrawKind.WireCylinder,
                    circle.Id,
                    colliderDimension: GravitasColliderDimension.TwoD,
                    collider2DType: circle.Shape,
                    center: center,
                    radius: circle.ScaledRadius,
                    height: height,
                    color: color);
                break;
            case LSAABBoxCollider2D box:
                AddDrawCommand(
                    GravitasDebugDrawKind.WireBox,
                    box.Id,
                    colliderDimension: GravitasColliderDimension.TwoD,
                    collider2DType: box.Shape,
                    center: center,
                    size: new Vector3d(box.ScaledSize.X, height, box.ScaledSize.Y),
                    color: color);
                break;
            case LSCompoundCollider2D compound:
                CaptureMixedCompoundParts(compound, color);
                break;
            default:
                CaptureMixedPolygon(collider, color);
                break;
        }
    }

    /// <summary>
    /// Emits an engine-agnostic line draw command.
    /// </summary>
    public void CaptureLine(Vector3d start, Vector3d end, GravitasDiagnosticColor color)
    {
        if (!Enabled)
            return;

        AddDrawCommand(
            GravitasDebugDrawKind.Line,
            start: start,
            end: end,
            color: color);
    }

    /// <summary>
    /// Emits an engine-agnostic ray draw command from an origin, direction, and length.
    /// </summary>
    public void CaptureRay(Vector3d origin, Vector3d direction, Fixed64 length, GravitasDiagnosticColor color)
    {
        if (!Enabled)
            return;

        Vector3d end = direction.MagnitudeSquared == Fixed64.Zero
            ? origin
            : origin + direction.Normalized * length;

        AddDrawCommand(
            GravitasDebugDrawKind.Ray,
            start: origin,
            end: end,
            color: color);
    }

    /// <summary>
    /// Emits an engine-agnostic point draw command.
    /// </summary>
    public void CapturePoint(Vector3d point, Fixed64 radius, GravitasDiagnosticColor color)
    {
        if (!Enabled)
            return;

        AddDrawCommand(
            GravitasDebugDrawKind.Point,
            center: point,
            radius: radius,
            color: color);
    }

    internal void Reset() => Clear();

    internal void EmitForceDelta(SolidBody body, Vector3d force, Vector3d accelerationDelta)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.ForceDelta,
            bodyId: body.DynamicId,
            colliderAId: body.Collider.Id,
            colliderAType: body.Collider.Shape,
            vector: force,
            pointA: accelerationDelta,
            scalarA: force.Magnitude);
    }

    internal void EmitTorqueDelta(SolidBody body, Vector3d torque)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.TorqueDelta,
            bodyId: body.DynamicId,
            colliderAId: body.Collider.Id,
            colliderAType: body.Collider.Shape,
            vector: torque,
            scalarA: torque.Magnitude);
    }

    internal void EmitLinearVelocityDelta(SolidBody body, Vector3d before, Vector3d after)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.LinearVelocityDelta,
            bodyId: body.DynamicId,
            colliderAId: body.Collider.Id,
            colliderAType: body.Collider.Shape,
            start: before,
            end: after,
            vector: after - before,
            scalarA: after.Magnitude);
    }

    internal void EmitAngularVelocityDelta(SolidBody body, Vector3d before, Vector3d after)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.AngularVelocityDelta,
            bodyId: body.DynamicId,
            colliderAId: body.Collider.Id,
            colliderAType: body.Collider.Shape,
            start: before,
            end: after,
            vector: after - before,
            scalarA: after.Magnitude);
    }

    internal void EmitGroundProbe(
        SolidBody body,
        GroundProbeMode mode,
        Vector3d origin,
        Vector3d end,
        Fixed64 radius,
        bool hit,
        Physics3DHit raycastHit)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.GroundProbe,
            bodyId: body.DynamicId,
            colliderAId: body.Collider.Id,
            colliderBId: raycastHit.Collider?.Id ?? -1,
            colliderAType: body.Collider.Shape,
            colliderBType: raycastHit.Collider?.Shape ?? ColliderType.None,
            start: origin,
            end: end,
            pointA: raycastHit.Point,
            vector: raycastHit.Normal,
            scalarA: radius,
            scalarB: raycastHit.Distance,
            dataA: (int)mode,
            hit: hit);
    }

    internal void EmitGroundProbe(
        SolidBody2D body,
        GroundProbeMode2D mode,
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        bool hit,
        Physics2DHit raycastHit)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.GroundProbe,
            bodyId: body.DynamicId,
            colliderAId: body.Collider.Id,
            colliderBId: raycastHit.Collider?.Id ?? -1,
            colliderADimension: GravitasColliderDimension.TwoD,
            colliderBDimension: raycastHit.Collider == null ? GravitasColliderDimension.None : GravitasColliderDimension.TwoD,
            colliderA2DType: body.Collider.Shape,
            colliderB2DType: raycastHit.Collider?.Shape ?? ColliderType2D.None,
            start: ToDiagnosticVector(start),
            end: ToDiagnosticVector(end),
            pointA: ToDiagnosticVector(raycastHit.Point),
            vector: ToDiagnosticVector(raycastHit.Normal),
            scalarA: radius,
            scalarB: raycastHit.Distance,
            dataA: (int)mode,
            dataB: (int)GravitasColliderDimension.TwoD,
            hit: hit);
    }

    internal void EmitRayQuery(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        int layerMaskBits,
        bool hit,
        int hitCount,
        Physics3DHit raycastHit)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.RayQuery,
            colliderAId: raycastHit.Collider?.Id ?? -1,
            colliderAType: raycastHit.Collider?.Shape ?? ColliderType.None,
            start: start,
            end: end,
            pointA: raycastHit.Point,
            vector: raycastHit.Normal,
            scalarA: radius,
            scalarB: raycastHit.Distance,
            dataA: layerMaskBits,
            dataB: hitCount,
            hit: hit);
    }

    internal void EmitCircleQuery(
        Vector3d center,
        Fixed64 radius,
        Vector3d direction,
        Fixed64 maxDistance,
        int layerMaskBits,
        bool hit,
        int hitCount,
        Physics3DHit raycastHit)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.CircleQuery,
            colliderAId: raycastHit.Collider?.Id ?? -1,
            colliderAType: raycastHit.Collider?.Shape ?? ColliderType.None,
            start: center,
            end: center + direction * maxDistance,
            pointA: raycastHit.Point,
            vector: direction,
            scalarA: radius,
            scalarB: raycastHit.Distance,
            dataA: layerMaskBits,
            dataB: hitCount,
            hit: hit);
    }

    internal void EmitQuerySummary(
        GravitasColliderDimension sourceDimension,
        GravitasColliderDimension targetDimension,
        Vector3d start,
        Vector3d end,
        int exactReducerAttempts,
        int acceptedHits,
        int fallbackHits,
        int rejectedConservativeCandidates)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.QuerySummary,
            colliderADimension: sourceDimension,
            colliderBDimension: targetDimension,
            start: start,
            end: end,
            scalarA: (Fixed64)fallbackHits,
            scalarB: (Fixed64)rejectedConservativeCandidates,
            dataA: exactReducerAttempts,
            dataB: acceptedHits,
            hit: fallbackHits > 0 || rejectedConservativeCandidates > 0);
    }

    internal void EmitContact(CollisionPair pair, bool hit)
    {
        if (!Enabled)
            return;

        ManifoldContact contact = pair.Manifold.HasContact
            ? pair.Manifold.PrimaryContact
            : default;

        AddEvent(
            GravitasDiagnosticEventKind.Contact,
            colliderAId: pair.ColliderA.Id,
            colliderBId: pair.ColliderB.Id,
            colliderAType: pair.ColliderA.Shape,
            colliderBType: pair.ColliderB.Shape,
            pointA: contact.PointA,
            pointB: contact.PointB,
            vector: contact.Normal,
            scalarA: contact.Depth,
            dataA: pair.Manifold.Count,
            hit: hit && pair.Manifold.HasContact);
    }

    internal void EmitResponseImpulse(CollisionPair pair, Vector3d impulse, Fixed64 normalVelocity)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.ResponseImpulse,
            colliderAId: pair.ColliderA.Id,
            colliderBId: pair.ColliderB.Id,
            colliderAType: pair.ColliderA.Shape,
            colliderBType: pair.ColliderB.Shape,
            pointA: pair.Manifold.PrimaryContact.PointA,
            pointB: pair.Manifold.PrimaryContact.PointB,
            vector: impulse,
            scalarA: impulse.Magnitude,
            scalarB: normalVelocity,
            hit: true);
    }

    internal void EmitMixedQuery(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        int layerMaskBits,
        bool hit,
        int hitCount,
        PhysicsMixedHit mixedHit)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.MixedQuery,
            colliderAId: mixedHit.Collider3D?.Id ?? -1,
            colliderBId: mixedHit.Collider2D?.Id ?? -1,
            colliderADimension: mixedHit.Collider3D == null ? GravitasColliderDimension.None : GravitasColliderDimension.ThreeD,
            colliderBDimension: mixedHit.Collider2D == null ? GravitasColliderDimension.None : GravitasColliderDimension.TwoD,
            colliderAType: mixedHit.Collider3D?.Shape ?? ColliderType.None,
            colliderB2DType: mixedHit.Collider2D?.Shape ?? ColliderType2D.None,
            start: start,
            end: end,
            pointA: mixedHit.Point3D,
            pointB: mixedHit.Point2D,
            vector: mixedHit.Normal3DTo2D,
            scalarA: radius,
            scalarB: mixedHit.Distance,
            dataA: layerMaskBits,
            dataB: hitCount,
            hit: hit);
    }

    internal void EmitMixedContact(CollisionPairMixed pair, MixedContact contact, bool hit)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.MixedContact,
            colliderAId: pair.Collider3DId,
            colliderBId: pair.Collider2DId,
            colliderADimension: GravitasColliderDimension.ThreeD,
            colliderBDimension: GravitasColliderDimension.TwoD,
            colliderAType: pair.Collider3D.Shape,
            colliderB2DType: pair.Collider2D.Shape,
            pointA: contact.Point3D,
            pointB: contact.Point2D,
            vector: contact.Normal3DTo2D,
            scalarA: contact.Depth,
            hit: hit && contact.HasContact);
    }

    internal void EmitMixedResponseImpulse(
        CollisionPairMixed pair,
        MixedContact contact,
        Vector3d impulse,
        Fixed64 normalVelocity,
        int iteration,
        int iterationLimit)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.MixedResponseImpulse,
            colliderAId: pair.Collider3DId,
            colliderBId: pair.Collider2DId,
            colliderADimension: GravitasColliderDimension.ThreeD,
            colliderBDimension: GravitasColliderDimension.TwoD,
            colliderAType: pair.Collider3D.Shape,
            colliderB2DType: pair.Collider2D.Shape,
            pointA: contact.Point3D,
            pointB: contact.Point2D,
            vector: impulse,
            scalarA: impulse.Magnitude,
            scalarB: normalVelocity,
            dataA: iteration,
            dataB: iterationLimit,
            hit: true);
    }

    internal void EmitMixedResponseIsland(
        int rootKey,
        int constraintCount,
        int iterationCount,
        bool reachedIterationLimit)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.MixedResponseIsland,
            bodyId: rootKey,
            dataA: constraintCount,
            dataB: iterationCount,
            hit: reachedIterationLimit);
    }

    private void CaptureMeshTriangles(LSMeshCollider mesh, GravitasDiagnosticColor color)
    {
        int triangleCount = mesh.Mesh.TriangleCount;
        for (int i = 0; i < triangleCount; i++)
        {
            mesh.Mesh.GetTriangleVertices(i, out Vector3d first, out Vector3d second, out Vector3d third);
            AddDrawCommand(
                GravitasDebugDrawKind.WireTriangle,
                mesh.Id,
                mesh.Shape,
                pointA: first,
                pointB: second,
                pointC: third,
                color: color);
        }
    }

    private void CaptureCompoundParts(LSCompoundCollider compound, GravitasDiagnosticColor color)
    {
        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider part = compound.GetPartCollider(i);
            switch (part)
            {
                case LSSphereCollider sphere:
                    AddDrawCommand(
                        GravitasDebugDrawKind.WireSphere,
                        compound.Id,
                        compound.Shape,
                        center: sphere.Center,
                        radius: sphere.ScaledRadius,
                        color: color);
                    break;
                case LSCapsuleCollider capsule:
                    AddDrawCommand(
                        GravitasDebugDrawKind.WireCapsule,
                        compound.Id,
                        compound.Shape,
                        center: capsule.Center,
                        rotation: capsule.Rotation,
                        radius: capsule.ScaledRadius,
                        height: capsule.ScaledSize.Y,
                        color: color);
                    break;
                case LSCuboidCollider cuboid:
                    AddDrawCommand(
                        GravitasDebugDrawKind.WireBox,
                        compound.Id,
                        compound.Shape,
                        center: cuboid.Center,
                        size: cuboid.ScaledSize,
                        rotation: cuboid.Rotation,
                        color: color);
                    break;
                case LSCylinderCollider cylinder:
                    AddDrawCommand(
                        GravitasDebugDrawKind.WireCylinder,
                        compound.Id,
                        compound.Shape,
                        center: cylinder.Center,
                        rotation: cylinder.Rotation,
                        radius: cylinder.ScaledRadius,
                        height: cylinder.Height,
                        color: color);
                    break;
                case LSMeshCollider mesh:
                    CaptureCompoundMeshTriangles(compound, mesh, color);
                    break;
            }
        }
    }

    private void CaptureCompoundMeshTriangles(
        LSCompoundCollider compound,
        LSMeshCollider mesh,
        GravitasDiagnosticColor color)
    {
        int triangleCount = mesh.Mesh.TriangleCount;
        for (int i = 0; i < triangleCount; i++)
        {
            mesh.Mesh.GetTriangleVertices(i, out Vector3d first, out Vector3d second, out Vector3d third);
            AddDrawCommand(
                GravitasDebugDrawKind.WireTriangle,
                compound.Id,
                compound.Shape,
                pointA: first,
                pointB: second,
                pointC: third,
                color: color);
        }
    }

    private void CaptureMixedCompoundParts(LSCompoundCollider2D compound, GravitasDiagnosticColor color)
    {
        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            Vector3d center = new(part.Center.X, part.MixedSlabCenterY, part.Center.Y);
            Fixed64 height = part.MixedHalfThickness * 2;
            switch (part)
            {
                case LSCircleCollider2D circle:
                    AddDrawCommand(
                        GravitasDebugDrawKind.WireCylinder,
                        compound.Id,
                        colliderDimension: GravitasColliderDimension.TwoD,
                        collider2DType: compound.Shape,
                        center: center,
                        radius: circle.ScaledRadius,
                        height: height,
                        color: color);
                    break;
                case LSAABBoxCollider2D box:
                    AddDrawCommand(
                        GravitasDebugDrawKind.WireBox,
                        compound.Id,
                        colliderDimension: GravitasColliderDimension.TwoD,
                        collider2DType: compound.Shape,
                        center: center,
                        size: new Vector3d(box.ScaledSize.X, height, box.ScaledSize.Y),
                        color: color);
                    break;
                default:
                    CaptureMixedPolygon(part, color, compound.Id, compound.Shape);
                    break;
            }
        }
    }

    private void CaptureMixedPolygon(LSCollider2D collider, GravitasDiagnosticColor color) =>
        CaptureMixedPolygon(collider, color, collider.Id, collider.Shape);

    private void CaptureMixedPolygon(
        LSCollider2D collider,
        GravitasDiagnosticColor color,
        int colliderId,
        ColliderType2D colliderType)
    {
        int vertexCount = collider.VertexCount;
        if (vertexCount <= 0)
            return;

        Fixed64 topY = collider.MixedSlabCenterY + collider.MixedHalfThickness;
        Fixed64 bottomY = collider.MixedSlabCenterY - collider.MixedHalfThickness;
        for (int i = 0; i < vertexCount; i++)
        {
            Vector2d current = collider.GetVertexUnchecked(i);
            Vector2d next = collider.GetVertexUnchecked((i + 1) % vertexCount);
            Vector3d currentTop = new(current.X, topY, current.Y);
            Vector3d nextTop = new(next.X, topY, next.Y);
            Vector3d currentBottom = new(current.X, bottomY, current.Y);
            Vector3d nextBottom = new(next.X, bottomY, next.Y);

            AddDrawCommand(
                GravitasDebugDrawKind.Line,
                colliderId,
                colliderDimension: GravitasColliderDimension.TwoD,
                collider2DType: colliderType,
                start: currentTop,
                end: nextTop,
                color: color);
            AddDrawCommand(
                GravitasDebugDrawKind.Line,
                colliderId,
                colliderDimension: GravitasColliderDimension.TwoD,
                collider2DType: colliderType,
                start: currentBottom,
                end: nextBottom,
                color: color);
            AddDrawCommand(
                GravitasDebugDrawKind.Line,
                colliderId,
                colliderDimension: GravitasColliderDimension.TwoD,
                collider2DType: colliderType,
                start: currentTop,
                end: currentBottom,
                color: color);
        }
    }

    private void AddEvent(
        GravitasDiagnosticEventKind kind,
        int bodyId = -1,
        int colliderAId = -1,
        int colliderBId = -1,
        GravitasColliderDimension colliderADimension = GravitasColliderDimension.None,
        GravitasColliderDimension colliderBDimension = GravitasColliderDimension.None,
        ColliderType colliderAType = ColliderType.None,
        ColliderType colliderBType = ColliderType.None,
        ColliderType2D colliderA2DType = ColliderType2D.None,
        ColliderType2D colliderB2DType = ColliderType2D.None,
        Vector3d start = default,
        Vector3d end = default,
        Vector3d pointA = default,
        Vector3d pointB = default,
        Vector3d vector = default,
        Fixed64 scalarA = default,
        Fixed64 scalarB = default,
        int dataA = 0,
        int dataB = 0,
        bool hit = false)
    {
        _events.Add(new GravitasDiagnosticEvent(
            _context.FrameCount,
            _eventSequence++,
            kind,
            bodyId,
            colliderAId,
            colliderBId,
            ResolveDimension(colliderADimension, colliderAType, colliderA2DType),
            ResolveDimension(colliderBDimension, colliderBType, colliderB2DType),
            colliderAType,
            colliderBType,
            colliderA2DType,
            colliderB2DType,
            start,
            end,
            pointA,
            pointB,
            vector,
            scalarA,
            scalarB,
            dataA,
            dataB,
            hit));
    }

    private void AddDrawCommand(
        GravitasDebugDrawKind kind,
        int colliderId = -1,
        ColliderType colliderType = ColliderType.None,
        GravitasColliderDimension colliderDimension = GravitasColliderDimension.None,
        ColliderType2D collider2DType = ColliderType2D.None,
        Vector3d start = default,
        Vector3d end = default,
        Vector3d center = default,
        Vector3d size = default,
        Vector3d pointA = default,
        Vector3d pointB = default,
        Vector3d pointC = default,
        FixedQuaternion rotation = default,
        Fixed64 radius = default,
        Fixed64 height = default,
        GravitasDiagnosticColor color = default)
    {
        _drawCommands.Add(new GravitasDebugDrawCommand(
            _context.FrameCount,
            _drawSequence++,
            kind,
            colliderId,
            ResolveDimension(colliderDimension, colliderType, collider2DType),
            colliderType,
            collider2DType,
            start,
            end,
            center,
            size,
            pointA,
            pointB,
            pointC,
            rotation,
            radius,
            height,
            color));
    }

    private static GravitasColliderDimension ResolveDimension(
        GravitasColliderDimension dimension,
        ColliderType colliderType,
        ColliderType2D collider2DType)
    {
        if (dimension != GravitasColliderDimension.None)
            return dimension;
        if (colliderType != ColliderType.None)
            return GravitasColliderDimension.ThreeD;
        return collider2DType != ColliderType2D.None
            ? GravitasColliderDimension.TwoD
            : GravitasColliderDimension.None;
    }

    private static Vector3d ToDiagnosticVector(Vector2d value) => new(value.X, Fixed64.Zero, value.Y);
}
