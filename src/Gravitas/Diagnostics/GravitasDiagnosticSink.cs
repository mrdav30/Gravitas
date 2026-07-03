//=======================================================================
// GravitasDiagnosticSink.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Constraints;
using Gravitas.Queries;
using SwiftCollections;
using System;

namespace Gravitas.Diagnostics;

/// <summary>
/// Context-owned deterministic diagnostics buffer for physics events and debug draw commands.
/// </summary>
public sealed partial class GravitasDiagnosticSink
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

    internal void EmitJointRegistered(Joint3D joint)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.JointRegistered,
            jointId: joint.Id,
            colliderAId: joint.BodyA.Collider.Id,
            colliderBId: joint.BodyB.Collider.Id,
            colliderAType: joint.BodyA.Collider.Shape,
            colliderBType: joint.BodyB.Collider.Shape,
            dataA: (int)joint.Type,
            dataB: (int)joint.CollisionPolicy);
    }

    internal void EmitJointRegistered(Joint2D joint)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.JointRegistered,
            jointId: joint.Id,
            colliderAId: joint.BodyA.Collider.Id,
            colliderBId: joint.BodyB.Collider.Id,
            colliderADimension: GravitasColliderDimension.TwoD,
            colliderBDimension: GravitasColliderDimension.TwoD,
            colliderA2DType: joint.BodyA.Collider.Shape,
            colliderB2DType: joint.BodyB.Collider.Shape,
            dataA: (int)joint.Type,
            dataB: (int)joint.CollisionPolicy);
    }

    internal void EmitJointRemoved(Joint3D joint)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.JointRemoved,
            jointId: joint.Id,
            colliderAId: joint.BodyA.Collider.Id,
            colliderBId: joint.BodyB.Collider.Id,
            colliderAType: joint.BodyA.Collider.Shape,
            colliderBType: joint.BodyB.Collider.Shape,
            dataA: (int)joint.Type,
            dataB: (int)joint.CollisionPolicy);
    }

    internal void EmitJointRemoved(Joint2D joint)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.JointRemoved,
            jointId: joint.Id,
            colliderAId: joint.BodyA.Collider.Id,
            colliderBId: joint.BodyB.Collider.Id,
            colliderADimension: GravitasColliderDimension.TwoD,
            colliderBDimension: GravitasColliderDimension.TwoD,
            colliderA2DType: joint.BodyA.Collider.Shape,
            colliderB2DType: joint.BodyB.Collider.Shape,
            dataA: (int)joint.Type,
            dataB: (int)joint.CollisionPolicy);
    }

    internal void EmitJointImpulse(Joint3D joint, JointSolveMetrics3D metrics)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.JointImpulse,
            jointId: joint.Id,
            colliderAId: joint.BodyA.Collider.Id,
            colliderBId: joint.BodyB.Collider.Id,
            colliderAType: joint.BodyA.Collider.Shape,
            colliderBType: joint.BodyB.Collider.Shape,
            vector: new Vector3d(
                metrics.MotorImpulseMagnitude,
                metrics.MotorErrorMagnitude,
                metrics.AngularLimitErrorMagnitude),
            scalarA: metrics.AccumulatedImpulseMagnitude,
            scalarB: metrics.LinearAnchorErrorMagnitude,
            dataA: metrics.PreparedRowCount,
            dataB: metrics.ClampedRowCount,
            hit: metrics.IncrementalImpulseMagnitude > Fixed64.Zero);
    }

    internal void EmitJointImpulse(Joint2D joint, JointSolveMetrics2D metrics)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.JointImpulse,
            jointId: joint.Id,
            colliderAId: joint.BodyA.Collider.Id,
            colliderBId: joint.BodyB.Collider.Id,
            colliderADimension: GravitasColliderDimension.TwoD,
            colliderBDimension: GravitasColliderDimension.TwoD,
            colliderA2DType: joint.BodyA.Collider.Shape,
            colliderB2DType: joint.BodyB.Collider.Shape,
            vector: new Vector3d(
                metrics.MotorImpulseMagnitude,
                metrics.MotorErrorMagnitude,
                metrics.LimitErrorMagnitude),
            scalarA: metrics.AccumulatedImpulseMagnitude,
            scalarB: metrics.LinearAnchorErrorMagnitude,
            dataA: metrics.PreparedRowCount,
            dataB: metrics.ClampedRowCount,
            hit: metrics.IncrementalImpulseMagnitude > Fixed64.Zero);
    }

    internal void EmitJointLimitReached(Joint3D joint, Fixed64 limitError)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.JointLimitReached,
            jointId: joint.Id,
            colliderAId: joint.BodyA.Collider.Id,
            colliderBId: joint.BodyB.Collider.Id,
            colliderAType: joint.BodyA.Collider.Shape,
            colliderBType: joint.BodyB.Collider.Shape,
            scalarB: limitError,
            dataA: (int)joint.Limits.Kind,
            hit: limitError != Fixed64.Zero);
    }

    internal void EmitJointLimitReached(Joint2D joint, Fixed64 limitError)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.JointLimitReached,
            jointId: joint.Id,
            colliderAId: joint.BodyA.Collider.Id,
            colliderBId: joint.BodyB.Collider.Id,
            colliderADimension: GravitasColliderDimension.TwoD,
            colliderBDimension: GravitasColliderDimension.TwoD,
            colliderA2DType: joint.BodyA.Collider.Shape,
            colliderB2DType: joint.BodyB.Collider.Shape,
            scalarB: limitError,
            dataA: (int)joint.Limits.Kind,
            hit: limitError != Fixed64.Zero);
    }

    internal void EmitRagdollActivated(int ragdollId, int linkCount, int jointCount, bool isActive)
    {
        if (!Enabled)
            return;

        AddEvent(
            GravitasDiagnosticEventKind.RagdollActivated,
            bodyId: ragdollId,
            dataA: linkCount,
            dataB: jointCount,
            hit: isActive);
    }

    private void AddEvent(
        GravitasDiagnosticEventKind kind,
        int bodyId = -1,
        int jointId = -1,
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
            jointId,
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
