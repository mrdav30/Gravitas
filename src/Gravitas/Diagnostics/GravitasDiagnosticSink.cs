using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Raycasting;
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

    public int EventCount => _events.Count;

    public int DrawCommandCount => _drawCommands.Count;

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
                    height: capsule.ScaledSize.y,
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
            case LSMeshCollider mesh:
                CaptureMeshTriangles(mesh, color);
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

        Vector3d end = direction.SqrMagnitude == Fixed64.Zero
            ? origin
            : origin + direction.Normal * length;

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

    internal void EmitForceDelta(StiffBody body, Vector3d force, Vector3d accelerationDelta)
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

    internal void EmitTorqueDelta(StiffBody body, Vector3d torque)
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

    internal void EmitLinearVelocityDelta(StiffBody body, Vector3d before, Vector3d after)
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

    internal void EmitAngularVelocityDelta(StiffBody body, Vector3d before, Vector3d after)
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
        StiffBody body,
        GroundProbeMode mode,
        Vector3d origin,
        Vector3d end,
        Fixed64 radius,
        bool hit,
        LSRaycastHit raycastHit)
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

    internal void EmitRayQuery(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        int layerMaskBits,
        bool hit,
        int hitCount,
        LSRaycastHit raycastHit)
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
        LSRaycastHit raycastHit)
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

    private void AddEvent(
        GravitasDiagnosticEventKind kind,
        int bodyId = -1,
        int colliderAId = -1,
        int colliderBId = -1,
        ColliderType colliderAType = ColliderType.None,
        ColliderType colliderBType = ColliderType.None,
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
            colliderAType,
            colliderBType,
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
            colliderType,
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
}
