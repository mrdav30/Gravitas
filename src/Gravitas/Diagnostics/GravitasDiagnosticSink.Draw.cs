//=======================================================================
// GravitasDiagnosticSink.Draw.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Constraints;

namespace Gravitas.Diagnostics;

public sealed partial class GravitasDiagnosticSink
{
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
            case LSConeCollider cone:
                AddDrawCommand(
                    GravitasDebugDrawKind.WireCone,
                    cone.Id,
                    cone.Shape,
                    center: cone.Center,
                    rotation: cone.Rotation,
                    radius: cone.ScaledRadius,
                    height: cone.Height,
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
            case LSCapsuleCollider2D capsule:
                CaptureMixedCapsule(capsule, color, capsule.Id, capsule.Shape);
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
    /// Emits engine-agnostic draw commands for a 3D joint's anchors and active angular axes.
    /// </summary>
    public void CaptureJoint(Joint3D joint, GravitasDiagnosticColor color)
    {
        if (!Enabled)
            return;

        SwiftThrowHelper.ThrowIfNull(joint, nameof(joint));
        SolidBody bodyA = joint.BodyA;
        SolidBody bodyB = joint.BodyB;
        FixedQuaternion rotationA = (bodyA.Rotation * joint.LocalFrameA.Rotation).Normalized;
        FixedQuaternion rotationB = (bodyB.Rotation * joint.LocalFrameB.Rotation).Normalized;
        Vector3d anchorA = bodyA.Position3d + bodyA.Rotation * joint.LocalFrameA.Position;
        Vector3d anchorB = bodyB.Position3d + bodyB.Rotation * joint.LocalFrameB.Position;
        Fixed64 pointRadius = Fixed64.One / (Fixed64)10;
        Fixed64 axisLength = Fixed64.One;

        AddDrawCommand(
            GravitasDebugDrawKind.Point,
            bodyA.Collider.Id,
            bodyA.Collider.Shape,
            center: anchorA,
            radius: pointRadius,
            color: color);
        AddDrawCommand(
            GravitasDebugDrawKind.Point,
            bodyB.Collider.Id,
            bodyB.Collider.Shape,
            center: anchorB,
            radius: pointRadius,
            color: color);
        AddDrawCommand(
            GravitasDebugDrawKind.Line,
            bodyA.Collider.Id,
            bodyA.Collider.Shape,
            start: anchorA,
            end: anchorB,
            color: color);

        switch (joint.Type)
        {
            case JointType3D.Hinge:
                CaptureJointAxis(anchorA, rotationA * Vector3d.Right, axisLength, bodyA.Collider, color);
                CaptureJointAxis(anchorB, rotationB * Vector3d.Right, axisLength, bodyB.Collider, color);
                break;
            case JointType3D.ConeTwist:
                CaptureJointAxis(anchorA, rotationA * Vector3d.Forward, axisLength, bodyA.Collider, color);
                CaptureJointAxis(anchorB, rotationB * Vector3d.Forward, axisLength, bodyB.Collider, color);
                break;
            case JointType3D.Fixed:
                CaptureJointAxis(anchorA, rotationA * Vector3d.Right, axisLength, bodyA.Collider, color);
                CaptureJointAxis(anchorA, rotationA * Vector3d.Up, axisLength, bodyA.Collider, color);
                CaptureJointAxis(anchorA, rotationA * Vector3d.Forward, axisLength, bodyA.Collider, color);
                break;
        }
    }

    /// <summary>
    /// Emits engine-agnostic draw commands for a pure 2D joint's anchors and active planar axis.
    /// </summary>
    public void CaptureJoint(Joint2D joint, GravitasDiagnosticColor color)
    {
        if (!Enabled)
            return;

        SwiftThrowHelper.ThrowIfNull(joint, nameof(joint));
        SolidBody2D bodyA = joint.BodyA;
        SolidBody2D bodyB = joint.BodyB;
        Fixed64 y = bodyA.Agent.Transform.Position.Y;
        Vector2d anchorA2D = bodyA.Position + Vector2d.Rotate(joint.LocalFrameA.Anchor, bodyA.Rotation);
        Vector2d anchorB2D = bodyB.Position + Vector2d.Rotate(joint.LocalFrameB.Anchor, bodyB.Rotation);
        Vector3d anchorA = new(anchorA2D.X, y, anchorA2D.Y);
        Vector3d anchorB = new(anchorB2D.X, y, anchorB2D.Y);
        Fixed64 pointRadius = Fixed64.One / (Fixed64)10;

        AddDrawCommand(
            GravitasDebugDrawKind.Point,
            bodyA.Collider.Id,
            colliderDimension: GravitasColliderDimension.TwoD,
            collider2DType: bodyA.Collider.Shape,
            center: anchorA,
            radius: pointRadius,
            color: color);
        AddDrawCommand(
            GravitasDebugDrawKind.Point,
            bodyB.Collider.Id,
            colliderDimension: GravitasColliderDimension.TwoD,
            collider2DType: bodyB.Collider.Shape,
            center: anchorB,
            radius: pointRadius,
            color: color);
        AddDrawCommand(
            GravitasDebugDrawKind.Line,
            bodyA.Collider.Id,
            colliderDimension: GravitasColliderDimension.TwoD,
            collider2DType: bodyA.Collider.Shape,
            start: anchorA,
            end: anchorB,
            color: color);

        if (joint.Type != JointType2D.Prismatic)
            return;

        Vector2d axis2D = Vector2d.Rotate(Vector2d.Right, bodyA.Rotation + joint.LocalFrameA.Angle);
        Vector3d axis = new(axis2D.X, Fixed64.Zero, axis2D.Y);

        AddDrawCommand(
            GravitasDebugDrawKind.Ray,
            bodyA.Collider.Id,
            colliderDimension: GravitasColliderDimension.TwoD,
            collider2DType: bodyA.Collider.Shape,
            start: anchorA,
            end: anchorA + axis.Normalized,
            color: color);
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

    private void CaptureJointAxis(
        Vector3d anchor,
        Vector3d axis,
        Fixed64 length,
        LSCollider collider,
        GravitasDiagnosticColor color)
    {
        AddDrawCommand(
            GravitasDebugDrawKind.Ray,
            collider.Id,
            collider.Shape,
            start: anchor,
            end: anchor + axis.Normalized * length,
            color: color);
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
                case LSConeCollider cone:
                    AddDrawCommand(
                        GravitasDebugDrawKind.WireCone,
                        compound.Id,
                        compound.Shape,
                        center: cone.Center,
                        rotation: cone.Rotation,
                        radius: cone.ScaledRadius,
                        height: cone.Height,
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
                case LSCapsuleCollider2D capsule:
                    CaptureMixedCapsule(capsule, color, compound.Id, compound.Shape);
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

    private void CaptureMixedCapsule(
        LSCapsuleCollider2D capsule,
        GravitasDiagnosticColor color,
        int colliderId,
        ColliderType2D colliderType)
    {
        Fixed64 height = capsule.MixedHalfThickness * 2;
        Vector3d firstCenter = new(capsule.SegmentStart.X, capsule.MixedSlabCenterY, capsule.SegmentStart.Y);
        Vector3d secondCenter = new(capsule.SegmentEnd.X, capsule.MixedSlabCenterY, capsule.SegmentEnd.Y);
        AddDrawCommand(
            GravitasDebugDrawKind.WireCylinder,
            colliderId,
            colliderDimension: GravitasColliderDimension.TwoD,
            collider2DType: colliderType,
            center: firstCenter,
            radius: capsule.ScaledRadius,
            height: height,
            color: color);
        AddDrawCommand(
            GravitasDebugDrawKind.WireCylinder,
            colliderId,
            colliderDimension: GravitasColliderDimension.TwoD,
            collider2DType: colliderType,
            center: secondCenter,
            radius: capsule.ScaledRadius,
            height: height,
            color: color);

        Fixed64 segmentLength = Vector2d.Distance(capsule.SegmentStart, capsule.SegmentEnd);
        if (segmentLength <= Fixed64.Epsilon)
            return;

        AddDrawCommand(
            GravitasDebugDrawKind.WireBox,
            colliderId,
            colliderDimension: GravitasColliderDimension.TwoD,
            collider2DType: colliderType,
            center: new Vector3d(capsule.Center.X, capsule.MixedSlabCenterY, capsule.Center.Y),
            size: new Vector3d(capsule.ScaledRadius * 2, height, segmentLength),
            rotation: FixedQuaternion.FromEulerAngles(Fixed64.Zero, capsule.Rotation, Fixed64.Zero),
            color: color);
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
}
