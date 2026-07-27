//=======================================================================
// CollisionDetectionMixed.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Deterministic mixed 2D/3D narrow-phase collision checks.
/// </summary>
public static partial class CollisionDetectionMixed
{
    public static bool TryCollide(LSCollider collider3D, LSCollider2D collider2D, out MixedContact contact)
    {
        SwiftThrowHelper.ThrowIfNull(collider3D, nameof(collider3D));
        SwiftThrowHelper.ThrowIfNull(collider2D, nameof(collider2D));

        if (!collider3D.Bounds.Intersects(collider2D.MixedBounds3D))
        {
            contact = default;
            return false;
        }

        if (collider2D is LSCompoundCollider2D compound2D)
            return TryEmbeddedCompound2D(collider3D, compound2D, out contact);

        return collider3D.Shape switch
        {
            ColliderType.Sphere => TrySphereEmbedded2D((LSSphereCollider)collider3D, collider2D, out contact),
            ColliderType.AABox or ColliderType.OBBox => TryCuboidEmbedded2D((LSCuboidCollider)collider3D, collider2D, out contact),
            ColliderType.Capsule => TryCapsuleEmbedded2D((LSCapsuleCollider)collider3D, collider2D, out contact),
            ColliderType.Cylinder => TryCylinderEmbedded2D((LSCylinderCollider)collider3D, collider2D, out contact),
            ColliderType.Cone => TryConeEmbedded2D((LSConeCollider)collider3D, collider2D, out contact),
            ColliderType.Mesh => TryMeshEmbedded2D((LSMeshCollider)collider3D, collider2D, out contact),
            ColliderType.Compound => TryCompoundEmbedded2D((LSCompoundCollider)collider3D, collider2D, out contact),
            _ => NoContact(out contact)
        };
    }

    private static bool TryEmbeddedCompound2D(
        LSCollider collider3D,
        LSCompoundCollider2D compound2D,
        out MixedContact contact)
    {
        bool found = false;
        MixedContact best = default;

        for (int i = 0; i < compound2D.PartCount; i++)
        {
            LSCollider2D part = compound2D.GetPartCollider(i);
            if (!collider3D.Bounds.Intersects(part.MixedBounds3D)
                || !TryCollide(collider3D, part, out MixedContact candidate))
            {
                continue;
            }

            candidate = candidate.WithFallbackMaterials(collider3D.Material, part.Material);

            if (ContactSelectionPolicy.ShouldReplaceWithShallower(candidate, found, best))
            {
                best = candidate;
                found = true;
            }
        }

        if (!found)
            return NoContact(out contact);

        contact = best;
        return true;
    }

    private static bool TrySphereEmbedded2D(LSSphereCollider sphere, LSCollider2D embedded, out MixedContact contact)
    {
        if (embedded is LSAABBoxCollider2D or LSPolygonCollider2D)
            return TryGetSphereConvexPrismContact(sphere, embedded, out contact);

        Vector3d sphereCenter = sphere.Center;
        Vector2d planarCenter = new(sphereCenter.X, sphereCenter.Z);
        bool planarInside = embedded.ContainsPoint(planarCenter);
        bool inside = planarInside
            && MixedEmbedded2DGeometry.ContainsPointInSlab(
                embedded,
                sphereCenter.Y);
        ContactAnchor embeddedAnchor =
            MixedEmbedded2DGeometry.GetClosestAnchorOnEmbeddedVolume(
                embedded,
                sphereCenter);
        if (!embeddedAnchor.TryGetOffsetFrom(
                sphereCenter,
                out Vector3d delta)
            || !Vector3d.TryGetMagnitude(delta, out Fixed64 distance)
            || (!inside && distance > sphere.ScaledRadius))
        {
            contact = default;
            return false;
        }

        Vector3d normal = distance > Fixed64.Zero
            ? delta / distance
            : ResolveFallbackNormal(sphereCenter, embedded);
        bool exactDepth = inside
            ? Fixed64.TryAdd(
                sphere.ScaledRadius,
                distance,
                out Fixed64 depth)
            : Fixed64.TrySubtract(
                sphere.ScaledRadius,
                distance,
                out depth);
        if (!exactDepth)
            depth = Fixed64.MaxValue;
        contact = new MixedContact(
            new ContactAnchor(
                sphereCenter,
                normal * sphere.ScaledRadius),
            embeddedAnchor,
            normal,
            depth,
            depthIsClamped: !exactDepth);
        return true;
    }

    private static bool TryCapsuleEmbedded2D(LSCapsuleCollider capsule, LSCollider2D embedded, out MixedContact contact)
    {
        if (embedded is LSAABBoxCollider2D or LSPolygonCollider2D)
            return TryGetCapsuleConvexPrismContact(capsule, embedded, out contact);

        bool collided = embedded.Shape == ColliderType2D.Circle
            ? TryTestCapsuleCircleSlab(capsule, (LSCircleCollider2D)embedded, out AxisPenetration penetration)
            : TryTestCapsuleCapsuleSlab(capsule, (LSCapsuleCollider2D)embedded, out penetration);

        return collided
            ? BuildCapsuleContact(capsule, embedded, penetration, out contact)
            : NoContact(out contact);
    }

    private static bool TryCylinderEmbedded2D(LSCylinderCollider cylinder, LSCollider2D embedded, out MixedContact contact)
    {
        if (embedded is LSAABBoxCollider2D or LSPolygonCollider2D)
            return TryGetCylinderConvexPrismContact(cylinder, embedded, out contact);

        bool collided = embedded.Shape == ColliderType2D.Circle
            ? TryTestCylinderCircleSlab(cylinder, (LSCircleCollider2D)embedded, out AxisPenetration penetration)
            : TryTestCylinderCapsuleSlab(cylinder, (LSCapsuleCollider2D)embedded, out penetration);

        return collided
            ? BuildCylinderContact(cylinder, embedded, penetration, out contact)
            : NoContact(out contact);
    }

    private static bool TryConeEmbedded2D(LSConeCollider cone, LSCollider2D embedded, out MixedContact contact)
    {
        if (embedded is LSAABBoxCollider2D or LSPolygonCollider2D)
            return TryGetConeConvexPrismContact(cone, embedded, out contact);

        bool collided = embedded.Shape == ColliderType2D.Circle
            ? TryTestConeCircleSlab(cone, (LSCircleCollider2D)embedded, out AxisPenetration penetration)
            : TryTestConeCapsuleSlab(cone, (LSCapsuleCollider2D)embedded, out penetration);

        return collided
            ? BuildConeContact(cone, embedded, penetration, out contact)
            : NoContact(out contact);
    }

    private static bool TryTestCapsuleCircleSlab(
        LSCapsuleCollider capsule,
        LSCircleCollider2D circle,
        out AxisPenetration penetration)
    {
        penetration = default;
        Vector3d capsuleAxis = GetRigidUpAxis(capsule.Rotation);
        CheckCapsuleCircleSlabAxis(
            capsule,
            circle,
            capsuleAxis,
            Vector3d.Up,
            ref penetration);

        if (!CheckCapsuleCircleSlabAxis(
                capsule,
                circle,
                capsuleAxis,
                capsuleAxis,
                ref penetration))
            return false;

        if (!CheckCapsuleCircleSlabAxis(
                capsule,
                circle,
                capsuleAxis,
                Vector3d.Cross(capsuleAxis, Vector3d.Up),
                ref penetration))
            return false;

        if (!FixedSegment.TryGetClosestPointsBetweenCenteredAxes(
                capsule.Center,
                capsuleAxis,
                capsule.AxisLength,
                GetEmbeddedCenter3D(circle),
                Vector3d.Up,
                circle.MixedHalfThickness * Fixed64.Two,
                out Vector3d capsulePoint,
                out Vector3d circlePoint)
            || !CheckCapsuleCircleSlabAxis(
                capsule,
                circle,
                capsuleAxis,
                circlePoint - capsulePoint,
                ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool TryTestCylinderCircleSlab(
        LSCylinderCollider cylinder,
        LSCircleCollider2D circle,
        out AxisPenetration penetration)
    {
        penetration = default;
        Vector3d cylinderAxis = GetRigidUpAxis(cylinder.Rotation);
        if (!CheckCylinderCircleSlabAxis(
                cylinder,
                circle,
                cylinderAxis,
                cylinderAxis,
                ref penetration))
            return false;

        CheckCylinderCircleSlabAxis(
            cylinder,
            circle,
            cylinderAxis,
            Vector3d.Up,
            ref penetration);

        if (!CheckCylinderCircleSlabAxis(
                cylinder,
                circle,
                cylinderAxis,
                Vector3d.Cross(cylinderAxis, Vector3d.Up),
                ref penetration))
            return false;

        if (!FixedSegment.TryGetClosestPointsBetweenCenteredAxes(
                cylinder.Center,
                cylinderAxis,
                cylinder.Height,
                GetEmbeddedCenter3D(circle),
                Vector3d.Up,
                circle.MixedHalfThickness * Fixed64.Two,
                out Vector3d cylinderPoint,
                out Vector3d circlePoint)
            || !CheckCylinderCircleSlabAxis(
                cylinder,
                circle,
                cylinderAxis,
                circlePoint - cylinderPoint,
                ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool TryTestConeCircleSlab(
        LSConeCollider cone,
        LSCircleCollider2D circle,
        out AxisPenetration penetration)
    {
        penetration = default;
        GetCircleSlabSegment(circle, out Vector3d circleStart, out Vector3d circleEnd);
        Vector3d coneAxis = GetRigidUpAxis(cone.Rotation);

        if (!CheckConeCircleSlabAxis(cone, circle, coneAxis, ref penetration))
            return false;

        CheckConeCircleSlabAxis(cone, circle, Vector3d.Up, ref penetration);

        if (!CheckConeCircleSlabAxis(
                cone,
                circle,
                Vector3d.Cross(coneAxis, Vector3d.Up),
                ref penetration))
            return false;

        Vector3d conePoint = cone.ClosestPointOnSurface(GetEmbeddedCenter3D(circle));
        Vector3d circlePoint = Vector3d.ClosestPointOnLineSegment(conePoint, circleStart, circleEnd);
        if (!CheckConeCircleSlabAxis(cone, circle, circlePoint - conePoint, ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool TryTestCapsuleCapsuleSlab(
        LSCapsuleCollider capsule,
        LSCapsuleCollider2D prism,
        out AxisPenetration penetration)
    {
        penetration = default;
        Vector3d capsuleAxis = GetRigidUpAxis(capsule.Rotation);

        if (!CheckCapsulePrismAxis(
                capsule,
                prism,
                capsuleAxis,
                capsuleAxis,
                ref penetration))
            return false;

        CheckCapsulePrismAxis(
            capsule,
            prism,
            capsuleAxis,
            Vector3d.Up,
            ref penetration);

        if (!CheckCapsulePrismAxis(
                capsule,
                prism,
                capsuleAxis,
                Vector3d.Cross(capsuleAxis, Vector3d.Up),
                ref penetration))
            return false;

        if (!CheckEmbeddedCapsuleAxes(
                capsule,
                capsuleAxis,
                prism,
                ref penetration)
            || !CheckCapsuleEmbeddedCapsuleEdgeAxis(
                capsule,
                capsuleAxis,
                prism,
                ref penetration))
        {
            return false;
        }

        if (!TryGetClosestPointOnCenteredAxis(
                GetEmbeddedCenter3D(prism),
                capsule.Center,
                capsuleAxis,
                capsule.AxisLength,
                out Vector3d linePoint))
        {
            return false;
        }

        ContactAnchor embeddedPoint =
            MixedEmbedded2DGeometry.GetClosestAnchorOnEmbeddedVolume(
                prism,
                linePoint);
        if (!embeddedPoint.TryGetOffsetFrom(
                linePoint,
                out Vector3d closestFeatureAxis)
            || !CheckCapsulePrismAxis(
                capsule,
                prism,
                capsuleAxis,
                closestFeatureAxis,
                ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool TryTestCylinderCapsuleSlab(
        LSCylinderCollider cylinder,
        LSCapsuleCollider2D prism,
        out AxisPenetration penetration)
    {
        penetration = default;
        Vector3d cylinderAxis = GetRigidUpAxis(cylinder.Rotation);

        if (!CheckCylinderPrismAxis(
                cylinder,
                prism,
                cylinderAxis,
                cylinderAxis,
                ref penetration))
            return false;

        CheckCylinderPrismAxis(
            cylinder,
            prism,
            cylinderAxis,
            Vector3d.Up,
            ref penetration);

        if (!CheckCylinderPrismAxis(
                cylinder,
                prism,
                cylinderAxis,
                Vector3d.Cross(cylinderAxis, Vector3d.Up),
                ref penetration))
            return false;

        if (!CheckEmbeddedCapsuleAxes(
                cylinder,
                cylinderAxis,
                prism,
                ref penetration)
            || !CheckCylinderEmbeddedCapsuleEdgeAxis(
                cylinder,
                cylinderAxis,
                prism,
                ref penetration))
        {
            return false;
        }

        if (!TryGetClosestPointOnCenteredAxis(
                GetEmbeddedCenter3D(prism),
                cylinder.Center,
                cylinderAxis,
                cylinder.Height,
                out Vector3d linePoint))
        {
            return false;
        }

        ContactAnchor embeddedPoint =
            MixedEmbedded2DGeometry.GetClosestAnchorOnEmbeddedVolume(
                prism,
                linePoint);
        if (!embeddedPoint.TryGetOffsetFrom(
                linePoint,
                out Vector3d closestFeatureAxis)
            || !CheckCylinderPrismAxis(
                cylinder,
                prism,
                cylinderAxis,
                closestFeatureAxis,
                ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool TryTestConeCapsuleSlab(
        LSConeCollider cone,
        LSCapsuleCollider2D prism,
        out AxisPenetration penetration)
    {
        penetration = default;
        Vector3d coneAxis = GetRigidUpAxis(cone.Rotation);

        if (!CheckConePrismAxis(cone, prism, coneAxis, ref penetration))
            return false;

        CheckConePrismAxis(cone, prism, Vector3d.Up, ref penetration);

        if (!CheckConePrismAxis(
                cone,
                prism,
                Vector3d.Cross(coneAxis, Vector3d.Up),
                ref penetration))
            return false;

        if (!CheckEmbeddedCapsuleAxes(cone, prism, ref penetration)
            || !CheckConeEmbeddedCapsuleEdgeAxis(
                cone,
                coneAxis,
                prism,
                ref penetration))
        {
            return false;
        }

        ContactAnchor embeddedPoint =
            MixedEmbedded2DGeometry.GetClosestAnchorOnEmbeddedVolume(
                prism,
                cone.Center);
        if (!embeddedPoint.TryGetWorldPoint(out Vector3d reference))
        {
            if (!embeddedPoint.TryGetOffsetFrom(
                    cone.Center,
                    out Vector3d centerToEmbedded)
                || !CheckConePrismAxis(
                    cone,
                    prism,
                    centerToEmbedded,
                    ref penetration))
            {
                return false;
            }

            return penetration.HasValue;
        }

        Vector3d conePoint = cone.ClosestPointOnSurface(reference);
        if (!embeddedPoint.TryGetOffsetFrom(
                conePoint,
                out Vector3d closestFeatureAxis)
            || !CheckConePrismAxis(
                cone,
                prism,
                closestFeatureAxis,
                ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool CheckCapsuleCircleSlabAxis(
        LSCapsuleCollider capsule,
        LSCircleCollider2D circle,
        Vector3d capsuleAxis,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        if (!FixedSegment.TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
            normalizedAxis,
            GetEmbeddedCenter3D(circle),
            Vector3d.Up,
            circle.MixedHalfThickness * Fixed64.Two,
            circle.ScaledRadius,
            capsule.Center,
            capsuleAxis,
            capsule.AxisLength,
            capsule.ScaledRadius,
            out Vector3d circleToCapsuleAxis,
            out Fixed64 depth,
            out bool depthIsClamped))
        {
            return false;
        }

        if (!penetration.HasValue || depth < penetration.Depth)
        {
            penetration = new AxisPenetration(
                -circleToCapsuleAxis,
                depth,
                depthIsClamped);
        }

        return true;
    }

    private static bool CheckCylinderCircleSlabAxis(
        LSCylinderCollider cylinder,
        LSCircleCollider2D circle,
        Vector3d cylinderAxis,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        if (!FixedSegment.TryGetCenteredFiniteCylinderCapsuleSlabAxisPenetration(
                normalizedAxis,
                cylinder.Center,
                cylinderAxis,
                cylinder.Height,
                cylinder.ScaledRadius,
                GetEmbeddedCenter3D(circle),
                Vector2d.Forward,
                Fixed64.Zero,
                circle.ScaledRadius,
                circle.MixedHalfThickness,
                out Vector3d orientedAxis,
                out Fixed64 depth,
                out bool depthIsClamped))
        {
            return false;
        }

        return KeepAxisPenetration(
            orientedAxis,
            depth,
            depthIsClamped,
            ref penetration);
    }

    private static bool CheckConeCircleSlabAxis(
        LSConeCollider cone,
        LSCircleCollider2D circle,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        if (!FixedSegment.TryGetCenteredFiniteConeCapsuleSlabAxisPenetration(
                normalizedAxis,
                cone.Center,
                GetRigidUpAxis(cone.Rotation),
                cone.Height,
                cone.ScaledRadius,
                GetEmbeddedCenter3D(circle),
                Vector2d.Forward,
                Fixed64.Zero,
                circle.ScaledRadius,
                circle.MixedHalfThickness,
                out Vector3d orientedAxis,
                out Fixed64 depth,
                out bool depthIsClamped))
        {
            return false;
        }

        return KeepAxisPenetration(
            orientedAxis,
            depth,
            depthIsClamped,
            ref penetration);
    }

    private static bool CheckCapsulePrismAxis(
        LSCapsuleCollider capsule,
        LSCapsuleCollider2D prism,
        Vector3d capsuleAxis,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        if (!FixedSegment.TryGetCenteredCapsuleCapsuleSlabAxisPenetration(
                normalizedAxis,
                capsule.Center,
                capsuleAxis,
                capsule.AxisLength,
                capsule.ScaledRadius,
                GetEmbeddedCenter3D(prism),
                prism.WorldAxis,
                prism.AxisLength,
                prism.ScaledRadius,
                prism.MixedHalfThickness,
                out Vector3d orientedAxis,
                out Fixed64 depth,
                out bool depthIsClamped))
            return false;

        return KeepAxisPenetration(
            orientedAxis,
            depth,
            depthIsClamped,
            ref penetration);
    }

    private static bool CheckCylinderPrismAxis(
        LSCylinderCollider cylinder,
        LSCapsuleCollider2D prism,
        Vector3d cylinderAxis,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        if (!FixedSegment.TryGetCenteredFiniteCylinderCapsuleSlabAxisPenetration(
                normalizedAxis,
                cylinder.Center,
                cylinderAxis,
                cylinder.Height,
                cylinder.ScaledRadius,
                GetEmbeddedCenter3D(prism),
                prism.WorldAxis,
                prism.AxisLength,
                prism.ScaledRadius,
                prism.MixedHalfThickness,
                out Vector3d orientedAxis,
                out Fixed64 depth,
                out bool depthIsClamped))
            return false;

        return KeepAxisPenetration(
            orientedAxis,
            depth,
            depthIsClamped,
            ref penetration);
    }

    private static bool CheckConePrismAxis(
        LSConeCollider cone,
        LSCapsuleCollider2D prism,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        if (!FixedSegment.TryGetCenteredFiniteConeCapsuleSlabAxisPenetration(
                normalizedAxis,
                cone.Center,
                GetRigidUpAxis(cone.Rotation),
                cone.Height,
                cone.ScaledRadius,
                GetEmbeddedCenter3D(prism),
                prism.WorldAxis,
                prism.AxisLength,
                prism.ScaledRadius,
                prism.MixedHalfThickness,
                out Vector3d orientedAxis,
                out Fixed64 depth,
                out bool depthIsClamped))
            return false;

        return KeepAxisPenetration(
            orientedAxis,
            depth,
            depthIsClamped,
            ref penetration);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool KeepAxisPenetration(
        Vector3d axis,
        Fixed64 depth,
        bool depthIsClamped,
        ref AxisPenetration penetration)
    {
        if (!penetration.HasValue || depth < penetration.Depth)
        {
            penetration = new AxisPenetration(
                axis,
                depth,
                depthIsClamped);
        }

        return true;
    }

    private static bool CheckEmbeddedCapsuleAxes(
        LSCapsuleCollider capsule3D,
        Vector3d capsule3DAxis,
        LSCapsuleCollider2D capsule2D,
        ref AxisPenetration penetration)
    {
        GetEmbeddedCapsuleAxes(capsule2D, out Vector3d axis, out Vector3d normal);
        return CheckCapsulePrismAxis(
                capsule3D,
                capsule2D,
                capsule3DAxis,
                axis,
                ref penetration)
            && CheckCapsulePrismAxis(
                capsule3D,
                capsule2D,
                capsule3DAxis,
                normal,
                ref penetration);
    }

    private static bool CheckEmbeddedCapsuleAxes(
        LSCylinderCollider cylinder,
        Vector3d cylinderAxis,
        LSCapsuleCollider2D capsule,
        ref AxisPenetration penetration)
    {
        GetEmbeddedCapsuleAxes(capsule, out Vector3d axis, out Vector3d normal);
        return CheckCylinderPrismAxis(
                cylinder,
                capsule,
                cylinderAxis,
                axis,
                ref penetration)
            && CheckCylinderPrismAxis(
                cylinder,
                capsule,
                cylinderAxis,
                normal,
                ref penetration);
    }

    private static bool CheckEmbeddedCapsuleAxes(
        LSConeCollider cone,
        LSCapsuleCollider2D capsule,
        ref AxisPenetration penetration)
    {
        GetEmbeddedCapsuleAxes(capsule, out Vector3d axis, out Vector3d normal);
        return CheckConePrismAxis(cone, capsule, axis, ref penetration)
            && CheckConePrismAxis(cone, capsule, normal, ref penetration);
    }

    private static bool CheckCapsuleEmbeddedCapsuleEdgeAxis(
        LSCapsuleCollider capsule3D,
        Vector3d capsule3DAxis,
        LSCapsuleCollider2D capsule2D,
        ref AxisPenetration penetration)
    {
        GetEmbeddedCapsuleAxes(capsule2D, out Vector3d capsuleAxis, out _);
        return CheckCapsulePrismAxis(
            capsule3D,
            capsule2D,
            capsule3DAxis,
            Vector3d.Cross(capsule3DAxis, capsuleAxis),
            ref penetration);
    }

    private static bool CheckCylinderEmbeddedCapsuleEdgeAxis(
        LSCylinderCollider cylinder,
        Vector3d cylinderAxis,
        LSCapsuleCollider2D capsule,
        ref AxisPenetration penetration)
    {
        GetEmbeddedCapsuleAxes(capsule, out Vector3d capsuleAxis, out _);
        return CheckCylinderPrismAxis(
            cylinder,
            capsule,
            cylinderAxis,
            Vector3d.Cross(cylinderAxis, capsuleAxis),
            ref penetration);
    }

    private static bool CheckConeEmbeddedCapsuleEdgeAxis(
        LSConeCollider cone,
        Vector3d coneAxis,
        LSCapsuleCollider2D capsule,
        ref AxisPenetration penetration)
    {
        GetEmbeddedCapsuleAxes(capsule, out Vector3d capsuleAxis, out _);
        return CheckConePrismAxis(
            cone,
            capsule,
            Vector3d.Cross(coneAxis, capsuleAxis),
            ref penetration);
    }

    private static bool BuildCapsuleContact(
        LSCapsuleCollider capsule,
        LSCollider2D embedded,
        AxisPenetration penetration,
        out MixedContact contact)
    {
        Vector3d localSurfaceDirection =
            capsule.Rotation.Inverse().Rotate(penetration.Axis).Normalized;
        var anchor3D = new ContactAnchor(
            FixedSegment.GetSurfaceAnchorOnCenteredCapsule(
                GetEmbeddedCenter3D(embedded),
                capsule.Center,
                capsule.Rotation,
                Vector3d.Up,
                capsule.AxisLength,
                capsule.ScaledRadius,
                localSurfaceDirection));
        if (!anchor3D.TryGetWorldPoint(out Vector3d point3D))
        {
            contact = new MixedContact(
                anchor3D,
                MixedEmbedded2DGeometry.GetSupportAnchor(
                    embedded,
                    -penetration.Axis),
                penetration.Axis,
                penetration.Depth,
                penetration.DepthIsClamped);
            return true;
        }

        ContactAnchor anchor2D =
            MixedEmbedded2DGeometry.GetClosestAnchorOnEmbeddedVolume(
                embedded,
                point3D);
        contact = new MixedContact(
            anchor3D,
            anchor2D,
            penetration.Axis,
            penetration.Depth,
            penetration.DepthIsClamped);
        return true;
    }

    private static bool BuildCylinderContact(
        LSCylinderCollider cylinder,
        LSCollider2D embedded,
        AxisPenetration penetration,
        out MixedContact contact)
    {
        contact = BuildCanonicalSupportContact(
            cylinder,
            embedded,
            penetration.Axis,
            penetration.Depth,
            penetration.DepthIsClamped);
        return true;
    }

    private static bool BuildConeContact(
        LSConeCollider cone,
        LSCollider2D embedded,
        AxisPenetration penetration,
        out MixedContact contact)
    {
        contact = BuildCanonicalSupportContact(
            cone,
            embedded,
            penetration.Axis,
            penetration.Depth,
            penetration.DepthIsClamped);
        return true;
    }

    private static MixedContact BuildCanonicalSupportContact(
        LSCollider collider3D,
        LSCollider2D embedded,
        Vector3d normal3DTo2D,
        Fixed64 depth,
        bool depthIsClamped)
    {
        return new MixedContact(
            new ContactAnchor(
                ConvexColliderSupport.GetSupportAnchor(
                    collider3D,
                    normal3DTo2D,
                    Vector3d.Zero)),
            MixedEmbedded2DGeometry.GetSupportAnchor(embedded, -normal3DTo2D),
            normal3DTo2D,
            depth,
            depthIsClamped);
    }

}
