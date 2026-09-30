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
    /// <summary>Tests a 3D collider against an embedded 2D collider.</summary>
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
        // Broad-phase overlap plus a closest-volume anchor bounds every
        // component of this relative offset to the Fixed64 domain.
        _ = embeddedAnchor.TryGetOffsetFrom(
            sphereCenter,
            out Vector3d delta);
        if (!Vector3d.TryGetMagnitude(delta, out Fixed64 distance)
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

        if (embedded is LSCircleCollider2D circle)
            return TryGetCapsuleCircleSlabContact(capsule, circle, circle.ScaledRadius, out contact);

        var prism = (LSCapsuleCollider2D)embedded;
        if (!WideConvexPrismRelations.TryGetCenteredCapsuleSlabCapsulePenetration(
                GetEmbeddedCenter3D(prism), prism.Rotation, prism.AxisLength, prism.ScaledRadius, prism.MixedHalfThickness,
                capsule.Center, capsule.Rotation, capsule.AxisLength, capsule.ScaledRadius,
                out Vector3d normal, out Fixed64 depth, out bool depthIsClamped))
            return NoContact(out contact);

        contact = BuildCanonicalSupportContact(capsule, prism, -normal, depth, depthIsClamped);
        return true;
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

    private static bool TryGetCapsuleCircleSlabContact(
        LSCapsuleCollider capsule,
        LSCollider2D slab,
        Fixed64 radius,
        out MixedContact contact)
    {
        // Keep the admitted half-thickness even when full height is wider
        // than Fixed64. The complete query owns rim and interior-core minima.
        Signed192 length = WideArithmetic.AddSigned192(
            Signed192.Raw(slab.MixedHalfThickness), Signed192.Raw(slab.MixedHalfThickness));
        if (!WideConvexPrismRelations.TryGetCenteredFiniteCylinderCapsulePenetration(
                GetEmbeddedCenter3D(slab), FixedQuaternion.Identity, Vector3d.Up, length, radius,
                capsule.Center, capsule.Rotation, Vector3d.Up, capsule.AxisLength, capsule.ScaledRadius,
                out Vector3d normal, out Fixed64 depth, out bool depthIsClamped))
            return NoContact(out contact);

        // The geometry query is cylinder-first; mixed contacts are 3D-to-2D.
        contact = BuildCanonicalSupportContact(capsule, slab, -normal, depth, depthIsClamped);
        return true;
    }

    private static bool TryTestCylinderCircleSlab(
        LSCylinderCollider cylinder,
        LSCircleCollider2D circle,
        out AxisPenetration penetration)
    {
        penetration = default;
        // Keep the full admitted half-thickness: doubling in Fixed64 would
        // shorten slabs whose physical full height exceeds the scalar domain.
        Signed192 slabLength = WideArithmetic.AddSigned192(
            Signed192.Raw(circle.MixedHalfThickness), Signed192.Raw(circle.MixedHalfThickness));
        if (!WideConvexPrismRelations.TryGetPositiveRadiusCylinderPairPenetration(
                cylinder.Center, cylinder.Rotation, Vector3d.Up,
                Signed192.Raw(cylinder.Height), cylinder.ScaledRadius,
                GetEmbeddedCenter3D(circle), FixedQuaternion.Identity, Vector3d.Up,
                slabLength, circle.ScaledRadius,
                out Vector3d normal, out Fixed64 depth, out bool depthIsClamped))
            return false;

        penetration = new AxisPenetration(normal, depth, depthIsClamped);
        return true;
    }

    private static bool TryTestConeCircleSlab(
        LSConeCollider cone,
        LSCircleCollider2D circle,
        out AxisPenetration penetration)
    {
        penetration = default;
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

        Vector3d circleCenter = GetEmbeddedCenter3D(circle);
        // The admitted broad-phase bounds and separating axes guarantee that
        // this closest-feature offset is representable in either rigid frame.
        _ = FixedSegment.TryGetClosestCenteredFiniteConeSurfaceAnchor(
            circleCenter,
            cone.Center,
            cone.Rotation,
            Vector3d.Up,
            cone.Height,
            cone.ScaledRadius,
            Vector3d.Right,
            out FixedPointAnchor conePoint,
            out _,
            out _);
        _ = conePoint.TryGetOffsetFrom(
            new FixedPointAnchor(
                circleCenter,
                FixedQuaternion.Identity,
                Vector3d.Zero),
            out Vector3d conePointFromCircle);

        Fixed64 closestCircleY = FixedMath.Clamp(
            conePointFromCircle.Y,
            -circle.MixedHalfThickness,
            circle.MixedHalfThickness);
        _ = Vector3d.TrySubtract(
            new Vector3d(Fixed64.Zero, closestCircleY, Fixed64.Zero),
            conePointFromCircle,
            out Vector3d closestFeatureAxis);
        if (!CheckConeCircleSlabAxis(
                cone,
                circle,
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
        GetEmbeddedCapsuleAxes(
            prism,
            out Vector2d prismPlanarAxis,
            out Vector3d prismAxis,
            out Vector3d prismNormal);

        if (!CheckCylinderPrismAxis(
                cylinder,
                prism,
                cylinderAxis,
                prismPlanarAxis,
                cylinderAxis,
                ref penetration))
            return false;

        CheckCylinderPrismAxis(
            cylinder,
            prism,
            cylinderAxis,
            prismPlanarAxis,
            Vector3d.Up,
            ref penetration);

        if (!CheckCylinderPrismAxis(
                cylinder,
                prism,
                cylinderAxis,
                prismPlanarAxis,
                Vector3d.Cross(cylinderAxis, Vector3d.Up),
                ref penetration))
            return false;

        if (!CheckEmbeddedCapsuleAxes(
                cylinder,
                cylinderAxis,
                prism,
                prismPlanarAxis,
                prismAxis,
                prismNormal,
                ref penetration)
            || !CheckCylinderEmbeddedCapsuleEdgeAxis(
                cylinder,
                cylinderAxis,
                prism,
                prismPlanarAxis,
                prismAxis,
                ref penetration))
        {
            return false;
        }

        Vector3d closestFeatureAxis =
            FixedSegment.GetClosestDirectionBetweenCenteredAxes(
                cylinder.Center,
                cylinderAxis,
                cylinder.Height,
                GetEmbeddedCenter3D(prism),
                prism.AxisLength <= Fixed64.Epsilon
                    ? Vector3d.Up
                    : prismAxis,
                prism.AxisLength);
        if (!CheckCylinderPrismAxis(
                cylinder,
                prism,
                cylinderAxis,
                prismPlanarAxis,
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
        GetEmbeddedCapsuleAxes(
            prism,
            out Vector2d prismPlanarAxis,
            out Vector3d prismAxis,
            out Vector3d prismNormal);

        if (!CheckConePrismAxis(
                cone,
                prism,
                prismPlanarAxis,
                coneAxis,
                ref penetration))
            return false;

        CheckConePrismAxis(
            cone,
            prism,
            prismPlanarAxis,
            Vector3d.Up,
            ref penetration);

        if (!CheckConePrismAxis(
                cone,
                prism,
                prismPlanarAxis,
                Vector3d.Cross(coneAxis, Vector3d.Up),
                ref penetration))
            return false;

        if (!CheckEmbeddedCapsuleAxes(
                cone,
                prism,
                prismPlanarAxis,
                prismAxis,
                prismNormal,
                ref penetration)
            || !CheckConeEmbeddedCapsuleEdgeAxis(
                cone,
                coneAxis,
                prism,
                prismPlanarAxis,
                prismAxis,
                ref penetration))
        {
            return false;
        }

        ContactAnchor embeddedPoint =
            MixedEmbedded2DGeometry.GetClosestAnchorOnEmbeddedVolume(
                prism,
                cone.Center);
        // Earlier exact axes admit a shared finite feature neighborhood, so
        // each relative offset remains representable through the cone frame.
        _ = embeddedPoint.TryGetOffsetFrom(
            cone.Center,
            out Vector3d centerToEmbedded);
        _ = cone.Rotation.Inverse().TryRotate(
            centerToEmbedded,
            out Vector3d localEmbeddedPoint);
        _ = FixedSegment.TryGetClosestCenteredFiniteConeSurfaceOffset(
            localEmbeddedPoint,
            Vector3d.Zero,
            Vector3d.Up,
            cone.Height,
            cone.ScaledRadius,
            Vector3d.Right,
            out Vector3d coneSurfaceOffset,
            out _,
            out _);
        _ = embeddedPoint.TryGetOffsetFrom(
            new ContactAnchor(
                cone.Center,
                cone.Rotation,
                coneSurfaceOffset),
            out Vector3d closestFeatureAxis);
        if (!CheckConePrismAxis(
                cone,
                prism,
                prismPlanarAxis,
                closestFeatureAxis,
                ref penetration))
            return false;

        return penetration.HasValue;
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

    private static bool CheckCylinderPrismAxis(
        LSCylinderCollider cylinder,
        LSCapsuleCollider2D prism,
        Vector3d cylinderAxis,
        Vector2d prismAxis,
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
                prismAxis,
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
        Vector2d prismAxis,
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
                prismAxis,
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
        LSCylinderCollider cylinder,
        Vector3d cylinderAxis,
        LSCapsuleCollider2D capsule,
        Vector2d capsulePlanarAxis,
        Vector3d capsuleAxis,
        Vector3d capsuleNormal,
        ref AxisPenetration penetration)
    {
        return CheckCylinderPrismAxis(
                cylinder,
                capsule,
                cylinderAxis,
                capsulePlanarAxis,
                capsuleAxis,
                ref penetration)
            && CheckCylinderPrismAxis(
                cylinder,
                capsule,
                cylinderAxis,
                capsulePlanarAxis,
                capsuleNormal,
                ref penetration);
    }

    private static bool CheckEmbeddedCapsuleAxes(
        LSConeCollider cone,
        LSCapsuleCollider2D capsule,
        Vector2d capsulePlanarAxis,
        Vector3d capsuleAxis,
        Vector3d capsuleNormal,
        ref AxisPenetration penetration)
    {
        return CheckConePrismAxis(
                cone,
                capsule,
                capsulePlanarAxis,
                capsuleAxis,
                ref penetration)
            && CheckConePrismAxis(
                cone,
                capsule,
                capsulePlanarAxis,
                capsuleNormal,
                ref penetration);
    }

    private static bool CheckCylinderEmbeddedCapsuleEdgeAxis(
        LSCylinderCollider cylinder,
        Vector3d cylinderAxis,
        LSCapsuleCollider2D capsule,
        Vector2d capsulePlanarAxis,
        Vector3d capsuleAxis,
        ref AxisPenetration penetration)
    {
        return CheckCylinderPrismAxis(
            cylinder,
            capsule,
            cylinderAxis,
            capsulePlanarAxis,
            Vector3d.Cross(cylinderAxis, capsuleAxis),
            ref penetration);
    }

    private static bool CheckConeEmbeddedCapsuleEdgeAxis(
        LSConeCollider cone,
        Vector3d coneAxis,
        LSCapsuleCollider2D capsule,
        Vector2d capsulePlanarAxis,
        Vector3d capsuleAxis,
        ref AxisPenetration penetration)
    {
        return CheckConePrismAxis(
            cone,
            capsule,
            capsulePlanarAxis,
            Vector3d.Cross(coneAxis, capsuleAxis),
            ref penetration);
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
