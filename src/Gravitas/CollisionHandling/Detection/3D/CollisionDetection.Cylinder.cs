//=======================================================================
// CollisionDetection.Cylinder.cs
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

public static partial class CollisionDetection
{
    #region Cylinder

    private static bool DoCylinderSphereCheck(CollisionWorkItem pair)
    {
        var cylinder = (LSCylinderCollider)pair.ColliderA;
        var sphere = (LSSphereCollider)pair.ColliderB;

        if (!FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                sphere.Center,
                cylinder.Center,
                cylinder.Rotation,
                Vector3d.Up,
                cylinder.Height,
                cylinder.ScaledRadius,
                Vector3d.Right,
                out FixedPointAnchor cylinderAnchor,
                out Vector3d outwardNormal,
                out Fixed64 signedDistance))
        {
            return false;
        }
        if (signedDistance > sphere.ScaledRadius)
            return false;

        Vector3d normal = signedDistance < Fixed64.Zero
            ? -outwardNormal
            : outwardNormal;
        ResolvePenetrationDepth(
            sphere.ScaledRadius,
            signedDistance,
            out Fixed64 penetrationDepth,
            out bool depthIsClamped);
        pair.Manifold.SetContact(
            new ContactAnchor(cylinderAnchor),
            new ContactAnchor(
                FixedSegment.GetCenteredCapsuleSupportAnchor(
                    sphere.Center,
                    sphere.Rotation,
                    Fixed64.Zero,
                    sphere.ScaledRadius,
                    -normal)),
            penetrationDepth,
            normal,
            depthIsClamped);

        return true;
    }

    private static bool DoCylinderCapsuleCheck(CollisionWorkItem pair)
    {
        LSCylinderCollider cylinder;
        LSCapsuleCollider capsule;
        if (pair.ColliderA is LSCylinderCollider cylinderA)
        {
            cylinder = cylinderA;
            capsule = (LSCapsuleCollider)pair.ColliderB;
        }
        else
        {
            cylinder = (LSCylinderCollider)pair.ColliderB;
            capsule = (LSCapsuleCollider)pair.ColliderA;
        }

        if (!FixedSegment.TryGetCenteredFiniteCylinderCapsuleContact(
                cylinder.Center,
                cylinder.Rotation,
                Vector3d.Up,
                cylinder.Height,
                cylinder.ScaledRadius,
                capsule.Center,
                capsule.Rotation,
                Vector3d.Up,
                capsule.AxisLength,
                capsule.ScaledRadius,
                out FixedContactAnchors contact))
        {
            return false;
        }

        SetContactInPairOrder(
            pair,
            cylinder,
            new ContactAnchor(contact.FirstAnchor),
            capsule,
            new ContactAnchor(contact.SecondAnchor),
            contact.Depth,
            contact.Normal,
            contact.DepthIsClamped);

        return true;
    }

    private static bool DoCylindersCheck(CollisionWorkItem pair)
    {
        var cylinderA = (LSCylinderCollider)pair.ColliderA;
        var cylinderB = (LSCylinderCollider)pair.ColliderB;

        if (!FixedSegment.TryGetCenteredFiniteCylindersContact(
                cylinderA.Center,
                cylinderA.Rotation,
                Vector3d.Up,
                cylinderA.Height,
                cylinderA.ScaledRadius,
                cylinderB.Center,
                cylinderB.Rotation,
                Vector3d.Up,
                cylinderB.Height,
                cylinderB.ScaledRadius,
                out FixedContactAnchors contact))
        {
            return false;
        }

        if (TryAddCylinderCylinderCapContacts(
                pair,
                cylinderA,
                cylinderB,
                contact))
        {
            return true;
        }

        pair.Manifold.SetContact(
            new ContactAnchor(contact.FirstAnchor),
            new ContactAnchor(contact.SecondAnchor),
            contact.Depth,
            contact.Normal,
            contact.DepthIsClamped);

        return true;
    }

    private static bool DoCuboidCylinderCheck(CollisionWorkItem pair)
    {
        var cuboid = (LSCuboidCollider)pair.ColliderA;
        var cylinder = (LSCylinderCollider)pair.ColliderB;

        Span<FixedContactLocalPoints> capFaceContacts =
            stackalloc FixedContactLocalPoints[ContactManifold.MaxContactCount];
        if (!cuboid.OrientedBox.TryGetCenteredCylinderContact(
                cylinder.Center,
                cylinder.Rotation,
                Vector3d.Up,
                cylinder.Height,
                cylinder.ScaledRadius,
                capFaceContacts,
                out FixedContactAnchors contact,
                out int capFaceContactCount))
        {
            return false;
        }

        if (capFaceContactCount > 0)
        {
            for (int index = 0; index < capFaceContactCount; index++)
            {
                FixedContactLocalPoints capContact = capFaceContacts[index];
                pair.Manifold.AddContact(
                    new ContactAnchor(
                        contact.FirstAnchor.Origin,
                        contact.FirstAnchor.Rotation,
                        capContact.FirstLocalPoint),
                    new ContactAnchor(
                        contact.SecondAnchor.Origin,
                        contact.SecondAnchor.Rotation,
                        capContact.SecondLocalPoint),
                    contact.Depth,
                    contact.Normal,
                    contact.DepthIsClamped);
            }
            return true;
        }

        pair.Manifold.SetContact(
            new ContactAnchor(contact.FirstAnchor),
            new ContactAnchor(contact.SecondAnchor),
            contact.Depth,
            contact.Normal,
            contact.DepthIsClamped);
        return true;
    }

    private static bool TryAddCylinderCylinderCapContacts(
        CollisionWorkItem pair,
        LSCylinderCollider cylinderA,
        LSCylinderCollider cylinderB,
        FixedContactAnchors contact)
    {
        if (contact.FirstAnchor.LocalDisplacement != Vector3d.Zero
            || contact.SecondAnchor.LocalDisplacement != Vector3d.Zero)
        {
            return false;
        }

        CylinderContactGeometry.GetCapBasis(cylinderA, out Vector3d tangentA, out Vector3d tangentB);
        int initialCount = pair.Manifold.Count;
        TryAddCylinderCylinderCapContact(pair, cylinderA, cylinderB, tangentA, contact);
        TryAddCylinderCylinderCapContact(pair, cylinderA, cylinderB, -tangentA, contact);
        TryAddCylinderCylinderCapContact(pair, cylinderA, cylinderB, tangentB, contact);
        TryAddCylinderCylinderCapContact(pair, cylinderA, cylinderB, -tangentB, contact);
        return pair.Manifold.Count > initialCount;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void TryAddCylinderCylinderCapContact(
        CollisionWorkItem pair,
        LSCylinderCollider cylinderA,
        LSCylinderCollider cylinderB,
        Vector3d radialDirection,
        FixedContactAnchors contact)
    {
        pair.Manifold.AddContact(
            new ContactAnchor(ConvexColliderSupport.GetSupportAnchor(
                cylinderA,
                contact.Normal + radialDirection,
                Vector3d.Zero)),
            new ContactAnchor(ConvexColliderSupport.GetSupportAnchor(
                cylinderB,
                -contact.Normal + radialDirection,
                Vector3d.Zero)),
            contact.Depth,
            contact.Normal,
            contact.DepthIsClamped);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ResolvePenetrationDepth(
        Fixed64 radius,
        Fixed64 signedDistance,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        if (Fixed64.TrySubtract(radius, signedDistance, out depth))
        {
            depthIsClamped = false;
            return;
        }

        depth = Fixed64.MaxValue;
        depthIsClamped = true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveNormal(Vector3d delta) =>
        delta.MagnitudeSquared > Fixed64.Epsilon
            ? delta.Normalized
            : Vector3d.Right;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SetContactInPairOrder(
        CollisionWorkItem pair,
        LSCollider first,
        ContactAnchor anchorOnFirst,
        LSCollider second,
        ContactAnchor anchorOnSecond,
        Fixed64 depth,
        Vector3d normalFirstToSecond,
        bool depthIsClamped)
    {
        if (ReferenceEquals(pair.ColliderA, first))
        {
            pair.Manifold.SetContact(
                anchorOnFirst,
                anchorOnSecond,
                depth,
                normalFirstToSecond,
                depthIsClamped);
            return;
        }

        pair.Manifold.SetContact(
            anchorOnSecond,
            anchorOnFirst,
            depth,
            -normalFirstToSecond,
            depthIsClamped);
    }

    #endregion

}
