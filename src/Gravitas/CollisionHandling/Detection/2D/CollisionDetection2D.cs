//=======================================================================
// CollisionDetection2D.cs
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
/// Deterministic pure 2D narrow-phase collision checks.
/// </summary>
internal static class CollisionDetection2D
{
    internal static bool TryCollide(LSCollider2D colliderA, LSCollider2D colliderB, out Contact2D contact)
    {
        SwiftThrowHelper.ThrowIfNull(colliderA, nameof(colliderA));
        SwiftThrowHelper.ThrowIfNull(colliderB, nameof(colliderB));

        CollisionType2D collisionType = ColliderSettings2D.GetCollisionType(colliderA.Shape, colliderB.Shape);
        return TryCollide(new CollisionWorkItem2D(colliderA, colliderB, collisionType), out contact);
    }

    internal static bool TryCollide(CollisionPair2D pair, ContactManifold2D manifold, int frame)
    {
        SwiftThrowHelper.ThrowIfNull(pair, nameof(pair));
        return TryCollide(CollisionWorkItem2D.Create(pair), manifold, frame);
    }

    internal static bool TryCollide(CollisionWorkItem2D item, ContactManifold2D manifold, int frame)
    {
        SwiftThrowHelper.ThrowIfNull(manifold, nameof(manifold));
        manifold.BeginUpdate(frame);
        return TryCollide(item, manifold);
    }

    internal static bool TryCollide(CollisionWorkItem2D item, out Contact2D contact)
    {
        LSCollider2D colliderA = item.ColliderA;
        LSCollider2D colliderB = item.ColliderB;
        if (!BoundsOverlap(colliderA, colliderB))
        {
            contact = default;
            return false;
        }

        switch (item.CollisionType)
        {
            case CollisionType2D.Circle_Circle:
                return TryCircleCircle((LSCircleCollider2D)colliderA, (LSCircleCollider2D)colliderB, out contact);
            case CollisionType2D.Circle_Convex:
                return TryCircleConvex((LSCircleCollider2D)colliderA, colliderB, out contact);
            case CollisionType2D.Convex_Circle:
                bool result = TryCircleConvex((LSCircleCollider2D)colliderB, colliderA, out Contact2D reversed);
                contact = result
                    ? new Contact2D(
                        reversed.AnchorB,
                        reversed.AnchorA,
                        -reversed.Normal,
                        reversed.Depth,
                        reversed.DepthIsClamped)
                    : default;
                return result;
            case CollisionType2D.Convex_Convex:
                return TryConvexConvex(colliderA, colliderB, out contact);
            case CollisionType2D.Capsule_Circle:
                return TryCapsuleCircle((LSCapsuleCollider2D)colliderA, (LSCircleCollider2D)colliderB, out contact);
            case CollisionType2D.Circle_Capsule:
                bool circleCapsule = TryCapsuleCircle((LSCapsuleCollider2D)colliderB, (LSCircleCollider2D)colliderA, out Contact2D circleCapsuleReversed);
                contact = circleCapsule
                    ? new Contact2D(
                        circleCapsuleReversed.AnchorB,
                        circleCapsuleReversed.AnchorA,
                        -circleCapsuleReversed.Normal,
                        circleCapsuleReversed.Depth,
                        circleCapsuleReversed.DepthIsClamped)
                    : default;
                return circleCapsule;
            case CollisionType2D.Capsule_Convex:
                return TryCapsuleConvex((LSCapsuleCollider2D)colliderA, colliderB, out contact);
            case CollisionType2D.Convex_Capsule:
                bool convexCapsule = TryCapsuleConvex((LSCapsuleCollider2D)colliderB, colliderA, out Contact2D convexCapsuleReversed);
                contact = convexCapsule
                    ? new Contact2D(
                        convexCapsuleReversed.AnchorB,
                        convexCapsuleReversed.AnchorA,
                        -convexCapsuleReversed.Normal,
                        convexCapsuleReversed.Depth,
                        convexCapsuleReversed.DepthIsClamped)
                    : default;
                return convexCapsule;
            case CollisionType2D.Capsule_Capsule:
                return TryCapsuleCapsule((LSCapsuleCollider2D)colliderA, (LSCapsuleCollider2D)colliderB, out contact);
            case CollisionType2D.Compound:
                return TryCompound(colliderA, colliderB, out contact);
            default:
                contact = default;
                return false;
        }
    }

    private static bool TryCollide(CollisionWorkItem2D item, ContactManifold2D manifold)
    {
        LSCollider2D colliderA = item.ColliderA;
        LSCollider2D colliderB = item.ColliderB;
        if (!BoundsOverlap(colliderA, colliderB))
            return false;

        return item.CollisionType switch
        {
            CollisionType2D.Circle_Circle => TryCircleCircle((LSCircleCollider2D)colliderA, (LSCircleCollider2D)colliderB, manifold),
            CollisionType2D.Circle_Convex => TryCircleConvex((LSCircleCollider2D)colliderA, colliderB, manifold),
            CollisionType2D.Convex_Circle => TryCircleConvexReversed((LSCircleCollider2D)colliderB, colliderA, manifold),
            CollisionType2D.Convex_Convex => TryConvexConvex(colliderA, colliderB, manifold),
            CollisionType2D.Capsule_Circle => TryCapsuleCircle((LSCapsuleCollider2D)colliderA, (LSCircleCollider2D)colliderB, manifold),
            CollisionType2D.Circle_Capsule => TryCapsuleCircleReversed((LSCapsuleCollider2D)colliderB, (LSCircleCollider2D)colliderA, manifold),
            CollisionType2D.Capsule_Convex => TryCapsuleConvex((LSCapsuleCollider2D)colliderA, colliderB, manifold),
            CollisionType2D.Convex_Capsule => TryCapsuleConvexReversed((LSCapsuleCollider2D)colliderB, colliderA, manifold),
            CollisionType2D.Capsule_Capsule => TryCapsuleCapsule((LSCapsuleCollider2D)colliderA, (LSCapsuleCollider2D)colliderB, manifold),
            CollisionType2D.Compound => TryCompound(colliderA, colliderB, manifold),
            _ => false
        };
    }

    private static bool TryCircleCircle(LSCircleCollider2D colliderA, LSCircleCollider2D colliderB, out Contact2D contact)
    {
        Vector2d delta = colliderB.Center - colliderA.Center;
        Fixed64 radius = colliderA.ScaledRadius + colliderB.ScaledRadius;
        Fixed64 distanceSquared = delta.MagnitudeSquared;
        if (distanceSquared > radius * radius)
        {
            contact = default;
            return false;
        }

        Fixed64 distance = distanceSquared > Fixed64.Zero ? FixedMath.Sqrt(distanceSquared) : Fixed64.Zero;
        Vector2d normal = distance > Fixed64.Zero ? delta / distance : Vector2d.Right;
        Fixed64 depth = radius - distance;
        contact = new Contact2D(
            new ContactAnchor2D(
                colliderA.Center,
                normal * colliderA.ScaledRadius),
            new ContactAnchor2D(
                colliderB.Center,
                -normal * colliderB.ScaledRadius),
            normal,
            depth);
        return true;
    }

    private static bool TryCircleCircle(
        LSCircleCollider2D colliderA,
        LSCircleCollider2D colliderB,
        ContactManifold2D manifold)
    {
        if (!TryCircleCircle(colliderA, colliderB, out Contact2D contact))
            return false;

        AddContact(manifold, contact, colliderA, colliderB);
        return true;
    }

    private static bool TryCircleConvex(LSCircleCollider2D circle, LSCollider2D convex, out Contact2D contact)
    {
        Span<Vector2d> convexOffsets = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> convexVertexOffsets =
            GetConvexVertexOffsets(convex, convexOffsets);
        if (!FixedConvex2dRelations.TryGetCircleContact(
                circle.Center,
                circle.Rotation,
                circle.ScaledRadius,
                convex.Center,
                convex.ConvexRotation,
                convexVertexOffsets,
                out FixedPointAnchor2d circleAnchor,
                out FixedPointAnchor2d convexAnchor,
                out Vector2d normal,
                out Fixed64 depth,
                out bool depthIsClamped))
        {
            contact = default;
            return false;
        }

        contact = new Contact2D(
            new ContactAnchor2D(circleAnchor),
            new ContactAnchor2D(convexAnchor),
            normal,
            depth,
            depthIsClamped);
        return true;
    }

    private static bool TryCircleConvex(
        LSCircleCollider2D circle,
        LSCollider2D convex,
        ContactManifold2D manifold)
    {
        if (!TryCircleConvex(circle, convex, out Contact2D contact))
            return false;

        AddContact(manifold, contact, circle, convex);
        return true;
    }

    private static bool TryCircleConvexReversed(
        LSCircleCollider2D circle,
        LSCollider2D convex,
        ContactManifold2D manifold)
    {
        if (!TryCircleConvex(circle, convex, out Contact2D contact))
            return false;

        manifold.AddContact(
            contact.AnchorB,
            contact.AnchorA,
            contact.Depth,
            -contact.Normal,
            convex.Material,
            circle.Material,
            contact.DepthIsClamped);
        return true;
    }

    private static bool TryCapsuleCircle(LSCapsuleCollider2D capsule, LSCircleCollider2D circle, out Contact2D contact)
    {
        Vector2d fallbackNormal = capsule.GetNormalFromCenteredAxis(circle.Center);
        if (!FixedSegment2d.TryGetCenteredCapsulesContact(
                capsule.Center,
                capsule.Rotation,
                capsule.AxisLength,
                capsule.ScaledRadius,
                circle.Center,
                circle.Rotation,
                Fixed64.Zero,
                circle.ScaledRadius,
                fallbackNormal,
                out FixedContactAnchors2d fixedContact))
        {
            contact = default;
            return false;
        }

        contact = new Contact2D(
            new ContactAnchor2D(fixedContact.FirstAnchor),
            new ContactAnchor2D(fixedContact.SecondAnchor),
            fixedContact.Normal,
            fixedContact.Depth,
            fixedContact.DepthIsClamped);
        return true;
    }

    private static bool TryCapsuleCircle(
        LSCapsuleCollider2D capsule,
        LSCircleCollider2D circle,
        ContactManifold2D manifold)
    {
        if (!TryCapsuleCircle(capsule, circle, out Contact2D contact))
            return false;

        AddContact(manifold, contact, capsule, circle);
        return true;
    }

    private static bool TryCapsuleCircleReversed(
        LSCapsuleCollider2D capsule,
        LSCircleCollider2D circle,
        ContactManifold2D manifold)
    {
        if (!TryCapsuleCircle(capsule, circle, out Contact2D contact))
            return false;

        manifold.AddContact(
            contact.AnchorB,
            contact.AnchorA,
            contact.Depth,
            -contact.Normal,
            circle.Material,
            capsule.Material,
            contact.DepthIsClamped);
        return true;
    }

    private static bool TryCapsuleCapsule(LSCapsuleCollider2D colliderA, LSCapsuleCollider2D colliderB, out Contact2D contact)
    {
        Vector2d fallbackNormal = colliderB.AxisLength <= Fixed64.Epsilon
            ? colliderA.GetNormalFromCenteredAxis(colliderB.Center)
            : colliderA.AxisLength <= Fixed64.Epsilon
                ? -colliderB.GetNormalFromCenteredAxis(colliderA.Center)
                : OrientCoincidentCapsuleNormal(colliderB.Center - colliderA.Center);
        if (!FixedSegment2d.TryGetCenteredCapsulesContact(
                colliderA.Center,
                colliderA.Rotation,
                colliderA.AxisLength,
                colliderA.ScaledRadius,
                colliderB.Center,
                colliderB.Rotation,
                colliderB.AxisLength,
                colliderB.ScaledRadius,
                fallbackNormal,
                out FixedContactAnchors2d fixedContact))
        {
            contact = default;
            return false;
        }

        contact = new Contact2D(
            new ContactAnchor2D(fixedContact.FirstAnchor),
            new ContactAnchor2D(fixedContact.SecondAnchor),
            fixedContact.Normal,
            fixedContact.Depth,
            fixedContact.DepthIsClamped);
        return true;
    }

    private static bool TryCapsuleCapsule(
        LSCapsuleCollider2D colliderA,
        LSCapsuleCollider2D colliderB,
        ContactManifold2D manifold)
    {
        if (!TryCapsuleCapsule(colliderA, colliderB, out Contact2D contact))
            return false;

        AddContact(manifold, contact, colliderA, colliderB);
        return true;
    }

    private static bool TryCapsuleConvex(LSCapsuleCollider2D capsule, LSCollider2D convex, out Contact2D contact)
    {
        Span<FixedPointAnchor2d> capsuleAnchors =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexAnchors =
            stackalloc FixedPointAnchor2d[2];
        if (!TryGetCapsuleConvexContacts(
                capsule,
                convex,
                capsuleAnchors,
                convexAnchors,
                out _,
                out Vector2d normal,
                out Fixed64 depth,
                out bool depthIsClamped))
        {
            contact = default;
            return false;
        }

        contact = new Contact2D(
            new ContactAnchor2D(capsuleAnchors[0]),
            new ContactAnchor2D(convexAnchors[0]),
            normal,
            depth,
            depthIsClamped);
        return true;
    }

    private static bool TryGetCapsuleConvexContacts(
        LSCapsuleCollider2D capsule,
        LSCollider2D convex,
        Span<FixedPointAnchor2d> capsuleAnchors,
        Span<FixedPointAnchor2d> convexAnchors,
        out int contactCount,
        out Vector2d normal,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        Span<Vector2d> vertexOffsets = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> convexVertexOffsets =
            GetConvexVertexOffsets(convex, vertexOffsets);
        return FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
            capsule.Center,
            capsule.Rotation,
            Vector2d.Forward,
            capsule.AxisLength,
            capsule.ScaledRadius,
            convex.Center,
            convex.ConvexRotation,
            convexVertexOffsets,
            capsuleAnchors,
            convexAnchors,
            out contactCount,
            out normal,
            out depth,
            out depthIsClamped);
    }

    private static ReadOnlySpan<Vector2d> GetConvexVertexOffsets(
        LSCollider2D convex,
        Span<Vector2d> scratch)
    {
        if (convex is LSPolygonCollider2D polygon)
            return polygon.ScaledLocalVertices;

        int vertexCount = convex.VertexCount;
        for (int i = 0; i < vertexCount; i++)
            scratch[i] = convex.GetScaledLocalVertexUnchecked(i);
        return scratch.Slice(0, vertexCount);
    }

    private static bool TryCapsuleConvex(
        LSCapsuleCollider2D capsule,
        LSCollider2D convex,
        ContactManifold2D manifold)
    {
        Span<FixedPointAnchor2d> capsuleAnchors =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexAnchors =
            stackalloc FixedPointAnchor2d[2];
        if (!TryGetCapsuleConvexContacts(
                capsule,
                convex,
                capsuleAnchors,
                convexAnchors,
                out int contactCount,
                out Vector2d normal,
                out Fixed64 depth,
                out bool depthIsClamped))
        {
            return false;
        }

        for (int i = 0; i < contactCount; i++)
        {
            manifold.AddContact(
                new ContactAnchor2D(capsuleAnchors[i]),
                new ContactAnchor2D(convexAnchors[i]),
                depth,
                normal,
                capsule.Material,
                convex.Material,
                depthIsClamped);
        }
        return true;
    }

    private static bool TryCapsuleConvexReversed(
        LSCapsuleCollider2D capsule,
        LSCollider2D convex,
        ContactManifold2D manifold)
    {
        Span<FixedPointAnchor2d> capsuleAnchors =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexAnchors =
            stackalloc FixedPointAnchor2d[2];
        if (!TryGetCapsuleConvexContacts(
                capsule,
                convex,
                capsuleAnchors,
                convexAnchors,
                out int contactCount,
                out Vector2d normal,
                out Fixed64 depth,
                out bool depthIsClamped))
        {
            return false;
        }

        for (int i = 0; i < contactCount; i++)
        {
            manifold.AddContact(
                new ContactAnchor2D(convexAnchors[i]),
                new ContactAnchor2D(capsuleAnchors[i]),
                depth,
                -normal,
                convex.Material,
                capsule.Material,
                depthIsClamped);
        }
        return true;
    }

    private static bool TryConvexConvex(LSCollider2D colliderA, LSCollider2D colliderB, out Contact2D contact)
    {
        Span<Vector2d> firstVertexScratch = stackalloc Vector2d[4];
        Span<Vector2d> secondVertexScratch = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> firstVertexOffsets =
            GetConvexVertexOffsets(colliderA, firstVertexScratch);
        ReadOnlySpan<Vector2d> secondVertexOffsets =
            GetConvexVertexOffsets(colliderB, secondVertexScratch);
        Span<FixedPointAnchor2d> firstContactAnchors =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> secondContactAnchors =
            stackalloc FixedPointAnchor2d[2];
        if (!FixedConvex2dRelations.TryGetConvexContacts(
                colliderA.Center,
                colliderA.ConvexRotation,
                firstVertexOffsets,
                colliderB.Center,
                colliderB.ConvexRotation,
                secondVertexOffsets,
                firstContactAnchors,
                secondContactAnchors,
                out _,
                out Vector2d normal,
                out Fixed64 depth,
                out bool depthIsClamped))
        {
            contact = default;
            return false;
        }

        contact = new Contact2D(
            new ContactAnchor2D(colliderA.GetConvexSupportAnchor(normal)),
            new ContactAnchor2D(colliderB.GetConvexSupportAnchor(-normal)),
            normal,
            depth,
            depthIsClamped);
        return true;
    }

    private static bool TryConvexConvex(
        LSCollider2D colliderA,
        LSCollider2D colliderB,
        ContactManifold2D manifold)
    {
        Span<Vector2d> firstVertexScratch = stackalloc Vector2d[4];
        Span<Vector2d> secondVertexScratch = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> firstVertexOffsets =
            GetConvexVertexOffsets(colliderA, firstVertexScratch);
        ReadOnlySpan<Vector2d> secondVertexOffsets =
            GetConvexVertexOffsets(colliderB, secondVertexScratch);
        Span<FixedPointAnchor2d> firstContactAnchors =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> secondContactAnchors =
            stackalloc FixedPointAnchor2d[2];
        if (!FixedConvex2dRelations.TryGetConvexContacts(
                colliderA.Center,
                colliderA.ConvexRotation,
                firstVertexOffsets,
                colliderB.Center,
                colliderB.ConvexRotation,
                secondVertexOffsets,
                firstContactAnchors,
                secondContactAnchors,
                out int contactCount,
                out Vector2d normal,
                out Fixed64 depth,
                out bool depthIsClamped))
        {
            return false;
        }

        for (int i = 0; i < contactCount; i++)
        {
            manifold.AddContact(
                new ContactAnchor2D(firstContactAnchors[i]),
                new ContactAnchor2D(secondContactAnchors[i]),
                depth,
                normal,
                colliderA.Material,
                colliderB.Material,
                depthIsClamped);
        }

        return true;
    }

    private static bool TryCompound(LSCollider2D colliderA, LSCollider2D colliderB, out Contact2D contact)
    {
        if (colliderA is LSCompoundCollider2D compoundA)
        {
            if (colliderB is LSCompoundCollider2D compoundB)
                return TryCompoundCompound(compoundA, compoundB, out contact);

            return TryCompoundOther(compoundA, colliderB, compoundIsA: true, out contact);
        }

        if (colliderB is LSCompoundCollider2D compound)
            return TryCompoundOther(compound, colliderA, compoundIsA: false, out contact);

        contact = default;
        return false;
    }

    private static bool TryCompoundCompound(
        LSCompoundCollider2D compoundA,
        LSCompoundCollider2D compoundB,
        out Contact2D contact)
    {
        bool found = false;
        Contact2D best = default;

        for (int i = 0; i < compoundA.PartCount; i++)
        {
            LSCollider2D partA = compoundA.GetPartCollider(i);
            for (int j = 0; j < compoundB.PartCount; j++)
            {
                LSCollider2D partB = compoundB.GetPartCollider(j);
                if (!TryCollide(partA, partB, out Contact2D candidate))
                    continue;

                if (ContactSelectionPolicy.ShouldReplaceWithDeeper(candidate, found, best))
                {
                    best = candidate;
                    found = true;
                }
            }
        }

        if (!found)
        {
            contact = default;
            return false;
        }

        contact = best;
        return true;
    }

    private static bool TryCompoundOther(
        LSCompoundCollider2D compound,
        LSCollider2D other,
        bool compoundIsA,
        out Contact2D contact)
    {
        bool found = false;
        Contact2D best = default;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            Contact2D candidate;
            bool collided = compoundIsA
                ? TryCollide(part, other, out candidate)
                : TryCollide(other, part, out candidate);

            if (!collided)
                continue;

            if (ContactSelectionPolicy.ShouldReplaceWithDeeper(candidate, found, best))
            {
                best = candidate;
                found = true;
            }
        }

        if (!found)
        {
            contact = default;
            return false;
        }

        contact = best;
        return true;
    }

    private static bool TryCompound(
        LSCollider2D colliderA,
        LSCollider2D colliderB,
        ContactManifold2D manifold)
    {
        if (colliderA is LSCompoundCollider2D compoundA)
        {
            if (colliderB is LSCompoundCollider2D compoundB)
                return TryCompoundCompound(compoundA, compoundB, manifold);

            return TryCompoundOther(compoundA, colliderB, compoundIsA: true, manifold);
        }

        if (colliderB is LSCompoundCollider2D compound)
            return TryCompoundOther(compound, colliderA, compoundIsA: false, manifold);

        return false;
    }

    private static bool TryCompoundCompound(
        LSCompoundCollider2D compoundA,
        LSCompoundCollider2D compoundB,
        ContactManifold2D manifold)
    {
        bool found = false;
        ContactManifold2D scratch = compoundA.PartManifoldScratch;
        for (int i = 0; i < compoundA.PartCount; i++)
        {
            LSCollider2D partA = compoundA.GetPartCollider(i);
            for (int j = 0; j < compoundB.PartCount; j++)
            {
                LSCollider2D partB = compoundB.GetPartCollider(j);
                CollisionType2D collisionType = ColliderSettings2D.GetCollisionType(partA.Shape, partB.Shape);
                scratch.BeginUpdate(manifold.LastUpdatedFrame);
                if (!TryCollide(new CollisionWorkItem2D(partA, partB, collisionType), scratch))
                    continue;

                found = true;
                AddCompoundPartContacts(
                    manifold,
                    scratch,
                    featureNamespaceA: i + 1,
                    featureNamespaceB: -(j + 1));
            }
        }

        return found;
    }

    private static bool TryCompoundOther(
        LSCompoundCollider2D compound,
        LSCollider2D other,
        bool compoundIsA,
        ContactManifold2D manifold)
    {
        bool found = false;
        ContactManifold2D scratch = compound.PartManifoldScratch;
        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            LSCollider2D colliderA = compoundIsA ? part : other;
            LSCollider2D colliderB = compoundIsA ? other : part;
            CollisionType2D collisionType = ColliderSettings2D.GetCollisionType(colliderA.Shape, colliderB.Shape);
            scratch.BeginUpdate(manifold.LastUpdatedFrame);
            if (!TryCollide(new CollisionWorkItem2D(colliderA, colliderB, collisionType), scratch))
                continue;

            found = true;
            AddCompoundPartContacts(
                manifold,
                scratch,
                featureNamespaceA: compoundIsA ? i + 1 : 0,
                featureNamespaceB: compoundIsA ? 0 : -(i + 1));
        }

        return found;
    }

    private static void AddCompoundPartContacts(
        ContactManifold2D destination,
        ContactManifold2D source,
        int featureNamespaceA,
        int featureNamespaceB)
    {
        for (int i = 0; i < source.Count; i++)
        {
            ManifoldContact2D contact = source[i];
            destination.AddContact(
                contact.AnchorA,
                contact.AnchorB,
                contact.Depth,
                contact.Normal,
                contact.MaterialA,
                contact.MaterialB,
                contact.DepthIsClamped,
                featureNamespaceA,
                featureNamespaceB);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddContact(
        ContactManifold2D manifold,
        Contact2D contact,
        LSCollider2D colliderA,
        LSCollider2D colliderB) =>
        manifold.AddContact(
            contact.AnchorA,
            contact.AnchorB,
            contact.Depth,
            contact.Normal,
            colliderA.Material,
            colliderB.Material,
            contact.DepthIsClamped);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool BoundsOverlap(LSCollider2D colliderA, LSCollider2D colliderB) =>
        colliderA.MinX <= colliderB.MaxX
        && colliderA.MaxX >= colliderB.MinX
        && colliderA.MinY <= colliderB.MaxY
        && colliderA.MaxY >= colliderB.MinY;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d OrientCoincidentCapsuleNormal(Vector2d direction) =>
        direction.MagnitudeSquared > Fixed64.Epsilon && direction.X < Fixed64.Zero
            ? -Vector2d.Right
            : Vector2d.Right;

}
