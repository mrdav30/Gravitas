//=======================================================================
// CollisionDetection.Compound.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    #region Compound

    private static bool DoCompoundCheck(CollisionWorkItem pair)
    {
        var compoundA = (LSCompoundCollider)pair.ColliderA;
        if (pair.ColliderB is LSCompoundCollider compoundB)
        {
            return DoCompoundCompoundCheck(pair, compoundA, compoundB);
        }

        return DoCompoundOtherCheck(pair, compoundA, pair.ColliderB);
    }

    private static bool DoCompoundOtherCheck(
        CollisionWorkItem pair,
        LSCompoundCollider compound,
        LSCollider other)
    {
        bool collided = false;
        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider part = compound.GetPartCollider(i);
            collided |= TryBuildCompoundPartContact(
                pair,
                part,
                featureNamespaceA: i + 1,
                other,
                featureNamespaceB: 0);
        }

        return collided;
    }

    private static bool DoCompoundCompoundCheck(
        CollisionWorkItem pair,
        LSCompoundCollider compoundA,
        LSCompoundCollider compoundB)
    {
        bool collided = false;
        for (int i = 0; i < compoundA.PartCount; i++)
        {
            LSCollider partA = compoundA.GetPartCollider(i);
            for (int j = 0; j < compoundB.PartCount; j++)
            {
                collided |= TryBuildCompoundPartContact(
                    pair,
                    partA,
                    featureNamespaceA: i + 1,
                    compoundB.GetPartCollider(j),
                    featureNamespaceB: -(j + 1));
            }
        }

        return collided;
    }

    private static bool TryBuildCompoundPartContact(
        CollisionWorkItem ownerPair,
        LSCollider first,
        int featureNamespaceA,
        LSCollider second,
        int featureNamespaceB)
    {
        if (!BoundsOverlapInclusive(first, second))
            return false;

        OrderPartPairForDetection(
            first,
            featureNamespaceA,
            second,
            featureNamespaceB,
            out LSCollider colliderA,
            out int orderedNamespaceA,
            out LSCollider colliderB,
            out int orderedNamespaceB);
        CollisionType collisionType = ColliderSettings.GetCollisionType(colliderA.Shape, colliderB.Shape);
        ContactManifold scratch = ownerPair.Context.CollisionScratch.CompoundPartManifold;
        scratch.BeginUpdate(ownerPair.Context.FrameCount);
        var partPair = new CollisionWorkItem(ownerPair.Context, colliderA, colliderB, collisionType, scratch);
        if (!DoCollisionCheck(partPair))
            return false;

        AddCompoundPartContactsInOwnerOrder(
            ownerPair,
            partPair,
            orderedNamespaceA,
            orderedNamespaceB);
        return true;
    }

    private static void AddCompoundPartContactsInOwnerOrder(
        CollisionWorkItem ownerPair,
        CollisionWorkItem partPair,
        int featureNamespaceA,
        int featureNamespaceB)
    {
        bool addInPartOrder = ((LSCompoundCollider)ownerPair.ColliderA).ContainsPartCollider(partPair.ColliderA);

        ContactManifold scratch = partPair.Manifold;
        for (int i = 0; i < scratch.Count; i++)
        {
            ManifoldContact contact = scratch[i];
            if (addInPartOrder)
            {
                ownerPair.Manifold.AddContact(
                    contact.AnchorA,
                    contact.AnchorB,
                    contact.Depth,
                    contact.Normal,
                    partPair.ColliderA.Material,
                    partPair.ColliderB.Material,
                    contact.DepthIsClamped,
                    featureNamespaceA,
                    featureNamespaceB);
                continue;
            }

            ownerPair.Manifold.AddContact(
                contact.AnchorB,
                contact.AnchorA,
                contact.Depth,
                -contact.Normal,
                partPair.ColliderB.Material,
                partPair.ColliderA.Material,
                contact.DepthIsClamped,
                featureNamespaceB,
                featureNamespaceA);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void OrderPartPairForDetection(
        LSCollider first,
        int firstFeatureNamespace,
        LSCollider second,
        int secondFeatureNamespace,
        out LSCollider colliderA,
        out int featureNamespaceA,
        out LSCollider colliderB,
        out int featureNamespaceB)
    {
        if (first.Priority >= second.Priority)
        {
            colliderA = first;
            featureNamespaceA = firstFeatureNamespace;
            colliderB = second;
            featureNamespaceB = secondFeatureNamespace;
            return;
        }

        colliderA = second;
        featureNamespaceA = secondFeatureNamespace;
        colliderB = first;
        featureNamespaceB = firstFeatureNamespace;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool BoundsOverlapInclusive(LSCollider colliderA, LSCollider colliderB)
    {
        return colliderA.BoundsMin.X <= colliderB.BoundsMax.X
            && colliderA.BoundsMax.X >= colliderB.BoundsMin.X
            && colliderA.BoundsMin.Y <= colliderB.BoundsMax.Y
            && colliderA.BoundsMax.Y >= colliderB.BoundsMin.Y
            && colliderA.BoundsMin.Z <= colliderB.BoundsMax.Z
            && colliderA.BoundsMax.Z >= colliderB.BoundsMin.Z;
    }

    #endregion

}
