using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Pool;
using SwiftCollections.Query;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    #region Compound

    private static bool DoCompoundCheck(CollisionWorkItem pair)
    {
        if (pair.ColliderA is LSCompoundCollider compoundA
            && pair.ColliderB is LSCompoundCollider compoundB)
        {
            return DoCompoundCompoundCheck(pair, compoundA, compoundB);
        }

        if (pair.ColliderA is LSCompoundCollider firstCompound)
            return DoCompoundOtherCheck(pair, firstCompound, pair.ColliderB);

        if (pair.ColliderB is LSCompoundCollider secondCompound)
            return DoCompoundOtherCheck(pair, secondCompound, pair.ColliderA);

        return false;
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
            collided |= TryBuildCompoundPartContact(pair, part, other);
        }

        return collided && pair.Manifold.HasContact;
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
                collided |= TryBuildCompoundPartContact(pair, partA, compoundB.GetPartCollider(j));
        }

        return collided && pair.Manifold.HasContact;
    }

    private static bool TryBuildCompoundPartContact(
        CollisionWorkItem ownerPair,
        LSCollider first,
        LSCollider second)
    {
        if (!BoundsOverlapInclusive(first, second))
            return false;

        OrderPartPairForDetection(first, second, out LSCollider colliderA, out LSCollider colliderB);
        CollisionType collisionType = ColliderSettings.GetCollisionType(colliderA.Shape, colliderB.Shape);
        if (collisionType == CollisionType.None || collisionType == CollisionType.Compound)
            return false;

        ContactManifold scratch = ownerPair.Context.CollisionScratch.CompoundPartManifold;
        scratch.BeginUpdate(ownerPair.Context.FrameCount);
        var partPair = new CollisionWorkItem(ownerPair.Context, colliderA, colliderB, collisionType, scratch);
        if (!DoCollisionCheck(partPair) || !scratch.HasContact)
            return false;

        AddCompoundPartContactsInOwnerOrder(ownerPair, partPair);
        return true;
    }

    private static void AddCompoundPartContactsInOwnerOrder(
        CollisionWorkItem ownerPair,
        CollisionWorkItem partPair)
    {
        bool addInPartOrder = BelongsToOwnerSide(partPair.ColliderA, ownerPair.ColliderA);
        bool flip = !addInPartOrder && BelongsToOwnerSide(partPair.ColliderA, ownerPair.ColliderB);
        if (!addInPartOrder && !flip)
            return;

        ContactManifold scratch = partPair.Manifold;
        for (int i = 0; i < scratch.Count; i++)
        {
            ManifoldContact contact = scratch[i];
            if (addInPartOrder)
            {
                ownerPair.Manifold.AddContact(contact.PointA, contact.PointB, contact.Depth, contact.Normal);
                continue;
            }

            ownerPair.Manifold.AddContact(contact.PointB, contact.PointA, contact.Depth, -contact.Normal);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void OrderPartPairForDetection(
        LSCollider first,
        LSCollider second,
        out LSCollider colliderA,
        out LSCollider colliderB)
    {
        if (first.Priority >= second.Priority)
        {
            colliderA = first;
            colliderB = second;
            return;
        }

        colliderA = second;
        colliderB = first;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool BelongsToOwnerSide(LSCollider candidate, LSCollider ownerSide)
    {
        if (ReferenceEquals(candidate, ownerSide))
            return true;

        return ownerSide is LSCompoundCollider compound && compound.ContainsPartCollider(candidate);
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
