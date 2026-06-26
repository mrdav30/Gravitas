//=======================================================================
// GravitasPhysicsService.SupportTypes.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.CollisionHandling;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas;

public sealed partial class GravitasPhysicsService
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsMovableIslandBody(SolidBody? body) =>
        body != null && body.DynamicId >= 0 && body.CanTranslate;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasAwakeResponseParticipant(CollisionPair pair) =>
        IsAwakeIslandBody(pair.ColliderA.Body) || IsAwakeIslandBody(pair.ColliderB.Body);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAwakeIslandBody(SolidBody? body) =>
        IsMovableIslandBody(body) && body!.IsAwakeForCollision;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void GetStablePairKey(CollisionPair pair, out int minColliderId, out int maxColliderId)
    {
        int idA = pair.ColliderA.Id;
        int idB = pair.ColliderB.Id;
        if (idA <= idB)
        {
            minColliderId = idA;
            maxColliderId = idB;
            return;
        }

        minColliderId = idB;
        maxColliderId = idA;
    }

    private sealed class CollisionPairStableKeyComparer : IComparer<CollisionPair>
    {
        public int Compare(CollisionPair? left, CollisionPair? right)
        {
            if (ReferenceEquals(left, right))
                return 0;
            if (left == null)
                return -1;
            if (right == null)
                return 1;

            GetStablePairKey(left, out int leftMin, out int leftMax);
            GetStablePairKey(right, out int rightMin, out int rightMax);

            int compare = leftMin.CompareTo(rightMin);
            return compare != 0 ? compare : leftMax.CompareTo(rightMax);
        }
    }

    private sealed class DiscreteIslandNodeComparer : IComparer<DiscreteIslandNode>
    {
        public int Compare(DiscreteIslandNode left, DiscreteIslandNode right) =>
            left.BodyKey.CompareTo(right.BodyKey);
    }

    private sealed class DiscreteIslandConstraintComparer : IComparer<DiscreteIslandConstraint>
    {
        public int Compare(DiscreteIslandConstraint left, DiscreteIslandConstraint right)
        {
            int compare = left.RootKey.CompareTo(right.RootKey);
            if (compare != 0)
                return compare;

            compare = left.MinColliderId.CompareTo(right.MinColliderId);
            return compare != 0 ? compare : left.MaxColliderId.CompareTo(right.MaxColliderId);
        }
    }

    private struct DiscreteIslandNode
    {
        public DiscreteIslandNode(int bodyKey, SolidBody body)
        {
            BodyKey = bodyKey;
            Body = body;
            ParentIndex = -1;
            RootKey = bodyKey;
        }

        public int BodyKey;
        public SolidBody Body;
        public int ParentIndex;
        public int RootKey;
    }

    private readonly struct DiscreteIslandConstraint
    {
        public DiscreteIslandConstraint(
            CollisionPair pair,
            int rootKey,
            int minColliderId,
            int maxColliderId)
        {
            Pair = pair;
            RootKey = rootKey;
            MinColliderId = minColliderId;
            MaxColliderId = maxColliderId;
        }

        public CollisionPair Pair { get; }
        public int RootKey { get; }
        public int MinColliderId { get; }
        public int MaxColliderId { get; }
    }

    private int InactiveFrameThreshold => _context.FrameRate * 8;
}
