//=======================================================================
// GravitasPhysics2DService.SupportTypes.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using GridForge.Spatial;
using SwiftCollections;
using SwiftCollections.Query;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas;

public sealed partial class GravitasPhysics2DService
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasAwakeMovableParticipant(LSCollider2D first, LSCollider2D second) =>
        IsAwakeMovable(first.Body) || IsAwakeMovable(second.Body);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasAwakeResponseParticipant(CollisionPair2D pair) =>
        IsAwakeIslandBody(pair.ColliderA.Body) || IsAwakeIslandBody(pair.ColliderB.Body);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAwakeMovable(StiffBody2D? body) =>
        body != null && body.CanTranslate && !body.IsSleeping;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsMovableIslandBody(StiffBody2D? body) =>
        body != null && body.DynamicId >= 0 && body.CanTranslate;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAwakeIslandBody(StiffBody2D? body) =>
        IsMovableIslandBody(body) && body!.IsAwakeForCollision;

    private bool IsLayerCollisionDisabled(PhysicsLayer layer1, PhysicsLayer layer2)
    {
        bool[,] matrix = _context.Settings.CollisionMatrix;
        int layerIndex1 = layer1.Index;
        int layerIndex2 = layer2.Index;
        if (layerIndex1 >= matrix.GetLength(0) || layerIndex2 >= matrix.GetLength(1))
            return false;

        return !matrix[layerIndex1, layerIndex2];
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void RemovePairReferences(CollisionPair2D pair)
    {
        pair.ColliderA.TryRemoveCollisionPair(pair.Id2, out _);
        pair.ColliderB.TryRemoveCollisionPairHolder(pair.Id1);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong CreatePairKey(int firstId, int secondId)
    {
        if (firstId > secondId)
            (firstId, secondId) = (secondId, firstId);

        return ((ulong)(uint)firstId << 32) | (uint)secondId;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void GetStablePairKey(CollisionPair2D pair, out int minColliderId, out int maxColliderId)
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

    private sealed class CollisionPair2DStableKeyComparer : IComparer<CollisionPair2D>
    {
        public int Compare(CollisionPair2D? left, CollisionPair2D? right)
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

    private sealed class DiscreteIslandNode2DComparer : IComparer<DiscreteIslandNode2D>
    {
        public int Compare(DiscreteIslandNode2D left, DiscreteIslandNode2D right) =>
            left.BodyKey.CompareTo(right.BodyKey);
    }

    private sealed class DiscreteIslandConstraint2DComparer : IComparer<DiscreteIslandConstraint2D>
    {
        public int Compare(DiscreteIslandConstraint2D left, DiscreteIslandConstraint2D right)
        {
            int compare = left.RootKey.CompareTo(right.RootKey);
            if (compare != 0)
                return compare;

            compare = left.MinColliderId.CompareTo(right.MinColliderId);
            return compare != 0 ? compare : left.MaxColliderId.CompareTo(right.MaxColliderId);
        }
    }

    private struct DiscreteIslandNode2D
    {
        public DiscreteIslandNode2D(int bodyKey, StiffBody2D body)
        {
            BodyKey = bodyKey;
            Body = body;
            ParentIndex = -1;
            RootKey = bodyKey;
        }

        public int BodyKey;
        public StiffBody2D Body;
        public int ParentIndex;
        public int RootKey;
    }

    private readonly struct DiscreteIslandConstraint2D
    {
        public DiscreteIslandConstraint2D(
            CollisionPair2D pair,
            int rootKey,
            int minColliderId,
            int maxColliderId)
        {
            Pair = pair;
            RootKey = rootKey;
            MinColliderId = minColliderId;
            MaxColliderId = maxColliderId;
        }

        public CollisionPair2D Pair { get; }
        public int RootKey { get; }
        public int MinColliderId { get; }
        public int MaxColliderId { get; }
    }
}
