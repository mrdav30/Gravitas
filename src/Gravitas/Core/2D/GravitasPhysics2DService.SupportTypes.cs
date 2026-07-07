//=======================================================================
// GravitasPhysics2DService.SupportTypes.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Support;
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
    private static bool IsAwakeMovable(SolidBody2D? body) =>
        body != null && body.CanTranslate && !body.IsSleeping;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsMovableIslandBody(SolidBody2D? body) =>
        body != null && body.DynamicId >= 0 && body.CanTranslate;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAwakeIslandBody(SolidBody2D? body) =>
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void GetStableJointKey(Joint2D joint, out int minColliderId, out int maxColliderId)
    {
        int idA = joint.BodyA.Collider.Id;
        int idB = joint.BodyB.Collider.Id;
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
            GetStablePairKey(left!, out int leftMin, out int leftMax);
            GetStablePairKey(right!, out int rightMin, out int rightMax);

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
            if (compare != 0)
                return compare;

            compare = left.MaxColliderId.CompareTo(right.MaxColliderId);
            if (compare != 0)
                return compare;

            compare = left.Kind.CompareTo(right.Kind);
            return compare != 0 ? compare : left.JointId.CompareTo(right.JointId);
        }
    }

    private struct DiscreteIslandNode2D
    {
        public DiscreteIslandNode2D(int bodyKey, SolidBody2D body)
        {
            BodyKey = bodyKey;
            Body = body;
            ParentIndex = -1;
            RootKey = bodyKey;
        }

        public int BodyKey;
        public SolidBody2D Body;
        public int ParentIndex;
        public int RootKey;
    }

    private readonly struct DiscreteIslandConstraint2D
    {
        private DiscreteIslandConstraint2D(
            DiscreteIslandConstraint2DKind kind,
            CollisionPair2D pair,
            Joint2D? joint,
            int rootKey,
            int minColliderId,
            int maxColliderId,
            int jointId)
        {
            Kind = kind;
            Pair = pair;
            Joint = joint;
            RootKey = rootKey;
            MinColliderId = minColliderId;
            MaxColliderId = maxColliderId;
            JointId = jointId;
        }

        public static DiscreteIslandConstraint2D CreatePair(
            CollisionPair2D pair,
            int rootKey,
            int minColliderId,
            int maxColliderId) =>
            new(DiscreteIslandConstraint2DKind.Contact, pair, null, rootKey, minColliderId, maxColliderId, 0);

        public static DiscreteIslandConstraint2D CreateJoint(
            Joint2D joint,
            int rootKey,
            int minColliderId,
            int maxColliderId) =>
            new(DiscreteIslandConstraint2DKind.Joint, null!, joint, rootKey, minColliderId, maxColliderId, joint.Id);

        public DiscreteIslandConstraint2DKind Kind { get; }
        public CollisionPair2D Pair { get; }
        public Joint2D? Joint { get; }
        public int RootKey { get; }
        public int MinColliderId { get; }
        public int MaxColliderId { get; }
        public int JointId { get; }
    }

    private enum DiscreteIslandConstraint2DKind : byte
    {
        Contact = 0,
        Joint = 1
    }
}
