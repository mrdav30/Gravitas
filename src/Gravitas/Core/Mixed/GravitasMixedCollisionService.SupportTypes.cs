//=======================================================================
// GravitasMixedCollisionService.SupportTypes.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using GridForge.Spatial;
using System.Collections.Generic;

namespace Gravitas;

internal sealed partial class GravitasMixedCollisionService
{
    private sealed class PhysicsMixedPartitionOrderComparer : IComparer<PhysicsMixedPartition>
    {
        public int Compare(PhysicsMixedPartition? left, PhysicsMixedPartition? right)
        {
            if (ReferenceEquals(left, right))
                return 0;
            if (left == null)
                return -1;
            if (right == null)
                return 1;

            WorldVoxelIndex leftIndex = left.WorldIndex;
            WorldVoxelIndex rightIndex = right.WorldIndex;

            int compare = leftIndex.GridIndex.CompareTo(rightIndex.GridIndex);
            if (compare != 0)
                return compare;

            compare = leftIndex.GridSpawnToken.CompareTo(rightIndex.GridSpawnToken);
            if (compare != 0)
                return compare;

            compare = leftIndex.VoxelIndex.x.CompareTo(rightIndex.VoxelIndex.x);
            if (compare != 0)
                return compare;

            compare = leftIndex.VoxelIndex.y.CompareTo(rightIndex.VoxelIndex.y);
            if (compare != 0)
                return compare;

            return leftIndex.VoxelIndex.z.CompareTo(rightIndex.VoxelIndex.z);
        }
    }

    private sealed class MixedColliderKeyComparer : IComparer<MixedColliderKey>
    {
        public int Compare(MixedColliderKey left, MixedColliderKey right) =>
            left.Key.CompareTo(right.Key);
    }

    private sealed class MixedResponsePairComparer : IComparer<CollisionPairMixed>
    {
        public int Compare(CollisionPairMixed? left, CollisionPairMixed? right)
        {
            if (ReferenceEquals(left, right))
                return 0;
            if (left == null)
                return -1;
            if (right == null)
                return 1;

            return left.Key.CompareTo(right.Key);
        }
    }

    private sealed class MixedIslandNodeComparer : IComparer<MixedIslandNode>
    {
        public int Compare(MixedIslandNode left, MixedIslandNode right) =>
            left.BodyKey.CompareTo(right.BodyKey);
    }

    private sealed class MixedIslandConstraintComparer : IComparer<MixedIslandConstraint>
    {
        public int Compare(MixedIslandConstraint left, MixedIslandConstraint right)
        {
            int compare = left.RootKey.CompareTo(right.RootKey);
            if (compare != 0)
                return compare;

            return left.PairKey.CompareTo(right.PairKey);
        }
    }

    private sealed class Collider2DIdComparer : IComparer<LSCollider2D>
    {
        public int Compare(LSCollider2D? left, LSCollider2D? right)
        {
            if (ReferenceEquals(left, right))
                return 0;
            if (left == null)
                return -1;
            if (right == null)
                return 1;

            return left.Id.CompareTo(right.Id);
        }
    }

    private sealed class Collider3DIdComparer : IComparer<LSCollider>
    {
        public int Compare(LSCollider? left, LSCollider? right)
        {
            if (ReferenceEquals(left, right))
                return 0;
            if (left == null)
                return -1;
            if (right == null)
                return 1;

            return left.Id.CompareTo(right.Id);
        }
    }

    private struct MixedIslandNode
    {
        public MixedIslandNode(int bodyKey, SolidBody? body3D, SolidBody2D? body2D)
        {
            BodyKey = bodyKey;
            Body3D = body3D;
            Body2D = body2D;
            ParentIndex = -1;
            RootKey = bodyKey;
        }

        public int BodyKey { get; }

        public SolidBody? Body3D { get; }

        public SolidBody2D? Body2D { get; }

        public int ParentIndex { get; set; }

        public int RootKey { get; set; }

        public bool IsAwakeForCollision =>
            Body3D?.IsAwakeForCollision ?? Body2D!.IsAwakeForCollision;

        public void WakeFromCollision()
        {
            if (Body3D != null)
            {
                Body3D.WakeFromCollision();
                return;
            }

            Body2D!.WakeFromCollision();
        }
    }

    private readonly struct MixedIslandConstraint
    {
        public MixedIslandConstraint(CollisionPairMixed pair, int rootKey, ulong pairKey)
        {
            Pair = pair;
            RootKey = rootKey;
            PairKey = pairKey;
        }

        public CollisionPairMixed Pair { get; }

        public int RootKey { get; }

        public ulong PairKey { get; }
    }
}
