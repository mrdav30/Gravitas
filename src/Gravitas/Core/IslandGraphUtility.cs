//=======================================================================
// IslandGraphUtility.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using SwiftCollections;
using System.Collections.Generic;

namespace Gravitas;

internal interface IIslandNodeState
{
    int BodyKey { get; }

    int ParentIndex { get; set; }

    int RootKey { get; set; }

    bool IsAwakeForCollision { get; }

    void WakeFromCollision();
}

internal sealed class IslandNodeKeyComparer<TNode> : IComparer<TNode>
    where TNode : struct, IIslandNodeState
{
    public int Compare(TNode left, TNode right) =>
        left.BodyKey.CompareTo(right.BodyKey);
}

internal static class IslandGraphUtility
{
    public static void SortAndDeduplicate<TNode>(SwiftList<TNode> nodes, IComparer<TNode> comparer)
        where TNode : struct, IIslandNodeState
    {
        if (nodes.Count == 0)
            return;

        if (nodes.Count == 1)
        {
            TNode singleNode = nodes[0];
            singleNode.ParentIndex = 0;
            singleNode.RootKey = singleNode.BodyKey;
            nodes[0] = singleNode;
            return;
        }

        nodes.SortInPlace(comparer);

        int writeIndex = 0;
        int previousKey = -1;
        for (int readIndex = 0; readIndex < nodes.Count; readIndex++)
        {
            TNode node = nodes[readIndex];
            if (node.BodyKey == previousKey)
                continue;

            node.ParentIndex = writeIndex;
            node.RootKey = node.BodyKey;
            nodes[writeIndex++] = node;
            previousKey = node.BodyKey;
        }

        while (nodes.Count > writeIndex)
            nodes.RemoveAt(nodes.Count - 1);
    }

    public static int Find<TNode>(SwiftList<TNode> nodes, int key)
        where TNode : struct, IIslandNodeState
    {
        int low = 0;
        int high = nodes.Count - 1;
        while (low <= high)
        {
            int mid = low + ((high - low) >> 1);
            int midKey = nodes[mid].BodyKey;
            if (midKey == key)
                return mid;

            if (midKey < key)
                low = mid + 1;
            else
                high = mid - 1;
        }

        return -1;
    }

    public static void Union<TNode>(SwiftList<TNode> nodes, int nodeA, int nodeB)
        where TNode : struct, IIslandNodeState
    {
        int rootA = FindRoot(nodes, nodeA);
        int rootB = FindRoot(nodes, nodeB);
        if (rootA == rootB)
            return;

        int keyA = nodes[rootA].BodyKey;
        int keyB = nodes[rootB].BodyKey;
        int parent = keyA <= keyB ? rootA : rootB;
        int child = parent == rootA ? rootB : rootA;

        TNode childNode = nodes[child];
        childNode.ParentIndex = parent;
        childNode.RootKey = nodes[parent].BodyKey;
        nodes[child] = childNode;
    }

    public static int FindRoot<TNode>(SwiftList<TNode> nodes, int index)
        where TNode : struct, IIslandNodeState
    {
        int root = index;
        while (nodes[root].ParentIndex != root)
            root = nodes[root].ParentIndex;

        while (index != root)
        {
            TNode node = nodes[index];
            int parent = node.ParentIndex;
            node.ParentIndex = root;
            node.RootKey = nodes[root].BodyKey;
            nodes[index] = node;
            index = parent;
        }

        return root;
    }

    public static void CompressRoots<TNode>(SwiftList<TNode> nodes)
        where TNode : struct, IIslandNodeState
    {
        for (int i = 0; i < nodes.Count; i++)
        {
            int root = FindRoot(nodes, i);
            TNode node = nodes[i];
            node.RootKey = nodes[root].BodyKey;
            nodes[i] = node;
        }
    }

    public static int ResolveConstraintRootKey<TNode>(SwiftList<TNode> nodes, int nodeA, int nodeB)
        where TNode : struct, IIslandNodeState
    {
        if (nodeA >= 0)
            return nodes[nodeA].RootKey;

        return nodeB >= 0 ? nodes[nodeB].RootKey : -1;
    }

    public static bool WakeBodies<TNode>(SwiftList<TNode> nodes, int rootKey)
        where TNode : struct, IIslandNodeState
    {
        bool hasAwakeBody = false;
        for (int i = 0; i < nodes.Count; i++)
        {
            TNode node = nodes[i];
            if (node.RootKey == rootKey && node.IsAwakeForCollision)
            {
                hasAwakeBody = true;
                break;
            }
        }

        if (!hasAwakeBody)
            return false;

        for (int i = 0; i < nodes.Count; i++)
        {
            TNode node = nodes[i];
            if (node.RootKey == rootKey)
                node.WakeFromCollision();
        }

        return true;
    }
}
