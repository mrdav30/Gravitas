//=======================================================================
// GravitasPhysics2DService.Response.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.CollisionHandling;

namespace Gravitas;

public sealed partial class GravitasPhysics2DService
{
    private void SolveDiscreteResponsePairs()
    {
        if (_discreteResponsePairs.Count == 0)
            return;

        if (_discreteResponsePairs.Count == 1)
        {
            CollisionPair2D pair = _discreteResponsePairs[0];
            if (!HasAwakeResponseParticipant(pair))
                return;

            pair.WakeSleepingBodiesForCollision();
            CollisionResponse2D.Resolve(pair);
            return;
        }

        _discreteResponsePairs.SortInPlace(ResponsePairComparer);
        BuildDiscreteIslands();
        if (_discreteIslandConstraints.Count == 0)
            return;

        _discreteIslandConstraints.SortInPlace(IslandConstraintComparer);

        int start = 0;
        while (start < _discreteIslandConstraints.Count)
        {
            int rootKey = _discreteIslandConstraints[start].RootKey;
            int end = start + 1;
            while (end < _discreteIslandConstraints.Count
                && _discreteIslandConstraints[end].RootKey == rootKey)
            {
                end++;
            }

            if (WakeIslandBodies(rootKey))
                SolveDiscreteIslandRange(start, end);

            start = end;
        }
    }

    private void BuildDiscreteIslands()
    {
        _discreteIslandNodes.FastClear();
        _discreteIslandConstraints.FastClear();

        for (int i = 0; i < _discreteResponsePairs.Count; i++)
        {
            CollisionPair2D pair = _discreteResponsePairs[i];
            AddIslandNodeIfMovable(pair.ColliderA.Body);
            AddIslandNodeIfMovable(pair.ColliderB.Body);
        }

        SortAndDeduplicateIslandNodes();
        if (_discreteIslandNodes.Count == 0)
            return;

        for (int i = 0; i < _discreteResponsePairs.Count; i++)
        {
            CollisionPair2D pair = _discreteResponsePairs[i];
            int nodeA = FindIslandNode(pair.ColliderA.Body);
            int nodeB = FindIslandNode(pair.ColliderB.Body);
            if (nodeA >= 0 && nodeB >= 0)
                UnionIslandNodes(nodeA, nodeB);
        }

        CompressIslandRoots();

        for (int i = 0; i < _discreteResponsePairs.Count; i++)
        {
            CollisionPair2D pair = _discreteResponsePairs[i];
            int nodeA = FindIslandNode(pair.ColliderA.Body);
            int nodeB = FindIslandNode(pair.ColliderB.Body);
            int rootKey = ResolveConstraintRootKey(nodeA, nodeB);
            if (rootKey < 0)
                continue;

            GetStablePairKey(pair, out int minColliderId, out int maxColliderId);
            _discreteIslandConstraints.Add(new DiscreteIslandConstraint2D(
                pair,
                rootKey,
                minColliderId,
                maxColliderId));
        }
    }

    private void AddIslandNodeIfMovable(StiffBody2D? body)
    {
        if (!IsMovableIslandBody(body))
            return;

        _discreteIslandNodes.Add(new DiscreteIslandNode2D(body!.DynamicId, body));
    }

    private void SortAndDeduplicateIslandNodes()
    {
        if (_discreteIslandNodes.Count == 0)
            return;

        if (_discreteIslandNodes.Count == 1)
        {
            DiscreteIslandNode2D singleNode = _discreteIslandNodes[0];
            singleNode.ParentIndex = 0;
            singleNode.RootKey = singleNode.BodyKey;
            _discreteIslandNodes[0] = singleNode;
            return;
        }

        _discreteIslandNodes.SortInPlace(IslandNodeComparer);

        int writeIndex = 0;
        int previousKey = -1;
        for (int readIndex = 0; readIndex < _discreteIslandNodes.Count; readIndex++)
        {
            DiscreteIslandNode2D node = _discreteIslandNodes[readIndex];
            if (node.BodyKey == previousKey)
                continue;

            node.ParentIndex = writeIndex;
            node.RootKey = node.BodyKey;
            _discreteIslandNodes[writeIndex++] = node;
            previousKey = node.BodyKey;
        }

        while (_discreteIslandNodes.Count > writeIndex)
            _discreteIslandNodes.RemoveAt(_discreteIslandNodes.Count - 1);
    }

    private int FindIslandNode(StiffBody2D? body)
    {
        if (!IsMovableIslandBody(body))
            return -1;

        int key = body!.DynamicId;
        int low = 0;
        int high = _discreteIslandNodes.Count - 1;
        while (low <= high)
        {
            int mid = low + ((high - low) >> 1);
            int midKey = _discreteIslandNodes[mid].BodyKey;
            if (midKey == key)
                return mid;

            if (midKey < key)
                low = mid + 1;
            else
                high = mid - 1;
        }

        return -1;
    }

    private void UnionIslandNodes(int nodeA, int nodeB)
    {
        int rootA = FindIslandRoot(nodeA);
        int rootB = FindIslandRoot(nodeB);
        if (rootA == rootB)
            return;

        int keyA = _discreteIslandNodes[rootA].BodyKey;
        int keyB = _discreteIslandNodes[rootB].BodyKey;
        int parent = keyA <= keyB ? rootA : rootB;
        int child = parent == rootA ? rootB : rootA;

        DiscreteIslandNode2D childNode = _discreteIslandNodes[child];
        childNode.ParentIndex = parent;
        childNode.RootKey = _discreteIslandNodes[parent].BodyKey;
        _discreteIslandNodes[child] = childNode;
    }

    private int FindIslandRoot(int index)
    {
        int root = index;
        while (_discreteIslandNodes[root].ParentIndex != root)
            root = _discreteIslandNodes[root].ParentIndex;

        while (index != root)
        {
            DiscreteIslandNode2D node = _discreteIslandNodes[index];
            int parent = node.ParentIndex;
            node.ParentIndex = root;
            node.RootKey = _discreteIslandNodes[root].BodyKey;
            _discreteIslandNodes[index] = node;
            index = parent;
        }

        return root;
    }

    private void CompressIslandRoots()
    {
        for (int i = 0; i < _discreteIslandNodes.Count; i++)
        {
            int root = FindIslandRoot(i);
            DiscreteIslandNode2D node = _discreteIslandNodes[i];
            node.RootKey = _discreteIslandNodes[root].BodyKey;
            _discreteIslandNodes[i] = node;
        }
    }

    private int ResolveConstraintRootKey(int nodeA, int nodeB)
    {
        if (nodeA >= 0)
            return _discreteIslandNodes[nodeA].RootKey;

        return nodeB >= 0 ? _discreteIslandNodes[nodeB].RootKey : -1;
    }

    private bool WakeIslandBodies(int rootKey)
    {
        bool hasAwakeBody = false;
        for (int i = 0; i < _discreteIslandNodes.Count; i++)
        {
            DiscreteIslandNode2D node = _discreteIslandNodes[i];
            if (node.RootKey == rootKey && node.Body.IsAwakeForCollision)
            {
                hasAwakeBody = true;
                break;
            }
        }

        if (!hasAwakeBody)
            return false;

        for (int i = 0; i < _discreteIslandNodes.Count; i++)
        {
            DiscreteIslandNode2D node = _discreteIslandNodes[i];
            if (node.RootKey == rootKey)
                node.Body.WakeFromCollision();
        }

        return true;
    }

    private void SolveDiscreteIslandRange(int start, int end)
    {
        if (end - start == 1)
        {
            CollisionResponse2D.Resolve(_discreteIslandConstraints[start].Pair);
            return;
        }

        int iterations = _context.Settings.DiscreteSolverIterations;
        for (int iteration = 0; iteration < iterations; iteration++)
        {
            bool applyCachedImpulse = iteration == 0;
            bool applyPositionCorrection = iteration == 0;
            for (int i = start; i < end; i++)
            {
                CollisionResponse2D.Resolve(
                    _discreteIslandConstraints[i].Pair,
                    applyCachedImpulse,
                    applyPositionCorrection);
            }
        }
    }

}
