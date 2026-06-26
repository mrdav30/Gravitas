//=======================================================================
// GravitasMixedCollisionService.Response.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System.Runtime.CompilerServices;

namespace Gravitas;

internal sealed partial class GravitasMixedCollisionService
{
    private void SolveMixedResponsePairs()
    {
        if (_mixedResponsePairs.Count == 0)
            return;

        if (_mixedResponsePairs.Count == 1)
        {
            CollisionPairMixed pair = _mixedResponsePairs[0];
            if (!HasAwakeResponseParticipant(pair))
                return;

            pair.WakeSleepingBodiesForCollision();
            CollisionResponseMixed.Resolve(pair, pair.Contact);
            return;
        }

        _mixedResponsePairs.SortInPlace(ResponsePairComparer);
        BuildMixedIslands();
        if (_mixedIslandConstraints.Count == 0)
            return;

        _mixedIslandConstraints.SortInPlace(IslandConstraintComparer);

        int start = 0;
        while (start < _mixedIslandConstraints.Count)
        {
            int rootKey = _mixedIslandConstraints[start].RootKey;
            int end = start + 1;
            while (end < _mixedIslandConstraints.Count
                && _mixedIslandConstraints[end].RootKey == rootKey)
            {
                end++;
            }

            if (WakeMixedIslandBodies(rootKey))
                SolveMixedIslandRange(rootKey, start, end);

            start = end;
        }
    }

    private void BuildMixedIslands()
    {
        _mixedIslandNodes.FastClear();
        _mixedIslandConstraints.FastClear();

        for (int i = 0; i < _mixedResponsePairs.Count; i++)
        {
            CollisionPairMixed pair = _mixedResponsePairs[i];
            AddMixedIslandNodeIfMovable(pair.Collider3D.Body);
            AddMixedIslandNodeIfMovable(pair.Collider2D.Body);
        }

        SortAndDeduplicateMixedIslandNodes();
        if (_mixedIslandNodes.Count == 0)
            return;

        for (int i = 0; i < _mixedResponsePairs.Count; i++)
        {
            CollisionPairMixed pair = _mixedResponsePairs[i];
            int node3D = FindMixedIslandNode(pair.Collider3D.Body);
            int node2D = FindMixedIslandNode(pair.Collider2D.Body);
            if (node3D >= 0 && node2D >= 0)
                UnionMixedIslandNodes(node3D, node2D);
        }

        CompressMixedIslandRoots();

        for (int i = 0; i < _mixedResponsePairs.Count; i++)
        {
            CollisionPairMixed pair = _mixedResponsePairs[i];
            int node3D = FindMixedIslandNode(pair.Collider3D.Body);
            int node2D = FindMixedIslandNode(pair.Collider2D.Body);
            int rootKey = ResolveMixedConstraintRootKey(node3D, node2D);
            if (rootKey < 0)
                continue;

            _mixedIslandConstraints.Add(new MixedIslandConstraint(pair, rootKey, pair.Key));
        }
    }

    private void AddMixedIslandNodeIfMovable(SolidBody? body)
    {
        if (!IsMovableMixedIslandBody(body))
            return;

        _mixedIslandNodes.Add(new MixedIslandNode(Create3DBodyKey(body!), body!, null));
    }

    private void AddMixedIslandNodeIfMovable(SolidBody2D? body)
    {
        if (!IsMovableMixedIslandBody(body))
            return;

        _mixedIslandNodes.Add(new MixedIslandNode(Create2DBodyKey(body!), null, body!));
    }

    private void SortAndDeduplicateMixedIslandNodes()
    {
        if (_mixedIslandNodes.Count == 0)
            return;

        if (_mixedIslandNodes.Count == 1)
        {
            MixedIslandNode singleNode = _mixedIslandNodes[0];
            singleNode.ParentIndex = 0;
            singleNode.RootKey = singleNode.BodyKey;
            _mixedIslandNodes[0] = singleNode;
            return;
        }

        _mixedIslandNodes.SortInPlace(IslandNodeComparer);

        int writeIndex = 0;
        int previousKey = -1;
        for (int readIndex = 0; readIndex < _mixedIslandNodes.Count; readIndex++)
        {
            MixedIslandNode node = _mixedIslandNodes[readIndex];
            if (node.BodyKey == previousKey)
                continue;

            node.ParentIndex = writeIndex;
            node.RootKey = node.BodyKey;
            _mixedIslandNodes[writeIndex++] = node;
            previousKey = node.BodyKey;
        }

        while (_mixedIslandNodes.Count > writeIndex)
            _mixedIslandNodes.RemoveAt(_mixedIslandNodes.Count - 1);
    }

    private int FindMixedIslandNode(SolidBody? body)
    {
        if (!IsMovableMixedIslandBody(body))
            return -1;

        return FindMixedIslandNode(Create3DBodyKey(body!));
    }

    private int FindMixedIslandNode(SolidBody2D? body)
    {
        if (!IsMovableMixedIslandBody(body))
            return -1;

        return FindMixedIslandNode(Create2DBodyKey(body!));
    }

    private int FindMixedIslandNode(int key)
    {
        int low = 0;
        int high = _mixedIslandNodes.Count - 1;
        while (low <= high)
        {
            int mid = low + ((high - low) >> 1);
            int midKey = _mixedIslandNodes[mid].BodyKey;
            if (midKey == key)
                return mid;

            if (midKey < key)
                low = mid + 1;
            else
                high = mid - 1;
        }

        return -1;
    }

    private void UnionMixedIslandNodes(int nodeA, int nodeB)
    {
        int rootA = FindMixedIslandRoot(nodeA);
        int rootB = FindMixedIslandRoot(nodeB);
        if (rootA == rootB)
            return;

        int keyA = _mixedIslandNodes[rootA].BodyKey;
        int keyB = _mixedIslandNodes[rootB].BodyKey;
        int parent = keyA <= keyB ? rootA : rootB;
        int child = parent == rootA ? rootB : rootA;

        MixedIslandNode childNode = _mixedIslandNodes[child];
        childNode.ParentIndex = parent;
        childNode.RootKey = _mixedIslandNodes[parent].BodyKey;
        _mixedIslandNodes[child] = childNode;
    }

    private int FindMixedIslandRoot(int index)
    {
        int root = index;
        while (_mixedIslandNodes[root].ParentIndex != root)
            root = _mixedIslandNodes[root].ParentIndex;

        while (index != root)
        {
            MixedIslandNode node = _mixedIslandNodes[index];
            int parent = node.ParentIndex;
            node.ParentIndex = root;
            node.RootKey = _mixedIslandNodes[root].BodyKey;
            _mixedIslandNodes[index] = node;
            index = parent;
        }

        return root;
    }

    private void CompressMixedIslandRoots()
    {
        for (int i = 0; i < _mixedIslandNodes.Count; i++)
        {
            int root = FindMixedIslandRoot(i);
            MixedIslandNode node = _mixedIslandNodes[i];
            node.RootKey = _mixedIslandNodes[root].BodyKey;
            _mixedIslandNodes[i] = node;
        }
    }

    private int ResolveMixedConstraintRootKey(int node3D, int node2D)
    {
        if (node3D >= 0)
            return _mixedIslandNodes[node3D].RootKey;

        return node2D >= 0 ? _mixedIslandNodes[node2D].RootKey : -1;
    }

    private bool WakeMixedIslandBodies(int rootKey)
    {
        bool hasAwakeBody = false;
        for (int i = 0; i < _mixedIslandNodes.Count; i++)
        {
            MixedIslandNode node = _mixedIslandNodes[i];
            if (node.RootKey == rootKey && node.IsAwakeForCollision)
            {
                hasAwakeBody = true;
                break;
            }
        }

        if (!hasAwakeBody)
            return false;

        for (int i = 0; i < _mixedIslandNodes.Count; i++)
        {
            MixedIslandNode node = _mixedIslandNodes[i];
            if (node.RootKey == rootKey)
                node.WakeFromCollision();
        }

        return true;
    }

    private void SolveMixedIslandRange(int rootKey, int start, int end)
    {
        if (end - start == 1)
        {
            CollisionPairMixed pair = _mixedIslandConstraints[start].Pair;
            CollisionResponseMixed.Resolve(pair, pair.Contact);
            return;
        }

        int iterationLimit = _context.Settings.DiscreteSolverIterations;
        int iterationsUsed = 0;
        for (int iteration = 0; iteration < iterationLimit; iteration++)
        {
            bool applyPositionCorrection = iteration == 0;
            for (int i = start; i < end; i++)
            {
                CollisionPairMixed pair = _mixedIslandConstraints[i].Pair;
                CollisionResponseMixed.Resolve(
                    pair,
                    pair.Contact,
                    iteration,
                    iterationLimit,
                    applyPositionCorrection);
            }

            iterationsUsed = iteration + 1;
        }

        _context.Diagnostics.EmitMixedResponseIsland(
            rootKey,
            end - start,
            iterationsUsed,
            iterationsUsed >= iterationLimit);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasAwakeMovableParticipant(LSCollider collider3D, LSCollider2D collider2D) =>
        IsAwakeMovable(collider3D.Body) || IsAwakeMovable(collider2D.Body);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HasAwakeResponseParticipant(CollisionPairMixed pair) =>
        IsAwakeMovable(pair.Collider3D.Body) || IsAwakeMovable(pair.Collider2D.Body);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAwakeMovable(SolidBody? body) =>
        body != null && body.Active && !body.Immovable && !body.IsKinematic && !body.IsSleeping && body.InverseMass > Fixed64.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAwakeMovable(SolidBody2D? body) =>
        body != null && body.CanTranslate && !body.IsSleeping;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsMovableMixedIslandBody(SolidBody? body) =>
        body != null && body.DynamicId >= 0 && body.CanTranslate;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsMovableMixedIslandBody(SolidBody2D? body) =>
        body != null && body.DynamicId >= 0 && body.CanTranslate;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int Create3DBodyKey(SolidBody body) =>
        body.DynamicId << 1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int Create2DBodyKey(SolidBody2D body) =>
        (body.DynamicId << 1) | 1;

}
