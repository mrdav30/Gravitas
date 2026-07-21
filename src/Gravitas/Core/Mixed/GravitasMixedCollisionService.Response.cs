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

    private void SortAndDeduplicateMixedIslandNodes() =>
        IslandGraphUtility.SortAndDeduplicate(_mixedIslandNodes, IslandNodeComparer);

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

    private int FindMixedIslandNode(int key) =>
        IslandGraphUtility.Find(_mixedIslandNodes, key);

    private void UnionMixedIslandNodes(int nodeA, int nodeB) =>
        IslandGraphUtility.Union(_mixedIslandNodes, nodeA, nodeB);

    private void CompressMixedIslandRoots() =>
        IslandGraphUtility.CompressRoots(_mixedIslandNodes);

    private int ResolveMixedConstraintRootKey(int node3D, int node2D) =>
        IslandGraphUtility.ResolveConstraintRootKey(_mixedIslandNodes, node3D, node2D);

    private bool WakeMixedIslandBodies(int rootKey) =>
        IslandGraphUtility.WakeBodies(_mixedIslandNodes, rootKey);

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
        body != null && body.HasSolverMobility && !body.IsSleeping;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsAwakeMovable(SolidBody2D? body) =>
        body != null && body.HasSolverMobility && !body.IsSleeping;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsMovableMixedIslandBody(SolidBody? body) =>
        body != null && body.DynamicId >= 0 && body.HasSolverMobility;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsMovableMixedIslandBody(SolidBody2D? body) =>
        body != null && body.DynamicId >= 0 && body.HasSolverMobility;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int Create3DBodyKey(SolidBody body) =>
        body.DynamicId << 1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int Create2DBodyKey(SolidBody2D body) =>
        (body.DynamicId << 1) | 1;

}
