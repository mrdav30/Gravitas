//=======================================================================
// GravitasPhysicsService.Response.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.CollisionHandling;
using Gravitas.Constraints;

namespace Gravitas;

public sealed partial class GravitasPhysicsService
{
    internal void QueueDiscreteResponsePair(CollisionPair pair)
    {
        SwiftThrowHelper.ThrowIfNull(pair, nameof(pair));
        _discreteResponsePairs.Add(pair);
    }

    private void SolveDiscreteResponsePairs()
    {
        bool hasJoints = _context.Constraints3D.HasActiveJoints;
        if (_discreteResponsePairs.Count == 0 && !hasJoints)
            return;

        if (!hasJoints && _discreteResponsePairs.Count == 1)
        {
            CollisionPair pair = _discreteResponsePairs[0];
            if (!HasAwakeResponseParticipant(pair))
                return;

            pair.WakeSleepingBodiesForCollision();
            CollisionResponse.CalculateImpulse(pair);
            return;
        }

        if (_discreteResponsePairs.Count > 1)
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
            CollisionPair pair = _discreteResponsePairs[i];
            AddIslandNodeIfMovable(pair.ColliderA.Body!);
            AddIslandNodeIfMovable(pair.ColliderB.Body!);
        }

        AddJointIslandNodes();

        SortAndDeduplicateIslandNodes();
        if (_discreteIslandNodes.Count == 0)
            return;

        for (int i = 0; i < _discreteResponsePairs.Count; i++)
        {
            CollisionPair pair = _discreteResponsePairs[i];
            int nodeA = FindIslandNode(pair.ColliderA.Body!);
            int nodeB = FindIslandNode(pair.ColliderB.Body!);
            if (nodeA >= 0 && nodeB >= 0)
                UnionIslandNodes(nodeA, nodeB);
        }

        UnionJointIslandNodes();

        CompressIslandRoots();

        for (int i = 0; i < _discreteResponsePairs.Count; i++)
        {
            CollisionPair pair = _discreteResponsePairs[i];
            int nodeA = FindIslandNode(pair.ColliderA.Body!);
            int nodeB = FindIslandNode(pair.ColliderB.Body!);
            int rootKey = ResolveConstraintRootKey(nodeA, nodeB);
            if (rootKey < 0)
                continue;

            GetStablePairKey(pair, out int minColliderId, out int maxColliderId);
            _discreteIslandConstraints.Add(DiscreteIslandConstraint.CreatePair(
                pair,
                rootKey,
                minColliderId,
                maxColliderId));
        }

        AddJointIslandConstraints();
    }

    private void AddJointIslandNodes()
    {
        GravitasConstraint3DService constraints = _context.Constraints3D;
        for (int jointId = 1; jointId <= constraints.PeakJointCount; jointId++)
        {
            if (!constraints.TryGetJointForSolver(jointId, out Joint3D? joint))
                continue;

            AddIslandNodeIfMovable(joint!.BodyA);
            AddIslandNodeIfMovable(joint.BodyB);
        }
    }

    private void UnionJointIslandNodes()
    {
        GravitasConstraint3DService constraints = _context.Constraints3D;
        for (int jointId = 1; jointId <= constraints.PeakJointCount; jointId++)
        {
            if (!constraints.TryGetJointForSolver(jointId, out Joint3D? joint))
                continue;

            int nodeA = FindIslandNode(joint!.BodyA);
            int nodeB = FindIslandNode(joint.BodyB);
            if (nodeA >= 0 && nodeB >= 0)
                UnionIslandNodes(nodeA, nodeB);
        }
    }

    private void AddJointIslandConstraints()
    {
        GravitasConstraint3DService constraints = _context.Constraints3D;
        for (int jointId = 1; jointId <= constraints.PeakJointCount; jointId++)
        {
            if (!constraints.TryGetJointForSolver(jointId, out Joint3D? joint))
                continue;

            int nodeA = FindIslandNode(joint!.BodyA);
            int nodeB = FindIslandNode(joint.BodyB);
            int rootKey = ResolveConstraintRootKey(nodeA, nodeB);

            GetStableJointKey(joint, out int minColliderId, out int maxColliderId);
            _discreteIslandConstraints.Add(DiscreteIslandConstraint.CreateJoint(
                joint,
                rootKey,
                minColliderId,
                maxColliderId));
        }
    }

    private void AddIslandNodeIfMovable(SolidBody body)
    {
        if (!IsMovableIslandBody(body))
            return;

        _discreteIslandNodes.Add(new DiscreteIslandNode(body.DynamicId, body));
    }

    private void SortAndDeduplicateIslandNodes() =>
        IslandGraphUtility.SortAndDeduplicate(_discreteIslandNodes, IslandNodeComparer);

    private int FindIslandNode(SolidBody body)
    {
        if (!IsMovableIslandBody(body))
            return -1;

        return IslandGraphUtility.Find(_discreteIslandNodes, body.DynamicId);
    }

    private void UnionIslandNodes(int nodeA, int nodeB) =>
        IslandGraphUtility.Union(_discreteIslandNodes, nodeA, nodeB);

    private void CompressIslandRoots() =>
        IslandGraphUtility.CompressRoots(_discreteIslandNodes);

    private int ResolveConstraintRootKey(int nodeA, int nodeB) =>
        IslandGraphUtility.ResolveConstraintRootKey(_discreteIslandNodes, nodeA, nodeB);

    private bool WakeIslandBodies(int rootKey) =>
        IslandGraphUtility.WakeBodies(_discreteIslandNodes, rootKey);

    private void SolveDiscreteIslandRange(int start, int end)
    {
        if (end - start == 1 && _discreteIslandConstraints[start].Kind == DiscreteIslandConstraintKind.Contact)
        {
            CollisionResponse.CalculateImpulse(_discreteIslandConstraints[start].Pair);
            return;
        }

        int iterations = _context.Settings.DiscreteSolverIterations;
        for (int iteration = 0; iteration < iterations; iteration++)
        {
            bool applyCachedImpulse = iteration == 0;
            bool applyPositionCorrection = iteration == 0;
            for (int i = start; i < end; i++)
            {
                DiscreteIslandConstraint constraint = _discreteIslandConstraints[i];
                if (constraint.Kind == DiscreteIslandConstraintKind.Contact)
                {
                    CollisionResponse.CalculateImpulse(
                        constraint.Pair,
                        applyCachedImpulse,
                        applyPositionCorrection);
                }
                else
                    JointSolver3D.Solve(constraint.Joint!, applyCachedImpulse);
            }
        }
    }

}
