//=======================================================================
// GravitasPhysics2DService.ContinuousCollision.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.CollisionHandling;
using SwiftCollections;
using SwiftCollections.Query;

namespace Gravitas;

public sealed partial class GravitasPhysics2DService
{
    internal void PrepareContinuousCollisionFrame()
    {
        int token = _context.LateSimulateToken;
        bool buildMixedIndex = _context.Settings.RuntimeMode.RunsMixedContacts();
        if (_continuousCollisionPreparedToken == token
            && _continuousCollisionPreparedMixedIndex == buildMixedIndex)
        {
            return;
        }

        _planarContinuousCollisionCandidates.Clear();
        _mixedContinuousCollisionCandidates.Clear();
        foreach (SolidBody2D body in _dynamicBodies)
        {
            body.EnsureContinuousCollisionFramePrepared(token);
            AddContinuousCollisionCandidate(body, buildMixedIndex);
        }

        _planarContinuousCollisionCandidates.Sort();
        if (buildMixedIndex)
            _mixedContinuousCollisionCandidates.Sort();

        _continuousCollisionPreparedToken = token;
        _continuousCollisionPreparedMixedIndex = buildMixedIndex;
    }

    private void AddContinuousCollisionCandidate(SolidBody2D body, bool buildMixedIndex)
    {
        if (body.IsPositionFullyFrozen
            || body.IsKinematic
            || body.Collider.IsTrigger)
        {
            return;
        }

        Fixed64 planarRadius = body.ResolveContinuousCollisionProxyRadius();
        if (planarRadius <= Fixed64.Epsilon)
            return;

        _planarContinuousCollisionCandidates.Add(
            body.DynamicId,
            DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
                body.ContinuousCollisionFrameStart,
                body.ContinuousCollisionFrameDisplacement,
                planarRadius));

        if (!buildMixedIndex)
            return;

        Vector2d mixedStart2D = body.ContinuousCollisionFrameStart;
        Vector2d mixedDisplacement2D = body.ContinuousCollisionFrameDisplacement;
        Fixed64 mixedRadius = FixedMath.Max(planarRadius, body.Collider.MixedHalfThickness);
        _mixedContinuousCollisionCandidates.Add(
            body.DynamicId,
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(
                new Vector3d(mixedStart2D.X, body.Collider.MixedSlabCenterY, mixedStart2D.Y),
                new Vector3d(mixedDisplacement2D.X, Fixed64.Zero, mixedDisplacement2D.Y),
                mixedRadius));
    }


    internal SwiftList<int> QueryPlanarContinuousCollisionCandidates(DynamicCcdPlanarBounds sourceBounds)
    {
        PrepareContinuousCollisionFrame();
        _planarContinuousCollisionCandidates.Query(sourceBounds, _continuousCollisionCandidateIds);
        return _continuousCollisionCandidateIds;
    }

    internal SwiftList<int> QueryMixedContinuousCollisionCandidates(FixedBoundVolume sourceBounds)
    {
        if (!_context.Settings.RuntimeMode.RunsMixedContacts())
        {
            _continuousCollisionCandidateIds.FastClear();
            return _continuousCollisionCandidateIds;
        }

        PrepareContinuousCollisionFrame();
        _mixedContinuousCollisionCandidates.Query(sourceBounds, _continuousCollisionCandidateIds);
        return _continuousCollisionCandidateIds;
    }

    internal void QueueContinuousCollisionHandoff(SolidBody2D body)
    {
        if (!_processedContinuousCollisionBodies.Contains(body)
            || !_queuedContinuousCollisionHandoffBodies.Add(body))
        {
            return;
        }

        _continuousCollisionHandoffQueue.Add(body);
    }

    internal bool ProcessQueuedContinuousCollisionHandoffs() =>
        ProcessQueuedContinuousCollisionHandoffs(_context.Settings.ContinuousCollisionMaxToiIterations) > 0;

    internal int ProcessQueuedContinuousCollisionHandoffs(int iterationBudget)
    {
        if (_continuousCollisionHandoffQueue.Count == 0)
            return 0;

        if (iterationBudget <= 0)
        {
            LastContinuousCollisionIslandLimitReached = true;
            DiscardContinuousCollisionHandoffQueue();
            return 0;
        }

        int readIndex = 0;
        int iterations = 0;
        bool processed = false;
        while (readIndex < _continuousCollisionHandoffQueue.Count && iterations < iterationBudget)
        {
            SolidBody2D body = _continuousCollisionHandoffQueue[readIndex++];
            // Consumption can synchronously return work to this body; dequeue ends dedupe ownership first.
            _queuedContinuousCollisionHandoffBodies.Remove(body);

            if (body.TryConsumeContinuousCollisionHandoff(updateSleepState: false, updateColliderState: false))
            {
                processed = true;
                iterations++;
            }
        }

        if (processed)
            LastContinuousCollisionIslandCount++;

        LastContinuousCollisionIslandIterationCount += iterations;
        bool limitReached = readIndex < _continuousCollisionHandoffQueue.Count;
        LastContinuousCollisionIslandLimitReached |= limitReached;
        if (limitReached)
            DiscardContinuousCollisionHandoffQueue();
        else
            ClearContinuousCollisionHandoffQueue();
        return iterations;
    }

    private void BeginContinuousCollisionHandoffFrame()
    {
        _processedContinuousCollisionBodies.Clear();
        ClearContinuousCollisionHandoffQueue();
        LastContinuousCollisionIslandCount = 0;
        LastContinuousCollisionIslandIterationCount = 0;
        LastContinuousCollisionIslandLimitReached = false;
    }

    private void ClearContinuousCollisionHandoffQueue()
    {
        _queuedContinuousCollisionHandoffBodies.Clear();
        _continuousCollisionHandoffQueue.FastClear();
    }

    private void DiscardContinuousCollisionHandoffQueue()
    {
        for (int i = 0; i < _continuousCollisionHandoffQueue.Count; i++)
            _continuousCollisionHandoffQueue[i].DiscardContinuousCollisionHandoff();

        ClearContinuousCollisionHandoffQueue();
    }

}
