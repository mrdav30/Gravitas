//=======================================================================
// GravitasPhysicsService.ContinuousCollision.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.CollisionHandling;
using SwiftCollections;
using SwiftCollections.Query;

namespace Gravitas;

public sealed partial class GravitasPhysicsService
{
    internal void PrepareContinuousCollisionFrame()
    {
        int token = _context.LateSimulateToken;
        if (_continuousCollisionPreparedToken == token)
            return;

        _continuousCollisionCandidates.Clear();
        int peak = _dynamicBodies.PeakCount;
        for (int i = 0; i < peak; i++)
        {
            if (_dynamicBodies.TryGetValue(i, out SolidBody body))
            {
                body.EnsureContinuousCollisionFramePrepared(token);
                AddContinuousCollisionCandidate(body);
            }
        }

        _continuousCollisionCandidates.Sort();
        _continuousCollisionPreparedToken = token;
    }

    private void AddContinuousCollisionCandidate(SolidBody body)
    {
        if (body.IsPositionFullyFrozen
            || body.IsKinematic
            || body.Collider.IsTrigger)
        {
            return;
        }

        Fixed64 radius = body.ResolveContinuousCollisionProxyRadius();
        if (radius <= Fixed64.Epsilon)
            return;

        _continuousCollisionCandidates.Add(
            body.DynamicId,
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(
                body.ContinuousCollisionFrameStart,
                body.ContinuousCollisionFrameDisplacement,
                radius));
    }

    /// <summary>
    /// Runs this context's visual interpolation step for dynamic bodies.
    /// </summary>

    internal SwiftList<int> QueryContinuousCollisionCandidates(FixedBoundVolume sourceBounds)
    {
        PrepareContinuousCollisionFrame();
        _continuousCollisionCandidates.Query(sourceBounds, _continuousCollisionCandidateIds);
        return _continuousCollisionCandidateIds;
    }

    internal void QueueContinuousCollisionHandoff(SolidBody body)
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
            SolidBody body = _continuousCollisionHandoffQueue[readIndex++];

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
