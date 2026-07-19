//=======================================================================
// GravitasPhysicsService.ContinuousCollision.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
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
        _dirtyContinuousCollisionCandidates.Clear();
        _dirtyContinuousCollisionBodyIds.Clear();
        _dirtyContinuousCollisionBodies.FastClear();
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
        if ((!body.IsKinematic && body.IsPositionFullyFrozen)
            || body.Collider.IsTrigger)
        {
            return;
        }

        Fixed64 radius = body.ResolveContinuousCollisionProxyRadius();
        if (radius <= Fixed64.Epsilon)
            return;

        _continuousCollisionCandidateLifetimes[body.DynamicId] =
            new ColliderLifetimeToken(body.Collider);
        _continuousCollisionCandidates.Add(
            body.DynamicId,
            DynamicCcdCandidateIndex.CreateBoundsBetween(
                body.ContinuousCollisionFrameStart,
                body.ContinuousCollisionFrameEnd,
                Vector3d.One * radius));
    }

    /// <summary>
    /// Runs this context's visual interpolation step for dynamic bodies.
    /// </summary>

    internal SwiftList<int> QueryContinuousCollisionCandidates(FixedBoundVolume sourceBounds)
    {
        PrepareContinuousCollisionFrame();
        _continuousCollisionCandidates.Query(sourceBounds, _continuousCollisionCandidateIds);
        _dirtyContinuousCollisionCandidates.Query(sourceBounds, _dirtyContinuousCollisionCandidateIds);
        int retainedCount = 0;
        for (int i = 0; i < _continuousCollisionCandidateIds.Count; i++)
        {
            int dynamicId = _continuousCollisionCandidateIds[i];
            if (!_dirtyContinuousCollisionBodyIds.Contains(dynamicId)
                && TryGetContinuousCollisionCandidate(dynamicId, out _))
            {
                _continuousCollisionCandidateIds[retainedCount++] = dynamicId;
            }
        }

        while (_continuousCollisionCandidateIds.Count > retainedCount)
            _continuousCollisionCandidateIds.RemoveAt(
                _continuousCollisionCandidateIds.Count - 1);

        for (int i = 0; i < _dirtyContinuousCollisionCandidateIds.Count; i++)
        {
            int dynamicId = _dirtyContinuousCollisionCandidateIds[i];
            if (TryGetContinuousCollisionCandidate(dynamicId, out _))
                _continuousCollisionCandidateIds.Add(dynamicId);
        }

        return _continuousCollisionCandidateIds;
    }

    internal bool TryGetContinuousCollisionCandidate(int dynamicId, out SolidBody body)
    {
        ColliderLifetimeToken registration = _continuousCollisionCandidateLifetimes[dynamicId];
        if (registration.Collider != null && registration.IsActive)
        {
            body = registration.Collider.Body!;
            return true;
        }

        body = null!;
        return false;
    }

    internal SolidBody GetContinuousCollisionCandidate(int dynamicId) =>
        _dynamicBodies[dynamicId];

    internal bool CanAdmitContinuousCollisionCandidateRefresh(SolidBody body) => true;

    internal bool CanAdmitContinuousCollisionCandidateRefresh(
        SolidBody first,
        SolidBody second) => true;

    internal bool TryReserveContinuousCollisionCandidateRefresh(SolidBody body)
    {
        if (_dirtyContinuousCollisionBodyIds.Add(body.DynamicId))
            _dirtyContinuousCollisionBodies.Add(body);

        return true;
    }

    private void ReleaseContinuousCollisionCandidateRefresh(SolidBody body)
    {
        if (!_dirtyContinuousCollisionBodyIds.Remove(body.DynamicId))
            return;

        _dirtyContinuousCollisionBodies.Remove(body);
        _dirtyContinuousCollisionCandidates.Remove(body.DynamicId);
    }

    internal bool TryReserveContinuousCollisionCandidateRefresh(
        SolidBody first,
        SolidBody second)
    {
        TryReserveContinuousCollisionCandidateRefresh(first);
        TryReserveContinuousCollisionCandidateRefresh(second);
        return true;
    }

    internal void RefreshContinuousCollisionCandidate(SolidBody body)
    {
        Fixed64 radius = body.ResolveContinuousCollisionProxyRadius();
        _dirtyContinuousCollisionCandidates.AddOrUpdate(
            body.DynamicId,
            body.ResolveContinuousCollisionTrajectoryBounds(radius));
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

    internal void ReportContinuousCollisionIterationLimit() =>
        LastContinuousCollisionIslandLimitReached = true;

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
        bool completed = false;
        bool limitReached = false;
        try
        {
            while (readIndex < _continuousCollisionHandoffQueue.Count && iterations < iterationBudget)
            {
                SolidBody body = _continuousCollisionHandoffQueue[readIndex++];
                // Consumption can synchronously return work to this body; dequeue ends dedupe ownership first.
                _queuedContinuousCollisionHandoffBodies.Remove(body);

                if (body.TryConsumeQueuedContinuousCollisionHandoff(
                        updateSleepState: false,
                        updateColliderState: false,
                        out bool shouldNotifyMovement))
                {
                    processed = true;
                    iterations++;
                    if (shouldNotifyMovement)
                        body.NotifyAuthoritativeMovement();
                }
            }

            limitReached = readIndex < _continuousCollisionHandoffQueue.Count;
            completed = true;
            return iterations;
        }
        finally
        {
            if (processed)
                LastContinuousCollisionIslandCount++;

            LastContinuousCollisionIslandIterationCount += iterations;
            LastContinuousCollisionIslandLimitReached |= completed && limitReached;
            if (!completed || limitReached)
                DiscardContinuousCollisionHandoffQueue();
            else
                ClearContinuousCollisionHandoffQueue();
        }
    }

    private void BeginContinuousCollisionHandoffFrame()
    {
        _processedContinuousCollisionBodies.Clear();
        DiscardContinuousCollisionHandoffQueue();
        LastContinuousCollisionIslandCount = 0;
        LastContinuousCollisionIslandIterationCount = 0;
        LastContinuousCollisionIslandLimitReached = false;
    }

    internal void AbortContinuousCollisionHandoffFrame()
    {
        _processedContinuousCollisionBodies.Clear();
        DiscardContinuousCollisionHandoffQueue();
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
