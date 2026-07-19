//=======================================================================
// GravitasPhysics2DService.ContinuousCollision.cs
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
        _dirtyPlanarContinuousCollisionCandidates.Clear();
        _dirtyPlanarContinuousCollisionBodies.FastClear();
        _dirtyPlanarContinuousCollisionBodySet.Clear();
        _mixedContinuousCollisionCandidates.Clear();
        _dirtyMixedContinuousCollisionCandidates.Clear();
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
        if ((!body.IsKinematic && body.IsPositionFullyFrozen)
            || body.Collider.IsTrigger)
        {
            return;
        }

        Fixed64 planarRadius = body.ResolveContinuousCollisionProxyRadius();
        if (planarRadius <= Fixed64.Epsilon)
            return;

        _continuousCollisionCandidateLifetimes[body.DynamicId] =
            new ColliderLifetimeToken2D(body.Collider);
        _planarContinuousCollisionCandidates.Add(
            body.DynamicId,
            DynamicCcdCandidateIndex2D.CreateBoundsBetween(
                body.ContinuousCollisionFrameStart,
                body.ContinuousCollisionFrameEnd,
                planarRadius));

        if (!buildMixedIndex)
            return;

        _mixedContinuousCollisionCandidates.Add(
            body.DynamicId,
            body.ResolveMixedContinuousCollisionTrajectoryBounds(planarRadius));
    }


    internal SwiftList<int> QueryPlanarContinuousCollisionCandidates(DynamicCcdPlanarBounds sourceBounds)
    {
        PrepareContinuousCollisionFrame();
        _planarContinuousCollisionCandidates.Query(sourceBounds, _continuousCollisionCandidateIds);
        _dirtyPlanarContinuousCollisionCandidates.Query(sourceBounds, _dirtyContinuousCollisionCandidateIds);
        return MergeContinuousCollisionCandidateIds();
    }

    internal void RefreshContinuousCollisionCandidate(SolidBody2D body)
    {
        Fixed64 radius = body.ResolveContinuousCollisionProxyRadius();
        TryReserveContinuousCollisionCandidateRefresh(body);

        _dirtyPlanarContinuousCollisionCandidates.AddOrUpdate(
            body.DynamicId,
            body.ResolveContinuousCollisionTrajectoryBounds(radius));
        if (_continuousCollisionPreparedMixedIndex)
        {
            _dirtyMixedContinuousCollisionCandidates.AddOrUpdate(
                body.DynamicId,
                body.ResolveMixedContinuousCollisionTrajectoryBounds(radius));
        }
    }

    internal bool CanAdmitContinuousCollisionCandidateRefresh(SolidBody2D body) => true;

    internal bool TryReserveContinuousCollisionCandidateRefresh(SolidBody2D body)
    {
        if (_dirtyPlanarContinuousCollisionBodySet.Add(body))
            _dirtyPlanarContinuousCollisionBodies.Add(body);

        return true;
    }

    private void ReleaseContinuousCollisionCandidateRefresh(SolidBody2D body)
    {
        if (!_dirtyPlanarContinuousCollisionBodySet.Remove(body))
            return;

        _dirtyPlanarContinuousCollisionBodies.Remove(body);
        _dirtyPlanarContinuousCollisionCandidates.Remove(body.DynamicId);
        _dirtyMixedContinuousCollisionCandidates.Remove(body.DynamicId);
    }

    internal bool CanAdmitContinuousCollisionCandidateRefresh(
        SolidBody2D first,
        SolidBody2D second) => true;

    internal bool TryReserveContinuousCollisionCandidateRefresh(
        SolidBody2D first,
        SolidBody2D second)
    {
        TryReserveContinuousCollisionCandidateRefresh(first);
        TryReserveContinuousCollisionCandidateRefresh(second);
        return true;
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
        _dirtyMixedContinuousCollisionCandidates.Query(
            sourceBounds,
            _dirtyContinuousCollisionCandidateIds);
        return MergeContinuousCollisionCandidateIds();
    }

    private SwiftList<int> MergeContinuousCollisionCandidateIds()
    {
        int retainedCount = 0;
        for (int i = 0; i < _continuousCollisionCandidateIds.Count; i++)
        {
            int dynamicId = _continuousCollisionCandidateIds[i];
            if (TryGetContinuousCollisionCandidate(dynamicId, out SolidBody2D body)
                && !_dirtyPlanarContinuousCollisionBodySet.Contains(body))
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

    internal bool TryGetContinuousCollisionCandidate(int dynamicId, out SolidBody2D body)
    {
        ColliderLifetimeToken2D registration = _continuousCollisionCandidateLifetimes[dynamicId];
        if (registration.Collider != null && registration.IsActive)
        {
            body = registration.Collider.Body!;
            return true;
        }

        body = null!;
        return false;
    }

    internal SolidBody2D GetContinuousCollisionCandidate(int dynamicId) =>
        _dynamicBodies[dynamicId];

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
                SolidBody2D body = _continuousCollisionHandoffQueue[readIndex++];
                // Consumption can synchronously return work to this body; dequeue ends dedupe ownership first.
                _queuedContinuousCollisionHandoffBodies.Remove(body);

                if (body.TryConsumeContinuousCollisionHandoff(updateSleepState: false, updateColliderState: false))
                {
                    processed = true;
                    iterations++;
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
