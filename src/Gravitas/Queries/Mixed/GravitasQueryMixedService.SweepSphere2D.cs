//=======================================================================
// GravitasQueryMixedService.SweepSphere2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Diagnostics;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Queries;

/// <summary>
/// Owns mixed 3D-sphere against embedded 2D-slab sweep query behavior.
/// </summary>
public sealed partial class GravitasQueryMixedService
{
    /// <summary>
    /// Sweeps a 3D sphere against embedded 2D mixed slabs and returns the closest hit.
    /// </summary>
    public bool SweepSphereAgainst2D(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        out PhysicsMixedHit hit,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        return SweepSphereAgainst2DClosestCore(
            start,
            end,
            radius,
            layerMask,
            excludedCollider,
            includeTriggers,
            staticTargetsOnly: false,
            cacheTargetPartitions: false,
            out hit);
    }

    /// <summary>
    /// Sweeps a 3D sphere against embedded 2D mixed slabs and writes hits from closest to farthest.
    /// </summary>
    public int SweepSphereAgainst2DAll(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<PhysicsMixedHit> results,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        return SweepSphereAgainst2DAllCore(
            start,
            end,
            radius,
            layerMask,
            results,
            excludedCollider,
            includeTriggers,
            staticTargetsOnly: false);
    }

    internal int SweepSphereAgainstStatic2DAll(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<PhysicsMixedHit> results,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true,
        bool cacheTargetPartitions = false)
    {
        return SweepSphereAgainst2DAllCore(
            start,
            end,
            radius,
            layerMask,
            results,
            excludedCollider,
            includeTriggers,
            staticTargetsOnly: true,
            cacheTargetPartitions);
    }

    private bool SweepSphereAgainst2DClosestCore(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly,
        bool cacheTargetPartitions,
        out PhysicsMixedHit hit)
    {
        SwiftThrowHelper.ThrowIfArgument(radius <= Fixed64.Zero, nameof(radius), "Mixed swept sphere radius must be greater than zero.");

        if (!Vector3d.TrySubtract(end, start, out Vector3d segment)
            || !Vector3d.TryGetMagnitude(segment, out Fixed64 length)
            || length <= Fixed64.Epsilon)
        {
            ResetLastQueryCounters();
            hit = default;
            return false;
        }

        Vector3d direction = segment.Normalized;
        CreateSweepBounds(start, end, radius, out Vector3d min, out Vector3d max);
        _context.MixedCollisions.Collect2DCandidatesInMixedBounds(
            min,
            max,
            layerMask,
            _candidates2D,
            staticStyleOnly: staticTargetsOnly,
            cachePartitionRefresh: cacheTargetPartitions);
        LastQueryCandidateCount = _candidates2D.Count;
        LastMeshTriangleCandidateCount = 0;
        bool captureReducerDiagnostics = _context.Diagnostics.Enabled;
        QueryReducerCounters reducerCounters = default;
        bool found = false;
        PhysicsMixedHit best = default;

        for (int i = 0; i < _candidates2D.Count; i++)
        {
            if (!TrySweepSphereAgainst2DCandidate(
                start,
                direction,
                length,
                radius,
                _candidates2D[i],
                excludedCollider,
                includeTriggers,
                staticTargetsOnly,
                captureReducerDiagnostics,
                ref reducerCounters,
                out PhysicsMixedHit candidate))
            {
                continue;
            }

            if (PhysicsHitSelectionPolicy.ShouldReplace(candidate, found, best))
            {
                best = candidate;
                found = true;
            }
        }

        hit = found ? best : default;
        EmitMixedSweepDiagnostics(
            GravitasColliderDimension.ThreeD,
            GravitasColliderDimension.TwoD,
            start,
            end,
            radius,
            layerMask.Bits,
            found,
            reducerCounters.AcceptedHits,
            hit,
            captureReducerDiagnostics,
            reducerCounters);
        return found;
    }

    private int SweepSphereAgainst2DAllCore(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<PhysicsMixedHit> results,
        LSCollider? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly,
        bool cacheTargetPartitions = false)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        SwiftThrowHelper.ThrowIfArgument(radius <= Fixed64.Zero, nameof(radius), "Mixed swept sphere radius must be greater than zero.");

        results.FastClear();
        if (!Vector3d.TrySubtract(end, start, out Vector3d segment)
            || !Vector3d.TryGetMagnitude(segment, out Fixed64 length)
            || length <= Fixed64.Epsilon)
        {
            ResetLastQueryCounters();
            return 0;
        }

        Vector3d direction = segment.Normalized;
        CreateSweepBounds(start, end, radius, out Vector3d min, out Vector3d max);
        _context.MixedCollisions.Collect2DCandidatesInMixedBounds(
            min,
            max,
            layerMask,
            _candidates2D,
            staticStyleOnly: staticTargetsOnly,
            cachePartitionRefresh: cacheTargetPartitions);
        LastQueryCandidateCount = _candidates2D.Count;
        LastMeshTriangleCandidateCount = 0;
        bool captureReducerDiagnostics = _context.Diagnostics.Enabled;
        QueryReducerCounters reducerCounters = default;

        for (int i = 0; i < _candidates2D.Count; i++)
        {
            if (TrySweepSphereAgainst2DCandidate(
                start,
                direction,
                length,
                radius,
                _candidates2D[i],
                excludedCollider,
                includeTriggers,
                staticTargetsOnly,
                captureReducerDiagnostics,
                ref reducerCounters,
                out PhysicsMixedHit candidate))
            {
                results.Add(candidate);
            }
        }

        PhysicsMixedHitSorter.SortByDistance(results);
        EmitMixedSweepDiagnostics(
            GravitasColliderDimension.ThreeD,
            GravitasColliderDimension.TwoD,
            start,
            end,
            radius,
            layerMask.Bits,
            results.Count > 0,
            results.Count,
            results.Count > 0 ? results[0] : default,
            captureReducerDiagnostics,
            reducerCounters);
        return results.Count;
    }

}
