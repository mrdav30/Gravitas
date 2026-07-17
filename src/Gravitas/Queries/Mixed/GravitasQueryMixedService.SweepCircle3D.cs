//=======================================================================
// GravitasQueryMixedService.SweepCircle3D.cs
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
/// Owns mixed 2D-circle against 3D-collider sweep query behavior.
/// </summary>
public sealed partial class GravitasQueryMixedService
{
    /// <summary>
    /// Sweeps a 2D circle embedded in a finite mixed slab against 3D colliders and returns the closest hit.
    /// </summary>
    public bool SweepCircleAgainst3D(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        PhysicsLayerMask layerMask,
        out PhysicsMixedHit hit,
        LSCollider2D? excludedCollider = null,
        bool includeTriggers = true)
    {
        return SweepCircleAgainst3DClosestCore(
            start,
            end,
            radius,
            slabCenterY,
            halfThickness,
            layerMask,
            excludedCollider,
            includeTriggers,
            staticTargetsOnly: false,
            cacheTargetPartitions: false,
            out hit);
    }

    /// <summary>
    /// Sweeps a 2D circle embedded in a finite mixed slab against 3D colliders and writes hits from closest to farthest.
    /// </summary>
    public int SweepCircleAgainst3DAll(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        PhysicsLayerMask layerMask,
        SwiftList<PhysicsMixedHit> results,
        LSCollider2D? excludedCollider = null,
        bool includeTriggers = true)
    {
        return SweepCircleAgainst3DAllCore(
            start,
            end,
            radius,
            slabCenterY,
            halfThickness,
            layerMask,
            results,
            excludedCollider,
            includeTriggers,
            staticTargetsOnly: false);
    }

    internal int SweepCircleAgainstStatic3DAll(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        PhysicsLayerMask layerMask,
        SwiftList<PhysicsMixedHit> results,
        LSCollider2D? excludedCollider = null,
        bool includeTriggers = true,
        bool cacheTargetPartitions = false)
    {
        return SweepCircleAgainst3DAllCore(
            start,
            end,
            radius,
            slabCenterY,
            halfThickness,
            layerMask,
            results,
            excludedCollider,
            includeTriggers,
            staticTargetsOnly: true,
            cacheTargetPartitions);
    }

    private bool SweepCircleAgainst3DClosestCore(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        PhysicsLayerMask layerMask,
        LSCollider2D? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly,
        bool cacheTargetPartitions,
        out PhysicsMixedHit hit)
    {
        SwiftThrowHelper.ThrowIfArgument(radius <= Fixed64.Zero, nameof(radius), "Mixed swept circle radius must be greater than zero.");
        SwiftThrowHelper.ThrowIfArgument(halfThickness <= Fixed64.Zero, nameof(halfThickness), "Mixed swept circle half-thickness must be greater than zero.");

        if (!Vector2d.TrySubtract(end, start, out Vector2d segment)
            || !Vector2d.TryGetMagnitude(segment, out Fixed64 length)
            || length <= Fixed64.Epsilon)
        {
            ResetLastQueryCounters();
            hit = default;
            return false;
        }

        Vector3d start3D = new(start.X, slabCenterY, start.Y);
        Vector3d end3D = new(end.X, slabCenterY, end.Y);
        Vector2d direction2D = segment.Normalized;
        Vector3d direction = new(direction2D.X, Fixed64.Zero, direction2D.Y);
        _circleSlabSweepWorker.PrepareCircleSlabSource(start3D, radius, halfThickness, direction * length);
        CreateCircleSlabSweepBounds(start, end, radius, slabCenterY, halfThickness, out Vector3d min, out Vector3d max);
        _context.MixedCollisions.Collect3DCandidatesInMixedBounds(
            min,
            max,
            layerMask,
            _candidates3D,
            staticStyleOnly: staticTargetsOnly,
            cachePartitionRefresh: cacheTargetPartitions);
        LastQueryCandidateCount = _candidates3D.Count;
        LastMeshTriangleCandidateCount = 0;
        bool captureReducerDiagnostics = _context.Diagnostics.Enabled;
        QueryReducerCounters reducerCounters = default;
        bool found = false;
        PhysicsMixedHit best = default;

        for (int i = 0; i < _candidates3D.Count; i++)
        {
            if (!TrySweepCircleAgainst3DCandidate(
                _candidates3D[i],
                start,
                direction2D,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction,
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
            GravitasColliderDimension.TwoD,
            GravitasColliderDimension.ThreeD,
            start3D,
            end3D,
            radius,
            layerMask.Bits,
            found,
            reducerCounters.AcceptedHits,
            hit,
            captureReducerDiagnostics,
            reducerCounters);
        return found;
    }

    private int SweepCircleAgainst3DAllCore(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        PhysicsLayerMask layerMask,
        SwiftList<PhysicsMixedHit> results,
        LSCollider2D? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly,
        bool cacheTargetPartitions = false)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        SwiftThrowHelper.ThrowIfArgument(radius <= Fixed64.Zero, nameof(radius), "Mixed swept circle radius must be greater than zero.");
        SwiftThrowHelper.ThrowIfArgument(halfThickness <= Fixed64.Zero, nameof(halfThickness), "Mixed swept circle half-thickness must be greater than zero.");

        results.FastClear();
        if (!Vector2d.TrySubtract(end, start, out Vector2d segment)
            || !Vector2d.TryGetMagnitude(segment, out Fixed64 length)
            || length <= Fixed64.Epsilon)
        {
            ResetLastQueryCounters();
            return 0;
        }

        Vector3d start3D = new(start.X, slabCenterY, start.Y);
        Vector3d end3D = new(end.X, slabCenterY, end.Y);
        Vector2d direction2D = segment.Normalized;
        Vector3d direction = new(direction2D.X, Fixed64.Zero, direction2D.Y);
        _circleSlabSweepWorker.PrepareCircleSlabSource(start3D, radius, halfThickness, direction * length);
        CreateCircleSlabSweepBounds(start, end, radius, slabCenterY, halfThickness, out Vector3d min, out Vector3d max);
        _context.MixedCollisions.Collect3DCandidatesInMixedBounds(
            min,
            max,
            layerMask,
            _candidates3D,
            staticStyleOnly: staticTargetsOnly,
            cachePartitionRefresh: cacheTargetPartitions);
        LastQueryCandidateCount = _candidates3D.Count;
        LastMeshTriangleCandidateCount = 0;
        bool captureReducerDiagnostics = _context.Diagnostics.Enabled;
        QueryReducerCounters reducerCounters = default;

        for (int i = 0; i < _candidates3D.Count; i++)
        {
            if (!TrySweepCircleAgainst3DCandidate(
                _candidates3D[i],
                start,
                direction2D,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction,
                excludedCollider,
                includeTriggers,
                staticTargetsOnly,
                captureReducerDiagnostics,
                ref reducerCounters,
                out PhysicsMixedHit candidate))
            {
                continue;
            }

            results.Add(candidate);
        }

        PhysicsMixedHitSorter.SortByDistance(results);
        EmitMixedSweepDiagnostics(
            GravitasColliderDimension.TwoD,
            GravitasColliderDimension.ThreeD,
            start3D,
            end3D,
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
