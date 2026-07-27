//=======================================================================
// DynamicMixedIntervalHit.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.CollisionHandling;
using Gravitas.Queries;

namespace Gravitas;

internal readonly struct DynamicMixedIntervalHit
{
    internal DynamicMixedIntervalHit(
        ContinuousCollisionMath.IntervalSearchStatus status,
        PhysicsMixedHit exactHit,
        Fixed64 safeDistance,
        Fixed64 closingSpeed,
        int targetId)
    {
        Status = status;
        ExactHit = exactHit;
        SafeDistance = safeDistance;
        ClosingSpeed = closingSpeed;
        TargetId = targetId;
    }

    internal ContinuousCollisionMath.IntervalSearchStatus Status { get; }

    internal PhysicsMixedHit ExactHit { get; }

    internal Fixed64 SafeDistance { get; }

    internal Fixed64 ClosingSpeed { get; }

    internal int TargetId { get; }

    internal static bool ShouldReplace(
        DynamicMixedIntervalHit candidate,
        DynamicMixedIntervalHit current,
        bool hasCurrent)
    {
        if (!hasCurrent)
            return true;

        int distance = candidate.SafeDistance.CompareTo(current.SafeDistance);
        if (distance != 0)
            return distance < 0;

        bool candidateIsExact =
            candidate.Status == ContinuousCollisionMath.IntervalSearchStatus.ExactHit;
        bool currentIsExact =
            current.Status == ContinuousCollisionMath.IntervalSearchStatus.ExactHit;
        if (candidateIsExact != currentIsExact)
            return candidateIsExact;

        if (candidateIsExact)
        {
            return ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(
                candidate.ExactHit,
                candidate.ClosingSpeed,
                true,
                true,
                current.ExactHit,
                current.ClosingSpeed);
        }

        return candidate.TargetId < current.TargetId;
    }

    internal static DynamicMixedIntervalHit Select(
        DynamicMixedIntervalHit candidate,
        DynamicMixedIntervalHit current,
        ref bool hasCurrent)
    {
        if (!ShouldReplace(candidate, current, hasCurrent))
            return current;

        hasCurrent = true;
        return candidate;
    }

    internal static bool ShouldReplaceStatic(
        DynamicMixedIntervalHit candidate,
        bool hasCandidate,
        PhysicsMixedHit current,
        bool hasCurrent)
    {
        if (!hasCandidate)
            return false;
        if (!hasCurrent)
            return true;
        if (candidate.SafeDistance < current.Distance)
            return true;
        if (candidate.Status != ContinuousCollisionMath.IntervalSearchStatus.ExactHit)
            return false;

        return ContinuousCollisionCandidateOrdering.ShouldReplaceMixedHit(
            candidate.ExactHit,
            candidate.ClosingSpeed,
            true,
            true,
            current,
            Fixed64.Zero);
    }

    internal static bool ShouldSelect2D(
        bool has2D,
        Fixed64 distance2D,
        bool hasMixed,
        Fixed64 distanceMixed)
    {
        if (!has2D)
            return false;
        if (!hasMixed)
            return true;

        return ContinuousCollisionCandidateOrdering.Is2DHitFirst(
            distance2D,
            distanceMixed);
    }

    internal static bool ShouldSelect3D(
        bool has3D,
        Fixed64 distance3D,
        bool hasMixed,
        Fixed64 distanceMixed)
    {
        if (!has3D)
            return false;
        if (!hasMixed)
            return true;

        return !ContinuousCollisionCandidateOrdering.Is2DHitFirst(
            distanceMixed,
            distance3D);
    }
}
