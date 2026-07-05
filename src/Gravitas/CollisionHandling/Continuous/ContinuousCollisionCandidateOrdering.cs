//=======================================================================
// ContinuousCollisionCandidateOrdering.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

internal static class ContinuousCollisionCandidateOrdering
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool ShouldReplaceHit(
        Physics3DHit candidate,
        Fixed64 candidateClosingSpeed,
        bool hasCandidate,
        bool hasCurrent,
        Physics3DHit current,
        Fixed64 currentClosingSpeed)
    {
        if (!hasCandidate)
            return false;
        if (!hasCurrent)
            return true;

        int distance = candidate.Distance.CompareTo(current.Distance);
        if (distance != 0)
            return distance < 0;

        int closing = candidateClosingSpeed.CompareTo(currentClosingSpeed);
        if (closing != 0)
            return closing > 0;

        int candidateId = candidate.Collider?.Id ?? -1;
        int currentId = current.Collider?.Id ?? -1;
        return candidateId < currentId;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool ShouldReplaceHit(
        Physics2DHit candidate,
        Fixed64 candidateClosingSpeed,
        bool hasCandidate,
        bool hasCurrent,
        Physics2DHit current,
        Fixed64 currentClosingSpeed)
    {
        if (!hasCandidate)
            return false;
        if (!hasCurrent)
            return true;

        int distance = candidate.Distance.CompareTo(current.Distance);
        if (distance != 0)
            return distance < 0;

        int closing = candidateClosingSpeed.CompareTo(currentClosingSpeed);
        if (closing != 0)
            return closing > 0;

        return candidate.Collider.Id < current.Collider.Id;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool ShouldReplaceMixedHit(
        PhysicsMixedHit candidate,
        Fixed64 candidateClosingSpeed,
        bool hasCandidate,
        bool hasCurrent,
        PhysicsMixedHit current,
        Fixed64 currentClosingSpeed)
    {
        if (!hasCandidate)
            return false;
        if (!hasCurrent)
            return true;

        int distance = candidate.Distance.CompareTo(current.Distance);
        if (distance != 0)
            return distance < 0;

        int closing = candidateClosingSpeed.CompareTo(currentClosingSpeed);
        if (closing != 0)
            return closing > 0;

        int reducer = candidate.ReducerKind.CompareTo(current.ReducerKind);
        if (reducer != 0)
            return reducer < 0;

        int candidate3D = candidate.Collider3D?.Id ?? -1;
        int current3D = current.Collider3D?.Id ?? -1;
        int collider3D = candidate3D.CompareTo(current3D);
        if (collider3D != 0)
            return collider3D < 0;

        int candidate2D = candidate.Collider2D?.Id ?? -1;
        int current2D = current.Collider2D?.Id ?? -1;
        return candidate2D < current2D;
    }

    public static bool IsIgnoredTarget(LSCollider hitCollider, LSCollider? ignored)
    {
        if (ignored == null)
            return false;

        if (ReferenceEquals(hitCollider, ignored))
            return true;

        SolidBody? ignoredBody = ignored.Body;
        if (ignoredBody != null && ReferenceEquals(hitCollider.Body, ignoredBody))
            return true;

        LSCollider? hitTopParent = hitCollider.TopParent3D;
        LSCollider? ignoredTopParent = ignored.TopParent3D;
        return (hitTopParent != null && ReferenceEquals(hitTopParent, ignored))
            || (ignoredTopParent != null && ReferenceEquals(hitCollider, ignoredTopParent))
            || (hitTopParent != null && ignoredTopParent != null && ReferenceEquals(hitTopParent, ignoredTopParent));
    }

    public static bool IsIgnoredTarget(LSCollider2D hitCollider, LSCollider2D? ignored)
    {
        if (ignored == null)
            return false;

        if (ReferenceEquals(hitCollider, ignored))
            return true;

        SolidBody2D? ignoredBody = ignored.Body;
        if (ignoredBody != null && ReferenceEquals(hitCollider.Body, ignoredBody))
            return true;

        LSCollider2D? hitTopParent = hitCollider.TopParent2D;
        LSCollider2D? ignoredTopParent = ignored.TopParent2D;
        return (hitTopParent != null && ReferenceEquals(hitTopParent, ignored))
            || (ignoredTopParent != null && ReferenceEquals(hitCollider, ignoredTopParent))
            || (hitTopParent != null && ignoredTopParent != null && ReferenceEquals(hitTopParent, ignoredTopParent));
    }
}
