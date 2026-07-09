//=======================================================================
// PhysicsHitSelectionPolicy.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

internal static class PhysicsHitSelectionPolicy
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool ShouldReplace(Physics2DHit candidate, bool found, Physics2DHit current) =>
        !found || Physics2DHitSorter.ComesBefore(candidate, current);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool ShouldReplace(Physics3DHit candidate, bool found, Physics3DHit current) =>
        !found || Physics3DHitSorter.ComesBefore(candidate, current);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool ShouldReplace(PhysicsMixedHit candidate, bool found, PhysicsMixedHit current) =>
        !found || PhysicsMixedHitSorter.ComesBefore(candidate, current);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool ShouldReplaceDistance(Fixed64 candidateDistance, bool found, Fixed64 currentDistance) =>
        !found || candidateDistance < currentDistance;
}
