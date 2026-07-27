//=======================================================================
// ContactSelectionPolicy.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

internal static class ContactSelectionPolicy
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool ShouldReplaceWithDeeper(Contact2D candidate, bool found, Contact2D current) =>
        !found
        || candidate.Depth > current.Depth
        || (candidate.Depth == current.Depth
            && candidate.DepthIsClamped
            && !current.DepthIsClamped);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool ShouldReplaceWithShallower(MixedContact candidate, bool found, MixedContact current) =>
        !found
        || candidate.Depth < current.Depth
        || (candidate.Depth == current.Depth
            && current.DepthIsClamped
            && !candidate.DepthIsClamped);
}
