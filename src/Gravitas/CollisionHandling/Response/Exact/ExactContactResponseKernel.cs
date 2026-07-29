//=======================================================================
// ExactContactResponseKernel.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.CollisionHandling;

/// <summary>
/// Resolves allocation-free exact normal and Coulomb contact response while
/// narrowing only completed body deltas and optional diagnostics.
/// </summary>
internal static partial class ExactContactResponseKernel
{
    // Valid point-anchor denominators are below 202 bits. The largest completed
    // response ratio is below 2,944 bits, including one carry word.
    private const int MaxResponseWords = 48;
}
