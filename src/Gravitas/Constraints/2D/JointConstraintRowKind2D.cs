//=======================================================================
// JointConstraintRowKind2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Constraints;

/// <summary>
/// Distinguishes linear, angular, and motor rows in the planar joint solver.
/// </summary>
internal enum JointConstraintRowKind2D : byte
{
    Linear,
    Angular,
    Motor
}
