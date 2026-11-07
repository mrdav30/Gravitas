//=======================================================================
// ContinuousCollisionRotationPath3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.CollisionHandling;

/// <summary>
/// Selects angular-velocity integration or kinematic interpolation for a 3D rotational sweep.
/// </summary>
internal enum ContinuousCollisionRotationPath3D : byte
{
    IntegratedAngularVelocity,
    KinematicSlerp
}
