//=======================================================================
// JointCollisionPolicy.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Constraints;

/// <summary>
/// Controls physical collision filtering between colliders linked by a joint.
/// </summary>
public enum JointCollisionPolicy : byte
{
    /// <summary>
    /// Suppresses physical collision between the directly linked colliders.
    /// </summary>
    SuppressLinked = 0,

    /// <summary>
    /// Allows the directly linked colliders to collide normally.
    /// </summary>
    Collide = 1
}
