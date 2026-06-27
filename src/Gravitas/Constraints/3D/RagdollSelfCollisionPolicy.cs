//=======================================================================
// RagdollSelfCollisionPolicy.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Constraints;

/// <summary>
/// Controls physical collision filtering inside one ragdoll articulation.
/// </summary>
public enum RagdollSelfCollisionPolicy : byte
{
    /// <summary>
    /// Suppresses collision only between directly jointed links.
    /// </summary>
    SuppressAdjacentLinks = 0,

    /// <summary>
    /// Allows all ragdoll links to collide unless a joint overrides its own policy.
    /// </summary>
    CollideAllLinks = 1,

    /// <summary>
    /// Suppresses collision between every pair of links in the ragdoll.
    /// </summary>
    SuppressAllLinks = 2
}
