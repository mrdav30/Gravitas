//=======================================================================
// RagdollLinkDefinition3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;

namespace Gravitas.Constraints;

/// <summary>
/// Authored link entry for a deterministic 3D ragdoll articulation.
/// </summary>
public readonly struct RagdollLinkDefinition3D
{
    /// <summary>
    /// Creates a ragdoll link definition.
    /// </summary>
    public RagdollLinkDefinition3D(int linkId, SolidBody body)
        : this(linkId, body, body?.Collider!)
    {
    }

    /// <summary>
    /// Creates a ragdoll link definition.
    /// </summary>
    public RagdollLinkDefinition3D(int linkId, SolidBody body, LSCollider collider)
    {
        LinkId = linkId;
        Body = body;
        Collider = collider;
    }

    /// <summary>
    /// Gets the stable authored link ID.
    /// </summary>
    public int LinkId { get; }

    /// <summary>
    /// Gets the body represented by this ragdoll link.
    /// </summary>
    public SolidBody Body { get; }

    /// <summary>
    /// Gets the collider represented by this ragdoll link.
    /// </summary>
    public LSCollider Collider { get; }
}
