//=======================================================================
// RagdollLinkDefinition2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;

namespace Gravitas.Constraints;

/// <summary>
/// Authored link entry for a deterministic pure 2D ragdoll articulation.
/// </summary>
public readonly struct RagdollLinkDefinition2D
{
    /// <summary>
    /// Creates a pure 2D ragdoll link definition.
    /// </summary>
    public RagdollLinkDefinition2D(int linkId, SolidBody2D body)
    {
        SwiftThrowHelper.ThrowIfNull(body, nameof(body));

        LinkId = linkId;
        Body = body;
        Collider = body.Collider;
    }

    /// <summary>
    /// Gets the stable authored link ID.
    /// </summary>
    public int LinkId { get; }

    /// <summary>
    /// Gets the body represented by this ragdoll link.
    /// </summary>
    public SolidBody2D Body { get; }

    /// <summary>
    /// Gets the collider represented by this ragdoll link.
    /// </summary>
    public LSCollider2D Collider { get; }
}
