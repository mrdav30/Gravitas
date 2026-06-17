using FixedMathSharp;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Declares one geometry part owned by an <see cref="LSCompoundCollider"/>.
/// </summary>
public readonly struct CompoundColliderPart
{
    public CompoundColliderPart(LSCollider collider)
        : this(collider, collider != null ? collider.LocalOffset : default, FixedQuaternion.Identity, Vector3d.One)
    {
    }

    public CompoundColliderPart(LSCollider collider, Vector3d localOffset)
        : this(collider, localOffset, FixedQuaternion.Identity, Vector3d.One)
    {
    }

    public CompoundColliderPart(LSCollider collider, FixedQuaternion localRotation)
        : this(collider, collider != null ? collider.LocalOffset : default, localRotation, Vector3d.One)
    {
    }

    public CompoundColliderPart(LSCollider collider, Vector3d localOffset, FixedQuaternion localRotation)
        : this(collider, localOffset, localRotation, Vector3d.One)
    {
    }

    public CompoundColliderPart(LSCollider collider, FixedQuaternion localRotation, Vector3d localScale)
        : this(collider, collider != null ? collider.LocalOffset : default, localRotation, localScale)
    {
    }

    public CompoundColliderPart(
        LSCollider collider,
        Vector3d localOffset,
        FixedQuaternion localRotation,
        Vector3d localScale)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            localScale.X <= Fixed64.Zero || localScale.Y <= Fixed64.Zero || localScale.Z <= Fixed64.Zero,
            nameof(localScale),
            "Compound collider part scale components must be greater than zero.");

        collider.LocalOffset = localOffset;

        Collider = collider;
        LocalOffset = localOffset;
        LocalRotation = localRotation;
        LocalScale = localScale;
    }

    /// <summary>
    /// Gets the part geometry. The part is owned by its compound collider and is
    /// not registered independently with the physics service.
    /// </summary>
    public LSCollider Collider { get; }

    /// <summary>
    /// Gets the deterministic local center offset applied relative to the
    /// owning compound collider.
    /// </summary>
    public Vector3d LocalOffset { get; }

    /// <summary>
    /// Gets the deterministic local rotation applied relative to the owning
    /// compound collider.
    /// </summary>
    public FixedQuaternion LocalRotation { get; }

    /// <summary>
    /// Gets the deterministic local scale applied relative to the owning
    /// compound collider.
    /// </summary>
    public Vector3d LocalScale { get; }

    internal bool IsDefault
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Collider == null;
    }
}
