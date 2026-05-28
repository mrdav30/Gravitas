using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Allocation-free narrow-phase work item used by real pairs and compound
/// collider internal part checks.
/// </summary>
internal readonly struct CollisionWorkItem
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public CollisionWorkItem(
        GravitasWorldContext context,
        LSCollider colliderA,
        LSCollider colliderB,
        CollisionType collisionType,
        ContactManifold manifold)
    {
        Context = context;
        ColliderA = colliderA;
        ColliderB = colliderB;
        CollisionType = collisionType;
        Manifold = manifold;
    }

    public GravitasWorldContext Context { get; }

    public LSCollider ColliderA { get; }

    public LSCollider ColliderB { get; }

    public CollisionType CollisionType { get; }

    public ContactManifold Manifold { get; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static CollisionWorkItem Create(CollisionPair pair) =>
        new(pair.Context, pair.ColliderA, pair.ColliderB, pair.CollisionType, pair.Manifold);
}
