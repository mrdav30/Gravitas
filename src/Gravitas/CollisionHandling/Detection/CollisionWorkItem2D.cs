using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas;

internal readonly struct CollisionWorkItem2D
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public CollisionWorkItem2D(
        LSCollider2D colliderA,
        LSCollider2D colliderB,
        CollisionType2D collisionType)
    {
        ColliderA = colliderA;
        ColliderB = colliderB;
        CollisionType = collisionType;
    }

    public LSCollider2D ColliderA { get; }

    public LSCollider2D ColliderB { get; }

    public CollisionType2D CollisionType { get; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static CollisionWorkItem2D Create(CollisionPair2D pair) =>
        new(pair.ColliderA, pair.ColliderB, pair.CollisionType);
}
