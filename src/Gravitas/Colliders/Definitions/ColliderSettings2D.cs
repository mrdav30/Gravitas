//=======================================================================
// ColliderSettings2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Provides deterministic priority and collision-dispatch metadata for pure 2D collider shapes.
/// </summary>
public static class ColliderSettings2D
{
    /// <summary>
    /// Gets the narrow-phase priority used to order 2D colliders before dispatch.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int GetPriority(ColliderType2D type) =>
        type switch
        {
            ColliderType2D.Circle => 0,
            ColliderType2D.Capsule => 0,
            ColliderType2D.AABox => 1,
            ColliderType2D.ConvexPolygon => 1,
            ColliderType2D.Compound => 2,
            _ => -1
        };

    /// <summary>
    /// Resolves the narrow-phase collision type for an ordered pair of 2D collider shapes.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static CollisionType2D GetCollisionType(ColliderType2D type1, ColliderType2D type2)
    {
        if (type1 == ColliderType2D.Compound || type2 == ColliderType2D.Compound)
            return CollisionType2D.Compound;

        bool firstCircle = type1 == ColliderType2D.Circle;
        bool secondCircle = type2 == ColliderType2D.Circle;
        bool firstCapsule = type1 == ColliderType2D.Capsule;
        bool secondCapsule = type2 == ColliderType2D.Capsule;
        if (firstCircle && secondCircle)
            return CollisionType2D.Circle_Circle;
        if (firstCapsule && secondCircle)
            return CollisionType2D.Capsule_Circle;
        if (firstCircle && secondCapsule)
            return CollisionType2D.Circle_Capsule;
        if (firstCapsule && secondCapsule)
            return CollisionType2D.Capsule_Capsule;
        if (firstCapsule)
            return IsConvex(type2) ? CollisionType2D.Capsule_Convex : CollisionType2D.None;
        if (secondCapsule)
            return IsConvex(type1) ? CollisionType2D.Convex_Capsule : CollisionType2D.None;
        if (firstCircle)
            return IsConvex(type2) ? CollisionType2D.Circle_Convex : CollisionType2D.None;
        if (secondCircle)
            return IsConvex(type1) ? CollisionType2D.Convex_Circle : CollisionType2D.None;

        return IsConvex(type1) && IsConvex(type2)
            ? CollisionType2D.Convex_Convex
            : CollisionType2D.None;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsConvex(ColliderType2D type) =>
        type == ColliderType2D.AABox || type == ColliderType2D.ConvexPolygon;
}
