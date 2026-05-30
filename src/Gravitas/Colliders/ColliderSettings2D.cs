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
            ColliderType2D.AABox => 1,
            ColliderType2D.ConvexPolygon => 1,
            _ => -1
        };

    /// <summary>
    /// Resolves the narrow-phase collision type for an ordered pair of 2D collider shapes.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static CollisionType2D GetCollisionType(ColliderType2D type1, ColliderType2D type2)
    {
        bool firstCircle = type1 == ColliderType2D.Circle;
        bool secondCircle = type2 == ColliderType2D.Circle;
        if (firstCircle && secondCircle)
            return CollisionType2D.Circle_Circle;
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
