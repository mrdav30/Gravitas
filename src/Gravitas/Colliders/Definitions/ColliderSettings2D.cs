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
    public static CollisionType2D GetCollisionType(ColliderType2D type1, ColliderType2D type2) =>
        (type1, type2) switch
        {
            (ColliderType2D.Circle, ColliderType2D.Circle) => CollisionType2D.Circle_Circle,
            (ColliderType2D.Circle, ColliderType2D.AABox or ColliderType2D.ConvexPolygon) =>
                CollisionType2D.Circle_Convex,
            (ColliderType2D.Circle, ColliderType2D.Capsule) => CollisionType2D.Circle_Capsule,
            (ColliderType2D.AABox or ColliderType2D.ConvexPolygon, ColliderType2D.Circle) =>
                CollisionType2D.Convex_Circle,
            (
                ColliderType2D.AABox or ColliderType2D.ConvexPolygon,
                ColliderType2D.AABox or ColliderType2D.ConvexPolygon) => CollisionType2D.Convex_Convex,
            (ColliderType2D.AABox or ColliderType2D.ConvexPolygon, ColliderType2D.Capsule) =>
                CollisionType2D.Convex_Capsule,
            (ColliderType2D.Capsule, ColliderType2D.Circle) => CollisionType2D.Capsule_Circle,
            (ColliderType2D.Capsule, ColliderType2D.AABox or ColliderType2D.ConvexPolygon) =>
                CollisionType2D.Capsule_Convex,
            (ColliderType2D.Capsule, ColliderType2D.Capsule) => CollisionType2D.Capsule_Capsule,
            (
                ColliderType2D.Compound,
                ColliderType2D.Circle
                    or ColliderType2D.AABox
                    or ColliderType2D.ConvexPolygon
                    or ColliderType2D.Compound
                    or ColliderType2D.Capsule) => CollisionType2D.Compound,
            (
                ColliderType2D.Circle
                    or ColliderType2D.AABox
                    or ColliderType2D.ConvexPolygon
                    or ColliderType2D.Capsule,
                ColliderType2D.Compound) => CollisionType2D.Compound,
            _ => CollisionType2D.None
        };
}
