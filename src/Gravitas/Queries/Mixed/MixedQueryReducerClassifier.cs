//=======================================================================
// MixedQueryReducerClassifier.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;

namespace Gravitas.Queries;

/// <summary>
/// Classifies mixed swept-query reducers before the exact candidate path runs.
/// </summary>
internal static class MixedQueryReducerClassifier
{
    internal static PhysicsQueryReducerKind ClassifySweepSphereAgainst2D(LSCollider2D collider)
    {
        if (collider is LSCircleCollider2D
            || collider is LSCapsuleCollider2D
            || collider is LSAABBoxCollider2D
            || collider is LSPolygonCollider2D)
        {
            return PhysicsQueryReducerKind.Exact;
        }

        if (collider is LSCompoundCollider2D)
            return PhysicsQueryReducerKind.Exact;

        return PhysicsQueryReducerKind.ConservativeFallback;
    }

    internal static PhysicsQueryReducerKind ClassifySweepCircleAgainst3D(LSCollider collider)
    {
        if (collider is LSSphereCollider
            || collider is LSCuboidCollider
            || collider is LSCapsuleCollider
            || collider is LSCylinderCollider
            || collider is LSConeCollider
            || collider is LSMeshCollider)
        {
            return PhysicsQueryReducerKind.Exact;
        }

        if (collider is LSCompoundCollider)
            return PhysicsQueryReducerKind.Exact;

        return PhysicsQueryReducerKind.ConservativeFallback;
    }
}
