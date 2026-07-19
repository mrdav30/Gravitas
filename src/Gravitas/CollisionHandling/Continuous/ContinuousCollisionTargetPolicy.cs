//=======================================================================
// ContinuousCollisionTargetPolicy.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.CollisionHandling;

internal static class ContinuousCollisionTargetPolicy
{
    internal static bool AllowsDynamic3DTarget(
        bool isSelf,
        bool active,
        bool positionFullyFrozen,
        bool kinematic,
        bool trigger,
        bool physicalPairRequired) =>
        !isSelf
        && active
        && !positionFullyFrozen
        && !kinematic
        && !trigger
        && physicalPairRequired;

    internal static bool AllowsDynamic2DTarget(
        bool isSelf,
        bool active,
        bool positionFullyFrozen,
        bool kinematic,
        bool trigger,
        bool physicalPairRequired) =>
        !isSelf
        && active
        && !positionFullyFrozen
        && !kinematic
        && !trigger
        && physicalPairRequired;

    internal static bool AllowsMixedDynamicTarget(
        bool active,
        bool positionFullyFrozen,
        bool kinematic,
        bool trigger,
        bool mixedPairRequired) =>
        active
        && !positionFullyFrozen
        && !kinematic
        && !trigger
        && mixedPairRequired;

    internal static bool AllowsIndexed3DTarget(
        bool isSelf,
        bool active,
        bool positionFullyFrozen,
        bool kinematic,
        bool movingKinematic,
        bool trigger,
        bool physicalPairRequired) =>
        !isSelf
        && active
        && (kinematic ? movingKinematic : !positionFullyFrozen)
        && !trigger
        && physicalPairRequired;

    internal static bool AllowsIndexed2DTarget(
        bool isSelf,
        bool active,
        bool positionFullyFrozen,
        bool kinematic,
        bool movingKinematic,
        bool trigger,
        bool physicalPairRequired) =>
        !isSelf
        && active
        && (kinematic ? movingKinematic : !positionFullyFrozen)
        && !trigger
        && physicalPairRequired;

    internal static bool AllowsMixedIndexedTarget(
        bool active,
        bool positionFullyFrozen,
        bool kinematic,
        bool movingKinematic,
        bool trigger,
        bool mixedPairRequired) =>
        active
        && (kinematic ? movingKinematic : !positionFullyFrozen)
        && !trigger
        && mixedPairRequired;

    internal static bool AllowsStaticOrKinematic3DTarget(
        bool hasCollider,
        bool isSelf,
        bool ignored,
        bool trigger,
        bool physicalPairRequired,
        bool isStatic,
        bool bodyKinematic) =>
        hasCollider
        && !isSelf
        && !ignored
        && !trigger
        && physicalPairRequired
        && (isStatic || bodyKinematic);

    internal static bool AllowsStaticOrKinematic2DTarget(
        bool hasCollider,
        bool isSelf,
        bool ignored,
        bool trigger,
        bool physicalPairRequired,
        bool isStatic,
        bool bodyKinematic) =>
        hasCollider
        && !isSelf
        && !ignored
        && !trigger
        && physicalPairRequired
        && (isStatic || bodyKinematic);

    internal static bool AllowsMixedStaticOrKinematicTarget(
        bool hasCollider,
        bool ignored,
        bool trigger,
        bool mixedPairRequired,
        bool isStatic,
        bool bodyKinematic) =>
        hasCollider
        && !ignored
        && !trigger
        && mixedPairRequired
        && (isStatic || bodyKinematic);
}
