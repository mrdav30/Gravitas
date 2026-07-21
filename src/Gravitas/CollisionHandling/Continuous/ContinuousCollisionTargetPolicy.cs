//=======================================================================
// ContinuousCollisionTargetPolicy.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.CollisionHandling;

internal static class ContinuousCollisionTargetPolicy
{
    internal static bool AllowsIndexed3DTarget(
        bool isSelf,
        bool active,
        bool dynamicBody,
        bool kinematic,
        bool movingKinematic,
        bool trigger,
        bool physicalPairRequired) =>
        !isSelf
        && active
        && (kinematic ? movingKinematic : dynamicBody)
        && !trigger
        && physicalPairRequired;

    internal static bool AllowsIndexed2DTarget(
        bool isSelf,
        bool active,
        bool dynamicBody,
        bool kinematic,
        bool movingKinematic,
        bool trigger,
        bool physicalPairRequired) =>
        !isSelf
        && active
        && (kinematic ? movingKinematic : dynamicBody)
        && !trigger
        && physicalPairRequired;

    internal static bool AllowsMixedIndexedTarget(
        bool active,
        bool dynamicBody,
        bool kinematic,
        bool movingKinematic,
        bool trigger,
        bool mixedPairRequired) =>
        active
        && (kinematic ? movingKinematic : dynamicBody)
        && !trigger
        && mixedPairRequired;

    internal static bool AllowsStaticOrKinematic3DTarget(
        bool isSelf,
        bool ignored,
        bool trigger,
        bool physicalPairRequired,
        bool isStatic,
        bool bodyKinematic) =>
        !isSelf
        && !ignored
        && !trigger
        && physicalPairRequired
        && (isStatic || bodyKinematic);

    internal static bool AllowsStaticOrKinematic2DTarget(
        bool isSelf,
        bool ignored,
        bool trigger,
        bool physicalPairRequired,
        bool isStatic,
        bool bodyKinematic) =>
        !isSelf
        && !ignored
        && !trigger
        && physicalPairRequired
        && (isStatic || bodyKinematic);

    internal static bool AllowsMixedStaticOrKinematicTarget(
        bool ignored,
        bool trigger,
        bool mixedPairRequired,
        bool isStatic,
        bool bodyKinematic) =>
        !ignored
        && !trigger
        && mixedPairRequired
        && (isStatic || bodyKinematic);
}
