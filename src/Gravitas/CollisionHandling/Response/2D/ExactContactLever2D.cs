//=======================================================================
// ExactContactLever2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Adapts planar body mobility and signed yaw to the shared exact 3D response
/// kernel.
/// </summary>
internal static class ExactContactLever2D
{
    internal static bool CanUseCompactResponse(
        SolidBody2D? bodyA,
        Vector2d linearVelocityA,
        Fixed64 angularVelocityA,
        Vector2d relativeContactPointA,
        SolidBody2D? bodyB,
        Vector2d linearVelocityB,
        Fixed64 angularVelocityB,
        Vector2d relativeContactPointB,
        Vector2d axis)
    {
        Vector3d spatialAxis = ToSpatial(axis);
        Vector3d leverA = ToSpatial(relativeContactPointA);
        Vector3d leverB = ToSpatial(relativeContactPointB);
        return ContactResponseArithmetic3D.CanUseFastPointVelocity(
                ToSpatial(linearVelocityA),
                new Vector3d(
                    Fixed64.Zero,
                    -angularVelocityA,
                    Fixed64.Zero),
                leverA,
                ToSpatial(linearVelocityB),
                new Vector3d(
                    Fixed64.Zero,
                    -angularVelocityB,
                    Fixed64.Zero),
                leverB,
                spatialAxis)
            && ContactResponseArithmetic3D.CanUseFastAngularResponse(
                leverA,
                spatialAxis,
                CreateInverseInertia(bodyA))
            && ContactResponseArithmetic3D.CanUseFastAngularResponse(
                leverB,
                spatialAxis,
                CreateInverseInertia(bodyB));
    }

    internal static ExactContactResponseOperand3D CreateResponseOperand(
        SolidBody2D? body,
        Vector2d linearVelocity,
        Fixed64 angularVelocity,
        in ExactLever3D lever,
        Vector3d signedAxis) =>
        new(
            lever,
            ToSpatial(linearVelocity),
            new Vector3d(
                Fixed64.Zero,
                -angularVelocity,
                Fixed64.Zero),
            body == null
                ? Vector3d.Zero
                : ToSpatial(body.ProjectLinearMotion(ToPlanar(signedAxis))),
            body?.EffectiveInverseMass ?? Fixed64.Zero,
            CreateInverseInertia(body));

    internal static bool TryGetNormalResponse(
        SolidBody2D? bodyA,
        Vector2d linearVelocityA,
        Fixed64 angularVelocityA,
        in ExactLever3D relativeContactPointA,
        SolidBody2D? bodyB,
        Vector2d linearVelocityB,
        Fixed64 angularVelocityB,
        in ExactLever3D relativeContactPointB,
        Vector2d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        out ExactNormalResponse3D response)
    {
        Vector3d spatialNormal = ToSpatial(normal);
        ExactContactResponseOperand3D first = CreateResponseOperand(
            bodyA,
            linearVelocityA,
            angularVelocityA,
            relativeContactPointA,
            -spatialNormal);
        ExactContactResponseOperand3D second = CreateResponseOperand(
            bodyB,
            linearVelocityB,
            angularVelocityB,
            relativeContactPointB,
            spatialNormal);
        return ExactContactResponseKernel.TryGetNormalResponse(
            first,
            second,
            spatialNormal,
            restitution,
            restitutionVelocityThreshold,
            out response);
    }

    internal static bool TryGetAccumulatedNormalResponse(
        SolidBody2D? bodyA,
        Vector2d linearVelocityA,
        Fixed64 angularVelocityA,
        in ExactLever3D relativeContactPointA,
        SolidBody2D? bodyB,
        Vector2d linearVelocityB,
        Fixed64 angularVelocityB,
        in ExactLever3D relativeContactPointB,
        Vector2d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 accumulatedImpulse,
        Fixed64 positiveImpulseScale,
        Fixed64 negativeImpulseScale,
        out ExactNormalResponse3D response)
    {
        Vector3d spatialNormal = ToSpatial(normal);
        ExactContactResponseOperand3D first = CreateResponseOperand(
            bodyA,
            linearVelocityA,
            angularVelocityA,
            relativeContactPointA,
            -spatialNormal);
        ExactContactResponseOperand3D second = CreateResponseOperand(
            bodyB,
            linearVelocityB,
            angularVelocityB,
            relativeContactPointB,
            spatialNormal);
        return ExactContactResponseKernel.TryGetAccumulatedNormalResponse(
            first,
            second,
            spatialNormal,
            restitution,
            restitutionVelocityThreshold,
            accumulatedImpulse,
            positiveImpulseScale,
            negativeImpulseScale,
            out response);
    }

    internal static Vector3d ToSpatial(Vector2d vector) =>
        new(vector.X, Fixed64.Zero, vector.Y);

    internal static Vector2d ToPlanar(Vector3d vector) =>
        new(vector.X, vector.Z);

    internal static Fixed64 ToPlanarAngular(Vector3d vector) =>
        -vector.Y;

    internal static bool TryGetImpulseVelocityDeltas(
        SolidBody2D? bodyA,
        in ExactLever3D relativeContactPointA,
        SolidBody2D? bodyB,
        in ExactLever3D relativeContactPointB,
        Vector2d firstAxis,
        Fixed64 firstScale,
        Vector2d secondAxis,
        Fixed64 secondScale,
        out Vector2d linearVelocityDeltaA,
        out Fixed64 angularVelocityDeltaA,
        out Vector2d linearVelocityDeltaB,
        out Fixed64 angularVelocityDeltaB)
    {
        Vector3d first = ToSpatial(firstAxis);
        Vector3d second = ToSpatial(secondAxis);
        bool firstResolved = TryGetParticipantVelocityDeltas(
            bodyA,
            relativeContactPointA,
            -first,
            firstScale,
            -second,
            secondScale,
            out linearVelocityDeltaA,
            out angularVelocityDeltaA);
        bool secondResolved = TryGetParticipantVelocityDeltas(
            bodyB,
            relativeContactPointB,
            first,
            firstScale,
            second,
            secondScale,
            out linearVelocityDeltaB,
            out angularVelocityDeltaB);
        return firstResolved & secondResolved;
    }

    internal static bool TryGetParticipantVelocityDeltas(
        SolidBody2D? body,
        in ExactLever3D lever,
        Vector3d firstAxis,
        Fixed64 firstScale,
        Vector3d secondAxis,
        Fixed64 secondScale,
        out Vector2d linearVelocityDelta,
        out Fixed64 angularVelocityDelta)
    {
        linearVelocityDelta = Vector2d.Zero;
        angularVelocityDelta = Fixed64.Zero;
        if (body?.HasSolverMobility != true)
            return true;

        Vector3d spatialLinear = Vector3d.Zero;
        bool linearResolved = !body.CanTranslate
            || Vector3d.TryScaledLinearCombination(
                ToSpatial(body.ProjectLinearMotion(ToPlanar(firstAxis))),
                firstScale,
                ToSpatial(body.ProjectLinearMotion(ToPlanar(secondAxis))),
                secondScale,
                Vector3d.Zero,
                Fixed64.Zero,
                body.EffectiveInverseMass,
                out spatialLinear);
        Vector3d spatialAngular = Vector3d.Zero;
        bool angularResolved = !body.CanRotate
            || ExactLever3D.TryGetTransformedWeightedCrossProduct(
                lever,
                firstAxis,
                firstScale,
                secondAxis,
                secondScale,
                Vector3d.Zero,
                Fixed64.Zero,
                CreateInverseInertia(body),
                out spatialAngular);
        linearVelocityDelta = ToPlanar(spatialLinear);
        angularVelocityDelta = ToPlanarAngular(spatialAngular);
        return linearResolved & angularResolved;
    }

    internal static Fixed3x3 CreateInverseInertia(SolidBody2D? body) =>
        body?.CanRotate == true
            ? new Fixed3x3(
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, body.EffectiveInverseMomentOfInertia, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero)
            : Fixed3x3.Zero;
}
