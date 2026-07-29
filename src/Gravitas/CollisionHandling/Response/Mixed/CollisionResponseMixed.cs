//=======================================================================
// CollisionResponseMixed.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.Materials;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Solves deterministic mixed 3D/2D contact response with the 2D body constrained to its X/Z plane.
/// </summary>
public static class CollisionResponseMixed
{
    public static readonly Fixed64 PenetrationSlop = (Fixed64)0.01f;

    public static readonly Fixed64 PenetrationCorrectionPercent = (Fixed64)0.8f;

    internal static bool Resolve(CollisionPairMixed pair, MixedContact contact) =>
        Resolve(pair, contact, iteration: 0, iterationLimit: 1, applyPositionCorrection: true);

    internal static bool Resolve(
        CollisionPairMixed pair,
        MixedContact contact,
        int iteration,
        int iterationLimit,
        bool applyPositionCorrection)
    {
        if (!contact.HasContact || pair.Collider3D.IsTrigger || pair.Collider2D.IsTrigger)
            return false;

        SolidBody? body3D = pair.Collider3D.Body;
        SolidBody2D? body2D = pair.Collider2D.Body;
        Vector3d normal = ResolveNormal(pair, contact);
        bool hasPlanarResponseCoupling = normal.ToVector2d() != Vector2d.Zero;

        Fixed64 inverseMass3D = body3D?.EffectiveInverseMass ?? Fixed64.Zero;
        Fixed64 inverseMass2D = body2D?.EffectiveInverseMass ?? Fixed64.Zero;
        Fixed64 correctionInverseMass = GetConstrainedInverseMass(body3D, normal)
            + GetConstrainedPlanarInverseMass(body2D, normal);

        Vector2d relative2D = default;
        bool resolved3D = body3D != null
            ? body3D.TryGetOffsetFromCenterOfMass(contact.Anchor3D, out Vector3d relative3D)
            : contact.Anchor3D.TryGetOffsetFrom(pair.Collider3D.Center, out relative3D);
        bool resolved2D = body2D == null
            || contact.TryGetPlanarOffset2DFrom(
                body2D.Position,
                body2D.Rotation,
                body2D.LocalCenterOfMassOffset,
                out relative2D);
        if (body2D == null)
            relative2D = Vector2d.Zero;
        Fixed64 inverseMoment2D = body2D?.EffectiveInverseMomentOfInertia ?? Fixed64.Zero;
        Vector3d normalLinearVelocity3D =
            body3D == null
                ? Vector3d.Zero
                : ResolveLinearVelocity(body3D);
        Vector3d normalAngularVelocity3D =
            body3D == null
                ? Vector3d.Zero
                : ResolveAngularVelocity(body3D);
        Vector2d normalLinearVelocity2D =
            body2D == null
                ? Vector2d.Zero
                : ResolveLinearVelocity(body2D);
        Fixed64 normalAngularVelocity2D =
            body2D == null
                ? Fixed64.Zero
                : ResolveAngularVelocity(body2D);

        bool normalResolved = TryGetNormalImpulse(
            pair,
            contact,
            body3D,
            body2D,
            normal,
            relative3D,
            relative2D,
            resolved3D,
            resolved2D,
            normalLinearVelocity3D,
            normalAngularVelocity3D,
            normalLinearVelocity2D,
            normalAngularVelocity2D,
            out ContactNormalImpulseResultMixed normalResult);
        if (!normalResolved
            || !CanApplyNormalVelocityDeltas(body3D, body2D, normalResult))
        {
            GravitasLogger.Channel.Write(
                DiagnosticLevel.Error,
                "Mixed contact response is outside the representable velocity domain.",
                nameof(CollisionResponseMixed));
            return false;
        }

        if (applyPositionCorrection && correctionInverseMass > Fixed64.Zero)
            ApplyPositionCorrection(body3D, body2D, normal, contact.Depth, correctionInverseMass);

        bool appliedImpulse = HasVelocityDelta(normalResult);
        if (appliedImpulse)
        {
            if (normalResult.HasRepresentableAppliedImpulse
                && normalResult.HasRepresentableNormalVelocity)
            {
                pair.Context.Diagnostics.EmitMixedResponseImpulse(
                    pair,
                    contact,
                    normal * normalResult.AppliedImpulseScalar,
                    normalResult.NormalVelocity,
                    iteration,
                    iterationLimit);
            }
            ApplyNormalVelocityDeltas(body3D, body2D, normalResult);
        }
        PhysicsMaterial material3D = contact.HasMaterialOverride
            ? contact.Material3D
            : pair.Collider3D.Material;
        PhysicsMaterial material2D = contact.HasMaterialOverride
            ? contact.Material2D
            : pair.Collider2D.Material;
        bool frictionResolved = TryApplyFrictionImpulse(
            pair,
            contact,
            body3D,
            body2D,
            normal,
            relative3D,
            relative2D,
            inverseMass3D,
            inverseMass2D,
            inverseMoment2D,
            material3D,
            material2D,
            normalResult,
            normalLinearVelocity3D,
            normalAngularVelocity3D,
            normalLinearVelocity2D,
            normalAngularVelocity2D,
            pair.Context.Settings.RestitutionVelocityThreshold,
            hasPlanarResponseCoupling,
            resolved3D,
            resolved2D,
            out bool frictionApplied);
        if (!frictionResolved)
        {
            GravitasLogger.Channel.Write(
                DiagnosticLevel.Error,
                "Mixed contact friction is outside the representable velocity domain.",
                nameof(CollisionResponseMixed));
        }

        return appliedImpulse | frictionApplied;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool HasPlanarResponseCoupling(CollisionPairMixed pair, MixedContact contact) =>
        ResolveNormal(pair, contact).ToVector2d() != Vector2d.Zero;

    private static void ApplyPositionCorrection(
        SolidBody? body3D,
        SolidBody2D? body2D,
        Vector3d normal,
        Fixed64 depth,
        Fixed64 effectiveInverseMass)
    {
        Fixed64 correctionDepth = depth - PenetrationSlop;
        if (correctionDepth <= Fixed64.Zero)
            return;

        Fixed64 correctionScalar = correctionDepth * PenetrationCorrectionPercent / effectiveInverseMass;
        Fixed64 constrainedInverseMass3D = GetConstrainedInverseMass(body3D, normal);
        if (constrainedInverseMass3D > Fixed64.Zero)
            body3D!.ApplyCollisionPositionCorrection(-normal * (correctionScalar * constrainedInverseMass3D));

        Fixed64 constrainedInverseMass2D = GetConstrainedPlanarInverseMass(body2D, normal);
        if (constrainedInverseMass2D <= Fixed64.Zero)
            return;

        Vector2d planarNormal = normal.ToVector2d();
        body2D!.ApplyCollisionPositionCorrection(planarNormal * (correctionScalar * constrainedInverseMass2D));
    }

    private static bool TryGetNormalImpulse(
        CollisionPairMixed pair,
        MixedContact contact,
        SolidBody? body3D,
        SolidBody2D? body2D,
        Vector3d normal,
        Vector3d relative3D,
        Vector2d relative2D,
        bool hasCompact3D,
        bool hasCompact2D,
        Vector3d linearVelocity3D,
        Vector3d angularVelocity3D,
        Vector2d linearVelocity2D,
        Fixed64 angularVelocity2D,
        out ContactNormalImpulseResultMixed result)
    {
        PhysicsMaterial material3D = contact.HasMaterialOverride
            ? contact.Material3D
            : pair.Collider3D.Material;
        PhysicsMaterial material2D = contact.HasMaterialOverride
            ? contact.Material2D
            : pair.Collider2D.Material;
        Fixed64 restitution =
            PhysicsMaterial.CombineRestitution(material3D, material2D);
        Fixed64 restitutionVelocityThreshold =
            pair.Context.Settings.RestitutionVelocityThreshold;
        if (hasCompact3D
            && hasCompact2D
            && ContactNormalImpulseMixed.CanUseCompactResponse(
                body3D,
                linearVelocity3D,
                angularVelocity3D,
                relative3D,
                body2D,
                linearVelocity2D,
                angularVelocity2D,
                relative2D,
                normal)
            && ContactNormalImpulseMixed.TryCalculateAccumulatedDelta(
                body3D,
                linearVelocity3D,
                angularVelocity3D,
                relative3D,
                body2D,
                linearVelocity2D,
                angularVelocity2D,
                relative2D,
                normal,
                restitution,
                restitutionVelocityThreshold,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out result))
        {
            return true;
        }

        GetExactLevers(
            pair,
            contact,
            body3D,
            body2D,
            out ExactLever3D exact3D,
            out ExactLever3D exact2D);
        return ContactNormalImpulseMixed.TryCalculateAccumulatedDeltaExact(
            body3D,
            linearVelocity3D,
            angularVelocity3D,
            exact3D,
            body2D,
            linearVelocity2D,
            angularVelocity2D,
            exact2D,
            normal,
            restitution,
            restitutionVelocityThreshold,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out result);
    }

    private static bool CanApplyNormalVelocityDeltas(
        SolidBody? body3D,
        SolidBody2D? body2D,
        ContactNormalImpulseResultMixed result) =>
        CanApplyVelocityDeltas(
            body3D,
            body2D,
            result.LinearVelocityDelta3D,
            result.AngularVelocityDelta3D,
            result.LinearVelocityDelta2D,
            result.AngularVelocityDelta2D);

    private static bool HasVelocityDelta(
        ContactNormalImpulseResultMixed result) =>
        result.LinearVelocityDelta3D != Vector3d.Zero
        || result.AngularVelocityDelta3D != Vector3d.Zero
        || result.LinearVelocityDelta2D != Vector2d.Zero
        || result.AngularVelocityDelta2D != Fixed64.Zero;

    private static void ApplyNormalVelocityDeltas(
        SolidBody? body3D,
        SolidBody2D? body2D,
        ContactNormalImpulseResultMixed result) =>
        ApplyVelocityDeltas(
            body3D,
            body2D,
            result.LinearVelocityDelta3D,
            result.AngularVelocityDelta3D,
            result.LinearVelocityDelta2D,
            result.AngularVelocityDelta2D);

    private static bool CanApplyVelocityDeltas(
        SolidBody? body3D,
        SolidBody2D? body2D,
        Vector3d linear3D,
        Vector3d angular3D,
        Vector2d linear2D,
        Fixed64 angular2D)
    {
        bool response3DFits = body3D?.CanApplyCollisionVelocityDeltas(
                linear3D,
                angular3D)
            ?? true;
        bool response2DFits = body2D?.CanApplyCollisionVelocityDeltas(
                linear2D,
                angular2D)
            ?? true;
        return response3DFits & response2DFits;
    }

    private static void ApplyVelocityDeltas(
        SolidBody? body3D,
        SolidBody2D? body2D,
        Vector3d linear3D,
        Vector3d angular3D,
        Vector2d linear2D,
        Fixed64 angular2D)
    {
        if (body3D != null)
        {
            body3D.ApplyCollisionLinearVelocityDelta(linear3D);
            body3D.ApplyCollisionAngularVelocityDelta(angular3D);
        }

        if (body2D != null)
        {
            body2D.ApplyCollisionLinearVelocityDelta(linear2D);
            body2D.ApplyCollisionAngularVelocityDelta(angular2D);
        }
    }

    private static bool TryApplyFrictionImpulse(
        CollisionPairMixed pair,
        MixedContact contact,
        SolidBody? body3D,
        SolidBody2D? body2D,
        Vector3d normal,
        Vector3d relative3D,
        Vector2d relative2D,
        Fixed64 inverseMass3D,
        Fixed64 inverseMass2D,
        Fixed64 inverseMoment2D,
        PhysicsMaterial material3D,
        PhysicsMaterial material2D,
        ContactNormalImpulseResultMixed normalResult,
        Vector3d normalLinearVelocity3D,
        Vector3d normalAngularVelocity3D,
        Vector2d normalLinearVelocity2D,
        Fixed64 normalAngularVelocity2D,
        Fixed64 restitutionVelocityThreshold,
        bool applyTo2D,
        bool hasCompact3D,
        bool hasCompact2D,
        out bool applied)
    {
        applied = false;
        Fixed64 normalImpulse = normalResult.HasRepresentableAppliedImpulse
            ? FixedMath.Max(
                Fixed64.Zero,
                normalResult.AppliedImpulseScalar)
            : Fixed64.Zero;
        if (normalResult.HasRepresentableAppliedImpulse
            && normalImpulse <= Fixed64.Zero)
            return true;

        PhysicsMaterial.CombineFriction(material3D, material2D, out Fixed64 staticFriction, out Fixed64 dynamicFriction);
        Fixed64 restitution =
            PhysicsMaterial.CombineRestitution(material3D, material2D);
        if (staticFriction <= Fixed64.Zero && dynamicFriction <= Fixed64.Zero)
            return true;

        if (!normalResult.HasRepresentableAppliedImpulse
            || !hasCompact3D
            || !hasCompact2D
            || !ContactNormalImpulseMixed.CanUseCompactResponse(
                body3D,
                body3D == null ? Vector3d.Zero : ResolveLinearVelocity(body3D),
                body3D == null ? Vector3d.Zero : ResolveAngularVelocity(body3D),
                relative3D,
                body2D,
                body2D == null ? Vector2d.Zero : ResolveLinearVelocity(body2D),
                body2D == null ? Fixed64.Zero : ResolveAngularVelocity(body2D),
                relative2D,
                normal))
        {
            return TryApplyFrictionImpulseExact(
                pair,
                contact,
                body3D,
                body2D,
                normal,
                normalLinearVelocity3D,
                normalAngularVelocity3D,
                normalLinearVelocity2D,
                normalAngularVelocity2D,
                restitutionVelocityThreshold,
                restitution,
                staticFriction,
                dynamicFriction,
                applyTo2D,
                out applied);
        }

        Vector3d relativeVelocity = ComputeRelativeVelocity(body3D, body2D, relative3D, relative2D);
        Vector3d tangentVelocity = relativeVelocity - normal * Vector3d.Dot(relativeVelocity, normal);
        if (tangentVelocity.MagnitudeSquared <= Fixed64.Epsilon)
            return true;

        Vector3d tangent = tangentVelocity.Normalized;
        Fixed64 denominator = GetConstrainedInverseMass(body3D, tangent)
            + ComputeAngularDenominator(body3D, relative3D, tangent);
        if (applyTo2D)
        {
            denominator += GetConstrainedPlanarInverseMass(body2D, tangent)
                + ComputePlanarAngularDenominator(relative2D, tangent.ToVector2d(), inverseMoment2D);
        }
        if (denominator <= Fixed64.Zero)
            return true;

        Fixed64 tangentVelocityMagnitude = Vector3d.Dot(relativeVelocity, tangent);
        if (!Fixed64.TryMultiplyDivide(
                -tangentVelocityMagnitude,
                Fixed64.One,
                denominator,
                out Fixed64 impulseScalar))
        {
            return TryApplyFrictionImpulseExact(
                pair,
                contact,
                body3D,
                body2D,
                normal,
                normalLinearVelocity3D,
                normalAngularVelocity3D,
                normalLinearVelocity2D,
                normalAngularVelocity2D,
                restitutionVelocityThreshold,
                restitution,
                staticFriction,
                dynamicFriction,
                applyTo2D,
                out applied);
        }
        if (!Fixed64.TryMultiplyDivide(
                normalImpulse,
                staticFriction,
                Fixed64.One,
                out Fixed64 staticLimit)
            || !Fixed64.TryMultiplyDivide(
                normalImpulse,
                dynamicFriction,
                Fixed64.One,
                out Fixed64 dynamicLimit))
        {
            return TryApplyFrictionImpulseExact(
                pair,
                contact,
                body3D,
                body2D,
                normal,
                normalLinearVelocity3D,
                normalAngularVelocity3D,
                normalLinearVelocity2D,
                normalAngularVelocity2D,
                restitutionVelocityThreshold,
                restitution,
                staticFriction,
                dynamicFriction,
                applyTo2D,
                out applied);
        }
        if (impulseScalar.Abs() > staticLimit)
            impulseScalar = FixedMath.Clamp(impulseScalar, -dynamicLimit, dynamicLimit);
        if (impulseScalar == Fixed64.Zero)
            return true;

        if (!TryApplyImpulse(
                body3D,
                body2D,
                tangent,
                relative3D,
                relative2D,
                inverseMass3D,
                inverseMass2D,
                inverseMoment2D,
                impulseScalar,
                applyTo2D))
        {
            return TryApplyFrictionImpulseExact(
                pair,
                contact,
                body3D,
                body2D,
                normal,
                normalLinearVelocity3D,
                normalAngularVelocity3D,
                normalLinearVelocity2D,
                normalAngularVelocity2D,
                restitutionVelocityThreshold,
                restitution,
                staticFriction,
                dynamicFriction,
                applyTo2D,
                out applied);
        }

        applied = true;
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryApplyFrictionImpulseExact(
        CollisionPairMixed pair,
        MixedContact contact,
        SolidBody? body3D,
        SolidBody2D? body2D,
        Vector3d normal,
        Vector3d normalLinearVelocity3D,
        Vector3d normalAngularVelocity3D,
        Vector2d normalLinearVelocity2D,
        Fixed64 normalAngularVelocity2D,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 restitution,
        Fixed64 staticFriction,
        Fixed64 dynamicFriction,
        bool applyTo2D,
        out bool applied)
    {
        applied = false;
        GetExactLevers(
            pair,
            contact,
            body3D,
            body2D,
            out ExactLever3D exact3D,
            out ExactLever3D exact2D);
        Vector3d tangent = SolverContact.CreateTangent(normal);
        Vector3d secondaryTangent =
            Vector3d.Cross(normal, tangent).Normalized;
        SolidBody2D? responseBody2D =
            applyTo2D ? body2D : null;
        Vector2d responseLinearVelocity2D =
            responseBody2D == null
                ? Vector2d.Zero
                : normalLinearVelocity2D;
        Fixed64 responseAngularVelocity2D =
            responseBody2D == null
                ? Fixed64.Zero
                : normalAngularVelocity2D;
        var normalConstraint = new ExactNormalConstraint3D(
            ExactContactLever3D.CreateResponseOperand(
                body3D,
                normalLinearVelocity3D,
                normalAngularVelocity3D,
                exact3D,
                -normal),
            ExactContactLever2D.CreateResponseOperand(
                responseBody2D,
                responseLinearVelocity2D,
                responseAngularVelocity2D,
                exact2D,
                normal),
            normal,
            restitution,
            restitutionVelocityThreshold,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One);
        Vector3d linearVelocity3D =
            body3D == null
                ? Vector3d.Zero
                : ResolveLinearVelocity(body3D);
        Vector3d angularVelocity3D =
            body3D == null
                ? Vector3d.Zero
                : ResolveAngularVelocity(body3D);
        Vector2d linearVelocity2D =
            responseBody2D == null
                ? Vector2d.Zero
                : ResolveLinearVelocity(responseBody2D);
        Fixed64 angularVelocity2D =
            responseBody2D == null
                ? Fixed64.Zero
                : ResolveAngularVelocity(responseBody2D);
        if (!ExactContactResponseKernel.TryGetCoulombDiskResponse(
                normalConstraint,
                ExactContactLever3D.CreateResponseOperand(
                    body3D,
                    linearVelocity3D,
                    angularVelocity3D,
                    exact3D,
                    -tangent),
                ExactContactLever2D.CreateResponseOperand(
                    responseBody2D,
                    linearVelocity2D,
                    angularVelocity2D,
                    exact2D,
                    tangent),
                tangent,
                ExactContactLever3D.CreateResponseOperand(
                    body3D,
                    linearVelocity3D,
                    angularVelocity3D,
                    exact3D,
                    -secondaryTangent),
                ExactContactLever2D.CreateResponseOperand(
                    responseBody2D,
                    linearVelocity2D,
                    angularVelocity2D,
                    exact2D,
                    secondaryTangent),
                secondaryTangent,
                staticFriction,
                dynamicFriction,
                out ExactCoulombResponse3D response))
        {
            return false;
        }

        Vector3d linear3D =
            response.FirstLinearVelocityDelta;
        Vector3d angular3D =
            response.FirstAngularVelocityDelta;
        Vector2d linear2D = ExactContactLever2D.ToPlanar(
            response.SecondLinearVelocityDelta);
        Fixed64 angular2D = ExactContactLever2D.ToPlanarAngular(
            response.SecondAngularVelocityDelta);
        if (!CanApplyVelocityDeltas(
                body3D,
                body2D,
                linear3D,
                angular3D,
                linear2D,
                angular2D))
        {
            return false;
        }

        if (!response.HasAppliedImpulse)
            return true;

        ApplyVelocityDeltas(
            body3D,
            body2D,
            linear3D,
            angular3D,
            linear2D,
            angular2D);
        applied = true;
        return true;
    }

    private static bool TryApplyImpulse(
        SolidBody? body3D,
        SolidBody2D? body2D,
        Vector3d axis,
        Vector3d relative3D,
        Vector2d relative2D,
        Fixed64 inverseMass3D,
        Fixed64 inverseMass2D,
        Fixed64 inverseMoment2D,
        Fixed64 impulseScalar,
        bool applyTo2D)
    {
        Vector3d linear3D = Vector3d.Zero;
        Vector3d angular3D = Vector3d.Zero;
        Vector2d linear2D = Vector2d.Zero;
        Fixed64 angular2D = Fixed64.Zero;
        bool linear3DResolved = body3D?.CanTranslate != true
            || ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                body3D.ProjectLinearMotion(-axis),
                impulseScalar,
                inverseMass3D,
                Fixed64.One,
                out linear3D);
        bool angular3DResolved = body3D?.CanRotate != true
            || ContactResponseArithmetic3D.TryScale(
                    Fixed3x3.TransformDirection(
                        body3D.GetConstrainedInverseInertiaTensor(),
                        Vector3d.Cross(relative3D, -axis)),
                    impulseScalar,
                    out angular3D);
        Vector2d planarAxis = axis.ToVector2d();
        bool linear2DResolved = !applyTo2D
            || body2D?.CanTranslate != true
            || ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
                body2D.ProjectLinearMotion(planarAxis),
                impulseScalar,
                inverseMass2D,
                Fixed64.One,
                out linear2D);
        bool angular2DResolved = !applyTo2D
            || body2D?.CanRotate != true
            || Fixed64.TryMultiplyDivide(
                    Vector2d.CrossProduct(relative2D, planarAxis),
                    impulseScalar,
                    inverseMoment2D,
                    Fixed64.One,
                    out angular2D);
        if (!(linear3DResolved
            & angular3DResolved
            & linear2DResolved
            & angular2DResolved)
            || !CanApplyVelocityDeltas(
                body3D,
                body2D,
                linear3D,
                angular3D,
                linear2D,
                angular2D))
        {
            return false;
        }

        ApplyVelocityDeltas(
            body3D,
            body2D,
            linear3D,
            angular3D,
            linear2D,
            angular2D);
        return true;
    }

    private static Vector3d ComputeRelativeVelocity(
        SolidBody? body3D,
        SolidBody2D? body2D,
        Vector3d relative3D,
        Vector2d relative2D)
    {
        Vector3d velocity3D = body3D == null
            ? Vector3d.Zero
            : ResolveLinearVelocity(body3D)
                + Vector3d.Cross(ResolveAngularVelocity(body3D), relative3D);
        Vector3d velocity2D = body2D == null
            ? Vector3d.Zero
            : (ResolveLinearVelocity(body2D)
                + AngularVelocityAtPoint(relative2D, ResolveAngularVelocity(body2D)))
                .ToVector3d(Fixed64.Zero);
        return velocity2D - velocity3D;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveLinearVelocity(SolidBody body) =>
        body.ProjectLinearMotion(
            body.IsKinematic
                ? body.SampleContinuousCollisionLinearVelocity(Fixed64.One)
                : body.LinearVelocity);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ResolveLinearVelocity(SolidBody2D body) =>
        body.ProjectLinearMotion(
            body.IsKinematic
                ? body.SampleContinuousCollisionLinearVelocity(Fixed64.One)
                : body.LinearVelocity);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveAngularVelocity(SolidBody body) =>
        body.ProjectAngularMotion(
            body.IsKinematic
                ? body.SampleContinuousCollisionAngularVelocity(Fixed64.One)
                : body.AngularVelocity);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ResolveAngularVelocity(SolidBody2D body) =>
        body.IsKinematic
            ? body.SampleContinuousCollisionAngularVelocity(Fixed64.One)
            : body.AngularVelocity;

    private static Fixed64 ComputeAngularDenominator(SolidBody? body3D, Vector3d relativeContactPoint, Vector3d axis)
    {
        if (body3D == null)
            return Fixed64.Zero;

        Vector3d angularVelocityDelta = body3D!.ApplyConstrainedInverseInertia(
            Vector3d.Cross(relativeContactPoint, axis));
        Vector3d angular = Vector3d.Cross(angularVelocityDelta, relativeContactPoint);
        Fixed64 denominator = Vector3d.Dot(angular, axis);
        return FixedMath.Max(denominator, Fixed64.Zero);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputePlanarAngularDenominator(
        Vector2d relativePoint,
        Vector2d axis,
        Fixed64 inverseMoment)
    {
        if (inverseMoment <= Fixed64.Zero || axis == Vector2d.Zero)
            return Fixed64.Zero;

        Fixed64 cross = Vector2d.CrossProduct(relativePoint, axis);
        return cross * cross * inverseMoment;
    }

    private static Vector3d ResolveNormal(CollisionPairMixed pair, MixedContact contact)
    {
        Vector3d fallback = ResolveFallbackDirection(pair, contact);
        if (contact.Normal3DTo2D.MagnitudeSquared > Fixed64.Epsilon)
        {
            Vector3d normal = contact.Normal3DTo2D.Normalized;
            return fallback.MagnitudeSquared > Fixed64.Epsilon && Vector3d.Dot(normal, fallback) < Fixed64.Zero
                ? -normal
                : normal;
        }

        return fallback == Vector3d.Zero ? Vector3d.Up : fallback;
    }

    private static Vector3d ResolveFallbackDirection(CollisionPairMixed pair, MixedContact contact)
    {
        LSCollider2D collider2D = pair.Collider2D;
        Vector3d embeddedCenter = new(
            collider2D.Center.X,
            collider2D.MixedSlabCenterY,
            collider2D.Center.Y);
        Vector3d centerDirection = embeddedCenter - pair.Collider3D.Center;
        if (centerDirection.MagnitudeSquared > Fixed64.Epsilon)
            return centerDirection.Normalized;

        if (contact.TryGetPoint2D(out Vector3d point2D)
            && contact.TryGetPoint3D(out Vector3d point3D))
        {
            Vector3d pointDirection = point2D - point3D;
            if (pointDirection.MagnitudeSquared > Fixed64.Epsilon)
                return pointDirection.Normalized;
        }

        return Vector3d.Zero;
    }

    private static void GetExactLevers(
        CollisionPairMixed pair,
        MixedContact contact,
        SolidBody? body3D,
        SolidBody2D? body2D,
        out ExactLever3D exact3D,
        out ExactLever3D exact2D)
    {
        ContactAnchor center3D = body3D?.GetCenterOfMassAnchor()
            ?? ContactAnchor.FromWorldPoint(pair.Collider3D.Center);
        exact3D = contact.Anchor3D.GetLeverFrom(center3D);
        exact2D = body2D == null
            ? contact.GetPlanarXZLeverFrom(
                pair.Collider2D.Center,
                Fixed64.Zero,
                Vector2d.Zero)
            : contact.GetPlanarXZLeverFrom(
                body2D.Position,
                body2D.Rotation,
                body2D.LocalCenterOfMassOffset);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetConstrainedInverseMass(SolidBody? body, Vector3d axis) =>
        body?.GetConstrainedInverseMass(axis) ?? Fixed64.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetConstrainedPlanarInverseMass(SolidBody2D? body, Vector3d axis)
    {
        if (body == null)
            return Fixed64.Zero;

        Vector2d planarAxis = axis.ToVector2d();
        if (planarAxis == Vector2d.Zero)
            return Fixed64.Zero;

        return body.GetConstrainedInverseMass(planarAxis) * planarAxis.MagnitudeSquared;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d AngularVelocityAtPoint(Vector2d relativePoint, Fixed64 angularVelocity) =>
        new(-angularVelocity * relativePoint.Y, angularVelocity * relativePoint.X);

}
