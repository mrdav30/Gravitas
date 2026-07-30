using Chronicler;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using SwiftCollections.Diagnostics;
using System;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedResponseTests
{
    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void Resolve_WithUnrepresentableMixedLeverArm_ShouldPreserveResponse(
        bool positiveFace)
    {
        var first = RunUnrepresentableMixedLeverResponse(positiveFace);
        var second = RunUnrepresentableMixedLeverResponse(positiveFace);

        first.Applied.Should().BeTrue();
        first.LinearVelocity3D.X.Should().BeLessThan(Fixed64.Two);
        first.LinearVelocity2D.X.Should().BeGreaterThan(Fixed64.Zero);
        first.AngularVelocity3D.Z.Should().BeGreaterThan(Fixed64.Zero);
        first.AngularVelocity2D.Should().BeLessThan(Fixed64.Zero);
        first.Should().Be(second);
    }

    [Fact]
    public void Resolve_WithUnrepresentablePlanarLeverArm_ShouldApplyFriction()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Collider.Material = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        body2D.Collider.Material = body3D.Collider.Material;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            new Vector3d(Fixed64.Two, Fixed64.Zero, (Fixed64)4));
        body2D.ApplyCollisionLinearVelocityDelta(
            Vector2d.Left * Fixed64.Two);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        var anchor2D = new ContactAnchor(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            new Vector3d(
                Fixed64.MinIncrement,
                Fixed64.Zero,
                Fixed64.One));
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            anchor2D,
            Vector3d.Right,
            Fixed64.Half);
        Fixed64 tangentialSpeed =
            (body2D.LinearVelocity.Y
                - body3D.Body.LinearVelocity.Z).Abs();
        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();

        (body2D.LinearVelocity.Y
            - body3D.Body.LinearVelocity.Z).Abs()
            .Should()
            .BeLessThan(tangentialSpeed);
    }

    [Fact]
    public void Resolve_WithExactVerticalContact_ShouldApplyFrictionOnlyTo3DParticipant()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        var material = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Up + (Vector3d.Forward * (Fixed64)4));
        body2D.Sleep();
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            new ContactAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                new Vector3d(
                    Fixed64.MinIncrement,
                    Fixed64.One,
                    Fixed64.Zero)),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Up,
            Fixed64.Half);
        Fixed64 tangentialSpeed = body3D.Body.LinearVelocity.Z.Abs();
        int errorCount = 0;
        DiagnosticLevel originalLevel = GravitasLogger.MinimumLevel;
        Action<DiagnosticLevel, string, string> originalHandler =
            GravitasLogger.LogHandler;
        try
        {
            GravitasLogger.MinimumLevel = DiagnosticLevel.Error;
            GravitasLogger.LogHandler = (level, _, _) =>
            {
                if (level == DiagnosticLevel.Error)
                    errorCount++;
            };
            CollisionResponseMixed.Resolve(
                    pair,
                    contact,
                    iteration: 0,
                    iterationLimit: 1,
                    applyPositionCorrection: false)
                .Should()
                .BeTrue();
        }
        finally
        {
            GravitasLogger.MinimumLevel = originalLevel;
            GravitasLogger.LogHandler = originalHandler;
        }

        errorCount.Should().Be(0);
        body3D.Body.LinearVelocity.Z.Abs().Should().BeLessThan(
            tangentialSpeed);
        body2D.IsSleeping.Should().BeTrue();
        body2D.LinearVelocity.Should().Be(Vector2d.Zero);
        body2D.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithUnrepresentableNormalImpulse_ShouldRetainMixedStaticFriction()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Diagnostics.Enable(
            eventCapacity: 4,
            drawCommandCapacity: 0);
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        var material = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        body3D.Body.Mass = Fixed64.MaxValue;
        body2D.Mass = Fixed64.MaxValue;
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes = BodyFreezeAxes2D.Rotation;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            new Vector3d(
                (Fixed64)6,
                Fixed64.Zero,
                Fixed64.FromFraction(1, 4)));
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Half);
        Fixed64 tangentialVelocity =
            body3D.Body.LinearVelocity.Z.Abs();
        context.Diagnostics.Clear();

        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();

        body3D.Body.LinearVelocity.X.Should().Be((Fixed64)3);
        body2D.LinearVelocity.X.Should().Be((Fixed64)3);
        (body3D.Body.LinearVelocity - new Vector3d(
                body2D.LinearVelocity.X,
                Fixed64.Zero,
                body2D.LinearVelocity.Y))
            .Z.Abs()
            .Should()
            .BeLessThan(tangentialVelocity);
        ReadOnlySpan<GravitasDiagnosticEvent> events =
            context.Diagnostics.Events;
        int responseImpulseCount = 0;
        for (int i = 0; i < events.Length; i++)
        {
            if (events[i].Kind
                == GravitasDiagnosticEventKind.MixedResponseImpulse)
            {
                responseImpulseCount++;
            }
        }
        responseImpulseCount.Should().Be(0);
    }

    [Fact]
    public void Resolve_WithUnrepresentableNormalVelocity_ShouldApplyWithoutProjectedDiagnostic()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Diagnostics.Enable(
            eventCapacity: 4,
            drawCommandCapacity: 0);
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        LSCollider2D collider2D =
            CreateBodylessCircle2D(context, Vector2d.Zero);
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        body3D.Body.ApplyCollisionAngularVelocityDelta(
            -Vector3d.Forward * Fixed64.Two);
        var pair = new CollisionPairMixed(
            body3D.Collider,
            collider2D);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(
                Vector3d.Up * Fixed64.MaxValue),
            ContactAnchor.FromWorldPoint(
                Vector3d.Up * Fixed64.MaxValue),
            Vector3d.Right,
            Fixed64.Zero);

        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();

        body3D.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        int responseImpulseCount = 0;
        foreach (GravitasDiagnosticEvent diagnostic
            in context.Diagnostics.Events)
        {
            if (diagnostic.Kind
                == GravitasDiagnosticEventKind.MixedResponseImpulse)
            {
                responseImpulseCount++;
            }
        }
        responseImpulseCount.Should().Be(0);
    }

    [Fact]
    public void Resolve_WhenExactMixedNormalDeltaWouldOverflowCurrentVelocity_ShouldRejectAtomically()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Environment.MaxSpeed = Fixed64.MaxValue;
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        var material = new PhysicsMaterial(
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        body3D.Body.Mass = Fixed64.MaxValue;
        body2D.Mass = Fixed64.Half;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right * Fixed64.MaxValue);
        body2D.ApplyCollisionLinearVelocityDelta(
            Vector2d.Right * (Fixed64.MaxValue - Fixed64.One));
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            new ContactAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                new Vector3d(
                    Fixed64.MinIncrement,
                    Fixed64.Zero,
                    Fixed64.Zero)),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Zero);
        Vector3d linear3D = body3D.Body.LinearVelocity;
        Vector3d angular3D = body3D.Body.AngularVelocity;
        Vector2d linear2D = body2D.LinearVelocity;
        Fixed64 angular2D = body2D.AngularVelocity;
        int errorCount = 0;
        string? error = null;
        DiagnosticLevel originalLevel = GravitasLogger.MinimumLevel;
        System.Action<DiagnosticLevel, string, string> originalHandler =
            GravitasLogger.LogHandler;
        bool resolved;
        try
        {
            GravitasLogger.MinimumLevel = DiagnosticLevel.Error;
            GravitasLogger.LogHandler = (level, message, _) =>
            {
                if (level != DiagnosticLevel.Error)
                    return;

                errorCount++;
                error = message;
            };
            resolved = CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false);
        }
        finally
        {
            GravitasLogger.MinimumLevel = originalLevel;
            GravitasLogger.LogHandler = originalHandler;
        }

        resolved.Should().BeFalse();
        body3D.Body.LinearVelocity.Should().Be(linear3D);
        body3D.Body.AngularVelocity.Should().Be(angular3D);
        body2D.LinearVelocity.Should().Be(linear2D);
        body2D.AngularVelocity.Should().Be(angular2D);
        errorCount.Should().Be(1);
        error.Should().Be(
            "Mixed contact response is outside the representable velocity domain.");
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void Resolve_WhenMixedBounceDeltaIsUnrepresentable_ShouldRejectBeforeMutation(
        bool diagnosticsEnabled)
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Environment.MaxSpeed = Fixed64.MaxValue;
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D =
            CreateCircle2D(context, Vector2d.Zero);
        PhysicsMaterial bouncy =
            PhysicsMaterialTestHelper.WithRestitution(Fixed64.One);
        body3D.Collider.Material = bouncy;
        body2D.Collider.Material = bouncy;
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes = BodyFreezeAxes2D.All;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right * Fixed64.MaxValue);
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Zero);
        Vector3d velocity = body3D.Body.LinearVelocity;
        int errorCount = 0;
        DiagnosticLevel originalLevel = GravitasLogger.MinimumLevel;
        Action<DiagnosticLevel, string, string> originalHandler =
            GravitasLogger.LogHandler;
        bool resolved;
        try
        {
            GravitasLogger.MinimumLevel = diagnosticsEnabled
                ? DiagnosticLevel.Error
                : DiagnosticLevel.None;
            GravitasLogger.LogHandler = (level, _, _) =>
            {
                if (level == DiagnosticLevel.Error)
                    errorCount++;
            };
            resolved = CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false);
        }
        finally
        {
            GravitasLogger.MinimumLevel = originalLevel;
            GravitasLogger.LogHandler = originalHandler;
        }

        resolved.Should().BeFalse();
        body3D.Body.LinearVelocity.Should().Be(velocity);
        body3D.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        body2D.LinearVelocity.Should().Be(Vector2d.Zero);
        body2D.AngularVelocity.Should().Be(Fixed64.Zero);
        errorCount.Should().Be(diagnosticsEnabled ? 1 : 0);
    }

    [Fact]
    public void Resolve_WhenMixedTangentMassIsNearSingular_ShouldUseExactFriction()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D =
            CreateCircle2D(context, Vector2d.Zero);
        var material = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        body3D.Body.Mass = Fixed64.MaxValue;
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes = BodyFreezeAxes2D.All;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            (Vector3d.Right * Fixed64.FromFraction(1, 4))
            + (Vector3d.Forward * Fixed64.Two));
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Zero);
        Fixed64 tangentialSpeed =
            body3D.Body.LinearVelocity.Z.Abs();

        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();

        body3D.Body.LinearVelocity.Z.Abs().Should().BeLessThan(
            tangentialSpeed);
    }

    [Fact]
    public void Resolve_WhenMixedFrictionLimitOverflowsCompactMath_ShouldUseExactResponse()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D =
            CreateCircle2D(context, Vector2d.Zero);
        var material = new PhysicsMaterial(
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes = BodyFreezeAxes2D.All;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            (Vector3d.Right * Fixed64.Two)
            + Vector3d.Forward);
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Zero);
        Fixed64 tangentialSpeed =
            body3D.Body.LinearVelocity.Z.Abs();

        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();

        body3D.Body.LinearVelocity.X.Should().BeLessThan(Fixed64.Two);
        body3D.Body.LinearVelocity.Z.Abs().Should().BeLessThan(
            tangentialSpeed);
    }

    [Fact]
    public void Resolve_WithMinValueTangentImpulse_ShouldUseExactDiskAndReplaySymmetrically()
    {
        var positive = RunMinValueTangentImpulse(positive: true);
        var repeatedPositive = RunMinValueTangentImpulse(positive: true);
        var negative = RunMinValueTangentImpulse(positive: false);

        positive.Should().Be(repeatedPositive);
        positive.LinearVelocity3D.Z.Should().Be(
            Fixed64.One - Fixed64.MinIncrement * Fixed64.Two);
        negative.LinearVelocity3D.Z.Should().Be(
            -Fixed64.One + Fixed64.MinIncrement * Fixed64.Two);
        positive.LinearVelocity3D.Z.Should().Be(
            -negative.LinearVelocity3D.Z);
        positive.LinearVelocity2D.Should().Be(Vector2d.Zero);
        negative.LinearVelocity2D.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void Resolve_WhenMixedTangentEffectiveMassSumOverflows_ShouldUseExactDisk()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var collider3D = new UnsupportedTestCollider3D
        {
            InertiaTensor = new Fixed3x3(
                Fixed64.One, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.One, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.One),
            MassPropertyWeight = ExactMassWeight.One
        };
        var body3D = new SolidBody(
            new TestMatterAgent(
                context,
                new FixedTransform(
                    Vector3d.Zero,
                    FixedQuaternion.Identity,
                    Vector3d.One)),
            collider3D)
        {
            Mass = Fixed64.MinIncrement,
            FreezeAxes =
                BodyFreezeAxes3D.PositionX
                | BodyFreezeAxes3D.PositionY
        };
        body3D.Initialize(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            BodyMotionType.Dynamic);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body2D.Mass = Fixed64.MinIncrement;
        body2D.FreezeAxes =
            BodyFreezeAxes2D.PositionX
            | BodyFreezeAxes2D.Rotation;
        var material = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        collider3D.Material = material;
        body2D.Collider.Material = material;
        body3D.ApplyCollisionLinearVelocityDelta(Vector3d.Forward);
        body3D.ApplyCollisionAngularVelocityDelta(
            -Vector3d.Forward * Fixed64.Two);
        body2D.ApplyCollisionLinearVelocityDelta(Vector2d.Backward);
        var pair = new CollisionPairMixed(collider3D, body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Up),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Zero);
        Fixed64 initialRelativeSpeed =
            (body2D.LinearVelocity.Y - body3D.LinearVelocity.Z).Abs();

        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();

        (body2D.LinearVelocity.Y - body3D.LinearVelocity.Z).Abs()
            .Should()
            .BeLessThan(initialRelativeSpeed);
        body3D.LinearVelocity.Z.Should().BeGreaterThanOrEqualTo(
            Fixed64.Zero);
        body2D.LinearVelocity.Y.Should().BeLessThanOrEqualTo(
            Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithSubprecisionPlanarAngularMass_ShouldUseExactDisk()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Environment.MinSpeed = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        body3D.Body.Mass = Fixed64.MaxValue;
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        SolidBody2D body2D = CreateCircle2D(
            context,
            Vector2d.Zero,
            radius: Fixed64.Two);
        body2D.Mass = Fixed64.MinIncrement;
        body2D.FreezeAxes =
            BodyFreezeAxes2D.PositionX
            | BodyFreezeAxes2D.PositionY;
        var material = new PhysicsMaterial(
            (Fixed64)4,
            (Fixed64)4,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right
            + Vector3d.Forward * (Fixed64.MinIncrement * (Fixed64)3));
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(
                Vector3d.Right * Fixed64.MinIncrement),
            Vector3d.Right,
            Fixed64.Zero);

        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();

        body3D.Body.LinearVelocity.Z.Should().Be(
            Fixed64.MinIncrement);
        body2D.AngularVelocity.Should().Be(
            Fixed64.FromFraction(3, 5));
    }

    [Fact]
    public void Resolve_WithSubprecisionCompactImpulseComponent_ShouldUseExactDisk()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Environment.MinSpeed = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        body3D.Body.Mass = Fixed64.MinIncrement;
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body2D.FreezeAxes = BodyFreezeAxes2D.All;
        var material = new PhysicsMaterial(
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Up
            + Vector3d.Forward
            + Vector3d.Right * Fixed64.MinIncrement);
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Up,
            Fixed64.Zero);

        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();

        body3D.Body.LinearVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void Resolve_WithNearVerticalTangent_ShouldRetainPlanarMassInExactFriction()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(
            context,
            Vector2d.Zero);
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes = BodyFreezeAxes2D.Rotation;
        var material = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        Fixed64 verticalSpeed = (Fixed64)128;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right
            + Vector3d.Up * verticalSpeed
            + Vector3d.Forward * Fixed64.FromFraction(1, 64));
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Zero);

        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();

        body3D.Body.LinearVelocity.Y.Should().BeLessThan(
            verticalSpeed);
        body2D.LinearVelocity.Y.Should().NotBe(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WhenPlanarTangentMassProductOverflows_ShouldUseExactDisk()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(
            context,
            Vector2d.Zero);
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.Mass = Fixed64.MinIncrement;
        body2D.FreezeAxes =
            BodyFreezeAxes2D.PositionX
            | BodyFreezeAxes2D.Rotation;
        var material = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        Fixed64 tangentialSpeed = Fixed64.FromRaw(
            Fixed64.One.m_rawValue + 1L);
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right
            + Vector3d.Forward * tangentialSpeed);
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Zero);

        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();

        body3D.Body.LinearVelocity.Z.Should().BeLessThan(
            tangentialSpeed);
        body2D.LinearVelocity.Y.Should().NotBe(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithVerticalAndPlanarTangent_ShouldApplyCompactFriction()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(
            context,
            Vector2d.Zero);
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes = BodyFreezeAxes2D.Rotation;
        var material = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right + Vector3d.Up + Vector3d.Forward);
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Zero);

        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();

        body3D.Body.LinearVelocity.Y.Should().BeLessThan(
            Fixed64.One);
        body3D.Body.LinearVelocity.Z.Should().BeLessThan(
            Fixed64.One);
        body2D.LinearVelocity.Y.Should().NotBe(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithMultiAxisTangentBeyondScalarDomain_ShouldUseExactDisk()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Environment.MinSpeed = Fixed64.Zero;
        context.Environment.MaxSpeed = Fixed64.MaxValue;
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(
            context,
            Vector2d.Zero);
        body3D.Body.Mass = Fixed64.MinIncrement;
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes = BodyFreezeAxes2D.All;
        var material = new PhysicsMaterial(
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right * Fixed64.Two
            + Vector3d.Up * Fixed64.MaxValue
            + Vector3d.Forward * Fixed64.MaxValue);
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Zero);

        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();
        body3D.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        body2D.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void Resolve_WithTangentialMotionBelowDeadzone_ShouldSkipFriction()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(
            context,
            Vector2d.Zero);
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes = BodyFreezeAxes2D.All;
        var material = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        Fixed64 tangentialSpeed =
            Fixed64.FromFraction(1, 8192);
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right
            + Vector3d.Forward * tangentialSpeed);
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Zero);

        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();

        body3D.Body.LinearVelocity.Should().Be(
            Vector3d.Forward * tangentialSpeed);
        body2D.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void Resolve_WhenExactMixedFrictionWouldOverflowCurrentVelocity_ShouldSkipFriction()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Environment.MaxSpeed = Fixed64.MaxValue;
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D =
            CreateCircle2D(context, Vector2d.Zero);
        var material = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        body2D.FreezeAxes = BodyFreezeAxes2D.All;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Up + (Vector3d.Forward * Fixed64.MaxValue));
        body3D.Body.ApplyCollisionAngularVelocityDelta(
            Vector3d.Up * Fixed64.MaxValue);
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Right * Fixed64.Two),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Up,
            Fixed64.Zero);
        int errorCount = 0;
        DiagnosticLevel originalLevel = GravitasLogger.MinimumLevel;
        Action<DiagnosticLevel, string, string> originalHandler =
            GravitasLogger.LogHandler;
        bool resolved;
        try
        {
            GravitasLogger.MinimumLevel = DiagnosticLevel.Error;
            GravitasLogger.LogHandler = (level, _, _) =>
            {
                if (level == DiagnosticLevel.Error)
                    errorCount++;
            };
            resolved = CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false);
        }
        finally
        {
            GravitasLogger.MinimumLevel = originalLevel;
            GravitasLogger.LogHandler = originalHandler;
        }

        resolved.Should().BeTrue();
        body3D.Body.LinearVelocity.Y.Should().BeLessThan(Fixed64.One);
        body3D.Body.LinearVelocity.Z.Should().Be(Fixed64.MaxValue);
        errorCount.Should().Be(1);
    }

    [Fact]
    public void Resolve_WithKinematicTangentAndNoResponseMobility_ShouldKeepNormalResponse()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        body3D.Body.FreezeAxes =
            BodyFreezeAxes3D.PositionY
            | BodyFreezeAxes3D.PositionZ
            | BodyFreezeAxes3D.Rotation;
        SolidBody2D body2D = CreatePreparedKinematicCircle2D(
            context,
            new Vector2d(-Fixed64.One, Fixed64.One));
        var material = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Zero);

        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();

        body3D.Body.LinearVelocity.X.Should().BeLessThan(
            Fixed64.Zero);
        body3D.Body.LinearVelocity.Z.Should().Be(Fixed64.Zero);
        body3D.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void Resolve_WhenExactMixedFrictionDeltaIsUnrepresentable_ShouldKeepNormalResponse()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var collider3D = new UnsupportedTestCollider3D
        {
            InertiaTensor = new Fixed3x3(
                Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.MinIncrement, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.MinIncrement)
        };
        var agent3D = new TestMatterAgent(
            context,
            new FixedTransform(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.One));
        var body3D = new SolidBody(agent3D, collider3D)
        {
            Mass = Fixed64.One,
            FreezeAxes =
                BodyFreezeAxes3D.PositionY
                | BodyFreezeAxes3D.PositionZ
        };
        body3D.Initialize(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            BodyMotionType.Dynamic);
        SolidBody2D body2D = CreatePreparedKinematicCircle2D(
            context,
            new Vector2d(
                -Fixed64.MaxValue,
                Fixed64.MaxValue));
        var material = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        collider3D.Material = material;
        body2D.Collider.Material = material;
        var pair = new CollisionPairMixed(
            collider3D,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(
                Vector3d.Right
                    * (Fixed64.MinIncrement * (Fixed64)3)),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Zero);
        int errorCount = 0;
        DiagnosticLevel originalLevel = GravitasLogger.MinimumLevel;
        Action<DiagnosticLevel, string, string> originalHandler =
            GravitasLogger.LogHandler;
        bool resolved;
        try
        {
            GravitasLogger.MinimumLevel = DiagnosticLevel.Error;
            GravitasLogger.LogHandler = (level, _, _) =>
            {
                if (level == DiagnosticLevel.Error)
                    errorCount++;
            };
            resolved = CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false);
        }
        finally
        {
            GravitasLogger.MinimumLevel = originalLevel;
            GravitasLogger.LogHandler = originalHandler;
        }

        resolved.Should().BeTrue();
        body3D.LinearVelocity.X.Should().BeLessThan(Fixed64.Zero);
        body3D.AngularVelocity.Should().Be(Vector3d.Zero);
        errorCount.Should().Be(1);
    }

    [Fact]
    public void Resolve_WhenCompactMixedFrictionDeltaIsUnrepresentable_ShouldKeepNormalResponse()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var desiredInverseInertia = new Fixed3x3(
            Fixed64.One, Fixed64.Two, Fixed64.Zero,
            Fixed64.Two, Fixed64.Epsilon * Fixed64.Two, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One);
        Fixed3x3.Invert(
                desiredInverseInertia,
                out Fixed3x3? inertiaTensor)
            .Should()
            .BeTrue();
        var collider3D = new UnsupportedTestCollider3D
        {
            InertiaTensor = inertiaTensor!.Value,
            MassPropertyWeight = ExactMassWeight.One
        };
        var agent3D = new TestMatterAgent(
            context,
            new FixedTransform(
                Vector3d.Left,
                FixedQuaternion.Identity,
                Vector3d.One));
        var body3D = new SolidBody(agent3D, collider3D)
        {
            Mass = Fixed64.One,
            FreezeAxes =
                BodyFreezeAxes3D.PositionY
                | BodyFreezeAxes3D.PositionZ
        };
        body3D.Initialize(
            Vector3d.Left,
            FixedQuaternion.Identity,
            BodyMotionType.Dynamic);
        SolidBody2D body2D = CreatePreparedKinematicCircle2D(
            context,
            new Vector2d(
                -Fixed64.One,
                (Fixed64)255));
        var material = new PhysicsMaterial(
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.Zero);
        collider3D.Material = material;
        body2D.Collider.Material = material;
        var pair = new CollisionPairMixed(
            collider3D,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Zero);
        int errorCount = 0;
        DiagnosticLevel originalLevel = GravitasLogger.MinimumLevel;
        Action<DiagnosticLevel, string, string> originalHandler =
            GravitasLogger.LogHandler;
        bool resolved;
        try
        {
            GravitasLogger.MinimumLevel = DiagnosticLevel.Error;
            GravitasLogger.LogHandler = (level, _, _) =>
            {
                if (level == DiagnosticLevel.Error)
                    errorCount++;
            };
            resolved = CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false);
        }
        finally
        {
            GravitasLogger.MinimumLevel = originalLevel;
            GravitasLogger.LogHandler = originalHandler;
        }

        resolved.Should().BeTrue();
        body3D.LinearVelocity.X.Should().BeLessThan(Fixed64.Zero);
        body3D.AngularVelocity.Should().Be(Vector3d.Zero);
        errorCount.Should().Be(1);
    }

    [Fact]
    public void Resolve_WithExactMixedLeverAndNoTangentialMotion_ShouldApplyOnlyNormalResponse()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D =
            CreateCircle2D(context, Vector2d.Zero);
        var material = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        body3D.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes = BodyFreezeAxes2D.Rotation;
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right);
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            new ContactAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                new Vector3d(
                    Fixed64.MinIncrement,
                    Fixed64.Zero,
                    Fixed64.Zero)),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Zero);

        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();

        body3D.Body.LinearVelocity.X.Should().BeLessThan(Fixed64.One);
        body3D.Body.LinearVelocity.Z.Should().Be(Fixed64.Zero);
        body2D.LinearVelocity.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Resolve_WithExactMixedFrictionFallback_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(
            context,
            Vector2d.Zero);
        var material = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            new ContactAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                new Vector3d(
                    Fixed64.MinIncrement,
                    Fixed64.One,
                    Fixed64.Zero)),
            ContactAnchor.FromWorldPoint(Vector3d.Forward),
            Vector3d.Right,
            Fixed64.Zero);
        Vector3d velocity3D =
            new(Fixed64.Two, Fixed64.Zero, (Fixed64)4);
        Vector2d velocity2D =
            Vector2d.Left * Fixed64.Two;

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () =>
            {
                body3D.Body.ApplyCollisionLinearVelocityDelta(
                    velocity3D - body3D.Body.LinearVelocity);
                body3D.Body.ApplyCollisionAngularVelocityDelta(
                    -body3D.Body.AngularVelocity);
                body2D.ApplyCollisionLinearVelocityDelta(
                    velocity2D - body2D.LinearVelocity);
                body2D.ApplyCollisionAngularVelocityDelta(
                    -body2D.AngularVelocity);
                _ = CollisionResponseMixed.Resolve(
                    pair,
                    contact,
                    iteration: 0,
                    iterationLimit: 1,
                    applyPositionCorrection: false);
            },
            warmupIterations: 4,
            stabilizationIterations: 2,
            measurementIterations: 4);

        allocatedBytes.Should().Be(0);
    }

    [Fact]
    public void Simulate_WithOffCenterVerticalContact_ShouldApplyFrictionOnlyTo3DParticipant()
    {
        PhysicsMaterial frictional = new((Fixed64)2, (Fixed64)2, Fixed64.Zero);
        var frictionlessResult = RunOffCenterVerticalFrictionScenario(PhysicsMaterial.Frictionless);
        var frictionalResult = RunOffCenterVerticalFrictionScenario(frictional);

        frictionalResult.NormalImpulsePlanar.Should().Be(Vector2d.Zero);
        frictionalResult.ContactPoint2D.X.Should().NotBe(Fixed64.Zero);
        frictionalResult.Sleeping2D.Should().BeTrue();
        frictionalResult.LinearVelocity2D.Should().Be(Vector2d.Zero);
        frictionalResult.AngularVelocity2D.Should().Be(Fixed64.Zero);
        frictionalResult.VerticalVelocity3D.Should().BeGreaterThan(-Fixed64.One);
        frictionalResult.PlanarSpeed3D.Should().BeLessThan(frictionlessResult.PlanarSpeed3D);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void Simulate_WithVerticalOnlyContact_ShouldKeepWakeStateIndependentOfUnrelatedResponsePair(bool sleep3D)
    {
        (bool Sleeping3D, bool Sleeping2D) isolated = RunVerticalWakeScenario(includeUnrelatedPair: false, sleep3D);
        (bool Sleeping3D, bool Sleeping2D) withUnrelatedPair = RunVerticalWakeScenario(includeUnrelatedPair: true, sleep3D);

        withUnrelatedPair.Should().Be(isolated);
        isolated.Sleeping3D.Should().Be(sleep3D);
        isolated.Sleeping2D.Should().Be(!sleep3D);
    }

    [Fact]
    public void Simulate_WithMultipleVerticalOnlyMixedContacts_ShouldNotWakeOrConnectSleeping2DParticipants()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> left3D = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-3), Fixed64.FromFraction(3, 4), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> right3D = CreateSphere3D(
            context,
            new Vector3d((Fixed64)3, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        SolidBody2D left2D = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        SolidBody2D right2D = CreateCircle2D(context, new Vector2d((Fixed64)3, Fixed64.Zero));
        PhysicsMaterial frictional = new((Fixed64)2, (Fixed64)2, Fixed64.Zero);
        left3D.Collider.Material = frictional;
        right3D.Collider.Material = frictional;
        left2D.Collider.Material = frictional;
        right2D.Collider.Material = frictional;
        Step(context);
        context.MixedCollisions.ActivePairCount.Should().Be(2);
        left3D.Body.SetPosition(new Vector3d((Fixed64)(-3), Fixed64.FromFraction(3, 4), Fixed64.Zero));
        right3D.Body.SetPosition(new Vector3d((Fixed64)3, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        left2D.SetPosition(new Vector2d((Fixed64)(-3), Fixed64.Zero));
        right2D.SetPosition(new Vector2d((Fixed64)3, Fixed64.Zero));
        left2D.Sleep();
        right2D.Sleep();
        left3D.Body.AddLinearImpulse(new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.Zero));
        right3D.Body.AddLinearImpulse(new Vector3d(-Fixed64.One, -Fixed64.One, Fixed64.Zero));

        Step(context);

        left2D.IsSleeping.Should().BeTrue();
        right2D.IsSleeping.Should().BeTrue();
        left2D.LinearVelocity.Should().Be(Vector2d.Zero);
        right2D.LinearVelocity.Should().Be(Vector2d.Zero);
        left2D.AngularVelocity.Should().Be(Fixed64.Zero);
        right2D.AngularVelocity.Should().Be(Fixed64.Zero);
        left3D.Body.LinearVelocity.Y.Should().BeGreaterThan(-Fixed64.One);
        right3D.Body.LinearVelocity.Y.Should().BeGreaterThan(-Fixed64.One);
    }

    private static (bool Sleeping3D, bool Sleeping2D) RunVerticalWakeScenario(
        bool includeUnrelatedPair,
        bool sleep3D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        if (includeUnrelatedPair)
        {
            ScenarioBody<LSSphereCollider> unrelated3D = CreateSphere3D(
                context,
                new Vector3d((Fixed64)5, Fixed64.FromFraction(3, 4), Fixed64.Zero));
            _ = CreateCircle2D(context, new Vector2d((Fixed64)5, Fixed64.Zero));
            unrelated3D.Body.AddLinearImpulse(-Vector3d.Up);
        }

        Step(context);
        context.MixedCollisions.ActivePairCount.Should().Be(includeUnrelatedPair ? 2 : 1);
        body3D.Body.SetPosition(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        body2D.SetPosition(Vector2d.Zero);
        if (sleep3D)
        {
            body3D.Body.Sleep();
            body2D.AddLinearImpulse(Vector2d.Right);
        }
        else
        {
            body2D.Sleep();
            body3D.Body.AddLinearImpulse(new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.Zero));
        }

        Step(context);

        return (body3D.Body.IsSleeping, body2D.IsSleeping);
    }

    private static (
        Fixed64 PlanarSpeed3D,
        Fixed64 VerticalVelocity3D,
        bool Sleeping2D,
        Vector2d LinearVelocity2D,
        Fixed64 AngularVelocity2D,
        Vector2d NormalImpulsePlanar,
        Vector2d ContactPoint2D) RunOffCenterVerticalFrictionScenario(PhysicsMaterial material)
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Diagnostics.Enable(eventCapacity: 16, drawCommandCapacity: 0);
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Half, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        SolidBody2D body2D = CreateBox2DTask7(context, Vector2d.Zero, new Vector2d((Fixed64)4, (Fixed64)4));
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        Step(context);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
        context.Diagnostics.Clear();
        body3D.Body.SetPosition(new Vector3d(Fixed64.Half, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        body2D.SetPosition(Vector2d.Zero);
        body2D.Sleep();
        body3D.Body.AddLinearImpulse(new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.One));

        Step(context);

        GravitasMixedResponseImpulseDiagnosticView impulse = FindFirstMixedImpulse(context);
        return (
            body3D.Body.LinearVelocity.Z.Abs(),
            body3D.Body.LinearVelocity.Y,
            body2D.IsSleeping,
            body2D.LinearVelocity,
            body2D.AngularVelocity,
            impulse.Impulse.ToVector2d(),
            impulse.Point2D.ToVector2d());
    }

    private static (
        bool Applied,
        Vector3d LinearVelocity3D,
        Vector3d AngularVelocity3D,
        Vector2d LinearVelocity2D,
        Fixed64 AngularVelocity2D,
        ChronicleHash ReplayHash) RunUnrepresentableMixedLeverResponse(
            bool positiveFace)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(
            context,
            Vector2d.Zero);
        body3D.Collider.Material = PhysicsMaterial.Frictionless;
        body2D.Collider.Material = PhysicsMaterial.Frictionless;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right * Fixed64.Two);
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        Fixed64 face = positiveFace
            ? Fixed64.MaxValue
            : Fixed64.MinValue;
        Fixed64 outward = positiveFace
            ? Fixed64.MinIncrement
            : -Fixed64.MinIncrement;
        var contact = new MixedContact(
            new ContactAnchor(
                new Vector3d(
                    face,
                    Fixed64.Zero,
                    Fixed64.Zero),
                new Vector3d(
                    outward,
                    Fixed64.One,
                    Fixed64.Zero)),
            ContactAnchor.FromWorldPoint(Vector3d.Forward),
            Vector3d.Right,
            Fixed64.Half);

        bool applied = CollisionResponseMixed.Resolve(
            pair,
            contact,
            iteration: 0,
            iterationLimit: 1,
            applyPositionCorrection: false);

        return (
            applied,
            body3D.Body.LinearVelocity,
            body3D.Body.AngularVelocity,
            body2D.LinearVelocity,
            body2D.AngularVelocity,
            context.ComputeReplayHash(
                GravitasReplayHashMode.AuthoritativeWithSolverCaches));
    }

    private static (
        Vector3d LinearVelocity3D,
        Vector3d AngularVelocity3D,
        Vector2d LinearVelocity2D,
        Fixed64 AngularVelocity2D,
        ChronicleHash ReplayHash) RunMinValueTangentImpulse(bool positive)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D =
            CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Body.Mass = Fixed64.MaxValue;
        body3D.Body.FreezeAxes =
            BodyFreezeAxes3D.PositionX
            | BodyFreezeAxes3D.PositionY
            | BodyFreezeAxes3D.Rotation;
        body2D.FreezeAxes =
            BodyFreezeAxes2D.PositionY
            | BodyFreezeAxes2D.Rotation;
        var material = new PhysicsMaterial(
            Fixed64.MaxValue,
            Fixed64.One,
            Fixed64.Zero);
        body3D.Collider.Material = material;
        body2D.Collider.Material = material;
        body3D.Body.ApplyCollisionLinearVelocityDelta(
            (positive ? Vector3d.Forward : -Vector3d.Forward)
                * Fixed64.One);
        body2D.ApplyCollisionLinearVelocityDelta(Vector2d.Left);
        var pair = new CollisionPairMixed(
            body3D.Collider,
            body2D.Collider);
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right,
            Fixed64.Zero);

        CollisionResponseMixed.Resolve(
                pair,
                contact,
                iteration: 0,
                iterationLimit: 1,
                applyPositionCorrection: false)
            .Should()
            .BeTrue();

        return (
            body3D.Body.LinearVelocity,
            body3D.Body.AngularVelocity,
            body2D.LinearVelocity,
            body2D.AngularVelocity,
            context.ComputeReplayHash(
                GravitasReplayHashMode.AuthoritativeWithSolverCaches));
    }

    private static SolidBody2D CreateBox2DTask7(
        GravitasWorldContext context,
        Vector2d position,
        Vector2d size)
    {
        var collider = new LSAABBoxCollider2D(size);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(
                new Vector3d(position.X, Fixed64.Zero, position.Y),
                FixedQuaternion.Identity,
                Vector3d.One));
        var body = new SolidBody2D(agent, collider) { Mass = Fixed64.One };
        collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        body.Initialize(position, motionType: BodyMotionType.Dynamic);
        return body;
    }

    private static SolidBody2D CreatePreparedKinematicCircle2D(
        GravitasWorldContext context,
        Vector2d frameVelocity)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var body = new SolidBody2D(
            new TestMatterAgent(context, transform),
            collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(
            Vector2d.Zero,
            motionType: BodyMotionType.Kinematic);
        transform.LocalPosition =
            (frameVelocity * context.DeltaTime)
            .ToVector3d(Fixed64.Zero);
        context.AdvanceLateSimulateToken();
        body.EnsureContinuousCollisionFramePrepared(
            context.LateSimulateToken);
        return body;
    }
}
