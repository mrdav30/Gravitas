using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using SwiftCollections.Diagnostics;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CollisionResponseExactLeverTests
{
    private static readonly Fixed64 StandardSpeed = Fixed64.Two;

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void CalculateImpulse_WithUnrepresentableParallelLeverComponent_ShouldPreserveResponse(
        bool positiveFace)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Right);
        left.Collider.Material = PhysicsMaterial.Frictionless;
        right.Collider.Material = PhysicsMaterial.Frictionless;
        Push(left.Body, Vector3d.Right * StandardSpeed);
        Push(right.Body, Vector3d.Left * StandardSpeed);
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        Fixed64 face = positiveFace
            ? Fixed64.MaxValue
            : Fixed64.MinValue;
        Fixed64 outward = positiveFace
            ? Fixed64.MinIncrement
            : -Fixed64.MinIncrement;
        pair.Manifold.SetContact(
            new ContactAnchor(
                new Vector3d(face, Fixed64.Zero, Fixed64.Zero),
                new Vector3d(
                    outward,
                    Fixed64.One,
                    Fixed64.Zero)),
            ContactAnchor.FromWorldPoint(
                right.Body.WorldCenterOfMass + Vector3d.Up),
            Fixed64.Half,
            Vector3d.Right);
        Vector3d leftVelocity = left.Body.LinearVelocity;
        Vector3d rightVelocity = right.Body.LinearVelocity;

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        left.Body.LinearVelocity.X.Should().BeLessThan(leftVelocity.X);
        right.Body.LinearVelocity.X.Should().BeGreaterThan(rightVelocity.X);
        left.Body.AngularVelocity.Z.Should().BeGreaterThan(Fixed64.Zero);
        right.Body.AngularVelocity.Z.Should().BeLessThan(Fixed64.Zero);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void CalculateImpulse_WhenExactResponseRoundsBelowVelocityPrecision_ShouldNotReportFailure(
        bool loggingEnabled)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> lower =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> upper =
            scenario.CreateSphere(Vector3d.Up);
        lower.Collider.Material = PhysicsMaterial.Frictionless;
        upper.Collider.Material = PhysicsMaterial.Frictionless;
        lower.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Up * StandardSpeed);
        upper.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Down * StandardSpeed);
        CollisionPair pair =
            scenario.CreatePair(lower.Collider, upper.Collider);
        pair.Manifold.SetContact(
            new ContactAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                Vector3d.Right * Fixed64.MinIncrement),
            ContactAnchor.FromWorldPoint(upper.Body.WorldCenterOfMass),
            Fixed64.Half,
            Vector3d.Up);
        Vector3d lowerLinearVelocity = lower.Body.LinearVelocity;
        Vector3d lowerAngularVelocity = lower.Body.AngularVelocity;
        Vector3d upperLinearVelocity = upper.Body.LinearVelocity;
        Vector3d upperAngularVelocity = upper.Body.AngularVelocity;
        string? loggedMessage = null;
        DiagnosticLevel originalLevel = GravitasLogger.MinimumLevel;
        Action<DiagnosticLevel, string, string> originalHandler =
            GravitasLogger.LogHandler;

        try
        {
            GravitasLogger.MinimumLevel = loggingEnabled
                ? DiagnosticLevel.Error
                : DiagnosticLevel.None;
            GravitasLogger.LogHandler =
                (_, message, _) => loggedMessage = message;

            CollisionResponse.CalculateImpulse(
                pair,
                applyCachedImpulse: false,
                applyPositionCorrection: false);
        }
        finally
        {
            GravitasLogger.MinimumLevel = originalLevel;
            GravitasLogger.LogHandler = originalHandler;
        }

        lower.Body.LinearVelocity.Should().Be(lowerLinearVelocity);
        lower.Body.AngularVelocity.Should().Be(lowerAngularVelocity);
        upper.Body.LinearVelocity.Should().Be(upperLinearVelocity);
        upper.Body.AngularVelocity.Should().Be(upperAngularVelocity);
        loggedMessage.Should().BeNull();
    }

    [Fact]
    public void CalculateImpulse_WhenExactImpulseRoundsToZero_ShouldApplyAngularResponse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 minimum = Fixed64.MinIncrement;
        var dynamicCollider = new UnsupportedTestCollider3D
        {
            InertiaTensor = new Fixed3x3(
                minimum, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, minimum, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, minimum)
        };
        ScenarioBody<UnsupportedTestCollider3D> dynamic =
            scenario.CreateBody(
                dynamicCollider,
                Vector3d.Zero,
                FixedQuaternion.Identity);
        dynamic.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        dynamic.Body.EffectiveInverseInertiaTensor.M33.Should()
            .Be(Fixed64.MaxValue);
        dynamic.Body.ApplyCollisionAngularVelocityDelta(-Vector3d.Forward);
        var wallCollider = new UnsupportedTestCollider3D();
        ScenarioBody<UnsupportedTestCollider3D> wall =
            scenario.CreateBody(
                wallCollider,
                Vector3d.Right,
                FixedQuaternion.Identity,
                immovable: true);
        dynamic.Collider.Material = PhysicsMaterial.Frictionless;
        wall.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair =
            scenario.CreatePair(dynamic.Collider, wall.Collider);
        SetExactContact(
            pair,
            dynamic.Collider,
            new ContactAnchor(
                Vector3d.Up * Fixed64.MaxValue,
                Vector3d.Up * Fixed64.MaxValue));

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        pair.TryGetWarmStartImpulse(
                pair.Manifold.PrimaryContact.ContactId,
                out ContactWarmStartImpulse cached)
            .Should()
            .BeTrue();
        cached.NormalImpulse.Should().Be(Fixed64.Zero);
        dynamic.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithWideSharedImpulse_ShouldApplyRepresentableBodyDeltas()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(
            Vector3d.Zero,
            mass: Fixed64.MaxValue);
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(
            Vector3d.Right,
            mass: Fixed64.MaxValue);
        left.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        right.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        left.Collider.Material = PhysicsMaterial.Frictionless;
        right.Collider.Material = PhysicsMaterial.Frictionless;
        left.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right * (Fixed64)6);
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        SetExactContact(
            pair,
            left.Collider,
            CreateExactParallelAnchor(Vector3d.Zero));

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        left.Body.LinearVelocity.Should().Be(Vector3d.Right * (Fixed64)3);
        right.Body.LinearVelocity.Should().Be(Vector3d.Right * (Fixed64)3);
        pair.TryGetWarmStartImpulse(
                pair.Manifold.PrimaryContact.ContactId,
                out ContactWarmStartImpulse cached)
            .Should()
            .BeTrue();
        cached.NormalImpulse.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CalculateImpulse_WithUnrepresentableFinalAngularDelta_ShouldRejectAtomically(
        bool loggingEnabled)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var moverCollider = new UnsupportedTestCollider3D
        {
            InertiaTensor = new Fixed3x3(
                Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.One)
        };
        ScenarioBody<UnsupportedTestCollider3D> mover =
            scenario.CreateBody(
                moverCollider,
                Vector3d.Zero,
                FixedQuaternion.Identity);
        ScenarioBody<UnsupportedTestCollider3D> wall =
            scenario.CreateBody(
                new UnsupportedTestCollider3D(),
                Vector3d.Forward,
                FixedQuaternion.Identity,
                immovable: true);
        mover.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        var elastic = new PhysicsMaterial(
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One);
        mover.Collider.Material = elastic;
        wall.Collider.Material = elastic;
        mover.Body.ApplyCollisionAngularVelocityDelta(
            Vector3d.Up * Fixed64.MaxValue);
        CollisionPair pair =
            scenario.CreatePair(mover.Collider, wall.Collider);
        SetExactContact(
            pair,
            mover.Collider,
            new ContactAnchor(
                Vector3d.Forward * Fixed64.MaxValue,
                Vector3d.Forward * Fixed64.MinIncrement
                - Vector3d.Right
                + Vector3d.Up * Fixed64.MinIncrement));
        Vector3d originalAngularVelocity = mover.Body.AngularVelocity;
        string? loggedMessage = null;
        DiagnosticLevel originalLevel = GravitasLogger.MinimumLevel;
        Action<DiagnosticLevel, string, string> originalHandler =
            GravitasLogger.LogHandler;

        try
        {
            GravitasLogger.MinimumLevel = loggingEnabled
                ? DiagnosticLevel.Error
                : DiagnosticLevel.None;
            GravitasLogger.LogHandler =
                (_, message, _) => loggedMessage = message;
            CollisionResponse.CalculateImpulse(
                pair,
                applyCachedImpulse: false,
                applyPositionCorrection: false);
        }
        finally
        {
            GravitasLogger.MinimumLevel = originalLevel;
            GravitasLogger.LogHandler = originalHandler;
        }

        mover.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        mover.Body.AngularVelocity.Should().Be(originalAngularVelocity);
        wall.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        wall.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(
                pair.Manifold.PrimaryContact.ContactId,
                out _)
            .Should()
            .BeFalse();
        loggedMessage.Should().Be(
            loggingEnabled
                ? "Contact response is outside the representable velocity domain."
                : null);
    }

    [Fact]
    public void CalculateImpulse_WithExactLeverWarmStart_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Right);
        left.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        right.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        pair.Manifold.SetContact(
            CreateExactParallelAnchor(Vector3d.Up),
            ContactAnchor.FromWorldPoint(
                right.Body.WorldCenterOfMass + Vector3d.Up),
            Fixed64.Half,
            Vector3d.Right);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            (Fixed64)4,
            Fixed64.One,
            Fixed64.Half);

        void Resolve() =>
            CollisionResponse.CalculateImpulse(
                pair,
                applyCachedImpulse: false,
                applyPositionCorrection: false);

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            Resolve,
            warmupIterations: 8,
            stabilizationIterations: 2,
            measurementIterations: 4);

        allocatedBytes.Should().Be(0);
        pair.TryGetWarmStartImpulse(
                pair.Manifold.PrimaryContact.ContactId,
                out ContactWarmStartImpulse impulse)
            .Should()
            .BeTrue();
        impulse.TangentImpulse.Should().Be(Fixed64.One);
        impulse.SecondaryTangentImpulse.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void CalculateImpulse_WithExactLeverSlidingContact_ShouldUseDynamicFriction()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            Vector3d.Zero,
            immovable: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(
            Vector3d.Right,
            mass: Fixed64.Half);
        PhysicsMaterial sliding = new(
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Zero);
        wall.Collider.Material = sliding;
        mover.Collider.Material = sliding;
        mover.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        Push(
            mover.Body,
            Vector3d.Left * StandardSpeed
            + Vector3d.Forward * (Fixed64)3);
        CollisionPair pair =
            scenario.CreatePair(wall.Collider, mover.Collider);
        SetExactContact(
            pair,
            wall.Collider,
            CreateExactParallelAnchor(Vector3d.Zero));

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        mover.Body.LinearVelocity.X.Should().Be(Fixed64.Zero);
        mover.Body.LinearVelocity.Z.Should().BeGreaterThan(Fixed64.Zero);
        mover.Body.LinearVelocity.Z.Should().BeLessThan((Fixed64)3);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CalculateImpulse_WithOpposingExtremePointVelocitiesAndCache_ShouldResolveExactly(
        bool exactLever,
        bool mirrored)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MaxSpeed = Fixed64.MaxValue;
        scenario.Context.Environment.MaxFallSpeed = Fixed64.MaxValue;
        ScenarioBody<LSSphereCollider> left =
            scenario.CreateSphere(Vector3d.Zero, mass: Fixed64.Two);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Right, mass: Fixed64.Two);
        left.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        right.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        var rough = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        left.Collider.Material = rough;
        right.Collider.Material = rough;
        Fixed64 direction = mirrored
            ? -Fixed64.One
            : Fixed64.One;
        left.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Forward * Fixed64.MaxValue * direction);
        right.Body.ApplyCollisionLinearVelocityDelta(
            -Vector3d.Forward * Fixed64.MaxValue * direction);
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        if (exactLever)
        {
            SetExactContact(
                pair,
                left.Collider,
                CreateExactParallelAnchor(Vector3d.Zero));
        }
        else
        {
            pair.Manifold.SetContact(
                left.Body.WorldCenterOfMass,
                right.Body.WorldCenterOfMass,
                Fixed64.Zero,
                Vector3d.Right);
        }

        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.MaxValue,
            Fixed64.MaxValue * direction,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        left.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        right.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse impulse)
            .Should()
            .BeTrue();
        impulse.NormalImpulse.Should().Be(Fixed64.MaxValue);
        impulse.TangentImpulse.Should().Be(
            -Fixed64.MaxValue * direction);
        impulse.SecondaryTangentImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithExactExtremeCachedDisk_ShouldProjectToDynamicLimit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MaxSpeed = Fixed64.MaxValue;
        scenario.Context.Environment.MaxFallSpeed = Fixed64.MaxValue;
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            Vector3d.Zero,
            immovable: true);
        ScenarioBody<LSSphereCollider> mover =
            scenario.CreateSphere(Vector3d.Right);
        mover.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        var rough = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        wall.Collider.Material = rough;
        mover.Collider.Material = rough;
        CollisionPair pair =
            scenario.CreatePair(wall.Collider, mover.Collider);
        SetExactContact(
            pair,
            mover.Collider,
            CreateExactParallelAnchor(Vector3d.Zero));
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.MaxValue);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse impulse)
            .Should()
            .BeTrue();
        impulse.TangentImpulse.Should().BeGreaterThan(Fixed64.Zero);
        impulse.SecondaryTangentImpulse.Should().Be(
            impulse.TangentImpulse);
        impulse.TangentImpulse.Should().BeLessThan(Fixed64.MaxValue);
        new Vector2d(
                impulse.TangentImpulse,
                impulse.SecondaryTangentImpulse)
            .Magnitude
            .Should()
            .BeLessThanOrEqualTo(Fixed64.MaxValue);
        mover.Body.LinearVelocity.Should().NotBe(Vector3d.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithExactFrictionlessCache_ShouldRemoveBothTangentAxes()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Right);
        left.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        right.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        left.Collider.Material = PhysicsMaterial.Frictionless;
        right.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        SetExactContact(
            pair,
            left.Collider,
            CreateExactParallelAnchor(Vector3d.Zero));
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Half);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        left.Body.LinearVelocity.Should().NotBe(Vector3d.Zero);
        right.Body.LinearVelocity.Should().Be(-left.Body.LinearVelocity);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse impulse)
            .Should()
            .BeTrue();
        impulse.TangentImpulse.Should().Be(Fixed64.Zero);
        impulse.SecondaryTangentImpulse.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CalculateImpulse_WithSubPrecisionCoulombDiskProducts_ShouldUseExactProjection(
        bool scaleUnderflows)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            Vector3d.Zero,
            immovable: true);
        ScenarioBody<LSSphereCollider> mover =
            scenario.CreateSphere(Vector3d.Right);
        mover.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        Fixed64 staticFriction = scaleUnderflows
            ? Fixed64.One
            : Fixed64.MinIncrement;
        var material = new PhysicsMaterial(
            staticFriction,
            Fixed64.MinIncrement,
            Fixed64.Zero);
        wall.Collider.Material = material;
        mover.Collider.Material = material;
        CollisionPair pair =
            scenario.CreatePair(wall.Collider, mover.Collider);
        pair.Manifold.SetContact(
            wall.Body.WorldCenterOfMass,
            mover.Body.WorldCenterOfMass,
            Fixed64.Zero,
            Vector3d.Right);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.One,
            scaleUnderflows
                ? Fixed64.Two
                : Fixed64.MinIncrement * Fixed64.Two,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse impulse)
            .Should()
            .BeTrue();
        impulse.TangentImpulse.Should().Be(Fixed64.MinIncrement);
        mover.Body.LinearVelocity.Should().NotBe(Vector3d.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithExactLeverAndDeadzoneTangentVelocity_ShouldRetainVelocity()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            Vector3d.Zero,
            immovable: true);
        ScenarioBody<LSSphereCollider> mover =
            scenario.CreateSphere(Vector3d.Right);
        mover.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        wall.Collider.Material = PhysicsMaterial.Default;
        mover.Collider.Material = PhysicsMaterial.Default;
        Vector3d velocity = Vector3d.Forward * Fixed64.Epsilon;
        mover.Body.ApplyCollisionLinearVelocityDelta(velocity);
        CollisionPair pair =
            scenario.CreatePair(wall.Collider, mover.Collider);
        SetExactContact(
            pair,
            wall.Collider,
            CreateExactParallelAnchor(Vector3d.Zero));
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        mover.Body.LinearVelocity.Should().Be(velocity);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse impulse)
            .Should()
            .BeTrue();
        impulse.TangentImpulse.Should().Be(Fixed64.Zero);
        impulse.SecondaryTangentImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithSubPrecisionFrictionLimit_ShouldPreserveAmplifiedResponse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            Vector3d.Zero,
            immovable: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(
            Vector3d.Right,
            mass: Fixed64.FromFraction(1, 4));
        mover.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        var material = new PhysicsMaterial(
            Fixed64.Half,
            Fixed64.Half,
            Fixed64.Zero);
        wall.Collider.Material = material;
        mover.Collider.Material = material;
        CollisionPair pair =
            scenario.CreatePair(wall.Collider, mover.Collider);
        pair.Manifold.SetContact(
            wall.Body.WorldCenterOfMass,
            mover.Body.WorldCenterOfMass,
            Fixed64.Zero,
            Vector3d.Right);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.MinIncrement,
            Fixed64.MinIncrement,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        mover.Body.LinearVelocity.Should().NotBe(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse impulse)
            .Should()
            .BeTrue();
        impulse.TangentImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithSubPrecisionTangentImpulse_ShouldPreserveAmplifiedResponse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            Vector3d.Zero,
            immovable: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(
            Vector3d.Right,
            mass: Fixed64.MinIncrement);
        mover.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        wall.Collider.Material = PhysicsMaterial.Default;
        mover.Collider.Material = PhysicsMaterial.Default;
        mover.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Forward
            * (Fixed64.Epsilon + Fixed64.MinIncrement));
        CollisionPair pair =
            scenario.CreatePair(wall.Collider, mover.Collider);
        pair.Manifold.SetContact(
            wall.Body.WorldCenterOfMass,
            mover.Body.WorldCenterOfMass,
            Fixed64.Zero,
            Vector3d.Right);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        mover.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(contact.ContactId, out _)
            .Should()
            .BeTrue();
    }

    [Fact]
    public void CalculateImpulse_WithSubPrecisionConstrainedMass_ShouldUseExactResponse()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> driver = scenario.CreateSphere(
            Vector3d.Zero,
            isKinematic: true);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(
            Vector3d.Right,
            mass: Fixed64.MinIncrement);
        target.Body.FreezeAxes =
            BodyFreezeAxes3D.PositionY
            | BodyFreezeAxes3D.PositionZ
            | BodyFreezeAxes3D.Rotation;
        driver.Collider.Material = PhysicsMaterial.Default;
        target.Collider.Material = PhysicsMaterial.Default;
        Vector3d normal = new Vector3d(
            Fixed64.One,
            Fixed64.FromRaw(23),
            Fixed64.Zero).Normalized;
        Vector3d tangent = SolverContact.CreateTangent(normal);
        driver.Body.Agent.Transform.LocalPosition =
            tangent * scenario.Context.DeltaTime;
        scenario.Context.AdvanceLateSimulateToken();
        driver.Body.EnsureContinuousCollisionFramePrepared(
            scenario.Context.LateSimulateToken);
        CollisionPair pair =
            scenario.CreatePair(driver.Collider, target.Collider);
        pair.Manifold.SetContact(
            driver.Body.WorldCenterOfMass,
            target.Body.WorldCenterOfMass,
            Fixed64.Zero,
            normal);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse cached)
            .Should()
            .BeTrue();
        target.Body.LinearVelocity.X.Should().NotBe(
            Fixed64.Zero,
            "the exact response should preserve constrained mass; driver {0}, cache {1}/{2}/{3}",
            driver.Body.SampleContinuousCollisionLinearVelocity(Fixed64.One),
            cached.NormalImpulse,
            cached.TangentImpulse,
            cached.SecondaryTangentImpulse);
    }

    [Fact]
    public void CalculateImpulse_WithSubPrecisionImpulseComponent_ShouldUseExactApplication()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            Vector3d.Zero,
            immovable: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(
            Vector3d.Right,
            mass: Fixed64.MinIncrement);
        mover.Body.FreezeAxes =
            BodyFreezeAxes3D.PositionY
            | BodyFreezeAxes3D.PositionZ
            | BodyFreezeAxes3D.Rotation;
        wall.Collider.Material = PhysicsMaterial.Frictionless;
        mover.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair =
            scenario.CreatePair(wall.Collider, mover.Collider);
        Vector3d normal = new Vector3d(
            Fixed64.One,
            Fixed64.FromRaw(23),
            Fixed64.Zero).Normalized;
        pair.Manifold.SetContact(
            wall.Body.WorldCenterOfMass,
            mover.Body.WorldCenterOfMass,
            Fixed64.Zero,
            normal);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.Zero,
            Fixed64.MinIncrement,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        mover.Body.LinearVelocity.X.Should().NotBe(Fixed64.Zero);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse impulse)
            .Should()
            .BeTrue();
        impulse.TangentImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithSubPrecisionTorque_ShouldUseExactApplication()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        Fixed64 inverseInertia = (Fixed64)8;
        Fixed64 inertia = Fixed64.One / inverseInertia;
        var collider = new UnsupportedTestCollider3D
        {
            InertiaTensor = new Fixed3x3(
                inertia, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, inertia, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, inertia)
        };
        ScenarioBody<UnsupportedTestCollider3D> mover =
            scenario.CreateBody(
                collider,
                Vector3d.Zero,
                FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            Vector3d.Right,
            immovable: true);
        mover.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        mover.Body.EffectiveInverseInertiaTensor.M11.Should()
            .Be(inverseInertia);
        mover.Collider.Material = PhysicsMaterial.Frictionless;
        wall.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair =
            scenario.CreatePair(mover.Collider, wall.Collider);
        bool moverIsA = ReferenceEquals(pair.ColliderA, mover.Collider);
        ContactAnchor moverAnchor = ContactAnchor.FromWorldPoint(
            mover.Body.WorldCenterOfMass
            + Vector3d.Up * Fixed64.MinIncrement);
        ContactAnchor wallAnchor =
            ContactAnchor.FromWorldPoint(wall.Body.WorldCenterOfMass);
        Vector3d normal =
            (pair.ColliderB.Center - pair.ColliderA.Center).Normalized;
        pair.Manifold.SetContact(
            moverIsA ? moverAnchor : wallAnchor,
            moverIsA ? wallAnchor : moverAnchor,
            Fixed64.Zero,
            normal);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        ExactLever3D moverLever =
            (moverIsA ? contact.AnchorA : contact.AnchorB).GetLeverFrom(
                mover.Body.GetCenterOfMassAnchor());
        Vector3d moverImpulse =
            SolverContact.CreateTangent(contact.Normal)
            * -Fixed64.FromFraction(1, 4);
        if (moverIsA)
            moverImpulse = -moverImpulse;
        Assert.True(ExactContactLever3D.TryGetAngularVelocityDelta(
                mover.Body,
                moverLever,
                moverImpulse,
                out Vector3d expectedAngularVelocity));
        Assert.NotEqual(Vector3d.Zero, expectedAngularVelocity);
        Assert.True((moverIsA ? contact.AnchorA : contact.AnchorB)
            .TryGetOffsetFrom(
                mover.Body.GetCenterOfMassAnchor(),
                out Vector3d compactLever));
        Assert.True(Vector3d.TryCross(
                compactLever,
                moverImpulse,
                out Vector3d compactTorque));
        Assert.Equal(Vector3d.Zero, compactTorque);
        Assert.False(ContactResponseArithmetic3D.TryCross(
                compactLever,
                moverImpulse,
                out compactTorque));
        Assert.False(
            ContactNormalImpulse3D.TryComputeAngularVelocityDelta(
                mover.Body,
                compactLever,
                moverImpulse,
                out Vector3d compactAngularVelocity));
        Assert.Equal(Vector3d.Zero, compactAngularVelocity);
        Vector3d initialAngularVelocity = compactLever.Normalized;
        mover.Body.ApplyCollisionAngularVelocityDelta(initialAngularVelocity);
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.Zero,
            Fixed64.FromFraction(1, 4),
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse impulse)
            .Should()
            .BeTrue();
        impulse.TangentImpulse.Should().Be(Fixed64.Zero);
        mover.Body.AngularVelocity.Should().Be(
            initialAngularVelocity + expectedAngularVelocity);
    }

    [Fact]
    public void CalculateImpulse_WithAngularEffectiveMassOverflow_ShouldResolveThroughExactFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new UnsupportedTestCollider3D
        {
            InertiaTensor = new Fixed3x3(
                Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.MinIncrement, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.MinIncrement)
        };
        ScenarioBody<UnsupportedTestCollider3D> mover =
            scenario.CreateBody(
                collider,
                Vector3d.Zero,
                FixedQuaternion.Identity);
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            Vector3d.Right,
            immovable: true);
        mover.Collider.Material = PhysicsMaterial.Default;
        wall.Collider.Material = PhysicsMaterial.Default;
        mover.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Forward);
        CollisionPair pair =
            scenario.CreatePair(mover.Collider, wall.Collider);
        pair.Manifold.SetContact(
            ContactAnchor.FromWorldPoint(
                mover.Body.WorldCenterOfMass
                + Vector3d.Up * Fixed64.FromFraction(3, 2)),
            ContactAnchor.FromWorldPoint(wall.Body.WorldCenterOfMass),
            Fixed64.Zero,
            Vector3d.Right);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            (Fixed64)4,
            Fixed64.Zero,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse impulse)
            .Should()
            .BeTrue();
        mover.Body.LinearVelocity.Should().Be(Vector3d.Forward);
        impulse.TangentImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithExactKinematicTangentAndFrozenTargetAxis_ShouldSkipUndefinedFriction()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> driver = scenario.CreateSphere(
            Vector3d.Zero,
            isKinematic: true);
        ScenarioBody<LSSphereCollider> target =
            scenario.CreateSphere(Vector3d.Right);
        driver.Collider.Material = PhysicsMaterial.Default;
        target.Collider.Material = PhysicsMaterial.Default;
        target.Body.FreezeAxes =
            BodyFreezeAxes3D.PositionZ
            | BodyFreezeAxes3D.Rotation;
        driver.Body.Agent.Transform.LocalPosition = new Vector3d(
            Fixed64.FromFraction(1, 10),
            Fixed64.Zero,
            Fixed64.FromFraction(1, 20));
        scenario.Context.AdvanceLateSimulateToken();
        driver.Body.EnsureContinuousCollisionFramePrepared(
            scenario.Context.LateSimulateToken);
        CollisionPair pair =
            scenario.CreatePair(driver.Collider, target.Collider);
        SetExactContact(
            pair,
            driver.Collider,
            CreateExactParallelAnchor(Vector3d.Zero));

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        target.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        target.Body.LinearVelocity.Z.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CalculateImpulse_WithUnrepresentableExactFrictionProduct_ShouldPreservePhysicalResponse(
        bool pointVelocityOverflows)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> mover =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            Vector3d.Right,
            immovable: true);
        PhysicsMaterial rough =
            new(Fixed64.One, Fixed64.One, Fixed64.Zero);
        mover.Collider.Material = rough;
        wall.Collider.Material = rough;
        Push(
            mover.Body,
            Vector3d.Right * StandardSpeed
            + Vector3d.Forward);
        if (pointVelocityOverflows)
        {
            mover.Body.ApplyCollisionAngularVelocityDelta(
                Vector3d.Up * Fixed64.MaxValue);
        }

        CollisionPair pair =
            scenario.CreatePair(mover.Collider, wall.Collider);
        SetExactContact(
            pair,
            mover.Collider,
            CreateExactParallelAnchor(Vector3d.Zero));

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        mover.Body.LinearVelocity.X.Abs().Should()
            .BeLessThanOrEqualTo(Fixed64.MinIncrement);
        pair.TryGetWarmStartImpulse(
                pair.Manifold.PrimaryContact.ContactId,
                out ContactWarmStartImpulse impulse)
            .Should()
            .BeTrue();
        if (pointVelocityOverflows)
        {
            (impulse.TangentImpulse != Fixed64.Zero
                || impulse.SecondaryTangentImpulse != Fixed64.Zero)
                .Should()
                .BeTrue();
        }
    }

    [Fact]
    public void CalculateImpulse_WithUnrepresentableExactWarmStart_ShouldDiscardCacheAndColdSolve()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> mover =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            Vector3d.Right,
            immovable: true);
        mover.Collider.Material = PhysicsMaterial.Frictionless;
        wall.Collider.Material = PhysicsMaterial.Frictionless;
        Push(mover.Body, Vector3d.Right * StandardSpeed);
        CollisionPair pair =
            scenario.CreatePair(mover.Collider, wall.Collider);
        SetExactContact(
            pair,
            mover.Collider,
            CreateExactParallelAnchor(Vector3d.Zero));
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: true,
            applyPositionCorrection: false);

        mover.Body.LinearVelocity.X.Should().Be(Fixed64.Zero);
        mover.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse replacement)
            .Should()
            .BeTrue();
        replacement.NormalImpulse.Should().BeGreaterThan(Fixed64.Zero);
        replacement.TangentImpulse.Should().Be(Fixed64.Zero);
        replacement.SecondaryTangentImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithExactWarmStartOverflowingCurrentState_ShouldRejectPairedMutation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MaxSpeed = Fixed64.MaxValue;
        scenario.Context.Environment.MaxFallSpeed = Fixed64.MaxValue;
        ScenarioBody<LSSphereCollider> left =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Right);
        left.Collider.Material = PhysicsMaterial.Frictionless;
        right.Collider.Material = PhysicsMaterial.Frictionless;
        right.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right * Fixed64.MaxValue);
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        SetExactContact(
            pair,
            left.Collider,
            CreateExactParallelAnchor(Vector3d.Zero));
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d leftLinear = left.Body.LinearVelocity;
        Vector3d leftAngular = left.Body.AngularVelocity;
        Vector3d rightLinear = right.Body.LinearVelocity;
        Vector3d rightAngular = right.Body.AngularVelocity;
        leftLinear.Should().Be(Vector3d.Zero);
        rightLinear.X.Should().Be(Fixed64.MaxValue);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: true,
            applyPositionCorrection: false);

        left.Body.LinearVelocity.Should().Be(leftLinear);
        left.Body.AngularVelocity.Should().Be(leftAngular);
        right.Body.LinearVelocity.Should().Be(rightLinear);
        right.Body.AngularVelocity.Should().Be(rightAngular);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse replacement)
            .Should()
            .BeTrue();
        replacement.NormalImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WhenNormalDeltaWouldOverflowCurrentVelocity_ShouldRejectPairedMutation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MaxSpeed = Fixed64.MaxValue;
        scenario.Context.Environment.MaxFallSpeed = Fixed64.MaxValue;
        ScenarioBody<LSSphereCollider> left =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Right);
        left.Collider.Material = PhysicsMaterial.Frictionless;
        right.Collider.Material = PhysicsMaterial.Frictionless;
        left.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right * Fixed64.MaxValue);
        right.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right * Fixed64.MaxValue);
        right.Body.ApplyCollisionAngularVelocityDelta(Vector3d.Forward);
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        pair.Manifold.SetContact(
            ContactAnchor.FromWorldPoint(left.Body.WorldCenterOfMass),
            ContactAnchor.FromWorldPoint(
                right.Body.WorldCenterOfMass + Vector3d.Up),
            Fixed64.Zero,
            Vector3d.Right);
        Vector3d leftLinear = left.Body.LinearVelocity;
        Vector3d leftAngular = left.Body.AngularVelocity;
        Vector3d rightLinear = right.Body.LinearVelocity;
        Vector3d rightAngular = right.Body.AngularVelocity;

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        left.Body.LinearVelocity.Should().Be(leftLinear);
        left.Body.AngularVelocity.Should().Be(leftAngular);
        right.Body.LinearVelocity.Should().Be(rightLinear);
        right.Body.AngularVelocity.Should().Be(rightAngular);
        pair.TryGetWarmStartImpulse(
                pair.Manifold.PrimaryContact.ContactId,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void CalculateImpulse_AfterDiscardingWarmStart_ShouldRetainPreCorrectionContactGeometry()
    {
        var cold = ResolveBoundaryContactWithPositionCorrection(
            seedInvalidWarmStart: false);
        var recovered = ResolveBoundaryContactWithPositionCorrection(
            seedInvalidWarmStart: true);

        recovered.Should().Be(cold);
    }

    [Fact]
    public void CalculateImpulse_WithUnrepresentableCompactWarmStartComposition_ShouldColdSolve()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MaxSpeed = Fixed64.MaxValue;
        scenario.Context.Environment.MaxFallSpeed = Fixed64.MaxValue;
        ScenarioBody<LSSphereCollider> left =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Right);
        left.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        right.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        left.Collider.Material = PhysicsMaterial.Frictionless;
        right.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        Vector3d normal = Vector3d.Right;
        pair.Manifold.SetContact(
            left.Body.WorldCenterOfMass,
            right.Body.WorldCenterOfMass,
            Fixed64.Zero,
            normal);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            normal,
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: true,
            applyPositionCorrection: false);

        left.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        right.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse replacement)
            .Should()
            .BeTrue();
        replacement.NormalImpulse.Should().Be(Fixed64.Zero);
        replacement.TangentImpulse.Should().Be(Fixed64.Zero);
        replacement.SecondaryTangentImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithUnmaterializableWarmStartAndRepresentableBodyDeltas_ShouldRetainCache()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MaxSpeed = Fixed64.MaxValue;
        scenario.Context.Environment.MaxFallSpeed = Fixed64.MaxValue;
        ScenarioBody<LSSphereCollider> left =
            scenario.CreateSphere(Vector3d.Zero, mass: Fixed64.Two);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Right, mass: Fixed64.Two);
        left.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        right.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        left.Collider.Material = PhysicsMaterial.Default;
        right.Collider.Material = PhysicsMaterial.Default;
        var expected = new Vector3d(
            (Fixed64)1073741824,
            Fixed64.Zero,
            (Fixed64)1073741824);
        left.Body.ApplyCollisionLinearVelocityDelta(expected);
        right.Body.ApplyCollisionLinearVelocityDelta(-expected);
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        pair.Manifold.SetContact(
            left.Body.WorldCenterOfMass,
            right.Body.WorldCenterOfMass,
            Fixed64.Zero,
            Vector3d.Right);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: true,
            applyPositionCorrection: false);

        left.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        right.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse retained)
            .Should()
            .BeTrue();
        retained.NormalImpulse.Should().Be(Fixed64.MaxValue);
        retained.TangentImpulse.Should().Be(-Fixed64.MaxValue);
        retained.SecondaryTangentImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithFrozenOverflowingImpulseAxis_ShouldProjectBeforeScaling()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MaxSpeed = Fixed64.MaxValue;
        scenario.Context.Environment.MaxFallSpeed = Fixed64.MaxValue;
        ScenarioBody<LSSphereCollider> left =
            scenario.CreateSphere(Vector3d.Zero, mass: Fixed64.Half);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Right, mass: Fixed64.Half);
        BodyFreezeAxes3D constraints =
            BodyFreezeAxes3D.PositionX | BodyFreezeAxes3D.Rotation;
        left.Body.FreezeAxes = constraints;
        right.Body.FreezeAxes = constraints;
        left.Collider.Material = PhysicsMaterial.Default;
        right.Collider.Material = PhysicsMaterial.Default;
        left.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Up * Fixed64.Two);
        right.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Down * Fixed64.Two);
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        pair.Manifold.SetContact(
            left.Body.WorldCenterOfMass,
            right.Body.WorldCenterOfMass,
            Fixed64.Zero,
            Vector3d.Right);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.One);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: true,
            applyPositionCorrection: false);

        left.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        right.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse retained)
            .Should()
            .BeTrue();
        retained.NormalImpulse.Should().Be(Fixed64.MaxValue);
        retained.TangentImpulse.Should().Be(Fixed64.Zero);
        retained.SecondaryTangentImpulse.Should().Be(Fixed64.One);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CalculateImpulse_WithMinimumVectorComponent_ShouldRejectUnrepresentableOppositeBody(
        bool minimumOnSecondaryTangent)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Right);
        left.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        right.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        left.Collider.Material = PhysicsMaterial.Frictionless;
        right.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        Vector3d normal = minimumOnSecondaryTangent
            ? Vector3d.Right
            : Vector3d.Left;
        pair.Manifold.SetContact(
            left.Body.WorldCenterOfMass,
            right.Body.WorldCenterOfMass,
            Fixed64.Zero,
            normal);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.Zero,
            minimumOnSecondaryTangent
                ? Fixed64.Zero
                : Fixed64.MinValue,
            minimumOnSecondaryTangent
                ? Fixed64.MinValue
                : Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: true,
            applyPositionCorrection: false);

        left.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        right.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse replacement)
            .Should()
            .BeTrue();
        replacement.NormalImpulse.Should().Be(Fixed64.Zero);
        replacement.TangentImpulse.Should().Be(Fixed64.Zero);
        replacement.SecondaryTangentImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ExactImpulseCombination_ShouldNarrowOnlyFinalBodyDeltas()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left =
            scenario.CreateSphere(Vector3d.Zero, mass: Fixed64.Two);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Right, mass: Fixed64.Two);
        left.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        right.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        ContactAnchor leftCenter = left.Body.GetCenterOfMassAnchor();
        ContactAnchor rightCenter = right.Body.GetCenterOfMassAnchor();
        ExactLever3D leftLever = leftCenter.GetLeverFrom(leftCenter);
        ExactLever3D rightLever = rightCenter.GetLeverFrom(rightCenter);
        ExactContactLever3D.TryGetAngularVelocityDelta(
                null,
                leftLever,
                Vector3d.One,
                out Vector3d bodylessAngular)
            .Should()
            .BeTrue();
        bodylessAngular.Should().Be(Vector3d.Zero);
        ExactContactLever3D.TryGetAngularVelocityDelta(
                left.Body,
                leftLever,
                Vector3d.One,
                out Vector3d frozenAngular)
            .Should()
            .BeTrue();
        frozenAngular.Should().Be(Vector3d.Zero);
        ExactContactLever3D.TryGetImpulseCombinationVelocityDeltas(
                null,
                leftLever,
                null,
                rightLever,
                Vector3d.Right,
                Fixed64.One,
                Vector3d.Up,
                Fixed64.One,
                Vector3d.Forward,
                Fixed64.One,
                out Vector3d bodylessLinearA,
                out Vector3d bodylessAngularA,
                out Vector3d bodylessLinearB,
                out Vector3d bodylessAngularB)
            .Should()
            .BeTrue();
        bodylessLinearA.Should().Be(Vector3d.Zero);
        bodylessAngularA.Should().Be(Vector3d.Zero);
        bodylessLinearB.Should().Be(Vector3d.Zero);
        bodylessAngularB.Should().Be(Vector3d.Zero);

        ExactContactLever3D.TryGetImpulseCombinationVelocityDeltas(
                left.Body,
                leftLever,
                right.Body,
                rightLever,
                Vector3d.Right,
                Fixed64.MaxValue,
                -Vector3d.Forward,
                Fixed64.MinValue,
                Vector3d.Up,
                Fixed64.Zero,
                out Vector3d linearA,
                out Vector3d angularA,
                out Vector3d linearB,
                out Vector3d angularB)
            .Should()
            .BeTrue();

        var expected = new Vector3d(
            (Fixed64)1073741824,
            Fixed64.Zero,
            (Fixed64)1073741824);
        linearA.Should().Be(-expected);
        angularA.Should().Be(Vector3d.Zero);
        linearB.Should().Be(expected);
        angularB.Should().Be(Vector3d.Zero);

        left.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        right.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        ExactContactLever3D.TryGetImpulseCombinationVelocityDeltas(
                left.Body,
                leftLever,
                right.Body,
                rightLever,
                Vector3d.Right,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                Vector3d.Zero,
                Fixed64.Zero,
                out linearA,
                out angularA,
                out linearB,
                out angularB)
            .Should()
            .BeTrue();
        linearA.Should().Be(Vector3d.Zero);
        angularA.Should().Be(Vector3d.Zero);
        linearB.Should().Be(Vector3d.Zero);
        angularB.Should().Be(Vector3d.Zero);

        ExactLever3D unitLever = ContactAnchor.FromWorldPoint(
                left.Body.WorldCenterOfMass + Vector3d.Right)
            .GetLeverFrom(leftCenter);
        ExactContactLever3D.TryGetImpulseCombinationVelocityDeltas(
                left.Body,
                unitLever,
                right.Body,
                rightLever,
                Vector3d.Up,
                Fixed64.MaxValue,
                Vector3d.Up,
                Fixed64.MaxValue,
                Vector3d.Zero,
                Fixed64.Zero,
                out _,
                out _,
                out _,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void CalculateImpulse_WithCompactPointVelocityOverflow_ShouldUseExactFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new UnsupportedTestCollider3D
        {
            InertiaTensor = new Fixed3x3(
                Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.MaxValue)
        };
        ScenarioBody<UnsupportedTestCollider3D> left =
            scenario.CreateBody(
                collider,
                Vector3d.Zero,
                FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Up);
        var rough = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero);
        left.Collider.Material = rough;
        right.Collider.Material = rough;
        left.Body.ApplyCollisionAngularVelocityDelta(
            Vector3d.Forward * Fixed64.MaxValue);
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        pair.Manifold.SetContact(
            ContactAnchor.FromWorldPoint(
                left.Body.WorldCenterOfMass
                + Vector3d.Up * Fixed64.MaxValue),
            ContactAnchor.FromWorldPoint(right.Body.WorldCenterOfMass),
            Fixed64.Zero,
            Vector3d.Up);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d leftAngularVelocity = left.Body.AngularVelocity;

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        Assert.NotEqual(Vector3d.Zero, left.Body.LinearVelocity);
        Assert.NotEqual(leftAngularVelocity, left.Body.AngularVelocity);
        right.Body.LinearVelocity.Should().Be(-left.Body.LinearVelocity);
        right.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse cached)
            .Should()
            .BeTrue();
        cached.NormalImpulse.Should().Be(Fixed64.One);
        (cached.TangentImpulse != Fixed64.Zero
            || cached.SecondaryTangentImpulse != Fixed64.Zero)
            .Should()
            .BeTrue();
    }

    [Fact]
    public void CalculateImpulse_WithOverflowingCompactEffectiveMass_ShouldUseExactFriction()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        scenario.Context.Environment.MaxSpeed = Fixed64.MaxValue;
        ScenarioBody<LSSphereCollider> left = scenario.CreateSphere(
            Vector3d.Zero,
            mass: Fixed64.MinIncrement);
        ScenarioBody<LSSphereCollider> right = scenario.CreateSphere(
            Vector3d.Right,
            mass: Fixed64.MinIncrement);
        left.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        right.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        left.Collider.Material = PhysicsMaterial.Default;
        right.Collider.Material = PhysicsMaterial.Default;
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        Vector3d normal =
            (pair.ColliderB.Center - pair.ColliderA.Center).Normalized;
        Vector3d tangent = SolverContact.CreateTangent(normal);
        pair.ColliderA.Body!.ApplyCollisionLinearVelocityDelta(tangent);
        pair.Manifold.SetContact(
            pair.ColliderA.Body.WorldCenterOfMass,
            pair.ColliderB.Body!.WorldCenterOfMass,
            Fixed64.Zero,
            normal);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            normal,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        Vector3d.Dot(pair.ColliderA.Body.LinearVelocity, tangent)
            .Should()
            .Be(Fixed64.Half);
        Vector3d.Dot(pair.ColliderB.Body.LinearVelocity, tangent)
            .Should()
            .Be(Fixed64.Half);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse cached)
            .Should()
            .BeTrue();
        cached.TangentImpulse.Abs().Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithOverflowingCompactTangentDot_ShouldUseExactFriction()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        scenario.Context.Environment.MaxSpeed = Fixed64.MaxValue;
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            Vector3d.Zero,
            immovable: true);
        ScenarioBody<LSSphereCollider> mover =
            scenario.CreateSphere(Vector3d.Right);
        mover.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        wall.Collider.Material = PhysicsMaterial.Default;
        mover.Collider.Material = PhysicsMaterial.Default;
        Vector3d velocity = new(
            Fixed64.Zero,
            -Fixed64.MaxValue,
            Fixed64.MaxValue);
        mover.Body.ApplyCollisionLinearVelocityDelta(velocity);
        CollisionPair pair =
            scenario.CreatePair(wall.Collider, mover.Collider);
        Vector3d normal = Vector3d.One.Normalized;
        pair.Manifold.SetContact(
            wall.Body.WorldCenterOfMass,
            mover.Body.WorldCenterOfMass,
            Fixed64.Zero,
            normal);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            normal,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        mover.Body.LinearVelocity.Should().NotBe(velocity);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse cached)
            .Should()
            .BeTrue();
        cached.TangentImpulse.Should().NotBe(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithOverflowingCompactTangentQuotient_ShouldUseExactFriction()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        scenario.Context.Environment.MaxSpeed = Fixed64.MaxValue;
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            Vector3d.Zero,
            immovable: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(
            Vector3d.Right,
            mass: (Fixed64)16_000_000);
        mover.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        wall.Collider.Material = PhysicsMaterial.Default;
        mover.Collider.Material = PhysicsMaterial.Default;
        CollisionPair pair =
            scenario.CreatePair(wall.Collider, mover.Collider);
        Vector3d normal =
            (pair.ColliderB.Center - pair.ColliderA.Center).Normalized;
        Vector3d tangent = SolverContact.CreateTangent(normal);
        mover.Body.ApplyCollisionLinearVelocityDelta(
            tangent * Fixed64.MaxValue);
        Vector3d velocity = mover.Body.LinearVelocity;
        pair.Manifold.SetContact(
            wall.Body.WorldCenterOfMass,
            mover.Body.WorldCenterOfMass,
            Fixed64.Zero,
            normal);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            normal,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        mover.Body.LinearVelocity.Should().NotBe(velocity);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse cached)
            .Should()
            .BeTrue();
        cached.TangentImpulse.Abs().Should().Be(Fixed64.MaxValue);
    }

    [Fact]
    public void CalculateImpulse_WithOverflowingCompactFrictionLimit_ShouldUseExactFriction()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            Vector3d.Zero,
            immovable: true);
        ScenarioBody<LSSphereCollider> mover =
            scenario.CreateSphere(Vector3d.Right);
        mover.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        var material = new PhysicsMaterial(
            Fixed64.Two,
            Fixed64.Two,
            Fixed64.Zero);
        wall.Collider.Material = material;
        mover.Collider.Material = material;
        CollisionPair pair =
            scenario.CreatePair(wall.Collider, mover.Collider);
        Vector3d normal =
            (pair.ColliderB.Center - pair.ColliderA.Center).Normalized;
        Vector3d tangent = SolverContact.CreateTangent(normal);
        mover.Body.ApplyCollisionLinearVelocityDelta(tangent);
        pair.Manifold.SetContact(
            wall.Body.WorldCenterOfMass,
            mover.Body.WorldCenterOfMass,
            Fixed64.Zero,
            normal);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            normal,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        mover.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse cached)
            .Should()
            .BeTrue();
        cached.TangentImpulse.Abs().Should().Be(Fixed64.One);
    }

    [Fact]
    public void CalculateImpulse_WithSubPrecisionDynamicDiskComponent_ShouldUseExactFriction()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            Vector3d.Zero,
            immovable: true);
        ScenarioBody<LSSphereCollider> mover =
            scenario.CreateSphere(Vector3d.Right);
        mover.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        var material = new PhysicsMaterial(
            Fixed64.One,
            Fixed64.MinIncrement,
            Fixed64.Zero);
        wall.Collider.Material = material;
        mover.Collider.Material = material;
        CollisionPair pair =
            scenario.CreatePair(wall.Collider, mover.Collider);
        Vector3d normal =
            (pair.ColliderB.Center - pair.ColliderA.Center).Normalized;
        pair.Manifold.SetContact(
            wall.Body.WorldCenterOfMass,
            mover.Body.WorldCenterOfMass,
            Fixed64.Zero,
            normal);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            normal,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.One);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        mover.Body.LinearVelocity.Should().NotBe(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse cached)
            .Should()
            .BeTrue();
        cached.TangentImpulse.Should().Be(Fixed64.Zero);
        cached.SecondaryTangentImpulse.Should()
            .Be(Fixed64.MinIncrement);
    }

    [Fact]
    public void CalculateImpulse_WithUnrepresentableCompactWarmStartAngularTransform_ShouldColdSolve()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new UnsupportedTestCollider3D
        {
            InertiaTensor = new Fixed3x3(
                Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.MinIncrement, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.MinIncrement)
        };
        ScenarioBody<UnsupportedTestCollider3D> left =
            scenario.CreateBody(
                collider,
                Vector3d.Zero,
                FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Right);
        left.Collider.Material = PhysicsMaterial.Frictionless;
        right.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        pair.Manifold.SetContact(
            ContactAnchor.FromWorldPoint(
                left.Body.WorldCenterOfMass
                + Vector3d.Up * (Fixed64.MinIncrement * (Fixed64)3)),
            ContactAnchor.FromWorldPoint(right.Body.WorldCenterOfMass),
            Fixed64.Zero,
            Vector3d.Right);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: true,
            applyPositionCorrection: false);

        left.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        left.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        right.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        right.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse replacement)
            .Should()
            .BeTrue();
        replacement.NormalImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithUnrepresentableCompactWarmStartTorque_ShouldColdSolve()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Right);
        left.Collider.Material = PhysicsMaterial.Frictionless;
        right.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        pair.Manifold.SetContact(
            ContactAnchor.FromWorldPoint(
                left.Body.WorldCenterOfMass + Vector3d.Up * Fixed64.Two),
            ContactAnchor.FromWorldPoint(right.Body.WorldCenterOfMass),
            Fixed64.Zero,
            Vector3d.Right);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            contact.Normal,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: true,
            applyPositionCorrection: false);

        left.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        left.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        right.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        right.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(
                contact.ContactId,
                out ContactWarmStartImpulse replacement)
            .Should()
            .BeTrue();
        replacement.NormalImpulse.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CalculateImpulse_WithUnrepresentableFrictionCacheRemoval_ShouldRejectAtomically()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.One);
        left.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        right.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        left.Collider.Material = PhysicsMaterial.Frictionless;
        right.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        Vector3d normal = Vector3d.One.Normalized;
        pair.Manifold.SetContact(
            ContactAnchor.FromWorldPoint(left.Body.WorldCenterOfMass),
            ContactAnchor.FromWorldPoint(right.Body.WorldCenterOfMass),
            Fixed64.Zero,
            normal);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        pair.StoreWarmStartImpulse(
            contact.ContactId,
            normal,
            Fixed64.Zero,
            Fixed64.MaxValue,
            Fixed64.MaxValue);

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: false,
            applyPositionCorrection: false);

        left.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        right.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        pair.TryGetWarmStartImpulse(contact.ContactId, out _)
            .Should()
            .BeFalse();
    }

    private static ContactAnchor CreateExactParallelAnchor(
        Vector3d localOffset) =>
        new(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            Vector3d.Right * Fixed64.MinIncrement + localOffset);

    private static void SetExactContact(
        CollisionPair pair,
        LSCollider exactCollider,
        ContactAnchor exactAnchor,
        Fixed64 depth = default)
    {
        bool exactIsA = ReferenceEquals(pair.ColliderA, exactCollider);
        LSCollider otherCollider = exactIsA
            ? pair.ColliderB
            : pair.ColliderA;
        ContactAnchor otherAnchor = ContactAnchor.FromWorldPoint(
            otherCollider.Body?.WorldCenterOfMass
            ?? otherCollider.Center);
        Vector3d normal =
            (pair.ColliderB.Center - pair.ColliderA.Center).Normalized;
        pair.Manifold.SetContact(
            exactIsA ? exactAnchor : otherAnchor,
            exactIsA ? otherAnchor : exactAnchor,
            depth,
            normal);
    }

    private static (
        Vector3d LeftLinear,
        Vector3d LeftAngular,
        Vector3d RightLinear,
        Vector3d RightAngular)
        ResolveBoundaryContactWithPositionCorrection(
            bool seedInvalidWarmStart)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> left =
            scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> right =
            scenario.CreateSphere(Vector3d.Right);
        PhysicsMaterial rough =
            new(Fixed64.Half, Fixed64.Half, Fixed64.Zero);
        left.Collider.Material = rough;
        right.Collider.Material = rough;
        Push(
            left.Body,
            Vector3d.Right * StandardSpeed + Vector3d.Forward);
        CollisionPair pair =
            scenario.CreatePair(left.Collider, right.Collider);
        ContactAnchor boundaryAnchor = new(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            Vector3d.Right * (Fixed64.One + Fixed64.MinIncrement)
            + Vector3d.Up);
        SetExactContact(
            pair,
            right.Collider,
            boundaryAnchor,
            Fixed64.One);
        if (seedInvalidWarmStart)
        {
            ManifoldContact contact = pair.Manifold.PrimaryContact;
            pair.StoreWarmStartImpulse(
                contact.ContactId,
                contact.Normal,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.Zero);
        }

        CollisionResponse.CalculateImpulse(
            pair,
            applyCachedImpulse: seedInvalidWarmStart,
            applyPositionCorrection: true);

        return (
            left.Body.LinearVelocity,
            left.Body.AngularVelocity,
            right.Body.LinearVelocity,
            right.Body.AngularVelocity);
    }

    private static void Push(SolidBody body, Vector3d velocity) =>
        body.AddLinearImpulse(velocity * body.Mass);
}
