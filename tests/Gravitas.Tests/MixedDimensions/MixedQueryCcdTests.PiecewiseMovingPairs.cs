using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using System.Reflection;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void MixedContinuousMode_AnalyticSphereCircle_WithLongSourceSweep_ShouldPreserveOneRawContactDistance(
        bool sphereIsSource)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        Fixed64 oneRaw = Fixed64.MinIncrement;
        Fixed64 sourceLength = (Fixed64)1_000_000;
        Fixed64 sourceStart = -Fixed64.One - oneRaw;

        if (sphereIsSource)
        {
            ScenarioBody<LSSphereCollider> source = CreateSphere3D(
                context,
                new Vector3d(sourceStart, Fixed64.Zero, Fixed64.Zero));
            SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
            source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

            context.AdvanceLateSimulateToken();
            context.Physics.PrepareContinuousCollisionFrame();
            context.Physics2D.PrepareContinuousCollisionFrame();
            object?[] arguments =
            {
                target,
                new Vector3d(sourceStart, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right * sourceLength,
                Fixed64.Half,
                target.ResolveMixedContinuousCollisionProxyRadius(),
                sourceLength,
                Fixed64.Zero,
                null
            };
            ContinuousCollisionMath.IntervalSearchStatus status =
                (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody)
                    .GetMethod(
                        "TryGetDynamicMixed2DContinuousCollisionHit",
                        BindingFlags.Instance | BindingFlags.NonPublic)!
                    .Invoke(source.Body, arguments)!;
            var hit = (DynamicMixedIntervalHit)arguments[7]!;

            status.Should().Be(ContinuousCollisionMath.IntervalSearchStatus.ExactHit);
            hit.ExactHit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
            hit.ExactHit.Distance.Should().Be(oneRaw);
            hit.SafeDistance.Should().Be(oneRaw);
            return;
        }

        SolidBody2D source2D = CreateCircle2D(
            context,
            new Vector2d(sourceStart, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target3D = CreateSphere3D(
            context,
            Vector3d.Zero);
        source2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        object?[] arguments2D =
        {
            target3D.Body,
            new Vector3d(sourceStart, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right * sourceLength,
            source2D.ResolveMixedContinuousCollisionProxyRadius(),
            sourceLength,
            Fixed64.Zero,
            null
        };
        ContinuousCollisionMath.IntervalSearchStatus status2D =
            (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody2D)
                .GetMethod(
                    "TryGetDynamicMixed3DContinuousCollisionHit",
                    BindingFlags.Instance | BindingFlags.NonPublic)!
                .Invoke(source2D, arguments2D)!;
        var hit2D = (DynamicMixedIntervalHit)arguments2D[6]!;

        status2D.Should().Be(ContinuousCollisionMath.IntervalSearchStatus.ExactHit);
        hit2D.ExactHit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
        hit2D.ExactHit.Distance.Should().Be(oneRaw);
        hit2D.SafeDistance.Should().Be(oneRaw);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void MixedContinuousMode_RotatingOffsetRadialSource_ShouldDeferToRotationalSearch(
        bool sphereIsSource)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        Vector3d sourceStart = -Vector3d.Right * (Fixed64)4;
        Vector3d sourceDisplacement = Vector3d.Right * Fixed64.Two;

        if (sphereIsSource)
        {
            ScenarioBody<LSSphereCollider> source = CreateSphere3D(
                context,
                sourceStart,
                isKinematic: true);
            SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
            source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            source.Collider.LocalOffset = Vector3d.Right * Fixed64.Two;
            source.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
            source.Body.Agent.Transform.LocalPosition =
                sourceStart + sourceDisplacement;
            source.Body.Agent.Transform.LocalRotation =
                FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Pi);

            context.AdvanceLateSimulateToken();
            context.Physics.PrepareContinuousCollisionFrame();
            context.Physics2D.PrepareContinuousCollisionFrame();
            object?[] arguments =
            {
                target,
                sourceStart,
                sourceDisplacement,
                Fixed64.FromFraction(5, 2),
                target.ResolveMixedContinuousCollisionProxyRadius(),
                Fixed64.Two,
                Fixed64.Zero,
                null
            };
            var status = (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody)
                .GetMethod(
                    "TryGetDynamicMixed2DContinuousCollisionHit",
                    BindingFlags.Instance | BindingFlags.NonPublic)!
                .Invoke(source.Body, arguments)!;

            status.Should().NotBe(ContinuousCollisionMath.IntervalSearchStatus.ExactHit);
            return;
        }

        SolidBody2D source2D = CreateCircle2D(
            context,
            sourceStart.ToVector2d(),
            isKinematic: true);
        ScenarioBody<LSSphereCollider> target3D =
            CreateSphere3D(context, Vector3d.Zero);
        source2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source2D.Collider.LocalOffset = Vector2d.Right * Fixed64.Two;
        source2D.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        source2D.Agent.Transform.LocalPosition =
            sourceStart + sourceDisplacement;
        source2D.Agent.Transform.LocalRotationXZRadians = Fixed64.Pi;

        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        object?[] arguments2D =
        {
            target3D.Body,
            sourceStart,
            sourceDisplacement,
            source2D.ResolveMixedContinuousCollisionProxyRadius(),
            Fixed64.Two,
            Fixed64.Zero,
            null
        };
        var status2D = (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody2D)
            .GetMethod(
                "TryGetDynamicMixed3DContinuousCollisionHit",
                BindingFlags.Instance | BindingFlags.NonPublic)!
            .Invoke(source2D, arguments2D)!;

        status2D.Should().NotBe(ContinuousCollisionMath.IntervalSearchStatus.ExactHit);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void MixedContinuousMode_RotatingOffsetRadialTarget_ShouldDeferToRotationalSearch(
        bool sphereIsSource)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        Vector3d sourceStart = -Vector3d.Right * (Fixed64)4;
        Vector3d sourceDisplacement = Vector3d.Right * Fixed64.Two;

        if (sphereIsSource)
        {
            ScenarioBody<LSSphereCollider> source = CreateSphere3D(
                context,
                sourceStart);
            SolidBody2D target = CreateCircle2D(
                context,
                Vector2d.Zero,
                isKinematic: true);
            source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            target.Collider.LocalOffset = Vector2d.Right * Fixed64.Two;
            target.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
            target.Agent.Transform.LocalRotationXZRadians = Fixed64.Pi;

            context.AdvanceLateSimulateToken();
            context.Physics.PrepareContinuousCollisionFrame();
            context.Physics2D.PrepareContinuousCollisionFrame();
            object?[] arguments =
            {
                target,
                sourceStart,
                sourceDisplacement,
                Fixed64.Half,
                target.ResolveMixedContinuousCollisionProxyRadius(),
                Fixed64.Two,
                Fixed64.Zero,
                null
            };
            var status = (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody)
                .GetMethod(
                    "TryGetDynamicMixed2DContinuousCollisionHit",
                    BindingFlags.Instance | BindingFlags.NonPublic)!
                .Invoke(source.Body, arguments)!;

            status.Should().NotBe(ContinuousCollisionMath.IntervalSearchStatus.ExactHit);
            return;
        }

        SolidBody2D source2D = CreateCircle2D(
            context,
            sourceStart.ToVector2d());
        ScenarioBody<LSSphereCollider> target3D = CreateSphere3D(
            context,
            Vector3d.Zero,
            isKinematic: true);
        source2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target3D.Collider.LocalOffset = Vector3d.Right * Fixed64.Two;
        target3D.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        target3D.Body.Agent.Transform.LocalRotation =
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Pi);

        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        object?[] arguments2D =
        {
            target3D.Body,
            sourceStart,
            sourceDisplacement,
            source2D.ResolveMixedContinuousCollisionProxyRadius(),
            Fixed64.Two,
            Fixed64.Zero,
            null
        };
        var status2D = (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody2D)
            .GetMethod(
                "TryGetDynamicMixed3DContinuousCollisionHit",
                BindingFlags.Instance | BindingFlags.NonPublic)!
            .Invoke(source2D, arguments2D)!;

        status2D.Should().NotBe(ContinuousCollisionMath.IntervalSearchStatus.ExactHit);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void MixedContinuousMode_RoundedSlabRimOutsidePlanarProxySphere_WithCenteredRotation_ShouldUseExactContact(
        bool sphereIsSource)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        Vector3d relativeStart = new(
            Fixed64.FromFraction(19, 10),
            Fixed64.FromFraction(8, 5),
            Fixed64.Zero);
        Vector3d relativeDisplacement = new(
            -Fixed64.One,
            -Fixed64.One,
            Fixed64.Zero);

        if (sphereIsSource)
        {
            ScenarioBody<LSSphereCollider> source = CreateSphere3D(
                context,
                relativeStart,
                isKinematic: true);
            SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
            source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            source.Body.Agent.Transform.LocalRotation =
                FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Pi);
            target.ApplyCollisionAngularVelocityDelta(Fixed64.Pi);

            context.AdvanceLateSimulateToken();
            context.Physics.PrepareContinuousCollisionFrame();
            context.Physics2D.PrepareContinuousCollisionFrame();
            Fixed64 sourceLength = relativeDisplacement.Magnitude;
            object?[] arguments =
            {
                target,
                relativeStart,
                relativeDisplacement,
                Fixed64.Half,
                target.ResolveMixedContinuousCollisionProxyRadius(),
                sourceLength,
                Fixed64.Zero,
                null
            };
            var status = (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody)
                .GetMethod(
                    "TryGetDynamicMixed2DContinuousCollisionHit",
                    BindingFlags.Instance | BindingFlags.NonPublic)!
                .Invoke(source.Body, arguments)!;

            status.Should().Be(ContinuousCollisionMath.IntervalSearchStatus.ExactHit);
            ((DynamicMixedIntervalHit)arguments[7]!).ExactHit.ReducerKind
                .Should().Be(PhysicsQueryReducerKind.Exact);
            return;
        }

        SolidBody2D source2D = CreateCircle2D(
            context,
            Vector2d.Zero,
            isKinematic: true);
        ScenarioBody<LSSphereCollider> target3D = CreateSphere3D(
            context,
            relativeStart);
        source2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source2D.Agent.Transform.LocalRotationXZRadians = Fixed64.Pi;
        target3D.Body.ApplyCollisionAngularVelocityDelta(
            Vector3d.Up * Fixed64.Pi);
        target3D.Body.ApplyCollisionLinearVelocityDelta(-Vector3d.Up);

        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target3D.Body.ContinuousCollisionFrameDisplacement
            .Should().Be(-Vector3d.Up);
        object?[] arguments2D =
        {
            target3D.Body,
            Vector3d.Zero,
            Vector3d.Right,
            source2D.ResolveMixedContinuousCollisionProxyRadius(),
            Fixed64.One,
            Fixed64.Zero,
            null
        };
        var status2D = (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody2D)
            .GetMethod(
                "TryGetDynamicMixed3DContinuousCollisionHit",
                BindingFlags.Instance | BindingFlags.NonPublic)!
            .Invoke(source2D, arguments2D)!;

        status2D.Should().Be(ContinuousCollisionMath.IntervalSearchStatus.ExactHit);
        ((DynamicMixedIntervalHit)arguments2D[6]!).ExactHit.ReducerKind
            .Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void MixedContinuousMode_ExactRadialRetry_ShouldPreserveAnalyticalSourceDistance(
        bool sphereIsSource)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        Fixed64 separatedStart = -Fixed64.Two - Fixed64.MinIncrement;

        if (sphereIsSource)
        {
            ScenarioBody<LSSphereCollider> source = CreateSphere3D(
                context,
                new Vector3d(separatedStart, Fixed64.Zero, Fixed64.Zero));
            SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
            source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            target.ApplyCollisionLinearVelocityDelta(-Vector2d.Right);

            context.AdvanceLateSimulateToken();
            context.Physics.PrepareContinuousCollisionFrame();
            context.Physics2D.PrepareContinuousCollisionFrame();
            object?[] arguments =
            {
                target,
                new Vector3d(separatedStart, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right,
                Fixed64.Half,
                target.ResolveMixedContinuousCollisionProxyRadius(),
                Fixed64.One,
                Fixed64.Zero,
                null
            };
            var status = (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody)
                .GetMethod(
                    "TryGetDynamicMixed2DContinuousCollisionHit",
                    BindingFlags.Instance | BindingFlags.NonPublic)!
                .Invoke(source.Body, arguments)!;
            var hit = (DynamicMixedIntervalHit)arguments[7]!;

            status.Should().Be(ContinuousCollisionMath.IntervalSearchStatus.ExactHit);
            hit.ExactHit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
            hit.ExactHit.Distance.Should().Be(Fixed64.Half);
            hit.SafeDistance.Should().Be(Fixed64.Half);
            return;
        }

        SolidBody2D source2D = CreateCircle2D(context, Vector2d.Zero);
        ScenarioBody<LSSphereCollider> target3D = CreateSphere3D(
            context,
            new Vector3d(separatedStart, Fixed64.Zero, Fixed64.Zero));
        source2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right);

        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        object?[] arguments2D =
        {
            target3D.Body,
            Vector3d.Zero,
            -Vector3d.Right,
            source2D.ResolveMixedContinuousCollisionProxyRadius(),
            Fixed64.One,
            Fixed64.Zero,
            null
        };
        var status2D = (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody2D)
            .GetMethod(
                "TryGetDynamicMixed3DContinuousCollisionHit",
                BindingFlags.Instance | BindingFlags.NonPublic)!
            .Invoke(source2D, arguments2D)!;
        var hit2D = (DynamicMixedIntervalHit)arguments2D[6]!;

        status2D.Should().Be(ContinuousCollisionMath.IntervalSearchStatus.ExactHit);
        hit2D.ExactHit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
        hit2D.ExactHit.Distance.Should().Be(Fixed64.Half);
        hit2D.SafeDistance.Should().Be(Fixed64.Half);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void MixedContinuousMode_ProxyRadiallySeparatingButSlabClosing_ShouldUseExactContact(
        bool sphereIsSource)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        Vector3d sphereStart = new(
            Fixed64.One,
            Fixed64.Two,
            Fixed64.Zero);
        Vector3d relativeDisplacement = new(
            (Fixed64)5,
            Fixed64.FromFraction(-3, 2),
            Fixed64.Zero);

        if (sphereIsSource)
        {
            ScenarioBody<LSSphereCollider> source = CreateSphere3D(
                context,
                sphereStart);
            SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
            source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            ((LSCircleCollider2D)target.Collider).Radius = (Fixed64)10;
            target.Collider.MixedHalfThicknessOverride = Fixed64.FromFraction(1, 10);
            target.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();

            context.AdvanceLateSimulateToken();
            context.Physics.PrepareContinuousCollisionFrame();
            context.Physics2D.PrepareContinuousCollisionFrame();
            Fixed64 sourceLength = relativeDisplacement.Magnitude;
            object?[] arguments =
            {
                target,
                sphereStart,
                relativeDisplacement,
                Fixed64.Half,
                target.ResolveMixedContinuousCollisionProxyRadius(),
                sourceLength,
                Fixed64.Zero,
                null
            };
            var status = (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody)
                .GetMethod(
                    "TryGetDynamicMixed2DContinuousCollisionHit",
                    BindingFlags.Instance | BindingFlags.NonPublic)!
                .Invoke(source.Body, arguments)!;

            status.Should().Be(ContinuousCollisionMath.IntervalSearchStatus.ExactHit);
            ((DynamicMixedIntervalHit)arguments[7]!).ExactHit.ReducerKind
                .Should().Be(PhysicsQueryReducerKind.Exact);
            return;
        }

        SolidBody2D source2D = CreateCircle2D(context, Vector2d.Zero);
        ScenarioBody<LSSphereCollider> target3D = CreateSphere3D(
            context,
            sphereStart);
        source2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        ((LSCircleCollider2D)source2D.Collider).Radius = (Fixed64)10;
        source2D.Collider.MixedHalfThicknessOverride = Fixed64.FromFraction(1, 10);
        source2D.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        target3D.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Down * Fixed64.FromFraction(3, 2));

        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        object?[] arguments2D =
        {
            target3D.Body,
            Vector3d.Zero,
            -Vector3d.Right * (Fixed64)5,
            source2D.ResolveMixedContinuousCollisionProxyRadius(),
            (Fixed64)5,
            Fixed64.Zero,
            null
        };
        var status2D = (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody2D)
            .GetMethod(
                "TryGetDynamicMixed3DContinuousCollisionHit",
                BindingFlags.Instance | BindingFlags.NonPublic)!
            .Invoke(source2D, arguments2D)!;

        status2D.Should().Be(ContinuousCollisionMath.IntervalSearchStatus.ExactHit);
        ((DynamicMixedIntervalHit)arguments2D[6]!).ExactHit.ReducerKind
            .Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void MixedContinuousMode_AnalyticSphereCircle_WhenSourceStartsMidTargetSegment_ShouldClipTargetDisplacement(
        bool sphereIsSource)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;

        // The target moves six units over the frame, but the source query only
        // covers the final half. The exact solve must use the clipped
        // three-unit target displacement and stop at the shared x=1.75 contact.
        if (sphereIsSource)
        {
            ScenarioBody<LSSphereCollider> source = CreateSphere3D(
                context,
                Vector3d.Zero);
            SolidBody2D target = CreateCircle2D(
                context,
                new Vector2d(Fixed64.FromFraction(-7, 4), Fixed64.Zero));
            source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            target.AddLinearImpulse(Vector2d.Right * (Fixed64)6);

            context.AdvanceLateSimulateToken();
            context.Physics.PrepareContinuousCollisionFrame();
            context.Physics2D.PrepareContinuousCollisionFrame();
            object?[] arguments =
            {
                target,
                Vector3d.Zero,
                Vector3d.Right * Fixed64.FromFraction(7, 2),
                Fixed64.Half,
                target.ResolveMixedContinuousCollisionProxyRadius(),
                Fixed64.FromFraction(7, 2),
                Fixed64.Half,
                null
            };
            ContinuousCollisionMath.IntervalSearchStatus status =
                (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody)
                    .GetMethod(
                        "TryGetDynamicMixed2DContinuousCollisionHit",
                        BindingFlags.Instance | BindingFlags.NonPublic)!
                    .Invoke(source.Body, arguments)!;
            var hit = (DynamicMixedIntervalHit)arguments[7]!;

            status.Should().Be(ContinuousCollisionMath.IntervalSearchStatus.ExactHit);
            hit.ExactHit.Distance.Should().Be(Fixed64.FromFraction(7, 4));
            return;
        }

        SolidBody2D source2D = CreateCircle2D(context, Vector2d.Zero);
        ScenarioBody<LSSphereCollider> target3D = CreateSphere3D(
            context,
            new Vector3d(
                Fixed64.FromFraction(-7, 4),
                Fixed64.Zero,
                Fixed64.Zero));
        source2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target3D.Body.AddLinearImpulse(Vector3d.Right * (Fixed64)6);

        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        object?[] arguments2D =
        {
            target3D.Body,
            Vector3d.Zero,
            Vector3d.Right * Fixed64.FromFraction(7, 2),
            source2D.ResolveMixedContinuousCollisionProxyRadius(),
            Fixed64.FromFraction(7, 2),
            Fixed64.Half,
            null
        };
        ContinuousCollisionMath.IntervalSearchStatus status2D =
            (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody2D)
                .GetMethod(
                    "TryGetDynamicMixed3DContinuousCollisionHit",
                    BindingFlags.Instance | BindingFlags.NonPublic)!
                .Invoke(source2D, arguments2D)!;
        var hit2D = (DynamicMixedIntervalHit)arguments2D[6]!;

        status2D.Should().Be(ContinuousCollisionMath.IntervalSearchStatus.ExactHit);
        hit2D.ExactHit.Distance.Should().Be(Fixed64.FromFraction(7, 4));
    }

    [Fact]
    public void MixedContinuousMode_Exact3DSphereContactAtSegmentEnd_ShouldUseSeparating2DSuccessor()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        Vector3d sourceStart = new(
            Fixed64.FromFraction(4, 5),
            Fixed64.FromFraction(3, 2),
            Fixed64.Zero);
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(context, sourceStart);
        SolidBody2D target = CreateCircle2D(context, Vector2d.Zero);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Collider.MixedHalfThicknessOverride = Fixed64.FromFraction(1, 10);
        target.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();

        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.ApplyContinuousCollisionHandoffState(
                Vector2d.Zero,
                Fixed64.Zero,
                -Vector2d.Right * (Fixed64)6,
                Fixed64.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();
        object?[] arguments =
        {
            target,
            sourceStart,
            -Vector3d.Up * Fixed64.Two,
            Fixed64.Half,
            target.ResolveMixedContinuousCollisionProxyRadius(),
            Fixed64.Two,
            Fixed64.Zero,
            null
        };
        var status = (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody)
            .GetMethod(
                "TryGetDynamicMixed2DContinuousCollisionHit",
                BindingFlags.Instance | BindingFlags.NonPublic)!
            .Invoke(source.Body, arguments)!;

        status.Should().Be(
            ContinuousCollisionMath.IntervalSearchStatus.CertifiedNoHit);
    }

    [Fact]
    public void MixedContinuousMode_Exact2DCircleContactAtSegmentEnd_ShouldUseSeparating3DSuccessor()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        Vector2d sourceStart2D = -Vector2d.Right;
        SolidBody2D source = CreateCircle2D(context, sourceStart2D);
        source.Collider.MixedHalfThicknessOverride = Fixed64.FromFraction(1, 10);
        source.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(
            context,
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(-8, 5),
                Fixed64.Zero));
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Up * Fixed64.Two);

        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                new Vector3d(
                    Fixed64.Zero,
                    Fixed64.FromFraction(-3, 5),
                    Fixed64.Zero),
                FixedQuaternion.Identity,
                -Vector3d.Up * Fixed64.Two,
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();
        object?[] arguments =
        {
            target.Body,
            sourceStart2D.ToVector3d(Fixed64.Zero),
            Vector3d.Right * Fixed64.Two,
            source.ResolveMixedContinuousCollisionProxyRadius(),
            Fixed64.Two,
            Fixed64.Zero,
            null
        };
        var status = (ContinuousCollisionMath.IntervalSearchStatus)typeof(SolidBody2D)
            .GetMethod(
                "TryGetDynamicMixed3DContinuousCollisionHit",
                BindingFlags.Instance | BindingFlags.NonPublic)!
            .Invoke(source, arguments)!;

        status.Should().Be(
            ContinuousCollisionMath.IntervalSearchStatus.CertifiedNoHit);
    }

    [Fact]
    public void MixedContinuousMode_PiecewiseOutAndReturn3DTarget_ShouldBlock2DSource()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 4;
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-5), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3));
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)10);
        target.Body.ApplyCollisionLinearVelocityDelta(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)6),
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        source.Position.X.Should().BeLessThan((Fixed64)5);
    }

    [Fact]
    public void MixedContinuousMode_PiecewiseOutAndReturn2DTarget_ShouldBlock3DSource()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 4;
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D target = CreateCircle2D(
            context,
            new Vector2d(Fixed64.Zero, (Fixed64)3));
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)10);
        target.ApplyCollisionLinearVelocityDelta(new Vector2d(Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.ApplyContinuousCollisionHandoffState(
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d(Fixed64.Zero, (Fixed64)6),
                Fixed64.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.Body.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        source.Body.Position3d.X.Should().BeLessThan((Fixed64)5);
    }

    [Fact]
    public void MixedContinuousMode_PiecewiseOutAndReturn2DTarget_ShouldReceive3DKinematicHandoff()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        SolidBody2D target = CreateCircle2D(
            context,
            new Vector2d(Fixed64.Zero, (Fixed64)3));
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.Agent.Transform.LocalPosition = new Vector3d(
            (Fixed64)5,
            Fixed64.Zero,
            Fixed64.Zero);
        target.ApplyCollisionLinearVelocityDelta(
            new Vector2d(Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.ApplyContinuousCollisionHandoffState(
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d(Fixed64.Zero, (Fixed64)6),
                Fixed64.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.Body.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void MixedContinuousMode_PiecewiseOutAndReturn3DTarget_ShouldReceive2DKinematicHandoff()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)3));
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-5), Fixed64.Zero),
            isKinematic: true);
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Agent.Transform.LocalPosition = new Vector3d(
            (Fixed64)5,
            Fixed64.Zero,
            Fixed64.Zero);
        target.Body.ApplyCollisionLinearVelocityDelta(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)6),
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void MixedContinuousMode_3DTargetReversingAtTouchBoundary_ShouldUseSeparatingSegment()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        SolidBody2D source = CreateCircle2D(
            context,
            new Vector2d((Fixed64)(-3), Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)4));
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)6);
        target.Body.ApplyCollisionLinearVelocityDelta(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
                FixedQuaternion.Identity,
                new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)6),
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.Position.X.Should().Be((Fixed64)3);
    }

    [Fact]
    public void MixedContinuousMode_2DTargetReversingAtTouchBoundary_ShouldUseSeparatingSegment()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero));
        SolidBody2D target = CreateCircle2D(
            context,
            new Vector2d(Fixed64.Zero, (Fixed64)4));
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)6);
        target.ApplyCollisionLinearVelocityDelta(
            new Vector2d(Fixed64.Zero, (Fixed64)(-6)));
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        target.ApplyContinuousCollisionHandoffState(
                new Vector2d(Fixed64.Zero, Fixed64.One),
                Fixed64.Zero,
                new Vector2d(Fixed64.Zero, (Fixed64)6),
                Fixed64.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.Body.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.Body.Position3d.X.Should().Be((Fixed64)3);
    }
}
