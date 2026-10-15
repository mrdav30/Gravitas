using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class BodyPositionDomainTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Correction3D_OutsideScalarDomain_ShouldThrowBeforeChangingPosition(bool negative)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody body = Create3D(context, Edge(negative));
        Vector3d before = body.Position3d;
        Action correct = () => body.ApplyCollisionPositionState(body.PrepareCollisionPositionCorrection(Vector3d.Right * Sign(negative)));

        correct.Should().Throw<OverflowException>();
        body.Position3d.Should().Be(before);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Correction2D_OutsideScalarDomain_ShouldThrowBeforeChangingPosition(bool negative)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = Create2D(context, Edge(negative));
        Vector2d before = body.Position;
        Action correct = () => body.ApplyCollisionPositionState(body.PrepareCollisionPositionCorrection(Vector2d.Right * Sign(negative)));

        correct.Should().Throw<OverflowException>();
        body.Position.Should().Be(before);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void Integration_OutsideScalarDomain_ShouldThrowBeforePublishingTruncatedPose(bool planar, bool negative)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody? spatial = planar ? null : Create3D(context, Edge(negative));
        SolidBody2D? flat = planar ? Create2D(context, Edge(negative)) : null;
        spatial?.ApplyCollisionLinearVelocityDelta(Vector3d.Right * Sign(negative));
        flat?.ApplyCollisionLinearVelocityDelta(Vector2d.Right * Sign(negative));
        context.Simulate();

        Action step = () => context.LateSimulate();

        step.Should().Throw<OverflowException>();
        (spatial?.Position3d.X ?? flat!.Position.X).Should().Be(Edge(negative));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PairedCorrection3D_WhenSecondEndpointOverflows_ShouldLeaveBothPositionsUnchanged(bool negative)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody first = Create3D(context, Edge(negative) - Sign(negative) * Fixed64.Quarter);
        SolidBody second = Create3D(context, Edge(negative));
        var pair = new CollisionPair(first.Collider, second.Collider);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold[0].Normal.Should().Be(Vector3d.Right * Sign(negative));
        pair.Manifold[0].Depth.Should().Be(Fixed64.FromFraction(3, 4));
        Vector3d beforeFirst = first.Position3d, beforeSecond = second.Position3d;

        Action solve = () => CollisionResponse.CalculateImpulse(pair);

        solve.Should().Throw<OverflowException>();
        first.Position3d.Should().Be(beforeFirst);
        second.Position3d.Should().Be(beforeSecond);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PairedCorrection2D_WhenSecondEndpointOverflows_ShouldLeaveBothPositionsUnchanged(bool negative)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D first = Create2D(context, Edge(negative) - Sign(negative) * Fixed64.Quarter);
        SolidBody2D second = Create2D(context, Edge(negative));
        var pair = new CollisionPair2D(first.Collider, second.Collider);
        CollisionDetection2D.TryCollide(pair.ColliderA, pair.ColliderB, out Contact2D contact).Should().BeTrue();
        contact.Normal.Should().Be(Vector2d.Right * Sign(negative));
        contact.Depth.Should().Be(Fixed64.FromFraction(3, 4));
        pair.Manifold.SetContact(contact.AnchorA, contact.AnchorB, contact.Depth, contact.Normal);
        Vector2d beforeFirst = first.Position, beforeSecond = second.Position;

        Action solve = () => CollisionResponse2D.Resolve(pair);

        solve.Should().Throw<OverflowException>();
        first.Position.Should().Be(beforeFirst);
        second.Position.Should().Be(beforeSecond);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PairedCorrectionMixed_WhenPlanarEndpointOverflows_ShouldLeaveBothPositionsUnchanged(bool negative)
    {
        using GravitasWorldContext context = CreateContext();
        context.Settings.Mixed2DHalfThickness = Fixed64.One;
        SolidBody first = Create3D(context, Edge(negative) - Sign(negative) * Fixed64.Quarter);
        SolidBody2D second = Create2D(context, Edge(negative));
        var pair = new CollisionPairMixed(first.Collider, second.Collider);
        CollisionDetectionMixed.TryCollide(first.Collider, second.Collider, out MixedContact contact).Should().BeTrue();
        contact.Normal3DTo2D.Should().Be(Vector3d.Right * Sign(negative));
        contact.Depth.Should().Be(Fixed64.FromFraction(3, 4));
        Vector3d beforeFirst = first.Position3d;
        Vector2d beforeSecond = second.Position;

        Action solve = () => CollisionResponseMixed.Resolve(pair, contact);

        solve.Should().Throw<OverflowException>();
        first.Position3d.Should().Be(beforeFirst);
        second.Position.Should().Be(beforeSecond);
    }

    [Theory]
    [InlineData(0, 0)]
    [InlineData(0, 1)]
    [InlineData(0, 2)]
    [InlineData(1, 0)]
    [InlineData(1, 1)]
    [InlineData(1, 2)]
    [InlineData(2, 0)]
    [InlineData(2, 1)]
    [InlineData(2, 2)]
    public void PairedCorrection_WithLargeCommonScale_ShouldApplyRepresentableWeightedMovement(int domain, int massCase)
    {
        using GravitasWorldContext context = CreateContext();
        Fixed64 radius = Fixed64.FromFraction(13, 10);
        Fixed64 mass = massCase == 1 ? Fixed64.MaxValue : massCase == 2 ? Fixed64.FromRaw(3) : Fixed64.One;
        SolidBody? first3D = domain == 1 ? null : Create3D(context, Fixed64.Zero, radius);
        SolidBody2D? first2D = domain == 1 ? Create2D(context, Fixed64.Zero, radius) : null;
        SolidBody? second3D = domain == 0 ? Create3D(context, Fixed64.Zero, radius) : null;
        SolidBody2D? second2D = domain == 0 ? null : Create2D(context, Fixed64.Zero, radius);
        if (first3D != null) first3D.Mass = mass;
        if (first2D != null) first2D.Mass = mass;
        if (second3D != null) second3D.Mass = mass;
        if (second2D != null) second2D.Mass = mass;
        Fixed64 depth = radius * Fixed64.Two;
        Fixed64 percent = domain == 1 ? CollisionResponse2D.PenetrationCorrectionPercent : CollisionResponse.PenetrationCorrectionPercent;
        Fixed64 expected = ((depth - CollisionResponse.PenetrationSlop) * percent) / Fixed64.Two;
        if (domain == 0)
        {
            var pair = new CollisionPair(first3D!.Collider, second3D!.Collider);
            pair.Manifold.SetContact(Vector3d.Zero, Vector3d.Zero, depth, Vector3d.Right);
            CollisionResponse.CalculateImpulse(pair);
        }
        else if (domain == 1)
        {
            var pair = new CollisionPair2D(first2D!.Collider, second2D!.Collider);
            pair.Manifold.SetContact(Vector2d.Zero, Vector2d.Zero, depth, Vector2d.Right);
            CollisionResponse2D.Resolve(pair);
        }
        else
        {
            var pair = new CollisionPairMixed(first3D!.Collider, second2D!.Collider);
            CollisionResponseMixed.Resolve(pair, new MixedContact(Vector3d.Zero, Vector3d.Zero, Vector3d.Right, depth));
        }

        (first3D?.Position3d.X ?? first2D!.Position.X).Should().Be(-expected);
        (second3D?.Position3d.X ?? second2D!.Position.X).Should().Be(expected);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void Handoff_OutsideScalarDomain_ShouldPreserveAuthoritativeState(bool planar, bool negative)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody? spatial = planar ? null : Create3D(context, Edge(negative));
        SolidBody2D? flat = planar ? Create2D(context, Edge(negative)) : null;
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        var before = context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);
        int count = spatial?.ContinuousCollisionTrajectoryCount ?? flat!.ContinuousCollisionTrajectoryCount;

        Action handoff = () =>
        {
            if (spatial != null)
                spatial.ApplyContinuousCollisionHandoff(spatial.Position3d, spatial.Rotation,
                    Vector3d.Right * Sign(negative), Vector3d.Zero, context.DeltaTime).Should().BeTrue();
            else
                flat!.ApplyContinuousCollisionHandoffState(flat.Position, flat.Rotation,
                    Vector2d.Right * Sign(negative), Fixed64.Zero, context.DeltaTime).Should().BeTrue();
        };

        handoff.Should().Throw<OverflowException>();
        (spatial?.ContinuousCollisionTrajectoryCount ?? flat!.ContinuousCollisionTrajectoryCount).Should().Be(count);
        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches).Should().Be(before);
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, true, false)]
    [InlineData(true, false, false)]
    [InlineData(true, true, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, true)]
    [InlineData(true, false, true)]
    [InlineData(true, true, true)]
    public void Handoff_ExactBoundaryOrFrozenMotion_ShouldRemainAccepted(bool planar, bool negative, bool frozen)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody? spatial = planar ? null : Create3D(context, Edge(negative));
        SolidBody2D? flat = planar ? Create2D(context, Edge(negative)) : null;
        Fixed64 velocity = Sign(negative) * (frozen ? Fixed64.One : Fixed64.Quarter);
        if (spatial != null)
        {
            if (frozen) spatial.FreezeAxes = BodyFreezeAxes3D.PositionX;
            spatial.ApplyContinuousCollisionHandoff(spatial.Position3d, spatial.Rotation,
                Vector3d.Right * velocity, Vector3d.Zero, context.DeltaTime).Should().BeTrue();
        }
        else
        {
            if (frozen) flat!.FreezeAxes = BodyFreezeAxes2D.PositionX;
            flat!.ApplyContinuousCollisionHandoffState(flat.Position, flat.Rotation,
                Vector2d.Right * velocity, Fixed64.Zero, context.DeltaTime).Should().BeTrue();
        }

        Fixed64 expected = frozen ? Edge(negative) : negative ? Fixed64.MinValue : Fixed64.MaxValue;
        (spatial?.ContinuousCollisionFrameEnd.X ?? flat!.ContinuousCollisionFrameEnd.X).Should().Be(expected);
        (spatial?.Position3d.X ?? flat!.Position.X).Should().Be(Edge(negative));
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    [InlineData(3, false)]
    [InlineData(3, true)]
    public void PairedCcdResponse_OutgoingTargetOutsideDomain_ShouldPreserveBothBodies(int domain, bool negative)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody? source3D = domain is 0 or 2 ? Create3D(context, Edge(negative) - Sign(negative)) : null;
        SolidBody2D? source2D = domain is 1 or 3 ? Create2D(context, Edge(negative) - Sign(negative)) : null;
        SolidBody? target3D = domain is 0 or 3 ? Create3D(context, Edge(negative)) : null;
        SolidBody2D? target2D = domain is 1 or 2 ? Create2D(context, Edge(negative)) : null;
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        context.Physics2D.PrepareContinuousCollisionFrame();
        source3D?.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4 * Sign(negative));
        source2D?.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64)4 * Sign(negative));
        var before = context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);
        Vector3d normal = Vector3d.Left * Sign(negative);

        Action respond = () =>
        {
            bool applied = domain switch
            {
                0 => source3D!.TryApplyContinuousCollisionDynamicResponse(target3D!, normal,
                    source3D.Position3d, target3D!.Position3d, Fixed64.Half, Fixed64.Half),
                1 => source2D!.TryApplyContinuousCollisionDynamicResponse(target2D!, normal.ToVector2d(),
                    source2D.Position, target2D!.Position, Fixed64.Half, Fixed64.Half),
                2 => source3D!.TryApplyContinuousCollisionMixed2DResponse(target2D!, normal,
                    source3D.Position3d, target2D!.Position, Fixed64.Half, Fixed64.Half),
                _ => source2D!.TryApplyContinuousCollisionMixed3DResponse(target3D!, normal.ToVector2d(),
                    source2D.Position, target3D!.Position3d, Fixed64.Half, Fixed64.Half)
            };
            applied.Should().BeTrue();
        };

        respond.Should().Throw<OverflowException>();
        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches).Should().Be(before);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Integration_HalfwayProduct_ShouldKeepMultiplyThenAddRounding(bool planar)
    {
        using GravitasWorldContext context = CreateContext();
        context.SetFrameRate(2);
        context.Environment.MinSpeed = Fixed64.Zero;
        SolidBody? spatial = planar ? null : Create3D(context, Fixed64.MinIncrement);
        SolidBody2D? flat = planar ? Create2D(context, Fixed64.MinIncrement) : null;
        Fixed64 velocity = Fixed64.One + Fixed64.MinIncrement;
        spatial?.ApplyCollisionLinearVelocityDelta(Vector3d.Right * velocity);
        flat?.ApplyCollisionLinearVelocityDelta(Vector2d.Right * velocity);

        context.Simulate();
        context.LateSimulate();

        (spatial?.Position3d.X ?? flat!.Position.X).Should().Be(Fixed64.Half + Fixed64.MinIncrement);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void Handoff_StationaryTailAtScalarBoundary_ShouldKeepMomentumWithoutOutOfDomainMotion(bool planar, bool shortTail)
    {
        using GravitasWorldContext context = CreateContext();
        context.Environment.MinSpeed = Fixed64.Zero;
        Fixed64 position = Fixed64.MaxValue - Fixed64.MinIncrement;
        Fixed64 velocity = shortTail ? Fixed64.One : Fixed64.Epsilon;
        Fixed64 remaining = shortTail ? Fixed64.Epsilon : Fixed64.One;
        SolidBody? spatial = planar ? null : Create3D(context, position);
        SolidBody2D? flat = planar ? Create2D(context, position) : null;

        if (spatial != null)
            spatial.ApplyContinuousCollisionHandoff(spatial.Position3d, spatial.Rotation,
                Vector3d.Right * velocity, Vector3d.Zero, remaining).Should().BeTrue();
        else
            flat!.ApplyContinuousCollisionHandoffState(flat.Position, flat.Rotation,
                Vector2d.Right * velocity, Fixed64.Zero, remaining).Should().BeTrue();

        (spatial?.ContinuousCollisionFrameEnd.X ?? flat!.ContinuousCollisionFrameEnd.X).Should().Be(position);
        (spatial?.Position3d.X ?? flat!.Position.X).Should().Be(position);
        (spatial?.LinearVelocity.X ?? flat!.LinearVelocity.X).Should().Be(planar && !shortTail ? Fixed64.Zero : velocity);
    }

    [Fact]
    public void Handoff3D_ClampedCommittedVelocity_ShouldValidateThePublishedTrajectory()
    {
        using GravitasWorldContext context = CreateContext();
        context.Environment.MaxSpeed = Fixed64.Quarter;
        SolidBody body = Create3D(context, Edge(false));

        body.ApplyContinuousCollisionHandoff(body.Position3d, body.Rotation,
            Vector3d.Right, Vector3d.Zero, context.DeltaTime).Should().BeTrue();

        body.LinearVelocity.Should().Be(Vector3d.Right * Fixed64.Quarter);
        body.ContinuousCollisionFrameEnd.X.Should().Be(Fixed64.MaxValue);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Handoff3D_UnchangedVelocityAfterLimitChange_ShouldPreserveTheNoRefreshContract(bool minimum)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody body = Create3D(context, Edge(false));
        body.ApplyCollisionLinearVelocityDelta(Vector3d.Right);
        if (minimum) context.Environment.MinSpeed = Fixed64.Two;
        else context.Environment.MaxSpeed = Fixed64.Quarter;
        var before = context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        Action handoff = () => body.ApplyContinuousCollisionHandoff(body.Position3d, body.Rotation,
            body.LinearVelocity, body.AngularVelocity, context.DeltaTime);

        handoff.Should().Throw<OverflowException>();
        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches).Should().Be(before);
    }

    [Fact]
    public void PairedCcdResponse3D_ClampedSourceVelocity_ShouldValidateThePublishedSourceTrajectory()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody source = Create3D(context, Edge(true));
        SolidBody target = Create3D(context, Fixed64.MinValue + Fixed64.One);
        target.FreezeAxes = BodyFreezeAxes3D.All;
        context.AdvanceLateSimulateToken();
        context.Physics.PrepareContinuousCollisionFrame();
        source.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4);
        context.Environment.MaxSpeed = Fixed64.Quarter;

        source.TryApplyContinuousCollisionDynamicResponse(target, Vector3d.Left,
            source.Position3d, target.Position3d, Fixed64.Zero, Fixed64.One).Should().BeTrue();

        source.LinearVelocity.Should().Be(Vector3d.Left * Fixed64.Quarter);
        source.ContinuousCollisionFrameEnd.X.Should().Be(Fixed64.MinValue);
        target.LinearVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void PairedCcdResponse2D_SubthresholdSourceVelocity_ShouldPublishAStationarySourceTrajectory()
    {
        using GravitasWorldContext context = CreateContext();
        Fixed64 sourceX = Fixed64.MinValue + Fixed64.MinIncrement;
        SolidBody2D source = Create2D(context, sourceX);
        SolidBody2D target = Create2D(context, Fixed64.MinValue + Fixed64.One);
        target.FreezeAxes = BodyFreezeAxes2D.All;
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * (Fixed64.Epsilon * Fixed64.Two));

        source.TryApplyContinuousCollisionDynamicResponse(target, Vector2d.Left,
            source.Position, target.Position, Fixed64.Zero, Fixed64.One).Should().BeTrue();

        source.LinearVelocity.Should().Be(Vector2d.Zero);
        source.ContinuousCollisionFrameEnd.X.Should().Be(sourceX);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void RotationalResponse2D_RoundedRemainingDuration_ShouldPreflightTheActualSourceTail()
    {
        using GravitasWorldContext context = CreateContext();
        Fixed64 elapsed = Fixed64.FromRaw(3);
        Fixed64 remaining = context.DeltaTime - elapsed;
        Fixed64 sourceX = Fixed64.FromRaw(Fixed64.MaxValue.m_rawValue - 2147483646L);
        SolidBody2D source = Create2D(context, sourceX);
        SolidBody2D target = Create2D(context, sourceX - Fixed64.Two);
        source.Collider.Material = target.Collider.Material = PhysicsMaterial.Frictionless;
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();
        source.ApplyCollisionLinearVelocityDelta(Vector2d.Right * Fixed64.Two);
        var before = context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);
        var contact = new Contact2D(source.Position, target.Position, Vector2d.Right, Fixed64.Zero);

        Action respond = () => source.TryApplyRotationalContinuousCollisionResponse(target, contact,
            Fixed64.Half, source.Position, source.Rotation, Vector2d.Zero, Fixed64.Zero, elapsed, remaining);

        respond.Should().Throw<OverflowException>();
        context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches).Should().Be(before);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PairedCorrection2D_BodylessParticipant_ShouldReceiveNoMovement(bool reverse)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D? actor = reverse ? Create2D(context, Fixed64.Half) : null;
        var fixedCollider = new LSCircleCollider2D(Fixed64.Half);
        fixedCollider.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        actor ??= Create2D(context, Fixed64.Half);
        var pair = reverse ? new CollisionPair2D(actor.Collider, fixedCollider)
            : new CollisionPair2D(fixedCollider, actor.Collider);
        pair.ColliderA.Should().Be(reverse ? actor.Collider : fixedCollider);
        Vector2d normal = reverse ? Vector2d.Left : Vector2d.Right;
        pair.Manifold.SetContact(reverse ? Vector2d.Zero : Vector2d.Right * Fixed64.Half,
            reverse ? Vector2d.Right * Fixed64.Half : Vector2d.Zero, Fixed64.Half, normal);

        CollisionResponse2D.Resolve(pair);

        actor.Position.X.Should().Be(Fixed64.One - CollisionResponse2D.PenetrationSlop);
        fixedCollider.Center.Should().Be(Vector2d.Zero);
        fixedCollider.Body.Should().BeNull();
    }

    [Fact]
    public void PairedCcdResponse2D_FrozenSourceAxis_ShouldKeepSourceStillAndReflectTheTarget()
    {
        using GravitasWorldContext context = CreateContext();
        Fixed64 sourceX = Fixed64.MaxValue - Fixed64.Two;
        SolidBody2D source = Create2D(context, sourceX);
        SolidBody2D target = Create2D(context, sourceX - Fixed64.Half);
        source.FreezeAxes = BodyFreezeAxes2D.PositionX;
        target.ApplyCollisionLinearVelocityDelta(Vector2d.Right);
        context.AdvanceLateSimulateToken();
        context.Physics2D.PrepareContinuousCollisionFrame();

        source.TryApplyContinuousCollisionDynamicResponse(target, Vector2d.Right,
            source.Position, target.Position, Fixed64.Zero, Fixed64.One).Should().BeTrue();

        source.Position.X.Should().Be(sourceX);
        source.LinearVelocity.Should().Be(Vector2d.Zero);
        source.ContinuousCollisionFrameEnd.X.Should().Be(sourceX);
        target.LinearVelocity.X.Should().Be(-Fixed64.Half);
        target.ContinuousCollisionFrameEnd.X.Should().Be(sourceX - Fixed64.One);
    }

    [Fact]
    public void PairedCorrection2D_UnrepresentableCompactScale_ShouldKeepTheExactWeightedDistance()
    {
        using GravitasWorldContext context = CreateContext();
        Fixed64 radius = (Fixed64)4095;
        var fixedCollider = new LSCircleCollider2D(radius);
        fixedCollider.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        var actor = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)),
            new LSCircleCollider2D(radius)) { Mass = Fixed64.MaxValue, SleepEnabled = false };
        actor.Initialize(Vector2d.Zero);
        Fixed64 depth = radius * Fixed64.Two;
        var pair = new CollisionPair2D(fixedCollider, actor.Collider);
        pair.Manifold.SetContact(Vector2d.Right * radius, Vector2d.Left * radius, depth, Vector2d.Right);

        CollisionResponse2D.Resolve(pair);

        actor.Position.X.Should().Be(depth - CollisionResponse2D.PenetrationSlop);
        fixedCollider.Center.Should().Be(Vector2d.Zero);
    }

    private static Fixed64 Sign(bool negative) => negative ? -Fixed64.One : Fixed64.One;
    private static Fixed64 Edge(bool negative) => negative ? Fixed64.MinValue + Fixed64.Quarter : Fixed64.MaxValue - Fixed64.Quarter;

    private static GravitasWorldContext CreateContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.SetFrameRate(1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Environment.MaxSpeed = Fixed64.MaxValue;
        return context;
    }

    private static SolidBody Create3D(GravitasWorldContext context, Fixed64 x, Fixed64? radius = null)
    {
        Vector3d position = Vector3d.Right * x;
        var body = new SolidBody(new TestMatterAgent(context, new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One)),
            new LSSphereCollider { Radius = radius ?? Fixed64.Half }) { Mass = Fixed64.One, SleepEnabled = false };
        body.Initialize(position, FixedQuaternion.Identity);
        return body;
    }

    private static SolidBody2D Create2D(GravitasWorldContext context, Fixed64 x, Fixed64? radius = null)
    {
        Vector2d position = Vector2d.Right * x;
        var body = new SolidBody2D(new TestMatterAgent(context, new FixedTransform(position.ToVector3d(Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One)),
            new LSCircleCollider2D(radius ?? Fixed64.Half)) { Mass = Fixed64.One, SleepEnabled = false };
        body.Initialize(position);
        return body;
    }
}
