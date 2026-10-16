using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class SleepingQueryGroundingParityTests
{
    public static TheoryData<PhysicsRuntimeMode, bool, int> InvalidatedSupports
    {
        get
        {
            var cases = new TheoryData<PhysicsRuntimeMode, bool, int>();
            foreach (PhysicsRuntimeMode mode in new[] { PhysicsRuntimeMode.TwoD, PhysicsRuntimeMode.Mixed })
            foreach (bool bodyless in new[] { false, true })
            foreach (int change in new[] { 0, 1, 2, 3, 4, 5 })
                cases.Add(mode, bodyless, change);
            return cases;
        }
    }

    [Theory]
    [MemberData(nameof(InvalidatedSupports))]
    public void InvalidatedQueryOnlySupport_ShouldWakeThePlanarSleeperAndResumeGravity(
        PhysicsRuntimeMode mode, bool bodyless, int change)
    {
        using var fixture = new QuerySupportScene(mode, bodyless);
        fixture.AssertSleepingQuerySupport();
        Vector2d position = fixture.Body.Position;

        fixture.InvalidateSupport(change);
        for (int frame = 0; frame < 8; frame++) fixture.Step();

        fixture.Body.IsSleeping.Should().BeFalse();
        fixture.Body.IsGrounded.Should().BeFalse();
        fixture.Body.Position.Y.Should().BeLessThan(position.Y);
        fixture.Body.LinearVelocity.Y.Should().BeLessThan(Fixed64.Zero);
    }

    [Theory]
    [InlineData(PhysicsRuntimeMode.TwoD)]
    [InlineData(PhysicsRuntimeMode.Mixed)]
    public void UnchangedQueryOnlySupport_ShouldPreserveSleepAndGrounding(PhysicsRuntimeMode mode)
    {
        using var fixture = new QuerySupportScene(mode, bodyless: true);
        fixture.AssertSleepingQuerySupport();
        Vector2d position = fixture.Body.Position;
        Vector2d groundPoint = fixture.Body.GroundPoint;
        int transitions = 0;
        fixture.Body.OnGrounded += _ => transitions++;

        fixture.Body.GroundProbeMode = fixture.Body.GroundProbeMode;
        fixture.Body.GroundProbeRadius = fixture.Body.GroundProbeRadius;
        fixture.Body.GroundedDistanceRay = fixture.Body.GroundedDistanceRay;

        for (int frame = 0; frame < 32; frame++) fixture.Step();

        fixture.AssertSleepingQuerySupport();
        fixture.Body.Position.Should().Be(position);
        fixture.Body.GroundPoint.Should().Be(groundPoint);
        transitions.Should().Be(0);
    }

    [Theory]
    [InlineData(PhysicsRuntimeMode.TwoD)]
    [InlineData(PhysicsRuntimeMode.Mixed)]
    public void DisabledGroundProbeReach_ShouldWakeAndReleaseQueryOnlySupport(PhysicsRuntimeMode mode)
    {
        using var fixture = new QuerySupportScene(mode, bodyless: true);
        fixture.AssertSleepingQuerySupport();
        Vector2d position = fixture.Body.Position;
        fixture.Body.GroundDownDistanceOnAir = Fixed64.Zero;
        fixture.Body.GroundedDistanceRay = Fixed64.Zero;

        fixture.Step();

        fixture.Body.IsSleeping.Should().BeFalse();
        fixture.Body.IsGrounded.Should().BeFalse();
        fixture.Body.Position.Y.Should().BeLessThan(position.Y);
    }

    [Theory]
    [InlineData(PhysicsRuntimeMode.TwoD)]
    [InlineData(PhysicsRuntimeMode.Mixed)]
    public void ManualSupport_ShouldRemainHostOwnedWhenTheFormerQueryColliderMoves(PhysicsRuntimeMode mode)
    {
        using var fixture = new QuerySupportScene(mode, bodyless: true);
        Vector2d point = new(2, 3);
        fixture.Body.SetManualGrounding(point, Vector2d.Forward);
        Vector2d position = fixture.Body.Position;

        fixture.InvalidateSupport(change: 0);
        for (int frame = 0; frame < 8; frame++) fixture.Step();

        fixture.Body.IsSleeping.Should().BeTrue();
        fixture.Body.IsGrounded.Should().BeTrue();
        fixture.Body.GroundPoint.Should().Be(point);
        fixture.Body.Position.Should().Be(position);
    }

    [Theory]
    [InlineData(PhysicsRuntimeMode.TwoD, 0)]
    [InlineData(PhysicsRuntimeMode.TwoD, 1)]
    [InlineData(PhysicsRuntimeMode.TwoD, 2)]
    [InlineData(PhysicsRuntimeMode.Mixed, 0)]
    [InlineData(PhysicsRuntimeMode.Mixed, 1)]
    [InlineData(PhysicsRuntimeMode.Mixed, 2)]
    public void RestoredSupportDependency_ShouldMatchUninterruptedSleepAndMotion(
        PhysicsRuntimeMode mode, int supportChange)
    {
        using var uninterrupted = new QuerySupportScene(mode, bodyless: false);
        using var restored = new QuerySupportScene(mode, bodyless: false);
        uninterrupted.AssertSleepingQuerySupport();
        restored.AssertSleepingQuerySupport();
        object payload = GravitasSerializationHarness.Serialize(
            uninterrupted.Body, GravitasSerializationTransport.Json);
        GravitasSerializationHarness.Populate(restored.Body, payload, GravitasSerializationTransport.Json);

        // A tangential translation keeps the accepted witness unchanged. A
        // vertical translation changes its point; removal loses it entirely.
        Vector2d offset = supportChange == 0 ? Vector2d.Right * Fixed64.FromFraction(1, 16)
            : supportChange == 1 ? -Vector2d.Forward * Fixed64.FromFraction(1, 16)
            : Vector2d.Right * (Fixed64)12;
        uninterrupted.MoveStaticSupport(offset);
        restored.MoveStaticSupport(offset);

        for (int frame = 0; frame < 4; frame++)
        {
            uninterrupted.Step();
            restored.Step();
            restored.Body.IsSleeping.Should().Be(uninterrupted.Body.IsSleeping);
            restored.Body.IsGrounded.Should().Be(uninterrupted.Body.IsGrounded);
            restored.Body.WasGrounded.Should().Be(uninterrupted.Body.WasGrounded);
            restored.Body.Position.Should().Be(uninterrupted.Body.Position);
            restored.Body.LinearVelocity.Should().Be(uninterrupted.Body.LinearVelocity);
            restored.Body.GroundNormal.Should().Be(uninterrupted.Body.GroundNormal);
            restored.Body.TryGetGroundPoint(out Vector2d restoredPoint).Should()
                .Be(uninterrupted.Body.TryGetGroundPoint(out Vector2d uninterruptedPoint));
            restoredPoint.Should().Be(uninterruptedPoint);
            restored.Context.ComputeReplayHash().Should().Be(uninterrupted.Context.ComputeReplayHash());
        }
        uninterrupted.Body.IsSleeping.Should().Be(supportChange == 0);
    }

    [Theory]
    [InlineData(PhysicsRuntimeMode.TwoD, false)]
    [InlineData(PhysicsRuntimeMode.TwoD, true)]
    [InlineData(PhysicsRuntimeMode.Mixed, false)]
    [InlineData(PhysicsRuntimeMode.Mixed, true)]
    public void SupportLossCallback_ShouldRespectBodyDeactivationOrReinitialization(
        PhysicsRuntimeMode mode, bool reinitialize)
    {
        using var fixture = new QuerySupportScene(mode, bodyless: false);
        fixture.AssertSleepingQuerySupport();
        int exits = 0;
        fixture.Body.OnGrounded += grounded =>
        {
            if (grounded) return;
            exits++;
            fixture.Body.Deactivate();
            if (reinitialize) fixture.Body.Initialize(new Vector2d(4, 0));
        };
        fixture.MoveStaticSupport(Vector2d.Right * (Fixed64)12);

        fixture.Step();

        exits.Should().Be(1);
        fixture.Body.Active.Should().Be(reinitialize);
        fixture.Body.IsGrounded.Should().BeFalse();
        fixture.Context.Physics2D.BodyCount.Should().Be(reinitialize ? 1 : 0);
        if (reinitialize)
        {
            fixture.Body.Position.X.Should().Be((Fixed64)4);
            fixture.Step();
            fixture.Body.Position.X.Should().Be((Fixed64)4);
            fixture.Body.LinearVelocity.Y.Should().BeLessThan(Fixed64.Zero);
        }
    }

    [Fact]
    public void DirectBodyStep_WhenGroundLossCallbackDeactivatesBody_ShouldStopMotionPreparation()
    {
        using var fixture = new QuerySupportScene(PhysicsRuntimeMode.TwoD, bodyless: false);
        fixture.AssertSleepingQuerySupport();
        fixture.Body.OnGrounded += grounded =>
        {
            if (!grounded) fixture.Body.Deactivate();
        };
        fixture.MoveStaticSupport(Vector2d.Right * (Fixed64)12);

        fixture.Body.LateSimulate();

        fixture.Body.Active.Should().BeFalse();
        fixture.Body.Collider.IsActive.Should().BeFalse();
        fixture.Context.Physics2D.BodyCount.Should().Be(0);
    }

    [Theory]
    [InlineData(PhysicsRuntimeMode.TwoD, 0)]
    [InlineData(PhysicsRuntimeMode.TwoD, 1)]
    [InlineData(PhysicsRuntimeMode.TwoD, 2)]
    [InlineData(PhysicsRuntimeMode.Mixed, 0)]
    [InlineData(PhysicsRuntimeMode.Mixed, 1)]
    [InlineData(PhysicsRuntimeMode.Mixed, 2)]
    public void GroundedCallbackPublishingChangedGeometry_ShouldRejectTheOldWitness(
        PhysicsRuntimeMode mode, int mutation)
    {
        using var fixture = new QuerySupportScene(mode, bodyless: false);
        fixture.Body.UseManualGrounding();
        fixture.Body.UseAutomaticGrounding(checkGroundImmediately: false);
        string transitions = string.Empty;
        fixture.Body.OnGrounded += grounded =>
        {
            transitions += grounded ? "enter;" : "exit;";
            if (!grounded) return;
            fixture.PublishChangedGeometry(mutation);
        };

        fixture.Body.CheckGround();

        transitions.Should().Be("enter;exit;");
        fixture.Body.IsGrounded.Should().BeFalse();
        fixture.Body.HasGroundPoint.Should().BeFalse();
    }

    private sealed class QuerySupportScene : IDisposable
    {
        private readonly PhysicsScenarioBuilder _scene = PhysicsScenarioBuilder.Create();
        private readonly LSAABBoxCollider2D _floor = new(new Vector2d(4, 1));
        private readonly TestMatterAgent _floorAgent;
        private readonly SolidBody2D? _floorBody;

        internal QuerySupportScene(PhysicsRuntimeMode mode, bool bodyless)
        {
            Context.SetFrameRate(16);
            Context.Settings.RuntimeMode = mode;
            Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(0);
            Context.Environment.Gravity = Fixed64.One;
            Context.Environment.AirDensity = Fixed64.Zero;
            Context.Environment.DampingFactor = Fixed64.Zero;
            Context.Environment.MinSpeed = Fixed64.Zero;
            Vector2d floorPosition = new(Fixed64.FromFraction(3, 2), -Fixed64.Half);
            _floorAgent = CreateAgent(floorPosition);
            if (bodyless) _floor.InitializeWithNoBody(_floorAgent);
            else
            {
                _floorBody = new SolidBody2D(_floorAgent, _floor) { Mass = Fixed64.One };
                _floorBody.Initialize(floorPosition, motionType: BodyMotionType.Static);
            }
            // The feet-origin probe sees the floor, while the actual circle's
            // offset leaves a half-unit gap and therefore no physical pair.
            var collider = new LSCircleCollider2D(Fixed64.Half) { LocalOffset = Vector2d.Forward };
            Body = new SolidBody2D(CreateAgent(Vector2d.Zero), collider)
                { Mass = Fixed64.One, Gravity = -Vector2d.Forward, SleepEnabled = false };
            Body.Initialize(Vector2d.Zero);
            for (int frame = 0; frame < 16; frame++) Step();
            Body.SleepEnabled = true;
            Body.Sleep();
            Step();
            Step();
        }

        internal GravitasWorldContext Context => _scene.Context;
        internal SolidBody2D Body { get; }

        internal void AssertSleepingQuerySupport()
        {
            Body.IsSleeping.Should().BeTrue();
            Body.IsGrounded.Should().BeTrue();
            Body.Collider.Bounds.Min.Y.Should().BeGreaterThan(_floor.Bounds.Max.Y);
            (Body.Collider.CollisionPairCount + Body.Collider.CollisionPairHolderCount).Should().Be(0);
            Context.MixedCollisions.ActivePairCount.Should().Be(0);
        }

        internal void InvalidateSupport(int change)
        {
            if (change == 0)
            {
                Vector2d position = _floor.Center + Vector2d.Right * (Fixed64)12;
                if (_floorBody != null) _floorBody.SetPosition(position);
                else _floorAgent.Transform.TrySetWorldPosition(
                    new Vector3d(position.X, Fixed64.Zero, position.Y)).Should().BeTrue();
                // Raw host and authored mutations are committed by the normal
                // lifecycle; the test must not manually refresh the collider.
            }
            else if (change == 1)
            {
                // Withdraw the horizontal footprint, so a lower face cannot
                // remain an eligible hit within the grounding sweep's reach.
                _floor.Size = new Vector2d(Fixed64.FromFraction(1, 16), Fixed64.One);
            }
            else if (change == 2) _floor.Deactivate();
            else if (change == 3) Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.None;
            else if (change == 4) _floor.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(Body.Collider.Layer);
            else
            {
                Vector2d position = _floor.Center + Vector2d.Right * (Fixed64)12;
                _floor.Deactivate();
                _floorAgent.Transform.TrySetWorldPosition(new Vector3d(position.X, Fixed64.Zero, position.Y))
                    .Should().BeTrue();
                if (_floorBody != null) _floorBody.Initialize(position, motionType: BodyMotionType.Static);
                else _floor.InitializeWithNoBody(_floorAgent);
            }
        }

        internal void Step() { Context.Simulate(); Context.LateSimulate(); }
        internal void MoveStaticSupport(Vector2d offset) => _floorBody!.SetPosition(_floor.Center + offset);
        internal void PublishChangedGeometry(int mutation)
        {
            if (mutation == 0) MoveStaticSupport(Vector2d.Right * (Fixed64)12);
            else if (mutation == 1)
            {
                _floor.Size = new Vector2d(Fixed64.FromFraction(1, 16), Fixed64.One);
                _floor.Simulate();
            }
            else
            {
                ((LSCircleCollider2D)Body.Collider).Radius = Fixed64.Quarter;
                Body.Collider.Simulate();
            }
        }
        public void Dispose() => _scene.Dispose();
        private TestMatterAgent CreateAgent(Vector2d position) => new(Context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
    }
}
