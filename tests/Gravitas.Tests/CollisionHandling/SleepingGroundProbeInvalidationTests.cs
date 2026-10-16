using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Diagnostics;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class SleepingGroundProbeInvalidationTests
{
    public static TheoryData<bool, bool, int> LostSupports
    {
        get
        {
            var cases = new TheoryData<bool, bool, int>();
            foreach (bool bodyless in new[] { false, true })
            foreach (bool mixed in new[] { false, true })
            for (int change = 0; change < 8; change++)
                if (bodyless || change != 3) cases.Add(bodyless, mixed, change);
            return cases;
        }
    }

    [Theory]
    [MemberData(nameof(LostSupports))]
    public void LostQueryOnlySupport_ShouldWakeAndResumeGravity(bool bodyless, bool mixed, int change)
    {
        using var fixture = new SupportedProbe(bodyless, mixed);
        Vector3d position = fixture.Body.Position3d;

        fixture.ChangeSupport(change);
        for (int frame = 0; frame < 8; frame++) fixture.Step();

        fixture.Body.IsSleeping.Should().BeFalse();
        fixture.Body.Position3d.Y.Should().BeLessThan(position.Y);
        if (change == 1)
        {
            // The lower face remains inside the probe; follow its changed point rather than fall through it.
            fixture.Body.IsGrounded.Should().BeTrue();
            fixture.Body.Position3d.Y.Should().Be(fixture.Floor.BoundsMax.Y);
        }
        else
        {
            fixture.Body.IsGrounded.Should().BeFalse();
            fixture.Body.LinearVelocity.Y.Should().BeLessThan(Fixed64.Zero);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UnchangedQuerySupportOrManualOwnership_ShouldPreserveSleep(bool manual)
    {
        using var fixture = new SupportedProbe(bodyless: true, mixed: false);
        Vector3d position = fixture.Body.Position3d;
        fixture.Body.GroundProbeMode = fixture.Body.GroundProbeMode;
        fixture.Body.GroundProbeRadius = fixture.Body.GroundProbeRadius;
        if (manual)
        {
            fixture.Body.UseManualGrounding(clearGrounding: false);
            fixture.Body.GroundProbeMode = GroundProbeMode.Ray;
            fixture.Body.GroundProbeRadius = Fixed64.Quarter;
            fixture.ChangeSupport(0);
        }
        for (int frame = 0; frame < 16; frame++) fixture.Step();

        fixture.Body.IsSleeping.Should().BeTrue();
        fixture.Body.IsGrounded.Should().BeTrue();
        fixture.Body.Position3d.Should().Be(position);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RestoredQuerySupportedSleeper_ShouldReconstructSupportBeforeResuming(bool remove)
    {
        using var fixture = new SupportedProbe(bodyless: true, mixed: false);
        var payload = GravitasSerializationHarness.Serialize(fixture.Body, GravitasSerializationTransport.Json);
        GravitasSerializationHarness.Populate(fixture.Body, payload, GravitasSerializationTransport.Json);
        fixture.Body.IsSleeping.Should().BeTrue();
        if (remove) fixture.ChangeSupport(0);

        for (int frame = 0; frame < 8; frame++) fixture.Step();

        fixture.Body.IsSleeping.Should().Be(!remove);
        fixture.Body.IsGrounded.Should().Be(!remove);
        if (remove) fixture.Body.Position3d.Y.Should().BeLessThan(Fixed64.Zero);
        else fixture.Body.Position3d.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void GroundedCallback_PopulatingAutomaticSupport_ShouldPreserveLoadedState()
    {
        using var fixture = new SupportedProbe(bodyless: true, mixed: false);
        Vector3d savedPoint = fixture.Body.HitPoint;
        object payload = GravitasSerializationHarness.Serialize(fixture.Body, GravitasSerializationTransport.Json);
        fixture.Body.UseManualGrounding();
        fixture.Body.UseAutomaticGrounding(checkGroundImmediately: false);
        int callbacks = 0;
        fixture.Body.OnGrounded = grounded =>
        {
            if (!grounded) return;
            callbacks++;
            fixture.Body.OnGrounded = null;
            GravitasSerializationHarness.Populate(fixture.Body, payload, GravitasSerializationTransport.Json);
        };

        fixture.Body.CheckGround();

        callbacks.Should().Be(1);
        fixture.Body.GroundingMode.Should().Be(GroundingMode.Automatic);
        fixture.Body.IsGrounded.Should().BeTrue();
        fixture.Body.IsSleeping.Should().BeTrue();
        fixture.Body.HitPoint.Should().Be(savedPoint);
        fixture.Step();
        fixture.Body.IsGrounded.Should().BeTrue();
        fixture.Body.IsSleeping.Should().BeTrue();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void GroundedCallback_DeactivatingOrReinitializingBody_ShouldPreserveItsNewLifetime(bool reinitialize)
    {
        using var fixture = new SupportedProbe(bodyless: true, mixed: false);
        fixture.Body.UseManualGrounding();
        fixture.Body.UseAutomaticGrounding(checkGroundImmediately: false);
        int callbacks = 0;
        fixture.Body.OnGrounded = grounded =>
        {
            if (!grounded) return;
            callbacks++;
            fixture.Body.OnGrounded = null;
            fixture.Body.Deactivate();
            if (reinitialize) fixture.Body.Initialize(Vector3d.Up * (Fixed64)4, FixedQuaternion.Identity);
        };

        fixture.Body.CheckGround();

        callbacks.Should().Be(1);
        fixture.Body.Active.Should().Be(reinitialize);
        fixture.Body.IsGrounded.Should().BeFalse();
        fixture.Body.HasHitPoint.Should().BeFalse();
        fixture.Context.Physics.BodyCount.Should().Be(reinitialize ? 1 : 0);
        if (reinitialize) fixture.Body.Position3d.Should().Be(Vector3d.Up * (Fixed64)4);
    }

    [Fact]
    public void GroundedCallback_TakingManualOwnership_ShouldRetainTheAcceptedHostPoint()
    {
        using var fixture = new SupportedProbe(bodyless: true, mixed: false);
        Vector3d point = fixture.Body.HitPoint;
        fixture.Body.UseManualGrounding();
        fixture.Body.UseAutomaticGrounding(checkGroundImmediately: false);
        fixture.Body.OnGrounded = grounded =>
        {
            if (grounded) fixture.Body.UseManualGrounding(clearGrounding: false);
        };

        fixture.Body.CheckGround();
        fixture.ChangeSupport(0);
        fixture.Step();

        fixture.Body.GroundingMode.Should().Be(GroundingMode.Manual);
        fixture.Body.IsGrounded.Should().BeTrue();
        fixture.Body.IsSleeping.Should().BeTrue();
        fixture.Body.HitPoint.Should().Be(point);
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(false, 1)]
    [InlineData(false, 2)]
    [InlineData(true, 0)]
    [InlineData(true, 1)]
    [InlineData(true, 2)]
    public void RestoredQuerySupport_ShouldMatchUninterruptedContinuation(bool mixed, int change)
    {
        using var uninterrupted = new SupportedProbe(bodyless: false, mixed);
        using var restored = new SupportedProbe(bodyless: false, mixed);
        var payload = GravitasSerializationHarness.Serialize(uninterrupted.Body, GravitasSerializationTransport.Json);
        GravitasSerializationHarness.Populate(restored.Body, payload, GravitasSerializationTransport.Json);
        Vector3d offset = change == 0 ? Vector3d.Right * Fixed64.FromFraction(1, 16)
            : change == 1 ? Vector3d.Down * Fixed64.FromFraction(1, 16)
            : Vector3d.Right * (Fixed64)12;
        uninterrupted.Floor.Body!.SetPosition(uninterrupted.Floor.Center + offset);
        restored.Floor.Body!.SetPosition(restored.Floor.Center + offset);

        for (int frame = 0; frame < 4; frame++)
        {
            uninterrupted.Step();
            restored.Step();
            restored.Body.IsSleeping.Should().Be(uninterrupted.Body.IsSleeping);
            restored.Body.IsGrounded.Should().Be(uninterrupted.Body.IsGrounded);
            restored.Body.WasGrounded.Should().Be(uninterrupted.Body.WasGrounded);
            restored.Body.Position3d.Should().Be(uninterrupted.Body.Position3d);
            restored.Body.LinearVelocity.Should().Be(uninterrupted.Body.LinearVelocity);
            restored.Body.GroundNormal.Should().Be(uninterrupted.Body.GroundNormal);
            restored.Body.TryGetHitPoint(out Vector3d restoredPoint).Should()
                .Be(uninterrupted.Body.TryGetHitPoint(out Vector3d uninterruptedPoint));
            restoredPoint.Should().Be(uninterruptedPoint);
            restored.Context.ComputeReplayHash().Should().Be(uninterrupted.Context.ComputeReplayHash());
        }
        uninterrupted.Body.IsSleeping.Should().Be(change == 0);
    }

    [Fact]
    public void PositivePairCulling_ShouldRetainBroadPhaseFreshnessInAuthoritativeHash()
    {
        using var uninterrupted = new SupportedProbe(bodyless: false, mixed: false);
        using var restored = new SupportedProbe(bodyless: false, mixed: false);
        uninterrupted.Context.Environment.CullTimeStep = restored.Context.Environment.CullTimeStep = 1;
        uninterrupted.Context.Environment.CullTimeMax = restored.Context.Environment.CullTimeMax = 8;
        var firstPair = uninterrupted.Context.Physics.GetCollisionPair(uninterrupted.Floor.Id, uninterrupted.Body.Collider.Id)!;
        var secondPair = restored.Context.Physics.GetCollisionPair(restored.Floor.Id, restored.Body.Collider.Id)!;
        firstPair.UpdateCollision();
        secondPair.UpdateCollision();
        firstPair.Manifold.HasContact.Should().BeFalse();
        firstPair.CullCounter.Should().BeGreaterThan(0);
        secondPair.CullCounter.Should().Be(firstPair.CullCounter);
        restored.Context.ComputeReplayHash().Should().Be(uninterrupted.Context.ComputeReplayHash());
        object payload = GravitasSerializationHarness.Serialize(uninterrupted.Body, GravitasSerializationTransport.Json);

        // Loading republishes identical geometry. A pending cull makes its
        // freshness meaningful: it determines whether the next check is deferred.
        GravitasSerializationHarness.Populate(restored.Body, payload, GravitasSerializationTransport.Json);

        restored.Body.Position3d.Should().Be(uninterrupted.Body.Position3d);
        secondPair.CullCounter.Should().Be(firstPair.CullCounter);
        restored.Context.ComputeReplayHash().Should().NotBe(uninterrupted.Context.ComputeReplayHash());
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SupportLossCallback_CanDeactivateOrReinitializeTheBody(bool reinitialize)
    {
        using var fixture = new SupportedProbe(bodyless: true, mixed: false);
        int callbacks = 0;
        fixture.Body.OnGrounded = grounded =>
        {
            if (grounded) return;
            callbacks++;
            fixture.Body.OnGrounded = null;
            fixture.Body.Deactivate();
            if (reinitialize) fixture.Body.Initialize(Vector3d.Up * (Fixed64)4, FixedQuaternion.Identity);
        };
        fixture.ChangeSupport(2);

        fixture.Step();

        callbacks.Should().Be(1);
        fixture.Body.Active.Should().Be(reinitialize);
        if (reinitialize) fixture.Body.Position3d.Y.Should().BeGreaterThan((Fixed64)3);
    }

    [Fact]
    public void SupportLossCallback_CanReplaceAutomaticStateWithManualSupport()
    {
        using var fixture = new SupportedProbe(bodyless: true, mixed: false);
        Vector3d manualPoint = Vector3d.Up * (Fixed64)5;
        fixture.Body.OnGrounded = grounded =>
        {
            if (grounded) return;
            fixture.Body.SetManualGrounding(manualPoint, Vector3d.Up);
            fixture.Body.Sleep();
        };
        fixture.ChangeSupport(2);

        fixture.Step();

        fixture.Body.GroundingMode.Should().Be(GroundingMode.Manual);
        fixture.Body.IsGrounded.Should().BeTrue();
        fixture.Body.IsSleeping.Should().BeTrue();
        fixture.Body.HitPoint.Should().Be(manualPoint);
        fixture.Body.GroundNormal.Should().Be(Vector3d.Up);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ChangedProbeGeometry_ShouldReconsiderSleepingSupport(bool originChange)
    {
        using var fixture = new SupportedProbe(bodyless: true, mixed: false);
        fixture.Body.GroundProbeMode = GroundProbeMode.Ray;
        fixture.Body.CheckGround();
        fixture.Body.GroundDownDistanceOnAir = Fixed64.Zero;
        if (originChange) fixture.Body.GroundOriginOffset = (Fixed64)8;
        else fixture.Body.GroundedDistanceRay = Fixed64.Zero;

        for (int frame = 0; frame < 4; frame++) fixture.Step();

        fixture.Body.IsSleeping.Should().BeFalse();
        fixture.Body.IsGrounded.Should().BeFalse();
        fixture.Body.LinearVelocity.Y.Should().BeLessThan(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DirectBodyLateSimulate_ShouldValidateSupportBeforeSkippingSleep(bool deactivateInCallback)
    {
        using var fixture = new SupportedProbe(bodyless: true, mixed: false);
        if (deactivateInCallback)
            fixture.Body.OnGrounded = grounded => { if (!grounded) fixture.Body.Deactivate(); };
        fixture.ChangeSupport(2);

        fixture.Body.LateSimulate();

        fixture.Body.Active.Should().Be(!deactivateInCallback);
        if (!deactivateInCallback)
        {
            fixture.Body.IsSleeping.Should().BeFalse();
            fixture.Body.IsGrounded.Should().BeFalse();
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void GroundedCallbackPublishingNewGeometryOrPolicy_ShouldRejectTheOldWitness(int change)
    {
        using var fixture = new SupportedProbe(bodyless: true, mixed: false);
        fixture.Body.UseManualGrounding();
        fixture.Body.UseAutomaticGrounding(checkGroundImmediately: false);
        fixture.Body.OnGrounded = grounded =>
        {
            if (!grounded) return;
            if (change == 0) fixture.Floor.Position += Vector3d.Right * (Fixed64)12;
            else if (change == 1) { fixture.Floor.Size = Vector3d.One * Fixed64.FromFraction(1, 16); fixture.Floor.Simulate(); }
            else if (change == 2) { fixture.Body.Collider.Radius = Fixed64.FromFraction(1, 4); fixture.Body.Collider.Simulate(); }
            else if (change == 3) fixture.Body.GroundOriginOffset = (Fixed64)8;
            else fixture.Body.GroundProbeMode = GroundProbeMode.Ray;
        };

        fixture.Body.CheckGround();

        fixture.Body.IsGrounded.Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EquivalentProbePolicyChange_ShouldRequeryOnceAndPreserveSleep(bool radiusChange)
    {
        using var fixture = new SupportedProbe(bodyless: true, mixed: false);
        fixture.Context.Diagnostics.Enable();
        fixture.Context.Diagnostics.Clear();
        if (radiusChange) fixture.Body.GroundProbeRadius = Fixed64.FromFraction(1, 4);
        else fixture.Body.GroundProbeMode = GroundProbeMode.Ray;

        for (int frame = 0; frame < 3; frame++) fixture.Step();

        fixture.Body.IsSleeping.Should().BeTrue();
        int probes = 0;
        foreach (var diagnostic in fixture.Context.Diagnostics.Events)
            if (diagnostic.Kind == GravitasDiagnosticEventKind.GroundProbe) probes++;
        probes.Should().Be(1);
    }

    private sealed class SupportedProbe : IDisposable
    {
        private readonly PhysicsScenarioBuilder _scene = PhysicsScenarioBuilder.Create();
        private readonly bool _bodyless;
        internal LSCuboidCollider Floor { get; } = new()
        {
            Size = new Vector3d(8, 1, 8), Material = PhysicsMaterial.Frictionless, Layer = new PhysicsLayer(1)
        };

        internal SupportedProbe(bool bodyless, bool mixed)
        {
            _bodyless = bodyless;
            Context.Settings.RuntimeMode = mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.ThreeD;
            Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
            Context.SetFrameRate(16);
            Context.Environment.Gravity = Fixed64.One;
            Context.Environment.AirDensity = Fixed64.Zero;
            Context.Environment.DampingFactor = Fixed64.Zero;
            Context.Environment.MinSpeed = Fixed64.Zero;
            AddFloor(Vector3d.Down * Fixed64.Half);
            var sphere = new LSSphereCollider
            {
                Radius = Fixed64.Half, LocalOffset = Vector3d.Up, Material = PhysicsMaterial.Frictionless
            };
            Body = _scene.CreateBody(sphere, Vector3d.Zero, FixedQuaternion.Identity, preventAngularForces: true).Body;
            Body.SleepEnabled = false;
            for (int frame = 0; frame < 16; frame++) Step();
            // The probe reaches the floor across a half-unit physical gap.
            Body.IsGrounded.Should().BeTrue();
            sphere.BoundsMin.Y.Should().BeGreaterThan(Floor.BoundsMax.Y);
            var pair = Context.Physics.GetCollisionPair(Floor.Id, sphere.Id);
            (pair?.Manifold.HasContact ?? false).Should().BeFalse();
            Body.SleepEnabled = true;
            Body.Sleep();
            Step();
            Body.IsSleeping.Should().BeTrue();
        }

        internal GravitasWorldContext Context => _scene.Context;
        internal SolidBody Body { get; }
        internal void Step() { Context.Simulate(); Context.LateSimulate(); }

        internal void ChangeSupport(int change)
        {
            switch (change)
            {
                case 0:
                    // Let the ordinary service pass publish raw host mutation.
                    Floor.AgentOrNull!.Transform.TrySetWorldPosition(Vector3d.Right * (Fixed64)12).Should().BeTrue();
                    if (Floor.Body != null) Floor.Body.SetPosition(Vector3d.Right * (Fixed64)12);
                    break;
                case 1: Floor.Size = new Vector3d((Fixed64)8, Fixed64.FromFraction(1, 16), (Fixed64)8); break;
                case 2: RemoveFloor(); break;
                case 3: Floor.IsTrigger = true; break;
                case 4: Context.Settings.GroundCheckLayerMask = PhysicsLayerMask.None; break;
                case 5: Floor.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(Body.Collider.Layer); break;
                case 6: Body.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(Floor.Layer); break;
                case 7:
                    RemoveFloor();
                    AddFloor(Vector3d.Right * (Fixed64)12);
                    break;
            }
        }

        private void RemoveFloor()
        {
            if (Floor.Body != null) Floor.Body.Deactivate();
            else Floor.Deactivate();
        }

        private void AddFloor(Vector3d position)
        {
            if (_bodyless) _scene.InitializeStaticCollider(Floor, position);
            else _scene.CreateBody(Floor, position, FixedQuaternion.Identity, immovable: true);
        }

        public void Dispose() => _scene.Dispose();
    }
}
