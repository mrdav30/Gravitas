using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class SleepingContactInvalidationParityTests
{
    [Theory]
    [InlineData(false, 0)]
    [InlineData(false, 1)]
    [InlineData(false, 2)]
    [InlineData(true, 0)]
    [InlineData(true, 1)]
    [InlineData(true, 2)]
    public void Pure2D_InvalidatedStaticSupport_ShouldWakeAndReleaseTheSleeper(bool bodyless, int change)
    {
        using var fixture = new SupportedScene(kind: 0, bodyless);
        AssertSupportLoss(fixture, change);
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(true, 1)]
    [InlineData(true, 2)]
    public void Mixed3D_InvalidatedPlanarSlab_ShouldWakeAndReleaseTheSleeper(bool bodyless, int change)
    {
        using var fixture = new SupportedScene(kind: 1, bodyless);
        AssertSupportLoss(fixture, change);
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(true, 1)]
    [InlineData(true, 2)]
    public void Mixed2D_InvalidatedSpatialSupport_ShouldWakeAndReleaseTheSleeper(bool bodyless, int change)
    {
        using var fixture = new SupportedScene(kind: 2, bodyless);
        AssertSupportLoss(fixture, change);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Pure2D_AutomaticSupportLoss_ShouldClearGroundingAndResumeGravity(bool bodyless)
    {
        using var fixture = new SupportedScene(kind: 0, bodyless, automaticGrounding: true);
        fixture.Mover2D!.IsGrounded.Should().BeTrue();

        AssertSupportLoss(fixture, change: 0);

        fixture.Mover2D.IsGrounded.Should().BeFalse();
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void UnchangedSupportAndUnrelatedTriggerExit_ShouldPreserveSleep(int kind)
    {
        using var fixture = new SupportedScene(kind, bodyless: true, withTrigger: true);
        Fixed64 height = fixture.Height;
        for (int frame = 0; frame < 16; frame++) fixture.Step();
        fixture.IsSleeping.Should().BeTrue();
        fixture.Height.Should().Be(height);

        fixture.MoveTriggerAway();
        for (int frame = 0; frame < 4; frame++) fixture.Step();

        fixture.TriggerExits.Should().Be(1);
        fixture.IsSleeping.Should().BeTrue();
        fixture.Height.Should().Be(height);
        fixture.HasSupportingContact.Should().BeTrue();
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void ChangedSolidGeometryThatStillOverlaps_ShouldWakeTheSleepingDependency(int kind)
    {
        using var fixture = new SupportedScene(kind, bodyless: true);

        fixture.ShiftSupport();
        fixture.Step();

        fixture.IsSleeping.Should().BeFalse();
        fixture.HasSupportingContact.Should().BeTrue();
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void ChangingPhysicalKindOfKnownOverlap_ShouldWakeTheSleepingDependency(int kind)
    {
        using var fixture = new SupportedScene(kind, bodyless: true);

        fixture.MakeSupportTrigger(true);
        fixture.Step();

        fixture.IsSleeping.Should().BeFalse();
        // This retained trigger overlap becomes a real constraint again.
        fixture.SleepMover();
        fixture.MakeSupportTrigger(false);
        fixture.Step();
        fixture.IsSleeping.Should().BeFalse();
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void SupportMaterialMutation_ShouldWakeOnlyForAnActualChange(int kind, bool changed)
    {
        using var fixture = new SupportedScene(kind, bodyless: true);

        fixture.SetSupportMaterial(changed ? PhysicsMaterial.Default : PhysicsMaterial.Frictionless);
        fixture.Step();

        fixture.IsSleeping.Should().Be(!changed);
        fixture.HasSupportingContact.Should().BeTrue();
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void AcceptedDynamicPositionCorrection_ShouldNotWakeSleepingNeighbors(bool mixed, bool noOpAssignments)
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.TwoD;
        scene.Context.Environment.Gravity = Fixed64.Zero;
        scene.Context.Environment.AirDensity = Fixed64.Zero;
        scene.Context.Environment.DampingFactor = Fixed64.Zero;
        SolidBody? spatial = mixed ? scene.CreateSphere(Vector3d.Zero, preventAngularForces: true).Body : null;
        SolidBody2D? planar = mixed ? null : CreateDynamicCircle(scene.Context, Vector2d.Zero);
        SolidBody2D neighbor = CreateDynamicCircle(scene.Context, Vector2d.Right * Fixed64.FromFraction(3, 4));
        if (spatial != null) { spatial.UseManualGrounding(); spatial.SleepEnabled = false; }
        scene.Context.Simulate();
        scene.Context.LateSimulate();
        neighbor.Position.X.Should().BeGreaterThan(Fixed64.FromFraction(3, 4));
        Vector2d neighborPosition = neighbor.Position;
        Vector3d firstPosition = spatial?.Position3d
            ?? new Vector3d(planar!.Position.X, Fixed64.Zero, planar.Position.Y);
        CollisionPair2D? pair = null;
        ulong contactId = 0;
        ContactWarmStartImpulse impulse = default;
        if (!mixed)
        {
            (planar!.Collider.TryGetCollisionPair(neighbor.Collider.Id, out pair)
                || neighbor.Collider.TryGetCollisionPair(planar.Collider.Id, out pair)).Should().BeTrue();
            contactId = pair!.Manifold.PrimaryContact.ContactId;
            pair.TryGetWarmStartImpulse(default, contactId, out impulse).Should().BeTrue();
        }
        neighbor.SleepEnabled = true;
        neighbor.Sleep();
        if (spatial != null) { spatial.SleepEnabled = true; spatial.Sleep(); }
        else { planar!.SleepEnabled = true; planar.Sleep(); }
        // Equal host assignments may publish a pending accepted solver pose,
        // but must not turn that publication into a new physical stimulus.
        if (noOpAssignments)
        {
            neighbor.SetPosition(neighbor.Position);
            neighbor.SetRotation(neighbor.Rotation);
            if (spatial != null) { spatial.SetPosition(spatial.Position3d); spatial.SetRotation(spatial.Rotation); }
            else { planar!.SetPosition(planar.Position); planar.SetRotation(planar.Rotation); }
        }

        for (int frame = 0; frame < 4; frame++)
        {
            scene.Context.Simulate();
            scene.Context.LateSimulate();
            neighbor.IsSleeping.Should().BeTrue();
            neighbor.Position.Should().Be(neighborPosition);
            (spatial?.IsSleeping ?? planar!.IsSleeping).Should().BeTrue();
            (spatial?.Position3d ?? new Vector3d(planar!.Position.X, Fixed64.Zero, planar.Position.Y))
                .Should().Be(firstPosition);
            if (pair != null)
            {
                pair.TryGetWarmStartImpulse(default, contactId, out ContactWarmStartImpulse retained).Should().BeTrue();
                retained.Should().Be(impulse);
            }
        }
    }

    private static SolidBody2D CreateDynamicCircle(GravitasWorldContext context, Vector2d position)
    {
        var agent = new TestMatterAgent(context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
            { Mass = Fixed64.One, SleepEnabled = false };
        body.Initialize(position);
        body.UseManualGrounding();
        return body;
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ResleptBodyWithBodylessNeighbor_ShouldIgnoreItsOwnPoseEditButWakeWhenNeighborIsRemoved(bool bodylessFirst)
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var target = new LSCircleCollider2D(Fixed64.Half);
        void RegisterTarget() => target.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        if (bodylessFirst) RegisterTarget();
        SolidBody2D body = CreateDynamicCircle(scene.Context, Vector2d.Right * Fixed64.FromFraction(3, 4));
        if (!bodylessFirst) RegisterTarget();
        StepContext(scene.Context);
        body.SleepEnabled = true;
        body.Sleep();
        StepContext(scene.Context);
        Vector2d requested = body.Position - Vector2d.Right * Fixed64.FromFraction(1, 64);

        body.SetPosition(requested);
        body.Sleep();
        StepContext(scene.Context);

        body.IsSleeping.Should().BeTrue();
        body.Position.Should().Be(requested);
        (target.TryGetCollisionPair(body.Collider.Id, out var pair)
            || body.Collider.TryGetCollisionPair(target.Id, out pair)).Should().BeTrue();
        pair!.Manifold.HasContact.Should().BeTrue();

        target.Deactivate();
        StepContext(scene.Context);

        body.IsSleeping.Should().BeFalse();
        (body.Collider.CollisionPairCount + body.Collider.CollisionPairHolderCount).Should().Be(0);
    }

    [Fact]
    public void AwakeDriver_ShouldExpandThroughAnUnchangedSleepingPlanarContactChain()
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        SolidBody2D driver = CreateDynamicCircle(scene.Context, Vector2d.Zero);
        SolidBody2D middle = CreateDynamicCircle(scene.Context, Vector2d.Right * Fixed64.FromFraction(3, 4));
        SolidBody2D far = CreateDynamicCircle(scene.Context, Vector2d.Right * Fixed64.FromFraction(3, 2));
        SolidBody2D unrelated = CreateDynamicCircle(scene.Context, Vector2d.Left * (Fixed64)8);
        SolidBody2D unrelatedNeighbor = CreateDynamicCircle(scene.Context,
            Vector2d.Left * Fixed64.FromFraction(29, 4));
        StepContext(scene.Context);
        foreach (SolidBody2D body in new[] { middle, far, unrelated, unrelatedNeighbor })
        {
            body.SleepEnabled = true;
            body.Sleep();
        }
        Vector2d unrelatedPosition = unrelated.Position;
        Vector2d unrelatedNeighborPosition = unrelatedNeighbor.Position;

        driver.AddLinearImpulse(Vector2d.Right);
        StepContext(scene.Context);

        middle.IsSleeping.Should().BeFalse();
        far.IsSleeping.Should().BeFalse();
        far.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
        unrelated.IsSleeping.Should().BeTrue();
        unrelatedNeighbor.IsSleeping.Should().BeTrue();
        unrelated.Position.Should().Be(unrelatedPosition);
        unrelatedNeighbor.Position.Should().Be(unrelatedNeighborPosition);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ReplacingVerticalAndPlanarMixedConstraints_ShouldWakeTheAffectedPlanarBody(bool planarFirst)
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        scene.Context.Environment.Gravity = Fixed64.Zero;
        var target = new LSSphereCollider();
        scene.InitializeStaticCollider(target, (planarFirst ? Vector3d.Right : Vector3d.Up) * Fixed64.FromFraction(3, 4));
        SolidBody2D body = CreateDynamicCircle(scene.Context, Vector2d.Zero);
        StepContext(scene.Context);
        scene.Context.MixedCollisions.ActivePairCount.Should().Be(1);
        body.SleepEnabled = true;
        body.Sleep();
        StepContext(scene.Context);
        body.IsSleeping.Should().BeTrue();
        Vector2d position = body.Position;

        target.Position = new Vector3d(position.X, Fixed64.Zero, position.Y)
            + (planarFirst ? Vector3d.Up : Vector3d.Right) * Fixed64.FromFraction(3, 4);
        target.Simulate();
        StepContext(scene.Context);
        StepContext(scene.Context);

        body.IsSleeping.Should().BeFalse();
        scene.Context.MixedCollisions.ActivePairCount.Should().Be(1);
        if (planarFirst) body.Position.Should().Be(position);
        else body.Position.X.Should().BeLessThan(position.X);
    }

    [Fact]
    public void VerticalOnlyMixedDependencyChangeAndRemoval_ShouldPreservePlanarSleep()
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        scene.Context.Environment.Gravity = Fixed64.Zero;
        var target = new LSSphereCollider();
        scene.InitializeStaticCollider(target, Vector3d.Up * Fixed64.FromFraction(3, 4));
        SolidBody2D body = CreateDynamicCircle(scene.Context, Vector2d.Zero);
        StepContext(scene.Context);
        scene.Context.MixedCollisions.ActivePairCount.Should().Be(1);
        body.SleepEnabled = true;
        body.Sleep();
        Vector2d position = body.Position;

        target.Material = PhysicsMaterial.Bouncy;
        StepContext(scene.Context);

        body.IsSleeping.Should().BeTrue();
        body.Position.Should().Be(position);
        scene.Context.MixedCollisions.ActivePairCount.Should().Be(1);

        target.Deactivate();
        StepContext(scene.Context);

        body.IsSleeping.Should().BeTrue();
        body.Position.Should().Be(position);
        scene.Context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void SingleMixedContactPublication_ShouldWakeAChangedPlanarDependency()
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var target = new LSSphereCollider();
        scene.InitializeStaticCollider(target, Vector3d.Right * Fixed64.FromFraction(3, 4));
        SolidBody2D body = CreateDynamicCircle(scene.Context, Vector2d.Zero);
        var pair = new CollisionPairMixed(target, body.Collider);
        CollisionDetectionMixed.TryCollide(target, body.Collider, out MixedContact contact).Should().BeTrue();
        contact.Normal3DTo2D.ToVector2d().Should().NotBe(Vector2d.Zero);
        pair.MarkColliding(scene.Context.FrameCount, contact);
        body.SleepEnabled = true;
        body.Sleep();
        target.Material = target.Material;
        pair.MarkColliding(scene.Context.FrameCount, contact);
        body.IsSleeping.Should().BeTrue();

        target.Material = PhysicsMaterial.Bouncy;
        CollisionDetectionMixed.TryCollide(target, body.Collider, out contact).Should().BeTrue();
        pair.MarkColliding(scene.Context.FrameCount, contact);

        pair.Manifold.HasContact.Should().BeTrue();
        body.IsSleeping.Should().BeFalse();
    }

    private static void StepContext(GravitasWorldContext context) { context.Simulate(); context.LateSimulate(); }

    private static void AssertSupportLoss(SupportedScene fixture, int change)
    {
        fixture.IsSleeping.Should().BeTrue();
        fixture.HasSupportingContact.Should().BeTrue();
        Fixed64 height = fixture.Height;

        fixture.ChangeSupport(change);
        for (int frame = 0; frame < 8; frame++) fixture.Step();

        fixture.IsSleeping.Should().BeFalse();
        fixture.Height.Should().BeLessThan(height);
        fixture.VerticalVelocity.Should().BeLessThan(Fixed64.Zero);
        fixture.HasSupportingContact.Should().BeFalse();
    }

    private sealed class SupportedScene : IDisposable
    {
        private readonly PhysicsScenarioBuilder _scene = PhysicsScenarioBuilder.Create();
        private readonly int _kind;
        private readonly LSAABBoxCollider2D? _floor2D;
        private readonly LSCuboidCollider? _floor3D;
        private readonly LSCircleCollider2D? _trigger;
        private readonly LSSphereCollider? _trigger3D;
        private readonly SolidBody? _mover3D;
        private bool _hasMixedSupport;

        internal SupportedScene(int kind, bool bodyless, bool automaticGrounding = false, bool withTrigger = false)
        {
            _kind = kind;
            Context.SetFrameRate(16);
            Context.Settings.RuntimeMode = kind == 0 ? PhysicsRuntimeMode.TwoD : PhysicsRuntimeMode.Mixed;
            Context.Environment.Gravity = Fixed64.One;
            Context.Environment.AirDensity = Fixed64.Zero;
            Context.Environment.DampingFactor = Fixed64.Zero;
            Context.Environment.MinSpeed = Fixed64.Zero;

            if (kind == 2)
            {
                _floor3D = new LSCuboidCollider { Size = new Vector3d(8, 1, 1), Material = PhysicsMaterial.Frictionless };
                Vector3d center = Vector3d.Backward * Fixed64.Half;
                if (bodyless) _scene.InitializeStaticCollider(_floor3D, center);
                else _scene.CreateBody(_floor3D, center, FixedQuaternion.Identity, immovable: true);
                _floor3D.OnMixedContactEnter += _ => _hasMixedSupport = true;
                _floor3D.OnMixedContactExit += _ => _hasMixedSupport = false;
            }
            else
            {
                _floor2D = new LSAABBoxCollider2D(new Vector2d((Fixed64)8, kind == 0 ? Fixed64.One : (Fixed64)8))
                    { Material = PhysicsMaterial.Frictionless };
                Vector2d center = kind == 0 ? -Vector2d.Forward * Fixed64.Half : Vector2d.Zero;
                var agent = CreateAgent(new Vector3d(center.X, Fixed64.Zero, center.Y));
                if (bodyless) _floor2D.InitializeWithNoBody(agent);
                else new SolidBody2D(agent, _floor2D).Initialize(center, motionType: BodyMotionType.Static);
                _floor2D.OnMixedContactEnter += _ => _hasMixedSupport = true;
                _floor2D.OnMixedContactExit += _ => _hasMixedSupport = false;
            }

            if (kind == 1)
            {
                _mover3D = _scene.CreateSphere(Vector3d.Up, preventAngularForces: true).Body;
                _mover3D.Collider.Material = PhysicsMaterial.Frictionless;
                _mover3D.UseManualGrounding();
                _mover3D.SleepEnabled = false;
            }
            else
            {
                var collider = new LSCircleCollider2D(Fixed64.Half) { Material = PhysicsMaterial.Frictionless };
                Mover2D = new SolidBody2D(CreateAgent(Vector3d.Zero), collider)
                    { Mass = Fixed64.One, Gravity = -Vector2d.Forward };
                Mover2D.Initialize(Vector2d.Forward * Fixed64.Half);
                Mover2D.SleepEnabled = false;
                if (!automaticGrounding) Mover2D.UseManualGrounding();
            }

            if (withTrigger)
            {
                Vector3d center = kind == 1 ? _mover3D!.Position3d
                    : new Vector3d(Mover2D!.Position.X, Fixed64.Zero, Mover2D.Position.Y);
                if (kind == 2)
                {
                    _trigger3D = new LSSphereCollider { Radius = Fixed64.One, IsTrigger = true };
                    _trigger3D.InitializeWithNoBody(CreateAgent(center));
                    _trigger3D.OnMixedTriggerExit += _ => TriggerExits++;
                }
                else
                {
                    _trigger = new LSCircleCollider2D(Fixed64.One) { IsTrigger = true };
                    _trigger.InitializeWithNoBody(CreateAgent(center));
                    if (kind == 0) _trigger.OnTriggerExit += _ => TriggerExits++;
                    else _trigger.OnMixedTriggerExit += _ => TriggerExits++;
                }
            }

            for (int frame = 0; frame < 16; frame++) Step();
            SleepMover();
            Step();
            IsSleeping.Should().BeTrue();
            HasSupportingContact.Should().BeTrue();
        }

        internal GravitasWorldContext Context => _scene.Context;
        internal SolidBody2D? Mover2D { get; }
        internal int TriggerExits { get; private set; }
        internal bool IsSleeping => _mover3D?.IsSleeping ?? Mover2D!.IsSleeping;
        internal Fixed64 Height => _mover3D?.Position3d.Y ?? Mover2D!.Position.Y;
        internal Fixed64 VerticalVelocity => _mover3D?.LinearVelocity.Y ?? Mover2D!.LinearVelocity.Y;
        internal bool HasSupportingContact
        {
            get
            {
                if (_kind != 0) return _hasMixedSupport && Context.MixedCollisions.ActivePairCount > 0;
                return (_floor2D!.TryGetCollisionPair(Mover2D!.Collider.Id, out var pair)
                    || Mover2D.Collider.TryGetCollisionPair(_floor2D.Id, out pair)) && pair!.Manifold.HasContact;
            }
        }

        internal void ChangeSupport(int change)
        {
            if (_floor3D != null)
            {
                if (change == 2) { if (_floor3D.Body != null) _floor3D.Body.Deactivate(); else _floor3D.Deactivate(); }
                else if (change == 1) _floor3D.Size = new Vector3d((Fixed64)8, Fixed64.One, Fixed64.FromFraction(1, 16));
                else if (_floor3D.Body != null) _floor3D.Body.SetPosition(_floor3D.Center + Vector3d.Right * (Fixed64)12);
                else _floor3D.Position = _floor3D.Center + Vector3d.Right * (Fixed64)12;
                if (change != 2) _floor3D.Simulate();
                return;
            }

            if (change == 2) { if (_floor2D!.Body != null) _floor2D.Body.Deactivate(); else _floor2D.Deactivate(); }
            else
            {
                if (change == 1)
                {
                    if (_kind == 1) _floor2D!.MixedHalfThicknessOverride = Fixed64.FromFraction(1, 16);
                    else _floor2D!.Size = new Vector2d((Fixed64)8, Fixed64.FromFraction(1, 16));
                }
                else if (_floor2D!.Body != null) _floor2D.Body.SetPosition(_floor2D.Center + Vector2d.Right * (Fixed64)12);
                else _floor2D.Agent.Transform.TrySetWorldPosition(
                    _floor2D.Agent.Transform.WorldPosition + Vector3d.Right * (Fixed64)12).Should().BeTrue();
                _floor2D!.Simulate();
            }
        }

        internal void MoveTriggerAway()
        {
            if (_trigger3D != null)
            {
                _trigger3D.Position = _trigger3D.Center + Vector3d.Right * (Fixed64)12;
                _trigger3D.Simulate();
            }
            else
            {
                _trigger!.Agent.Transform.TrySetWorldPosition(
                    _trigger.Agent.Transform.WorldPosition + Vector3d.Right * (Fixed64)12).Should().BeTrue();
                _trigger.Simulate();
            }
        }

        internal void ShiftSupport()
        {
            Vector3d shift = Vector3d.Right * Fixed64.FromFraction(1, 64);
            if (_floor3D != null)
            {
                _floor3D.Position = _floor3D.Center + shift;
                _floor3D.Simulate();
            }
            else
            {
                _floor2D!.Agent.Transform.TrySetWorldPosition(_floor2D.Agent.Transform.WorldPosition + shift)
                    .Should().BeTrue();
                _floor2D.Simulate();
            }
        }

        internal void MakeSupportTrigger(bool value)
        {
            if (_floor3D != null) _floor3D.IsTrigger = value;
            else _floor2D!.IsTrigger = value;
        }

        internal void SetSupportMaterial(PhysicsMaterial material)
        {
            if (_floor3D != null) _floor3D.Material = material;
            else _floor2D!.Material = material;
        }

        internal void SleepMover()
        {
            if (_mover3D != null) { _mover3D.SleepEnabled = true; _mover3D.Sleep(); }
            else { Mover2D!.SleepEnabled = true; Mover2D.Sleep(); }
        }

        internal void Step() { Context.Simulate(); Context.LateSimulate(); }
        public void Dispose() => _scene.Dispose();
        private TestMatterAgent CreateAgent(Vector3d position) => new(Context,
            new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
    }
}
