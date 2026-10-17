using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedResponseTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CachedMixedCandidate_WhenEarlierCallbackInvalidatesIt_ShouldNotPublishStaleContact(bool move)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> spatial = CreateSphere3D(context, Vector3d.Up * Fixed64.FromFraction(3, 4));
        spatial.Body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.PositionZ | BodyFreezeAxes3D.Rotation;
        SolidBody2D first = CreateCircle2D(context, Vector2d.Zero, immovable: true);
        SolidBody2D later = CreateCircle2D(context, Vector2d.Zero);
        later.FreezeAxes = BodyFreezeAxes2D.All;
        later.SleepEnabled = false;
        Step(context);
        context.MixedCollisions.ActivePairCount.Should().Be(2);
        int stayed = 0, exited = 0;
        later.Collider.OnMixedContact += _ => stayed++;
        later.Collider.OnMixedContactExit += _ => exited++;
        first.Collider.OnMixedContact += _ =>
        {
            if (move)
            {
                later.SetPosition(Vector2d.Right * (Fixed64)6);
                later.Collider.Simulate();
            }
            else later.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(spatial.Collider.Layer);
        };

        Step(context);

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(2);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
        stayed.Should().Be(0);
        exited.Should().Be(1);
    }

    [Fact]
    public void CachedMixedManifold_WithUnchangedParticipants_ShouldRepublishExactRowsAndClearStepImpulses()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var fixture = CreateCachedMixedPair(context);
        var copied = new ContactManifold();
        fixture.Pair.TryCopyCachedManifold(2, copied).Should().BeTrue();
        copied.LastUpdatedFrame.Should().Be(2);
        copied.GroupCount.Should().Be(fixture.Source.GroupCount);
        copied.Count.Should().Be(fixture.Source.Count);
        for (int index = 0; index < copied.Count; index++)
            copied[index].Should().Be(fixture.Source[index]);
        fixture.Source.Reset();
        fixture.Pair.Manifold.HasContact.Should().BeTrue();
        fixture.Pair.SolveImpulses.Set(copied[0].ContactId, copied[0].Normal, Fixed64.One, Fixed64.Zero);
        var exactImpulse = new ExactContactImpulseState();
        var unit = new ulong[ExactContactImpulseState.Words]; unit[0] = 1;
        exactImpulse.SetNormal(unit, unit);
        fixture.Pair.ExactSolveImpulses = new[] { exactImpulse };
        fixture.Pair.ResponseSnapshot = new ContactResponseSnapshot(fixture.Spatial, null,
            Vector3d.Right, Vector3d.Zero, Vector3d.Zero, Vector3d.Zero);

        fixture.Pair.MarkColliding(2, copied);

        fixture.Pair.LastFrame.Should().Be(2);
        fixture.Pair.Manifold.LastUpdatedFrame.Should().Be(2);
        fixture.Pair.SolveImpulses.Count.Should().Be(0);
        exactImpulse.HasNormal.Should().BeFalse();
        fixture.Pair.ResponseSnapshot.IsValid.Should().BeFalse();
        fixture.Pair.TryCopyCachedManifold(3, copied).Should().BeTrue();
        copied.LastUpdatedFrame.Should().Be(3);
    }

    [Theory]
    [InlineData("2D translation")]
    [InlineData("2D yaw")]
    [InlineData("2D integrated translation")]
    [InlineData("2D integrated yaw")]
    [InlineData("2D geometry")]
    [InlineData("2D material")]
    [InlineData("2D thickness")]
    [InlineData("world thickness")]
    [InlineData("3D translation")]
    [InlineData("3D rotation")]
    [InlineData("3D geometry")]
    [InlineData("3D material")]
    public void CachedMixedManifold_AfterPhysicalDependencyChanges_ShouldRequireFreshGeometry(string change)
    {
        using GravitasWorldContext context = CreateMixedContext();
        var fixture = CreateCachedMixedPair(context);
        var copied = new ContactManifold();
        fixture.Pair.TryCopyCachedManifold(2, copied).Should().BeTrue();
        switch (change)
        {
            case "2D translation": fixture.Planar.SetPosition(Vector2d.Right * Fixed64.FromFraction(1, 1024)); break;
            case "2D yaw": fixture.Planar.SetRotation(Fixed64.FromFraction(1, 1024)); break;
            case "2D integrated translation":
                fixture.Planar.AddLinearImpulse(Vector2d.Right * Fixed64.FromFraction(1, 1024));
                fixture.Planar.LateSimulate(updateSleepState: false, updateColliderState: true);
                fixture.Planar.Position.Should().NotBe(Vector2d.Zero);
                break;
            case "2D integrated yaw":
                fixture.Planar.AddAngularImpulse(Fixed64.FromFraction(1, 1024));
                fixture.Planar.LateSimulate(updateSleepState: false, updateColliderState: true);
                fixture.Planar.Rotation.Should().NotBe(Fixed64.Zero);
                break;
            case "2D geometry": ((LSCapsuleCollider2D)fixture.Planar.Collider).Radius = Fixed64.FromFraction(3, 4); break;
            case "2D material": fixture.Planar.Collider.Material = PhysicsMaterial.Bouncy; break;
            case "2D thickness": fixture.Planar.Collider.MixedHalfThicknessOverride = Fixed64.One; break;
            case "world thickness": context.Settings.Mixed2DHalfThickness = Fixed64.One; break;
            case "3D translation": fixture.Spatial.SetPosition(fixture.Spatial.Position3d + Vector3d.Right * Fixed64.FromFraction(1, 1024)); break;
            case "3D rotation": fixture.Spatial.SetRotation(FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.FromFraction(1, 1024))); break;
            case "3D geometry": fixture.Spatial.Collider.Size = new Vector3d(2, 1, 1); break;
            case "3D material": fixture.Spatial.Collider.Material = PhysicsMaterial.Bouncy; break;
            default: throw new ArgumentOutOfRangeException(nameof(change));
        }
        fixture.Spatial.Collider.Simulate();
        fixture.Planar.Collider.Simulate();

        fixture.Pair.TryCopyCachedManifold(3, copied).Should().BeFalse(change);
        CollisionDetectionMixed.TryBuildManifold(fixture.Spatial.Collider, fixture.Planar.Collider, copied).Should().BeTrue();
        fixture.Pair.MarkColliding(3, copied);
        fixture.Pair.TryCopyCachedManifold(4, copied).Should().BeTrue();
    }

    [Theory]
    [InlineData("scalar resting")]
    [InlineData("scalar contact")]
    [InlineData("separation")]
    [InlineData("reinitialize")]
    public void CachedMixedManifold_AfterPairStateReplacement_ShouldNotReuseOldRows(string change)
    {
        using GravitasWorldContext context = CreateMixedContext();
        var fixture = CreateCachedMixedPair(context);
        var copied = new ContactManifold();
        fixture.Pair.TryCopyCachedManifold(2, copied).Should().BeTrue();
        switch (change)
        {
            case "scalar resting": fixture.Pair.MarkResting(2); break;
            case "scalar contact": fixture.Pair.MarkColliding(2, fixture.Pair.Contact); break;
            case "separation": fixture.Pair.MarkSeparated(); break;
            case "reinitialize": fixture.Pair.Initialize(fixture.Spatial.Collider, fixture.Planar.Collider); break;
            default: throw new ArgumentOutOfRangeException(nameof(change));
        }
        fixture.Pair.TryCopyCachedManifold(3, copied).Should().BeFalse(change);
    }

    [Fact]
    public void CachedMixedManifold_WhenContactCallbackMovesParticipant_ShouldRequireFreshGeometry()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var fixture = CreateCachedMixedPair(context);
        fixture.Spatial.Collider.OnMixedContact += _ =>
            fixture.Planar.SetPosition(Vector2d.Right * Fixed64.FromFraction(1, 1024));

        fixture.Pair.MarkColliding(2, fixture.Source);

        fixture.Planar.Position.Should().Be(Vector2d.Right * Fixed64.FromFraction(1, 1024));
        fixture.Pair.TryCopyCachedManifold(3, new ContactManifold()).Should().BeFalse();
    }

    private static (SolidBody Spatial, SolidBody2D Planar, CollisionPairMixed Pair, ContactManifold Source)
        CreateCachedMixedPair(GravitasWorldContext context)
    {
        var spatialCollider = new LSCuboidCollider { Size = Vector3d.One, Material = PhysicsMaterial.Frictionless };
        Vector3d position = Vector3d.Up * Fixed64.FromFraction(3, 4);
        var spatial = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One)), spatialCollider) { Mass = Fixed64.One };
        spatial.Initialize(position, FixedQuaternion.Identity);
        var planarCollider = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)2) { Material = PhysicsMaterial.Frictionless };
        var planar = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), planarCollider) { Mass = Fixed64.One };
        planar.Initialize(Vector2d.Zero);
        spatialCollider.Simulate();
        planarCollider.Simulate();
        var source = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(spatialCollider, planarCollider, source).Should().BeTrue();
        var pair = new CollisionPairMixed(spatialCollider, planarCollider);
        pair.MarkColliding(1, source);
        return (spatial, planar, pair, source);
    }

    [Fact]
    public void Simulate_WithSeparatedFormerMixedPair_ShouldEmitExitAndRecyclePair()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        int exited3D = 0;
        int exited2D = 0;
        body3D.Collider.OnMixedContactExit += other =>
        {
            other.Should().BeSameAs(body2D.Collider);
            exited3D++;
        };
        body2D.Collider.OnMixedContactExit += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            exited2D++;
        };

        Step(context);
        body3D.Body.SetPosition(new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero));
        Step(context);

        exited3D.Should().Be(1);
        exited2D.Should().Be(1);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
        context.MixedCollisions.PooledPairCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void Deactivate2DParticipant_WithActiveMixedPair_ShouldEmitExitAndRecyclePair()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        int exited3D = 0;
        int exited2D = 0;
        body3D.Collider.OnMixedContactExit += other =>
        {
            other.Should().BeSameAs(body2D.Collider);
            exited3D++;
        };
        body2D.Collider.OnMixedContactExit += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            exited2D++;
        };

        Step(context);
        body2D.Deactivate();

        exited3D.Should().Be(1);
        exited2D.Should().Be(1);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
        context.MixedCollisions.PooledPairCount.Should().BeGreaterThan(0);
        body2D.Collider.IsActive.Should().BeFalse();
    }

    [Fact]
    public void Deactivate3DParticipant_WithActiveMixedPair_ShouldEmitExitAndRecyclePair()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        int exited3D = 0;
        int exited2D = 0;
        body3D.Collider.OnMixedContactExit += other =>
        {
            other.Should().BeSameAs(body2D.Collider);
            exited3D++;
        };
        body2D.Collider.OnMixedContactExit += other =>
        {
            other.Should().BeSameAs(body3D.Collider);
            exited2D++;
        };

        Step(context);
        body3D.Body.Deactivate();

        exited3D.Should().Be(1);
        exited2D.Should().Be(1);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
        context.MixedCollisions.PooledPairCount.Should().BeGreaterThan(0);
        body3D.Collider.IsActive.Should().BeFalse();
    }

    [Fact]
    public void Deactivate3DParticipant_WhenExitRebinds2DParticipant_ShouldNotNotifyNewLifetime()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        Vector2d reboundPosition = new((Fixed64)8, Fixed64.Zero);
        string events3D = string.Empty;
        string events2D = string.Empty;
        bool rebound = false;
        body3D.Collider.OnMixedContactExit += _ =>
        {
            events3D += "exit;";
            body2D.Deactivate();
            body2D.Initialize(reboundPosition);
            rebound = true;
        };
        body2D.Collider.OnMixedContactExit += _ => events2D += rebound ? "new-exit;" : "old-exit;";

        Step(context);
        body3D.Body.Deactivate();

        events3D.Should().Be("exit;");
        events2D.Should().BeEmpty();
        body2D.Active.Should().BeTrue();
        body2D.Position.Should().Be(reboundPosition);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void Deactivate3DParticipant_WhenExitDeactivates2DParticipant_ShouldFinishBothLifecycles()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        int exited3D = 0;
        int exited2D = 0;
        body3D.Collider.OnMixedContactExit += _ =>
        {
            exited3D++;
            body2D.Deactivate();
        };
        body2D.Collider.OnMixedContactExit += _ => exited2D++;

        Step(context);
        body3D.Body.Deactivate();

        exited3D.Should().Be(1);
        exited2D.Should().Be(1);
        body2D.Active.Should().BeFalse();
        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void MixedEnter_WhenCallbackDeactivatesParticipant_ShouldDeferExitUntilCallbackReturns()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        string events = string.Empty;
        int entered2D = 0;
        int exited2D = 0;
        body3D.Collider.OnMixedContactEnter += _ =>
        {
            events += "enter-start;";
            body3D.Body.Deactivate();
            events += "enter-end;";
        };
        body3D.Collider.OnMixedContactExit += _ => events += "exit;";
        body3D.Collider.OnMixedContact += _ => events += "stay;";
        body2D.Collider.OnMixedContactEnter += _ => entered2D++;
        body2D.Collider.OnMixedContactExit += _ => exited2D++;

        Step(context);

        events.Should().Be("enter-start;enter-end;exit;");
        entered2D.Should().Be(0);
        exited2D.Should().Be(0);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
    }

    [Fact]
    public void MarkColliding_WhenDeferredExitCallbacksThrow_ShouldRetainGuardAndAggregateInDimensionOrder()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        int exits3D = 0;
        int exits2D = 0;
        bool guarded3D = false;
        bool guarded2D = false;
        body2D.Collider.OnMixedContactEnter += _ => pair.MarkSeparated();
        body3D.Collider.OnMixedContactExit += _ =>
        {
            exits3D++;
            guarded3D = pair.IsNotificationInProgress;
            throw new InvalidOperationException("3D exit failure");
        };
        body2D.Collider.OnMixedContactExit += _ =>
        {
            exits2D++;
            guarded2D = pair.IsNotificationInProgress;
            throw new ArgumentException("2D exit failure");
        };

        Action notify = () => pair.MarkColliding(frame: 1, contact: default);

        AggregateException exception = notify.Should().Throw<AggregateException>().Which.Flatten();
        exception.InnerExceptions.Should().HaveCount(2);
        exception.InnerExceptions[0].Should().BeOfType<InvalidOperationException>()
            .Which.Message.Should().Be("3D exit failure");
        exception.InnerExceptions[1].Should().BeOfType<ArgumentException>()
            .Which.Message.Should().Be("2D exit failure");
        exits3D.Should().Be(1);
        exits2D.Should().Be(1);
        guarded3D.Should().BeTrue();
        guarded2D.Should().BeTrue();
        pair.IsNotificationInProgress.Should().BeFalse();

        Action retry = pair.MarkSeparated;

        retry.Should().NotThrow();
        exits3D.Should().Be(1);
        exits2D.Should().Be(1);
    }

    [Fact]
    public void MarkColliding_When3DEnterThrows_ShouldResetGuardAndRetainOnlyDeliveredAdmission()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        var pair = new CollisionPairMixed(body3D.Collider, body2D.Collider);
        int entered2D = 0;
        int exited3D = 0;
        int exited2D = 0;
        body3D.Collider.OnMixedContactEnter += _ => throw new InvalidOperationException("3D enter failure");
        body2D.Collider.OnMixedContactEnter += _ => entered2D++;
        body3D.Collider.OnMixedContactExit += _ => exited3D++;
        body2D.Collider.OnMixedContactExit += _ => exited2D++;

        Action notify = () => pair.MarkColliding(frame: 1, contact: default);

        notify.Should().Throw<InvalidOperationException>().WithMessage("3D enter failure");
        entered2D.Should().Be(0);
        pair.IsNotificationInProgress.Should().BeFalse();

        pair.MarkSeparated();

        exited3D.Should().Be(1);
        exited2D.Should().Be(0);
    }
}
