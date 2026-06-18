using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Diagnostics;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed class MixedQueryCcdTests
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    [Fact]
    public void SweepSphereAgainst2D_ShouldHitEmbeddedSlabWithoutChangingPure3DQuerySurface()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D platform = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)4, (Fixed64)4));
        var pureHits = new SwiftList<Physics3DHit>();

        int pureCount = context.Query3D.SweepSphereAll(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            pureHits);
        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        pureCount.Should().Be(0);
        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(platform);
        hit.Collider3D.Should().BeNull();
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal3DTo2D.Should().Be(-Vector3d.Up);
    }

    [Fact]
    public void SweepSphereAgainst2D_ShouldReturnCompound2DOwnerThroughPartGeometry()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D compound = CreateBodylessCompound2D(context, Vector2d.Zero);

        bool mixedHit = context.QueryMixed.SweepSphereAgainst2D(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider2D.Should().BeSameAs(compound);
        hit.Collider3D.Should().BeNull();
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal3DTo2D.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void SweepCircleAgainst3D_ShouldHit3DPrimitiveWithoutChangingPure2DQuerySurface()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> target = CreateSphere3D(context, Vector3d.Zero, immovable: true);
        var pureHits = new SwiftList<Physics2DHit>();

        int pureCount = context.Query2D.SweepCircleAll(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            IncludeLayerZero,
            pureHits);
        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        pureCount.Should().Be(0);
        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(target.Collider);
        hit.Collider2D.Should().BeNull();
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal3DTo2D.Should().Be(-Vector3d.Right);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithTallSlabAndPlanarSeparation_ShouldRejectProxyOnlySphereHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateSphere3D(context, Vector3d.Zero, immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), (Fixed64)2),
            new Vector2d((Fixed64)3, (Fixed64)2),
            Fixed64.Half,
            Fixed64.Zero,
            (Fixed64)2,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainst3D_NearSlabCorner_ShouldUseVerticalOverlapToReducePlanarSphereReach()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(9, 10), Fixed64.Zero),
            immovable: true);
        var hits = new SwiftList<PhysicsMixedHit>();

        int count = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(9, 10)),
            new Vector2d((Fixed64)3, Fixed64.FromFraction(9, 10)),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            hits);

        count.Should().Be(0);
        hits.Count.Should().Be(0);
    }

    [Fact]
    public void SweepCircleAgainst3D_ShouldHitMeshTargetThroughTriangleGeometry()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            MeshTestFixtures.CreateVerticalQuad(Fixed64.Zero, -Fixed64.One, Fixed64.One),
            Vector3d.Zero,
            immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.One,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(mesh.Collider);
        hit.Distance.Should().Be(Fixed64.FromFraction(5, 2));
        hit.Point3D.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));
        hit.Normal3DTo2D.Should().Be(-Vector3d.Right);
    }

    [Fact]
    public void SweepCircleAgainst3D_ShouldHitCompoundTargetThroughEarliestPartGeometry()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCompoundCollider> compound = CreateCompound3D(context, Vector3d.Zero, immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-3), Fixed64.Zero),
            new Vector2d((Fixed64)3, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(compound.Collider);
        hit.Distance.Should().Be(Fixed64.One);
        hit.Normal3DTo2D.Should().Be(-Vector3d.Right);
    }

    [Fact]
    public void SweepCircleAgainst3D_WithStartingOverlapInsideCompoundPart_ShouldReturnStableHit()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCompoundCollider> compound = CreateCompound3D(context, Vector3d.Zero, immovable: true);

        bool mixedHit = context.QueryMixed.SweepCircleAgainst3D(
            new Vector2d((Fixed64)(-1), Fixed64.Zero),
            new Vector2d(Fixed64.One, Fixed64.Zero),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Half,
            IncludeLayerZero,
            out PhysicsMixedHit hit);

        mixedHit.Should().BeTrue();
        hit.Collider3D.Should().BeSameAs(compound.Collider);
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Point3D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        hit.Normal3DTo2D.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_WithMixed3DContinuousCollision_ShouldClampBeforeCrossing2DSlab()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        ScenarioBody<LSSphereCollider> falling = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero));
        _ = CreateBodylessBox2D(context, Vector2d.Zero, new Vector2d((Fixed64)4, (Fixed64)4));

        falling.Body.AddForce(Vector3d.Down * (Fixed64)10);
        context.Simulate();
        context.LateSimulate();

        falling.Body.Position3d.Y.Should().BeGreaterThanOrEqualTo(Fixed64.One);
        falling.Body.LinearVelocity.Y.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_WithMixed2DContinuousCollision_ShouldClampBeforeCrossing3DPrimitive()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        _ = CreateSphere3D(context, Vector3d.Zero, immovable: true);
        StiffBody2D moving2D = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));

        moving2D.AddForce(Vector2d.Right * (Fixed64)10);
        context.Simulate();
        context.LateSimulate();

        moving2D.Position.X.Should().BeLessThanOrEqualTo((Fixed64)(-1));
        moving2D.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_WithMixed2DContinuousCollision_ShouldClampBeforeCrossing3DMesh()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        _ = CreateMesh3D(
            context,
            MeshTestFixtures.CreateVerticalQuad(Fixed64.Zero, -Fixed64.One, Fixed64.One),
            Vector3d.Zero,
            immovable: true);
        StiffBody2D moving2D = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));

        moving2D.AddForce(Vector2d.Right * (Fixed64)10);
        context.Simulate();
        context.LateSimulate();

        moving2D.Position.X.Should().BeLessThanOrEqualTo(-Fixed64.Half);
        moving2D.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_WithMixed2DContinuousCollision_ShouldClampBeforeCrossing3DCompound()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.DefaultContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        _ = CreateCompound3D(context, Vector3d.Zero, immovable: true);
        StiffBody2D moving2D = CreateCircle2D(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));

        moving2D.AddForce(Vector2d.Right * (Fixed64)10);
        context.Simulate();
        context.LateSimulate();

        moving2D.Position.X.Should().BeLessThanOrEqualTo((Fixed64)(-2));
        moving2D.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void LateSimulate_WithMixedDynamicContinuousCollision_ShouldClampBothAtSharedTimeOfImpact()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        StiffBody2D body2D = CreateCircle2D(context, new Vector2d((Fixed64)5, Fixed64.Zero));
        body3D.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body2D.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        body3D.Body.AddForce(Vector3d.Right * (Fixed64)5);
        body2D.AddForce(-Vector2d.Right * (Fixed64)5);
        context.LateSimulate();

        body3D.Body.Position3d.X.Should().BeLessThanOrEqualTo(-Fixed64.Half);
        body2D.Position.X.Should().BeGreaterThanOrEqualTo(Fixed64.Half);
        (body2D.Position.X - body3D.Body.Position3d.X).Should().BeGreaterThanOrEqualTo(Fixed64.One);
        body3D.Body.LinearVelocity.X.Should().Be(Fixed64.Zero);
        body2D.LinearVelocity.X.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void MixedDiagnostics_ShouldRecordContactResponseAndDimensionTaggedPayloads()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(
            context,
            new Vector3d(-Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        StiffBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body3D.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right);
        context.Diagnostics.Enable(eventCapacity: 16, drawCommandCapacity: 0);

        context.Simulate();

        ReadOnlySpan<GravitasDiagnosticEvent> events = context.Diagnostics.Events;
        events[0].Kind.Should().Be(GravitasDiagnosticEventKind.MixedContact);
        events[0].ColliderADimension.Should().Be(GravitasColliderDimension.ThreeD);
        events[0].ColliderBDimension.Should().Be(GravitasColliderDimension.TwoD);
        events[0].ColliderAId.Should().Be(body3D.Collider.Id);
        events[0].ColliderBId.Should().Be(body2D.Collider.Id);
        events[0].ColliderAType.Should().Be(ColliderType.Sphere);
        events[0].ColliderB2DType.Should().Be(ColliderType2D.Circle);
        events[0].ScalarA.Should().BeGreaterThan(Fixed64.Zero);
        events[0].Hit.Should().BeTrue();
        events[1].Kind.Should().Be(GravitasDiagnosticEventKind.MixedResponseImpulse);
        events[1].ColliderADimension.Should().Be(GravitasColliderDimension.ThreeD);
        events[1].ColliderBDimension.Should().Be(GravitasColliderDimension.TwoD);
        events[1].Vector.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
        events[1].ScalarA.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void CaptureMixedCollider_ShouldDrawEmbeddedSlabGeometry()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D circle = CreateBodylessCircle2D(context, new Vector2d((Fixed64)2, (Fixed64)3));
        LSCollider2D box = CreateBodylessBox2D(
            context,
            new Vector2d((Fixed64)(-2), (Fixed64)(-3)),
            new Vector2d((Fixed64)4, (Fixed64)2));
        context.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 4);

        context.Diagnostics.CaptureMixedCollider(circle, GravitasDiagnosticColor.Cyan);
        context.Diagnostics.CaptureMixedCollider(box, GravitasDiagnosticColor.Yellow);

        ReadOnlySpan<GravitasDebugDrawCommand> commands = context.Diagnostics.DrawCommands;
        commands.Length.Should().Be(2);
        commands[0].Kind.Should().Be(GravitasDebugDrawKind.WireCylinder);
        commands[0].ColliderDimension.Should().Be(GravitasColliderDimension.TwoD);
        commands[0].Collider2DType.Should().Be(ColliderType2D.Circle);
        commands[0].Center.Should().Be(new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)3));
        commands[0].Height.Should().Be(context.Settings.Mixed2DHalfThickness * 2);
        commands[1].Kind.Should().Be(GravitasDebugDrawKind.WireBox);
        commands[1].ColliderDimension.Should().Be(GravitasColliderDimension.TwoD);
        commands[1].Collider2DType.Should().Be(ColliderType2D.AABox);
        commands[1].Center.Should().Be(new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)(-3)));
        commands[1].Size.Should().Be(new Vector3d((Fixed64)4, context.Settings.Mixed2DHalfThickness * 2, (Fixed64)2));
    }

    private static GravitasWorldContext CreateMixedContext(int frameRate = 4)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.ApplySettings(new PhysicsSettings(frameRate, null));
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-8), (Fixed64)(-4), (Fixed64)(-8)),
                new Vector3d((Fixed64)8, (Fixed64)4, (Fixed64)8)),
            out _).Should().BeTrue();
        return context;
    }

    private static ScenarioBody<LSSphereCollider> CreateSphere3D(
        GravitasWorldContext context,
        Vector3d position,
        bool immovable = false)
    {
        return CreateBody3D(context, new LSSphereCollider(), position, immovable: immovable);
    }

    private static ScenarioBody<LSMeshCollider> CreateMesh3D(
        GravitasWorldContext context,
        LSMeshCollider collider,
        Vector3d position,
        bool immovable = false)
    {
        return CreateBody3D(context, collider, position, immovable: immovable);
    }

    private static ScenarioBody<LSCompoundCollider> CreateCompound3D(
        GravitasWorldContext context,
        Vector3d position,
        bool immovable = false)
    {
        var collider = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)(-1), Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero)));
        return CreateBody3D(context, collider, position, immovable: immovable);
    }

    private static ScenarioBody<TCollider> CreateBody3D<TCollider>(
        GravitasWorldContext context,
        TCollider collider,
        Vector3d position,
        bool immovable = false)
        where TCollider : LSCollider
    {
        var agent = new TestMatterAgent(context, new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = immovable,
            RestitutionCoefficient = Fixed64.Zero
        };
        body.Initialize(position, FixedQuaternion.Identity);
        return new ScenarioBody<TCollider>(body, collider);
    }

    private static StiffBody2D CreateCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        var body = new StiffBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            RestitutionCoefficient = Fixed64.Zero
        };
        body.Initialize(position);
        return body;
    }

    private static LSCollider2D CreateBodylessCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static LSCollider2D CreateBodylessBox2D(
        GravitasWorldContext context,
        Vector2d position,
        Vector2d size)
    {
        var collider = new LSAABBoxCollider2D(size);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }

    private static LSCollider2D CreateBodylessCompound2D(GravitasWorldContext context, Vector2d position)
    {
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)4, Fixed64.Zero)));
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One));
        collider.InitializeWithNoBody(agent);
        return collider;
    }
}
