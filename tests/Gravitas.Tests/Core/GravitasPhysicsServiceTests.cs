using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections.Diagnostics;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class GravitasPhysicsServiceTests
{
    [Fact]
    public void AssimilateCollider_ShouldAllocateContextLocalColliderIds()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        var colliderA = new LSSphereCollider();
        var colliderB = new LSSphereCollider();

        int idA = contextA.Physics.AssimilateCollider(colliderA);
        int idB = contextB.Physics.AssimilateCollider(colliderB);

        idA.Should().Be(0);
        idB.Should().Be(0);
        colliderA.Id.Should().Be(0);
        colliderB.Id.Should().Be(0);
        colliderA.Context.Should().BeSameAs(contextA);
        colliderB.Context.Should().BeSameAs(contextB);
        contextA.Physics.ColliderCount.Should().Be(1);
        contextB.Physics.ColliderCount.Should().Be(1);
    }

    [Fact]
    public void TryGetColliderById_ShouldResolveOnlyWithinOwningContext()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        var colliderA1 = new LSSphereCollider();
        var colliderA2 = new LSSphereCollider();
        var colliderB1 = new LSSphereCollider();

        int idA1 = contextA.Physics.AssimilateCollider(colliderA1);
        int idA2 = contextA.Physics.AssimilateCollider(colliderA2);
        int idB1 = contextB.Physics.AssimilateCollider(colliderB1);

        contextA.Physics.TryGetColliderById(idA1, out LSCollider? foundA1).Should().BeTrue();
        contextA.Physics.TryGetColliderById(idA2, out LSCollider? foundA2).Should().BeTrue();
        contextB.Physics.TryGetColliderById(idB1, out LSCollider? foundB1).Should().BeTrue();
        contextB.Physics.TryGetColliderById(idA2, out LSCollider? missingInB).Should().BeFalse();
        foundA1.Should().BeSameAs(colliderA1);
        foundA2.Should().BeSameAs(colliderA2);
        foundB1.Should().BeSameAs(colliderB1);
        missingInB.Should().BeNull();
    }

    [Fact]
    public void ColliderRegistration_ShouldUseCompactServiceIndicesAndReusableIds()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var first = new LSSphereCollider();
        var second = new LSSphereCollider();
        var third = new LSSphereCollider();

        int firstId = context.Physics.AssimilateCollider(first);
        int secondId = context.Physics.AssimilateCollider(second);
        int thirdId = context.Physics.AssimilateCollider(third);

        context.Physics.ColliderCount.Should().Be(3);
        context.Physics.TryGetColliderByServiceIndex(0, out LSCollider? firstByIndex).Should().BeTrue();
        context.Physics.TryGetColliderByServiceIndex(1, out LSCollider? secondByIndex).Should().BeTrue();
        context.Physics.TryGetColliderByServiceIndex(2, out LSCollider? thirdByIndex).Should().BeTrue();
        firstByIndex.Should().BeSameAs(first);
        secondByIndex.Should().BeSameAs(second);
        thirdByIndex.Should().BeSameAs(third);

        context.Physics.DessimilateCollider(second);
        var replacement = new LSSphereCollider();
        int replacementId = context.Physics.AssimilateCollider(replacement);

        firstId.Should().Be(0);
        secondId.Should().Be(1);
        thirdId.Should().Be(2);
        replacementId.Should().Be(1);
        second.Id.Should().Be(-1);
        context.Physics.PeakColliderCount.Should().Be(3);
        context.Physics.ColliderCount.Should().Be(3);
        context.Physics.TryGetColliderById(secondId, out LSCollider? replacementById).Should().BeTrue();
        context.Physics.TryGetColliderByServiceIndex(0, out LSCollider? compactFirst).Should().BeTrue();
        context.Physics.TryGetColliderByServiceIndex(1, out LSCollider? compactThird).Should().BeTrue();
        context.Physics.TryGetColliderByServiceIndex(2, out LSCollider? compactReplacement).Should().BeTrue();
        replacementById.Should().BeSameAs(replacement);
        compactFirst.Should().BeSameAs(first);
        compactThird.Should().BeSameAs(third);
        compactReplacement.Should().BeSameAs(replacement);

        context.Physics.DessimilateCollider(second);
        context.Physics.TryGetColliderById(replacementId, out LSCollider? replacementAfterStaleRemoval).Should().BeTrue();
        replacementAfterStaleRemoval.Should().BeSameAs(replacement);
    }

    [Fact]
    public void TryGetColliderByServiceIndex_ShouldRejectOutOfRangeIndicesFor3DAnd2DRegistries()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider3D = new LSSphereCollider();
        var collider2D = new LSCircleCollider2D(Fixed64.Half);
        context.Physics.AssimilateCollider(collider3D);
        collider2D.InitializeWithNoBody(new TestMatterAgent(context));

        context.Physics.TryGetColliderByServiceIndex(-1, out LSCollider? negative3D).Should().BeFalse();
        context.Physics.TryGetColliderByServiceIndex(1, out LSCollider? tooHigh3D).Should().BeFalse();
        context.Physics2D.TryGetColliderByServiceIndex(-1, out LSCollider2D? negative2D).Should().BeFalse();
        context.Physics2D.TryGetColliderByServiceIndex(1, out LSCollider2D? tooHigh2D).Should().BeFalse();
        negative3D.Should().BeNull();
        tooHigh3D.Should().BeNull();
        negative2D.Should().BeNull();
        tooHigh2D.Should().BeNull();
    }

    [Fact]
    public void ColliderIsStatic_ShouldBeTrueForBodylessAndPositionFrozenColliders()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var bodyless = new LSSphereCollider();
        bodyless.InitializeWithNoBody(new TestMatterAgent(context));
        SolidBody body = CreateInitializedBody(context);
        SolidBody nonDynamicBody = CreateInitializedBody(context, isDynamic: false);

        bodyless.IsStatic.Should().BeTrue();
        body.Collider.IsStatic.Should().BeFalse();
        nonDynamicBody.Collider.IsStatic.Should().BeTrue();

        body.FreezeAxes = BodyFreezeAxes3D.Position;
        body.Collider.IsStatic.Should().BeTrue();

        body.FreezeAxes = BodyFreezeAxes3D.None;
        body.Collider.IsStatic.Should().BeFalse();
        body.IsKinematic = true;
        body.Collider.IsStatic.Should().BeFalse();
    }

    [Fact]
    public void CollisionPair_ShouldRejectCollidersFromDifferentContexts()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        var colliderA = new LSSphereCollider();
        var colliderB = new LSSphereCollider();
        contextA.Physics.AssimilateCollider(colliderA);
        contextB.Physics.AssimilateCollider(colliderB);

        Action crossContextPair = () => _ = new CollisionHandling.CollisionPair(colliderA, colliderB);

        crossContextPair.Should()
            .Throw<ArgumentException>()
            .WithMessage("*same context*");
    }

    [Fact]
    public void DessimilateCollider_ShouldClear3DPairReferencesBeforeReusingId()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * Fixed64.Half);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();
        first.Collider.TryGetCollisionPair(second.Collider.Id, out _).Should().BeTrue();
        int firstId = first.Collider.Id;

        scenario.Context.Physics.DessimilateCollider(first.Collider);
        ScenarioBody<LSSphereCollider> replacement = scenario.CreateSphere(Vector3d.Up * (Fixed64)4);

        first.Collider.Id.Should().Be(-1);
        replacement.Collider.Id.Should().Be(firstId);
        second.Collider.CollisionPairHolderCount.Should().Be(0);
    }

    [Fact]
    public void SolidBodyInitialize_ShouldRegisterBodyAndColliderWithContextPhysics()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider();
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        body.Context.Should().BeSameAs(context);
        collider.Context.Should().BeSameAs(context);
        body.DynamicId.Should().Be(0);
        collider.Id.Should().Be(0);
        context.Physics.BodyCount.Should().Be(1);
        context.Physics.ColliderCount.Should().Be(1);
        context.Physics.TryGetColliderById(collider.Id, out LSCollider? resolved).Should().BeTrue();
        resolved.Should().BeSameAs(collider);
    }

    [Fact]
    public void Reset_ShouldClearContextLocalColliderTable()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new LSSphereCollider();
        int id = context.Physics.AssimilateCollider(collider);

        context.Physics.Reset();

        context.Physics.ColliderCount.Should().Be(0);
        context.Physics.TryGetColliderById(id, out LSCollider? resolved).Should().BeFalse();
        resolved.Should().BeNull();
    }

    [Fact]
    public void ContextReset_ShouldClear3DPartitionAndColliderRegistryState()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        int id = sphere.Collider.Id;

        scenario.Context.Collisions.ActivePartitionCount.Should().BeGreaterThan(0);
        scenario.Context.Collisions.RetainedPartitionCount.Should().BeGreaterThan(0);
        sphere.Collider.IsPartitioned.Should().BeTrue();

        scenario.Context.Reset();

        scenario.Context.Collisions.ActivePartitionCount.Should().Be(0);
        scenario.Context.Collisions.RetainedPartitionCount.Should().Be(0);
        scenario.Context.Collisions.InactivePartitionCount.Should().Be(0);
        scenario.Context.Physics.ColliderCount.Should().Be(0);
        scenario.Context.Physics.TryGetColliderById(id, out LSCollider? resolved).Should().BeFalse();
        resolved.Should().BeNull();
        sphere.Collider.IsPartitioned.Should().BeFalse();
        (sphere.Collider.PartitionCoordinates?.Count ?? 0).Should().Be(0);
        sphere.Collider.Id.Should().Be(-1);
    }

    [Fact]
    public void DessimilateBody_AfterReset_ShouldNotUnderflowBodyCount()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider();
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        context.Physics.Reset();
        body.Deactivate();

        context.Physics.BodyCount.Should().Be(0);
    }

    [Fact]
    public void LateSimulate_WhenPhysicsIsDisabled_ShouldNotAdvanceBodiesOrLateStepToken()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        SolidBody body = CreateInitializedBody(context);
        body.AddForce(Vector3d.Right);
        context.Physics.SimulatePhysics = false;

        context.Physics.LateSimulate();

        body.Position3d.Should().Be(Vector3d.Zero);
        body.LinearVelocity.Should().Be(Vector3d.Zero);
        context.LateSimulateToken.Should().Be(0);
    }

    [Fact]
    public void Visualize_WithRemovedEarlierBody_ShouldStillPublishLaterBody()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        SolidBody removed = CreateInitializedBody(context);
        SolidBody retained = CreateInitializedBody(context);
        retained.CanSetVisualPosition = true;
        retained.SetPosition(Vector3d.Right);
        retained.CheckChangedValues();
        retained.SetVisualPosition(Vector3d.Right);
        removed.Deactivate();

        context.Visualize();

        context.Physics.BodyCount.Should().Be(1);
        retained.PositionTransform.LocalPosition.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void DessimilateCollider_WithUnregisteredCollider_ShouldRespectTheWarningLogGate()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        DiagnosticLevel originalMinimumLevel = GravitasLogger.MinimumLevel;
        Action<DiagnosticLevel, string, string> originalLogHandler = GravitasLogger.LogHandler;
        var entries = new List<(DiagnosticLevel Level, string Message, string Source)>();

        try
        {
            GravitasLogger.MinimumLevel = DiagnosticLevel.Warning;
            GravitasLogger.LogHandler = (level, message, source) => entries.Add((level, message, source));

            context.Physics.DessimilateCollider(new LSSphereCollider());

            entries.Should().Equal((
                DiagnosticLevel.Warning,
                "Object with ID -1 cannot be dessimilated because it is not assimilated.",
                "GravitasPhysicsService.DessimilateCollider"));

            GravitasLogger.MinimumLevel = DiagnosticLevel.Error;
            context.Physics.DessimilateCollider(new LSSphereCollider());

            entries.Should().HaveCount(1);
            context.Physics.ColliderCount.Should().Be(0);
        }
        finally
        {
            GravitasLogger.LogHandler = originalLogHandler;
            GravitasLogger.MinimumLevel = originalMinimumLevel;
        }
    }

    [Fact]
    public void DessimilateCollider_WithForeignSameIdCollider_ShouldPreserveBothContexts()
    {
        using PhysicsScenarioBuilder scenarioA = PhysicsScenarioBuilder.Create();
        using PhysicsScenarioBuilder scenarioB = PhysicsScenarioBuilder.Create();
        GravitasWorldContext contextA = scenarioA.Context;
        GravitasWorldContext contextB = scenarioB.Context;
        LSSphereCollider colliderA = scenarioA.CreateStaticSphere(Vector3d.Zero);
        LSSphereCollider colliderB = scenarioB.CreateStaticSphere(Vector3d.Zero);
        DiagnosticLevel originalMinimumLevel = GravitasLogger.MinimumLevel;
        Action<DiagnosticLevel, string, string> originalLogHandler = GravitasLogger.LogHandler;
        var entries = new List<(DiagnosticLevel Level, string Message, string Source)>();

        try
        {
            GravitasLogger.MinimumLevel = DiagnosticLevel.Warning;
            GravitasLogger.LogHandler = (level, message, source) => entries.Add((level, message, source));

            contextA.Physics.DessimilateCollider(colliderB);

            colliderA.IsPartitioned.Should().BeTrue();
            colliderB.IsPartitioned.Should().BeTrue();
            colliderA.ServiceRefreshIndex.Should().Be(0);
            colliderB.ServiceRefreshIndex.Should().Be(0);
            contextA.Physics.ColliderCount.Should().Be(1);
            contextB.Physics.ColliderCount.Should().Be(1);
            contextA.Physics.TryGetColliderById(0, out LSCollider? resolvedA).Should().BeTrue();
            contextB.Physics.TryGetColliderById(0, out LSCollider? resolvedB).Should().BeTrue();
            resolvedA.Should().BeSameAs(colliderA);
            resolvedB.Should().BeSameAs(colliderB);
            contextA.Query3D.Raycast(
                -Vector3d.Right * (Fixed64)2,
                Vector3d.Right,
                (Fixed64)4,
                out _,
                PhysicsLayerMask.All).Should().BeTrue();
            contextB.Query3D.Raycast(
                -Vector3d.Right * (Fixed64)2,
                Vector3d.Right,
                (Fixed64)4,
                out _,
                PhysicsLayerMask.All).Should().BeTrue();
            entries.Should().Equal((
                DiagnosticLevel.Warning,
                "Object with ID 0 cannot be dessimilated because it is not assimilated.",
                "GravitasPhysicsService.DessimilateCollider"));
        }
        finally
        {
            GravitasLogger.LogHandler = originalLogHandler;
            GravitasLogger.MinimumLevel = originalMinimumLevel;
        }
    }

    private static SolidBody CreateInitializedBody(GravitasWorldContext context, bool isDynamic = true)
    {
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider();
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, isDynamic);
        return body;
    }
}
