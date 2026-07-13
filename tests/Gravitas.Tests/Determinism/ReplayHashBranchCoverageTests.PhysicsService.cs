//=======================================================================
// ReplayHashBranchCoverageTests.PhysicsService.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Determinism;

public sealed partial class ReplayHashBranchCoverageTests
{
    [Fact]
    public void Physics2DReplayHash_ShouldTreatHandoffOrderAsSolverCacheIdentityOnly()
    {
        using GravitasWorldContext firstOrder = Physics2DTestWorld.CreateContext(frameRate: 1);
        using GravitasWorldContext secondOrder = Physics2DTestWorld.CreateContext(frameRate: 1);
        SolidBody2D firstA = CreateBody2D(firstOrder, Vector2d.Zero);
        SolidBody2D firstB = CreateBody2D(firstOrder, Vector2d.Right * (Fixed64)2);
        SolidBody2D secondA = CreateBody2D(secondOrder, Vector2d.Zero);
        SolidBody2D secondB = CreateBody2D(secondOrder, Vector2d.Right * (Fixed64)2);

        firstA.DynamicId.Should().Be(secondA.DynamicId);
        firstB.DynamicId.Should().Be(secondB.DynamicId);
        firstA.DynamicId.Should().NotBe(firstB.DynamicId);
        firstOrder.Physics2D.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();
        secondOrder.Physics2D.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();

        ApplyPendingHandoff(firstA);
        ApplyPendingHandoff(firstB);
        ApplyPendingHandoff(secondB);
        ApplyPendingHandoff(secondA);

        HashBody2D(firstA, GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should().Be(HashBody2D(secondA, GravitasReplayHashMode.AuthoritativeWithSolverCaches));
        HashBody2D(firstB, GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should().Be(HashBody2D(secondB, GravitasReplayHashMode.AuthoritativeWithSolverCaches));

        ChronicleHash firstAuthoritative = HashPhysics2D(firstOrder, GravitasReplayHashMode.Authoritative);
        ChronicleHash secondAuthoritative = HashPhysics2D(secondOrder, GravitasReplayHashMode.Authoritative);
        ChronicleHash firstCaches = HashPhysics2D(firstOrder, GravitasReplayHashMode.AuthoritativeWithSolverCaches);
        ChronicleHash secondCaches = HashPhysics2D(secondOrder, GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        firstAuthoritative.Should().Be(secondAuthoritative);
        firstCaches.Should().NotBe(secondCaches);
        HashPhysics2D(firstOrder, GravitasReplayHashMode.Authoritative).Should().Be(firstAuthoritative);
        HashPhysics2D(secondOrder, GravitasReplayHashMode.Authoritative).Should().Be(secondAuthoritative);
        HashPhysics2D(firstOrder, GravitasReplayHashMode.AuthoritativeWithSolverCaches).Should().Be(firstCaches);
        HashPhysics2D(secondOrder, GravitasReplayHashMode.AuthoritativeWithSolverCaches).Should().Be(secondCaches);
    }

    [Fact]
    public void Physics3DReplayHash_ShouldTreatHandoffOrderAsSolverCacheIdentityOnly()
    {
        using PhysicsScenarioBuilder firstOrder = PhysicsScenarioBuilder.Create();
        using PhysicsScenarioBuilder secondOrder = PhysicsScenarioBuilder.Create();
        firstOrder.Context.SetFrameRate(1);
        secondOrder.Context.SetFrameRate(1);
        SolidBody firstA = firstOrder.CreateSphere(Vector3d.Zero).Body;
        SolidBody firstB = firstOrder.CreateSphere(Vector3d.Right * (Fixed64)2).Body;
        SolidBody secondA = secondOrder.CreateSphere(Vector3d.Zero).Body;
        SolidBody secondB = secondOrder.CreateSphere(Vector3d.Right * (Fixed64)2).Body;

        firstA.DynamicId.Should().Be(secondA.DynamicId);
        firstB.DynamicId.Should().Be(secondB.DynamicId);
        firstA.DynamicId.Should().NotBe(firstB.DynamicId);
        firstOrder.Context.Physics.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();
        secondOrder.Context.Physics.BeginLateSimulateBodies(continuousCollisionFramePrepared: false).Should().BeTrue();

        ApplyPendingHandoff(firstA);
        ApplyPendingHandoff(firstB);
        ApplyPendingHandoff(secondB);
        ApplyPendingHandoff(secondA);

        HashBody3D(firstA, GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should().Be(HashBody3D(secondA, GravitasReplayHashMode.AuthoritativeWithSolverCaches));
        HashBody3D(firstB, GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should().Be(HashBody3D(secondB, GravitasReplayHashMode.AuthoritativeWithSolverCaches));

        ChronicleHash firstAuthoritative = HashPhysics3D(firstOrder.Context, GravitasReplayHashMode.Authoritative);
        ChronicleHash secondAuthoritative = HashPhysics3D(secondOrder.Context, GravitasReplayHashMode.Authoritative);
        ChronicleHash firstCaches = HashPhysics3D(firstOrder.Context, GravitasReplayHashMode.AuthoritativeWithSolverCaches);
        ChronicleHash secondCaches = HashPhysics3D(secondOrder.Context, GravitasReplayHashMode.AuthoritativeWithSolverCaches);

        firstAuthoritative.Should().Be(secondAuthoritative);
        firstCaches.Should().NotBe(secondCaches);
        HashPhysics3D(firstOrder.Context, GravitasReplayHashMode.Authoritative).Should().Be(firstAuthoritative);
        HashPhysics3D(secondOrder.Context, GravitasReplayHashMode.Authoritative).Should().Be(secondAuthoritative);
        HashPhysics3D(firstOrder.Context, GravitasReplayHashMode.AuthoritativeWithSolverCaches).Should().Be(firstCaches);
        HashPhysics3D(secondOrder.Context, GravitasReplayHashMode.AuthoritativeWithSolverCaches).Should().Be(secondCaches);
    }

    [Fact]
    public void Physics2DReplayHash_WithPairStoredOnFirstCollider_ShouldIncludeRetainedPairState()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        (SolidBody2D first, SolidBody2D second) = CreatePairBodies(context);
        ChronicleHash noPair = HashPhysics2D(context, GravitasReplayHashMode.Authoritative);
        var pair = new CollisionPair2D(first.Collider, second.Collider);
        pair.MarkResting(17);

        first.Collider.TryGetCollisionPair(second.Collider.Id, out _).Should().BeFalse();
        second.Collider.TryGetCollisionPair(first.Collider.Id, out _).Should().BeFalse();
        first.Collider.TryAddCollisionPair(second.Collider.Id, pair).Should().BeTrue();
        first.Collider.TryGetCollisionPair(second.Collider.Id, out CollisionPair2D? stored)
            .Should().BeTrue();
        stored.Should().BeSameAs(pair);

        ChronicleHash withPair = HashPhysics2D(context, GravitasReplayHashMode.Authoritative);
        withPair.Should().NotBe(noPair);
        HashPhysics2D(context, GravitasReplayHashMode.Authoritative).Should().Be(withPair);
    }

    private static void ApplyPendingHandoff(SolidBody2D body) =>
        body.ApplyContinuousCollisionHandoff(
            body.Position,
            Vector2d.Right,
            Fixed64.Half);

    private static void ApplyPendingHandoff(SolidBody body) =>
        body.ApplyContinuousCollisionHandoff(
            body.Position3d,
            Vector3d.Right,
            Fixed64.Half);

    private static (SolidBody2D First, SolidBody2D Second) CreatePairBodies(GravitasWorldContext context) =>
        (CreateBody2D(context, Vector2d.Zero), CreateBody2D(context, Vector2d.Right * Fixed64.Half));

    private static ChronicleHash HashPhysics2D(GravitasWorldContext context, GravitasReplayHashMode mode) =>
        Hash((ref ChronicleHashWriter writer) => context.Physics2D.ContributeReplayHash(ref writer, mode));

    private static ChronicleHash HashPhysics3D(GravitasWorldContext context, GravitasReplayHashMode mode) =>
        Hash((ref ChronicleHashWriter writer) => context.Physics.ContributeReplayHash(ref writer, mode));
}
