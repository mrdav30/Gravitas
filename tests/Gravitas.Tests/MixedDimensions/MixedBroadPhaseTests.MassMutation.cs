using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedBroadPhaseTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MassSetter_AcrossZero_ShouldSynchronizeMixed2DAwakeMembershipAndCandidates(bool initiallyZero)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSSphereCollider target = CreateBodylessSphere3D(context, Vector3d.Zero);
        // A trigger keeps both admission measurements on the same geometry: the
        // test exercises awake routing, rather than solver position correction.
        target.IsTrigger = true;
        var collider = new LSCircleCollider2D(Fixed64.Half);
        var body = new SolidBody2D(new TestMatterAgent(context), collider)
        {
            Mass = initiallyZero ? Fixed64.Zero : Fixed64.One
        };
        body.Initialize(Vector2d.Zero);
        Step(context);
        var coordinates = collider.MixedPartitionCoordinates!;
        int coordinateCount = coordinates.Count;
        int dynamicId = body.DynamicId;
        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(initiallyZero ? 0 : 1);

        body.Mass = initiallyZero ? Fixed64.One : Fixed64.Zero;

        body.MotionType.Should().Be(BodyMotionType.Dynamic);
        body.DynamicId.Should().Be(dynamicId);
        body.CanTranslate.Should().Be(initiallyZero);
        body.CanRotate.Should().Be(initiallyZero);
        body.IsAwakeForCollision.Should().Be(initiallyZero);
        collider.MixedPartitionCoordinates.Should().BeSameAs(coordinates);
        coordinates.Count.Should().Be(coordinateCount);
        coordinateCount.Should().BeGreaterThan(0);
        foreach (WorldVoxelIndex coordinate in coordinates)
        {
            context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsMixedPartition? partition).Should().BeTrue();
            ContainsId(partition!.ContainedDynamic2DObjects, collider.Id).Should().BeTrue();
            ContainsId(partition.ContainedAwakeDynamic2DObjects, collider.Id).Should().Be(initiallyZero);
            partition.Dynamic2DObjectCount.Should().Be(1);
            partition.Kinematic2DObjectCount.Should().Be(0);
            partition.Static2DObjectCount.Should().Be(0);
        }

        Step(context);

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(initiallyZero ? 1 : 0);
        if (initiallyZero)
        {
            context.MixedCollisions.GetCandidate(0).Collider3DId.Should().Be(target.Id);
            context.MixedCollisions.GetCandidate(0).Collider2DId.Should().Be(collider.Id);
        }
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(false, -2)]
    [InlineData(true, -2)]
    public void MassSetter_AcrossNonpositiveMass_ShouldSynchronizeMixed3DAwakeMembershipAndCandidates(
        bool initiallyNonpositive, int nonpositiveMass)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCircleCollider2D target = CreateBodylessCircle2D(context, Vector2d.Zero);
        target.IsTrigger = true;
        var collider = new LSSphereCollider();
        var body = new SolidBody(new TestMatterAgent(context), collider)
        {
            Mass = initiallyNonpositive ? (Fixed64)nonpositiveMass : Fixed64.One
        };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        Step(context);
        var coordinates = collider.MixedPartitionCoordinates!;
        int coordinateCount = coordinates.Count;
        int dynamicId = body.DynamicId;
        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(initiallyNonpositive ? 0 : 1);

        body.Mass = initiallyNonpositive ? Fixed64.One : (Fixed64)nonpositiveMass;

        body.MotionType.Should().Be(BodyMotionType.Dynamic);
        body.DynamicId.Should().Be(dynamicId);
        body.InverseMass.Should().Be(initiallyNonpositive ? Fixed64.One : Fixed64.Zero);
        body.CanTranslate.Should().Be(initiallyNonpositive);
        body.CanRotate.Should().Be(initiallyNonpositive);
        body.IsAwakeForCollision.Should().Be(initiallyNonpositive);
        (body.InverseInertiaTensor == Fixed3x3.Zero).Should().Be(!initiallyNonpositive);
        collider.MixedPartitionCoordinates.Should().BeSameAs(coordinates);
        coordinates.Count.Should().Be(coordinateCount);
        coordinateCount.Should().BeGreaterThan(0);
        foreach (WorldVoxelIndex coordinate in coordinates)
        {
            context.World.TryGetVoxel(coordinate, out Voxel? voxel).Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsMixedPartition? partition).Should().BeTrue();
            ContainsId(partition!.ContainedDynamic3DObjects, collider.Id).Should().BeTrue();
            ContainsId(partition.ContainedAwakeDynamic3DObjects, collider.Id).Should().Be(initiallyNonpositive);
            partition.Dynamic3DObjectCount.Should().Be(1);
            partition.Kinematic3DObjectCount.Should().Be(0);
            partition.Static3DObjectCount.Should().Be(0);
        }

        Step(context);

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(initiallyNonpositive ? 1 : 0);
        if (initiallyNonpositive)
        {
            context.MixedCollisions.GetCandidate(0).Collider3DId.Should().Be(collider.Id);
            context.MixedCollisions.GetCandidate(0).Collider2DId.Should().Be(target.Id);
        }
    }

    [Fact]
    public void MassSetter_WhenMixed3DBodyIsSleeping_ShouldKeepSameMassAsleepAndWakeOnChange()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCircleCollider2D target = CreateBodylessCircle2D(context, Vector2d.Zero);
        target.IsTrigger = true;
        ScenarioBody<LSSphereCollider> body = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        Step(context);
        body.Body.Sleep();
        body.Body.IsSleeping.Should().BeTrue();
        PhysicsMixedPartition partition = GetFirstMixedPartition(context, body.Collider.MixedPartitionCoordinates!);
        ContainsId(partition.ContainedAwakeDynamic3DObjects, body.Collider.Id).Should().BeFalse();

        body.Body.Mass = Fixed64.One;

        body.Body.IsSleeping.Should().BeTrue();
        ContainsId(partition.ContainedAwakeDynamic3DObjects, body.Collider.Id).Should().BeFalse();

        body.Body.Mass = (Fixed64)2;

        body.Body.IsSleeping.Should().BeFalse();
        ContainsId(partition.ContainedDynamic3DObjects, body.Collider.Id).Should().BeTrue();
        ContainsId(partition.ContainedAwakeDynamic3DObjects, body.Collider.Id).Should().BeTrue();
        Step(context);
        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(1);
    }

}
