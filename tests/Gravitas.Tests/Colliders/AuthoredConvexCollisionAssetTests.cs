using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;
using Gravitas.Tests.Support;
using GridForge.Grids;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class AuthoredConvexCollisionAssetTests
{
    [Fact]
    public void CompoundPartFactory_ShouldAuthorShapeAndTransformWithoutRuntimeCollider()
    {
        Vector3d size = new((Fixed64)2, Fixed64.Half, (Fixed64)3);
        Vector3d localOffset = new((Fixed64)(-2), Fixed64.One, Fixed64.Half);
        FixedQuaternion localRotation = PhysicsScenarioBuilder.Yaw(35);
        Vector3d localScale = new(Fixed64.One, (Fixed64)2, Fixed64.Half);

        CompoundColliderPart part = CompoundColliderPart.Cuboid(size, localOffset, localRotation, localScale);

        part.Shape.Kind.Should().Be(ColliderShapeDefinitionKind.Cuboid);
        part.Shape.Size.Should().Be(size);
        part.LocalOffset.Should().Be(localOffset);
        part.LocalRotation.Should().Be(localRotation);
        part.LocalScale.Should().Be(localScale);

        typeof(CompoundColliderPart)
            .GetProperties()
            .Should()
            .NotContain(property => typeof(LSCollider).IsAssignableFrom(property.PropertyType));
    }

    [Fact]
    public void CompoundPartConstructor_ShouldApplyAuthoringTransformAtomically()
    {
        ColliderShapeDefinition shape = ColliderShapeDefinition.Cuboid(new Vector3d((Fixed64)2, Fixed64.One, Fixed64.Half));
        Vector3d localOffset = new((Fixed64)2, Fixed64.Half, (Fixed64)(-1));
        FixedQuaternion localRotation = PhysicsScenarioBuilder.Yaw(35);
        Vector3d localScale = new((Fixed64)2, Fixed64.One, Fixed64.Half);

        var descriptor = new CompoundColliderPart(shape, localOffset, localRotation, localScale);

        descriptor.Shape.Should().Be(shape);
        descriptor.LocalOffset.Should().Be(localOffset);
        descriptor.LocalRotation.Should().Be(localRotation);
        descriptor.LocalScale.Should().Be(localScale);
    }

    [Fact]
    public void AuthoredConvexCompound_ShouldKeepPiecesBehindOneBroadPhaseIdentity()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(
            new CompoundColliderPart(MeshTestFixtures.CreateConvexCubeDefinition(), new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One),
            new CompoundColliderPart(MeshTestFixtures.CreateConvexCubeDefinition(), new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One));

        ScenarioBody<LSCompoundCollider> body = scenario.CreateBody(
            compound,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            preventAngularForces: true);
        LSCollider leftPiece = body.Collider.GetPartCollider(0);
        LSCollider rightPiece = body.Collider.GetPartCollider(1);

        body.Collider.Id.Should().BeGreaterThanOrEqualTo(0);
        leftPiece.BoundsMin.Should().Be(new Vector3d(-Fixed64.FromFraction(3, 2), -Fixed64.Half, -Fixed64.Half));
        leftPiece.BoundsMax.Should().Be(new Vector3d(-Fixed64.Half, Fixed64.Half, Fixed64.Half));
        rightPiece.BoundsMin.Should().Be(new Vector3d(Fixed64.Half, -Fixed64.Half, -Fixed64.Half));
        rightPiece.BoundsMax.Should().Be(new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Half, Fixed64.Half));
        leftPiece.Id.Should().Be(-1);
        rightPiece.Id.Should().Be(-1);
        scenario.Context.Physics.ColliderCount.Should().Be(1);
        scenario.Context.Physics.TryGetColliderById(body.Collider.Id, out LSCollider? ownerById).Should().BeTrue();
        ownerById.Should().BeSameAs(body.Collider);
        scenario.Context.Physics.TryGetColliderById(leftPiece.Id, out _).Should().BeFalse();
        scenario.Context.Physics.TryGetColliderById(rightPiece.Id, out _).Should().BeFalse();

        body.Collider.PartitionCoordinates.Should().NotBeNull();
        for (int i = 0; i < body.Collider.PartitionCoordinates!.Count; i++)
        {
            scenario.Context.World.TryGetVoxel(body.Collider.PartitionCoordinates[i], out Voxel? voxel)
                .Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsPartition? partition).Should().BeTrue();
            partition!.ContainedDynamicObjects!.Contains(body.Collider.Id).Should().BeTrue();
            partition.ContainedDynamicObjects.Contains(leftPiece.Id).Should().BeFalse();
            partition.ContainedDynamicObjects.Contains(rightPiece.Id).Should().BeFalse();
        }
    }

    [Fact]
    public void AuthoredConvexCompound_ShouldReportContactsThroughOwnerOnly()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(
            new CompoundColliderPart(MeshTestFixtures.CreateConvexCubeDefinition(), Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One));
        ScenarioBody<LSCompoundCollider> compoundBody = scenario.CreateBody(
            compound,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            preventAngularForces: true);
        LSCollider meshPiece = compoundBody.Collider.GetPartCollider(0);
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero),
            preventAngularForces: true);
        int ownerContactEnterCount = 0;
        int partContactEnterCount = 0;
        int sphereContactEnterCount = 0;
        compoundBody.Collider.OnContactEnter += _ => ownerContactEnterCount++;
        meshPiece.OnContactEnter += _ => partContactEnterCount++;
        sphere.Collider.OnContactEnter += _ => sphereContactEnterCount++;
        CollisionPair pair = scenario.CreatePair(compoundBody.Collider, sphere.Collider);

        pair.UpdateCollision();
        pair.NotifyCollidersOfContact();

        pair.Manifold.HasContact.Should().BeTrue();
        pair.ColliderA.Id.Should().NotBe(meshPiece.Id);
        pair.ColliderB.Id.Should().NotBe(meshPiece.Id);
        ownerContactEnterCount.Should().Be(1);
        sphereContactEnterCount.Should().Be(1);
        partContactEnterCount.Should().Be(0);
    }

    [Fact]
    public void Diagnostics_ShouldCaptureAuthoredConvexMeshPartsWithOwnerIdentity()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(
            new CompoundColliderPart(MeshTestFixtures.CreateConvexCubeDefinition(), Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One));
        ScenarioBody<LSCompoundCollider> body = scenario.CreateBody(
            compound,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            preventAngularForces: true);

        scenario.Context.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 16);
        scenario.Context.Diagnostics.CaptureCollider(body.Collider, GravitasDiagnosticColor.White);

        ReadOnlySpan<GravitasDebugDrawCommand> commands = scenario.Context.Diagnostics.DrawCommands;
        commands.Length.Should().Be(body.Collider.GetPartCollider(0) is LSMeshCollider mesh ? mesh.Mesh.TriangleCount : 0);
        for (int i = 0; i < commands.Length; i++)
        {
            commands[i].Kind.Should().Be(GravitasDebugDrawKind.WireTriangle);
            commands[i].ColliderId.Should().Be(body.Collider.Id);
            commands[i].ColliderType.Should().Be(ColliderType.Compound);
        }
    }
}
