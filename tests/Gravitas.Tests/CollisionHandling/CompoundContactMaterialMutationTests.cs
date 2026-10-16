using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CompoundContactMaterialMutationTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RetainedContact_ShouldRefreshChangedChildMaterialWithoutWakingForEqualMaterial(bool changed)
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Environment.Gravity = Fixed64.Zero;
        scene.Context.Environment.AirDensity = Fixed64.Zero;
        scene.Context.Environment.DampingFactor = Fixed64.Zero;
        scene.Context.Environment.MinSpeed = Fixed64.Zero;
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero, PhysicsMaterial.Frictionless));
        scene.InitializeStaticCollider(compound, Vector3d.Zero);
        SolidBody mover = scene.CreateSphere(Vector3d.Right * Fixed64.FromFraction(3, 4),
            preventAngularForces: true).Body;
        mover.Collider.Material = PhysicsMaterial.Frictionless;
        mover.UseManualGrounding();
        mover.SleepEnabled = false;
        for (int frame = 0; frame < 16; frame++) Step(scene.Context);
        mover.SleepEnabled = true;
        mover.Sleep();
        // Publish any accepted solver pose before isolating a material-only change.
        Step(scene.Context);
        mover.IsSleeping.Should().BeTrue();
        CollisionPair pair = scene.Context.Physics.GetCollisionPair(compound.Id, mover.Collider.Id)!;
        pair.Manifold.HasContact.Should().BeTrue();
        pair.Manifold.PrimaryContact.HasMaterialOverride.Should().BeTrue();
        GetCompoundMaterial(pair, compound).Should().Be(PhysicsMaterial.Frictionless);
        uint broadPhaseVersion = compound.BroadPhaseVersion;
        PhysicsMaterial requested = changed ? PhysicsMaterial.Default : PhysicsMaterial.Frictionless;

        compound.GetPartCollider(0).Material = requested;
        Step(scene.Context);

        compound.BroadPhaseVersion.Should().Be(broadPhaseVersion);
        pair.Active.Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        GetCompoundMaterial(pair, compound).Should().Be(requested);
        mover.IsSleeping.Should().Be(!changed);
    }

    private static PhysicsMaterial GetCompoundMaterial(CollisionPair pair, LSCollider compound) =>
        ReferenceEquals(pair.ColliderA, compound)
            ? pair.Manifold.PrimaryContact.MaterialA : pair.Manifold.PrimaryContact.MaterialB;

    private static void Step(GravitasWorldContext context) { context.Simulate(); context.LateSimulate(); }
}
