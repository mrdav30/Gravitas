using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContactGroupResponseTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CompoundNamespaces_ShouldRetainNineIndependentGroups(bool reverse)
    {
        var manifold = new ContactManifold();
        for (int step = 0; step < 9; step++)
        {
            int surface = reverse ? 8 - step : step;
            AddContact(manifold, surface, Vector3d.Right);
        }

        manifold.Count.Should().Be(9, "independent compound surfaces must not compete for four pair-wide slots");
        for (int index = 0; index < 9; index++)
            manifold[index].FeatureNamespaceA.Should().Be(index);
    }

    [Fact]
    public void IndependentWallNormals_ShouldBlockBothIncomingComponents()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var wall = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
        var actor = scenario.CreateCuboid(new Vector3d(1, 0, 1), preventAngularForces: true);
        wall.Collider.Material = actor.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair = scenario.CreatePair(wall.Collider, actor.Collider);
        AddContact(pair.Manifold, 0, Vector3d.Right);
        AddContact(pair.Manifold, 1, Vector3d.Forward);
        actor.Body.AddLinearImpulse(new Vector3d(-1, 0, -1) * actor.Body.Mass);

        CollisionResponse.CalculateImpulse(pair);

        actor.Body.LinearVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void SuppliedNormal_ShouldRemainAuthoritativeAgainstColliderCenters()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var wall = scenario.CreateCuboid(new Vector3d(4, 0, 0), immovable: true);
        var actor = scenario.CreateCuboid(Vector3d.Zero, preventAngularForces: true);
        wall.Collider.Material = actor.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair = scenario.CreatePair(wall.Collider, actor.Collider);
        AddContact(pair.Manifold, 0, Vector3d.Right);
        actor.Body.AddLinearImpulse(Vector3d.Left * actor.Body.Mass);

        CollisionResponse.CalculateImpulse(pair);

        actor.Body.LinearVelocity.Should().Be(Vector3d.Zero,
            "a local concave-surface normal cannot be inferred from the whole collider centroid");
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void Restitution_ShouldRetainPreWarmStartTargetAcrossIterations(bool cached, bool exact)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var wall = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
        var actor = scenario.CreateCuboid(new Vector3d(1, 0, 0), preventAngularForces: true);
        wall.Collider.Material = actor.Collider.Material =
            PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.Zero, Fixed64.One);
        CollisionPair pair = scenario.CreatePair(wall.Collider, actor.Collider);
        ContactAnchor first = exact
            ? new ContactAnchor(Vector3d.Right * Fixed64.MaxValue, Vector3d.Right)
            : ContactAnchor.FromWorldPoint(Vector3d.Zero);
        pair.Manifold.SetContact(first, ContactAnchor.FromWorldPoint(Vector3d.Zero), Fixed64.Zero, Vector3d.Right);
        actor.Body.AddLinearImpulse(Vector3d.Left * actor.Body.Mass);
        if (cached)
            pair.StoreWarmStartImpulse(pair.Manifold[0].ContactId, Vector3d.Right, Fixed64.Half, Fixed64.Zero);

        CollisionResponse.PrepareSolve(pair);
        CollisionResponse.CalculateImpulse(pair, applyCachedImpulse: true, applyPositionCorrection: true);
        actor.Body.LinearVelocity.Should().Be(Vector3d.Right);
        CollisionResponse.CalculateImpulse(pair, applyCachedImpulse: false, applyPositionCorrection: false);

        actor.Body.LinearVelocity.Should().Be(Vector3d.Right,
            "an additional solver iteration must not retract the impact's restitution target");
    }

    private static void AddContact(ContactManifold manifold, int surface, Vector3d normal) =>
        manifold.AddContact(ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero), Fixed64.Zero, normal,
            PhysicsMaterial.Frictionless, PhysicsMaterial.Frictionless,
            featureNamespaceA: surface);
}
