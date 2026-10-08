using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContainedCurvedNormalTests
{
    private static readonly Fixed64 Tolerance = Fixed64.FromFraction(1, 1_000_000);

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void DetectContainedSphere_ShouldRetainOutwardEscapeNormal(bool cylinder, bool reverse)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateContainedPair(scenario, cylinder, reverse,
            out CollisionPair pair, out _, out _, out Vector3d outward);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();

        ManifoldContact contact = pair.Manifold.PrimaryContact;
        Vector3d resolved = pair.ColliderA is LSSphereCollider ? -contact.Normal : contact.Normal;
        Vector3d.Dot(resolved, outward).Should().BeGreaterThan(Fixed64.One - Tolerance);
        contact.Depth.Should().BeGreaterThan(CollisionResponse.PenetrationSlop);
        // The sphere's inward support and solid's outward surface must remain
        // paired with the escape direction even while the sphere is contained.
        (Vector3d.Dot(contact.PointA - contact.PointB, contact.Normal) - contact.Depth)
            .Abs().Should().BeLessThan(Tolerance);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ResolveContainedSphere_ShouldCorrectOutwardWithoutBrakingEscape(bool cylinder, bool reverse)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        CreateContainedPair(scenario, cylinder, reverse,
            out CollisionPair pair, out SolidBody sphere, out SolidBody solid, out Vector3d outward);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        sphere.AddLinearImpulse(outward * (Fixed64)2);
        Vector3d initialVelocity = sphere.LinearVelocity;
        Vector3d initialPosition = sphere.Position3d;
        Vector3d solidPosition = solid.Position3d;

        CollisionResponse.CalculateImpulse(pair);

        Vector3d correction = sphere.Position3d - initialPosition;
        Vector3d.Dot(correction, outward).Should().BeGreaterThan(Fixed64.Zero);
        (sphere.LinearVelocity - initialVelocity).Magnitude.Should().BeLessThan(Tolerance);
        solid.Position3d.Should().Be(solidPosition);
    }

    private static void CreateContainedPair(
        PhysicsScenarioBuilder scenario,
        bool cylinder,
        bool reverse,
        out CollisionPair pair,
        out SolidBody sphereBody,
        out SolidBody solidBody,
        out Vector3d outward)
    {
        PhysicsMaterial material = PhysicsMaterial.Frictionless;
        LSCollider solidCollider = cylinder
            ? new LSCylinderCollider { Radius = Fixed64.Half, Size = Vector3d.One, Material = material }
            : new LSConeCollider { Radius = Fixed64.Half, Size = Vector3d.One, Material = material };
        ScenarioBody<LSCollider> solid = scenario.CreateBody(
            solidCollider, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        // The cone's nearest lateral surface points upward while its center
        // delta points downward. Collider-center orientation is therefore not
        // a valid substitute for the admitted surface's escape direction.
        Vector3d position = cylinder
            ? Vector3d.Right * Fixed64.FromFraction(1, 10)
            : new Vector3d(Fixed64.FromFraction(1, 100), -Fixed64.FromFraction(1, 10), Fixed64.Zero);
        outward = cylinder ? Vector3d.Right : new Vector3d((Fixed64)2, Fixed64.One, Fixed64.Zero).Normalized;
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.FromFraction(1, 20), Material = material },
            position, FixedQuaternion.Identity, preventAngularForces: true);
        sphereBody = sphere.Body;
        solidBody = solid.Body;
        pair = reverse
            ? scenario.CreatePair(sphere.Collider, solid.Collider)
            : scenario.CreatePair(solid.Collider, sphere.Collider);
    }
}
