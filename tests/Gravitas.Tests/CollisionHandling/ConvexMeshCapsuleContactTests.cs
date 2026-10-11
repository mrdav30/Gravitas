using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ConvexMeshCapsuleContactTests
{
    [Theory]
    [InlineData(-1, false)]
    [InlineData(1, false)]
    [InlineData(-1, true)]
    [InlineData(1, true)]
    public void DetectedContact_ShouldMatchBothAnchorsAndClearTheWholeCapsule(int side, bool box)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCollider wall = box ? new LSCuboidCollider() : MeshTestFixtures.CreateConvexCube();
        scenario.InitializeStaticCollider(wall, Vector3d.Zero);
        Vector3d center = new(Fixed64.Zero, (Fixed64)side * Fixed64.FromFraction(3, 2), -Fixed64.FromFraction(1, 8));
        var capsule = new LSCapsuleCollider { Radius = Fixed64.FromFraction(1, 8),
            Size = new Vector3d(Fixed64.Quarter, Fixed64.FromFraction(17, 4), Fixed64.Quarter) };
        scenario.InitializeStaticCollider(capsule, center);
        foreach (bool reverse in new[] { false, true })
        {
            CollisionPair pair = reverse ? scenario.CreatePair(capsule, wall) : scenario.CreatePair(wall, capsule);
            CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
            pair.Manifold.Count.Should().Be(1);
            ManifoldContact contact = pair.Manifold.PrimaryContact;
            contact.AnchorA.TryGetOffsetFrom(contact.AnchorB, out Vector3d offset).Should().BeTrue();
            Vector3d.Cross(offset, contact.Normal).Should().Be(Vector3d.Zero);
            contact.Depth.Should().Be(Fixed64.Half);
            Vector3d direction = ReferenceEquals(pair.ColliderB, capsule) ? contact.Normal : -contact.Normal;
            foreach (int raw in new[] { -1, 0, 1 })
            {
                var moved = new LSCapsuleCollider { Radius = capsule.Radius, Size = capsule.Size };
                scenario.InitializeStaticCollider(moved, center + direction * (contact.Depth + Fixed64.FromRaw(raw)));
                CollisionPair movedPair = scenario.CreatePair(wall, moved);
                CollisionDetection.DoCollisionCheck(movedPair).Should().Be(raw <= 0);
                if (raw <= 0) movedPair.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.FromRaw(-raw));
            }
        }
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(1, false)]
    [InlineData(-1, true)]
    [InlineData(1, true)]
    public void FullHostLoop_ShouldConserveTotalAngularMomentum(int side, bool box)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        LSCollider wall = box ? new LSCuboidCollider() : MeshTestFixtures.CreateConvexCube();
        SolidBody hull = scenario.CreateBody(wall, Vector3d.Zero, FixedQuaternion.Identity).Body;
        var capsule = new LSCapsuleCollider { Radius = Fixed64.FromFraction(1, 8),
            Size = new Vector3d(Fixed64.Quarter, Fixed64.FromFraction(17, 4), Fixed64.Quarter) };
        SolidBody curved = scenario.CreateBody(capsule,
            new Vector3d(Fixed64.Zero, (Fixed64)side * Fixed64.FromFraction(3, 2), -Fixed64.FromFraction(1, 8)),
            FixedQuaternion.Identity).Body;
        wall.Material = capsule.Material = PhysicsMaterial.Frictionless;
        hull.UseManualGrounding();
        curved.UseManualGrounding();
        curved.AddLinearImpulse(Vector3d.Forward * Fixed64.Two);
        Fixed64 inertiaA = wall.CalculateInertiaTensor(hull.Mass, hull.LocalCenterOfMassOffset).M11;
        Fixed64 inertiaB = capsule.CalculateInertiaTensor(curved.Mass, curved.LocalCenterOfMassOffset).M11;
        Fixed64 before = AngularMomentumX(hull, inertiaA) + AngularMomentumX(curved, inertiaB);
        Vector3d linearBefore = hull.LinearVelocity * hull.Mass + curved.LinearVelocity * curved.Mass;

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        // Z-only translation/correction and X spin preserve this inertia component.
        // Unit-scale tensors and velocities leave only final fixed-point products;
        // 256 raw units are a conservative rounding allowance, not physical drift.
        Fixed64 difference = AngularMomentumX(hull, inertiaA) + AngularMomentumX(curved, inertiaB) - before;
        difference.Abs().Should().BeLessThanOrEqualTo(Fixed64.FromRaw(256));
        (hull.LinearVelocity * hull.Mass + curved.LinearVelocity * curved.Mass).Should().Be(linearBefore);
        curved.LinearVelocity.Z.Should().BeLessThan(Fixed64.Two);
        hull.LinearVelocity.X.Should().Be(Fixed64.Zero);
        hull.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        curved.LinearVelocity.X.Should().Be(Fixed64.Zero);
        curved.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        hull.AngularVelocity.Y.Should().Be(Fixed64.Zero);
        hull.AngularVelocity.Z.Should().Be(Fixed64.Zero);
        curved.AngularVelocity.Y.Should().Be(Fixed64.Zero);
        curved.AngularVelocity.Z.Should().Be(Fixed64.Zero);
        hull.WorldCenterOfMass.Y.Should().Be(Fixed64.Zero);
        curved.WorldCenterOfMass.Y.Should().Be((Fixed64)side * Fixed64.FromFraction(3, 2));
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(1, false)]
    [InlineData(-1, true)]
    [InlineData(1, true)]
    public void FullHostLoop_ShouldApplyOffCenterAngularResponse(int side, bool box)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.MinSpeed = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        LSCollider wall = box ? new LSCuboidCollider() : MeshTestFixtures.CreateConvexCube();
        SolidBody solid = scenario.CreateBody(wall, Vector3d.Zero, FixedQuaternion.Identity, immovable: true).Body;
        var capsule = new LSCapsuleCollider { Radius = Fixed64.FromFraction(1, 8),
            Size = new Vector3d(Fixed64.Quarter, Fixed64.FromFraction(17, 4), Fixed64.Quarter) };
        SolidBody body = scenario.CreateBody(capsule,
            new Vector3d(Fixed64.Zero, (Fixed64)side * Fixed64.FromFraction(3, 2), -Fixed64.FromFraction(1, 8)),
            FixedQuaternion.Identity).Body;
        wall.Material = capsule.Material = PhysicsMaterial.Frictionless;
        body.UseManualGrounding();
        body.AddLinearImpulse(Vector3d.Forward);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        (body.AngularVelocity.X * (Fixed64)side).Should().BeGreaterThan(Fixed64.Zero);
        body.AngularVelocity.Y.Should().Be(Fixed64.Zero);
        body.AngularVelocity.Z.Should().Be(Fixed64.Zero);
        body.LinearVelocity.Z.Should().BeLessThan(Fixed64.One);
        solid.Position3d.Should().Be(Vector3d.Zero);
        solid.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    private static Fixed64 AngularMomentumX(SolidBody body, Fixed64 inertia) =>
        body.WorldCenterOfMass.Y * body.LinearVelocity.Z * body.Mass + inertia * body.AngularVelocity.X;
}
