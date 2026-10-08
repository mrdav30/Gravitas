using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

/// <content>Verifies CCD admission and response for two-sided mesh contact witnesses.</content>
public sealed partial class ContinuousCollisionDetectionTests
{
    [Theory]
    [InlineData(MeshColliderMode.Convex, false, false)]
    [InlineData(MeshColliderMode.Convex, true, false)]
    [InlineData(MeshColliderMode.Concave, false, false)]
    [InlineData(MeshColliderMode.Concave, true, false)]
    [InlineData(MeshColliderMode.Convex, false, true)]
    [InlineData(MeshColliderMode.Convex, true, true)]
    public void ContinuousMode_OnMeshSurface_ShouldAllowSeparationAndRejectClosingRegardlessOfWinding(
        MeshColliderMode mode, bool reverseWinding, bool compoundTarget)
    {
        foreach (int side in new[] { -1, 1 })
        foreach (Fixed64 gap in new[] { Fixed64.Quarter, Fixed64.Half })
        foreach (int motion in new[] { -1, 0, 1 })
        {
            using PhysicsScenarioBuilder scenario = CreateCcdScenario();
            LSMeshCollider mesh = MeshTestFixtures.CreateConvexQuadFloor(mode);
            FixedQuaternion rotation = reverseWinding
                ? new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero)
                : FixedQuaternion.Identity;
            LSCollider target = compoundTarget
                ? new LSCompoundCollider(new CompoundColliderPart(
                    MeshTestFixtures.CreateDefinition(mesh), Vector3d.Zero, rotation))
                : mesh;
            target.Material = PhysicsMaterial.Frictionless;
            target.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
                new FixedTransform(Vector3d.Zero, compoundTarget ? FixedQuaternion.Identity : rotation, Vector3d.One)));
            Vector3d normal = Vector3d.Up * (Fixed64)side;
            Vector3d start = normal * gap;
            SolidBody source = scenario.CreateSphere(start).Body;
            source.Collider.Material = PhysicsMaterial.Frictionless;
            source.UseManualGrounding();
            source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
            Vector3d displacement = motion == 0 ? Vector3d.Right : normal * (Fixed64)motion;
            source.AddLinearImpulse(displacement * source.Mass);

            scenario.Context.Simulate();
            scenario.Context.LateSimulate();

            if (motion < 0)
            {
                (source.Position3d.Y * (Fixed64)side).Should().BeGreaterThanOrEqualTo(gap - Fixed64.Epsilon);
                source.LinearVelocity.Y.Should().Be(Fixed64.Zero);
                source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
            }
            else
            {
                if (motion == 0 && gap < Fixed64.Half)
                {
                    // CCD must admit tangent travel; the discrete solver may
                    // still correct the existing penetration along the normal.
                    source.Position3d.X.Should().Be(displacement.X);
                    source.Position3d.Z.Should().Be(Fixed64.Zero);
                    (source.Position3d.Y * (Fixed64)side).Should().BeGreaterThanOrEqualTo(gap - Fixed64.Epsilon);
                }
                else
                    source.Position3d.Should().Be(start + displacement);
                source.LinearVelocity.Should().Be(displacement);
                source.LastContinuousCollisionToiIterationCount.Should().Be(0);
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousMode_SphereApproachingMeshEdgeInItsPlane_ShouldStopAtRadialContact(bool reverseWinding)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        mesh.Material = PhysicsMaterial.Frictionless;
        mesh.InitializeWithNoBody(new TestMatterAgent(scenario.Context, new FixedTransform(
            Vector3d.Zero,
            reverseWinding ? new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero)
                : FixedQuaternion.Identity,
            Vector3d.One)));
        SolidBody source = scenario.CreateSphere(Vector3d.Right * (Fixed64)3).Body;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.AddLinearImpulse(Vector3d.Left * (Fixed64)2 * source.Mass);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        source.Position3d.X.Should().BeInRange(Fixed64.FromFraction(5, 2) - Fixed64.Epsilon,
            Fixed64.FromFraction(5, 2) + Fixed64.Epsilon);
        source.LinearVelocity.X.Should().Be(Fixed64.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }
}
