using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

/// <content>Verifies that a separating initial contact cannot hide a later closing contact in one target.</content>
public sealed partial class ContinuousCollisionDetectionTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ContinuousCuboid_BetweenMeshSurfaces_ShouldReachClosingContactAfterSeparatingFirstContact(
        bool compoundTarget, bool kinematic)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSMeshCollider quad = MeshTestFixtures.CreateConvexQuadFloor();
        LSCollider target;
        if (compoundTarget)
        {
            ColliderShapeDefinition definition = MeshTestFixtures.CreateDefinition(quad);
            target = new LSCompoundCollider(
                new CompoundColliderPart(definition, Vector3d.Down * Fixed64.Half),
                new CompoundColliderPart(definition, Vector3d.Up * Fixed64.Half));
        }
        else
        {
            Vector3d[] quadVertices = quad.Mesh.LocalVertices.ToArray();
            int[] quadTriangles = quad.Mesh.Triangles.ToArray();
            var vertices = new Vector3d[quadVertices.Length * 2];
            var triangles = new int[quadTriangles.Length * 2];
            for (int i = 0; i < quadVertices.Length; i++)
            {
                vertices[i] = quadVertices[i] + Vector3d.Down * Fixed64.Half;
                vertices[i + quadVertices.Length] = quadVertices[i] + Vector3d.Up * Fixed64.Half;
            }
            for (int i = 0; i < quadTriangles.Length; i++)
            {
                triangles[i] = quadTriangles[i];
                triangles[i + quadTriangles.Length] = quadTriangles[i] + quadVertices.Length;
            }
            target = new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation);
        }
        target.Material = PhysicsMaterial.Frictionless;
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        SolidBody source = scenario.CreateBody(new LSCuboidCollider(), Vector3d.Zero,
            FixedQuaternion.Identity, preventAngularForces: true, isKinematic: kinematic).Body;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        // Both surfaces initially touch the unit cube. Floor parts/triangles
        // are authored first and separate during upward travel; the ceiling
        // must still be considered before reducing to one hit for this target.
        if (kinematic)
            source.Agent.Transform.LocalPosition = Vector3d.Up;
        else
            source.AddLinearImpulse(Vector3d.Up * source.Mass);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        source.Position3d.Y.Should().BeInRange(-Fixed64.Epsilon, Fixed64.Epsilon);
        source.Position3d.X.Should().Be(Fixed64.Zero);
        source.Position3d.Z.Should().Be(Fixed64.Zero);
        source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        if (!kinematic)
            source.LinearVelocity.Should().Be(Vector3d.Zero);
    }
}
