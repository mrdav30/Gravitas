using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CylinderTriangleContactRegressionTests
{
    [Fact]
    public void CylinderContact_ShouldFindTriangleIntrusionAwayFromCenterClosestPoint()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(0, 7, -5), new Vector3d(10, 1, -5), new Vector3d(5, 4, 5)
        }, new[] { 0, 1, 2 }, MeshColliderMode.Concave);
        var surface = scenario.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        var cylinder = scenario.CreateBody(
            new LSCylinderCollider(ColliderShapeDefinition.Cylinder((Fixed64)5, (Fixed64)10)),
            Vector3d.Zero, FixedQuaternion.Identity);
        CollisionPair pair = scenario.CreatePair(surface.Collider, cylinder.Collider);

        // The triangle lies on 3x+5y=35. Its closest point to the cylinder
        // center has y=175/34>5 and is outside the upper cap. Nevertheless,
        // (4,23/5,0) lies in the triangle and strictly inside the cylinder.
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
    }
}
