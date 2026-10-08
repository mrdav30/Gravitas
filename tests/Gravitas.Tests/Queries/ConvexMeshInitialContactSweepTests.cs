using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class ConvexMeshInitialContactSweepTests
{
    private static readonly PhysicsLayerMask LayerZero = PhysicsLayerMask.FromLayer(0);

    [Theory]
    [InlineData(ColliderType.Capsule, false)]
    [InlineData(ColliderType.Capsule, true)]
    [InlineData(ColliderType.Cylinder, false)]
    [InlineData(ColliderType.Cylinder, true)]
    [InlineData(ColliderType.Cone, false)]
    [InlineData(ColliderType.Cone, true)]
    public void AnalyticSourceInitialMeshContact_ShouldPreserveSourceSide(
        ColliderType sourceType, bool reverseWinding)
    {
        foreach (int side in new[] { -1, 1 })
        foreach (bool overlapping in new[] { false, true })
        {
            using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
            LSCollider target = AddTarget(scenario, MeshColliderMode.Concave, reverseWinding, false);
            LSCollider source = sourceType switch
            {
                ColliderType.Capsule => new LSCapsuleCollider { Size = new Vector3d(1, 2, 1) },
                ColliderType.Cylinder => new LSCylinderCollider(),
                _ => new LSConeCollider()
            };
            Fixed64 halfHeight = sourceType == ColliderType.Capsule ? Fixed64.One : Fixed64.Half;
            Vector3d expected = Vector3d.Up * (Fixed64)side;
            scenario.CreateBody(source, expected * (overlapping ? halfHeight - Fixed64.Quarter : halfHeight),
                FixedQuaternion.Identity);

            foreach (Vector3d travel in new[] { expected, -expected, Vector3d.Right })
            {
                Physics3DHit hit;
                bool found = source switch
                {
                    LSCapsuleCollider capsule => scenario.Context.Query3D.SweepCapsule(capsule, travel, LayerZero, out hit),
                    LSCylinderCollider cylinder => scenario.Context.Query3D.SweepCylinder(cylinder, travel, LayerZero, out hit),
                    _ => scenario.Context.Query3D.SweepCone((LSConeCollider)source, travel, LayerZero, out hit)
                };
                found.Should().BeTrue();
                AssertInitialNormal(hit, target, expected);
            }
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void InitialEdgeOrVertexContact_ShouldRetainSeparatingWitnessNormal(bool vertex, bool toleranceGap)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCollider target = AddTarget(scenario, MeshColliderMode.Concave, false, false);
        // A positive gap half the worker's contact tolerance admits a separated
        // GJK witness at time zero; it must keep its radial exit direction.
        Fixed64 gap = toleranceGap ? ConvexSweepQueryWorker.ContactTolerance / (Fixed64)2 : Fixed64.Zero;
        Fixed64 edgeCenter = Fixed64.FromFraction(5, 2) + gap;
        var source = new LSCuboidCollider();
        scenario.CreateBody(source, new Vector3d(edgeCenter, Fixed64.Zero, vertex ? edgeCenter : Fixed64.Zero),
            FixedQuaternion.Identity);
        Vector3d outward = vertex ? (Vector3d.Right + Vector3d.Forward).Normalized : Vector3d.Right;
        Physics3DHit? previous = null;
        foreach (Vector3d travel in new[] { outward, -outward, Vector3d.Up })
        {
            scenario.Context.Query3D.SweepCuboid(source, travel, LayerZero, out Physics3DHit hit).Should().BeTrue();
            hit.Collider.Should().BeSameAs(target);
            hit.Distance.Should().Be(Fixed64.Zero);
            Vector3d.Dot(hit.Normal, outward).Should().BeGreaterThan(Fixed64.Half);
            if (previous.HasValue)
                hit.Normal.Should().Be(previous.Value.Normal);
            previous = hit;
        }
    }

    [Theory]
    [InlineData(MeshColliderMode.Convex, false, false)]
    [InlineData(MeshColliderMode.Convex, true, false)]
    [InlineData(MeshColliderMode.Concave, false, false)]
    [InlineData(MeshColliderMode.Concave, true, false)]
    [InlineData(MeshColliderMode.Convex, false, true)]
    [InlineData(MeshColliderMode.Convex, true, true)]
    public void CuboidInitialMeshContact_ShouldPreserveGeometricSourceSideIndependentlyOfTravel(
        MeshColliderMode mode, bool reverseWinding, bool compoundTarget)
    {
        foreach (int side in new[] { -1, 1 })
        foreach (Fixed64 gap in new[] { Fixed64.Quarter, Fixed64.Half })
        {
            using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
            LSCollider target = AddTarget(scenario, mode, reverseWinding, compoundTarget);
            var source = new LSCuboidCollider();
            scenario.CreateBody(source, Vector3d.Up * (Fixed64)side * gap, FixedQuaternion.Identity);
            Vector3d expected = Vector3d.Up * (Fixed64)side;
            var hits = new SwiftList<Physics3DHit>();

            foreach (Vector3d travel in new[] { expected, -expected, Vector3d.Right })
            {
                scenario.Context.Query3D.SweepCuboid(source, travel, LayerZero, out Physics3DHit hit)
                    .Should().BeTrue();
                scenario.Context.Query3D.SweepCuboidAll(source, travel, LayerZero, hits).Should().Be(1);

                AssertInitialNormal(hit, target, expected);
                hits[0].Should().Be(hit);
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OffsetSourceGeometryInitialContact_ShouldUseActualLeafSideRatherThanAuthoredOrigin(bool compoundSource)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCollider target = AddTarget(scenario, MeshColliderMode.Concave, false, false);
        LSCollider source = CreateOffsetSource(compoundSource);
        // The authored origin is above the plane; the actual cube is centered
        // below it. A center-only orientation shortcut therefore picks the wrong side.
        scenario.CreateBody(source, Vector3d.Up * Fixed64.FromFraction(3, 2), FixedQuaternion.Identity);
        source.BoundsMax.Y.Should().Be(Fixed64.Zero);

        foreach (Vector3d travel in new[] { Vector3d.Down, Vector3d.Up, Vector3d.Right })
        {
            Physics3DHit hit;
            bool found = source is LSCompoundCollider compound
                ? scenario.Context.Query3D.SweepCompound(compound, travel, LayerZero, out hit)
                : scenario.Context.Query3D.SweepConvexMesh((LSMeshCollider)source, travel, LayerZero, out hit);

            found.Should().BeTrue();
            AssertInitialNormal(hit, target, Vector3d.Down);
        }
    }

    private static LSCollider AddTarget(PhysicsScenarioBuilder scenario, MeshColliderMode mode,
        bool reverseWinding, bool compoundTarget)
    {
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexQuadFloor(mode);
        FixedQuaternion rotation = reverseWinding
            ? new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero)
            : FixedQuaternion.Identity;
        LSCollider target = compoundTarget
            ? new LSCompoundCollider(new CompoundColliderPart(MeshTestFixtures.CreateDefinition(mesh), Vector3d.Zero, rotation))
            : mesh;
        target.InitializeWithNoBody(new TestMatterAgent(scenario.Context,
            new FixedTransform(Vector3d.Zero, compoundTarget ? FixedQuaternion.Identity : rotation, Vector3d.One)));
        return target;
    }

    [Fact]
    public void OneRawEdgeGap_ShouldRetainSeparatingFeatureRatherThanTriangleWinding()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCollider target = AddTarget(scenario, MeshColliderMode.Concave, false, false);
        var source = new LSCuboidCollider();
        scenario.CreateBody(source,
            new Vector3d(Fixed64.FromFraction(5, 2) + Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity);
        foreach (Vector3d travel in new[] { Vector3d.Right, Vector3d.Left, Vector3d.Up })
        {
            scenario.Context.Query3D.SweepCuboid(source, travel, LayerZero, out Physics3DHit hit)
                .Should().BeTrue();
            hit.Collider.Should().BeSameAs(target);
            hit.Distance.Should().Be(Fixed64.Zero);
            hit.Normal.Should().Be(Vector3d.Right);
        }
    }

    private static LSCollider CreateOffsetSource(bool compoundSource)
    {
        if (compoundSource)
            return new LSCompoundCollider(new CompoundColliderPart(
                MeshTestFixtures.CreateConvexCubeDefinition(), Vector3d.Down * (Fixed64)2));

        LSMeshCollider cube = MeshTestFixtures.CreateConvexCube();
        Vector3d[] vertices = cube.Mesh.LocalVertices.ToArray();
        for (int i = 0; i < vertices.Length; i++)
            vertices[i] += Vector3d.Down * (Fixed64)2;
        return new LSMeshCollider(vertices, cube.Mesh.Triangles.ToArray());
    }

    private static void AssertInitialNormal(Physics3DHit hit, LSCollider target, Vector3d expected)
    {
        hit.Collider.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Point.Y.Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        hit.Normal.Should().Be(expected);
    }
}
