using System.Collections.Generic;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests;

public sealed class MeshConeContinuousAdmissionTests
{
    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    [InlineData(3, false)]
    [InlineData(3, true)]
    [InlineData(4, false)]
    [InlineData(4, true)]
    [InlineData(5, false)]
    [InlineData(5, true)]
    [InlineData(6, false)]
    [InlineData(6, true)]
    [InlineData(7, false)]
    [InlineData(7, true)]
    [InlineData(8, false)]
    [InlineData(8, true)]
    public void FiniteAdmission_ShouldMatchCompleteManifoldAtRealBoundaries(int geometry, bool affine)
    {
        foreach (Fixed64 x in new[] { Fixed64.Zero, Fixed64.MaxValue - (Fixed64)32,
            Fixed64.MinValue + (Fixed64)32 })
            foreach (FixedQuaternion rotation in new[] { FixedQuaternion.Identity, PhysicsScenarioBuilder.Yaw(31) })
            {
                using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
                var vertices = new List<Vector3d>();
                var triangles = new List<int>();
                CreateGeometry(geometry, vertices, triangles);
                if (affine)
                    for (int i = 0; i < vertices.Count; i++)
                        // Half-raw physical Z placement requires the exact source chart.
                        vertices[i] = new(vertices[i].X, vertices[i].Y,
                            vertices[i].Z * Fixed64.Two + Fixed64.MinIncrement);
                var mesh = new LSMeshCollider(vertices.ToArray(), triangles.ToArray(),
                    MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
                Vector3d origin = Vector3d.Right * x;
                mesh.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                    new FixedTransform(origin, rotation,
                        new Vector3d(Fixed64.One, Fixed64.One, affine ? Fixed64.Half : Fixed64.One))));
                var cone = new LSConeCollider
                {
                    Radius = geometry == 8 ? Fixed64.MinIncrement : Fixed64.Half,
                    Size = new Vector3d(Fixed64.One, geometry == 8 ? Fixed64.MinIncrement : Fixed64.One, Fixed64.One)
                };
                cone.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                    new FixedTransform(origin, rotation, Vector3d.One)));

                bool expected = geometry != 4 && geometry != 6 && geometry != 7;
                CollisionDetection.IntersectsMeshConeSurface(mesh, cone, scene.Context.CollisionScratch)
                    .Should().Be(expected);
                CollisionDetection.DoCollisionCheck(scene.CreatePair(mesh, cone)).Should().Be(expected);
            }
    }

    private static void CreateGeometry(int kind, List<Vector3d> vertices, List<int> triangles)
    {
        if (kind == 6)
        {
            vertices.AddRange(new[] { Vector3d.Zero, Vector3d.Right * (Fixed64)10,
                Vector3d.Forward * (Fixed64)10 });
            // Move the finite triangle's empty aggregate corner to the cone origin.
            for (int i = 0; i < vertices.Count; i++) vertices[i] -= new Vector3d(9, 0, 9);
            triangles.AddRange(new[] { 0, 1, 2 });
            return;
        }
        if (kind == 4)
        {
            AddQuad(-Fixed64.Two, -Fixed64.Two, -Fixed64.Half, Fixed64.Two, Fixed64.Zero, vertices, triangles);
            AddQuad(Fixed64.Half, -Fixed64.Two, Fixed64.Two, Fixed64.Two, Fixed64.Zero, vertices, triangles);
            AddQuad(-Fixed64.Half, -Fixed64.Two, Fixed64.Half, -Fixed64.Half, Fixed64.Zero, vertices, triangles);
            AddQuad(-Fixed64.Half, Fixed64.Half, Fixed64.Half, Fixed64.Two, Fixed64.Zero, vertices, triangles);
            return;
        }
        if (kind == 5)
        {
            for (int side = -1; side <= 1; side += 2)
            {
                int first = vertices.Count;
                vertices.Add(new(Fixed64.FromFraction(side, 8), Fixed64.Zero, Fixed64.FromFraction(-1, 16)));
                vertices.Add(new(Fixed64.FromFraction(side, 4), Fixed64.Zero, Fixed64.Zero));
                vertices.Add(new(Fixed64.FromFraction(side, 8), Fixed64.Zero, Fixed64.FromFraction(1, 16)));
                triangles.AddRange(new[] { first, first + 1, first + 2 });
            }
            return;
        }
        Fixed64 height = kind == 0 ? -Fixed64.Half : kind == 1 ? Fixed64.Half
            : kind == 7 ? Fixed64.One : Fixed64.Zero;
        AddQuad(-Fixed64.Two, -Fixed64.Two, Fixed64.Two, Fixed64.Two, height, vertices, triangles);
        if (kind == 2 || kind == 3)
            for (int i = 0; i < vertices.Count; i++)
            {
                Vector3d v = vertices[i];
                vertices[i] = new(kind == 2 ? Fixed64.Half : Fixed64.Quarter - Fixed64.Half * v.X,
                    v.X, v.Z);
            }
    }

    private static void AddQuad(Fixed64 x0, Fixed64 z0, Fixed64 x1, Fixed64 z1, Fixed64 y,
        List<Vector3d> vertices, List<int> triangles)
    {
        int first = vertices.Count;
        vertices.AddRange(new[] { new Vector3d(x0, y, z0), new Vector3d(x1, y, z0),
            new Vector3d(x1, y, z1), new Vector3d(x0, y, z1) });
        triangles.AddRange(new[] { first, first + 1, first + 2, first, first + 2, first + 3 });
    }
}
