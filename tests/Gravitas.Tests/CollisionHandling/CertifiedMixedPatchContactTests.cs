using System.Linq;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CertifiedMixedPatchContactTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CertifiedPatch_ShouldRetainOneGlobalFiniteExit(bool affine, bool reverse)
    {
        foreach (int subdivisions in new[] { 1, 4 })
            foreach (Fixed64 x in new[] { Fixed64.Zero, Fixed64.MinValue + (Fixed64)32 })
            {
                using var context = GravitasWorldContext.CreateOwned();
                context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
                var mesh = Quad(subdivisions, affine, reverse);
                Fixed64 scale = affine ? Fixed64.Half : Fixed64.One;
                Fixed64 depth = affine ? Fixed64.FromFraction(1, 8) : Fixed64.Quarter;
                var transform = new FixedTransform(new(x, Fixed64.Zero, depth), FixedQuaternion.Identity, Vector3d.One * scale);
                mesh.InitializeWithNoBody(new TestMatterAgent(context, transform));
                var slab = new LSAABBoxCollider2D(new Vector2d(16, 2))
                { MixedHalfThicknessOverride = Fixed64.Two, Material = PhysicsMaterial.Bouncy };
                slab.InitializeWithNoBody(new TestMatterAgent(context,
                    new FixedTransform(new(x, Fixed64.Zero, -Fixed64.One), FixedQuaternion.Identity, Vector3d.One)));
                var contacts = new ContactManifold();
                Assert.True(CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, slab));
                Assert.True(CollisionDetectionMixed.TryBuildManifold(mesh, slab, contacts));
                Assert.Equal(1, contacts.GroupCount);
                Assert.Equal(2, contacts.Count);
                foreach (ManifoldContact row in contacts)
                {
                    Assert.Equal(Vector3d.Backward, row.Normal);
                    Assert.Equal(depth, row.Depth);
                    Assert.False(row.DepthIsClamped);
                    Assert.Equal(mesh.Material, row.MaterialA);
                    Assert.Equal(slab.Material, row.MaterialB);
                    Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d a));
                    Assert.True(row.AnchorB.TryGetWorldPoint(out Vector3d b));
                    Assert.Equal(Fixed64.Zero, a.Y);
                    Assert.Equal(-depth, a.Z);
                    Assert.Equal(Fixed64.Zero, b.Z);
                    Assert.Equal(a.X, b.X);
                    Assert.Equal(Fixed64.Zero, b.Y);
                    Assert.InRange((a.X - x).m_rawValue, (-Fixed64.Half * scale).m_rawValue,
                        (Fixed64.Half * scale + Fixed64.MinIncrement).m_rawValue);
                }
                var expected = contacts.Select(row => (row.ContactId, row.AnchorA, row.AnchorB, row.Depth, row.Normal)).ToArray();
                Assert.True(CollisionDetectionMixed.TryBuildManifold(mesh, slab, contacts));
                Assert.Equal(expected, contacts.Select(row => (row.ContactId, row.AnchorA, row.AnchorB, row.Depth, row.Normal)).ToArray());
                transform.LocalPosition += Vector3d.Forward * (depth + Fixed64.MinIncrement * 64);
                mesh.Simulate();
                // Independent authored-triangle admission checks whole-shape clearance.
                Assert.False(CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, slab));
                Assert.False(CollisionDetectionMixed.TryBuildManifold(mesh, slab, contacts));
            }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ObliqueRim_ShouldKeepFiniteWitnessesAndClearTheWholePatch(bool affine, bool miss)
    {
        using var context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var mesh = Quad(2, affine, reverse: false);
        Fixed64 scale = affine ? Fixed64.Half : Fixed64.One;
        var transform = new FixedTransform(Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4), Vector3d.One * scale);
        mesh.InitializeWithNoBody(new TestMatterAgent(context, transform));
        Fixed64 center = (miss ? Fixed64.FromFraction(3, 5) : Fixed64.FromFraction(2, 5)) * scale;
        Fixed64 size = (miss ? Fixed64.FromFraction(1, 10) : Fixed64.Half) * scale;
        var slab = new LSAABBoxCollider2D(Vector2d.One * size) { MixedHalfThicknessOverride = Fixed64.One };
        slab.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new(center, Fixed64.Zero, center), FixedQuaternion.Identity, Vector3d.One)));
        Assert.True(mesh.Bounds.Intersects(slab.MixedBounds3D));
        Assert.Equal(!miss, CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, slab));
        var contacts = new ContactManifold();
        Assert.Equal(!miss, CollisionDetectionMixed.TryBuildManifold(mesh, slab, contacts));
        if (miss) return;
        Assert.Equal(1, contacts.GroupCount);
        foreach (ManifoldContact row in contacts)
        {
            Assert.True(row.AnchorA.TryGetWorldPoint(out Vector3d a));
            Assert.True(row.AnchorB.TryGetWorldPoint(out Vector3d b));
            Vector3d closest = mesh.ClosestPointOnSurface(a);
            for (int axis = 0; axis < 3; axis++)
                Assert.InRange((a[axis] - closest[axis]).Abs().m_rawValue, 0, 4);
            Fixed64 min = center - size * Fixed64.Half, max = center + size * Fixed64.Half;
            Assert.InRange(b.X.m_rawValue, (min - Fixed64.MinIncrement).m_rawValue, (max + Fixed64.MinIncrement).m_rawValue);
            Assert.InRange(b.Z.m_rawValue, (min - Fixed64.MinIncrement).m_rawValue, (max + Fixed64.MinIncrement).m_rawValue);
            Assert.InRange(b.Y.m_rawValue, -Fixed64.One.m_rawValue, Fixed64.One.m_rawValue);
            Fixed64 boundaryGap = new[] { (b.X - min).Abs(), (b.X - max).Abs(), (b.Z - min).Abs(),
                (b.Z - max).Abs(), (b.Y - Fixed64.One).Abs(), (b.Y + Fixed64.One).Abs() }.Min();
            Assert.InRange(boundaryGap.m_rawValue, 0, 4);
        }
        ManifoldContact selected = contacts.PrimaryContact;
        // 64 raw units cover normal/depth/translation rounding for these unit-sized shapes.
        transform.LocalPosition -= selected.Normal * (selected.Depth + Fixed64.MinIncrement * 64);
        mesh.Simulate();
        Assert.False(CollisionDetectionMixed.IntersectsForContinuousCollision(mesh, slab));
    }

    [Fact]
    public void CompoundPatchAndPrismLeaves_ShouldRetainNamespacesAndMaterials()
    {
        using var context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        Fixed64 h = Fixed64.Half;
        var shape = ColliderShapeDefinition.ConvexMesh(new[] { new Vector3d(-h, Fixed64.Zero, -h), new(h, Fixed64.Zero, -h),
            new(-h, Fixed64.Zero, h), new(h, Fixed64.Zero, h) }, new[] { 0, 2, 1, 1, 2, 3 },
            MeshInertiaPolicy.SurfaceApproximation, PhysicsMaterial.Bouncy);
        var mesh = new LSCompoundCollider(new CompoundColliderPart(shape), CompoundColliderPart.Sphere(h, new(4, 4, 4)));
        mesh.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity, Vector3d.One)));
        var slab = new LSCompoundCollider2D(
            new CompoundColliderPart2D(ColliderShapeDefinition2D.AABBox(new Vector2d(2, 2), PhysicsMaterial.Default), new(-1, 0)),
            new CompoundColliderPart2D(ColliderShapeDefinition2D.AABBox(new Vector2d(2, 2), PhysicsMaterial.Frictionless), new(1, 0)));
        slab.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new(0, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
        var contacts = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mesh, slab, contacts));
        Assert.Equal(2, contacts.GroupCount);
        foreach (ManifoldContact row in contacts)
        {
            Assert.Equal(1, row.FeatureNamespaceA);
            Assert.Contains(row.FeatureNamespaceB, new[] { -1, -2 });
            Assert.Equal(PhysicsMaterial.Bouncy, row.MaterialA);
            Assert.Equal(row.FeatureNamespaceB == -1 ? PhysicsMaterial.Default : PhysicsMaterial.Frictionless, row.MaterialB);
        }
    }

    [Fact]
    public void FreelyRotatingPatchFullSteps_ShouldPreserveDriveAndRepeatReplay()
    {
        Assert.Equal(Run(), Run());
        static string Run()
        {
            using var scene = PhysicsScenarioBuilder.Create();
            var context = scene.Context;
            context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
            context.SetFrameRate(16);
            context.Environment.Gravity = context.Environment.AirDensity = context.Environment.DampingFactor = context.Environment.MinSpeed = Fixed64.Zero;
            var slab = new LSAABBoxCollider2D(new Vector2d(16, 2))
            { Material = PhysicsMaterial.Frictionless, MixedHalfThicknessOverride = Fixed64.Two };
            slab.InitializeWithNoBody(new TestMatterAgent(context,
                new FixedTransform(new(0, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
            var mesh = Quad(1, affine: false, reverse: false);
            var body = scene.CreateBody(mesh, new(-Fixed64.Two, Fixed64.Zero, Fixed64.Half), FixedQuaternion.Identity).Body;
            body.FreezeAxes = BodyFreezeAxes3D.None;
            body.SleepEnabled = false;
            body.UseManualGrounding();
            Fixed64 direction = Fixed64.One;
            for (int frame = 0; frame < 128; frame++)
            {
                if (body.Position3d.X >= Fixed64.Two) direction = -Fixed64.One;
                else if (body.Position3d.X <= -Fixed64.Two) direction = Fixed64.One;
                body.AddLinearImpulse(Vector3d.Right * (direction - body.LinearVelocity.X));
                body.AddLinearImpulse(Vector3d.Backward * Fixed64.FromFraction(1, 16));
                context.Simulate(); context.LateSimulate();
                Assert.Equal(direction, body.LinearVelocity.X);
                Assert.Equal(Fixed64.Zero, body.Position3d.Y);
                Assert.True(body.Position3d.Z > Fixed64.Zero);
            }
            return context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches).ToString();
        }
    }

    private static LSMeshCollider Quad(int divisions, bool affine, bool reverse)
    {
        var vertices = new Vector3d[(divisions + 1) * (divisions + 1)];
        var indices = new int[6 * divisions * divisions];
        Fixed64 size = Fixed64.One + (affine ? Fixed64.MinIncrement : Fixed64.Zero);
        for (int x = 0; x <= divisions; x++)
            for (int z = 0; z <= divisions; z++)
                vertices[x * (divisions + 1) + z] = new(-Fixed64.Half + size * x / divisions, Fixed64.Zero, -Fixed64.Half + size * z / divisions);
        for (int x = 0, offset = 0; x < divisions; x++)
            for (int z = 0; z < divisions; z++)
            {
                int a = x * (divisions + 1) + z, b = a + divisions + 1;
                indices[offset++] = a; indices[offset++] = a + 1; indices[offset++] = b;
                indices[offset++] = b; indices[offset++] = a + 1; indices[offset++] = b + 1;
            }
        if (reverse) for (int i = 0; i < indices.Length; i += 3) (indices[i + 1], indices[i + 2]) = (indices[i + 2], indices[i + 1]);
        return new LSMeshCollider(vertices, indices, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
    }
}
