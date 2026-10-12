using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

/// <content>Requires warmed finite mixed surface detection and full-host reuse to remain allocation-free.</content>
public sealed partial class MixedNarrowPhaseTests
{
    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.AABox)]
    [InlineData(ColliderType2D.ConvexPolygon)]
    public void MixedSurfaceManifoldAndInitialQuery_ShouldNotAllocateAfterWarmup(ColliderType2D shape)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSMeshCollider mesh = CreateSurfaceReductionFloor(4);
        CreateMesh3D(context, mesh, Vector3d.Zero);
        Vector2d start = new(Fixed64.FromFraction(3, 10), Fixed64.FromFraction(3, 10));
        SolidBody2D slab = CreateSurfaceReductionSlab(context, shape, start);
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold).Should().BeTrue();
        manifold.GroupCount.Should().Be(1);
        manifold.PrimaryContact.Normal.Should().Be(Vector3d.Down);
        context.Simulate();
        bool allContact = true;

        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            allContact &= CollisionDetectionMixed.TryBuildManifold(mesh, slab.Collider, manifold);
            allContact &= CollisionDetectionMixed.TryCollide(mesh, slab.Collider, out _);
        }, warmupIterations: 16, stabilizationIterations: 4, measurementIterations: 16);

        allContact.Should().BeTrue();
        allocated.Should().Be(0);
        if (shape != ColliderType2D.Circle) return;
        bool allHit = true;
        allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            allHit &= context.QueryMixed.SweepCircleAgainst3D(start, start + Vector2d.Left,
                Fixed64.Half, -Fixed64.Quarter, Fixed64.Half, PhysicsLayerMask.FromLayer(0), out _);
        }, warmupIterations: 16, stabilizationIterations: 4, measurementIterations: 16);
        allHit.Should().BeTrue();
        allocated.Should().Be(0);
    }

    [Theory]
    [InlineData(ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule)]
    public void MixedSurfaceFullHost_ShouldReuseRetainedPairsWithoutAllocating(ColliderType2D shape)
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.SetFrameRate(1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Environment.MinSpeed = Fixed64.Zero;
        LSMeshCollider mesh = CreateSurfaceReductionFloor(4);
        mesh.Material = PhysicsMaterial.Frictionless;
        mesh.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        Vector2d start = new(Fixed64.FromFraction(3, 10), Fixed64.FromFraction(3, 10));
        SolidBody2D slab = CreateSurfaceReductionSlab(context, shape, start);
        slab.FreezeAxes = BodyFreezeAxes2D.Rotation;
        slab.Collider.Material = PhysicsMaterial.Frictionless;
        slab.UseManualGrounding();
        Vector2d travel = Vector2d.Left * Fixed64.FromFraction(1, 16);

        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            slab.ResetPosition(start);
            slab.AddLinearImpulse(travel * slab.Mass);
            context.Simulate();
            context.LateSimulate();
        }, warmupIterations: 16, stabilizationIterations: 4, measurementIterations: 16);

        allocated.Should().Be(0);
        slab.Position.Should().Be(start + travel);
        slab.LinearVelocity.Should().Be(travel);
        slab.Agent.Transform.LocalPosition.Y.Should().Be(-Fixed64.Quarter);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
    }
}
