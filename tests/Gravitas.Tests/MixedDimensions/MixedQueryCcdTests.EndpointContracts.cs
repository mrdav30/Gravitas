using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Fact]
    public void SweepCircleAgainstCuboid_WhenFirstContactIsAtEndpoint_ShouldPreserveEndpointDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBody3D(
            context,
            new LSCuboidCollider(),
            Vector3d.Zero,
            immovable: true);

        context.QueryMixed.SweepCircleAgainst3D(
                new Vector2d((Fixed64)(-2), Fixed64.Zero),
                -Vector2d.Right,
                Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Half,
                IncludeLayerZero,
                out PhysicsMixedHit hit)
            .Should()
            .BeTrue();

        hit.Distance.Should().Be(Fixed64.One);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainstSphere_WhenFirstContactIsAtEndpoint_ShouldPreserveEndpointDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateSphere3D(context, Vector3d.Zero, immovable: true);

        context.QueryMixed.SweepCircleAgainst3D(
                new Vector2d((Fixed64)(-2), Fixed64.Zero),
                -Vector2d.Right,
                Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Half,
                IncludeLayerZero,
                out PhysicsMixedHit hit)
            .Should()
            .BeTrue();

        hit.Distance.Should().Be(Fixed64.One);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainstCapsule_WhenFirstContactIsAtEndpoint_ShouldPreserveEndpointDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBody3D(
            context,
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
            },
            Vector3d.Zero,
            immovable: true);

        context.QueryMixed.SweepCircleAgainst3D(
                new Vector2d((Fixed64)(-2), Fixed64.Zero),
                -Vector2d.Right,
                Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Half,
                IncludeLayerZero,
                out PhysicsMixedHit hit)
            .Should()
            .BeTrue();

        hit.Distance.Should().Be(Fixed64.One);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepCircleAgainstMesh_WhenFirstContactIsAtEndpoint_ShouldPreserveEndpointDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateMesh3D(
            context,
            MeshTestFixtures.CreateVerticalQuad(Fixed64.Zero, -Fixed64.One, Fixed64.One),
            Vector3d.Zero,
            immovable: true);

        context.QueryMixed.SweepCircleAgainst3D(
                new Vector2d(Fixed64.FromFraction(-3, 2), Fixed64.Zero),
                new Vector2d(-Fixed64.Half, Fixed64.Zero),
                Fixed64.Half,
                Fixed64.Zero,
                Fixed64.Half,
                IncludeLayerZero,
                out PhysicsMixedHit hit)
            .Should()
            .BeTrue();

        hit.Distance.Should().Be(Fixed64.One);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void SweepSphereAgainstAabb_WhenFirstContactIsAtEndpoint_ShouldPreserveEndpointDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        _ = CreateBodylessBox2D(context, Vector2d.Zero, Vector2d.One * Fixed64.Two);

        context.QueryMixed.SweepSphereAgainst2D(
                new Vector3d(Fixed64.FromFraction(-5, 2), Fixed64.Zero, Fixed64.Zero),
                new Vector3d(Fixed64.FromFraction(-3, 2), Fixed64.Zero, Fixed64.Zero),
                Fixed64.Half,
                IncludeLayerZero,
                out PhysicsMixedHit hit)
            .Should()
            .BeTrue();

        hit.Distance.Should().Be(Fixed64.One);
        hit.ReducerKind.Should().Be(PhysicsQueryReducerKind.Exact);
    }

    [Fact]
    public void CapsuleBoundaryReducer_WhenFirstContactIsAtEndpoint_ShouldPreserveEndpointDistance()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var capsule = (LSCapsuleCollider2D)CreateBodylessCapsule2D(context, Vector2d.Zero);
        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;

        GravitasQueryMixedService.TryKeepCapsuleSlabBoundaryEdgeSweep(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            -Vector3d.Right,
            Vector3d.Right,
            Fixed64.One,
            capsule,
            new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.One),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
            Fixed64.One,
            ref found,
            ref bestDistance);

        found.Should().BeTrue();
        bestDistance.Should().Be(Fixed64.One);
    }
}
