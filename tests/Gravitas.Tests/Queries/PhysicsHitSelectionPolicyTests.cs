using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class PhysicsHitSelectionPolicyTests
{
    [Fact]
    public void ShouldReplace2D_ShouldAcceptFirstAndEarlierSortedHit()
    {
        LSCircleCollider2D currentCollider = CreateCircle2D();
        LSCircleCollider2D candidateCollider = CreateCircle2D();
        Physics2DHit current = new(currentCollider, Vector2d.Zero, Vector2d.Right, (Fixed64)5);
        Physics2DHit farther = new(candidateCollider, Vector2d.Zero, Vector2d.Right, (Fixed64)6);
        Physics2DHit nearer = new(candidateCollider, Vector2d.Zero, Vector2d.Right, (Fixed64)4);

        PhysicsHitSelectionPolicy.ShouldReplace(farther, found: false, current).Should().BeTrue();
        PhysicsHitSelectionPolicy.ShouldReplace(farther, found: true, current).Should().BeFalse();
        PhysicsHitSelectionPolicy.ShouldReplace(nearer, found: true, current).Should().BeTrue();
    }

    [Fact]
    public void ShouldReplace3D_ShouldAcceptFirstAndEarlierSortedHit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider candidateCollider = scenario.CreateStaticSphere(Vector3d.Left);
        LSSphereCollider currentCollider = scenario.CreateStaticSphere(Vector3d.Right);
        Physics3DHit current = new(currentCollider, Vector3d.Zero, Vector3d.Right, (Fixed64)5, Vector3d.Right);
        Physics3DHit farther = new(candidateCollider, Vector3d.Zero, Vector3d.Right, (Fixed64)6, Vector3d.Right);
        Physics3DHit earlierIdTie = new(candidateCollider, Vector3d.Zero, Vector3d.Right, (Fixed64)5, Vector3d.Right);

        PhysicsHitSelectionPolicy.ShouldReplace(farther, found: false, current).Should().BeTrue();
        PhysicsHitSelectionPolicy.ShouldReplace(farther, found: true, current).Should().BeFalse();
        PhysicsHitSelectionPolicy.ShouldReplace(earlierIdTie, found: true, current).Should().BeTrue();
    }

    [Fact]
    public void ShouldReplaceMixed_ShouldAcceptFirstAndEarlierSortedHit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider candidate3D = scenario.CreateStaticSphere(Vector3d.Left);
        LSSphereCollider current3D = scenario.CreateStaticSphere(Vector3d.Right);
        LSCircleCollider2D current2D = CreateCircle2D();
        LSCircleCollider2D candidate2D = CreateCircle2D();
        PhysicsMixedHit current = new(
            current3D,
            current2D,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            PhysicsQueryReducerKind.Exact,
            (Fixed64)5,
            Vector3d.Right);
        PhysicsMixedHit farther = new(
            candidate3D,
            candidate2D,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            PhysicsQueryReducerKind.Exact,
            (Fixed64)6,
            Vector3d.Right);
        PhysicsMixedHit earlierIdTie = new(
            candidate3D,
            candidate2D,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            PhysicsQueryReducerKind.Exact,
            (Fixed64)5,
            Vector3d.Right);

        PhysicsHitSelectionPolicy.ShouldReplace(farther, found: false, current).Should().BeTrue();
        PhysicsHitSelectionPolicy.ShouldReplace(farther, found: true, current).Should().BeFalse();
        PhysicsHitSelectionPolicy.ShouldReplace(earlierIdTie, found: true, current).Should().BeTrue();
    }

    [Fact]
    public void ShouldReplaceDistance_ShouldAcceptFirstAndSmallerDistanceOnly()
    {
        PhysicsHitSelectionPolicy.ShouldReplaceDistance((Fixed64)5, found: false, (Fixed64)4).Should().BeTrue();
        PhysicsHitSelectionPolicy.ShouldReplaceDistance((Fixed64)5, found: true, (Fixed64)4).Should().BeFalse();
        PhysicsHitSelectionPolicy.ShouldReplaceDistance((Fixed64)4, found: true, (Fixed64)4).Should().BeFalse();
        PhysicsHitSelectionPolicy.ShouldReplaceDistance((Fixed64)3, found: true, (Fixed64)4).Should().BeTrue();
    }

    private static LSCircleCollider2D CreateCircle2D() => new(Fixed64.One);
}
