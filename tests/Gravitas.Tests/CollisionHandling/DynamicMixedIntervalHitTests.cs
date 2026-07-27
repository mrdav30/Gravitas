using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class DynamicMixedIntervalHitTests
{
    [Fact]
    public void ShouldReplace_ShouldOrderDistanceStatusAndStableIdentity()
    {
        DynamicMixedIntervalHit unresolvedLowId = CreateUnresolved(
            safeDistance: Fixed64.One,
            targetId: 1);
        DynamicMixedIntervalHit unresolvedHighId = CreateUnresolved(
            safeDistance: Fixed64.One,
            targetId: 2);
        DynamicMixedIntervalHit exact = CreateExact(
            safeDistance: Fixed64.One,
            exactDistance: Fixed64.One);

        DynamicMixedIntervalHit.ShouldReplace(
                unresolvedHighId,
                default,
                hasCurrent: false)
            .Should()
            .BeTrue();
        DynamicMixedIntervalHit.ShouldReplace(
                CreateUnresolved(Fixed64.Half, 2),
                unresolvedLowId,
                hasCurrent: true)
            .Should()
            .BeTrue();
        DynamicMixedIntervalHit.ShouldReplace(
                CreateUnresolved(Fixed64.Two, 2),
                unresolvedLowId,
                hasCurrent: true)
            .Should()
            .BeFalse();
        DynamicMixedIntervalHit.ShouldReplace(
                exact,
                unresolvedLowId,
                hasCurrent: true)
            .Should()
            .BeTrue();
        DynamicMixedIntervalHit.ShouldReplace(
                unresolvedLowId,
                exact,
                hasCurrent: true)
            .Should()
            .BeFalse();
        DynamicMixedIntervalHit.ShouldReplace(
                CreateExact(
                    safeDistance: Fixed64.One,
                    exactDistance: Fixed64.Half),
                exact,
                hasCurrent: true)
            .Should()
            .BeTrue();
        DynamicMixedIntervalHit.ShouldReplace(
                unresolvedLowId,
                unresolvedHighId,
                hasCurrent: true)
            .Should()
            .BeTrue();
    }

    [Fact]
    public void ShouldReplaceStatic_ShouldRequireAnAdmittedEarlierOrExactCandidate()
    {
        PhysicsMixedHit current = CreateHit(Fixed64.One);
        DynamicMixedIntervalHit unresolved = CreateUnresolved(
            safeDistance: Fixed64.One,
            targetId: 1);

        DynamicMixedIntervalHit.ShouldReplaceStatic(
                unresolved,
                hasCandidate: false,
                current,
                hasCurrent: true)
            .Should()
            .BeFalse();
        DynamicMixedIntervalHit.ShouldReplaceStatic(
                unresolved,
                hasCandidate: true,
                current,
                hasCurrent: false)
            .Should()
            .BeTrue();
        DynamicMixedIntervalHit.ShouldReplaceStatic(
                CreateUnresolved(Fixed64.Half, 1),
                hasCandidate: true,
                current,
                hasCurrent: true)
            .Should()
            .BeTrue();
        DynamicMixedIntervalHit.ShouldReplaceStatic(
                unresolved,
                hasCandidate: true,
                current,
                hasCurrent: true)
            .Should()
            .BeFalse();
        DynamicMixedIntervalHit.ShouldReplaceStatic(
                CreateExact(Fixed64.One, Fixed64.Half),
                hasCandidate: true,
                current,
                hasCurrent: true)
            .Should()
            .BeTrue();
    }

    [Fact]
    public void Select_ShouldRetainOrReplaceAndPublishOwnership()
    {
        DynamicMixedIntervalHit current = CreateUnresolved(Fixed64.One, 1);
        bool hasCurrent = true;

        DynamicMixedIntervalHit retained = DynamicMixedIntervalHit.Select(
            CreateUnresolved(Fixed64.Two, 2),
            current,
            ref hasCurrent);
        retained.TargetId.Should().Be(1);
        hasCurrent.Should().BeTrue();

        hasCurrent = false;
        DynamicMixedIntervalHit selected = DynamicMixedIntervalHit.Select(
            CreateUnresolved(Fixed64.One, 2),
            default,
            ref hasCurrent);
        selected.TargetId.Should().Be(2);
        hasCurrent.Should().BeTrue();
    }

    [Fact]
    public void DimensionalSelection_ShouldHonorPresenceDistanceAndStableTieOrder()
    {
        DynamicMixedIntervalHit.ShouldSelect2D(
                has2D: false,
                Fixed64.One,
                hasMixed: true,
                Fixed64.One)
            .Should()
            .BeFalse();
        DynamicMixedIntervalHit.ShouldSelect2D(
                has2D: true,
                Fixed64.One,
                hasMixed: false,
                Fixed64.Zero)
            .Should()
            .BeTrue();
        DynamicMixedIntervalHit.ShouldSelect2D(
                has2D: true,
                Fixed64.Half,
                hasMixed: true,
                Fixed64.One)
            .Should()
            .BeTrue();

        DynamicMixedIntervalHit.ShouldSelect3D(
                has3D: false,
                Fixed64.One,
                hasMixed: true,
                Fixed64.One)
            .Should()
            .BeFalse();
        DynamicMixedIntervalHit.ShouldSelect3D(
                has3D: true,
                Fixed64.One,
                hasMixed: false,
                Fixed64.Zero)
            .Should()
            .BeTrue();
        DynamicMixedIntervalHit.ShouldSelect3D(
                has3D: true,
                Fixed64.Half,
                hasMixed: true,
                Fixed64.One)
            .Should()
            .BeTrue();
    }

    private static DynamicMixedIntervalHit CreateUnresolved(
        Fixed64 safeDistance,
        int targetId) =>
        new(
            ContinuousCollisionMath.IntervalSearchStatus.Unresolved,
            default,
            safeDistance,
            Fixed64.Zero,
            targetId);

    private static DynamicMixedIntervalHit CreateExact(
        Fixed64 safeDistance,
        Fixed64 exactDistance) =>
        new(
            ContinuousCollisionMath.IntervalSearchStatus.ExactHit,
            CreateHit(exactDistance),
            safeDistance,
            Fixed64.One,
            targetId: 1);

    private static PhysicsMixedHit CreateHit(Fixed64 distance) =>
        new(
            null,
            null,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            PhysicsQueryReducerKind.Exact,
            distance,
            Vector3d.Right);
}
