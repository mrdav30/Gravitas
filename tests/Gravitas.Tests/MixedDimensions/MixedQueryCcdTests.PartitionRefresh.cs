//=======================================================================
// MixedQueryCcdTests.PartitionRefresh.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Fact]
    public void SweepSphereAgainst2D_AfterBodylessHostPoseChange_ShouldRebuildBeforePartitionRefresh()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D target = CreateBodylessCircle2D(context, Vector2d.Zero);
        Vector3d oldStart = new(Fixed64.Zero, (Fixed64)3, Fixed64.Zero);
        Vector3d oldEnd = new(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero);
        Vector3d newStart = new((Fixed64)8, (Fixed64)3, Fixed64.Zero);
        Vector3d newEnd = new((Fixed64)8, (Fixed64)(-3), Fixed64.Zero);

        context.QueryMixed.SweepSphereAgainst2D(
                oldStart,
                oldEnd,
                Fixed64.Half,
                PhysicsLayerMask.All,
                out _)
            .Should()
            .BeTrue();

        target.Agent.Transform.LocalPosition = new Vector3d(
            (Fixed64)8,
            Fixed64.Zero,
            Fixed64.Zero);

        context.QueryMixed.SweepSphereAgainst2D(
                oldStart,
                oldEnd,
                Fixed64.Half,
                PhysicsLayerMask.All,
                out _)
            .Should()
            .BeFalse();
        context.QueryMixed.SweepSphereAgainst2D(
                newStart,
                newEnd,
                Fixed64.Half,
                PhysicsLayerMask.All,
                out PhysicsMixedHit hit)
            .Should()
            .BeTrue();

        target.Center.Should().Be(new Vector2d((Fixed64)8, Fixed64.Zero));
        hit.Collider2D.Should().BeSameAs(target);
    }
}
