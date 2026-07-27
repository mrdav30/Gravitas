//=======================================================================
// MixedQueryCcdTests.RefinementBudget.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using SwiftCollections.Query;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Fact]
    public void MixedContinuous2D_RefinementExhaustion_ShouldClampReportAndRefreshOutsideResponseBudget()
    {
        var sourceFirst = Run2DRefinementExhaustion(targetFirst: false);
        var sourceFirstRepeat = Run2DRefinementExhaustion(targetFirst: false);
        var targetFirst = Run2DRefinementExhaustion(targetFirst: true);

        sourceFirstRepeat.Should().Be(sourceFirst);
        targetFirst.Should().Be(sourceFirst);
        sourceFirst.SourceToiIterations.Should().Be(1);
        sourceFirst.SourceLimitReached.Should().BeTrue();
        sourceFirst.IslandLimitReached.Should().BeTrue();
        sourceFirst.SourceTrajectoryCount.Should().Be(2);
        sourceFirst.SourceVelocity.Should().Be(
            Vector2d.Forward * Fixed64.FromFraction(1, 5));
        Vector2d certifiedFrontier = new(
            Fixed64.FromRaw(13_314_398_618L),
            Fixed64.FromRaw(-193_147_700L));
        sourceFirst.SourcePosition.Should().Be(certifiedFrontier);
        sourceFirst.ColliderCenter.Should().Be(certifiedFrontier);
        sourceFirst.TerminalStart.Should().Be(certifiedFrontier);
        sourceFirst.TerminalEnd.Should().Be(certifiedFrontier);
        sourceFirst.TerminalDisplacement.Should().Be(Vector2d.Zero);
        sourceFirst.FrontierCandidateRetained.Should().BeTrue();
        sourceFirst.StaleTailCandidateRetained.Should().BeFalse();
        sourceFirst.IsMixedPartitioned.Should().BeTrue();
    }

    [Fact]
    public void MixedContinuous3D_RefinementExhaustion_ShouldClampReportAndRefreshOutsideResponseBudget()
    {
        var sourceFirst = Run3DRefinementExhaustion(targetFirst: false);
        var sourceFirstRepeat = Run3DRefinementExhaustion(targetFirst: false);
        var targetFirst = Run3DRefinementExhaustion(targetFirst: true);

        sourceFirstRepeat.Should().Be(sourceFirst);
        targetFirst.Should().Be(sourceFirst);
        sourceFirst.SourceToiIterations.Should().Be(1);
        sourceFirst.SourceLimitReached.Should().BeTrue();
        sourceFirst.IslandLimitReached.Should().BeTrue();
        sourceFirst.SourceTrajectoryCount.Should().Be(2);
        sourceFirst.SourceVelocity.Should().Be(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.FromRaw(661_693_395L)));
        Vector3d certifiedFrontier = new(
            Fixed64.FromRaw(13_314_398_618L),
            Fixed64.Zero,
            Fixed64.FromRaw(-247_317_022L));
        sourceFirst.SourcePosition.Should().Be(certifiedFrontier);
        sourceFirst.TerminalStart.Should().Be(certifiedFrontier);
        sourceFirst.TerminalEnd.Should().Be(certifiedFrontier);
        sourceFirst.TerminalDisplacement.Should().Be(Vector3d.Zero);
        sourceFirst.ColliderCenter.Should().Be(sourceFirst.SourcePosition);
        sourceFirst.FrontierCandidateRetained.Should().BeTrue();
        sourceFirst.StaleTailCandidateRetained.Should().BeFalse();
        sourceFirst.IsMixedPartitioned.Should().BeTrue();
    }

    private static RefinementExhaustionResult2D Run2DRefinementExhaustion(
        bool targetFirst)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Settings.ContinuousCollisionMaxToiIterations = 1;

        FixedQuaternion startRotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Up,
            FixedMath.DegToRad((Fixed64)(-45)));
        FixedQuaternion targetRotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Up,
            FixedMath.DegToRad((Fixed64)45));
        Vector2d sourcePosition = new(
            Fixed64.FromFraction(31, 10),
            Fixed64.FromFraction(-1, 10));

        ScenarioBody<LSCuboidCollider> target;
        SolidBody2D source;
        if (targetFirst)
        {
            target = CreateRotationalMixedBlade3D(context);
            source = CreateRefinementSource2D(context, sourcePosition);
        }
        else
        {
            source = CreateRefinementSource2D(context, sourcePosition);
            target = CreateRotationalMixedBlade3D(context);
        }

        target.Body.ResetPosition(Vector3d.Zero, startRotation);
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.Agent.Transform.LocalRotation = targetRotation;
        source.AddLinearImpulse(
            Vector2d.Forward * Fixed64.FromFraction(1, 5));

        context.LateSimulate();

        ContinuousCollisionMotionSegment2D terminal =
            source.GetContinuousCollisionTrajectorySegment(
                source.ContinuousCollisionTrajectoryCount - 1);
        FixedBoundVolume frontierQuery =
            DynamicCcdCandidateIndex.CreateBoundsBetween(
                source.Position.ToVector3d(Fixed64.Zero),
                source.Position.ToVector3d(Fixed64.Zero),
                Vector3d.One * Fixed64.MinIncrement);
        Vector3d staleTailProbe = new(
            source.Position.X,
            Fixed64.Zero,
            Fixed64.FromFraction(3, 8));
        FixedBoundVolume staleTailQuery =
            DynamicCcdCandidateIndex.CreateBoundsBetween(
                staleTailProbe,
                staleTailProbe,
                Vector3d.One * Fixed64.MinIncrement);
        bool frontierCandidateRetained =
            context.Physics2D
                .QueryMixedContinuousCollisionCandidates(frontierQuery)
                .Contains(source.DynamicId);
        bool staleTailCandidateRetained =
            context.Physics2D
                .QueryMixedContinuousCollisionCandidates(staleTailQuery)
                .Contains(source.DynamicId);

        return new RefinementExhaustionResult2D(
            source.Position,
            source.LinearVelocity,
            source.LastContinuousCollisionToiIterationCount,
            source.LastContinuousCollisionToiIterationLimitReached,
            context.Physics2D.LastContinuousCollisionIslandLimitReached,
            source.ContinuousCollisionTrajectoryCount,
            terminal.StartPosition,
            terminal.EndPosition,
            terminal.Displacement,
            source.Collider.Center,
            frontierCandidateRetained,
            staleTailCandidateRetained,
            source.Collider.IsMixedPartitioned);
    }

    private static RefinementExhaustionResult3D Run3DRefinementExhaustion(
        bool targetFirst)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Settings.ContinuousCollisionMaxToiIterations = 1;

        Fixed64 startRotation =
            -FixedMath.DegToRad((Fixed64)45);
        Fixed64 targetRotation =
            FixedMath.DegToRad((Fixed64)45);
        Vector3d sourcePosition = new(
            Fixed64.FromFraction(31, 10),
            Fixed64.Zero,
            Fixed64.FromFraction(-1, 10));

        SolidBody2D target;
        ScenarioBody<LSCuboidCollider> source;
        if (targetFirst)
        {
            target = CreateRotationalMixedBlade2D(context);
            source = CreateRefinementSource3D(context, sourcePosition);
        }
        else
        {
            source = CreateRefinementSource3D(context, sourcePosition);
            target = CreateRotationalMixedBlade2D(context);
        }

        target.ResetPosition(Vector2d.Zero, startRotation);
        target.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        source.Body.ContinuousCollisionMode =
            ContinuousCollisionMode.Continuous;
        target.Agent.Transform.LocalRotationXZRadians = targetRotation;
        source.Body.AddLinearImpulse(
            Vector3d.Forward * Fixed64.FromFraction(1, 5));

        context.LateSimulate();

        ContinuousCollisionMotionSegment3D terminal =
            source.Body.GetContinuousCollisionTrajectorySegment(
                source.Body.ContinuousCollisionTrajectoryCount - 1);
        FixedBoundVolume frontierQuery =
            DynamicCcdCandidateIndex.CreateBoundsBetween(
                source.Body.Position3d,
                source.Body.Position3d,
                Vector3d.One * Fixed64.MinIncrement);
        Vector3d staleTailProbe = new(
            source.Body.Position3d.X,
            Fixed64.Zero,
            Fixed64.FromFraction(9, 20));
        FixedBoundVolume staleTailQuery =
            DynamicCcdCandidateIndex.CreateBoundsBetween(
                staleTailProbe,
                staleTailProbe,
                Vector3d.One * Fixed64.MinIncrement);
        bool frontierCandidateRetained =
            context.Physics
                .QueryContinuousCollisionCandidates(frontierQuery)
                .Contains(source.Body.DynamicId);
        bool staleTailCandidateRetained =
            context.Physics
                .QueryContinuousCollisionCandidates(staleTailQuery)
                .Contains(source.Body.DynamicId);

        return new RefinementExhaustionResult3D(
            source.Body.Position3d,
            source.Body.LinearVelocity,
            source.Body.LastContinuousCollisionToiIterationCount,
            source.Body.LastContinuousCollisionToiIterationLimitReached,
            context.Physics.LastContinuousCollisionIslandLimitReached,
            source.Body.ContinuousCollisionTrajectoryCount,
            terminal.StartPosition,
            terminal.EndPosition,
            terminal.Displacement,
            source.Collider.Center,
            frontierCandidateRetained,
            staleTailCandidateRetained,
            source.Collider.IsMixedPartitioned);
    }

    private static SolidBody2D CreateRefinementSource2D(
        GravitasWorldContext context,
        Vector2d position)
    {
        Fixed64 halfSize = Fixed64.FromFraction(1, 4);
        var collider = new LSPolygonCollider2D(
            new Vector2d(-halfSize, -halfSize),
            new Vector2d(halfSize, -halfSize),
            new Vector2d(halfSize, halfSize),
            new Vector2d(-halfSize, halfSize));
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(
                position.ToVector3d(Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.One));
        var source = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };
        source.Initialize(position);
        return source;
    }

    private static ScenarioBody<LSCuboidCollider> CreateRefinementSource3D(
        GravitasWorldContext context,
        Vector3d position) =>
        CreateBody3D(
            context,
            new LSCuboidCollider
            {
                Size = Vector3d.One * Fixed64.Half
            },
            position);

    private readonly record struct RefinementExhaustionResult2D(
        Vector2d SourcePosition,
        Vector2d SourceVelocity,
        int SourceToiIterations,
        bool SourceLimitReached,
        bool IslandLimitReached,
        int SourceTrajectoryCount,
        Vector2d TerminalStart,
        Vector2d TerminalEnd,
        Vector2d TerminalDisplacement,
        Vector2d ColliderCenter,
        bool FrontierCandidateRetained,
        bool StaleTailCandidateRetained,
        bool IsMixedPartitioned);

    private readonly record struct RefinementExhaustionResult3D(
        Vector3d SourcePosition,
        Vector3d SourceVelocity,
        int SourceToiIterations,
        bool SourceLimitReached,
        bool IslandLimitReached,
        int SourceTrajectoryCount,
        Vector3d TerminalStart,
        Vector3d TerminalEnd,
        Vector3d TerminalDisplacement,
        Vector3d ColliderCenter,
        bool FrontierCandidateRetained,
        bool StaleTailCandidateRetained,
        bool IsMixedPartitioned);
}
