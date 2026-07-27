//=======================================================================
// PhysicsMixedHitTests.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;
using Gravitas.Queries;
using System;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed class PhysicsMixedHitTests
{
    [Fact]
    public void WorldPointConstructor_ShouldExposeMaterializedAnchors()
    {
        Vector3d point3D = new(Fixed64.One, (Fixed64)2, (Fixed64)3);
        Vector3d point2D = new((Fixed64)4, (Fixed64)5, (Fixed64)6);
        var hit = new PhysicsMixedHit(
            null,
            null,
            point3D,
            point2D,
            Vector3d.Right,
            PhysicsQueryReducerKind.Exact,
            Fixed64.Half,
            Vector3d.Forward);

        hit.TryGetPoint3D(out Vector3d materialized3D).Should().BeTrue();
        materialized3D.Should().Be(point3D);
        hit.TryGetPoint2D(out Vector3d materialized2D).Should().BeTrue();
        materialized2D.Should().Be(point2D);
        hit.Point3D.Should().Be(point3D);
        hit.Point2D.Should().Be(point2D);
    }

    [Fact]
    public void RelativeAnchorConstructor_ShouldPreserveUnmaterializableWitnesses()
    {
        ContactAnchor anchor3D = new(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right);
        ContactAnchor anchor2D = new(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            -Vector3d.Right);
        var hit = new PhysicsMixedHit(
            null,
            null,
            anchor3D,
            anchor2D,
            Vector3d.Right,
            PhysicsQueryReducerKind.Exact,
            Fixed64.Zero,
            Vector3d.Forward);

        hit.Anchor3D.Should().Be(anchor3D);
        hit.Anchor2D.Should().Be(anchor2D);
        hit.TryGetPoint3D(out _).Should().BeFalse();
        hit.TryGetPoint2D(out _).Should().BeFalse();
        FluentActions.Invoking(() => _ = hit.Point3D)
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*TryGet*");
        FluentActions.Invoking(() => _ = hit.Point2D)
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*TryGet*");
    }

    [Fact]
    public void RelativeAnchorConstructor_ShouldRejectEitherInvalidRigidFrame()
    {
        ContactAnchor valid = ContactAnchor.FromWorldPoint(Vector3d.Zero);
        Action createInvalid3D = () =>
            _ = new PhysicsMixedHit(
                null,
                null,
                default,
                valid,
                Vector3d.Right,
                PhysicsQueryReducerKind.Exact,
                Fixed64.Zero,
                Vector3d.Forward);
        Action createInvalid2D = () =>
            _ = new PhysicsMixedHit(
                null,
                null,
                valid,
                default,
                Vector3d.Right,
                PhysicsQueryReducerKind.Exact,
                Fixed64.Zero,
                Vector3d.Forward);

        createInvalid3D.Should()
            .Throw<ArgumentException>()
            .WithParameterName("anchor3D");
        createInvalid2D.Should()
            .Throw<ArgumentException>()
            .WithParameterName("anchor2D");
    }

    [Fact]
    public void MixedQueryDiagnostics_ShouldReportUnavailableRelativeWitnesses()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Diagnostics.Enable(eventCapacity: 1, drawCommandCapacity: 0);
        var hit = new PhysicsMixedHit(
            null,
            null,
            new ContactAnchor(
                new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right),
            new ContactAnchor(
                new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
                -Vector3d.Right),
            Vector3d.Right,
            PhysicsQueryReducerKind.Exact,
            Fixed64.Zero,
            Vector3d.Forward);

        context.Diagnostics.EmitMixedQuery(
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.Half,
            layerMaskBits: 1,
            hit: true,
            hitCount: 1,
            hit);

        GravitasDiagnosticEvent diagnostic = context.Diagnostics.Events[0];
        diagnostic.Kind.Should().Be(GravitasDiagnosticEventKind.MixedQuery);
        diagnostic.HasPointA.Should().BeFalse();
        diagnostic.HasPointB.Should().BeFalse();
    }
}
