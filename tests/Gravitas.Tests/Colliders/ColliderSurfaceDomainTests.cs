//=======================================================================
// ColliderSurfaceDomainTests.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderSurfaceDomainTests
{
    [Fact]
    public void AxisAlignedBoxAtScalarFace_ShouldReturnRepresentableInwardClosestPoint()
    {
        using GravitasWorldContext context = CreatePositiveScalarFaceContext();
        var collider = new LSAABBoxCollider2D(Vector2d.One);
        Initialize(context, collider);
        Vector2d query = collider.Center - Vector2d.Right;

        collider.GetClosestPoint(query).Should().Be(
            collider.Center - Vector2d.Right * Fixed64.Half);
    }

    [Fact]
    public void Round2DSurfacesAtScalarFace_ShouldRejectUnrepresentableClosestPoints()
    {
        using GravitasWorldContext context = CreatePositiveScalarFaceContext();
        var circle = new LSCircleCollider2D(Fixed64.Half);
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two);
        Initialize(context, circle);
        Initialize(context, capsule);

        FluentActions.Invoking(() => circle.GetClosestPoint(circle.Center))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*coordinate*domain*");
        FluentActions.Invoking(() => circle.GetClosestPoint(
                new Vector2d(Fixed64.MinValue, Fixed64.Zero)))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*coordinate*domain*");
        FluentActions.Invoking(() => capsule.GetClosestPoint(capsule.Center))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*coordinate*domain*");
    }

    [Fact]
    public void OffsetPolygonAtScalarFace_ShouldRejectUnrepresentableClosestPoint()
    {
        using GravitasWorldContext context = CreatePositiveScalarFaceContext();
        var collider = new LSPolygonCollider2D(
            new Vector2d(Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.Two, -Fixed64.One),
            new Vector2d(Fixed64.Two, Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.One));
        Initialize(context, collider);

        FluentActions.Invoking(() => collider.GetClosestPoint(collider.Center))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*coordinate*domain*");
        collider.TryGetClosestBoundaryAnchor(
                collider.Center,
                out _,
                out _)
            .Should().BeTrue();
        collider.TryGetClosestBoundaryAnchor(
                new Vector2d(Fixed64.MinValue, Fixed64.Zero),
                out _,
                out _)
            .Should().BeFalse();
    }

    [Fact]
    public void FiniteRound3DSurfacesAtScalarFace_ShouldRejectUnrepresentableResults()
    {
        using GravitasWorldContext context = CreatePositiveScalarFaceContext();
        var capsule = new LSCapsuleCollider
        {
            Radius = Fixed64.Half,
            Size = new Vector3d(Fixed64.One, Fixed64.Two, Fixed64.One)
        };
        var cylinder = new LSCylinderCollider
        {
            Radius = Fixed64.Half,
            Size = Vector3d.One
        };
        var cone = new LSConeCollider
        {
            Radius = Fixed64.Half,
            Size = Vector3d.One
        };
        Initialize(context, capsule);
        Initialize(context, cylinder);
        Initialize(context, cone);

        AssertSurfacePointOutsideDomain(capsule);
        AssertSurfacePointOutsideDomain(cylinder);
        AssertSurfacePointOutsideDomain(cone);
        AssertSurfaceDistanceOutsideDomain(capsule);
        AssertSurfaceDistanceOutsideDomain(cylinder);
        AssertSurfaceDistanceOutsideDomain(cone);
        AssertSurfaceNormalOutsideDomain(capsule);
        AssertSurfaceNormalOutsideDomain(cylinder);
        AssertSurfaceNormalOutsideDomain(cone);
    }

    [Fact]
    public void CuboidAtScalarFace_ShouldRejectUnrepresentableSelectedSurface()
    {
        using GravitasWorldContext context = CreatePositiveScalarFaceContext();
        var collider = new LSCuboidCollider { Size = Vector3d.One };
        Initialize(context, collider);

        FluentActions.Invoking(() => collider.ClosestPointOnSurface(collider.Center))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*coordinate*domain*");
    }

    private static void AssertSurfacePointOutsideDomain(LSCollider collider) =>
        FluentActions.Invoking(() => collider.ClosestPointOnSurface(collider.Center))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*representable*domain*");

    private static void AssertSurfaceDistanceOutsideDomain(LSCollider collider) =>
        FluentActions.Invoking(() => collider.ClosestPointOnSurface(
                new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero)))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*representable*domain*");

    private static void AssertSurfaceNormalOutsideDomain(LSCollider collider) =>
        FluentActions.Invoking(() => collider.GetNormalAtPoint(
                new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero)))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*representable*domain*");

    private static void Initialize(
        GravitasWorldContext context,
        LSCollider collider)
    {
        var transform = new FixedTransform(
            ScalarFaceCenter,
            FixedQuaternion.Identity,
            Vector3d.One);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
    }

    private static void Initialize(
        GravitasWorldContext context,
        LSCollider2D collider)
    {
        var transform = new FixedTransform(
            ScalarFaceCenter,
            FixedQuaternion.Identity,
            Vector3d.One);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
    }

    private static GravitasWorldContext CreatePositiveScalarFaceContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d(
                    Fixed64.MaxValue - (Fixed64)4,
                    (Fixed64)(-4),
                    (Fixed64)(-4)),
                new Vector3d(
                    Fixed64.MaxValue,
                    (Fixed64)4,
                    (Fixed64)4)),
            out _).Should().BeTrue();
        return context;
    }

    private static Vector3d ScalarFaceCenter => new(
        Fixed64.MaxValue - Fixed64.FromFraction(1, 8),
        Fixed64.Zero,
        Fixed64.Zero);
}
