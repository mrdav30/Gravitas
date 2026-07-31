//=======================================================================
// ColliderSurfaceDomainTests.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
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
    public void Round2DSurfacesAtScalarFace_ShouldRejectOnlyUnrepresentableClosestPoints()
    {
        using GravitasWorldContext context = CreatePositiveScalarFaceContext();
        var circle = new LSCircleCollider2D(Fixed64.Half);
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two);
        Initialize(context, circle);
        Initialize(context, capsule);

        FluentActions.Invoking(() => circle.GetClosestPoint(circle.Center))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*coordinate*domain*");
        circle.GetClosestPoint(
                new Vector2d(Fixed64.MinValue, Fixed64.Zero))
            .Should().Be(
                circle.Center - Vector2d.Right * Fixed64.Half);
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
    public void CompoundOffsetPolygonAtScalarFace_ShouldRejectUnrepresentableClosestPoint()
    {
        using GravitasWorldContext context = CreatePositiveScalarFaceContext();
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.ConvexPolygon(
                new[]
                {
                    new Vector2d(Fixed64.One, -Fixed64.One),
                    new Vector2d(Fixed64.Two, -Fixed64.One),
                    new Vector2d(Fixed64.Two, Fixed64.One),
                    new Vector2d(Fixed64.One, Fixed64.One)
                },
                Vector2d.Zero,
                Fixed64.Zero,
                Vector2d.One));
        Initialize(context, collider);

        FluentActions.Invoking(() =>
                collider.GetClosestPoint(collider.Center))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*coordinate*domain*");
        collider.TryGetClosestBoundaryAnchor(
                new Vector2d(Fixed64.MinValue, Fixed64.Zero),
                out _,
                out _)
            .Should().BeFalse();
    }

    [Fact]
    public void FiniteRound3DSurfacesAtScalarFace_ShouldRejectOnlyUnrepresentablePoints()
    {
        using GravitasWorldContext context = CreatePositiveScalarFaceContext();
        var sphere = new LSSphereCollider
        {
            Radius = Fixed64.Half
        };
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
        Initialize(context, sphere);
        Initialize(context, capsule);
        Initialize(context, cylinder);
        Initialize(context, cone);

        AssertSurfacePointOutsideDomain(sphere);
        AssertSurfacePointOutsideDomain(capsule);
        AssertSurfacePointOutsideDomain(cylinder);
        AssertSurfacePointOutsideDomain(cone);
        AssertRepresentableInwardSurface(sphere);
        AssertRepresentableInwardSurface(capsule);
        AssertRepresentableInwardSurface(cylinder);
        AssertRepresentableInwardSurface(cone);
    }

    [Fact]
    public void CompoundAtScalarFace_ShouldRankSemanticPartAnchorsExactly()
    {
        using GravitasWorldContext context = CreatePositiveScalarFaceContext();
        var collider = new LSCompoundCollider(
            CompoundColliderPart.Sphere(
                Fixed64.Half,
                Vector3d.Zero),
            CompoundColliderPart.Sphere(
                Fixed64.Half,
                -Vector3d.Right * Fixed64.Two));
        Initialize(context, collider);

        Vector3d closest = collider.ClosestPointOnSurface(
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero));

        closest.Should().Be(new Vector3d(
            ScalarFaceCenter.X - Fixed64.FromFraction(5, 2),
            Fixed64.Zero,
            Fixed64.Zero));
        collider.GetNormalAtPoint(
                new Vector3d(
                    Fixed64.MinValue,
                    Fixed64.Zero,
                    Fixed64.Zero))
            .Should()
            .Be(Vector3d.Left);
    }

    [Fact]
    public void CompoundAtScalarScale_ShouldPreserveFirstAuthoredExactTie()
    {
        using GravitasWorldContext context =
            CreatePositiveScalarFaceContext();
        var collider = new LSCompoundCollider(
            CompoundColliderPart.Sphere(
                Fixed64.Half,
                Vector3d.Up),
            CompoundColliderPart.Sphere(
                Fixed64.Half,
                Vector3d.Down));
        Initialize(context, collider);

        Vector3d closest = collider.ClosestPointOnSurface(
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero));

        closest.Y.Should().Be(Fixed64.One);
    }

    [Fact]
    public void CompoundAtScalarFace_ShouldRejectUnrepresentableSelectedSurface()
    {
        using GravitasWorldContext context =
            CreatePositiveScalarFaceContext();
        var collider = new LSCompoundCollider(
            CompoundColliderPart.Sphere(
                Fixed64.Half,
                Vector3d.Zero));
        Initialize(context, collider);

        FluentActions.Invoking(() =>
                collider.ClosestPointOnSurface(collider.Center))
            .Should()
            .Throw<InvalidOperationException>()
            .WithMessage("*outside*representable*domain*");
    }

    [Fact]
    public void SphereCenter_ShouldSeparateSurfaceFallbackFromUndefinedPointNormal()
    {
        using GravitasWorldContext context =
            GravitasWorldContext.CreateOwned();
        var collider = new LSSphereCollider
        {
            Radius = Fixed64.Half
        };
        collider.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.One)));

        collider.ClosestPointOnSurface(collider.Center)
            .Should()
            .Be(Vector3d.Right * Fixed64.Half);
        collider.GetNormalAtPoint(collider.Center)
            .Should()
            .Be(Vector3d.Zero);
        FixedPointAnchor anchor =
            collider.GetClosestSurfaceAnchor(
                collider.Center,
                out Vector3d anchorNormal);
        anchor.TryGetPoint(out Vector3d anchorPoint)
            .Should()
            .BeTrue();
        anchorPoint.Should().Be(Vector3d.Right * Fixed64.Half);
        anchorNormal.Should().Be(Vector3d.Right);
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

    private static void AssertRepresentableInwardSurface(LSCollider collider)
    {
        Vector3d query = new(
            Fixed64.MinValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Vector3d closest = collider.ClosestPointOnSurface(query);
        closest.X.Should().BeLessThan(collider.Center.X);
        collider.GetNormalAtPoint(query).IsNormalized().Should().BeTrue();
        collider.GetNormalAtPoint(query).X.Should().BeLessThan(Fixed64.Zero);
    }

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
