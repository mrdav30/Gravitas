//=======================================================================
// MixedContactTests.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class MixedContactTests
{
    [Fact]
    public void RelativeAnchors_ShouldRemainAuthoritativeWhenWorldPointsCannotMaterialize()
    {
        Vector3d offset3D = Vector3d.Right * Fixed64.Half;
        Vector3d offset2D = Vector3d.Left * Fixed64.Half;
        var contact = new MixedContact(
            new ContactAnchor(
                new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
                offset3D),
            new ContactAnchor(
                new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
                offset2D),
            Vector3d.Right,
            Fixed64.MaxValue,
            depthIsClamped: true);

        contact.HasContact.Should().BeTrue();
        contact.Anchor3D.Offset.Should().Be(offset3D);
        contact.Anchor2D.Offset.Should().Be(offset2D);
        contact.TryGetPoint3D(out _).Should().BeFalse();
        contact.TryGetPoint2D(out _).Should().BeFalse();
        contact.Depth.Should().Be(Fixed64.MaxValue);
        contact.DepthIsClamped.Should().BeTrue();
        FluentActions.Invoking(() => _ = contact.Point3D)
            .Should()
            .Throw<InvalidOperationException>()
            .WithMessage("*TryGet*");
        FluentActions.Invoking(() => _ = contact.Point2D)
            .Should()
            .Throw<InvalidOperationException>()
            .WithMessage("*TryGet*");
    }

    [Fact]
    public void MaterialCopies_ShouldPreserveRelativeAnchorsAndDepthClampState()
    {
        var contact = new MixedContact(
            new ContactAnchor(Vector3d.Right, Vector3d.Up),
            new ContactAnchor(Vector3d.Left, Vector3d.Down),
            Vector3d.Right,
            Fixed64.One,
            depthIsClamped: true);
        var material3D = new PhysicsMaterial(Fixed64.One, Fixed64.Half, Fixed64.Zero);
        var material2D = new PhysicsMaterial(Fixed64.Half, Fixed64.Half, Fixed64.One);

        MixedContact overridden = contact.WithMaterialOverride(material3D, material2D);

        overridden.Anchor3D.Should().Be(contact.Anchor3D);
        overridden.Anchor2D.Should().Be(contact.Anchor2D);
        overridden.DepthIsClamped.Should().BeTrue();
        overridden.Material3D.Should().Be(material3D);
        overridden.Material2D.Should().Be(material2D);
    }

    [Fact]
    public void PlanarOffsetConvenience_ShouldRebaseAgainstUnrotatedBodyOrigin()
    {
        var contact = new MixedContact(
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            new ContactAnchor(
                new Vector3d((Fixed64)10, (Fixed64)3, (Fixed64)20),
                new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Two)),
            Vector3d.Right,
            Fixed64.One);

        contact.TryGetPlanarOffset2DFrom(
                new Vector2d((Fixed64)10, (Fixed64)20),
                out Vector2d offset)
            .Should()
            .BeTrue();

        offset.Should().Be(new Vector2d(Fixed64.One, Fixed64.Two));
    }

    [Fact]
    public void RelativeAnchorConstructor_ShouldRejectEitherInvalidRigidFrame()
    {
        ContactAnchor valid = ContactAnchor.FromWorldPoint(Vector3d.Zero);
        Action createInvalid3D = () =>
            _ = new MixedContact(
                default,
                valid,
                Vector3d.Right,
                Fixed64.Zero);
        Action createInvalid2D = () =>
            _ = new MixedContact(
                valid,
                default,
                Vector3d.Right,
                Fixed64.Zero);

        createInvalid3D.Should()
            .Throw<ArgumentException>()
            .WithParameterName("anchor3D");
        createInvalid2D.Should()
            .Throw<ArgumentException>()
            .WithParameterName("anchor2D");
    }
}
