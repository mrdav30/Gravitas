//=======================================================================
// ContactAnchorTests.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContactAnchorTests
{
    [Fact]
    public void PublicAnchorConsumers_ShouldRejectAnExplicitInvalidRigidFrame()
    {
        Action wrapInvalidPoint = () =>
            _ = new ContactAnchor(default(FixedPointAnchor));
        Action createInvalidHit = () =>
            _ = new Physics3DHit(
                null,
                default(ContactAnchor),
                Vector3d.Up,
                Fixed64.Zero,
                Vector3d.Zero);
        ContactAnchor valid = ContactAnchor.FromWorldPoint(Vector3d.Zero);
        Action createInvalidFirstContact = () =>
            _ = new ManifoldContact(
                1,
                default,
                valid,
                Fixed64.Zero,
                Vector3d.Up);
        Action createInvalidSecondContact = () =>
            _ = new ManifoldContact(
                1,
                valid,
                default,
                Fixed64.Zero,
                Vector3d.Up);

        wrapInvalidPoint.Should().Throw<ArgumentException>();
        createInvalidHit.Should().Throw<ArgumentException>();
        createInvalidFirstContact.Should().Throw<ArgumentException>();
        createInvalidSecondContact.Should().Throw<ArgumentException>();
    }

    [Fact]
    public void ContactAnchor_ShouldRebaseRigidFeatureWithoutDependingOnWorldPose()
    {
        FixedQuaternion firstOwnerRotation =
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        FixedQuaternion secondOwnerRotation =
            FixedQuaternion.FromAxisAngle(Vector3d.Up, -Fixed64.HalfPi);
        Vector3d ownerLocalPartOrigin = new(2, 3, 4);
        Vector3d partLocalFeature = new(1, 0, -1);
        ContactAnchor first = CreatePartAnchor(
            new Vector3d(100, 20, -30),
            firstOwnerRotation,
            ownerLocalPartOrigin,
            partLocalFeature);
        ContactAnchor second = CreatePartAnchor(
            new Vector3d(-80, -20, 50),
            secondOwnerRotation,
            ownerLocalPartOrigin,
            partLocalFeature);

        first.TryRebase(
            new Vector3d(100, 20, -30),
            firstOwnerRotation,
            out ContactAnchor firstRebased).Should().BeTrue();
        second.TryRebase(
            new Vector3d(-80, -20, 50),
            secondOwnerRotation,
            out ContactAnchor secondRebased).Should().BeTrue();
        firstRebased.LocalPoint.Should().Be(secondRebased.LocalPoint);
        firstRebased.LocalDisplacement.Should().Be(Vector3d.Zero);
        secondRebased.LocalDisplacement.Should().Be(Vector3d.Zero);

        first.TryRebase(
            new Vector3d(100, 20, -30),
            out ContactAnchor worldFrameRebased).Should().BeTrue();
        first.TryGetWorldPoint(out Vector3d firstPoint).Should().BeTrue();
        worldFrameRebased.TryGetWorldPoint(out Vector3d rebasedPoint).Should().BeTrue();
        rebasedPoint.Should().Be(firstPoint);
    }

    [Fact]
    public void ContactAnchor_ShouldMaterializeOnlyWhenTheFinalWorldPointIsRepresentable()
    {
        var representable = new ContactAnchor(
            new Vector3d(Fixed64.MaxValue, Fixed64.One, -Fixed64.One),
            new Vector3d(-Fixed64.One, Fixed64.Two, Fixed64.One));
        var outside = new ContactAnchor(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right);

        representable.TryGetWorldPoint(out Vector3d point).Should().BeTrue();
        point.Should().Be(new Vector3d(
            Fixed64.MaxValue - Fixed64.One,
            (Fixed64)3,
            Fixed64.Zero));
        outside.TryGetWorldPoint(out Vector3d unavailable).Should().BeFalse();
        unavailable.Should().Be(Vector3d.Zero);

        outside.TryRebase(
            Vector3d.Zero,
            out ContactAnchor unavailableRebase).Should().BeFalse();
        unavailableRebase.Should().Be(default(ContactAnchor));
        outside.TryRebase(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            out unavailableRebase).Should().BeFalse();
        unavailableRebase.Should().Be(default(ContactAnchor));

        var manifold3D = new ManifoldContact(
            1,
            outside,
            representable,
            Fixed64.Zero,
            Vector3d.Right);
        manifold3D.TryGetPointA(out _).Should().BeFalse();
        Action readPoint3D = () => _ = manifold3D.PointA;
        readPoint3D.Should().Throw<InvalidOperationException>();

        var outside2D = new ContactAnchor2D(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            Vector2d.Right);
        var manifold2D = new ManifoldContact2D(
            1,
            outside2D,
            ContactAnchor2D.FromWorldPoint(Vector2d.Zero),
            Fixed64.Zero,
            Vector2d.Right);
        manifold2D.TryGetPointA(out _).Should().BeFalse();
        Action readPoint2D = () => _ = manifold2D.PointA;
        readPoint2D.Should().Throw<InvalidOperationException>();

        var detectionContact = new Contact2D(
            outside2D,
            ContactAnchor2D.FromWorldPoint(Vector2d.Zero),
            Vector2d.Right,
            Fixed64.Zero);
        Action readDetectionPoint = () => _ = detectionContact.PointA;
        readDetectionPoint.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void ContactAnchor_ShouldRebaseWithOneFinalExactAddSubtract()
    {
        var anchor = new ContactAnchor(
            new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.Zero),
            new Vector3d(Fixed64.MinValue, Fixed64.MaxValue, Fixed64.One));
        var reference = new Vector3d(
            Fixed64.Zero,
            Fixed64.FromRaw(-1L),
            Fixed64.One);

        anchor.TryGetOffsetFrom(reference, out Vector3d offset).Should().BeTrue();
        offset.Should().Be(new Vector3d(
            Fixed64.FromRaw(-1L),
            Fixed64.Zero,
            Fixed64.Zero));
    }

    [Fact]
    public void ContactAnchor_ShouldRetainAnUnrepresentableRotatedOffset()
    {
        var anchor = new ContactAnchor(
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero),
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver4),
            new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.Zero));
        var samePoint = new ContactAnchor(
            anchor.Origin,
            anchor.Rotation,
            anchor.LocalPoint);

        anchor.TryGetWorldPoint(out _).Should().BeFalse();
        anchor.TryGetOffsetFrom(samePoint, out Vector3d offset).Should().BeTrue();
        offset.Should().Be(Vector3d.Zero);
        anchor.TryGetOffset(out Vector3d unavailable).Should().BeFalse();
        unavailable.Should().Be(Vector3d.Zero);
        Action readOffset = () => _ = anchor.Offset;
        readOffset.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void ContactAnchor_ShouldDeferLocalDisplacementAdditionUntilWorldEvaluation()
    {
        var anchor = new ContactAnchor(
            -Vector3d.Right,
            FixedQuaternion.Identity,
            Vector3d.Right * Fixed64.MaxValue,
            Vector3d.Right);

        anchor.LocalDisplacement.Should().Be(Vector3d.Right);
        anchor.TryGetWorldPoint(out Vector3d point).Should().BeTrue();
        point.Should().Be(Vector3d.Right * Fixed64.MaxValue);
        anchor.TryGetOffset(out Vector3d offset).Should().BeFalse();
        offset.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void ContactAnchor2D_ShouldMaterializeAndRebaseAtomically()
    {
        var rotated = new ContactAnchor2D(
            Vector2d.One,
            Fixed64.PiOver4,
            Vector2d.Right);
        rotated.Origin.Should().Be(Vector2d.One);
        rotated.Rotation.Should().Be(Fixed64.PiOver4);
        rotated.LocalPoint.Should().Be(Vector2d.Right);
        rotated.TryGetOffset(out Vector2d rotatedOffset).Should().BeTrue();
        rotated.Offset.Should().Be(rotatedOffset);
        rotated.TryRebase(Vector2d.Zero, out ContactAnchor2D rebased)
            .Should().BeTrue();
        rebased.Origin.Should().Be(Vector2d.Zero);
        rebased.Rotation.Should().Be(Fixed64.Zero);
        rebased.LocalPoint.Should().Be(Vector2d.One + rotatedOffset);
        rotated.TryRebase(
            Vector2d.Zero,
            Fixed64.Zero,
            out ContactAnchor2D rotatedRebased).Should().BeTrue();
        rotatedRebased.TryGetWorldPoint(out Vector2d rotatedPoint).Should().BeTrue();
        rotated.TryGetWorldPoint(out Vector2d originalPoint).Should().BeTrue();
        rotatedPoint.Should().Be(originalPoint);
        ContactAnchor2D.FromWorldPoint(Vector2d.One)
            .TryGetWorldPoint(out Vector2d materialized).Should().BeTrue();
        materialized.Should().Be(Vector2d.One);

        var anchor = new ContactAnchor2D(
            new Vector2d(Fixed64.MinValue, Fixed64.MaxValue),
            new Vector2d(Fixed64.MaxValue, Fixed64.MinValue));

        anchor.TryGetWorldPoint(out Vector2d worldPoint).Should().BeTrue();
        worldPoint.Should().Be(new Vector2d(
            Fixed64.FromRaw(-1L),
            Fixed64.FromRaw(-1L)));
        anchor.TryGetOffsetFrom(
            new Vector2d(Fixed64.Zero, Fixed64.MinValue),
            out Vector2d offset).Should().BeTrue();
        offset.Should().Be(new Vector2d(
            Fixed64.FromRaw(-1L),
            Fixed64.MaxValue));

        var unrepresentable = new ContactAnchor2D(
            new Vector2d(Fixed64.MaxValue, Fixed64.One),
            Vector2d.Zero);
        unrepresentable.TryGetOffsetFrom(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            out offset).Should().BeFalse();
        offset.Should().Be(Vector2d.Zero);
        unrepresentable.TryRebase(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            out ContactAnchor2D unavailableRebase).Should().BeFalse();
        unavailableRebase.Should().Be(default(ContactAnchor2D));

        var unrepresentableRotation = new ContactAnchor2D(
            Vector2d.Zero,
            Fixed64.PiOver4,
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue));
        unrepresentableRotation.TryGetOffset(out offset).Should().BeFalse();
        Action readOffset = () => _ = unrepresentableRotation.Offset;
        readOffset.Should().Throw<InvalidOperationException>();
    }

    private static ContactAnchor CreatePartAnchor(
        Vector3d ownerOrigin,
        FixedQuaternion ownerRotation,
        Vector3d ownerLocalPartOrigin,
        Vector3d partLocalFeature)
    {
        Vector3d worldPartOrigin =
            ownerOrigin + ownerRotation * ownerLocalPartOrigin;
        return new ContactAnchor(
            worldPartOrigin,
            ownerRotation,
            partLocalFeature);
    }

    [Fact]
    public void ContactAnchor2D_ShouldRetainDisplacementAndRejectInexactRebase()
    {
        var fullDomain = new ContactAnchor2D(
            -Vector2d.Right,
            Fixed64.Zero,
            Vector2d.Right * Fixed64.MaxValue,
            Vector2d.Right);
        fullDomain.LocalDisplacement.Should().Be(Vector2d.Right);
        fullDomain.TryGetWorldPoint(out Vector2d point).Should().BeTrue();
        point.Should().Be(Vector2d.Right * Fixed64.MaxValue);
        fullDomain.TryGetOffset(out Vector2d offset).Should().BeFalse();
        offset.Should().Be(Vector2d.Zero);

        var rotated = new ContactAnchor2D(
            new Vector2d((Fixed64)10, Fixed64.Zero),
            Fixed64.PiOver4,
            new Vector2d(Fixed64.Two, Fixed64.One),
            new Vector2d(Fixed64.Half, -Fixed64.Half));
        rotated.TryRebase(
            Vector2d.One,
            -Fixed64.PiOver4,
            out ContactAnchor2D rebased).Should().BeFalse();
        rebased.Should().Be(default(ContactAnchor2D));
    }
}
