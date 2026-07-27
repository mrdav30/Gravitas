//=======================================================================
// ContactSelectionPolicyTests.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContactSelectionPolicyTests
{
    [Fact]
    public void ShouldReplaceWithDeeper_ShouldAcceptFirstAndDeeper2DContact()
    {
        var current = new Contact2D(Vector2d.Zero, Vector2d.Zero, Vector2d.Right, (Fixed64)2);
        var shallower = new Contact2D(Vector2d.Zero, Vector2d.Zero, Vector2d.Right, Fixed64.One);
        var deeper = new Contact2D(Vector2d.Zero, Vector2d.Zero, Vector2d.Right, (Fixed64)3);
        var clamped = new Contact2D(
            Vector2d.Zero,
            Vector2d.Zero,
            Vector2d.Right,
            (Fixed64)2,
            depthIsClamped: true);

        ContactSelectionPolicy.ShouldReplaceWithDeeper(shallower, found: false, current).Should().BeTrue();
        ContactSelectionPolicy.ShouldReplaceWithDeeper(shallower, found: true, current).Should().BeFalse();
        ContactSelectionPolicy.ShouldReplaceWithDeeper(current, found: true, current).Should().BeFalse();
        ContactSelectionPolicy.ShouldReplaceWithDeeper(deeper, found: true, current).Should().BeTrue();
        ContactSelectionPolicy.ShouldReplaceWithDeeper(clamped, found: true, current).Should().BeTrue();
        ContactSelectionPolicy.ShouldReplaceWithDeeper(current, found: true, clamped).Should().BeFalse();
    }

    [Fact]
    public void ShouldReplaceWithShallower_ShouldAcceptFirstAndShallowerMixedContact()
    {
        var current = new MixedContact(Vector3d.Zero, Vector3d.Zero, Vector3d.Right, (Fixed64)2);
        var deeper = new MixedContact(Vector3d.Zero, Vector3d.Zero, Vector3d.Right, (Fixed64)3);
        var shallower = new MixedContact(Vector3d.Zero, Vector3d.Zero, Vector3d.Right, Fixed64.One);
        var clamped = new MixedContact(
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)2,
            depthIsClamped: true);

        ContactSelectionPolicy.ShouldReplaceWithShallower(deeper, found: false, current).Should().BeTrue();
        ContactSelectionPolicy.ShouldReplaceWithShallower(deeper, found: true, current).Should().BeFalse();
        ContactSelectionPolicy.ShouldReplaceWithShallower(current, found: true, current).Should().BeFalse();
        ContactSelectionPolicy.ShouldReplaceWithShallower(shallower, found: true, current).Should().BeTrue();
        ContactSelectionPolicy.ShouldReplaceWithShallower(current, found: true, clamped).Should().BeTrue();
        ContactSelectionPolicy.ShouldReplaceWithShallower(clamped, found: true, current).Should().BeFalse();
    }
}
