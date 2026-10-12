//=======================================================================
// MixedNarrowPhaseTests.MeshAnchors.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedNarrowPhaseTests
{
    [Fact]
    public void MeshCircleSlab_FaceInteriorContact_ShouldUseIncidentInteriorWitness()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSMeshCollider> mesh = CreateMesh3D(
            context,
            CreateTriangleMesh(
                new Vector3d((Fixed64)(-4), Fixed64.Zero, (Fixed64)(-4)),
                new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)(-4)),
                new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)4)),
            Vector3d.Zero);
        Vector2d circleCenter = new(
            Fixed64.FromFraction(1, 4),
            Fixed64.FromFraction(1, 8));
        SolidBody2D circle = CreateBody2D(
            context,
            new LSCircleCollider2D(Fixed64.Half),
            circleCenter);

        CollisionDetectionMixed.TryCollide(
                mesh.Collider,
                circle.Collider,
                out MixedContact contact)
            .Should()
            .BeTrue();

        contact.Anchor3D.TryGetWorldPoint(out Vector3d meshPoint)
            .Should()
            .BeTrue();
        meshPoint.Y.Should().Be(Fixed64.Zero);
        // A finite face family may select a cardinal disk sample rather than
        // its center. Verify its actual footprint and matched ray gap instead.
        Fixed64 dx = meshPoint.X - circleCenter.X, dz = meshPoint.Z - circleCenter.Y;
        (dx * dx + dz * dz).Should().BeLessThanOrEqualTo(Fixed64.Quarter + Fixed64.FromRaw(4));
        meshPoint.X.Should().BeInRange((Fixed64)(-4), (Fixed64)4);
        meshPoint.Z.Should().BeInRange((Fixed64)(-4), (Fixed64)4);
        contact.Depth.Should().Be(Fixed64.Half);
        contact.Normal3DTo2D.Should().Be(Vector3d.Down);
        (contact.Point2D - meshPoint).Should().Be(Vector3d.Up * Fixed64.Half);
        meshPoint.Should().NotBe(new Vector3d(
            (Fixed64)(-4),
            Fixed64.Zero,
            (Fixed64)(-4)));
    }
}
