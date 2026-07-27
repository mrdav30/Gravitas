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
        meshPoint.Should().Be(new Vector3d(
            circleCenter.X,
            Fixed64.Zero,
            circleCenter.Y));
        meshPoint.Should().NotBe(new Vector3d(
            (Fixed64)(-4),
            Fixed64.Zero,
            (Fixed64)(-4)));
    }
}
