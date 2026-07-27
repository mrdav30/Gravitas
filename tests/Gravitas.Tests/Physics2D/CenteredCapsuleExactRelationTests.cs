//=======================================================================
// CenteredCapsuleExactRelationTests.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class CenteredCapsuleExactRelationTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleConvexSweep_AtScalarFace_DoesNotRequireRepresentableAxisEndpoints(
        bool maximumFace)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        Fixed64 face = maximumFace ? Fixed64.MaxValue : Fixed64.MinValue;
        Fixed64 inward = maximumFace ? -Fixed64.One : Fixed64.One;
        Fixed64 rotation = maximumFace ? Fixed64.HalfPi : -Fixed64.HalfPi;
        SolidBody2D mover = CreateBody(
            context,
            new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3),
            new Vector2d(face, Fixed64.Zero),
            rotation);
        SolidBody2D target = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d(Fixed64.One, (Fixed64)6)),
            new Vector2d(face + inward * (Fixed64)4, Fixed64.Zero));

        bool found = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            inward * (Fixed64)3 * Vector2d.Right,
            target.Collider,
            out Physics2DHit hit);

        found.Should().BeTrue();
        hit.Distance.Should().Be((Fixed64)2);
        hit.Normal.Should().Be(-inward * Vector2d.Right);
    }

    [Fact]
    public void CapsuleCapsuleSweep_PreservesOddRawAxisLengths()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        Fixed64 radius = Fixed64.FromRaw(257L);
        Fixed64 height = Fixed64.FromRaw(771L);
        SolidBody2D mover = CreateBody(
            context,
            new LSCapsuleCollider2D(radius, height),
            new Vector2d(Fixed64.FromRaw(-3000L), Fixed64.Zero));
        SolidBody2D target = CreateBody(
            context,
            new LSCapsuleCollider2D(radius, height),
            Vector2d.Zero);

        bool found = QueryDetection2D.TrySweepMoverShape(
            mover.Collider,
            new Vector2d(Fixed64.FromRaw(4000L), Fixed64.Zero),
            target.Collider,
            out Physics2DHit hit);

        found.Should().BeTrue();
        hit.Distance.Should().Be(Fixed64.FromRaw(2486L));
        hit.Normal.Should().Be(Vector2d.Left);
    }

    [Fact]
    public void CapsuleConvexCollision_AtScalarFace_PreservesGeometricOverlap()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        Fixed64 face = Fixed64.MaxValue;
        SolidBody2D capsule = CreateBody(
            context,
            new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3),
            new Vector2d(face, Fixed64.Zero),
            Fixed64.HalfPi);
        SolidBody2D box = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d(Fixed64.One, (Fixed64)4)),
            new Vector2d(face - Fixed64.FromFraction(5, 4), Fixed64.Zero));

        bool collided = CollisionDetection2D.TryCollide(
            capsule.Collider,
            box.Collider,
            out Contact2D contact);

        collided.Should().BeTrue();
        contact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        contact.Normal.Should().Be(Vector2d.Left);
    }

    [Fact]
    public void CapsuleSupport_WithSmallRepresentableDirection_ShouldHonorItsSign()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D capsule = CreateBody(
            context,
            new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3),
            Vector2d.Zero,
            Fixed64.PiOver4);

        Vector2d tinyDirection = capsule.Collider.GetSupportPoint(
            new Vector2d(Fixed64.FromRaw(1L), Fixed64.Zero));
        Vector2d unitDirection = capsule.Collider.GetSupportPoint(Vector2d.Right);

        tinyDirection.Should().Be(unitDirection);
    }

    [Fact]
    public void ExactCapsuleRelations_DoNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D capsule = CreateBody(
            context,
            new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3),
            new Vector2d((Fixed64)(-3), Fixed64.FromFraction(1, 4)));
        SolidBody2D box = CreateBody(
            context,
            new LSAABBoxCollider2D(Vector2d.One),
            Vector2d.Zero);
        Vector2d displacement = new((Fixed64)4, Fixed64.Zero);

        Action query = () =>
        {
            _ = QueryDetection2D.TrySweepMoverShape(
                capsule.Collider,
                displacement,
                box.Collider,
                out _);
            _ = CollisionDetection2D.TryCollide(capsule.Collider, box.Collider, out _);
        };

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(query);

        allocatedBytes.Should().Be(0);
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        Fixed64 rotation = default)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var body = new SolidBody2D(new TestMatterAgent(context, transform), collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, rotation, BodyMotionType.Static);
        return body;
    }
}
