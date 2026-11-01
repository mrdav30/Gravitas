using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class PlanarTransformSamplingTests
{
    [Theory]
    [InlineData(false, -1)]
    [InlineData(false, 0)]
    [InlineData(true, 1)]
    public void UnitPlanarBasis_ShouldPreserveTranslationAndIgnoreYScale(bool hierarchy, int yScale)
    {
        var parent = new FixedTransform(new(4, 7, -2), FixedQuaternion.Identity, new(2, yScale, 4));
        var transform = hierarchy
            ? new FixedTransform(new(1, 3, 2), FixedQuaternion.Identity, new(Fixed64.Half, Fixed64.One, Fixed64.Quarter), parent)
            : new FixedTransform(new(6, 7, -2), FixedQuaternion.Identity, new(1, yScale, 1));
        foreach (bool captureRotation in new[] { false, true })
        {
            Vector2d scale = ColliderScalePolicy.CapturePlanar(transform, out Fixed4x4 matrix, out Fixed64 yaw, captureRotation);
            scale.Should().Be(Vector2d.One);
            yaw.Should().Be(Fixed64.Zero);
            matrix.Translation.Should().Be(hierarchy ? new Vector3d(6, 10, 6) : new Vector3d(6, 7, -2));
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void NonidentityBasis_ShouldPreserveDecompositionRounding(int kind)
    {
        Fixed64 nearOne = Fixed64.One + Fixed64.MinIncrement;
        var transform = new FixedTransform(Vector3d.Zero,
            kind == 2 ? FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4) : FixedQuaternion.Identity,
            kind == 0 ? new Vector3d(nearOne, Fixed64.One, nearOne) : new Vector3d(2, 1, 3));
        transform.TryGetLocalToWorldMatrix(out Fixed4x4 matrix).Should().BeTrue();
        Fixed4x4.Decompose(matrix, out _, out FixedQuaternion rotation, out Vector3d expectedScale).Should().BeTrue();
        if (kind == 0) expectedScale.ToVector2d().Should().Be(Vector2d.One);
        Vector3d right = rotation.Rotate(Vector3d.Right);
        Fixed64 expectedYaw = PlanarRotation.Canonicalize(FixedMath.Atan2(right.Z, right.X));
        foreach (bool captureRotation in new[] { false, true })
        {
            ColliderScalePolicy.CapturePlanar(transform, out Fixed4x4 captured, out Fixed64 yaw, captureRotation)
                .Should().Be(expectedScale.ToVector2d());
            captured.Should().Be(matrix);
            yaw.Should().Be(captureRotation ? expectedYaw : Fixed64.Zero);
        }
    }

    [Fact]
    public void CanceledReflectionsAndNonplanarCoupling_ShouldStillReject()
    {
        var parent = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, new(-1, 1, -1));
        var child = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, new(-1, 1, -1), parent);
        child.TryGetLocalToWorldMatrix(out Fixed4x4 matrix).Should().BeTrue();
        matrix.Should().Be(Fixed4x4.Identity);
        Action reflected = () => ColliderScalePolicy.CapturePlanar(child, out _, out _);
        reflected.Should().Throw<ArgumentException>();
        var tilted = new FixedTransform(Vector3d.Zero,
            new FixedQuaternion(Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero, Fixed64.One), Vector3d.One);
        Action coupled = () => ColliderScalePolicy.CapturePlanar(tilted, out _, out _);
        coupled.Should().Throw<ArgumentException>();
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ServiceSampling_ShouldTrackHostHierarchyAndShapeChanges(bool mixed, bool explicitStatic)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = mixed ? PhysicsRuntimeMode.Mixed : PhysicsRuntimeMode.TwoD;
        context.World.TryAddGrid(new GridConfiguration(new(-32, -8, -32), new(32, 8, 32)), out _).Should().BeTrue();
        var parent = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One, parent);
        var agent = new TestMatterAgent(context, transform);
        var collider = new LSAABBoxCollider2D(new Vector2d(2, 4));
        if (explicitStatic) new SolidBody2D(agent, collider).Initialize(Vector2d.Zero, motionType: BodyMotionType.Static);
        else collider.InitializeWithNoBody(agent);
        Step();
        uint version = collider.RuntimeShapeVersion;
        Step();
        collider.RuntimeShapeVersion.Should().Be(version);
        parent.LocalScale = new(2, 1, 3);
        if (!explicitStatic) parent.LocalPosition = new(12, 2, 0);
        Step();
        collider.LocalScale.Should().Be(new Vector2d(2, 3));
        collider.RuntimeShapeVersion.Should().BeGreaterThan(version);
        collider.Center.Should().Be(explicitStatic ? Vector2d.Zero : new Vector2d(12, 0));
        var hits = new SwiftList<Physics2DHit>();
        context.Query2D.OverlapCircleAll(collider.Center, Fixed64.Quarter, hits).Should().Be(1);
        hits[0].Collider.Should().BeSameAs(collider);
        if (mixed)
        {
            Fixed64 y = collider.MixedSlabCenterY;
            context.QueryMixed.SweepSphereAgainst2D(new(collider.Center.X, y + 4, collider.Center.Y),
                new(collider.Center.X, y - 4, collider.Center.Y), Fixed64.Quarter,
                Gravitas.Support.PhysicsLayerMask.All, out PhysicsMixedHit hit).Should().BeTrue();
            hit.Collider2D.Should().BeSameAs(collider);
        }
        if (!explicitStatic)
        {
            parent.LocalRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Pi * Fixed64.Half);
            parent.LocalScale = Vector3d.One;
            Step();
            collider.Rotation.Should().NotBe(Fixed64.Zero);
        }
        parent.LocalRotation = FixedQuaternion.Identity;
        parent.LocalScale = Vector3d.One;
        collider.LocalOffset = Vector2d.Right;
        Step();
        collider.LocalScale.Should().Be(Vector2d.One);
        collider.Rotation.Should().Be(Fixed64.Zero);
        collider.Center.Should().Be(explicitStatic ? Vector2d.Right : new Vector2d(13, 0));
        void Step() { context.Simulate(); context.LateSimulate(); }
    }
}
