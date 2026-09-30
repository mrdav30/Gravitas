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
    public void CapsuleCapsuleSlab_WithPositiveCore_ShouldRetainItsEndpointExtent()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCapsuleCollider> capsule = CreateBody3D(context,
            new LSCapsuleCollider { Radius = Fixed64.One, Size = new Vector3d(2, 4, 2) },
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(7, 4)),
            FixedQuaternion.Identity);
        var slab = new LSCapsuleCollider2D(Fixed64.One, Fixed64.Two + Fixed64.FromRaw(2))
        {
            MixedHalfThicknessOverride = Fixed64.One
        };
        CreateBody2D(context, slab, Vector2d.Zero);

        // Only an exactly zero core is a circle. Even this two-raw-unit core
        // extends the stadium endpoint by one raw unit beyond the circle.
        CollisionDetectionMixed.TryCollide(capsule.Collider, slab, out MixedContact contact).Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4) + Fixed64.FromRaw(1));
        contact.Normal3DTo2D.Should().Be(-Vector3d.Forward);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleCircleSlab_WithSeparatedEndpointRim_ShouldRejectOverlappingBounds(bool zeroCoreSlab)
    {
        using GravitasWorldContext context = CreateMixedContext();
        Fixed64 q = Fixed64.FromRaw(3_037_000_500L);
        ScenarioBody<LSCapsuleCollider> capsule = CreateBody3D(context,
            new LSCapsuleCollider { Radius = Fixed64.One, Size = new Vector3d(2, 22, 2) },
            new Vector3d(Fixed64.FromFraction(83, 4), Fixed64.FromFraction(7, 4), Fixed64.Zero),
            new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, -q, q));
        LSCollider2D slab = zeroCoreSlab
            ? new LSCapsuleCollider2D((Fixed64)10, (Fixed64)20)
            : new LSCircleCollider2D((Fixed64)10);
        slab.MixedHalfThicknessOverride = Fixed64.One;
        CreateBody2D(context, slab, Vector2d.Zero);

        // Closest core endpoint is (43/4,7/4,0). Its cap-rim gap
        // squared is (3/4)^2 + (3/4)^2 = 9/8, greater than radius^2.
        capsule.Collider.Bounds.Intersects(slab.MixedBounds3D).Should().BeTrue();
        CollisionDetectionMixed.TryCollide(capsule.Collider, slab, out MixedContact contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleCircleSlab_WithUnrepresentableFullHeight_ShouldPreserveAdmittedHalfThickness(bool zeroCoreSlab)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCapsuleCollider> capsule = CreateBody3D(context,
            new LSCapsuleCollider { Radius = Fixed64.One, Size = new Vector3d(2, 4, 2) },
            Vector3d.Zero, FixedQuaternion.Identity);
        LSCollider2D slab = zeroCoreSlab
            ? new LSCapsuleCollider2D(Fixed64.One, Fixed64.Two)
            : new LSCircleCollider2D(Fixed64.One);
        slab.MixedHalfThicknessOverride = Fixed64.MaxValue;
        CreateBody2D(context, slab, Vector2d.Zero);
        capsule.Collider.LocalOffset = new Vector3d(Fixed64.Zero, Fixed64.MaxValue - Fixed64.One, Fixed64.Zero);
        capsule.Collider.RebuildRuntimeShapeOnly().Should().BeTrue();

        CollisionDetectionMixed.TryCollide(capsule.Collider, slab, out MixedContact contact).Should().BeTrue();
        // Radial escape is 1 + 1; the upper cap requires a translation of 3.
        contact.Depth.Should().Be(Fixed64.Two);
        contact.Normal3DTo2D.Y.Should().Be(Fixed64.Zero);
        contact.DepthIsClamped.Should().BeFalse();
        contact.Anchor3D.Origin.Should().Be(capsule.Collider.Center);
        contact.Anchor2D.Origin.Should().Be(Vector3d.Zero);
    }

    [Theory]
    [InlineData(false, -1L)]
    [InlineData(false, 0L)]
    [InlineData(false, 1L)]
    [InlineData(true, -1L)]
    [InlineData(true, 0L)]
    [InlineData(true, 1L)]
    public void CapsuleCircleSlab_AtEndpointRim_ShouldClassifyBeforeRounding(bool zeroCoreSlab, long radiusOffset)
    {
        using GravitasWorldContext context = CreateMixedContext();
        Fixed64 radius = Fixed64.FromFraction(5, 4) + Fixed64.FromRaw(radiusOffset);
        Fixed64 q = Fixed64.FromRaw(3_037_000_500L);
        ScenarioBody<LSCapsuleCollider> capsule = CreateBody3D(context,
            new LSCapsuleCollider
            {
                Radius = radius,
                Size = new Vector3d(radius * Fixed64.Two, (Fixed64)20 + radius * Fixed64.Two, radius * Fixed64.Two)
            },
            new Vector3d(Fixed64.FromFraction(83, 4), Fixed64.Two, Fixed64.Zero),
            new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, -q, q));
        LSCollider2D slab = zeroCoreSlab
            ? new LSCapsuleCollider2D((Fixed64)10, (Fixed64)20)
            : new LSCircleCollider2D((Fixed64)10);
        slab.MixedHalfThicknessOverride = Fixed64.One;
        CreateBody2D(context, slab, Vector2d.Zero);

        // The endpoint-to-rim offset (3/4,1,0) has exact length 5/4.
        bool hit = CollisionDetectionMixed.TryCollide(capsule.Collider, slab, out MixedContact contact);
        hit.Should().Be(radiusOffset >= 0);
        if (!hit)
            return;
        contact.Depth.Should().Be(Fixed64.FromRaw(radiusOffset));
        contact.Normal3DTo2D.Should().Be(new Vector3d(Fixed64.FromFraction(-3, 5), Fixed64.FromFraction(-4, 5), Fixed64.Zero));
        contact.Anchor3D.Origin.Should().Be(capsule.Collider.Center);
        contact.Anchor3D.Rotation.Should().Be(capsule.Collider.Rotation);
        contact.Anchor2D.Origin.Should().Be(Vector3d.Zero);
        contact.Point2D.Should().Be(new Vector3d(10, 1, 0));
        contact.DepthIsClamped.Should().BeFalse();
        contact.HasMaterialOverride.Should().BeFalse();
    }

    [Theory]
    [InlineData(-1L)]
    [InlineData(0L)]
    [InlineData(1L)]
    public void CapsuleCircleSlab_AtObliqueInteriorRim_ShouldKeepExactRigidFrame(long radiusOffset)
    {
        using GravitasWorldContext context = CreateMixedContext();
        const long scale = 607_400_100L;
        FixedQuaternion rotation = new(Fixed64.FromRaw(5 * scale), Fixed64.Zero,
            Fixed64.FromRaw(-3 * scale), Fixed64.FromRaw(4 * scale));
        Fixed64 radius = (Fixed64)5 + Fixed64.FromRaw(radiusOffset);
        ScenarioBody<LSCapsuleCollider> capsule = CreateBody3D(context,
            new LSCapsuleCollider
            {
                Radius = radius,
                Size = new Vector3d(radius * Fixed64.Two, Fixed64.Two + radius * Fixed64.Two, radius * Fixed64.Two)
            },
            new Vector3d(4, 5, 0), rotation);
        SolidBody2D circle = CreateBody2D(context,
            new LSCircleCollider2D(Fixed64.One) { MixedHalfThicknessOverride = Fixed64.One }, Vector2d.Zero);

        // Exact axis (12,-9,20)/25 is perpendicular to the radius-five
        // residual (3,4,0); the nearest capsule-core feature is interior.
        bool hit = CollisionDetectionMixed.TryCollide(capsule.Collider, circle.Collider, out MixedContact contact);
        hit.Should().Be(radiusOffset >= 0);
        if (!hit)
            return;
        contact.Depth.Should().Be(Fixed64.FromRaw(radiusOffset));
        contact.Normal3DTo2D.Should().Be(new Vector3d(Fixed64.FromFraction(-3, 5), Fixed64.FromFraction(-4, 5), Fixed64.Zero));
        contact.Point2D.Should().Be(new Vector3d(1, 1, 0));
        contact.Anchor3D.Rotation.Should().Be(capsule.Collider.Rotation);
    }

    [Fact]
    public void CapsuleCircleSlab_WithUnrepresentableDepth_ShouldPreserveClampFlag()
    {
        using GravitasWorldContext context = CreateMixedContext();
        Fixed64 radius = (Fixed64)1_000_000_000;
        ScenarioBody<LSCapsuleCollider> capsule = CreateBody3D(context,
            new LSCapsuleCollider { Radius = radius, Size = new Vector3d(2_000_000_000, 2_000_000_000, 2_000_000_000) },
            Vector3d.Zero, FixedQuaternion.Identity);
        SolidBody2D circle = CreateBody2D(context,
            new LSCircleCollider2D((Fixed64)1_500_000_000) { MixedHalfThicknessOverride = Fixed64.MaxValue }, Vector2d.Zero);

        CollisionDetectionMixed.TryCollide(capsule.Collider, circle.Collider, out MixedContact contact).Should().BeTrue();
        // Both radial (2.5 billion) and axial escape exceed the scalar domain.
        contact.Depth.Should().Be(Fixed64.MaxValue);
        contact.DepthIsClamped.Should().BeTrue();
        contact.Anchor3D.Origin.Should().Be(Vector3d.Zero);
        contact.Anchor2D.Origin.Should().Be(Vector3d.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleCircleSlab_AfterWarmup_ShouldNotAllocate(bool zeroCoreSlab)
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSCapsuleCollider> capsule = CreateCapsule3D(context,
            new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        LSCollider2D slab = zeroCoreSlab
            ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.One)
            : new LSCircleCollider2D(Fixed64.Half);
        CreateBody2D(context, slab, Vector2d.Zero);
        bool allContacts = true;
        long bytes = AllocationTestHelper.MeasureSteadyState(() =>
            allContacts &= CollisionDetectionMixed.TryCollide(capsule.Collider, slab, out _));
        allContacts.Should().BeTrue();
        bytes.Should().Be(0);
    }
}
