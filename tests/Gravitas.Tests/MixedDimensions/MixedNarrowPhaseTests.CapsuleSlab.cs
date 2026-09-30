using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedNarrowPhaseTests
{
    [Fact]
    public void CapsuleCapsuleSlab_WithSeparatedRim_ShouldRejectOverlappingBounds()
    {
        using GravitasWorldContext context = CreateMixedContext();
        Fixed64 q = Fixed64.FromRaw(3_037_000_500L);
        ScenarioBody<LSCapsuleCollider> capsule = CreateBody3D(context,
            new LSCapsuleCollider { Radius = Fixed64.One, Size = new Vector3d(2, 22, 2) },
            new Vector3d(Fixed64.FromFraction(83, 4), Fixed64.FromFraction(7, 4), Fixed64.Zero),
            new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, -q, q));
        var slab = new LSCapsuleCollider2D((Fixed64)10, (Fixed64)22)
        {
            MixedHalfThicknessOverride = Fixed64.One
        };
        CreateBody2D(context, slab, Vector2d.Zero);

        // The Z core cannot shorten the X/Y gap from (43/4,7/4,0)
        // to the cap rim (10,1,0): its squared length is 9/8 > radius^2.
        capsule.Collider.Bounds.Intersects(slab.MixedBounds3D).Should().BeTrue();
        CollisionDetectionMixed.TryCollide(capsule.Collider, slab, out MixedContact contact).Should().BeFalse();
        contact.Should().Be(default(MixedContact));
    }

    [Theory]
    [InlineData(-1L)]
    [InlineData(0L)]
    [InlineData(1L)]
    public void CapsuleCapsuleSlab_AtStraightRim_ShouldClassifyBeforeRounding(long radiusOffset)
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
        var slab = new LSCapsuleCollider2D((Fixed64)10, (Fixed64)22)
        {
            MixedHalfThicknessOverride = Fixed64.One
        };
        CreateBody2D(context, slab, Vector2d.Zero);

        // The capsule core ends at (43/4,2,0), exactly 5/4 from (10,1,0).
        // Radius offsets change only signed penetration, not either core.
        bool hit = CollisionDetectionMixed.TryCollide(capsule.Collider, slab, out MixedContact contact);
        hit.Should().Be(radiusOffset >= 0);
        if (!hit)
        {
            contact.Should().Be(default(MixedContact));
            return;
        }
        contact.Depth.Should().Be(Fixed64.FromRaw(radiusOffset));
        contact.Normal3DTo2D.Should().Be(new Vector3d(Fixed64.FromFraction(-3, 5), Fixed64.FromFraction(-4, 5), Fixed64.Zero));
        contact.Anchor3D.Origin.Should().Be(capsule.Collider.Center);
        contact.Anchor3D.Rotation.Should().Be(capsule.Collider.Rotation);
        contact.Anchor2D.Origin.Should().Be(Vector3d.Zero);
        contact.DepthIsClamped.Should().BeFalse();
    }

    [Fact]
    public void CapsuleCapsuleSlab_ObliqueInterior_ShouldRemainDeterministicAndAllocationFree()
    {
        using GravitasWorldContext context = CreateMixedContext();
        const long scale = 858_993_459L;
        FixedQuaternion rotation = new(Fixed64.FromRaw(scale), Fixed64.FromRaw(2 * scale),
            Fixed64.FromRaw(-4 * scale), Fixed64.FromRaw(2 * scale));
        Fixed64 radius = Fixed64.FromFraction(21, 4);
        ScenarioBody<LSCapsuleCollider> capsule = CreateBody3D(context,
            new LSCapsuleCollider { Radius = radius, Size = new Vector3d(radius * 2, Fixed64.Two + radius * 2, radius * 2) },
            new Vector3d(0, 5, -5), rotation);
        var slab = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4) { MixedHalfThicknessOverride = Fixed64.One };
        CreateBody2D(context, slab, Vector2d.Zero);
        CollisionDetectionMixed.TryCollide(capsule.Collider, slab, out MixedContact expected).Should().BeTrue();
        expected.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        expected.Normal3DTo2D.Should().Be(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(-4, 5), Fixed64.FromFraction(3, 5)));
        expected.Anchor3D.Rotation.Should().Be(rotation);
        expected.Anchor3D.Origin.Should().Be(capsule.Collider.Center);
        expected.Anchor2D.Origin.Should().Be(Vector3d.Zero);
        expected.DepthIsClamped.Should().BeFalse();
        bool stable = true;
        long bytes = AllocationTestHelper.MeasureSteadyState(() =>
        {
            stable &= CollisionDetectionMixed.TryCollide(capsule.Collider, slab, out MixedContact actual);
            stable &= actual.Depth == expected.Depth && actual.Normal3DTo2D == expected.Normal3DTo2D
                && actual.Anchor3D.Origin == expected.Anchor3D.Origin && actual.Anchor3D.Rotation == expected.Anchor3D.Rotation
                && actual.Anchor3D.LocalPoint == expected.Anchor3D.LocalPoint
                && actual.Anchor3D.LocalDisplacement == expected.Anchor3D.LocalDisplacement
                && actual.Anchor2D.Origin == expected.Anchor2D.Origin && actual.Anchor2D.Rotation == expected.Anchor2D.Rotation
                && actual.Anchor2D.LocalPoint == expected.Anchor2D.LocalPoint
                && actual.Anchor2D.LocalDisplacement == expected.Anchor2D.LocalDisplacement;
        });
        stable.Should().BeTrue(); bytes.Should().Be(0);
    }

    [Fact]
    public void CapsuleCapsuleSlab_RuntimePair_ShouldResolveCapContactAndPreservePlanarFrame()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Environment.Gravity = Fixed64.Zero;
        Vector3d start = new(Fixed64.Zero, Fixed64.FromFraction(11, 4), Fixed64.Zero);
        ScenarioBody<LSCapsuleCollider> capsule = CreateBody3D(context,
            new LSCapsuleCollider { Radius = Fixed64.One, Size = new Vector3d(2, 4, 2) },
            start, FixedQuaternion.Identity);
        Fixed64 yaw = Fixed64.FromFraction(7, 13);
        var slab = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4) { MixedHalfThicknessOverride = Fixed64.One };
        SolidBody2D platform = CreateBody2D(context, slab, Vector2d.Zero, yaw);
        CollisionDetectionMixed.TryCollide(capsule.Collider, slab, out MixedContact contact).Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Normal3DTo2D.Should().Be(-Vector3d.Up);
        contact.Anchor3D.Origin.Should().Be(start);
        contact.Anchor2D.Origin.Should().Be(Vector3d.Zero);
        contact.Anchor2D.Rotation.Should().Be(FixedQuaternion.FromAxisAngle(Vector3d.Up, -yaw));
        int entered3D = 0, entered2D = 0;
        capsule.Collider.OnMixedContactEnter += other => { other.Should().BeSameAs(slab); entered3D++; };
        slab.OnMixedContactEnter += other => { other.Should().BeSameAs(capsule.Collider); entered2D++; };

        context.Simulate();
        context.LateSimulate();

        entered3D.Should().Be(1);
        entered2D.Should().Be(1);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
        capsule.Body.Position3d.Y.Should().BeGreaterThan(start.Y);
        capsule.Body.Position3d.X.Should().Be(Fixed64.Zero);
        capsule.Body.Position3d.Z.Should().Be(Fixed64.Zero);
        platform.Position.Should().Be(Vector2d.Zero);
        platform.Rotation.Should().Be(yaw);
    }

    [Fact]
    public void CompoundCapsuleAgainstCapsuleSlab_ShouldPreserveChildContactAndMaterials()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var compound3D = new LSCompoundCollider(CompoundColliderPart.Capsule(
            Fixed64.One, (Fixed64)4, Vector3d.Zero, PhysicsMaterial.Frictionless));
        var compound2D = new LSCompoundCollider2D(CompoundColliderPart2D.Capsule(
            Fixed64.One, (Fixed64)4, Vector2d.Zero, PhysicsMaterial.Bouncy)) { MixedHalfThicknessOverride = Fixed64.One };
        ScenarioBody<LSCompoundCollider> capsule = CreateBody3D(context, compound3D,
            new Vector3d(Fixed64.FromFraction(7, 4), Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity);
        CreateBody2D(context, compound2D, Vector2d.Zero);
        CollisionDetectionMixed.TryCollide(capsule.Collider, compound2D, out MixedContact contact).Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.HasMaterialOverride.Should().BeTrue();
        contact.Material3D.Should().Be(PhysicsMaterial.Frictionless);
        contact.Material2D.Should().Be(PhysicsMaterial.Bouncy);
    }
}
