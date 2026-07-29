using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class CompoundCollider2DTests
{
    [Fact]
    public void Initialize_WithUnderflowedPartWorldScale_ShouldRejectBeforeBinding()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(CompoundColliderPart2D.Circle(
            Fixed64.Half,
            Vector2d.Zero,
            Fixed64.Zero,
            new Vector2d(Fixed64.MinIncrement, Fixed64.One)));
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.One));

        Action initialize = () => compound.InitializeWithNoBody(new TestMatterAgent(context, transform));

        initialize.Should().Throw<ArgumentException>().WithParameterName("dimension");
        compound.Id.Should().Be(-1);
        compound.HasHostBinding.Should().BeFalse();
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void BodyInitialize_WithUnderflowedPartWorldScale_ShouldRejectBeforeRuntimeMutation()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(CompoundColliderPart2D.Circle(
            Fixed64.Half,
            Vector2d.Zero,
            Fixed64.Zero,
            new Vector2d(Fixed64.MinIncrement, Fixed64.One)));
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.One));
        var body = new SolidBody2D(new TestMatterAgent(context, transform), compound);

        Action initialize = () => body.Initialize(Vector2d.Zero);

        initialize.Should().Throw<ArgumentException>().WithParameterName("dimension");
        body.Active.Should().BeFalse();
        body.DynamicId.Should().Be(-1);
        compound.Id.Should().Be(-1);
        compound.Body.Should().BeNull();
        compound.HasHostBinding.Should().BeFalse();
        context.Physics2D.BodyCount.Should().Be(0);
        context.Physics2D.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void Initialize_ShouldRegisterOnlyOwningColliderAndAggregatePartBounds()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(-Fixed64.One, Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)2, Fixed64.Zero)));

        SolidBody2D body = CreateBody(context, compound, Vector2d.Zero);

        body.Collider.Shape.Should().Be(ColliderType2D.Compound);
        compound.PartCount.Should().Be(2);
        compound.GetPartId(0).Should().Be(0);
        compound.GetPartId(1).Should().Be(1);
        context.Physics2D.ColliderCount.Should().Be(1);
        context.Physics2D.TryGetColliderById(compound.GetPartCollider(0).Id, out _).Should().BeFalse();
        context.Physics2D.TryGetColliderById(compound.GetPartCollider(1).Id, out _).Should().BeFalse();

        compound.Bounds.Min.X.Should().Be(-Fixed64.FromFraction(3, 2));
        compound.Bounds.Max.X.Should().Be(Fixed64.FromFraction(5, 2));
        compound.Bounds.Min.Y.Should().Be(-Fixed64.Half);
        compound.Bounds.Max.Y.Should().Be(Fixed64.Half);
        compound.Center.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void Constructor_ShouldRejectDefaultParts()
    {
        Action act = () => _ = new LSCompoundCollider2D(default(CompoundColliderPart2D));

        act.Should().Throw<ArgumentException>()
            .WithMessage("*default*");
    }

    [Fact]
    public void CompoundColliderPart2D_ShouldCaptureAuthoredShapeTransformScaleAndMaterial()
    {
        PhysicsMaterial material = PhysicsMaterial.Frictionless;
        Vector2d offset = new(Fixed64.One, (Fixed64)2);
        Vector2d scale = new((Fixed64)2, (Fixed64)3);
        Fixed64 rotation = Fixed64.Half;
        Vector2d[] vertices =
        {
            new(-Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One),
            new(Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One)
        };

        CompoundColliderPart2D shapeDefaults = new(ColliderShapeDefinition2D.Circle(Fixed64.Half));
        CompoundColliderPart2D transformDefaults = new(
            ColliderShapeDefinition2D.AABBox(Vector2d.One),
            offset,
            rotation);
        CompoundColliderPart2D circle = CompoundColliderPart2D.Circle(Fixed64.Half, offset, material);
        CompoundColliderPart2D materialCapsule = CompoundColliderPart2D.Capsule(Fixed64.Half, (Fixed64)3, offset, material);
        CompoundColliderPart2D capsule = CompoundColliderPart2D.Capsule(Fixed64.Half, (Fixed64)3, offset, rotation, scale);
        CompoundColliderPart2D box = CompoundColliderPart2D.AABBox(new Vector2d((Fixed64)2, (Fixed64)4), offset);
        CompoundColliderPart2D transformedBox = CompoundColliderPart2D.AABBox(
            new Vector2d((Fixed64)2, (Fixed64)4),
            offset,
            rotation,
            scale);
        CompoundColliderPart2D polygon = CompoundColliderPart2D.ConvexPolygon(vertices, offset, material);
        CompoundColliderPart2D transformedPolygon = CompoundColliderPart2D.ConvexPolygon(vertices, offset, rotation, scale);

        shapeDefaults.Shape.Kind.Should().Be(ColliderShapeDefinition2DKind.Circle);
        shapeDefaults.LocalOffset.Should().Be(Vector2d.Zero);
        shapeDefaults.LocalRotation.Should().Be(Fixed64.Zero);
        shapeDefaults.LocalScale.Should().Be(Vector2d.One);

        transformDefaults.Shape.Kind.Should().Be(ColliderShapeDefinition2DKind.AABBox);
        transformDefaults.LocalOffset.Should().Be(offset);
        transformDefaults.LocalRotation.Should().Be(rotation);
        transformDefaults.LocalScale.Should().Be(Vector2d.One);

        circle.Shape.Kind.Should().Be(ColliderShapeDefinition2DKind.Circle);
        circle.LocalOffset.Should().Be(offset);
        circle.LocalScale.Should().Be(Vector2d.One);
        circle.Material.Should().Be(material);

        materialCapsule.Material.Should().Be(material);
        capsule.Shape.Kind.Should().Be(ColliderShapeDefinition2DKind.Capsule);
        capsule.LocalRotation.Should().Be(rotation);
        capsule.LocalScale.Should().Be(scale);
        capsule.Material.Should().Be(PhysicsMaterial.Default);

        box.Shape.Kind.Should().Be(ColliderShapeDefinition2DKind.AABBox);
        transformedBox.LocalRotation.Should().Be(rotation);
        transformedBox.LocalScale.Should().Be(scale);
        polygon.Shape.Kind.Should().Be(ColliderShapeDefinition2DKind.ConvexPolygon);
        polygon.Material.Should().Be(material);
        transformedPolygon.Shape.Kind.Should().Be(ColliderShapeDefinition2DKind.ConvexPolygon);
        transformedPolygon.LocalRotation.Should().Be(rotation);
        transformedPolygon.LocalScale.Should().Be(scale);
    }

    [Fact]
    public void CompoundColliderPart2D_ShouldRejectNonPositiveScaleComponents()
    {
        ColliderShapeDefinition2D shape = ColliderShapeDefinition2D.Circle(Fixed64.One);

        Action zeroX = () => _ = new CompoundColliderPart2D(
            shape,
            Vector2d.Zero,
            Fixed64.Zero,
            new Vector2d(Fixed64.Zero, Fixed64.One));
        Action negativeY = () => _ = new CompoundColliderPart2D(
            shape,
            Vector2d.Zero,
            Fixed64.Zero,
            new Vector2d(Fixed64.One, -Fixed64.One));

        zeroX.Should().Throw<ArgumentException>().WithParameterName("localScale");
        negativeY.Should().Throw<ArgumentException>().WithParameterName("localScale");
    }

    [Fact]
    public void Constructor_ShouldReservePartsForCompoundLifecycleOnly()
    {
        var compound = new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        LSCollider2D part = compound.GetPartCollider(0);

        Action act = part.Simulate;

        act.Should().Throw<InvalidOperationException>()
            .WithMessage("*standalone lifecycle*");
    }

    [Fact]
    public void RecordData_OnContextBoundPart_ShouldRemainGeometryOnly()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        _ = CreateBody(context, compound, Vector2d.Zero);
        LSCollider2D part = compound.GetPartCollider(0);
        var chronicler = new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            ["Active"] = false,
            ["Radius"] = Fixed64.One
        });

        part.RecordData(chronicler);

        context.Physics2D.ColliderCount.Should().Be(1);
        context.Physics2D.TryGetColliderById(part.Id, out _).Should().BeFalse();
        compound.IsActive.Should().BeTrue();
    }

    [Fact]
    public void RecordData_OnUnboundCompound_ShouldApplySharedStateWithoutShapePayload()
    {
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        Vector2d loadedOffset = new(Fixed64.One, (Fixed64)(-2));
        var chronicler = new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            ["Active"] = false,
            ["LocalOffset"] = loadedOffset
        });

        compound.RecordData(chronicler);

        compound.IsActive.Should().BeFalse();
        compound.LocalOffset.Should().Be(loadedOffset);
        compound.PartCount.Should().Be(1);
        compound.IsPartitioned.Should().BeFalse();
        compound.IsMixedPartitioned.Should().BeFalse();
    }

    [Fact]
    public void Initialize_ShouldApplyOwnerLocalOffsetToAggregateBounds()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(-Fixed64.One, Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)2, Fixed64.Zero)))
        {
            LocalOffset = new Vector2d((Fixed64)5, (Fixed64)(-2))
        };

        _ = CreateBody(context, compound, Vector2d.Zero);

        compound.Bounds.Min.X.Should().Be(Fixed64.FromFraction(7, 2));
        compound.Bounds.Max.X.Should().Be(Fixed64.FromFraction(15, 2));
        compound.Bounds.Min.Y.Should().Be(-Fixed64.FromFraction(5, 2));
        compound.Bounds.Max.Y.Should().Be(-Fixed64.FromFraction(3, 2));
        compound.Center.Should().Be(new Vector2d((Fixed64)5, (Fixed64)(-2)));
    }

    [Fact]
    public void PartShapeMutation_ShouldRefreshAggregateBounds()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        _ = CreateBody(context, compound, Vector2d.Zero);
        var part = (LSCircleCollider2D)compound.GetPartCollider(0);

        part.Radius = Fixed64.One;
        compound.Simulate();

        compound.Bounds.Min.X.Should().Be(-Fixed64.One);
        compound.Bounds.Max.X.Should().Be(Fixed64.One);
        compound.Bounds.Min.Y.Should().Be(-Fixed64.One);
        compound.Bounds.Max.Y.Should().Be(Fixed64.One);
        compound.RuntimeShapeVersion.Should().BeGreaterThan(1u);
    }

    [Fact]
    public void ScaledRadius_ShouldUseFarthestCanonicalPartProxy()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(-Fixed64.One, Fixed64.Zero)),
            CompoundColliderPart2D.Capsule(Fixed64.Half, (Fixed64)4, new Vector2d(Fixed64.Zero, (Fixed64)2)),
            CompoundColliderPart2D.AABBox(new Vector2d((Fixed64)2, (Fixed64)2), new Vector2d((Fixed64)3, Fixed64.Zero)),
            CompoundColliderPart2D.Capsule(Fixed64.Half, Fixed64.One, Vector2d.Zero));

        _ = CreateBody(context, compound, Vector2d.Zero);

        new Vector2d(Fixed64.One, Fixed64.One)
            .TryGetMagnitudeCeiling(out Fixed64 boxRadius)
            .Should().BeTrue();
        compound.ScaledRadius.Should().Be((Fixed64)3 + boxRadius);
    }

    [Fact]
    public void PreInitializationQueries_WithRotatedOffsetPart_ShouldMatchCommittedGeometry()
    {
        var offset = new Vector2d((Fixed64)3, Fixed64.Zero);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(
                Fixed64.One,
                offset,
                Fixed64.HalfPi,
                Vector2d.One));

        Fixed64 preInitializationRadius = compound.ScaledRadius;
        bool preInitializationContains = compound.ContainsPoint(offset);
        Vector2d preInitializationSupport =
            compound.GetSupportPoint(Vector2d.Right);

        using GravitasWorldContext context =
            Physics2DTestWorld.CreateContext();
        _ = CreateBody(context, compound, Vector2d.Zero);

        preInitializationRadius.Should().Be((Fixed64)4);
        preInitializationRadius.Should().Be(compound.ScaledRadius);
        preInitializationContains.Should().BeTrue();
        compound.ContainsPoint(offset).Should().BeTrue();
        preInitializationSupport.Should().Be(
            offset + Vector2d.Right);
        compound.GetSupportPoint(Vector2d.Right).Should().Be(
            preInitializationSupport);
    }

    [Fact]
    public void ScaledRadius_WithNearerTrailingCircle_ShouldRetainFarthestPart()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.One, Vector2d.Right * (Fixed64)4),
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        _ = CreateBody(context, compound, Vector2d.Zero);

        compound.ScaledRadius.Should().Be((Fixed64)5);
    }

    [Fact]
    public void CapsulePart_WithScaledHeightBelowDiameter_ShouldRejectInitializationAtomically()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Capsule(
                Fixed64.One,
                (Fixed64)2,
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d((Fixed64)2, Fixed64.Half)));
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        var body = new SolidBody2D(
            new TestMatterAgent(context, transform),
            compound);

        Action initialize = () => body.Initialize(
            Vector2d.Zero,
            motionType: BodyMotionType.Static);

        initialize.Should()
            .Throw<ArgumentException>()
            .WithParameterName("snapshot")
            .WithMessage("*Scaled 2D capsule height must be at least the capsule diameter.*");
        body.Active.Should().BeFalse();
        compound.Body.Should().BeNull();
        compound.Id.Should().Be(-1);
    }

    [Fact]
    public void GetClosestPoint_ShouldUseNearestPart()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-4), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)2, Fixed64.Zero)));
        _ = CreateBody(context, compound, Vector2d.Zero);

        Vector2d closest = compound.GetClosestPoint(new Vector2d((Fixed64)3, Fixed64.Zero));
        Vector2d closestToFirst = compound.GetClosestPoint(new Vector2d((Fixed64)(-5), Fixed64.Zero));
        Vector2d equalDistanceTie = compound.GetClosestPoint(new Vector2d(-Fixed64.One, Fixed64.Zero));

        closest.Should().Be(new Vector2d(Fixed64.FromFraction(5, 2), Fixed64.Zero));
        closestToFirst.Should().Be(new Vector2d(-Fixed64.FromFraction(9, 2), Fixed64.Zero));
        equalDistanceTie.Should().Be(new Vector2d(-Fixed64.FromFraction(7, 2), Fixed64.Zero));
    }

    [Fact]
    public void GetClosestPoint_WithFarCompound_ShouldSelectExactNearestPart()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d(
                    Fixed64.MaxValue - (Fixed64)8,
                    (Fixed64)(-4),
                    (Fixed64)(-4)),
                new Vector3d(
                    Fixed64.MaxValue,
                    (Fixed64)4,
                    (Fixed64)4)),
            out _).Should().BeTrue();
        Vector2d center = new(
            Fixed64.MaxValue - (Fixed64)4,
            Fixed64.Zero);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(
                Fixed64.One,
                Vector2d.Right),
            CompoundColliderPart2D.Circle(
                Fixed64.One,
                Vector2d.Left));
        _ = CreateBody(context, compound, center);

        compound.GetClosestPoint(
                new Vector2d(Fixed64.MinValue, Fixed64.Zero))
            .Should().Be(
                center - Vector2d.Right * Fixed64.Two);
    }

    [Fact]
    public void ContainsPoint_ShouldCheckAllParts()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-4), Fixed64.Zero)),
            CompoundColliderPart2D.AABBox(new Vector2d((Fixed64)2, (Fixed64)2), new Vector2d((Fixed64)2, Fixed64.Zero)));
        _ = CreateBody(context, compound, Vector2d.Zero);

        compound.ContainsPoint(new Vector2d((Fixed64)2, Fixed64.Zero)).Should().BeTrue();
        compound.ContainsPoint(new Vector2d(Fixed64.Zero, (Fixed64)3)).Should().BeFalse();
    }

    [Fact]
    public void GetSupportPoint_ShouldUseFarthestPart()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-2), Fixed64.Zero)),
            CompoundColliderPart2D.AABBox(new Vector2d((Fixed64)2, (Fixed64)2), new Vector2d((Fixed64)3, Fixed64.Zero)));
        _ = CreateBody(context, compound, Vector2d.Zero);

        compound.GetSupportPoint(Vector2d.Right).Should().Be(new Vector2d((Fixed64)4, Fixed64.One));
        compound.GetSupportPoint(Vector2d.Left).Should().Be(new Vector2d(-Fixed64.FromFraction(5, 2), Fixed64.Zero));
    }

    [Fact]
    public void ScaledRadius_WithSaturatingVertexSquares_ShouldRemainConservative()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(extent: 2);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.ConvexPolygon(
                new[]
                {
                    Vector2d.Zero,
                    new Vector2d((Fixed64)60000, Fixed64.Zero),
                    new Vector2d((Fixed64)60000, (Fixed64)80000)
                },
                Vector2d.Zero));
        _ = CreateBody(context, compound, Vector2d.Zero);

        compound.ScaledRadius.Should().Be((Fixed64)100000);
    }

    [Fact]
    public void ScaledRadius_WithConceptualVertexBeyondScalarFace_ShouldUseCanonicalOffset()
    {
        Fixed64 centerX = Fixed64.MaxValue - Fixed64.FromFraction(1, 4);
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d(
                    Fixed64.MaxValue - (Fixed64)4,
                    (Fixed64)(-2),
                    (Fixed64)(-2)),
                new Vector3d(
                    Fixed64.MaxValue,
                    (Fixed64)2,
                    (Fixed64)2)),
            out _).Should().BeTrue();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(
                new Vector2d((Fixed64)2, (Fixed64)2),
                Vector2d.Zero));
        _ = CreateBody(context, compound, new Vector2d(centerX, Fixed64.Zero));

        compound.ScaledRadius.Should().Be(FixedMath.Sqrt((Fixed64)2));
    }

    [Fact]
    public void ScaledRadius_WithSubUnitDiagonalPartOffset_ShouldRoundOutward()
    {
        Fixed64 raw = Fixed64.MinIncrement;
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(
                raw,
                new Vector2d(raw, raw)));
        _ = CreateBody(context, compound, Vector2d.Zero);

        compound.ScaledRadius.Should().Be(Fixed64.FromRaw(3));
    }

    [Fact]
    public void ScaledRadius_WithOddRawCapsuleAxis_ShouldRoundOutward()
    {
        Fixed64 raw = Fixed64.MinIncrement;
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Capsule(
                raw,
                Fixed64.FromRaw(3),
                Vector2d.Zero));
        _ = CreateBody(context, compound, Vector2d.Zero);

        compound.ScaledRadius.Should().Be(Fixed64.FromRaw(2));
    }

    [Fact]
    public void ScaledRadius_WhenFinalPartRadiusExceedsDomain_ShouldReturnMaximum()
    {
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(
                Fixed64.MinIncrement,
                new Vector2d(Fixed64.MaxValue, Fixed64.Zero)));

        compound.ScaledRadius.Should().Be(Fixed64.MaxValue);
    }

    private static SolidBody2D CreateBody(GravitasWorldContext context, LSCollider2D collider, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, motionType: BodyMotionType.Static);
        return body;
    }
}
