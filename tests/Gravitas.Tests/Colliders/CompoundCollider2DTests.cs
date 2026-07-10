using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class CompoundCollider2DTests
{
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

        CompoundColliderPart2D circle = CompoundColliderPart2D.Circle(Fixed64.Half, offset, material);
        CompoundColliderPart2D capsule = CompoundColliderPart2D.Capsule(Fixed64.Half, (Fixed64)3, offset, rotation, scale);
        CompoundColliderPart2D box = CompoundColliderPart2D.AABox(new Vector2d((Fixed64)2, (Fixed64)4), offset);
        CompoundColliderPart2D polygon = CompoundColliderPart2D.ConvexPolygon(vertices, offset, material);

        circle.Shape.Kind.Should().Be(ColliderShapeDefinition2DKind.Circle);
        circle.LocalOffset.Should().Be(offset);
        circle.LocalScale.Should().Be(Vector2d.One);
        circle.Material.Should().Be(material);

        capsule.Shape.Kind.Should().Be(ColliderShapeDefinition2DKind.Capsule);
        capsule.LocalRotation.Should().Be(rotation);
        capsule.LocalScale.Should().Be(scale);
        capsule.Material.Should().Be(PhysicsMaterial.Default);

        box.Shape.Kind.Should().Be(ColliderShapeDefinition2DKind.AABBox);
        polygon.Shape.Kind.Should().Be(ColliderShapeDefinition2DKind.ConvexPolygon);
        polygon.Material.Should().Be(material);
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
    public void ScaledRadius_ShouldUseFarthestCircleCapsuleAndVertexPart()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(-Fixed64.One, Fixed64.Zero)),
            CompoundColliderPart2D.Capsule(Fixed64.Half, (Fixed64)4, new Vector2d(Fixed64.Zero, (Fixed64)2)),
            CompoundColliderPart2D.AABox(new Vector2d((Fixed64)2, (Fixed64)2), new Vector2d((Fixed64)3, Fixed64.Zero)));

        _ = CreateBody(context, compound, Vector2d.Zero);

        compound.ScaledRadius.Should().Be(FixedMath.Sqrt((Fixed64)17));
    }

    [Fact]
    public void CapsulePart_WithWideLocalScale_ShouldClampScaledHeightToScaledDiameter()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Capsule(
                Fixed64.One,
                (Fixed64)2,
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d((Fixed64)2, Fixed64.Half)));

        _ = CreateBody(context, compound, Vector2d.Zero);

        var capsule = (LSCapsuleCollider2D)compound.GetPartCollider(0);
        capsule.ScaledHeight.Should().Be((Fixed64)4);
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

        closest.Should().Be(new Vector2d(Fixed64.FromFraction(5, 2), Fixed64.Zero));
    }

    [Fact]
    public void ContainsPoint_ShouldCheckAllParts()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-4), Fixed64.Zero)),
            CompoundColliderPart2D.AABox(new Vector2d((Fixed64)2, (Fixed64)2), new Vector2d((Fixed64)2, Fixed64.Zero)));
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
            CompoundColliderPart2D.AABox(new Vector2d((Fixed64)2, (Fixed64)2), new Vector2d((Fixed64)3, Fixed64.Zero)));
        _ = CreateBody(context, compound, Vector2d.Zero);

        compound.GetSupportPoint(Vector2d.Right).Should().Be(new Vector2d((Fixed64)4, Fixed64.One));
        compound.GetSupportPoint(Vector2d.Left).Should().Be(new Vector2d(-Fixed64.FromFraction(5, 2), Fixed64.Zero));
    }

    private static SolidBody2D CreateBody(GravitasWorldContext context, LSCollider2D collider, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Position
        };
        body.Initialize(position);
        return body;
    }
}
