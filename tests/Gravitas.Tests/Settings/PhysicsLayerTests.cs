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

namespace Gravitas.Tests.Settings;

public sealed class PhysicsLayerTests
{
    [Fact]
    public void PhysicsLayer_ShouldExposeIndexWithoutChangingItIntoMask()
    {
        var layer = new PhysicsLayer(3, "Gameplay");

        layer.Index.Should().Be(3);
        layer.MaskBit.Should().Be(1 << 3);
        layer.ToString().Should().Be("3");
        layer.GetHashCode().Should().Be(3);
        PhysicsLayer.LayerToName(3).Should().Be("Gameplay");
        PhysicsLayer.NameToLayer("Gameplay").Should().Be(3);
        PhysicsLayer.LayerToName(12).Should().BeNull();
        PhysicsLayer.NameToLayer("Missing").Should().Be(-1);
    }

    [Fact]
    public void PhysicsLayer_ShouldSupportMutationEqualityAndIndexValidation()
    {
        var layer = new PhysicsLayer(1);

        layer.Set(4);

        layer.Should().Be(new PhysicsLayer(4));
        layer.Equals((object)new PhysicsLayer(4)).Should().BeTrue();
        layer.Equals("4").Should().BeFalse();
        (layer == new PhysicsLayer(4)).Should().BeTrue();
        (layer != new PhysicsLayer(5)).Should().BeTrue();

        Action belowRange = () => _ = new PhysicsLayer(-1);
        Action aboveRange = () => layer.Set(32);
        belowRange.Should().Throw<ArgumentOutOfRangeException>().WithParameterName("index");
        aboveRange.Should().Throw<ArgumentOutOfRangeException>().WithParameterName("index");
    }

    [Fact]
    public void PhysicsLayerMask_ShouldIncludeSingleMultipleAllAndNone()
    {
        PhysicsLayer layerZero = new(0);
        PhysicsLayer layerThree = new(3);
        PhysicsLayer layerFive = new(5);

        PhysicsLayerMask.FromLayer(layerThree).Includes(layerThree).Should().BeTrue();
        PhysicsLayerMask.FromLayer(layerThree).Includes(layerZero).Should().BeFalse();

        PhysicsLayerMask multiple = PhysicsLayerMask.FromLayers(layerZero, layerFive);
        multiple.Includes(layerZero).Should().BeTrue();
        multiple.Includes(layerFive).Should().BeTrue();
        multiple.Includes(layerThree).Should().BeFalse();
        multiple.GetHashCode().Should().Be(multiple.Bits);

        PhysicsLayerMask.All.Includes(layerThree).Should().BeTrue();
        PhysicsLayerMask.All.Bits.Should().Be(-1);
        PhysicsLayerMask.None.Includes(layerZero).Should().BeFalse();
        PhysicsLayerMask.None.Bits.Should().Be(0);
        PhysicsLayerMask.None.ToString().Should().Be("0");
        PhysicsLayerMask.FromLayer(3).Should().Be(PhysicsLayerMask.FromLayer(layerThree));
        PhysicsLayerMask.FromLayer(3).Includes(3).Should().BeTrue();
        (PhysicsLayerMask.FromLayer(3) == PhysicsLayerMask.FromLayer(layerThree)).Should().BeTrue();
        (PhysicsLayerMask.FromLayer(3) != PhysicsLayerMask.FromLayer(layerFive)).Should().BeTrue();
        PhysicsLayerMask.FromLayer(3).Equals((object)PhysicsLayerMask.FromLayer(layerThree)).Should().BeTrue();
        PhysicsLayerMask.FromLayer(3).Equals("3").Should().BeFalse();
    }

    [Fact]
    public void PhysicsLayerMask_Excluding_ShouldPreserveAllOtherLayers()
    {
        PhysicsLayerMask mask = PhysicsLayerMask.Excluding(new PhysicsLayer(7), new PhysicsLayer(10));

        mask.Includes(new PhysicsLayer(0)).Should().BeTrue();
        mask.Includes(new PhysicsLayer(7)).Should().BeFalse();
        mask.Includes(new PhysicsLayer(10)).Should().BeFalse();
        mask.Includes(new PhysicsLayer(31)).Should().BeTrue();
    }

    [Fact]
    public void QueryServices_ShouldTreatLayerMaskAsIncludeMask()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSSphereCollider layerZero = CreateDynamicSphere(context, Vector3d.Zero, new PhysicsLayer(0));
        LSSphereCollider layerOne = CreateDynamicSphere(context, new Vector3d(2, 0, 0), new PhysicsLayer(1));
        var rayHits = new SwiftList<Physics3DHit>();
        var circleHits = new SwiftList<Physics3DHit>();
        PhysicsLayerMask onlyLayerOne = PhysicsLayerMask.FromLayer(1);

        int rayCount = context.Query3D.RaycastAll(
            new Vector3d((Fixed64)(-2), -Fixed64.FromFraction(1, 4), Fixed64.Zero),
            new Vector3d((Fixed64)4, Fixed64.FromFraction(1, 4), Fixed64.Zero),
            onlyLayerOne,
            rayHits);
        int circleCount = context.Query3D.OverlapCircleAll(
            new Vector3d(1, 0, 0),
            (Fixed64)4,
            onlyLayerOne,
            circleHits);

        rayCount.Should().Be(1);
        rayHits[0].Collider.Should().BeSameAs(layerOne);
        circleCount.Should().Be(1);
        circleHits[0].Collider.Should().BeSameAs(layerOne);
        rayHits.Should().NotContain(hit => ReferenceEquals(hit.Collider, layerZero));
        circleHits.Should().NotContain(hit => ReferenceEquals(hit.Collider, layerZero));
    }

    [Fact]
    public void CollisionMatrix_ShouldAllowAndDenyByLayerIndex()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var matrix = new[,]
        {
            { true, false },
            { false, true }
        };
        context.ApplySettings(new PhysicsSettings(PhysicsSettings.DefaultFrameRate, matrix));
        LSSphereCollider layerZero = CreateDynamicSphere(context, Vector3d.Zero, new PhysicsLayer(0));
        LSSphereCollider layerOne = CreateDynamicSphere(context, new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), new PhysicsLayer(1));
        LSSphereCollider layerZeroOther = CreateDynamicSphere(context, new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), new PhysicsLayer(0));

        context.Physics.RequireCollisionPair(layerZero, layerOne).Should().BeFalse();
        context.Physics.RequireCollisionPair(layerZero, layerZeroOther).Should().BeTrue();
    }

    [Fact]
    public void GroundCheckLayerMask_ShouldBeSettingsOwnedAndConfigurable()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        PhysicsLayerMask defaultMask = context.Settings.GroundCheckLayerMask;
        PhysicsLayerMask customMask = PhysicsLayerMask.FromLayer(2);
        var settings = new PhysicsSettings(PhysicsSettings.DefaultFrameRate, null, customMask);

        context.ApplySettings(settings);

        defaultMask.Includes(new PhysicsLayer(0)).Should().BeTrue();
        context.Settings.GroundCheckLayerMask.Should().Be(customMask);
    }

    private static LSSphereCollider CreateDynamicSphere(
        GravitasWorldContext context,
        Vector3d position,
        PhysicsLayer layer)
    {
        EnsureGrid(context);
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider
        {
            Layer = layer
        };
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return collider;
    }

    private static void EnsureGrid(GravitasWorldContext context)
    {
        if (context.World.ActiveGrids.Count > 0)
            return;

        GridConfiguration configuration = new(
            new Vector3d(-4, -4, -4),
            new Vector3d(6, 6, 6));

        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
    }
}
