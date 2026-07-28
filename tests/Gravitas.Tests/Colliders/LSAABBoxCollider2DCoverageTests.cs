using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class LSAABBoxCollider2DCoverageTests
{
    [Fact]
    public void ReassigningCurrentPrimitiveDimensions_ShouldNotRebuildRuntimeGeometry()
    {
        using GravitasWorldContext context =
            Physics2DTestWorld.CreateContext();
        var box = new LSAABBoxCollider2D(Vector2d.One);
        var circle = new LSCircleCollider2D(Fixed64.Half);
        var capsule = new LSCapsuleCollider2D(
            Fixed64.Half,
            Fixed64.Two);
        box.InitializeWithNoBody(new TestMatterAgent(context));
        circle.InitializeWithNoBody(new TestMatterAgent(context));
        capsule.InitializeWithNoBody(new TestMatterAgent(context));
        uint boxVersion = box.RuntimeShapeVersion;
        uint circleVersion = circle.RuntimeShapeVersion;
        uint capsuleVersion = capsule.RuntimeShapeVersion;

        box.Size = box.Size;
        circle.Radius = circle.Radius;
        capsule.Radius = capsule.Radius;
        capsule.Height = capsule.Height;
        box.Simulate();
        circle.Simulate();
        capsule.Simulate();

        box.RuntimeShapeVersion.Should().Be(boxVersion);
        circle.RuntimeShapeVersion.Should().Be(circleVersion);
        capsule.RuntimeShapeVersion.Should().Be(capsuleVersion);
    }

    [Fact]
    public void HalfExtents_ShouldRemainHalfOfAuthoredSize()
    {
        var collider = new LSAABBoxCollider2D(new Vector2d((Fixed64)6, (Fixed64)4));

        collider.HalfExtents.Should().Be(new Vector2d((Fixed64)3, (Fixed64)2));
    }

    [Fact]
    public void ClosestPoint_InsideBox_ShouldPreserveTheQuery()
    {
        var collider = new LSAABBoxCollider2D(
            new Vector2d((Fixed64)6, (Fixed64)4));
        Vector2d point = new(Fixed64.One, Fixed64.Half);

        collider.GetClosestPoint(point).Should().Be(point);
    }

    [Fact]
    public void Area_WhenTheExactProductExceedsFixed64_ShouldRemainSemantic()
    {
        var collider = new LSAABBoxCollider2D(
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue));

        collider.CalculateAreaForMassProperties()
            .TryGetMeasure(out _)
            .Should().BeFalse();
    }

    [Fact]
    public void RecordData_WithNonPositiveLoadedYSize_ShouldRejectWithoutChangingGeometry()
    {
        Vector2d originalSize = new((Fixed64)6, (Fixed64)4);
        var collider = new LSAABBoxCollider2D(originalSize);
        var chronicler = new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            ["Size"] = new Vector2d(Fixed64.One, Fixed64.Zero)
        });
        Action load = () => collider.RecordData(chronicler);

        load.Should().Throw<ArgumentException>().WithParameterName("size");
        collider.Size.Should().Be(originalSize);
        collider.HalfExtents.Should().Be(originalSize * Fixed64.Half);
    }

    [Fact]
    public void RecordData_WithNonPositiveLoadedXSize_ShouldRejectWithoutChangingGeometry()
    {
        Vector2d originalSize = new((Fixed64)6, (Fixed64)4);
        var collider = new LSAABBoxCollider2D(originalSize);
        var chronicler = new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            ["Size"] = new Vector2d(Fixed64.Zero, Fixed64.One)
        });
        Action load = () => collider.RecordData(chronicler);

        load.Should().Throw<ArgumentException>().WithParameterName("size");
        collider.Size.Should().Be(originalSize);
        collider.HalfExtents.Should().Be(originalSize * Fixed64.Half);
    }

    [Fact]
    public void RecordData_WithValidLoadedSize_ShouldUpdateSizeAndHalfExtents()
    {
        var collider = new LSAABBoxCollider2D(Vector2d.One);
        Vector2d loadedSize = new((Fixed64)8, (Fixed64)2);
        var chronicler = new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            ["Size"] = loadedSize
        });

        collider.RecordData(chronicler);

        collider.Size.Should().Be(loadedSize);
        collider.HalfExtents.Should().Be(new Vector2d((Fixed64)4, Fixed64.One));
    }
}
