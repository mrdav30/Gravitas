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
    public void HalfExtents_ShouldRemainHalfOfAuthoredSize()
    {
        var collider = new LSAABBoxCollider2D(new Vector2d((Fixed64)6, (Fixed64)4));

        collider.HalfExtents.Should().Be(new Vector2d((Fixed64)3, (Fixed64)2));
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
