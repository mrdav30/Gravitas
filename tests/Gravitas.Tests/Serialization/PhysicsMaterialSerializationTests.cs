using FixedMathSharp;
using FluentAssertions;
using Gravitas.Materials;
#if !GRAVITAS_DISABLE_MEMORYPACK
using MemoryPack;
#endif
using System.Text.Json;
using Xunit;

namespace Gravitas.Tests.Serialization;

public sealed class PhysicsMaterialSerializationTests
{
    [Fact]
    public void MaterialJsonRoundTrip_ShouldPreserveCoefficientsAndPolicies()
    {
        var source = new PhysicsMaterial(
            Fixed64.FromFraction(5, 4),
            Fixed64.FromFraction(3, 4),
            Fixed64.FromFraction(7, 8),
            PhysicsMaterialCombine.Maximum,
            PhysicsMaterialCombine.Average);

        string payload = JsonSerializer.Serialize(source);
        PhysicsMaterial clone = JsonSerializer.Deserialize<PhysicsMaterial>(payload);

        clone.Should().Be(source);
    }

#if !GRAVITAS_DISABLE_MEMORYPACK
    [Fact]
    public void MaterialMemoryPackRoundTrip_ShouldPreserveCoefficientsAndPolicies()
    {
        var source = new PhysicsMaterial(
            Fixed64.FromFraction(9, 8),
            Fixed64.FromFraction(5, 8),
            Fixed64.FromFraction(3, 4),
            PhysicsMaterialCombine.Multiply,
            PhysicsMaterialCombine.Maximum);

        byte[] payload = MemoryPackSerializer.Serialize(source);
        PhysicsMaterial clone = MemoryPackSerializer.Deserialize<PhysicsMaterial>(payload);

        clone.Should().Be(source);
    }
#endif
}
