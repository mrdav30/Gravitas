using FluentAssertions;
using Gravitas.Benchmarking;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.Support;

public sealed class ContinuousCollisionBenchmarkLayoutTests
{
    [Fact]
    public void CreateDescriptors_ShouldBeDeterministicAcrossRepeatedConstruction()
    {
        ContinuousCollisionBenchmarkDescriptor[] first = ContinuousCollisionBenchmarkLayout
            .CreateDescriptors(bodyCount: 129)
            .ToArray();
        ContinuousCollisionBenchmarkDescriptor[] second = ContinuousCollisionBenchmarkLayout
            .CreateDescriptors(bodyCount: 129)
            .ToArray();

        first.Should().Equal(second);
        first.Should().OnlyHaveUniqueItems(descriptor => descriptor.ColliderOrdinal);
        first.Select(descriptor => descriptor.Layer).Should().OnlyContain(layer => layer == default);
    }
}
