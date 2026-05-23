using FixedMathSharp;
using FluentAssertions;
using Gravitas.Support;
using Xunit;

namespace Gravitas.Tests.Settings;

public sealed class PhysicsSettingsSaverTests
{
    [Fact]
    public void ApplyTo_ShouldApplySettingsOnlyToTargetContext()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        var saver = new PhysicsSettingsSaver
        {
            FrameRate = 20,
            CollisionMatrix = new[]
            {
                new MatrixRow { row = new[] { true, false } },
                new MatrixRow { row = new[] { false, true } }
            },
            GroundCheckLayerMaskBits = PhysicsLayerMask.FromLayer(1).Bits
        };

        saver.ApplyTo(contextA);

        contextA.Settings.FrameRate.Should().Be(20);
        contextA.FrameRate.Should().Be(20);
        contextA.DeltaTime.Should().Be(Fixed64.One / (Fixed64)20);
        contextA.Settings.CollisionMatrix[0, 0].Should().BeTrue();
        contextA.Settings.CollisionMatrix[0, 1].Should().BeFalse();
        contextA.Settings.CollisionMatrix[1, 0].Should().BeFalse();
        contextA.Settings.CollisionMatrix[1, 1].Should().BeTrue();
        contextA.Settings.GroundCheckLayerMask.Should().Be(PhysicsLayerMask.FromLayer(1));

        contextB.Settings.FrameRate.Should().Be(PhysicsSettings.DefaultFrameRate);
        contextB.FrameRate.Should().Be(PhysicsSettings.DefaultFrameRate);
        contextB.DeltaTime.Should().Be(Fixed64.One / (Fixed64)PhysicsSettings.DefaultFrameRate);
    }
}
