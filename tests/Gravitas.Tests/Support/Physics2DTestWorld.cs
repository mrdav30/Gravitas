using FixedMathSharp;
using FluentAssertions;
using GridForge.Configuration;

namespace Gravitas.Tests.Support;

internal static class Physics2DTestWorld
{
    public static GravitasWorldContext CreateContext(int frameRate = 4, int extent = 32)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(frameRate);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        EnsureGrid(context, extent);
        return context;
    }

    public static void EnsureGrid(GravitasWorldContext context, int extent = 32)
    {
        if (context.World.ActiveGrids.Count > 0)
            return;

        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-extent), Fixed64.Zero, (Fixed64)(-extent)),
                new Vector3d((Fixed64)extent, Fixed64.Zero, (Fixed64)extent)),
            out _).Should().BeTrue();
    }
}
