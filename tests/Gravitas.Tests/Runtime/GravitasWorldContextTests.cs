using FixedMathSharp;
using FluentAssertions;
using GridForge.Configuration;
using GridForge.Grids;
using GridForge.Grids.Topology;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Runtime;

public sealed class GravitasWorldContextTests
{
    [Fact]
    public void Attach_ShouldBindExternalWorldWithoutTakingOwnershipByDefault()
    {
        using var world = new GridWorld();

        using GravitasWorldContext context = GravitasWorldContext.Attach(world);

        context.World.Should().BeSameAs(world);
        context.VoxelSize.Should().Be(GridWorld.DefaultRectangularCellSize);
        context.IsDisposed.Should().BeFalse();

        context.Dispose();

        context.IsDisposed.Should().BeTrue();
        world.IsActive.Should().BeTrue();
    }

    [Fact]
    public void Attach_ShouldRejectNullOrInactiveWorlds()
    {
        Action nullWorld = () => GravitasWorldContext.Attach(null!);
        nullWorld.Should().Throw<ArgumentNullException>().WithParameterName("world");

        var inactiveWorld = new GridWorld();
        inactiveWorld.Dispose();

        Action inactive = () => GravitasWorldContext.Attach(inactiveWorld);
        inactive.Should().Throw<InvalidOperationException>().WithMessage("*active GridWorld*");
    }

    [Fact]
    public void Attach_ShouldRejectSameWorldUntilExistingContextIsDisposed()
    {
        using var world = new GridWorld();
        using GravitasWorldContext contextA = GravitasWorldContext.Attach(world);
        GravitasWorldContext? duplicate = null;

        Action duplicateAttach = () => duplicate = GravitasWorldContext.Attach(world);

        duplicateAttach.Should()
            .Throw<InvalidOperationException>()
            .WithMessage("*already*GravitasWorldContext*");
        duplicate?.Dispose();

        contextA.Dispose();

        using GravitasWorldContext contextB = GravitasWorldContext.Attach(world);
        contextB.World.Should().BeSameAs(world);
    }

    [Fact]
    public void Attach_ShouldDisposeExternalWorld_WhenOwnershipIsTaken()
    {
        var world = new GridWorld();

        using GravitasWorldContext context = GravitasWorldContext.Attach(world, takeOwnership: true);

        context.World.Should().BeSameAs(world);

        context.Dispose();

        world.IsActive.Should().BeFalse();
    }

    [Fact]
    public void CreateOwned_ShouldCreateContextOwnedWorld()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        GridWorld world = context.World;

        world.IsActive.Should().BeTrue();
        context.VoxelSize.Should().Be(GridWorld.DefaultRectangularCellSize);

        context.Dispose();

        world.IsActive.Should().BeFalse();
    }

    [Fact]
    public void DisposeOwned_ShouldKeepWorldRegisteredDuringWorldReset()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        GridWorld world = context.World;
        GravitasWorldContext? reentrantContext = null;
        Exception? attachException = null;
        world.OnReset += () =>
            attachException = Record.Exception(() => reentrantContext = GravitasWorldContext.Attach(world));

        try
        {
            context.Dispose();

            attachException.Should()
                .BeOfType<InvalidOperationException>()
                .Which.Message.Should().Contain("already attached");
            reentrantContext.Should().BeNull();
            world.IsActive.Should().BeFalse();
        }
        finally
        {
            reentrantContext?.Dispose();
            context.Dispose();
        }
    }

    [Fact]
    public void VoxelSize_ShouldReflectRepresentativeGridTopologyMetric()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var configuration = new GridConfiguration(
            new Vector3d(-8, -8, -8),
            new Vector3d(8, 8, 8),
            topologyMetrics: GridTopologyMetrics.Rectangular((Fixed64)2, (Fixed64)3, (Fixed64)4));

        context.World.TryAddGrid(configuration, out _).Should().BeTrue();

        context.VoxelSize.Should().Be((Fixed64)4);
    }

    [Fact]
    public void Reset_ShouldClearOnlyThisContextClock()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();

        contextA.Simulate();
        contextB.Simulate();
        contextB.Simulate();

        contextA.Reset();

        contextA.FrameCount.Should().Be(0);
        contextA.TotalTime.Should().Be(Fixed64.Zero);
        contextB.FrameCount.Should().Be(2);
        contextB.TotalTime.Should().Be(contextB.DeltaTime * 2);
    }

    [Fact]
    public void RegisterOnSimulate_ShouldInvokeContextHooksInOrder()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var calls = new List<string>();

        using IDisposable late = context.RegisterOnSimulate("ContextHook.Late", 100, () => calls.Add("late"));
        using IDisposable early = context.RegisterOnSimulate("ContextHook.Early", -100, () => calls.Add("early"));

        context.Simulate();

        calls.Should().ContainInOrder("early", "late");
    }

    [Fact]
    public void LifecycleHookRegistration_ShouldRejectInvalidOwnersCallbacksAndDuplicates()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        using IDisposable first = context.RegisterOnSimulate("duplicate", 0, () => { });

        Action whitespaceOwner = () => context.RegisterOnSimulate(" ", 0, () => { });
        Action nullCallback = () => context.RegisterOnSimulate("null-callback", 0, null!);
        Action duplicateOwner = () => context.RegisterOnSimulate("duplicate", 1, () => { });

        whitespaceOwner.Should().Throw<ArgumentException>().WithParameterName("owner");
        nullCallback.Should().Throw<ArgumentNullException>().WithParameterName("callback");
        duplicateOwner.Should().Throw<InvalidOperationException>().WithMessage("*duplicate*");
    }
}
