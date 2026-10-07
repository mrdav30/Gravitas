using Chronicler.Hashing;
using Chronicler.Timing;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Determinism;

public sealed class ReplayConformanceHarnessTests
{
    [Fact]
    public void RunTrace_AppliesCommandsThenBothPhasesExactlyOnceBeforeHashing()
    {
        using PhysicsScenarioBuilder scenario = CreateScenario(PhysicsRuntimeMode.ThreeD);
        GravitasWorldContext context = scenario.Context;
        // Keep the phase-order arithmetic below the world's speed cap.
        context.Environment.MaxSpeed = (Fixed64)64;
        SolidBody body = scenario.CreateSphere(Vector3d.Zero).Body;
        var calls = new List<string>();
        var finalHashes = new List<ChronicleHash>();
        var earlierHashes = new List<ChronicleHash>();
        context.Coroutines.StartCoroutine(ApplyCoroutineCommands());

        // Register in reverse order so the trace also verifies hook ordering.
        using IDisposable simulateLast = context.RegisterOnSimulate("simulate-last", 10, () =>
        {
            Record("simulate-last");
            body.AddLinearImpulse(Vector3d.Right * (Fixed64)3);
        });
        using IDisposable simulateFirst = context.RegisterOnSimulate("simulate-first", -10, () =>
        {
            Record("simulate-first");
            body.AddLinearImpulse(body.LinearVelocity);
        });
        using IDisposable lateLast = context.RegisterOnLateSimulate("late-last", 10, () =>
        {
            Record("late-last");
            earlierHashes.Add(context.ComputeReplayHash());
            body.AddLinearImpulse(body.LinearVelocity);
            finalHashes.Add(context.ComputeReplayHash());
        });
        using IDisposable lateFirst = context.RegisterOnLateSimulate("late-first", -10, () =>
        {
            Record("late-first");
            body.AddLinearImpulse(Vector3d.Right * (Fixed64)5);
        });

        ReplayHashTrace trace = ReplayConformanceHarness.RunTrace(context, 3, (_, frame) =>
        {
            Record($"command-{frame}");
            body.AddLinearImpulse(Vector3d.Right * (Fixed64)(2 + frame) - body.LinearVelocity);
        });

        Assert.Equal(new[]
        {
            "command-0:0:0", "coroutine:1:0", "simulate-first:1:0", "simulate-last:1:0", "late-first:1:1", "late-last:1:1",
            "command-1:1:1", "coroutine:2:1", "simulate-first:2:1", "simulate-last:2:1", "late-first:2:2", "late-last:2:2",
            "command-2:2:2", "coroutine:3:2", "simulate-first:3:2", "simulate-last:3:2", "late-first:3:3", "late-last:3:3"
        }, calls);
        Assert.Equal(3, context.FrameCount);
        Assert.Equal(3, context.LateSimulateToken);
        Assert.Equal(new ChronicleTimestamp(0, 3u << 29), context.ElapsedTime);
        Assert.Equal(Vector3d.Right * (Fixed64)36, body.LinearVelocity);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(33, 8), Fixed64.Zero, Fixed64.Zero), body.Position3d);
        Assert.Equal(finalHashes, trace.Hashes);
        for (int frame = 0; frame < trace.Hashes.Length; frame++)
            Assert.NotEqual(earlierHashes[frame], trace[frame]);

        void Record(string phase) => calls.Add($"{phase}:{context.FrameCount}:{context.LateSimulateToken}");

        IEnumerator<ILockedYieldInstruction> ApplyCoroutineCommands()
        {
            for (int frame = 0; frame < 3; frame++)
            {
                Record("coroutine");
                body.AddLinearImpulse(Vector3d.Right);
                yield return context.Coroutines.WaitForNextSimulate();
            }
        }
    }

    [Fact]
    public void AssertNextFramesMatch_AdvancesBothContextsThroughFullPhases()
    {
        using PhysicsScenarioBuilder first = CreateScenario(PhysicsRuntimeMode.ThreeD);
        using PhysicsScenarioBuilder second = CreateScenario(PhysicsRuntimeMode.ThreeD);
        SolidBody firstBody = first.CreateSphere(Vector3d.Zero).Body;
        SolidBody secondBody = second.CreateSphere(Vector3d.Zero).Body;
        var firstCalls = new List<string>();
        var secondCalls = new List<string>();
        using IDisposable firstSimulate = first.Context.RegisterOnSimulate("simulate", 0, () =>
            firstCalls.Add($"simulate:{first.Context.FrameCount}:{first.Context.LateSimulateToken}"));
        using IDisposable firstLate = first.Context.RegisterOnLateSimulate("late", 0, () =>
            firstCalls.Add($"late:{first.Context.FrameCount}:{first.Context.LateSimulateToken}"));
        using IDisposable secondSimulate = second.Context.RegisterOnSimulate("simulate", 0, () =>
            secondCalls.Add($"simulate:{second.Context.FrameCount}:{second.Context.LateSimulateToken}"));
        using IDisposable secondLate = second.Context.RegisterOnLateSimulate("late", 0, () =>
            secondCalls.Add($"late:{second.Context.FrameCount}:{second.Context.LateSimulateToken}"));
        first.Context.Coroutines.StartCoroutine(ApplyForces(first.Context, firstBody, firstCalls));
        second.Context.Coroutines.StartCoroutine(ApplyForces(second.Context, secondBody, secondCalls));

        ReplayConformanceHarness.AssertNextFramesMatch(first.Context, second.Context, 3);

        string[] expected =
        {
            "coroutine:1:0", "simulate:1:0", "late:1:1",
            "coroutine:2:1", "simulate:2:1", "late:2:2",
            "coroutine:3:2", "simulate:3:2", "late:3:3"
        };
        Assert.Equal(expected, firstCalls);
        Assert.Equal(expected, secondCalls);
        Assert.Equal(3, first.Context.FrameCount);
        Assert.Equal(3, second.Context.FrameCount);
        Assert.Equal(new ChronicleTimestamp(0, 3u << 29), first.Context.ElapsedTime);
        Assert.Equal(first.Context.ElapsedTime, second.Context.ElapsedTime);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero), firstBody.Position3d);
        Assert.Equal(firstBody.Position3d, secondBody.Position3d);
        Assert.Equal(Vector3d.Right * (Fixed64)3, firstBody.LinearVelocity);
        Assert.Equal(firstBody.LinearVelocity, secondBody.LinearVelocity);

        static IEnumerator<ILockedYieldInstruction> ApplyForces(
            GravitasWorldContext context, SolidBody body, List<string> calls)
        {
            for (int frame = 0; frame < 3; frame++)
            {
                calls.Add($"coroutine:{context.FrameCount}:{context.LateSimulateToken}");
                body.AddForce(Vector3d.Right * (Fixed64)8);
                yield return context.Coroutines.WaitForNextSimulate();
            }
        }
    }

    [Theory]
    [InlineData(PhysicsRuntimeMode.TwoD)]
    [InlineData(PhysicsRuntimeMode.ThreeD)]
    [InlineData(PhysicsRuntimeMode.Both)]
    [InlineData(PhysicsRuntimeMode.Mixed)]
    public void RunTrace_MatchesFullHostLoopWithMotionContactsAndPresentation(PhysicsRuntimeMode mode)
    {
        using PhysicsScenarioBuilder first = CreateScenario(mode);
        using PhysicsScenarioBuilder control = CreateScenario(mode);
        int[] contacts = new int[3];
        int[] controlContacts = new int[3];
        var (body3D, body2D) = Populate(first, mode, contacts);
        var (control3D, control2D) = Populate(control, mode, controlContacts);
        Vector3d initial3D = body3D?.Position3d ?? Vector3d.Zero;
        Vector2d initial2D = body2D?.Position ?? Vector2d.Zero;
        const int frameCount = 4;

        ReplayHashTrace actual = ReplayConformanceHarness.RunTrace(first.Context, frameCount, (_, _) =>
            ApplyCommand(body3D, body2D));
        var expected = new ChronicleHash[frameCount];
        for (int frame = 0; frame < frameCount; frame++)
        {
            ApplyCommand(control3D, control2D);
            control.Context.Simulate();
            control.Context.LateSimulate();
            expected[frame] = control.Context.ComputeReplayHash();
            control.Context.Visualize();
            control.Context.LateVisualize();
            Assert.Equal(expected[frame], control.Context.ComputeReplayHash());
        }

        if (body3D != null)
        {
            Assert.NotEqual(initial3D, body3D.Position3d);
            Assert.True(contacts[0] > 0, "The 3D trace must solve real contacts.");
        }
        if (body2D != null)
        {
            Assert.NotEqual(initial2D, body2D.Position);
            Assert.True(contacts[1] > 0, "The planar trace must solve real contacts.");
        }
        if (mode == PhysicsRuntimeMode.Mixed)
            Assert.True(contacts[2] > 0, "Mixed must exercise a cross-dimensional contact.");
        else
            Assert.Equal(0, contacts[2]);
        Assert.Equal(controlContacts, contacts);
        Assert.Equal(expected, actual.Hashes);
        Assert.Equal(frameCount, first.Context.FrameCount);
        Assert.Equal(frameCount, first.Context.LateSimulateToken);
        Assert.Equal(control3D?.Position3d, body3D?.Position3d);
        Assert.Equal(control2D?.Position, body2D?.Position);
        ChronicleHash authoritative = first.Context.ComputeReplayHash();
        first.Context.Visualize();
        first.Context.LateVisualize();
        Assert.Equal(authoritative, first.Context.ComputeReplayHash());
    }

    private static PhysicsScenarioBuilder CreateScenario(PhysicsRuntimeMode mode)
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(8);
        scenario.Context.Settings.RuntimeMode = mode;
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        return scenario;
    }

    private static (SolidBody? Body3D, SolidBody2D? Body2D) Populate(
        PhysicsScenarioBuilder scenario, PhysicsRuntimeMode mode, int[] contacts)
    {
        SolidBody? body3D = null;
        SolidBody2D? body2D = null;
        if (mode != PhysicsRuntimeMode.TwoD)
        {
            var mover = scenario.CreateSphere(new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)4));
            body3D = mover.Body;
            body3D.CanSetVisualPosition = true;
            mover.Collider.OnContactEnter += _ => contacts[0]++;
            mover.Collider.OnMixedContactEnter += _ => contacts[2]++;
            _ = scenario.CreateSphere(new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, (Fixed64)4), immovable: true);
        }
        if (mode != PhysicsRuntimeMode.ThreeD)
        {
            body2D = CreateCircle(scenario.Context, new Vector2d(Fixed64.Zero, (Fixed64)(-4)), BodyMotionType.Dynamic);
            body2D.Collider.OnContactEnter += _ => contacts[1]++;
            body2D.Collider.OnMixedContactEnter += _ => contacts[2]++;
            _ = CreateCircle(scenario.Context, new Vector2d(Fixed64.FromFraction(3, 4), (Fixed64)(-4)), BodyMotionType.Static);
        }
        if (mode == PhysicsRuntimeMode.Mixed)
            _ = scenario.CreateSphere(new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, (Fixed64)(-4)), immovable: true);
        return (body3D, body2D);
    }

    private static SolidBody2D CreateCircle(GravitasWorldContext context, Vector2d position, BodyMotionType motionType)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var body = new SolidBody2D(new TestMatterAgent(context, transform), new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, motionType: motionType);
        return body;
    }

    private static void ApplyCommand(SolidBody? body3D, SolidBody2D? body2D)
    {
        body3D?.AddForce(Vector3d.Right * (Fixed64)8);
        body2D?.AddForce(Vector2d.Right * (Fixed64)8);
    }
}
