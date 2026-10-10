using FixedMathSharp;
using Gravitas.Support;
using System;
using System.IO;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.Determinism;

public sealed class SharedReplayFixtureTests
{
    [Fact]
    public void SharedFixture_ShouldIgnoreAmbientLayerDisplayNames()
    {
        ReplayFixture fixture = ReplayFixture.Read(Path.Combine(AppContext.BaseDirectory,
            "Determinism", "Fixtures", "three-d-caches-v1.json"));
        var originalNames = PhysicsLayer.LayerNamesCache;
        try
        {
            PhysicsLayer.LayerNamesCache = new();
            using var unnamed = new ReplayFixtureRunner(fixture);
            ReplayFrame[] baseline = unnamed.Run();
            _ = new PhysicsLayer(0, "Fixture default");
            _ = new PhysicsLayer(1, "Unrelated host layer");
            using var named = new ReplayFixtureRunner(fixture);
            ReplayFrame[] afterNames = named.Run();
            for (int frame = 0; frame < baseline.Length; frame++)
            {
                ReplayFixture.AssertFrame(fixture, frame, baseline[frame]);
                ReplayFixture.AssertFrame(fixture, frame, afterNames[frame]);
            }
        }
        finally
        {
            PhysicsLayer.LayerNamesCache = originalNames;
        }
    }

    [Theory]
    [InlineData("two-d-lifecycle-v1")]
    [InlineData("three-d-lifecycle-v1")]
    [InlineData("both-lifecycle-v1")]
    [InlineData("mixed-lifecycle-v1")]
    [InlineData("three-d-caches-v1")]
    [InlineData("three-d-surface-contacts-v1")]
    public void OrderedCommands_MatchSharedExpectedFramesAndRepeatedRuns(string name)
    {
        ReplayFixture fixture = ReplayFixture.Read(Path.Combine(AppContext.BaseDirectory, "Determinism", "Fixtures", name + ".json"));
        using var first = new ReplayFixtureRunner(fixture);
        ReplayFrame[] actual = first.Run();
        // Capture before comparison so a deliberately failing new fixture still
        // leaves observations for review. It never writes expected values or
        // bypasses their comparison; installing expectations is a separate review.
        string? captureDirectory = Environment.GetEnvironmentVariable("GRAVITAS_REPLAY_CAPTURE_DIRECTORY");
        if (captureDirectory != null)
            ReplayCapture.Write(captureDirectory, fixture, actual);
        for (int frame = 0; frame < actual.Length; frame++)
            ReplayFixture.AssertFrame(fixture, frame, actual[frame]);

        AssertScenarioSemantics(fixture, actual, first);

        using var repeated = new ReplayFixtureRunner(fixture);
        ReplayFrame[] repeatedFrames = repeated.Run();
        for (int frame = 0; frame < repeatedFrames.Length; frame++)
            ReplayFixture.AssertFrame(fixture, frame, repeatedFrames[frame]);
        if (fixture.Commands.Any(command => command.Operation == "Restore"))
        {
            // Same host timeline and commands, but a reference free body continues
            // uninterrupted. Body populate does not promise solver-cache restore.
            using var uninterrupted = new ReplayFixtureRunner(fixture, skipRestore: true);
            ReplayFrame[] reference = uninterrupted.Run();
            for (int frame = 0; frame < actual.Length; frame++)
            {
                Assert.Null(actual[frame].SolverCaches);
                ReplayFixture.AssertFrame(fixture, frame, reference[frame]);
            }
        }
    }

    private static void AssertScenarioSemantics(ReplayFixture fixture, ReplayFrame[] frames, ReplayFixtureRunner runner)
    {
        if (fixture.Name == "three-d-surface-contacts-v1")
        {
            ReplayBodyState walls = Body(frames[0], 81);
            Assert.True(walls.VelocityRaw[0] >= 0 && walls.VelocityRaw[2] >= 0,
                "The combined mesh must block approach into both independent walls.");
            ReplayBodyState tab = Body(frames[0], 91);
            Assert.True(tab.VelocityRaw[1] <= 0,
                "The nonconvex square/tab face must stop upward approach.");
            Assert.Contains(frames[0].Events, value => value == "enter:81:80" || value == "enter:80:81");
            Assert.Contains(frames[0].Events, value => value == "enter:91:90" || value == "enter:90:91");
        }
        foreach (ReplayActor actor in fixture.Actors.Where(actor => actor.Continuous))
        {
            ReplayBodyState impact = Body(frames[0], actor.Id);
            // At 8 Hz the mass-one impulse requests four units from X=-2, crossing
            // the wall at X=1. Contact center is 1 - wall half-width 1/16
            // - sphere/circle radius 1/2 = 7/16, with no restitution.
            Assert.True(impact.ToiIterations > 0, $"Actor {actor.Id} must exercise a CCD impact.");
            Assert.Equal(Fixed64.FromFraction(7, 16).m_rawValue, impact.PositionRaw[0]);
            Assert.Equal(0, impact.VelocityRaw[0]);
            if (actor.Dimension == "3D")
            {
                Assert.Equal(actor.PositionRaw[1], impact.PositionRaw[1]);
                Assert.Equal(actor.PositionRaw[2], impact.PositionRaw[2]);
            }
            else
                Assert.Equal(actor.PositionRaw[2], impact.PositionRaw[1]);
            Assert.Contains(frames.SelectMany(frame => frame.Events), value =>
                value == $"enter:{actor.Id}:{actor.Id + 10}"
                || value == $"mixed-enter:{actor.Id}:{actor.Id + 10}");
        }

        ReplayCommand[] spawns = fixture.Commands.Where(command => command.Operation == "Spawn").ToArray();
        foreach (ReplayCommand spawn in spawns)
            Assert.True(Body(frames[spawn.Frame], spawn.Actor).Sleeping);
        foreach (ReplayCommand despawn in fixture.Commands.Where(command => command.Operation == "Despawn"))
        {
            Assert.DoesNotContain(frames[despawn.Frame].Bodies, body => body.Actor == despawn.Actor);
            Assert.Contains($"query:{despawn.Actor}:none", frames[despawn.Frame].Queries);
        }
        foreach (ReplayCommand command in fixture.Commands.Where(command => command.Operation == "Impulse"
            && spawns.Any(spawn => spawn.Actor == command.Actor)))
        {
            ReplayBodyState awake = Body(frames[command.Frame], command.Actor);
            Assert.False(awake.Sleeping);
            Assert.Equal(Fixed64.Eighth.m_rawValue, awake.PositionRaw[0]);
            Assert.Equal(Fixed64.One.m_rawValue, awake.VelocityRaw[0]);
        }
        Assert.Equal(fixture.Commands.Count(command => command.Operation == "Despawn"), runner.ReusedSlots);
        foreach (ReplayCommand query in fixture.Commands.Where(command => command.Operation == "Query"))
        {
            bool retired = fixture.Commands.Any(command => command.Actor == query.Actor
                && command.Operation == "Despawn" && command.Frame <= query.Frame);
            Assert.Contains($"query:{query.Actor}:{(retired ? "none" : query.Actor.ToString(System.Globalization.CultureInfo.InvariantCulture))}",
                frames[query.Frame].Queries);
        }
        foreach (ReplayCommand restore in fixture.Commands.Where(command => command.Operation == "Restore"))
            for (int frame = 0; frame < frames.Length; frame++)
            {
                // Free mass-one bodies move one eighth-unit each full step.
                ReplayBodyState free = Body(frames[frame], restore.Actor);
                Assert.Equal(Fixed64.FromFraction(frame + 1, 8).m_rawValue, free.PositionRaw[0]);
                Assert.Equal(Fixed64.One.m_rawValue, free.VelocityRaw[0]);
            }
        if (runner.Joints.Count > 0)
        {
            Assert.All(runner.Joints, joint => Assert.True(joint.LastSolvedRowCount > 0));
            Assert.Contains(frames.SelectMany(frame => frame.Events), value => value == "enter:60:70" || value == "enter:70:60");
            // Equal masses, no external force: the connected pair retains the
            // initial quarter-unit Z momentum while exchanging internal impulses.
            foreach (ReplayFrame frame in frames)
            {
                ReplayBodyState a = Body(frame, 60);
                ReplayBodyState b = Body(frame, 70);
                Assert.Equal(Fixed64.FromFraction(3, 4).m_rawValue, a.PositionRaw[0] + b.PositionRaw[0]);
                Assert.Equal(0, a.VelocityRaw[0] + b.VelocityRaw[0]);
                Assert.Equal(Fixed64.Quarter.m_rawValue, a.VelocityRaw[2] + b.VelocityRaw[2]);
            }
        }
        if (fixture.RuntimeMode == PhysicsRuntimeMode.Mixed)
            Assert.Contains(frames.SelectMany(frame => frame.Events), value => value.StartsWith("mixed-enter:", StringComparison.Ordinal));
        if (fixture.RuntimeMode == PhysicsRuntimeMode.Both)
            Assert.DoesNotContain(frames.SelectMany(frame => frame.Events), value => value.StartsWith("mixed-", StringComparison.Ordinal));
    }

    private static ReplayBodyState Body(ReplayFrame frame, int id) => Assert.Single(frame.Bodies, body => body.Actor == id);
}
