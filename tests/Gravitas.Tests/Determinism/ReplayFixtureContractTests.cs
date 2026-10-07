using System;
using System.IO;
using System.Text.Json;
using System.Text.Json.Nodes;
using Xunit;
using Xunit.Sdk;

namespace Gravitas.Tests.Determinism;

public sealed class ReplayFixtureContractTests
{
    [Fact]
    public void Read_RejectsUnsupportedVersion()
    {
        ReplayFixture fixture = CreateFixture();
        fixture.Version = 2;
        InvalidDataException error = Assert.Throws<InvalidDataException>(() => ReadFixture(fixture));
        Assert.Contains("Version", error.Message);
    }

    [Fact]
    public void Read_RejectsOmittedVersion()
    {
        JsonObject json = JsonNode.Parse(ReplayFixture.Serialize(CreateFixture()))!.AsObject();
        json.Remove("Version");
        InvalidDataException error = Assert.Throws<InvalidDataException>(() => ReadJson(json.ToJsonString()));
        Assert.Contains("Version", error.Message);
    }

    [Theory]
    [InlineData("Name")]
    [InlineData("FrameCount")]
    [InlineData("FrameRate")]
    [InlineData("SolverIterations")]
    [InlineData("MaxSpeedRaw")]
    [InlineData("RuntimeMode")]
    [InlineData("Actors duplicate")]
    [InlineData("Dimension")]
    [InlineData("dimension mode")]
    [InlineData("Shape")]
    [InlineData("PositionRaw")]
    [InlineData("SizeRaw")]
    [InlineData("MassRaw")]
    [InlineData("MotionType")]
    [InlineData("Expected length")]
    [InlineData("FrameCount stamp")]
    [InlineData("LateToken")]
    [InlineData("Authoritative")]
    [InlineData("SolverCaches")]
    [InlineData("Bodies unknown")]
    [InlineData("Bodies duplicate")]
    [InlineData("RotationRaw")]
    [InlineData("VelocityRaw")]
    [InlineData("AngularVelocityRaw")]
    [InlineData("ToiIterations")]
    [InlineData("Commands range")]
    [InlineData("Commands order")]
    [InlineData("Operation")]
    [InlineData("Actor reference")]
    [InlineData("ValueRaw")]
    [InlineData("BallSocket reference")]
    [InlineData("2D impulse Y")]
    [InlineData("2D Box Y")]
    [InlineData("SolverCaches policy")]
    [InlineData("Restore caches")]
    public void Read_RejectsMalformedFixture(string defect)
    {
        ReplayFixture fixture = CreateFixture();
        fixture.Commands = new[] { new ReplayCommand { Frame = 0, Operation = "Sleep", Actor = 7 } };
        if (defect is "2D impulse Y" or "2D Box Y")
        {
            fixture.RuntimeMode = PhysicsRuntimeMode.Mixed;
            fixture.Actors = new[]
            {
                fixture.Actors[0],
                new ReplayActor { Id = 2, Dimension = "2D", Shape = "Sphere", PositionRaw = new long[3], MassRaw = 1L << 32 }
            };
        }
        switch (defect)
        {
            case "Name": fixture.Name = " "; break;
            case "FrameCount": fixture.FrameCount = 0; break;
            case "FrameRate": fixture.FrameRate = 0; break;
            case "SolverIterations": fixture.SolverIterations = 0; break;
            case "MaxSpeedRaw": fixture.MaxSpeedRaw = -1; break;
            case "RuntimeMode": fixture.RuntimeMode = PhysicsRuntimeMode.None; break;
            case "Actors duplicate": fixture.Actors = new[] { fixture.Actors[0], fixture.Actors[0] }; break;
            case "Dimension": fixture.Actors[0].Dimension = "4D"; break;
            case "dimension mode": fixture.RuntimeMode = PhysicsRuntimeMode.TwoD; break;
            case "Shape": fixture.Actors[0].Shape = "Mesh"; break;
            case "PositionRaw": fixture.Actors[0].PositionRaw = new long[2]; break;
            case "SizeRaw": fixture.Actors[0].Shape = "Box"; break;
            case "MassRaw": fixture.Actors[0].MassRaw = 0; break;
            case "MotionType": fixture.Actors[0].MotionType = (BodyMotionType)99; break;
            case "Expected length": fixture.Expected = Array.Empty<ReplayFrame>(); break;
            case "FrameCount stamp": fixture.Expected[1].FrameCount = 1; break;
            case "LateToken": fixture.Expected[1].LateToken = 1; break;
            case "Authoritative": fixture.Expected[0].Authoritative = "0123456789ABCDEF0123456789ABCDEF"; break;
            case "SolverCaches": fixture.Expected[0].SolverCaches = "123"; break;
            case "Bodies unknown": fixture.Expected[0].Bodies[0].Actor = 9; break;
            case "Bodies duplicate": fixture.Expected[0].Bodies = new[] { fixture.Expected[0].Bodies[0], fixture.Expected[0].Bodies[0] }; break;
            case "RotationRaw": fixture.Expected[0].Bodies[0].RotationRaw = new long[3]; break;
            case "VelocityRaw": fixture.Expected[0].Bodies[0].VelocityRaw = new long[2]; break;
            case "AngularVelocityRaw": fixture.Expected[0].Bodies[0].AngularVelocityRaw = new long[1]; break;
            case "ToiIterations": fixture.Expected[0].Bodies[0].ToiIterations = -1; break;
            case "Commands range": fixture.Commands[0].Frame = 3; break;
            case "Commands order": fixture.Commands = new[] { new ReplayCommand { Frame = 1, Actor = 7, Operation = "Sleep" }, fixture.Commands[0] }; break;
            case "Operation": fixture.Commands[0].Operation = "Teleport"; break;
            case "Actor reference": fixture.Commands[0].Actor = 9; break;
            case "ValueRaw": fixture.Commands[0].Operation = "Impulse"; break;
            case "BallSocket reference": fixture.Commands[0].Operation = "BallSocket"; fixture.Commands[0].OtherActor = 9; break;
            case "2D impulse Y": fixture.Commands[0].Actor = 2; fixture.Commands[0].Operation = "Impulse"; fixture.Commands[0].ValueRaw = new long[] { 0, 1, 0 }; break;
            case "2D Box Y": fixture.Actors[1].Shape = "Box"; fixture.Actors[1].SizeRaw = new long[] { 1L << 32, 4L << 32, 1L << 32 }; break;
            case "SolverCaches policy": fixture.Expected[1].SolverCaches = null; break;
            case "Restore caches": fixture.Commands[0].Operation = "Restore"; break;
            default: throw new ArgumentOutOfRangeException(nameof(defect));
        }

        Assert.Throws<InvalidDataException>(() => ReadFixture(fixture));
    }

    [Fact]
    public void Read_RejectsMisspelledField()
    {
        string json = ReplayFixture.Serialize(CreateFixture()).Replace("\"FrameRate\"", "\"FrameRat\"", StringComparison.Ordinal);
        Assert.Throws<InvalidDataException>(() => ReadJson(json));
    }

    [Fact]
    public void Read_PreservesIntegerPrecisionDimensionalStateAndOrderedCommands()
    {
        ReplayFixture fixture = CreateFixture();
        fixture.RuntimeMode = PhysicsRuntimeMode.Mixed;
        fixture.Actors[0].PositionRaw[0] = long.MaxValue;
        fixture.Actors = new[]
        {
            fixture.Actors[0],
            new ReplayActor { Id = 2, Dimension = "2D", Shape = "Box", PositionRaw = new long[3], SizeRaw = new long[] { 1L << 32, 1L << 32, 1L << 32 }, MassRaw = 1L << 32 }
        };
        fixture.Commands = new[]
        {
            new ReplayCommand { Frame = 0, Actor = 7, Operation = "Impulse", ValueRaw = new long[] { long.MinValue, 0, 0 } },
            new ReplayCommand { Frame = 0, Actor = 7, Operation = "Query" }
        };
        foreach (ReplayFrame frame in fixture.Expected)
            frame.SolverCaches = null;
        fixture.Expected[0].Bodies = new[]
        {
            fixture.Expected[0].Bodies[0],
            new ReplayBodyState { Actor = 2, PositionRaw = new long[2], RotationRaw = new long[1], VelocityRaw = new long[2], AngularVelocityRaw = new long[1] }
        };
        fixture.Expected[0].Events = new[] { "enter:7:2", "exit:7:2" };

        ReplayFixture read = ReadFixture(fixture);
        Assert.Equal(long.MaxValue, read.Actors[0].PositionRaw[0]);
        Assert.Equal(long.MinValue, read.Commands[0].ValueRaw[0]);
        Assert.Equal(new[] { "Impulse", "Query" }, Array.ConvertAll(read.Commands, command => command.Operation));
        Assert.Equal(new[] { 7, 2 }, Array.ConvertAll(read.Expected[0].Bodies, body => body.Actor));
        Assert.Equal(new[] { "enter:7:2", "exit:7:2" }, read.Expected[0].Events);
        Assert.Null(read.Expected[0].SolverCaches);
        Assert.True(read.Actors[0].StartActive);
        using JsonDocument json = JsonDocument.Parse(ReplayFixture.Serialize(read));
        Assert.Equal("Mixed", json.RootElement.GetProperty("RuntimeMode").GetString());
        ReplayFixture.AssertFrame(read, 0, read.Expected[0]);
    }

    [Fact]
    public void Read_AcceptsRestoreWithAuthoritativeOnlyFrames()
    {
        ReplayFixture fixture = CreateFixture();
        fixture.Commands = new[] { new ReplayCommand { Frame = 1, Actor = 7, Operation = "Restore" } };
        foreach (ReplayFrame frame in fixture.Expected)
            frame.SolverCaches = null;

        ReplayFixture read = ReadFixture(fixture);
        Assert.Equal("Restore", read.Commands[0].Operation);
        Assert.All(read.Expected, frame => Assert.Null(frame.SolverCaches));
    }

    [Fact]
    public void AssertFrame_ReportsFirstDivergenceWithPrecedingCommandsAndRawState()
    {
        ReplayFixture fixture = CreateFixture();
        ReplayFrame actual = CreateFrame(2);
        actual.Bodies[0].VelocityRaw[0] = 123;
        actual.Events = new[] { "contact:7:9", "exit:7:9" };
        fixture.Commands = new[]
        {
            new ReplayCommand { Frame = 0, Operation = "Impulse", Actor = 7, ValueRaw = new long[] { 11, 0, 0 } },
            new ReplayCommand { Frame = 1, Operation = "Sleep", Actor = 7 },
            new ReplayCommand { Frame = 2, Operation = "Wake", Actor = 7 }
        };

        XunitException error = Assert.Throws<XunitException>(() => ReplayFixture.AssertFrame(fixture, 1, actual));
        Assert.Contains("contract-test", error.Message);
        Assert.Contains("version 1", error.Message);
        Assert.Contains("mode ThreeD", error.Message);
        Assert.Contains("frame 2", error.Message);
        Assert.Contains("Impulse", error.Message);
        Assert.Contains("Sleep", error.Message);
        Assert.DoesNotContain("Wake", error.Message);
        Assert.Contains("Expected", error.Message);
        Assert.Contains("Actual", error.Message);
        Assert.Contains("VelocityRaw", error.Message);
        Assert.Contains("123", error.Message);
        Assert.True(error.Message.IndexOf("contact:7:9", StringComparison.Ordinal) < error.Message.IndexOf("exit:7:9", StringComparison.Ordinal));
    }

    private static ReplayFixture ReadFixture(ReplayFixture fixture)
        => ReadJson(ReplayFixture.Serialize(fixture));

    private static ReplayFixture ReadJson(string json)
    {
        string path = Path.GetTempFileName();
        try
        {
            File.WriteAllText(path, json);
            return ReplayFixture.Read(path);
        }
        finally
        {
            File.Delete(path);
        }
    }

    private static ReplayFixture CreateFixture() => new()
    {
        Version = 1,
        Name = "contract-test",
        RuntimeMode = PhysicsRuntimeMode.ThreeD,
        FrameCount = 3,
        FrameRate = 8,
        SolverIterations = 4,
        MaxSpeedRaw = 64L << 32,
        Actors = new[]
        {
            new ReplayActor
            {
                Id = 7, Dimension = "3D", Shape = "Sphere", PositionRaw = new long[3],
                MassRaw = 1L << 32, MotionType = BodyMotionType.Dynamic
            }
        },
        Expected = new[] { CreateFrame(1), CreateFrame(2), CreateFrame(3) }
    };

    private static ReplayFrame CreateFrame(long frame) => new()
    {
        FrameCount = frame,
        LateToken = frame,
        Authoritative = "0123456789abcdef0123456789abcdef",
        SolverCaches = "fedcba9876543210fedcba9876543210",
        Bodies = new[]
        {
            new ReplayBodyState
            {
                Actor = 7, PositionRaw = new long[3], RotationRaw = new long[] { 0, 0, 0, 1L << 32 },
                VelocityRaw = new long[3], AngularVelocityRaw = new long[3]
            }
        }
    };
}
