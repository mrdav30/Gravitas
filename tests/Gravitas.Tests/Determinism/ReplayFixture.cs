using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.IO;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit.Sdk;

namespace Gravitas.Tests.Determinism;

internal sealed class ReplayFixture
{
    private static readonly JsonSerializerOptions Options = new()
    {
        WriteIndented = true,
        UnmappedMemberHandling = JsonUnmappedMemberHandling.Disallow,
        Converters = { new JsonStringEnumConverter() }
    };

    public int Version { get; set; }
    public string Name { get; set; } = string.Empty;
    public PhysicsRuntimeMode RuntimeMode { get; set; }
    public int FrameCount { get; set; }
    public int FrameRate { get; set; }
    public int SolverIterations { get; set; }
    public long MaxSpeedRaw { get; set; }
    public ReplayActor[] Actors { get; set; } = Array.Empty<ReplayActor>();
    public ReplayCommand[] Commands { get; set; } = Array.Empty<ReplayCommand>();
    public ReplayFrame[] Expected { get; set; } = Array.Empty<ReplayFrame>();

    public static ReplayFixture Read(string path)
    {
        ReplayFixture fixture;
        try
        {
            fixture = JsonSerializer.Deserialize<ReplayFixture>(File.ReadAllText(path), Options)
                ?? throw new InvalidDataException($"Fixture '{path}' must be a JSON object.");
        }
        catch (JsonException error)
        {
            throw new InvalidDataException($"Invalid fixture '{path}': {error.Message}", error);
        }
        fixture.Validate();
        return fixture;
    }

    public static string Serialize(object value) => JsonSerializer.Serialize(value, Options);

    // Frame and command indices are zero-based; recorded clock stamps start at one.
    public static void AssertFrame(ReplayFixture fixture, int frame, ReplayFrame actual)
    {
        string expectedJson = Serialize(fixture.Expected[frame]);
        string actualJson = Serialize(actual);
        if (string.Equals(expectedJson, actualJson, StringComparison.Ordinal))
            return;

        throw new XunitException(
            $"Fixture '{fixture.Name}' version {fixture.Version}, mode {fixture.RuntimeMode}: first divergent frame {frame + 1} (index {frame}).\n"
            + "Preceding commands (including this frame):\n"
            + Serialize(Array.FindAll(fixture.Commands, command => command.Frame <= frame))
            + "\nExpected raw state and ordered events/queries:\n" + expectedJson
            + "\nActual raw state and ordered events/queries:\n" + actualJson);
    }

    private void Validate()
    {
        Require(Version == 1, "Version must be 1.");
        Require(!string.IsNullOrWhiteSpace(Name), "Name must be nonempty.");
        Require(FrameCount > 0, "FrameCount must be positive.");
        Require(FrameRate > 0, "FrameRate must be positive.");
        Require(SolverIterations > 0, "SolverIterations must be positive.");
        Require(MaxSpeedRaw > 0, "MaxSpeedRaw must be positive.");
        Require(RuntimeMode is PhysicsRuntimeMode.TwoD or PhysicsRuntimeMode.ThreeD or PhysicsRuntimeMode.Both or PhysicsRuntimeMode.Mixed,
            "RuntimeMode must be TwoD, ThreeD, Both, or Mixed.");
        Require(Actors != null && Commands != null && Expected != null, "Actors, Commands, and Expected must be arrays.");
        Require(Expected.Length == FrameCount, "Expected length must equal FrameCount.");
        bool includeSolverCaches = Expected[0]?.SolverCaches != null;

        var actors = new Dictionary<int, (ReplayActor Definition, int Order)>();
        for (int index = 0; index < Actors.Length; index++)
        {
            ReplayActor actor = Actors[index];
            Require(actor != null, "Actors cannot contain null.");
            Require(actors.TryAdd(actor.Id, (actor, index)), $"Actors contains duplicate Id {actor.Id}.");
            Require(actor.Dimension is "3D" or "2D", $"Actor {actor.Id} Dimension must be 3D or 2D.");
            Require(actor.Dimension == "3D" ? RuntimeMode != PhysicsRuntimeMode.TwoD : RuntimeMode != PhysicsRuntimeMode.ThreeD,
                $"Actor {actor.Id} Dimension is unsupported by RuntimeMode.");
            Require(actor.Shape is "Sphere" or "Box", $"Actor {actor.Id} Shape must be Sphere or Box.");
            Require(actor.PositionRaw is { Length: 3 }, $"Actor {actor.Id} PositionRaw must contain X/Y/Z.");
            Require(actor.SizeRaw != null && actor.SizeRaw.Length == (actor.Shape == "Box" ? 3 : 0),
                $"Actor {actor.Id} SizeRaw must have three entries for Box and none for Sphere.");
            foreach (long size in actor.SizeRaw)
                Require(size > 0, $"Actor {actor.Id} SizeRaw entries must be positive.");
            // Pure 2D boxes use X/Z size; Y is a canonical unused placeholder, not mixed thickness.
            if (actor.Dimension == "2D" && actor.Shape == "Box")
                Require(actor.SizeRaw[1] == 1L << 32, $"Actor {actor.Id} 2D Box SizeRaw Y must be fixed-point one.");
            Require(actor.MassRaw > 0, $"Actor {actor.Id} MassRaw must be positive.");
            Require(actor.MotionType is BodyMotionType.Dynamic or BodyMotionType.Kinematic or BodyMotionType.Static,
                $"Actor {actor.Id} MotionType is unsupported.");
        }

        int previousFrame = -1;
        foreach (ReplayCommand command in Commands)
        {
            Require(command != null, "Commands cannot contain null.");
            Require(command.Frame >= 0 && command.Frame < FrameCount && command.Frame >= previousFrame,
                "Commands must be ordered by Frame with indices inside FrameCount.");
            previousFrame = command.Frame;
            Require(command.Operation is "Impulse" or "Sleep" or "Wake" or "Spawn" or "Despawn" or "Query" or "Restore" or "BallSocket",
                $"Command Operation '{command.Operation}' is unsupported.");
            Require(actors.ContainsKey(command.Actor), $"Command Actor {command.Actor} is undefined.");
            Require(command.ValueRaw != null && command.ValueRaw.Length == (command.Operation == "Impulse" ? 3 : 0),
                "Command ValueRaw must contain X/Y/Z for Impulse and be empty otherwise.");
            if (command.Operation == "Impulse" && actors[command.Actor].Definition.Dimension == "2D")
                Require(command.ValueRaw[1] == 0, "2D Impulse ValueRaw Y must be zero.");
            if (command.Operation == "Restore")
                Require(!includeSolverCaches, "Restore fixtures must use authoritative-only frames without SolverCaches.");
            if (command.Operation == "BallSocket")
            {
                Require(actors.ContainsKey(command.OtherActor), $"BallSocket OtherActor {command.OtherActor} is undefined.");
                Require(command.Actor != command.OtherActor
                    && actors[command.Actor].Definition.Dimension == "3D"
                    && actors[command.OtherActor].Definition.Dimension == "3D",
                    "BallSocket requires two distinct 3D actors.");
            }
        }

        for (int index = 0; index < Expected.Length; index++)
        {
            ReplayFrame frame = Expected[index];
            Require(frame != null, "Expected cannot contain null.");
            Require(frame.FrameCount == index + 1, $"Expected[{index}] FrameCount must be {index + 1}.");
            Require(frame.LateToken == index + 1, $"Expected[{index}] LateToken must be {index + 1}.");
            Require(IsCanonicalHash(frame.Authoritative), $"Expected[{index}] Authoritative must be 32 lowercase hexadecimal characters.");
            Require(frame.SolverCaches == null || IsCanonicalHash(frame.SolverCaches),
                $"Expected[{index}] SolverCaches must be null or 32 lowercase hexadecimal characters.");
            Require((frame.SolverCaches != null) == includeSolverCaches, "SolverCaches presence must be uniform across all expected frames.");
            Require(frame.Bodies != null && frame.Events != null && frame.Queries != null,
                $"Expected[{index}] Bodies, Events, and Queries must be arrays.");
            int previousActorOrder = -1;
            foreach (ReplayBodyState body in frame.Bodies)
            {
                Require(body != null, "Bodies cannot contain null.");
                Require(actors.ContainsKey(body.Actor), $"Body Actor {body.Actor} is undefined.");
                var actor = actors[body.Actor];
                Require(actor.Order > previousActorOrder, "Bodies must be unique and follow fixture actor order.");
                previousActorOrder = actor.Order;
                bool is3D = actor.Definition.Dimension == "3D";
                Require(body.PositionRaw != null && body.PositionRaw.Length == (is3D ? 3 : 2), $"Body {body.Actor} PositionRaw has invalid dimensional length.");
                Require(body.RotationRaw != null && body.RotationRaw.Length == (is3D ? 4 : 1), $"Body {body.Actor} RotationRaw has invalid dimensional length.");
                Require(body.VelocityRaw != null && body.VelocityRaw.Length == (is3D ? 3 : 2), $"Body {body.Actor} VelocityRaw has invalid dimensional length.");
                Require(body.AngularVelocityRaw != null && body.AngularVelocityRaw.Length == (is3D ? 3 : 1), $"Body {body.Actor} AngularVelocityRaw has invalid dimensional length.");
                Require(body.ToiIterations >= 0, $"Body {body.Actor} ToiIterations cannot be negative.");
            }
        }
    }

    private static bool IsCanonicalHash(string? value)
    {
        if (value is not { Length: 32 })
            return false;
        foreach (char character in value)
            if (character is not (>= '0' and <= '9') and not (>= 'a' and <= 'f'))
                return false;
        return true;
    }

    private static void Require([DoesNotReturnIf(false)] bool valid, string message)
    {
        if (!valid)
            throw new InvalidDataException(message);
    }
}

internal sealed class ReplayActor
{
    public int Id { get; set; }
    public string Dimension { get; set; } = string.Empty;
    public string Shape { get; set; } = string.Empty;
    public long[] PositionRaw { get; set; } = Array.Empty<long>();
    public long[] SizeRaw { get; set; } = Array.Empty<long>();
    public long MassRaw { get; set; }
    public BodyMotionType MotionType { get; set; }
    public bool Continuous { get; set; }
    public bool StartActive { get; set; } = true;
}

internal sealed class ReplayCommand
{
    public int Frame { get; set; }
    public string Operation { get; set; } = string.Empty;
    public int Actor { get; set; }
    public int OtherActor { get; set; }
    public long[] ValueRaw { get; set; } = Array.Empty<long>();
}

internal sealed class ReplayFrame
{
    public long FrameCount { get; set; }
    public long LateToken { get; set; }
    public string Authoritative { get; set; } = string.Empty;
    public string? SolverCaches { get; set; }
    public ReplayBodyState[] Bodies { get; set; } = Array.Empty<ReplayBodyState>();
    public string[] Events { get; set; } = Array.Empty<string>();
    public string[] Queries { get; set; } = Array.Empty<string>();
}

internal sealed class ReplayBodyState
{
    public int Actor { get; set; }
    public long[] PositionRaw { get; set; } = Array.Empty<long>();
    public long[] RotationRaw { get; set; } = Array.Empty<long>();
    public long[] VelocityRaw { get; set; } = Array.Empty<long>();
    public long[] AngularVelocityRaw { get; set; } = Array.Empty<long>();
    public bool Sleeping { get; set; }
    public int ToiIterations { get; set; }
}
