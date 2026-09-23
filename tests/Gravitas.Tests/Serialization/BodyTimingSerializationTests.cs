using Chronicler;
using Chronicler.Timing;
using FixedMathSharp;
using Gravitas.Tests.Support;
using System;
using System.Reflection;
using Xunit;

namespace Gravitas.Tests.Serialization;

public sealed class BodyTimingSerializationTests
{
    public static TheoryData<GravitasSerializationTransport> Transports => GravitasSerializationTransportCases.All();

    [Theory]
    [MemberData(nameof(Transports))]
    public void GroundingFrameRoundTripsBeyondIntRange(GravitasSerializationTransport transport)
    {
        using var source = PhysicsScenarioBuilder.Create();
        using var target = PhysicsScenarioBuilder.Create();
        long frame = (long)int.MaxValue + 17;
        TimingTestUtility.SetClock(source.Context, frame, new ChronicleTimestamp(100, 0));
        TimingTestUtility.SetClock(target.Context, frame, new ChronicleTimestamp(100, 0));
        var sourceBody = source.CreateSphere(Vector3d.Zero).Body;
        var targetBody = target.CreateSphere(Vector3d.Right).Body;
        sourceBody.CheckGround();
        object payload = GravitasSerializationHarness.Serialize(sourceBody, transport);
        GravitasSerializationHarness.Populate(targetBody, payload, transport);
        Assert.Equal(frame, ReadGroundingFrame(targetBody));
        Assert.Equal(source.Context.ComputeReplayHash(), target.Context.ComputeReplayHash());
        source.Context.Simulate();
        target.Context.Simulate();
        source.Context.LateSimulate();
        target.Context.LateSimulate();
        Assert.Equal(source.Context.ComputeReplayHash(), target.Context.ComputeReplayHash());
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void ObsoleteOrInvalidTimingRecordRejectsBeforeBodyMutation(GravitasSerializationTransport transport)
    {
        foreach (int? schema in new int?[] { null, 0, 2, 1 })
        {
            using var scenario = PhysicsScenarioBuilder.Create();
            var body = scenario.CreateSphere(Vector3d.Right).Body;
            var before = scenario.Context.ComputeReplayHash();
            object payload = GravitasSerializationHarness.Serialize(new TimingRecord(schema), transport);
            Assert.Throws<InvalidOperationException>(() => GravitasSerializationHarness.Populate(body, payload, transport));
            Assert.Equal(before, scenario.Context.ComputeReplayHash());
            Assert.Equal(Vector3d.Right, body.PositionTransform.WorldPosition);
        }
    }

    private static long ReadGroundingFrame(SolidBody body) =>
        (long)typeof(SolidBody).GetField("_lastGroundCheckFrame", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(body)!;

    private sealed class TimingRecord : IRecordable
    {
        private readonly int? _schema;

        internal TimingRecord(int? schema) => _schema = schema;

        public void RecordData(IChronicler chronicler)
        {
            if (_schema.HasValue)
            {
                int version = _schema.Value;
                RecordValues.Look(chronicler, ref version, "BodySchemaVersion", 0);
            }
            if (_schema == 1)
            {
                long invalidFrame = -2;
                RecordValues.Look(chronicler, ref invalidFrame, "LastGroundCheckFrame");
            }
            else
            {
                int oldFrame = 7;
                RecordValues.Look(chronicler, ref oldFrame, "LastGroundCheckFrame");
            }
            Vector2d position = new(12, 13);
            RecordValues.Look(chronicler, ref position, "Position2d");
        }
    }
}
