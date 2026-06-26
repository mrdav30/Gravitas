using FixedMathSharp;
using FluentAssertions;
using Gravitas.Support;
using Xunit;

namespace Gravitas.Tests.Determinism;

public sealed class GravitasReplayHashWriterTests
{
    [Fact]
    public void Writer_ShouldProduceSameHashForSamePrimitiveSequence()
    {
        GravitasReplayHash first = WritePrimitiveSequence();
        GravitasReplayHash second = WritePrimitiveSequence();

        second.Should().Be(first);
    }

    [Fact]
    public void Writer_ShouldChangeHashWhenSectionOrderChanges()
    {
        var first = new GravitasReplayHashWriter();
        first.WriteSection("first", 1);
        first.WriteInt32(7);
        first.WriteSection("second", 1);
        first.WriteInt32(11);

        var second = new GravitasReplayHashWriter();
        second.WriteSection("second", 1);
        second.WriteInt32(11);
        second.WriteSection("first", 1);
        second.WriteInt32(7);

        second.ToHash().Should().NotBe(first.ToHash());
    }

    [Fact]
    public void Writer_ShouldChangeHashWhenVectorOrQuaternionComponentChanges()
    {
        var first = new GravitasReplayHashWriter();
        first.WriteSection("shape", 1);
        first.WriteVector3d(new Vector3d(Fixed64.One, Fixed64.Two, Fixed64.Three));
        first.WriteQuaternion(FixedQuaternion.Identity);

        var second = new GravitasReplayHashWriter();
        second.WriteSection("shape", 1);
        second.WriteVector3d(new Vector3d(Fixed64.One, Fixed64.Two, Fixed64.FromRaw(Fixed64.Three.m_rawValue + 1)));
        second.WriteQuaternion(FixedQuaternion.Identity);

        second.ToHash().Should().NotBe(first.ToHash());
    }

    [Fact]
    public void Writer_ShouldHashFixed64RawPayloadNotTruncatedHashCode()
    {
        Fixed64 value = Fixed64.FromRaw(unchecked((long)0x7fff_ffff_0000_0001UL));

        var direct = new GravitasReplayHashWriter();
        direct.WriteSection("fixed", 1);
        direct.WriteFixed64(value);

        var truncated = new GravitasReplayHashWriter();
        truncated.WriteSection("fixed", 1);
        truncated.WriteInt32(value.GetHashCode());

        direct.ToHash().Should().NotBe(truncated.ToHash());
    }

    private static GravitasReplayHash WritePrimitiveSequence()
    {
        var writer = new GravitasReplayHashWriter();
        writer.WriteSection("primitive", 1);
        writer.WriteBool(true);
        writer.WriteByte(17);
        writer.WriteInt32(-21);
        writer.WriteUInt32(22);
        writer.WriteInt64(-23);
        writer.WriteUInt64(24);
        writer.WriteEnum(PhysicsRuntimeMode.Mixed);
        writer.WriteFixed64(Fixed64.FromFraction(3, 8));
        writer.WriteVector2d(new Vector2d(Fixed64.One, Fixed64.Two));
        writer.WriteVector3d(new Vector3d(Fixed64.One, Fixed64.Two, Fixed64.Three));
        writer.WriteVector4d(new Vector4d(Fixed64.One, Fixed64.Two, Fixed64.Three, Fixed64.Half));
        writer.WriteQuaternion(FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.One, Fixed64.Two, Fixed64.Three));
        writer.WriteTransform(new FixedTransform(
            new Vector3d(Fixed64.One, Fixed64.Two, Fixed64.Three),
            FixedQuaternion.Identity,
            Vector3d.One));
        writer.WritePhysicsLayer(new PhysicsLayer(3));
        writer.WritePhysicsLayerMask(PhysicsLayerMask.FromLayer(new PhysicsLayer(4)));
        return writer.ToHash();
    }
}
