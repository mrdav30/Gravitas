using Chronicler;
using FluentAssertions;
using Gravitas.Constraints;
using Gravitas.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Determinism;

public sealed class GravitasChronicleHashWriterExtensionsTests
{
    [Fact]
    public void WritePhysicsLayer_ShouldUseLayerIndexPayload()
    {
        ChronicleHash layerHash = Hash((ref ChronicleHashWriter writer) =>
            writer.WritePhysicsLayer(new PhysicsLayer(3)));
        ChronicleHash indexHash = Hash((ref ChronicleHashWriter writer) =>
            writer.WriteInt32(3));

        layerHash.Should().Be(indexHash);
    }

    [Fact]
    public void WritePhysicsLayerMask_ShouldUseMaskBitsPayload()
    {
        PhysicsLayerMask mask = PhysicsLayerMask.FromLayer(new PhysicsLayer(4));

        ChronicleHash maskHash = Hash((ref ChronicleHashWriter writer) =>
            writer.WritePhysicsLayerMask(mask));
        ChronicleHash bitsHash = Hash((ref ChronicleHashWriter writer) =>
            writer.WriteInt32(mask.Bits));

        maskHash.Should().Be(bitsHash);
    }

    [Fact]
    public void Writer_ShouldChangeHashWhenSectionOrderChanges()
    {
        var first = new ChronicleHashWriter();
        first.WriteSection("first", 1);
        first.WriteInt32(7);
        first.WriteSection("second", 1);
        first.WriteInt32(11);

        var second = new ChronicleHashWriter();
        second.WriteSection("second", 1);
        second.WriteInt32(11);
        second.WriteSection("first", 1);
        second.WriteInt32(7);

        second.ToHash().Should().NotBe(first.ToHash());
    }

    [Fact]
    public void ChronicleHashSerializer_ShouldNotReplaceRagdollRuntimeReplayContributor()
    {
        var runtime = new RagdollRuntime3D(
            7,
            Array.Empty<SolidBody>(),
            Array.Empty<Joint3D>(),
            RagdollSelfCollisionPolicy.SuppressAllLinks);

        ChronicleHash serializerHash = Hash((ref ChronicleHashWriter writer) =>
            ChronicleHashSerializer.Contribute(runtime, ref writer));
        ChronicleHash replayPayloadHash = Hash((ref ChronicleHashWriter writer) =>
        {
            writer.WriteInt32(runtime.Id);
            writer.WriteEnum(runtime.SelfCollisionPolicy);
            writer.WriteBool(runtime.IsActive);
            writer.WriteInt32(runtime.LinkCount);
            writer.WriteInt32(runtime.JointCount);
        });

        serializerHash.Should().NotBe(replayPayloadHash);
    }

    private static ChronicleHash Hash(WriterAction action)
    {
        var writer = new ChronicleHashWriter();
        action(ref writer);
        return writer.ToHash();
    }

    private delegate void WriterAction(ref ChronicleHashWriter writer);
}
