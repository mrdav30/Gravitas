using Chronicler.Timing;
using FixedMathSharp;
using Gravitas.Support;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class SolidBody2DGroundingTests
{
    [Theory]
    [InlineData(0L)]
    [InlineData((long)int.MaxValue - 5)]
    [InlineData(long.MaxValue - 10)]
    public void AutomaticGroundCacheExpiresAfterTenFramesAtWideOrigins(long initialFrame)
    {
        using var context = CreateContext();
        TimingTestUtility.SetClock(context, initialFrame, new ChronicleTimestamp(1, 0));
        CreateStaticFloor(context, layer: new PhysicsLayer(1));
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        var body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        Assert.True(body.IsGrounded);
        body.GroundedDistanceRay = Fixed64.Zero;

        TimingTestUtility.SetClock(context, initialFrame + 9, new ChronicleTimestamp(2, 0));
        body.BeginAutomaticGroundingRefresh();
        body.CompleteAutomaticGroundingRefresh();
        Assert.True(body.IsGrounded);

        context.Simulate();
        body.BeginAutomaticGroundingRefresh();
        body.CompleteAutomaticGroundingRefresh();
        Assert.False(body.IsGrounded);
        Assert.True(body.WasGrounded);
    }

    public static TheoryData<GravitasSerializationTransport> TimingTransports => GravitasSerializationTransportCases.All();

    [Theory]
    [MemberData(nameof(TimingTransports))]
    public void PopulateAtWideFrameInvalidatesAutomaticGroundCache(GravitasSerializationTransport transport)
    {
        using var context = CreateContext();
        TimingTestUtility.SetClock(context, (long)int.MaxValue + 17, new ChronicleTimestamp(3155760000L, 0));
        CreateStaticFloor(context, layer: new PhysicsLayer(1));
        context.Settings.GroundCheckLayerMask = PhysicsLayerMask.FromLayer(1);
        var body = CreateCircle(context, new Vector2d(Fixed64.Zero, Fixed64.One));
        Assert.True(body.IsGrounded);
        body.GroundedDistanceRay = Fixed64.Zero;
        var payload = GravitasSerializationHarness.Serialize(body, transport);
        GravitasSerializationHarness.Populate(body, payload, transport);

        body.BeginAutomaticGroundingRefresh();
        body.CompleteAutomaticGroundingRefresh();
        Assert.False(body.IsGrounded);
        Assert.True(body.WasGrounded);
    }
}
