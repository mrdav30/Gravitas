using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyColliderReconfigurationClearanceTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SubRawCornerPenetration_ShouldBlockDespiteRoundedZeroContactDepth(bool capsule)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 raw = Fixed64.FromRaw(1);
        Vector3d position = new((Fixed64)3 + raw, Fixed64.Zero, (Fixed64)4 - raw);
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        LSCollider source = capsule
            ? new LSCapsuleCollider(ColliderShapeDefinition.Capsule((Fixed64)4, (Fixed64)8))
            : new LSSphereCollider(ColliderShapeDefinition.Sphere((Fixed64)4));
        var body = new SolidBody(new TestMatterAgent(context, transform), source) { Mass = Fixed64.One };
        body.Initialize(position, FixedQuaternion.Identity, BodyMotionType.Kinematic);
        var obstacle = new LSCuboidCollider(ColliderShapeDefinition.Cuboid(new Vector3d(2, 2, 2)));
        obstacle.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(-1, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
        var definition = capsule
            ? ColliderShapeDefinition.Capsule((Fixed64)5, (Fixed64)10)
            : ColliderShapeDefinition.Sphere((Fixed64)5);
        var hash = context.ComputeReplayHash();

        var result = body.TryReconfigureCollider(definition, Vector3d.Zero, position, out var blocker, out var failure);

        result.Should().Be(ColliderReconfigurationStatus.Blocked);
        blocker.Should().BeSameAs(obstacle);
        failure.Should().BeNull();
        context.ComputeReplayHash().Should().Be(hash);
    }
}
