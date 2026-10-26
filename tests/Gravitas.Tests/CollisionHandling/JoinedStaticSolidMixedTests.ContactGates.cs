using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests;

/// <content>Checks query expansion and overlap-only interior members through real contact requests.</content>
public sealed partial class JoinedStaticSolidMixedTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DeepInitiallyJoinedMember_ShouldExpandBeforeReadmittingItsCoveredNativeExit(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -10, 0, -10, 10, (Fixed64)10),
            Rectangle(scene, 0, 10, -10, 10, (Fixed64)10) };
        LSCollider mover = Mover(scene, hull, Vector3d.Left, FixedQuaternion.Identity);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        // Both members are initially near the body, but the covered seam's
        // native 1.5-unit exit already exceeds its circumscribed radius.
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Right, sample.Row.Normal);
            Assert.Equal(19 * Fixed64.Half, sample.Row.Depth);
            Assert.False(sample.Row.DepthIsClamped);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(-Fixed64.Half, moving.X); Assert.Equal((Fixed64)(-10), source.X);
        }
        AssertWholeShapeExit(mover, members, new(-21 * Fixed64.Half, Fixed64.Zero, Fixed64.Zero), Vector3d.Left);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EnclosedLowerCellWithoutAnAdmittedExit_ShouldKeepOverlapAndNeighborResponse(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new LSCollider2D[9];
        for (int x = 0; x < 3; x++)
            for (int z = 0; z < 3; z++)
                members[3 * x + z] = Rectangle(scene, 2 * x - 3, 2 * x - 1, 2 * z - 3, 2 * z - 1,
                    x == 1 && z == 1 ? Fixed64.One : Fixed64.Two);
        LSCollider mover = Mover(scene, hull, Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)31, (Fixed64)23),
            new((Fixed64)6, Fixed64.Quarter, Fixed64.Quarter));
        var native = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildManifold(mover, members[4], native));
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var central = new ContactManifold();
        Assert.True(CollisionDetectionMixed.TryBuildDiscreteManifold(mover, members[4], central));
        Assert.False(central.HasContact);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.NotEqual(members[4].Id + 1, sample.Supplier);
            Assert.True(sample.Row.Depth > Fixed64.Zero); Assert.False(sample.Row.DepthIsClamped);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out _));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
            LSCollider2D supplier = System.Array.Find(members, member => member.Id + 1 == sample.Supplier)!;
            Assert.NotNull(supplier); Assert.True(supplier.ContainsPoint(new(point.X, point.Z)));
            Assert.InRange(point.Y.m_rawValue, (-Fixed64.Two).m_rawValue, Fixed64.Two.m_rawValue);
        }
    }
}
