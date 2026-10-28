using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System.Linq;
using Xunit;

namespace Gravitas.Tests;

/// <content>Checks source-response selection for spheres and zero-core capsules.</content>
public sealed partial class JoinedStaticSolidMixedTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RoundedPointUnequalHeightJoin_ShouldPreserveUnforcedAxesDuringAFullStep(bool pointCapsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[]
        {
            Rectangle(scene, -2, 0, -2, 0, 3 * Fixed64.Quarter),
            Rectangle(scene, 0, 2, -2, 0, Fixed64.Quarter, 3 * Fixed64.FromFraction(1, 8)),
            Rectangle(scene, -2, 0, -4, -2, Fixed64.Quarter, -Fixed64.Half)
        };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = pointCapsule
            ? new LSCapsuleCollider { Radius = Fixed64.Half, Size = Vector3d.One }
            : new LSSphereCollider { Radius = Fixed64.Half };
        mover.Material = PhysicsMaterial.Frictionless;
        SolidBody body = scene.CreateBody(mover, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        body.SleepEnabled = false; body.UseManualGrounding();
        var initial = Rows(mover, members); Assert.NotEmpty(initial);
        foreach (var sample in initial)
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal);
            Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
        }
        body.AddLinearImpulse(Vector3d.Backward * Fixed64.FromFraction(1, 16));
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Vector3d position = body.Position3d, velocity = body.LinearVelocity;

        // Probe the complete shape only after capturing the measured frame;
        // the common front plane provides a horizontal exit for both forms.
        AssertWholeShapeExit(mover, members, Vector3d.Forward * Fixed64.Half, Vector3d.Forward);
        Assert.Equal(Fixed64.Zero, position.X);
        Assert.Equal(Fixed64.Zero, position.Y);
        Assert.InRange(position.Z.m_rawValue, Fixed64.Quarter.m_rawValue, Fixed64.Half.m_rawValue);
        Assert.Equal(Vector3d.Zero, velocity);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RoundedPointOffsetHeightNeighbors_ShouldRetainTheAdmittedWallSupport(bool pointCapsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D wall = Rectangle(scene, -2, 2, -2, 0, Fixed64.Two);
        var members = new[]
        {
            wall,
            Rectangle(scene, -1, 1, 0, 2, Fixed64.One, -Fixed64.One),
            Rectangle(scene, 1, 2, 0, 2, Fixed64.One, Fixed64.One)
        };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = pointCapsule
            ? new LSCapsuleCollider { Radius = Fixed64.Half, Size = Vector3d.One }
            : new LSSphereCollider { Radius = Fixed64.Half };
        mover.Material = PhysicsMaterial.Frictionless;
        scene.CreateBody(mover, new(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Quarter), FixedQuaternion.Identity);
        var rows = Rows(mover, wall); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal);
            Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), source);
        }
        // The complete sphere exits this original wall at Z=.5. The lower
        // neighbor still supplies its independent top contact at this height.
        AssertWholeShapeExit(mover, new[] { wall }, new(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Half), Vector3d.Forward);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RoundedPointZigzagJoin_ShouldUseACompleteExitInsteadOfOpposingSourceCuts(bool pointCapsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[]
        {
            Rectangle(scene, -1, 1, -4, 4, 3 * Fixed64.Half),
            Rectangle(scene, -8, -1, -4, 0, 3 * Fixed64.Half),
            Rectangle(scene, 1, 8, 0, 4, 3 * Fixed64.Half)
        };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, pointCapsule, Vector3d.Zero, Fixed64.Two);
        var rows = Rows(mover, members[0]); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(Fixed64.Zero, sample.Row.Normal.X);
            Assert.Equal(Fixed64.One, sample.Row.Normal.Y.Abs());
            Assert.Equal(Fixed64.Zero, sample.Row.Normal.Z);
            Assert.Equal(7 * Fixed64.Half, sample.Row.Depth);
        }
        // The central source's closest side windows oppose one another.
        // Either tied cap exit clears the complete declaration instead.
        Vector3d outward = -rows[0].Row.Normal;
        AssertWholeShapeExit(mover, members, outward * (7 * Fixed64.Half), outward);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RoundedPointDisconnectedConvexJoin_ShouldKeepItsOwnSupplierBesideAConcaveComponent(bool pointCapsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[]
        {
            Rectangle(scene, -16, 0, -16, 16, (Fixed64)8),
            Rectangle(scene, 0, 16, -16, 0, (Fixed64)8),
            Rectangle(scene, 4, 8, 1, 2, (Fixed64)8),
            Rectangle(scene, 4, 8, 2, 3, (Fixed64)8)
        };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, pointCapsule, new(2, 0, 2), (Fixed64)4);
        var rows = Rows(mover, members);
        Assert.Contains(rows, sample => sample.Row.Normal == Vector3d.Left && sample.Row.Depth == Fixed64.Two);
        Assert.Contains(rows, sample => sample.Row.Normal == Vector3d.Backward && sample.Row.Depth == Fixed64.Two);
        var convex = Assert.Single(rows, sample => sample.Row.Normal == Vector3d.Right);
        Assert.Equal(Fixed64.Two, convex.Row.Depth);
        Assert.Contains(convex.Supplier, new[] { members[2].Id + 1, members[3].Id + 1 });
        Assert.True(convex.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
        Assert.Equal(new Vector3d(4, 0, 2), source);
        AssertWholeShapeExit(mover, members, new(4, 0, 7), Vector3d.Right + Vector3d.Forward);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RoundedPointJoinedSourceSelection_ShouldPreserveExactMemberOverlapIdentity(bool pointCapsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[]
        {
            Rectangle(scene, -1, 1, -4, 4, (Fixed64)8),
            Rectangle(scene, -8, -1, -4, 0, (Fixed64)4),
            Rectangle(scene, 1, 8, 0, 4, Fixed64.Two),
            Rectangle(scene, 4, 6, -3, -1, Fixed64.One),
            Rectangle(scene, 6, 8, -3, -1, Fixed64.One)
        };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, pointCapsule, Vector3d.Zero, Fixed64.Two);
        var manifold = new ContactManifold();
        // Complete containment, concave windows, height changes and another
        // component may change response ownership, but never member overlap events.
        foreach (int x in new[] { -2, 0, 2, 6 })
            foreach (int y in new[] { 0, 3 })
                foreach (int z in new[] { -3, 0, 3 })
                {
                    Vector3d position = new(x, y, z); SetPosition(mover, position);
                    foreach (LSCollider2D member in members)
                    {
                        var bounds = member.MixedBounds3D; Fixed64 squared = Fixed64.Zero;
                        for (int axis = 0; axis < 3; axis++)
                        {
                            Fixed64 gap = FixedMath.Max(Fixed64.Zero,
                                FixedMath.Max(bounds.Min[axis] - position[axis], position[axis] - bounds.Max[axis]));
                            squared += gap * gap;
                        }
                        Assert.Equal(squared <= (Fixed64)4,
                            CollisionDetectionMixed.TryBuildDiscreteManifold(mover, member, manifold));
                    }
                }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RoundedPointCoveredSource_ShouldKeepItsOverlapWithoutPublishingOpposingCapCuts(bool pointCapsule)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[]
        {
            Rectangle(scene, -1, 1, -1, 1, Fixed64.One),
            Rectangle(scene, -4, -1, -4, 4, (Fixed64)4),
            Rectangle(scene, 1, 4, -4, 4, (Fixed64)4),
            Rectangle(scene, -1, 1, 1, 4, (Fixed64)4),
            Rectangle(scene, -1, 1, -4, -1, (Fixed64)4)
        };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = SphericalMover(scene, pointCapsule, new(2, 0, 0), Fixed64.Two);
        Assert.True(CollisionDetectionMixed.TryBuildDiscreteManifold(mover, members[0], new ContactManifold()));
        var rows = Rows(mover, members);
        // The center's orthogonal cap projections lie outside this footprint.
        // Its two finite cap cuts cannot form a compatible source exit, while
        // the right member's contacts and the central overlap must remain live.
        Assert.DoesNotContain(rows, sample => sample.Supplier == members[0].Id + 1
            && sample.Row.Normal.Y != Fixed64.Zero && sample.Row.Depth > Fixed64.Zero);
        Assert.Contains(rows, sample => sample.Supplier == members[2].Id + 1 && sample.Row.Depth > Fixed64.Zero);
        AssertWholeShapeExit(mover, members, new(6, 0, 0), Vector3d.Right);
    }
}
