using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

/// <content>Checks actual separation of retained support bands and finite cap connections.</content>
public sealed partial class JoinedStaticSolidMixedTests
{
    [Fact]
    public void RepeatedCompoundLeafResponse_ShouldKeepItsOwnerDependencyAndProvenance()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two),
            Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var compound = new LSCompoundCollider(CompoundColliderPart.Cuboid(
            Vector3d.One, Vector3d.Zero, PhysicsMaterial.Bouncy));
        scene.CreateBody(compound, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity);
        Assert.Equal(Capture(), Capture());

        List<(ContactGroupKey Key, ManifoldContact Row)> Capture()
        {
            var manifold = new ContactManifold(); manifold.BeginUpdate(scene.Context.FrameCount);
            Assert.True(CollisionDetectionMixed.AppendManifold(compound.GetPartCollider(0), members[0],
                manifold, 1, 0, surfaceResponse: true));
            var result = new List<(ContactGroupKey Key, ManifoldContact Row)>();
            for (int group = 0; group < manifold.GroupCount; group++)
            {
                ref var contacts = ref manifold.GetGroup(group);
                Assert.Equal(1, contacts.Key.NamespaceA); Assert.True(contacts.Key.SourceIdB > 0);
                for (int row = 0; row < contacts.Count; row++)
                {
                    Assert.Equal(1, contacts[row].FeatureNamespaceA);
                    Assert.Equal(PhysicsMaterial.Bouncy, contacts[row].MaterialA);
                    Assert.Equal(Vector3d.Backward, contacts[row].Normal); Assert.Equal(Fixed64.Quarter, contacts[row].Depth);
                    result.Add((contacts.Key, contacts[row]));
                }
            }
            Assert.NotEmpty(result); return result;
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void RemoteJoinedFaceAndTipWithSeparatedRetainedHeights_ShouldKeepIndependentRegions(bool hull, bool clippedTip)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D face = Rectangle(scene, -2, 0, -2, 0, Fixed64.Two);
        var tip = new LSPolygonCollider2D(new(0, 0), new(1, -2), new(2, -2))
            { MixedHalfThicknessOverride = clippedTip ? Fixed64.Two : Fixed64.One,
                Material = PhysicsMaterial.Frictionless };
        tip.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(
            new Vector3d(Fixed64.Zero, clippedTip ? Fixed64.Zero : -Fixed64.One, Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.One)));
        var members = new List<LSCollider2D> { face, tip,
            Rectangle(scene, -2, 2, -4, -2, Fixed64.Two),
            Rectangle(scene, -2, 0, 0, 2, 3 * Fixed64.Half, -Fixed64.Half) };
        if (clippedTip)
        {
            var cover = new LSPolygonCollider2D(new(0, 0), new(-1, 1), new(1, -2))
                { MixedHalfThicknessOverride = 3 * Fixed64.Half, Material = PhysicsMaterial.Frictionless };
            cover.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                new FixedTransform(Vector3d.Up * Fixed64.Half, FixedQuaternion.Identity, Vector3d.One)));
            members.Add(cover);
        }
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members.ToArray());
        LSCollider mover = Mover(scene, hull, Vector3d.Forward * Fixed64.Quarter,
            FixedQuaternion.Identity, new(3, 6, 1));
        var rows = Rows(mover, face, tip); Assert.NotEmpty(rows);
        var regions = new HashSet<int>(); var suppliers = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(-Fixed64.Quarter, moving.Z); Assert.Equal(Fixed64.Zero, source.Z);
            if (sample.Supplier == face.Id + 1)
            {
                Assert.InRange(source.X.m_rawValue, (-3 * Fixed64.Half).m_rawValue, Fixed64.Zero.m_rawValue);
                Assert.InRange(source.Y.m_rawValue, (clippedTip ? -Fixed64.One : Fixed64.One).m_rawValue, Fixed64.Two.m_rawValue);
            }
            else
            {
                Assert.Equal(tip.Id + 1, sample.Supplier); Assert.Equal(Fixed64.Zero, source.X);
                Assert.InRange(source.Y.m_rawValue, (-Fixed64.Two).m_rawValue,
                    (clippedTip ? -Fixed64.One : Fixed64.Zero).m_rawValue);
            }
            regions.Add(sample.Region); suppliers.Add(sample.Supplier);
        }
        // The footprint point alone cannot bridge the front supports: actual
        // continuity runs through the rear, outside this moving support plane.
        Assert.Equal(2, regions.Count);
        Assert.Contains(face.Id + 1, suppliers); Assert.Contains(tip.Id + 1, suppliers);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void JoinedFrontFacesWithDisjointRetainedBands_ShouldKeepIndependentConstraints(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D left = Rectangle(scene, -2, 0, -2, 0, Fixed64.Two);
        LSCollider2D right = Rectangle(scene, 0, 2, -2, 0, Fixed64.Two);
        var members = new[] { left, right,
            Rectangle(scene, -2, 0, 0, 2, 5 * Fixed64.Quarter, -3 * Fixed64.Quarter),
            Rectangle(scene, 0, 2, 0, 2, 5 * Fixed64.Quarter, 3 * Fixed64.Quarter) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = Mover(scene, hull, Vector3d.Forward * Fixed64.Quarter,
            FixedQuaternion.Identity, new(3, 6, 1));
        var rows = Rows(mover, left, right); Assert.NotEmpty(rows);
        var regions = new HashSet<int>(); var suppliers = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(-Fixed64.Quarter, moving.Z); Assert.Equal(Fixed64.Zero, source.Z);
            if (sample.Supplier == left.Id + 1)
            {
                Assert.InRange(source.X.m_rawValue, (-3 * Fixed64.Half).m_rawValue, Fixed64.Zero.m_rawValue);
                Assert.InRange(source.Y.m_rawValue, Fixed64.Half.m_rawValue, Fixed64.Two.m_rawValue);
            }
            else
            {
                Assert.Equal(right.Id + 1, sample.Supplier);
                Assert.InRange(source.X.m_rawValue, Fixed64.Zero.m_rawValue, (3 * Fixed64.Half).m_rawValue);
                Assert.InRange(source.Y.m_rawValue, (-Fixed64.Two).m_rawValue, (-Fixed64.Half).m_rawValue);
            }
            regions.Add(sample.Region); suppliers.Add(sample.Supplier);
        }
        // Whole slabs join, but the real z=0 support patches are separated
        // by a covered height band; shared membership cannot bridge that gap.
        Assert.Equal(2, regions.Count);
        Assert.Contains(left.Id + 1, suppliers); Assert.Contains(right.Id + 1, suppliers);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void JoinedLowerCaps_ShouldConnectTheirFiniteSupportsFromBelow(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -2, 0, Fixed64.Two),
            Rectangle(scene, 0, 4, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = Mover(scene, hull, new(Fixed64.Zero, -7 * Fixed64.Quarter, -Fixed64.One), FixedQuaternion.Identity);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        var regions = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Up, sample.Row.Normal); Assert.Equal(3 * Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(-5 * Fixed64.Quarter, moving.Y); Assert.Equal(-Fixed64.Two, source.Y);
            Assert.Equal(moving.X, source.X); Assert.Equal(moving.Z, source.Z);
            regions.Add(sample.Region);
        }
        Assert.Single(regions);
        AssertWholeShapeExit(mover, members, new(Fixed64.Zero, -5 * Fixed64.Half, -Fixed64.One), Vector3d.Down);
    }

    [Fact]
    public void ScalarYawJoinedCaps_ShouldKeepFiniteVerticalProjectionAndWholeShapeClearance()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = new LSPolygonCollider2D(new(-4, -2), new(0, -2), new(0, 2), new(-4, 2))
            { MixedHalfThicknessOverride = Fixed64.Two, Material = PhysicsMaterial.Frictionless };
        var right = new LSPolygonCollider2D(new(0, -2), new(4, -2), new(4, 2), new(0, 2))
            { MixedHalfThicknessOverride = Fixed64.Two, Material = PhysicsMaterial.Frictionless };
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)31, Fixed64.Zero);
        foreach (LSCollider2D member in new[] { left, right })
            member.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                new FixedTransform(Vector3d.Zero, rotation, Vector3d.One)));
        Assert.NotEqual(Fixed64.Zero, left.ConvexRotation); Assert.Equal(left.ConvexRotation, right.ConvexRotation);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        LSCollider mover = Mover(scene, false, Vector3d.Up * (7 * Fixed64.Quarter), FixedQuaternion.Identity);
        var rows = Rows(mover, left, right); Assert.NotEmpty(rows);
        var regions = new HashSet<int>();
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Down, sample.Row.Normal); Assert.Equal(3 * Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(5 * Fixed64.Quarter, moving.Y); Assert.Equal(Fixed64.Two, source.Y);
            Assert.Equal(moving.X, source.X); Assert.Equal(moving.Z, source.Z);
            Assert.InRange(source.X.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
            Assert.InRange(source.Z.m_rawValue, (-Fixed64.Half).m_rawValue, Fixed64.Half.m_rawValue);
            regions.Add(sample.Region);
        }
        Assert.Single(regions);
        AssertWholeShapeExit(mover, new LSCollider2D[] { left, right }, Vector3d.Up * (5 * Fixed64.Half), Vector3d.Up);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NestedFrontBandInsideTheMovingSupport_ShouldConnectThroughItsRetainedEndpoints(bool hull)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        LSCollider2D wall = Rectangle(scene, -4, 4, -2, 0, Fixed64.Two);
        var members = new[] { wall, Rectangle(scene, -4, -1, 0, 2, Fixed64.One, Fixed64.One),
            Rectangle(scene, 1, 4, 0, 2, Fixed64.One, Fixed64.One) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = Mover(scene, hull, Vector3d.Forward * Fixed64.Quarter,
            FixedQuaternion.Identity, new(6, 6, 1));
        var rows = Rows(mover, wall); Assert.NotEmpty(rows);
        var regions = new HashSet<int>(); bool below = false, above = false;
        foreach (var sample in rows)
        {
            Assert.Equal(wall.Id + 1, sample.Supplier);
            Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(-Fixed64.Quarter, moving.Z); Assert.Equal(Fixed64.Zero, source.Z);
            Assert.InRange(source.X.m_rawValue, ((Fixed64)(-3)).m_rawValue, ((Fixed64)3).m_rawValue);
            Assert.InRange(source.Y.m_rawValue, (-Fixed64.Two).m_rawValue, Fixed64.Two.m_rawValue);
            if (source.Y > Fixed64.Zero)
                Assert.InRange(source.X.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
            below |= source.Y < Fixed64.Zero; above |= source.Y > Fixed64.Zero;
            regions.Add(sample.Region);
        }
        // The full lower face and narrow upper face meet inside the support
        // face. Its outer edges never intersect their common finite segment.
        Assert.Single(regions); Assert.True(below && above);
    }
}
