using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids;
using SwiftCollections;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Physics2DQueryCandidateOrderingTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void BoundsCandidates_WithOverlappingGridsAndMembershipHistory_ShouldKeepUniqueSortedOwnersAndQueryWitnesses(
        bool raycastMarker, bool staticStyleOnly)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.World.TryAddGrid(new GridConfiguration(
            new Vector3d(-8, 0, -8), new Vector3d(8, 0, 8)), out _).Should().BeTrue();
        LSCollider2D[] owners =
        {
            CreateBox(context, BodyMotionType.Static),
            CreateBox(context, BodyMotionType.Kinematic),
            CreateBox(context, BodyMotionType.Dynamic),
            CreateBodylessBox(context),
            CreateBox(context, BodyMotionType.Kinematic),
            CreateBox(context, BodyMotionType.Dynamic)
        };
        LSCollider2D inactive = CreateBox(context, BodyMotionType.Dynamic);
        inactive.IsActive = false;
        LSCollider2D masked = CreateBox(context, BodyMotionType.Kinematic);
        masked.Layer = new PhysicsLayer(1);
        LSCollider2D outside = CreateBox(context, BodyMotionType.Static, new Vector2d(10, 0));
        context.RefreshQueryPartitions();
        owners[0].PartitionCoordinates!.Count.Should().BeGreaterThan(1);
        owners[0].PartitionCoordinates!.Select(coordinate => coordinate.GridIndex)
            .Distinct().Count().Should().Be(2);
        var partitions = new SwiftList<PhysicsPartition2D>();
        for (int i = 0; i < owners[0].PartitionCoordinates!.Count; i++)
        {
            context.World.TryGetVoxel(owners[0].PartitionCoordinates![i], out Voxel? voxel).Should().BeTrue();
            voxel!.TryGetPartition(out PhysicsPartition2D? partition).Should().BeTrue();
            partitions.Add(partition!);
            // Stale and misplaced memberships exercise lookup/activity/bounds
            // filtering before the sole observable candidate ordering step.
            partition!.AddDynamicObject(999);
            partition.AddKinematicObject(999);
            partition.AddStaticObject(999);
            partition.AddDynamicObject(inactive.Id);
            partition.AddStaticObject(outside.Id);
        }
        LSCollider2D[] expected = owners.Where(owner => !staticStyleOnly
            || owner.IsStatic || owner.Body!.IsKinematic).ToArray();
        var candidates = new SwiftList<LSCollider2D>();
        Vector2d min = new(-4, -4);
        Vector2d max = new(4, 4);
        PhysicsLayerMask layers = PhysicsLayerMask.FromLayer(0);

        context.Collisions2D.CollectBoundsCandidates(min, max, layers, 100000, raycastMarker, candidates, staticStyleOnly);
        candidates.Should().Equal(expected);
        // Ray and circle stamps are independent even for an identical numeric
        // version. Repeating the same stamp suppresses every duplicate owner.
        context.Collisions2D.CollectBoundsCandidates(min, max, layers, 100000, !raycastMarker, candidates, staticStyleOnly);
        candidates.Should().Equal(expected);
        context.Collisions2D.CollectBoundsCandidates(min, max, layers, 100000, raycastMarker, candidates, staticStyleOnly);
        candidates.Should().BeEmpty();
        var rayHits = new SwiftList<Physics2DHit>();
        context.Query2D.RaycastAll(new Vector2d(-4, 0), new Vector2d(4, 0), layers, rayHits).Should().Be(owners.Length);
        context.Query2D.LastQueryCandidateCount.Should().Be(owners.Length);
        Physics2DHit[] expectedRayHits = rayHits.ToArray();
        for (int i = 0; i < rayHits.Count; i++)
        {
            rayHits[i].Collider.Should().BeSameAs(owners[i]);
            rayHits[i].Point.Should().Be(new Vector2d(-2, 0));
            rayHits[i].Normal.Should().Be(-Vector2d.Right);
            rayHits[i].Distance.Should().Be((Fixed64)2);
        }
        var circleHits = new SwiftList<Physics2DHit>();
        CollectCircleHits(context, staticStyleOnly, circleHits).Should().Be(expected.Length);
        context.Query2D.LastQueryCandidateCount.Should().Be(expected.Length);
        circleHits.Select(hit => hit.Collider).Should().Equal(expected);
        Physics2DHit[] expectedCircleHits = circleHits.ToArray();

        for (int p = partitions.Count - 1; p >= 0; p--)
        {
            PhysicsPartition2D partition = partitions[p];
            for (int i = 0; i < owners.Length; i++)
                RemoveOwner(partition, owners[i]);
            for (int i = owners.Length - 1; i >= 0; i--)
                AddOwner(partition, owners[i]);
        }

        context.Collisions2D.CollectBoundsCandidates(min, max, layers, 100001, raycastMarker, candidates, staticStyleOnly);
        candidates.Should().Equal(expected);
        context.Query2D.RaycastAll(new Vector2d(-4, 0), new Vector2d(4, 0), layers, rayHits).Should().Be(owners.Length);
        context.Query2D.LastQueryCandidateCount.Should().Be(owners.Length);
        rayHits.Should().Equal(expectedRayHits);
        CollectCircleHits(context, staticStyleOnly, circleHits).Should().Be(expected.Length);
        context.Query2D.LastQueryCandidateCount.Should().Be(expected.Length);
        circleHits.Should().Equal(expectedCircleHits);
    }

    private static int CollectCircleHits(GravitasWorldContext context, bool staticStyleOnly, SwiftList<Physics2DHit> hits)
        => staticStyleOnly
            ? context.Query2D.OverlapCircleAgainstStaticAll(new Vector2d(-4, 0), (Fixed64)4,
                PhysicsLayerMask.FromLayer(0), hits)
            : context.Query2D.OverlapCircleAll(new Vector2d(-4, 0), (Fixed64)4,
                PhysicsLayerMask.FromLayer(0), hits);

    private static LSCollider2D CreateBox(GravitasWorldContext context, BodyMotionType motionType, Vector2d? position = null)
    {
        Vector2d center = position ?? Vector2d.Zero;
        var transform = new FixedTransform(new Vector3d(center.X, Fixed64.Zero, center.Y),
            FixedQuaternion.Identity, Vector3d.One);
        var collider = new LSAABBoxCollider2D(new Vector2d(4, 4));
        var body = new SolidBody2D(new TestMatterAgent(context, transform), collider) { Mass = Fixed64.One };
        body.Initialize(center, motionType: motionType);
        return collider;
    }

    private static LSCollider2D CreateBodylessBox(GravitasWorldContext context)
    {
        var collider = new LSAABBoxCollider2D(new Vector2d(4, 4));
        collider.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        return collider;
    }

    private static void RemoveOwner(PhysicsPartition2D partition, LSCollider2D owner)
    {
        if (owner.IsStatic)
            partition.RemoveStaticObject(owner.Id);
        else if (owner.Body!.IsKinematic)
            partition.RemoveKinematicObject(owner.Id);
        else
            partition.RemoveDynamicObject(owner.Id);
    }

    private static void AddOwner(PhysicsPartition2D partition, LSCollider2D owner)
    {
        if (owner.IsStatic)
            partition.AddStaticObject(owner.Id);
        else if (owner.Body!.IsKinematic)
            partition.AddKinematicObject(owner.Id);
        else
            partition.AddDynamicObject(owner.Id);
    }
}
