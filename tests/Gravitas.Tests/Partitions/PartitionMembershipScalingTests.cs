using FluentAssertions;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests;

public class PartitionMembershipScalingTests
{
    [Theory]
    [InlineData(PhysicsRuntimeMode.ThreeD)]
    [InlineData(PhysicsRuntimeMode.TwoD)]
    [InlineData(PhysicsRuntimeMode.Mixed)]
    public void LocalMembership_ShouldNotAllocateByGlobalColliderId(PhysicsRuntimeMode mode)
    {
        using var context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = mode;
        Action add;
        if (mode == PhysicsRuntimeMode.ThreeD)
        {
            var partition = new PhysicsPartition();
            partition.SetOwner(context.Collisions);
            add = () => partition.AddDynamicObject(1 << 20);
        }
        else if (mode == PhysicsRuntimeMode.TwoD)
        {
            var partition = new PhysicsPartition2D();
            partition.SetOwner(context.Collisions2D);
            add = () => partition.AddDynamicObject(1 << 20);
        }
        else
        {
            var partition = new PhysicsMixedPartition();
            partition.SetOwner(context.MixedCollisions);
            add = () => partition.AddDynamic3DObject(1 << 20);
        }

        // A cell containing one high ID needs small local sets, not multi-megabyte
        // sparse arrays. Include dynamic and awake membership creation in the gate.
        long before = GC.GetAllocatedBytesForCurrentThread();
        add();
        (GC.GetAllocatedBytesForCurrentThread() - before).Should().BeLessThan(16_384);
    }

    [Fact]
    public void MembershipAdds_ShouldRejectNegativeColliderIds()
    {
        using var context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var spatial = new PhysicsPartition();
        spatial.SetOwner(context.Collisions);
        var planar = new PhysicsPartition2D();
        planar.SetOwner(context.Collisions2D);
        var mixed = new PhysicsMixedPartition();
        mixed.SetOwner(context.MixedCollisions);
        Action[] adds = {
            () => spatial.AddDynamicObject(-1), () => spatial.AddKinematicObject(-1), () => spatial.AddStaticObject(-1),
            () => planar.AddDynamicObject(-1), () => planar.AddKinematicObject(-1), () => planar.AddStaticObject(-1),
            () => mixed.AddDynamic3DObject(-1), () => mixed.AddKinematic3DObject(-1), () => mixed.AddStatic3DObject(-1),
            () => mixed.AddDynamic2DObject(-1), () => mixed.AddKinematic2DObject(-1), () => mixed.AddStatic2DObject(-1)
        };
        foreach (Action add in adds)
            add.Should().Throw<ArgumentOutOfRangeException>();
    }

    [Fact]
    public void PublicInspection_ShouldCopySortedIdsAndExposeRoleCounts()
    {
        using var context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var spatial = new PhysicsPartition();
        spatial.SetOwner(context.Collisions);
        var planar = new PhysicsPartition2D();
        planar.SetOwner(context.Collisions2D);
        var mixed = new PhysicsMixedPartition();
        mixed.SetOwner(context.MixedCollisions);
        spatial.AddDynamicObject(30); spatial.AddKinematicObject(10); spatial.AddStaticObject(20);
        planar.AddDynamicObject(30); planar.AddKinematicObject(10); planar.AddStaticObject(20);
        mixed.AddDynamic3DObject(30); mixed.AddKinematic3DObject(10); mixed.AddStatic3DObject(20);
        mixed.AddDynamic2DObject(60); mixed.AddKinematic2DObject(40); mixed.AddStatic2DObject(50);
        int[] counts = { spatial.DynamicObjectCount, spatial.KinematicObjectCount, spatial.StaticObjectCount,
            planar.DynamicObjectCount, planar.KinematicObjectCount, planar.StaticObjectCount,
            mixed.Dynamic3DObjectCount, mixed.Kinematic3DObjectCount, mixed.Static3DObjectCount,
            mixed.Dynamic2DObjectCount, mixed.Kinematic2DObjectCount, mixed.Static2DObjectCount,
            spatial.AwakeDynamicObjectCount, planar.AwakeDynamicObjectCount,
            mixed.AwakeDynamic3DObjectCount, mixed.AwakeDynamic2DObjectCount };
        foreach (int count in counts)
            count.Should().Be(1);
        mixed.AwakeDynamicObjectCount.Should().Be(2);

        var ids = new SwiftList<int> { 99 };
        Action<SwiftList<int>>[] copies = { spatial.CopyAllColliderIds, planar.CopyAllColliderIds,
            mixed.Copy3DColliderIds, mixed.Copy2DColliderIds };
        for (int i = 0; i < copies.Length; i++)
        {
            copies[i](ids);
            ids.ToArray().Should().Equal(i == 3 ? new[] { 40, 50, 60 } : new[] { 10, 20, 30 });
            Action copyNull = () => copies[i](null!);
            copyNull.Should().Throw<ArgumentNullException>().WithParameterName("destination");
        }
    }
}
