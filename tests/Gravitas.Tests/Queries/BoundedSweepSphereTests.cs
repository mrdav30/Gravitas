using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids.Storage;
using GridForge.Grids.Topology;
using GridForge.Spatial;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class BoundedSweepSphereTests
{
    public static TheoryData<GravitasSerializationTransport> Transports => GravitasSerializationTransportCases.All();

    [Theory]
    [MemberData(nameof(Transports))]
    public void BoundedSweep_ReplaysAfterRemovedMembershipsAndCheckpointRestore(GravitasSerializationTransport transport)
    {
        using var live = CreateContext();
        using var restored = CreateContext();
        var liveFloor = AddBoxFloor(live);
        var restoredFloor = AddBoxFloor(restored);
        var liveSecond = AddBoxFloor(live);
        var restoredSecond = AddBoxFloor(restored);
        var discarded = new LSCuboidCollider[16];
        for (int i = 0; i < discarded.Length; i++)
            discarded[i] = AddBoxFloor(live);
        foreach (var collider in discarded)
            collider.Deactivate();
        Vector3d floorPosition = liveFloor.Position;
        liveFloor.Position = floorPosition + Vector3d.Right * (Fixed64)32;
        liveFloor.Position = floorPosition;
        var transient = new LSCuboidCollider { Size = Vector3d.One };
        transient.InitializeWithNoBody(new TestMatterAgent(live,
            new FixedTransform(Vector3d.Up * (Fixed64)2, FixedQuaternion.Identity, Vector3d.One)));
        Assert.True(live.World.TryGetVoxel(Vector3d.Up * (Fixed64)2, out var occupiedHistory));
        Assert.NotNull(occupiedHistory!.GetPartitionOrDefault<PhysicsPartition>());
        transient.Deactivate();
        var planarTransient = new LSCircleCollider2D(Fixed64.Half);
        planarTransient.InitializeWithNoBody(new TestMatterAgent(live,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        Assert.True(live.World.TryGetVoxel(Vector3d.Zero, out var planarHistory));
        Assert.NotNull(planarHistory!.GetPartitionOrDefault<PhysicsPartition2D>());
        planarTransient.Deactivate();
        live.Simulate();
        live.LateSimulate();
        GravitasSerializationHarness.Populate(restoredFloor,
            GravitasSerializationHarness.Serialize(liveFloor, transport), transport);
        GravitasSerializationHarness.Populate(restoredSecond,
            GravitasSerializationHarness.Serialize(liveSecond, transport), transport);
        restored.Simulate();
        restored.LateSimulate();
        Assert.True(restored.World.TryGetVoxel(Vector3d.Up * (Fixed64)2, out var untouched));
        Assert.True(Assert.IsType<PhysicsPartition>(occupiedHistory.GetPartitionOrDefault<PhysicsPartition>()).IsEmpty);
        Assert.Null(untouched!.GetPartitionOrDefault<PhysicsPartition>());
        Assert.True(restored.World.TryGetVoxel(Vector3d.Zero, out var untouchedPlanar));
        Assert.True(Assert.IsType<PhysicsPartition2D>(planarHistory.GetPartitionOrDefault<PhysicsPartition2D>()).IsEmpty);
        Assert.Null(untouchedPlanar!.GetPartitionOrDefault<PhysicsPartition2D>());

        var scratch = new PhysicsQueryScratch(512, 16);
        var ample = new PhysicsQueryBudget(8192, 8192, 8192);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            ReplaySweep(restored, ample, scratch, out var hit, out var expected));
        Assert.Same(restoredFloor, hit.GetValueOrDefault().Collider);
        var exact = new PhysicsQueryBudget(expected.Discovery, expected.Candidates, expected.Geometry);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            ReplaySweep(live, exact, scratch, out var replay, out var used));
        Assert.Equal(expected, used);
        Assert.Same(liveFloor, replay.GetValueOrDefault().Collider);
        Assert.Equal(hit!.Value.Point, replay!.Value.Point);
        Assert.Equal(hit.Value.Distance, replay.Value.Distance);
        Assert.Equal(hit.Value.Normal, replay.Value.Normal);

        foreach (var oneBelow in new[]
        {
            new PhysicsQueryBudget(expected.Discovery - 1, expected.Candidates, expected.Geometry),
            new PhysicsQueryBudget(expected.Discovery, expected.Candidates - 1, expected.Geometry)
        })
        {
            Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
                ReplaySweep(live, oneBelow, scratch, out replay, out used));
            Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
                ReplaySweep(restored, oneBelow, scratch, out hit, out var restoredWork));
            Assert.Equal(restoredWork, used);
            Assert.Null(hit);
            Assert.Null(replay);
        }

        static PhysicsQueryExecutionStatus ReplaySweep(
            GravitasWorldContext context, PhysicsQueryBudget budget, PhysicsQueryScratch scratch,
            out Physics3DHit? hit, out PhysicsQueryWork work) =>
            context.Query3D.SweepSphere(Vector3d.Up * (Fixed64)2, Fixed64.Half, Vector3d.Down, (Fixed64)2,
                PhysicsLayerMask.All, null, budget, scratch, out hit, out work);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void BoundedSweep_ReplaysAfterRemovedLinksAndCheckpointRestore(GravitasSerializationTransport transport)
    {
        using var live = CreateContext();
        using var restored = CreateContext();
        var first = CreateLinkedFloorScene(live);
        var second = CreateLinkedFloorScene(restored);
        var discarded = new SolidBody[32];
        for (int i = 0; i < discarded.Length; i++)
        {
            var body = new SolidBody(new TestMatterAgent(live), new LSSphereCollider { Radius = Fixed64.Half });
            body.Initialize(new Vector3d(3, 3, 3), FixedQuaternion.Identity, PhysicsMobilityType.Kinematic);
            discarded[i] = body;
            Link(live, first.Source, body);
        }
        foreach (var body in discarded)
            body.Deactivate();
        Assert.Equal(1, live.Constraints3D.RegisteredJointCount);
        GravitasSerializationHarness.Populate(second.Joint,
            GravitasSerializationHarness.Serialize(first.Joint, transport), transport);
        live.Simulate();
        live.LateSimulate();
        restored.Simulate();
        restored.LateSimulate();

        var scratch = new PhysicsQueryScratch(512, 4);
        var ample = new PhysicsQueryBudget(8192, 8192, 8192);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            Sweep(restored, ample, scratch, out var hit, out var expected, second.Source.Collider));
        Assert.Same(second.Floor, hit.GetValueOrDefault().Collider);
        var exact = new PhysicsQueryBudget(expected.Discovery, expected.Candidates, expected.Geometry);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            Sweep(live, exact, scratch, out var replay, out var used, first.Source.Collider));
        Assert.Equal(expected, used);
        Assert.Same(first.Floor, replay.GetValueOrDefault().Collider);
        Assert.Equal(hit!.Value.Point, replay!.Value.Point);
        Assert.Equal(hit.Value.Distance, replay.Value.Distance);
        Assert.Equal(hit.Value.Normal, replay.Value.Normal);

        var oneBelow = new PhysicsQueryBudget(expected.Discovery, expected.Candidates - 1, expected.Geometry);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(live, oneBelow, scratch, out replay, out used, first.Source.Collider));
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(restored, oneBelow, scratch, out hit, out expected, second.Source.Collider));
        Assert.Equal(expected, used);
        Assert.Null(hit);
        Assert.Null(replay);

        var beforeLinkedProbe = new PhysicsQueryBudget(exact.DiscoveryLimit, 1, exact.GeometryLimit);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(live, beforeLinkedProbe, scratch, out replay, out used, first.Source.Collider));
        Assert.Equal(1UL, used.Candidates);
        Assert.Equal(0UL, used.Geometry);
        Assert.Null(replay);

        static (SolidBody Source, LSCuboidCollider Floor, Joint3D Joint)
            CreateLinkedFloorScene(GravitasWorldContext context)
        {
            context.Environment.Gravity = Fixed64.Zero;
            var source = new SolidBody(new TestMatterAgent(context),
                new LSSphereCollider { Radius = Fixed64.Half });
            source.Initialize(new Vector3d(3, 3, 3), FixedQuaternion.Identity);
            source.UseManualGrounding();
            var linked = new SolidBody(new TestMatterAgent(context),
                new LSCuboidCollider { Size = new Vector3d(3, 1, 3) });
            linked.Initialize(-Vector3d.Up * Fixed64.Half, FixedQuaternion.Identity, PhysicsMobilityType.Kinematic);
            var floor = AddBoxFloor(context);
            return (source, floor, Link(context, source, linked));
        }

        static Joint3D Link(GravitasWorldContext context, SolidBody source, SolidBody target)
        {
            var frame = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
            Joint3D joint = context.Constraints3D.RegisterJoint(new JointDefinition3D(source, target,
                frame, frame, JointType3D.BallSocket, JointLimit3D.Unrestricted,
                JointMotor3D.Disabled, JointCollisionPolicy.SuppressLinked));
            joint.IsEnabled = false;
            return joint;
        }
    }

    [Fact]
    public void BoxFloorMatchesSynchronousSweepAndReplaysAtExactLimits()
    {
        using GravitasWorldContext context = CreateContext();
        var floor = AddBoxFloor(context);
        AddBoxFloor(context);
        context.Simulate();
        context.LateSimulate();

        Assert.True(context.Query3D.SweepSphere(Vector3d.Up, Fixed64.Half,
            Vector3d.Down, Fixed64.One, out Physics3DHit expected, PhysicsLayerMask.All));
        Assert.Equal(Fixed64.Half, expected.Distance);
        Assert.Equal(Vector3d.Zero, expected.Point);

        var scratch = new PhysicsQueryScratch(512, 2);
        var ample = new PhysicsQueryBudget(8192, 8192, 8192);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            Sweep(context, ample, scratch, out Physics3DHit? hit, out PhysicsQueryWork used));
        Assert.Equal(expected, hit!.Value);

        var exact = new PhysicsQueryBudget(used.Discovery, used.Candidates, used.Geometry);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(context, new PhysicsQueryBudget(used.Discovery, used.Candidates, used.Geometry - 1),
                scratch, out hit, out PhysicsQueryWork failed));
        Assert.Null(hit);
        Assert.True(failed.Geometry < used.Geometry);

        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(context, new PhysicsQueryBudget(used.Discovery - 1, used.Candidates, used.Geometry),
                scratch, out hit, out _));
        Assert.Null(hit);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(context, new PhysicsQueryBudget(used.Discovery, used.Candidates - 1, used.Geometry),
                scratch, out hit, out _));
        Assert.Null(hit);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(context, new PhysicsQueryBudget(used.Discovery, 0, used.Geometry),
                scratch, out hit, out PhysicsQueryWork noCandidates));
        Assert.Null(hit);
        Assert.Equal(0UL, noCandidates.Candidates);

        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(context, new PhysicsQueryBudget(used.Discovery - 2, used.Candidates, used.Geometry),
                scratch, out hit, out var partial));
        Assert.Null(hit);
        Assert.Equal(used.Discovery - 2, partial.Discovery);

        Assert.Equal(PhysicsQueryExecutionStatus.ScratchCapacityExceeded,
            Sweep(context, exact, new PhysicsQueryScratch(512, 1), out hit, out _));
        Assert.Null(hit);
        Assert.Equal(PhysicsQueryExecutionStatus.ScratchCapacityExceeded,
            Sweep(context, exact, new PhysicsQueryScratch(0, 2), out hit, out _));
        Assert.Null(hit);

        context.Query3D.RaycastVersion = uint.MaxValue;
        uint floorVersion = floor.RaycastVersion;
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            Sweep(context, exact, scratch, out hit, out PhysicsQueryWork replay));
        Assert.Equal(expected, hit!.Value);
        Assert.Equal(used, replay);
        Assert.Equal(uint.MaxValue, context.Query3D.RaycastVersion);
        Assert.Equal(floorVersion, floor.RaycastVersion);
        Assert.True(context.Query3D.SweepSphere(Vector3d.Up, Fixed64.Half,
            Vector3d.Down, Fixed64.One, out var afterRollover, PhysicsLayerMask.All));
        Assert.Equal(expected, afterRollover);
        Assert.Equal(1U, context.Query3D.RaycastVersion);

        Assert.True(context.World.TryGetVoxel(Vector3d.Zero, out var voxel));
        var partition = Assert.IsType<PhysicsPartition>(voxel!.GetPartitionOrDefault<PhysicsPartition>());
        partition.AddStaticObject(int.MaxValue);
        try
        {
            Assert.Equal(PhysicsQueryExecutionStatus.Completed, Sweep(context, ample, scratch, out hit, out _));
            Assert.Equal(expected, hit!.Value);
        }
        finally
        {
            partition.RemoveStaticObject(int.MaxValue);
        }
    }

    [Theory]
    [InlineData(PhysicsMobilityType.Dynamic)]
    [InlineData(PhysicsMobilityType.Kinematic)]
    public void SweepChargesEachMobilityClassBeforeReturningAHit(PhysicsMobilityType mobility)
    {
        using var context = CreateContext();
        context.Environment.Gravity = Fixed64.Zero;
        var target = new SolidBody(new TestMatterAgent(context), new LSSphereCollider { Radius = Fixed64.Half });
        target.Initialize(Vector3d.Zero, FixedQuaternion.Identity, mobility);
        target.UseManualGrounding();
        context.Simulate();
        context.LateSimulate();
        var scratch = new PhysicsQueryScratch(512, 1);
        var ample = new PhysicsQueryBudget(8192, 8192, 8192);

        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(context, new PhysicsQueryBudget(8192, 0, 8192), scratch, out var hit, out var work));
        Assert.Equal(0UL, work.Candidates);
        Assert.Null(hit);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(context, new PhysicsQueryBudget(8192, 8192, 0), scratch, out hit, out work));
        Assert.Equal(0UL, work.Geometry);
        Assert.Null(hit);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed, Sweep(context, ample, scratch, out hit, out _));
        Assert.Same(target.Collider, hit!.Value.Collider);
        Assert.Equal(Fixed64.Zero, hit.Value.Distance);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SphereMissDistinguishesBoundsRejectionFromNarrowphase(bool overlapsBounds)
    {
        using var context = CreateContext();
        var target = new LSSphereCollider { Radius = Fixed64.FromFraction(1, 8) };
        Fixed64 x = overlapsBounds ? Fixed64.Half : Fixed64.FromFraction(3, 4);
        target.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(x, Fixed64.Half, Fixed64.Half), FixedQuaternion.Identity, Vector3d.One)));
        context.Simulate();
        context.LateSimulate();
        var budget = new PhysicsQueryBudget(8192, 8192, 8192);
        var scratch = new PhysicsQueryScratch(512, 1);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed, Sweep(context, budget, scratch, out var hit, out var work));
        Assert.Null(hit);
        Assert.Equal(overlapsBounds ? 6UL : 1UL, work.Geometry);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            context.Query3D.SweepSphere(Vector3d.Up, Fixed64.Half, Vector3d.Down, Fixed64.One,
                PhysicsLayerMask.None, null, budget, scratch, out hit, out work));
        Assert.Null(hit);
        Assert.Equal(0UL, work.Geometry);
    }

    [Fact]
    public void QueryWaitsForHostReconciliationAndDoesNotRunIt()
    {
        using GravitasWorldContext context = CreateContext();
        AddBoxFloor(context);
        context.Simulate();
        context.LateSimulate();
        Assert.True(context.World.TryAddGrid(
            new GridConfiguration(new Vector3d(8, 8, 8), new Vector3d(12, 12, 12)), out _));

        var scratch = new PhysicsQueryScratch(512, 1);
        var ample = new PhysicsQueryBudget(8192, 8192, 8192);
        for (int i = 0; i < 2; ++i)
        {
            Assert.Equal(PhysicsQueryExecutionStatus.WorldNotReady,
                Sweep(context, ample, scratch, out Physics3DHit? hit, out PhysicsQueryWork work));
            Assert.Null(hit);
            Assert.Equal(default, work);
        }

        context.Simulate();
        context.LateSimulate();
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            Sweep(context, ample, scratch, out Physics3DHit? recovered, out _));
        Assert.Equal(Vector3d.Zero, recovered!.Value.Point);
    }

    [Theory]
    [InlineData(ColliderType.Sphere)]
    [InlineData(ColliderType.Capsule)]
    [InlineData(ColliderType.Cylinder)]
    [InlineData(ColliderType.Cone)]
    [InlineData(ColliderType.Compound)]
    public void PrimitiveMatchesSynchronousContactAtExactAllowance(ColliderType type)
    {
        using GravitasWorldContext context = CreateContext();
        AddBoxFloor(context);
        LSCollider target = type switch
        {
            ColliderType.Sphere => new LSSphereCollider { Radius = Fixed64.Half },
            ColliderType.Capsule => new LSCapsuleCollider { Radius = Fixed64.Half },
            ColliderType.Cylinder => new LSCylinderCollider { Radius = Fixed64.Half },
            ColliderType.Cone => new LSConeCollider { Radius = Fixed64.Half },
            ColliderType.Compound => new LSCompoundCollider(
                new CompoundColliderPart(ColliderShapeDefinition.Sphere(Fixed64.Half)),
                new CompoundColliderPart(ColliderShapeDefinition.Cuboid(Vector3d.One), Vector3d.Right * Fixed64.Two)),
            _ => throw new System.ArgumentOutOfRangeException(nameof(type))
        };
        target.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        context.Simulate();
        context.LateSimulate();

        var scratch = new PhysicsQueryScratch(512, 2);
        var ample = new PhysicsQueryBudget(8192, 8192, ulong.MaxValue);
        Assert.True(context.Query3D.SweepSphere(Vector3d.Up, Fixed64.Half, Vector3d.Down, Fixed64.One,
            out Physics3DHit synchronous, PhysicsLayerMask.All));
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            Sweep(context, ample, scratch, out Physics3DHit? hit, out PhysicsQueryWork used));
        Assert.Equal(synchronous, hit!.Value);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            Sweep(context, new PhysicsQueryBudget(used.Discovery, used.Candidates, used.Geometry),
                scratch, out hit, out PhysicsQueryWork replay));
        Assert.Equal(synchronous, hit!.Value);
        Assert.Equal(used, replay);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(context, new PhysicsQueryBudget(used.Discovery, used.Candidates, used.Geometry - 1),
                scratch, out hit, out _));
        Assert.Null(hit);

        if (type == ColliderType.Compound)
        {
            // The floor consumes 58 units and the compound bounds consume one.
            Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
                Sweep(context, new PhysicsQueryBudget(used.Discovery, used.Candidates, 59),
                    scratch, out hit, out var partial));
            Assert.Null(hit);
            Assert.Equal(59UL, partial.Geometry);
        }

        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            context.Query3D.SweepSphere(Vector3d.Up, Fixed64.Half, Vector3d.Down, Fixed64.One,
                PhysicsLayerMask.All, target, ample, scratch, out hit, out _));
        Assert.Equal(Vector3d.Zero, hit!.Value.Point);
    }

    [Fact]
    public void SweepRejectsUnrepresentableBoundsAndCompletesEmptySegmentsBeforeDiscovery()
    {
        using GravitasWorldContext context = CreateContext();
        var scratch = new PhysicsQueryScratch(0, 0);
        Check(Vector3d.Up, Fixed64.Zero, Vector3d.Down, Fixed64.One, PhysicsQueryExecutionStatus.Completed);
        Check(Vector3d.Up, Fixed64.Half, Vector3d.Zero, Fixed64.One, PhysicsQueryExecutionStatus.Completed);
        Check(Vector3d.Up, Fixed64.Half, Vector3d.Down, Fixed64.Zero, PhysicsQueryExecutionStatus.Completed);
        Check(Vector3d.Up, Fixed64.Half, Vector3d.Down, Fixed64.Epsilon, PhysicsQueryExecutionStatus.Completed);
        var scalarFace = new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        Check(scalarFace, Fixed64.Half, Vector3d.Right, Fixed64.One, PhysicsQueryExecutionStatus.Unrepresentable);
        Check(scalarFace, Fixed64.Half, Vector3d.Down, Fixed64.One, PhysicsQueryExecutionStatus.Unrepresentable);
        Check(new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half, Vector3d.Down, Fixed64.One, PhysicsQueryExecutionStatus.Unrepresentable);
        Check(new Vector3d(Fixed64.MaxValue - Fixed64.One, Fixed64.One, Fixed64.Zero),
            Fixed64.Half, Vector3d.Down, Fixed64.One, PhysicsQueryExecutionStatus.Unrepresentable);

        context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        Check(Vector3d.Up, Fixed64.Half, Vector3d.Down, Fixed64.One, PhysicsQueryExecutionStatus.UnsupportedRuntimeMode);

        void Check(Vector3d start, Fixed64 radius, Vector3d direction, Fixed64 distance,
            PhysicsQueryExecutionStatus expected)
        {
            Assert.Equal(expected, context.Query3D.SweepSphere(start, radius, direction, distance,
                PhysicsLayerMask.All, null, default, scratch, out var hit, out var work));
            Assert.Null(hit);
            Assert.Equal(default, work);
        }
    }

    [Theory]
    [InlineData(false, false, false, false)]
    [InlineData(true, false, false, false)]
    [InlineData(false, true, false, false)]
    [InlineData(false, false, true, false)]
    [InlineData(false, false, false, true)]
    public void MeshUsesFixedScratchForSweepAndWitness(bool compound, bool concave, bool affine, bool fan)
    {
        using GravitasWorldContext context = CreateContext();
        LSMeshCollider mesh;
        if (fan)
        {
            mesh = new LSMeshCollider(
                new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Forward, -Vector3d.Right, -Vector3d.Forward },
                new[] { 0, 2, 1, 0, 3, 2, 0, 4, 3, 0, 1, 4 },
                MeshColliderMode.Convex, MeshInertiaPolicy.SurfaceApproximation);
            Assert.True(mesh.Mesh.HasSingleConvexCoplanarPatch);
        }
        else
        {
            mesh = concave
                ? MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave)
                : MeshTestFixtures.CreateConvexCube();
        }
        LSCollider target = compound
            ? new LSCompoundCollider(
                new CompoundColliderPart(MeshTestFixtures.CreateDefinition(mesh)),
                new CompoundColliderPart(ColliderShapeDefinition.Sphere(Fixed64.Half), Vector3d.Right * Fixed64.Two))
            : mesh;
        var scale = affine ? new Vector3d(Fixed64.FromFraction(4, 3), Fixed64.FromFraction(2, 3), Fixed64.One) : Vector3d.One;
        target.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, scale)));
        context.Simulate();
        context.LateSimulate();

        var ample = new PhysicsQueryBudget(8192, 8192, ulong.MaxValue);
        int triangles = mesh.Mesh.TriangleCount;
        var scratch = new PhysicsQueryScratch(512, 2, triangles);
        Assert.Equal(triangles, scratch.MeshTriangleCapacity);
        Assert.True(context.Query3D.SweepSphere(Vector3d.Up, Fixed64.Half, Vector3d.Down, Fixed64.One,
            out Physics3DHit synchronous, PhysicsLayerMask.All));
        Assert.Same(target, synchronous.Collider);
        Assert.Equal(PhysicsQueryExecutionStatus.ScratchCapacityExceeded,
            Sweep(context, ample, new PhysicsQueryScratch(512, 2), out var noMeshStorage, out _));
        Assert.Null(noMeshStorage);
        Assert.Equal(PhysicsQueryExecutionStatus.ScratchCapacityExceeded,
            Sweep(context, ample, new PhysicsQueryScratch(512, 2, triangles - 1), out Physics3DHit? hit, out _));
        Assert.Null(hit);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            Sweep(context, ample, scratch, out hit, out PhysicsQueryWork used));
        Assert.Equal(synchronous, hit!.Value);
        Assert.Equal(PhysicsQueryExecutionStatus.WorkBudgetExceeded,
            Sweep(context, new PhysicsQueryBudget(used.Discovery, used.Candidates, used.Geometry - 1),
                scratch, out hit, out _));
        Assert.Null(hit);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed,
            Sweep(context, new PhysicsQueryBudget(used.Discovery, used.Candidates, used.Geometry),
                scratch, out hit, out PhysicsQueryWork replay));
        Assert.Equal(synchronous, hit!.Value);
        Assert.Equal(used, replay);

        long before = System.GC.GetAllocatedBytesForCurrentThread();
        var status = PhysicsQueryExecutionStatus.Completed;
        for (int i = 0; i < 16 && status == PhysicsQueryExecutionStatus.Completed; i++)
            status = Sweep(context, ample, scratch, out hit, out replay);
        Assert.Equal(0L, System.GC.GetAllocatedBytesForCurrentThread() - before);
        Assert.Equal(PhysicsQueryExecutionStatus.Completed, status);
        Assert.Equal(synchronous, hit!.Value);
        Assert.Equal(used, replay);
    }

    [Theory]
    [InlineData(HexOrientation.PointyTop, GridStorageKind.Dense)]
    [InlineData(HexOrientation.FlatTop, GridStorageKind.Sparse)]
    public void HexSweepUsesOnlyPhysicalVoxels(HexOrientation orientation, GridStorageKind storage)
    {
        using var context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
        var metrics = GridTopologyMetrics.Hex(Fixed64.One, Fixed64.One, orientation);
        Assert.True(context.World.TryAddGrid(new GridConfiguration(Vector3d.Zero, new Vector3d(24, 4, 24),
            topologyKind: GridTopologyKind.HexPrism, topologyMetrics: metrics, storageKind: storage), out ushort gridIndex));
        var grid = context.World.ActiveGrids[gridIndex];
        var index = new VoxelIndex(4, 1, 4);
        if (storage == GridStorageKind.Sparse)
            Assert.True(grid.TryAddVoxel(index, out _));
        Assert.True(grid.TryGetVoxel(index, out var voxel));
        Vector3d center = voxel!.WorldPosition;
        var target = new LSSphereCollider { Radius = Fixed64.Half };
        target.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(center, FixedQuaternion.Identity, Vector3d.One)));
        context.Simulate();
        context.LateSimulate();

        var scratch = new PhysicsQueryScratch(128, 8);
        var budget = new PhysicsQueryBudget(8192, 8192, 8192);
        var start = center + Vector3d.Up * Fixed64.Two;
        Assert.True(context.Query3D.SweepSphere(start, Fixed64.Half, Vector3d.Down, Fixed64.Two,
            out var synchronous, PhysicsLayerMask.All));
        Assert.Equal(PhysicsQueryExecutionStatus.Completed, Sweep(out var bounded));
        Assert.Equal(synchronous, bounded!.Value);

        if (storage == GridStorageKind.Sparse)
        {
            Assert.True(voxel.TryRemovePartition<PhysicsPartition>());
            Assert.True(grid.TryRemoveVoxel(index));
            Assert.Equal(PhysicsQueryExecutionStatus.WorldNotReady, Sweep(out bounded));
            Assert.Null(bounded);
            context.Simulate();
            context.LateSimulate();
            Assert.Equal(PhysicsQueryExecutionStatus.Completed, Sweep(out bounded));
            Assert.Null(bounded);
        }

        PhysicsQueryExecutionStatus Sweep(out Physics3DHit? hit) =>
            context.Query3D.SweepSphere(start, Fixed64.Half, Vector3d.Down, Fixed64.Two,
                PhysicsLayerMask.All, null, budget, scratch, out hit, out _);
    }

    private static PhysicsQueryExecutionStatus Sweep(
        GravitasWorldContext context, PhysicsQueryBudget budget, PhysicsQueryScratch scratch,
        out Physics3DHit? hit, out PhysicsQueryWork work, LSCollider? excluded = null) =>
        context.Query3D.SweepSphere(Vector3d.Up, Fixed64.Half, Vector3d.Down, Fixed64.One,
            PhysicsLayerMask.All, excluded, budget, scratch, out hit, out work);

    private static GravitasWorldContext CreateContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
        Assert.True(context.World.TryAddGrid(
            new GridConfiguration(new Vector3d(-4, -4, -4), new Vector3d(4, 4, 4)), out _));
        return context;
    }

    private static LSCuboidCollider AddBoxFloor(GravitasWorldContext context)
    {
        var floor = new LSCuboidCollider { Size = new Vector3d(3, 1, 3) };
        floor.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(-Vector3d.Up * Fixed64.Half, FixedQuaternion.Identity, Vector3d.One)));
        return floor;
    }
}
