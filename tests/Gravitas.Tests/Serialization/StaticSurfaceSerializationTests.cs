using Chronicler;
using Chronicler.Serialization;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Tests.Determinism;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Serialization;

public sealed class StaticSurfaceSerializationTests
{
    public static TheoryData<bool, GravitasSerializationTransport> Profiles
    {
        get
        {
            var data = new TheoryData<bool, GravitasSerializationTransport>();
            foreach (bool planar in new[] { false, true })
            {
                data.Add(planar, GravitasSerializationTransport.Json);
#if !GRAVITAS_DISABLE_MEMORYPACK
                data.Add(planar, GravitasSerializationTransport.MemoryPack);
#endif
            }
            return data;
        }
    }

    [Theory]
    [MemberData(nameof(Profiles))]
    public void PopulateExisting_ShouldRestoreStableLinkedMembershipAcrossSlotLayouts(bool planar, GravitasSerializationTransport transport)
    {
        using var source = PhysicsScenarioBuilder.Create();
        using var target = PhysicsScenarioBuilder.Create();
        StaticSurfaceMembershipHashTests.Members(source, false, out var meshes, out var areas);
        StaticSurfaceMembershipHashTests.Members(target, true, out var restoredMeshes, out var restoredAreas);
        IRecordable original = StaticSurfaceMembershipHashTests.Declare(source.Context, planar, meshes[..2], areas[..2]);
        using var sourceLifetime = (IDisposable)original;
        IRecordable restored = StaticSurfaceMembershipHashTests.Declare(target.Context, planar, restoredMeshes[2..], restoredAreas[2..]);
        using var targetLifetime = (IDisposable)restored;
        object payload = GravitasSerializationHarness.Serialize(original, transport, Links(meshes, areas));
        GravitasSerializationHarness.Populate(restored, payload, transport, Links(restoredMeshes, restoredAreas));
        Assert.Equal(source.Context.ComputeReplayHash(), target.Context.ComputeReplayHash());
        for (int index = 0; index < 4; index++)
        {
            object? association = planar ? restoredAreas[index].StaticSurface : restoredMeshes[index].StaticSurface;
            if (index < 2) Assert.Same(restored, association);
            else Assert.Null(association);
        }
        StaticSurfaceMembershipHashTests.Replace(original, Array.Empty<LSMeshCollider>(), Array.Empty<LSCollider2D>());
        payload = GravitasSerializationHarness.Serialize(original, transport);
        GravitasSerializationHarness.Populate(restored, payload, transport);
        Assert.Equal(source.Context.ComputeReplayHash(), target.Context.ComputeReplayHash());
    }

    [Theory]
    [InlineData(false, 0)] [InlineData(true, 0)]
    [InlineData(false, 1)] [InlineData(true, 1)]
    [InlineData(false, 2)] [InlineData(true, 2)]
    [InlineData(false, 3)] [InlineData(true, 3)]
    [InlineData(false, 4)] [InlineData(true, 4)]
    [InlineData(false, 5)] [InlineData(true, 5)]
    [InlineData(false, 6)] [InlineData(true, 6)]
    [InlineData(false, 7)] [InlineData(true, 7)]
    [InlineData(false, 8)] [InlineData(true, 8)]
    public void RejectedRecord_ShouldKeepThePreviousMembership(bool planar, int invalid)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        using var foreign = PhysicsScenarioBuilder.Create();
        StaticSurfaceMembershipHashTests.Members(scene, false, out var meshes, out var areas);
        StaticSurfaceMembershipHashTests.Members(foreign, false, out var foreignMeshes, out var foreignAreas);
        IRecordable declaration = StaticSurfaceMembershipHashTests.Declare(scene.Context, planar, meshes[2..], areas[2..]);
        using var lifetime = (IDisposable)declaration;
        var before = scene.Context.ComputeReplayHash();
        var links = Links(meshes, areas);
        string payload = invalid switch
        {
            0 => "{}",
            1 => "{\"SurfaceSchemaVersion\":2,\"MemberCount\":0}",
            2 => "{\"SurfaceSchemaVersion\":1,\"MemberCount\":-1}",
            3 => "{\"SurfaceSchemaVersion\":1,\"MemberCount\":999}",
            4 => "{\"SurfaceSchemaVersion\":1,\"MemberCount\":2,\"Member0\":\"piece-0\",\"Member1\":\"missing\"}",
            5 => "{\"SurfaceSchemaVersion\":1,\"MemberCount\":2,\"Member0\":\"piece-0\",\"Member1\":\"piece-1\"}",
            6 => "{\"SurfaceSchemaVersion\":1,\"MemberCount\":2,\"Member0\":\"piece-0\",\"Member1\":\"piece-0\"}",
            7 => "{\"SurfaceSchemaVersion\":1}",
            _ => "{\"SurfaceSchemaVersion\":1,\"MemberCount\":2,\"Member0\":\"piece-0\",\"Member1\":null}"
        };
        if (invalid == 5) links = Links(foreignMeshes, foreignAreas);
        if (invalid is 0 or 1 or 4)
            Assert.Throws<InvalidOperationException>(() => JsonRecordSerializer.Populate(declaration, payload, links));
        else if (invalid is 2 or 3 or 7)
            Assert.Throws<ArgumentOutOfRangeException>(() => JsonRecordSerializer.Populate(declaration, payload, links));
        else Assert.Throws<ArgumentException>(() => JsonRecordSerializer.Populate(declaration, payload, links));
        Assert.Equal(before, scene.Context.ComputeReplayHash());
        for (int index = 0; index < 4; index++)
        {
            object? association = planar ? areas[index].StaticSurface : meshes[index].StaticSurface;
            if (index >= 2) Assert.Same(declaration, association);
            else Assert.Null(association);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void StableLinks_ShouldRejectAnUnloadedMemberAndBindItsHostReplacement(bool planar)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        StaticSurfaceMembershipHashTests.Members(scene, false, out var meshes, out var areas);
        IRecordable declaration = StaticSurfaceMembershipHashTests.Declare(scene.Context, planar, meshes[..2], areas[..2]);
        using var lifetime = (IDisposable)declaration;
        var links = Links(meshes, areas);
        string payload = JsonRecordSerializer.Serialize(declaration, links);
        StaticSurfaceMembershipHashTests.Replace(declaration, meshes[2..], areas[2..]);
        if (planar) areas[0].Deactivate(); else meshes[0].Deactivate();
        Assert.Throws<ArgumentException>(() => JsonRecordSerializer.Populate(declaration, payload, links));
        Assert.Same(declaration, planar ? areas[2].StaticSurface : meshes[2].StaticSurface);
        Assert.Same(declaration, planar ? areas[3].StaticSurface : meshes[3].StaticSurface);
        if (planar)
        {
            LSCollider2D replacement = new LSAABBoxCollider2D(new Vector2d(4, 4));
            replacement.InitializeWithNoBody(new TestMatterAgent(scene.Context,
                new FixedTransform(new(-6, 0, 0), FixedQuaternion.Identity, Vector3d.One)));
            links.Links.RegisterInstance("piece-0", replacement);
            JsonRecordSerializer.Populate(declaration, payload, links);
            Assert.Same(declaration, replacement.StaticSurface);
            Assert.Same(declaration, areas[1].StaticSurface);
            Assert.Null(areas[0].StaticSurface); Assert.Null(areas[2].StaticSurface); Assert.Null(areas[3].StaticSurface);
        }
        else
        {
            LSMeshCollider replacement = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
            scene.InitializeStaticCollider(replacement, new(-6, 0, 0));
            links.Links.RegisterInstance("piece-0", replacement);
            JsonRecordSerializer.Populate(declaration, payload, links);
            Assert.Same(declaration, replacement.StaticSurface);
            Assert.Same(declaration, meshes[1].StaticSurface);
            Assert.Null(meshes[0].StaticSurface); Assert.Null(meshes[2].StaticSurface); Assert.Null(meshes[3].StaticSurface);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Recording_ShouldRejectMissingLinksExpiredHandlesAndMidStepLoad(bool planar)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        StaticSurfaceMembershipHashTests.Members(scene, false, out var meshes, out var areas);
        IRecordable declaration = StaticSurfaceMembershipHashTests.Declare(scene.Context, planar, meshes[..2], areas[..2]);
        Assert.Throws<ArgumentNullException>(() => declaration.RecordData(null!));
        Assert.Throws<InvalidOperationException>(() => JsonRecordSerializer.Serialize(declaration));
        var links = Links(meshes, areas);
        string payload = JsonRecordSerializer.Serialize(declaration, links);
        scene.Context.Simulate();
        Assert.Throws<InvalidOperationException>(() => JsonRecordSerializer.Populate(declaration, payload, links));
        scene.Context.LateSimulate();
        ((IDisposable)declaration).Dispose();
        Assert.Throws<ObjectDisposedException>(() => JsonRecordSerializer.Populate(declaration, payload, links));
        IRecordable reset = StaticSurfaceMembershipHashTests.Declare(scene.Context, planar, meshes[..1], areas[..1]);
        scene.Context.Reset();
        Assert.Throws<ObjectDisposedException>(() => JsonRecordSerializer.Serialize(reset, links));
    }

    private static ChronicleContext Links(LSMeshCollider[] meshes, LSCollider2D[] areas)
    {
        var context = new ChronicleContext();
        for (int index = 0; index < meshes.Length; index++)
        {
            context.Links.RegisterInstance($"piece-{index}", meshes[index]);
            context.Links.RegisterInstance($"piece-{index}", areas[index]);
        }
        return context;
    }
}
