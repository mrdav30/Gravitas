using Chronicler;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Determinism;

public sealed class StaticSurfaceMembershipHashTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Membership_ShouldBeAuthoritativeWhileEmptyHandlesHaveNoMembership(bool planar)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Members(scene, false, out LSMeshCollider[] meshes, out LSCollider2D[] areas);
        var original = scene.Context.ComputeReplayHash();
        IRecordable declaration = Declare(scene.Context, planar, Array.Empty<LSMeshCollider>(), Array.Empty<LSCollider2D>());
        using var lifetime = (IDisposable)declaration;
        Assert.Equal(original, scene.Context.ComputeReplayHash());
        Replace(declaration, meshes[..2], areas[..2]);
        var joined = scene.Context.ComputeReplayHash();
        Assert.NotEqual(original, joined);
        Replace(declaration, meshes[..1], areas[..1]);
        Assert.NotEqual(original, scene.Context.ComputeReplayHash());
        Assert.NotEqual(joined, scene.Context.ComputeReplayHash());
        Replace(declaration, Array.Empty<LSMeshCollider>(), Array.Empty<LSCollider2D>());
        Assert.Equal(original, scene.Context.ComputeReplayHash());
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EquivalentMembership_ShouldIgnoreHandleMemberOrderAndRecycledSlots(bool planar)
    {
        using var compact = PhysicsScenarioBuilder.Create();
        using var recycled = PhysicsScenarioBuilder.Create();
        Members(compact, false, out LSMeshCollider[] a, out LSCollider2D[] a2);
        Members(recycled, true, out LSMeshCollider[] b, out LSCollider2D[] b2);
        Assert.NotEqual(a[0].Id, b[0].Id); Assert.NotEqual(a2[0].Id, b2[0].Id);
        using var first = (IDisposable)Declare(compact.Context, planar, a[..2], a2[..2]);
        using var second = (IDisposable)Declare(compact.Context, planar, a[2..], a2[2..]);
        using var reverseSecond = (IDisposable)Declare(recycled.Context, planar, new[] { b[3], b[2] }, new[] { b2[3], b2[2] });
        using var reverseFirst = (IDisposable)Declare(recycled.Context, planar, new[] { b[1], b[0] }, new[] { b2[1], b2[0] });
        Assert.Equal(compact.Context.ComputeReplayHash(), recycled.Context.ComputeReplayHash());
        // A different partition of the same colliders is a different declaration.
        Replace((IRecordable)reverseFirst, new[] { b[0] }, new[] { b2[0] });
        Assert.NotEqual(compact.Context.ComputeReplayHash(), recycled.Context.ComputeReplayHash());
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MembershipHash_ShouldNotAllocateAfterWarmup(bool planar)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Members(scene, false, out LSMeshCollider[] meshes, out LSCollider2D[] areas);
        using var declaration = (IDisposable)Declare(scene.Context, planar, meshes, areas);
        long bytes = AllocationTestHelper.MeasureSteadyState(() =>
        {
            _ = scene.Context.ComputeReplayHash();
            _ = scene.Context.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches);
        });
        Assert.Equal(0, bytes);
    }

    internal static void Members(PhysicsScenarioBuilder scene, bool recycled,
        out LSMeshCollider[] meshes, out LSCollider2D[] areas)
    {
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        if (recycled)
        {
            var first = new LSSphereCollider(); var second = new LSSphereCollider();
            scene.InitializeStaticCollider(first, new(12, 0, 12)); scene.InitializeStaticCollider(second, new(13, 0, 12));
            first.Deactivate(); second.Deactivate();
            var a = new LSAABBoxCollider2D(Vector2d.One); var b = new LSAABBoxCollider2D(Vector2d.One);
            a.InitializeWithNoBody(new TestMatterAgent(scene.Context)); b.InitializeWithNoBody(new TestMatterAgent(scene.Context));
            a.Deactivate(); b.Deactivate();
        }
        meshes = new LSMeshCollider[4]; areas = new LSCollider2D[4];
        for (int index = 0; index < 4; index++)
        {
            meshes[index] = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
            scene.InitializeStaticCollider(meshes[index], new(index * 4 - 6, 0, 0));
            areas[index] = new LSAABBoxCollider2D(new Vector2d(4, 4));
            areas[index].InitializeWithNoBody(new TestMatterAgent(scene.Context,
                new FixedTransform(new(index * 4 - 6, 0, 0), FixedQuaternion.Identity, Vector3d.One)));
        }
        scene.InitializeStaticCollider(new LSSphereCollider(), new(12, 0, 12));
    }

    internal static IRecordable Declare(GravitasWorldContext context, bool planar,
        LSMeshCollider[] meshes, LSCollider2D[] areas) => planar
            ? context.Physics2D.CreateStaticSurface(areas) : context.Physics.CreateStaticSurface(meshes);

    internal static void Replace(IRecordable declaration, LSMeshCollider[] meshes, LSCollider2D[] areas)
    {
        if (declaration is StaticSurface spatial) spatial.ReplaceMembers(meshes);
        else ((StaticSurface2D)declaration).ReplaceMembers(areas);
    }
}
