using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

public sealed class JoinedStaticSurfaceTests
{
    [Fact]
    public void DeclarationOrder_ShouldRepeatTheCompletePhysicalTrace()
    {
        Assert.Equal(Trace(false), Trace(true));

        static List<long> Trace(bool reverse)
        {
            using var scene = PhysicsScenarioBuilder.Create();
            scene.Context.SetFrameRate(16);
            scene.Context.Environment.Gravity = Fixed64.One;
            scene.Context.Environment.AirDensity = Fixed64.Zero;
            scene.Context.Environment.DampingFactor = Fixed64.Zero;
            scene.Context.Environment.MinSpeed = Fixed64.Zero;
            var members = new[] { Floor(scene, -8, 0, false), Floor(scene, 0, 8, false) };
            using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { members[1], members[0] } : members);
            var sphere = new LSSphereCollider { Radius = Fixed64.Half, Material = PhysicsMaterial.Frictionless };
            var body = scene.CreateBody(sphere, new((Fixed64)(-2), Fixed64.Half, Fixed64.Zero), FixedQuaternion.Identity).Body;
            body.UseManualGrounding(); body.SleepEnabled = false;
            var result = new List<long>();
            for (int frame = 0; frame < 96; frame++)
            {
                if (frame == 32) body.AddLinearImpulse(Vector3d.Right);
                scene.Context.Simulate(); scene.Context.LateSimulate();
                Add(body.Position3d); Add(body.LinearVelocity); Add(body.AngularVelocity);
                foreach (var member in members)
                {
                    if (!member.TryGetCollisionPair(sphere.Id, out var pair)) { result.Add(-1); continue; }
                    result.Add(pair!.Manifold.GroupCount);
                    foreach (var row in pair.Manifold)
                    {
                        result.Add(unchecked((long)row.ContactId)); result.Add(row.Depth.m_rawValue); Add(row.Normal);
                        Assert.True(row.AnchorA.TryGetWorldPoint(out var a)); Add(a);
                        Assert.True(row.AnchorB.TryGetWorldPoint(out var b)); Add(b);
                    }
                }
            }
            return result;

            void Add(Vector3d value)
            { result.Add(value.X.m_rawValue); result.Add(value.Y.m_rawValue); result.Add(value.Z.m_rawValue); }
        }
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, true, false)]
    [InlineData(true, false, false)]
    [InlineData(true, true, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, true)]
    [InlineData(true, false, true)]
    [InlineData(true, true, true)]
    public void Crossing_ShouldPreserveTangentialMotion(bool capsule, bool staticBodies, bool compound)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.SetFrameRate(16);
        scene.Context.Environment.Gravity = Fixed64.One;
        scene.Context.Environment.AirDensity = Fixed64.Zero;
        scene.Context.Environment.DampingFactor = Fixed64.Zero;
        scene.Context.Environment.MinSpeed = Fixed64.Zero;
        LSMeshCollider left = Floor(scene, -8, 0, staticBodies);
        LSMeshCollider right = Floor(scene, 0, 8, staticBodies);
        using var surface = scene.Context.Physics.CreateStaticSurface(capsule ? new[] { right, left } : new[] { left, right });
        int enters = 0, exits = 0;
        left.OnContactEnter += _ => enters++;
        right.OnContactEnter += _ => enters++;
        left.OnContactExit += _ => exits++;
        right.OnContactExit += _ => exits++;
        LSCollider shape = capsule
            ? new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) }
            : new LSSphereCollider { Radius = Fixed64.Half };
        if (compound) shape = new LSCompoundCollider(capsule
            ? CompoundColliderPart.Capsule(Fixed64.Half, Fixed64.Two, Vector3d.Zero, PhysicsMaterial.Frictionless)
            : CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero, PhysicsMaterial.Frictionless));
        shape.Material = PhysicsMaterial.Frictionless;
        var moving = scene.CreateBody(shape, new((Fixed64)(-2), capsule ? Fixed64.One : Fixed64.Half, Fixed64.Zero),
            FixedQuaternion.Identity, preventAngularForces: staticBodies).Body;
        moving.UseManualGrounding();
        moving.SleepEnabled = false;
        for (int frame = 0; frame < 32; frame++) Step();
        moving.AddLinearImpulse(Vector3d.Right);
        for (int frame = 0; frame < 64; frame++)
        {
            Step();
            Assert.Equal(Fixed64.One, moving.LinearVelocity.X);
            Assert.Equal(Fixed64.Zero, moving.LinearVelocity.Z);
            Assert.Equal(Vector3d.Zero, moving.AngularVelocity);
        }
        Assert.True(moving.Position3d.X > Fixed64.One);
        Assert.Equal(2, enters);
        Assert.Equal(1, exits);
        bool valid = true;
        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            for (int frame = 0; frame < 64; frame++)
            {
                if (moving.Position3d.X >= Fixed64.Two && moving.LinearVelocity.X > Fixed64.Zero)
                    moving.AddLinearImpulse(-Vector3d.Right * Fixed64.Two);
                else if (moving.Position3d.X <= -Fixed64.Two && moving.LinearVelocity.X < Fixed64.Zero)
                    moving.AddLinearImpulse(Vector3d.Right * Fixed64.Two);
                Step();
                valid &= FixedMath.Abs(moving.LinearVelocity.X) == Fixed64.One;
            }
        }, warmupIterations: 4, stabilizationIterations: 2, measurementIterations: 4);
        Assert.True(valid);
        Assert.Equal(0, allocated);

        void Step()
        {
            scene.Context.Simulate();
            scene.Context.LateSimulate();
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExposedRim_ShouldRetainRadialContact(bool corner)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSMeshCollider left = Floor(scene, -8, 0, false), right = Floor(scene, 0, 8, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var sphere = new LSSphereCollider { Radius = Fixed64.Half };
        scene.CreateBody(sphere, new((Fixed64)(-8) - Fixed64.Quarter, Fixed64.Quarter,
            corner ? (Fixed64)(-4) - Fixed64.Quarter : Fixed64.Zero), FixedQuaternion.Identity);
        var pair = scene.CreatePair(left, sphere);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        bool radial = false;
        foreach (var contact in pair.Manifold)
        {
            Assert.True(contact.AnchorA.TryGetWorldPoint(out var anchor));
            Assert.True(anchor.X >= (Fixed64)(-8) && anchor.X <= Fixed64.Zero);
            if (contact.Normal.X != Fixed64.Zero)
            {
                Assert.Equal((Fixed64)(-8), anchor.X);
                radial = true;
            }
        }
        Assert.True(radial);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SharedOuterEndpoint_ShouldHaveOneRadialSupplier(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Floor(scene, -8, 0, false);
        var right = Floor(scene, 0, 8, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { right, left });
        LSCollider shape = capsule ? new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) }
            : new LSSphereCollider { Radius = Fixed64.Half };
        scene.CreateBody(shape, new(Fixed64.Zero, Fixed64.Quarter, (Fixed64)4 + Fixed64.Quarter), FixedQuaternion.Identity);
        int radial = 0;
        foreach (var member in new[] { left, right })
        {
            var pair = scene.CreatePair(member, shape);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            foreach (var row in pair.Manifold)
                if (row.Normal.Z != Fixed64.Zero) { radial++; Assert.Same(left, member); }
        }
        Assert.Equal(1, radial);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SharedCornerAcrossDisconnectedOwners_ShouldHaveOneRadialSupplier(bool capsule)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var vertices = new[] { Vector3d.Zero, new Vector3d(4, 0, 0), new(4, 0, 1), new(4, 0, 2), new(0, 0, 4) };
        var outer = new LSMeshCollider(vertices, new[] { 0, 2, 1, 0, 4, 3 }, MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        var middle = new LSMeshCollider(new[] { vertices[0], vertices[3], vertices[2] }, new[] { 0, 1, 2 }, MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(outer, Vector3d.Zero);
        scene.InitializeStaticCollider(middle, Vector3d.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { middle, outer });
        LSCollider shape = capsule ? new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) }
            : new LSSphereCollider { Radius = Fixed64.Half };
        scene.CreateBody(shape, capsule ? Vector3d.Zero : new(-Fixed64.Quarter, Fixed64.Quarter, -Fixed64.Quarter), FixedQuaternion.Identity);
        for (int repeat = 0; repeat < 2; repeat++)
        {
            int radial = 0;
            foreach (var member in new[] { outer, middle })
            {
                var pair = scene.CreatePair(member, shape);
                Assert.True(CollisionDetection.DoCollisionCheck(pair));
                foreach (var row in pair.Manifold)
                    if (row.Normal.X != Fixed64.Zero || row.Normal.Z != Fixed64.Zero)
                    {
                        radial++;
                        Assert.Same(outer, member);
                        Assert.True(row.AnchorA.TryGetWorldPoint(out var point));
                        Assert.Equal(Vector3d.Zero, point);
                    }
            }
            Assert.Equal(1, radial);
        }
    }

    [Fact]
    public void Replacement_ShouldInvalidateRemovedMemberAndRejectPartialJoinAtomically()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSMeshCollider left = Floor(scene, -8, 0, false), right = Floor(scene, 0, 8, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        uint before = right.ContactDependencyVersion;
        surface.ReplaceMembers(new[] { left });
        Assert.Null(right.StaticSurface);
        Assert.True(right.ContactDependencyVersion > before);
        surface.ReplaceMembers(new[] { left, right });
        var partial = new LSMeshCollider(new[] { new Vector3d(0, 0, -1), new(1, 0, -1), new(0, 0, 1) },
            new[] { 0, 2, 1 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(partial, Vector3d.Zero);
        Assert.Throws<NotSupportedException>(() => surface.ReplaceMembers(new[] { left, partial }));
        Assert.Same(surface, left.StaticSurface);
        Assert.Same(surface, right.StaticSurface);
        Assert.Null(partial.StaticSurface);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DisabledOrTriggerNeighbor_ShouldExposeTheSeam(bool trigger)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSMeshCollider left = Floor(scene, -8, 0, false), right = Floor(scene, 0, 8, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var sphere = new LSSphereCollider { Radius = Fixed64.Half };
        scene.CreateBody(sphere, new(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        var pair = scene.CreatePair(left, sphere);
        pair.UpdateCollisionDeferred();
        foreach (var row in pair.Manifold) Assert.Equal(Fixed64.Zero, row.Normal.X);
        if (trigger) right.IsTrigger = true;
        else right.IsActive = false;
        pair.UpdateCollisionDeferred();
        bool radial = false;
        foreach (var row in pair.Manifold) radial |= row.Normal.X != Fixed64.Zero;
        Assert.True(radial);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    [InlineData(7)]
    public void InvalidReplacement_ShouldPreserveUsableMembership(int invalidKind)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        using var foreign = PhysicsScenarioBuilder.Create();
        LSMeshCollider left = Floor(scene, -8, 0, false), right = Floor(scene, 0, 8, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        LSMeshCollider candidate = invalidKind == 2 ? Floor(foreign, 0, 8, false) : Floor(scene, 8, 12, true);
        using var otherSurface = scene.Context.Physics.CreateStaticSurface(invalidKind == 6 ? new[] { candidate } : Array.Empty<LSMeshCollider>());
        if (invalidKind == 3) candidate.Body!.SetMotionType(BodyMotionType.Dynamic);
        if (invalidKind == 4) candidate.IsActive = false;
        if (invalidKind == 5)
        {
            candidate = new LSMeshCollider(new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Forward }, new[] { 0, 1, 2 }, MeshColliderMode.Convex);
            scene.InitializeStaticCollider(candidate, Vector3d.Zero);
        }
        if (invalidKind == 7) candidate = new LSMeshCollider(new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Forward }, new[] { 0, 1, 2 });
        LSMeshCollider[] invalid = invalidKind == 0 ? new LSMeshCollider[] { null! }
            : invalidKind == 1 ? new[] { left, left } : new[] { left, candidate };
        Assert.Throws<ArgumentException>(() => surface.ReplaceMembers(invalid));
        Assert.Same(surface, left.StaticSurface);
        Assert.Same(surface, right.StaticSurface);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ResetOrDispose_ShouldEndHandleLifetime(bool dispose)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSMeshCollider member = Floor(scene, -8, 0, false);
        var surface = scene.Context.Physics.CreateStaticSurface(new[] { member });
        if (dispose) scene.Context.Dispose();
        else scene.Context.Reset();
        Assert.Null(member.StaticSurface);
        Assert.Throws<ObjectDisposedException>(() => surface.ReplaceMembers(Array.Empty<LSMeshCollider>()));
        surface.Dispose();
        surface.Dispose();
    }

    [Fact]
    public void MembershipMutation_ShouldRequireTheFullStepBoundary()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var member = Floor(scene, -8, 0, false);
        var surface = scene.Context.Physics.CreateStaticSurface(new[] { member });
        scene.Context.Simulate();
        Assert.Throws<InvalidOperationException>(() => surface.ReplaceMembers(Array.Empty<LSMeshCollider>()));
        Assert.Throws<InvalidOperationException>(() => surface.Dispose());
        scene.Context.LateSimulate();
        surface.Dispose();
        Assert.Throws<ObjectDisposedException>(() => surface.ReplaceMembers(new[] { member }));
        using var replacement = scene.Context.Physics.CreateStaticSurface(new[] { member });
        Assert.Same(replacement, member.StaticSurface);
    }

    [Fact]
    public void DeactivationAndRoleChange_ShouldRemoveMembershipBeforeReuse()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSMeshCollider left = Floor(scene, -8, 0, true), right = Floor(scene, 0, 8, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        left.Body!.SetMotionType(BodyMotionType.Dynamic);
        Assert.Null(left.StaticSurface);
        right.Deactivate();
        Assert.Null(right.StaticSurface);
        surface.Remove(right); // Repeated cleanup must not affect a later lifetime.
        scene.InitializeStaticCollider(right, Vector3d.Zero);
        Assert.Null(right.StaticSurface);
        surface.ReplaceMembers(new[] { right });
        Assert.Same(surface, right.StaticSurface);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void ChangedNeighborEligibilityOrGeometry_ShouldRefreshRetainedRows(int change)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSMeshCollider left = Floor(scene, -8, 0, false), right = Floor(scene, 0, 8, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var sphere = new LSSphereCollider { Radius = Fixed64.Half };
        scene.CreateBody(sphere, new(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        var pair = scene.CreatePair(left, sphere);
        if (change == 1)
        {
            scene.Context.ApplySettings(new PhysicsSettings(16, new[,] { { true, true }, { true, true } }));
            right.Layer = new Gravitas.Support.PhysicsLayer(1);
        }
        pair.UpdateCollisionDeferred();
        if (change == 0) right.IgnoredCollisionLayers = Gravitas.Support.PhysicsLayerMask.FromLayer(sphere.Layer.Index);
        else if (change == 1)
        {
            scene.Context.Settings.CollisionMatrix[0, 1] = false;
            scene.Context.Settings.CollisionMatrix[1, 0] = false;
        }
        else right.Position = new Vector3d(2, 0, 0);
        pair.UpdateCollisionDeferred();
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        bool radial = false;
        foreach (var row in pair.Manifold) radial |= row.Normal.X != Fixed64.Zero;
        Assert.True(radial);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void JoinedVertex_ShouldKeepOnlyGenuineBoundaryConstraints(bool concave)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var pieces = new LSMeshCollider[concave ? 3 : 4];
        for (int index = 0; index < pieces.Length; index++)
        {
            pieces[index] = Floor(scene, index % 2 == 0 ? -2 : 0, index % 2 == 0 ? 0 : 2, false,
                index < 2 ? -2 : 0, index < 2 ? 0 : 2);
        }
        using var surface = scene.Context.Physics.CreateStaticSurface(pieces);
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) };
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        scene.CreateBody(capsule, Vector3d.Zero, rotation);
        foreach (var piece in pieces)
        {
            var pair = scene.CreatePair(piece, capsule);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            foreach (var row in pair.Manifold)
            {
                Assert.Equal(Fixed64.Zero, row.Normal.X);
                Assert.Equal(Fixed64.Zero, row.Normal.Z);
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ResetRegistration_ShouldRejectAStaleColliderEvenIfItsIdWasReused(bool reuseId)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var stale = Floor(scene, -8, 0, false);
        scene.Context.Reset();
        if (reuseId) Floor(scene, 0, 8, false);
        Assert.Throws<ArgumentException>(() => scene.Context.Physics.CreateStaticSurface(new[] { stale }));
    }

    [Fact]
    public void ExitCallback_ShouldNotReadmitADeactivatingMember()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var member = Floor(scene, -8, 0, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { member });
        var sphere = new LSSphereCollider { Radius = Fixed64.Half };
        scene.CreateBody(sphere, new(-Fixed64.One, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        var pair = scene.Context.Physics.GetCollisionPair(member.Id, sphere.Id)!;
        pair.UpdateCollisionDeferred(); pair.NotifyCollidersOfContact();
        bool attempted = false;
        member.OnContactExit += _ =>
        {
            attempted = true;
            Assert.Throws<ArgumentException>(() => scene.Context.Physics.CreateStaticSurface(new[] { member }));
        };
        member.Deactivate();
        Assert.True(attempted);
        Assert.Null(member.StaticSurface);
    }

    [Fact]
    public void SleepingAtAJoin_ShouldRetainBothOverlapLifetimesAndWakeOnNeighborChange()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Floor(scene, -8, 0, false);
        var right = Floor(scene, 0, 8, false);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right });
        var sphere = new LSSphereCollider { Radius = Fixed64.Half };
        var body = scene.CreateBody(sphere, new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity).Body;
        body.GravityScale = Fixed64.Zero; body.UseManualGrounding();
        scene.Context.Simulate(); scene.Context.LateSimulate();
        int exits = 0;
        left.OnContactExit += _ => exits++;
        right.OnContactExit += _ => exits++;
        body.Sleep();
        for (int frame = 0; frame < 32; frame++) { scene.Context.Simulate(); scene.Context.LateSimulate(); }
        Assert.True(body.IsSleeping);
        Assert.Equal(0, exits);
        Assert.True(left.TryGetCollisionPair(sphere.Id, out _));
        Assert.True(right.TryGetCollisionPair(sphere.Id, out _));
        right.IsTrigger = true;
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.False(body.IsSleeping);
    }

    [Fact]
    public void LocalGather_ShouldExcludeUnrelatedAndVerticallySeparatedPieces()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Floor(scene, -8, 0, false);
        var right = Floor(scene, 0, 8, false);
        var above = Floor(scene, -8, 0, false);
        var below = Floor(scene, 0, 8, false);
        above.Position = new Vector3d(0, 4, 0);
        below.Position = new Vector3d(0, -4, 0);
        var unrelated = Floor(scene, -8, 0, false);
        unrelated.IgnoredCollisionLayers = Gravitas.Support.PhysicsLayerMask.FromLayer(0);
        unrelated.IsTrigger = true;
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { left, right, above, below });
        left.Material = PhysicsMaterial.Bouncy;
        var sphere = new LSSphereCollider { Radius = Fixed64.Half };
        scene.CreateBody(sphere, new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        var pair = scene.CreatePair(left, sphere);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        Assert.True(pair.Manifold.HasContact);
        foreach (var row in pair.Manifold)
        {
            Assert.Equal(left.Material, row.MaterialA);
            Assert.Equal(sphere.Material, row.MaterialB);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SharedOuterCorner_ShouldPreserveExcludedOrAmbiguousNeighborRims(bool ambiguous)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Floor(scene, -2, 0, false, -2, 0);
        var right = Floor(scene, 0, 2, false, -2, 0);
        var top = Floor(scene, -2, 0, false, 0, 2);
        LSMeshCollider[] members;
        if (ambiguous) members = new[] { left, right, Floor(scene, 0, 2, false, -2, 0) };
        else
        {
            top.IsTrigger = true;
            members = new[] { left, right, top };
        }
        using var surface = scene.Context.Physics.CreateStaticSurface(members);
        var capsule = new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) };
        scene.CreateBody(capsule, new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        bool found = false;
        foreach (var member in members)
        {
            if (member.IsTrigger) continue;
            var pair = scene.CreatePair(member, capsule);
            found |= CollisionDetection.DoCollisionCheck(pair);
            foreach (var row in pair.Manifold)
            {
                Assert.True(row.AnchorA.TryGetWorldPoint(out var point));
                Assert.Equal(Fixed64.Zero, point.Y);
            }
        }
        Assert.True(found);
    }

    private static LSMeshCollider Floor(PhysicsScenarioBuilder scene, int left, int right, bool body, int near = -4, int far = 4)
    {
        var mesh = new LSMeshCollider(new[] { new Vector3d(left, 0, near), new(right, 0, near),
            new(left, 0, far), new(right, 0, far) }, new[] { 0, 2, 1, 1, 2, 3 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
        if (body) scene.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        else scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        return mesh;
    }
}
