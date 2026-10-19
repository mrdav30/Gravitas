using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Support;
using SwiftCollections;
using Gravitas.Tests.Support;
using System.Numerics;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class MeshAuthoredFrameTests
{
    [Theory]
    [InlineData(0, false, 4)]
    [InlineData(45, false, 4)]
    [InlineData(0, true, 5)]
    [InlineData(45, true, 5)]
    public void IndependentlyCenteredPieces_ShouldRetainSharedAuthoredEdgeAndHostResponse(
        int degrees, bool fractionalScale, int rightEdge)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Environment.Gravity = scene.Context.Environment.AirDensity =
            scene.Context.Environment.DampingFactor = Fixed64.Zero;
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero, Fixed64.Zero, (Fixed64)degrees);
        Vector3d scale = fractionalScale
            ? new(Fixed64.FromFraction(2, 3), Fixed64.One, Fixed64.FromFraction(3, 5))
            : Vector3d.One;
        Fixed64 seam = Fixed64.FromFraction(9, 10);
        LSMeshCollider first = CreateFloor(scene, (Fixed64)(-4), seam, rotation, scale);
        LSMeshCollider second = CreateFloor(scene, seam, (Fixed64)rightEdge, rotation, scale);

        Assert.True(first.Mesh.TryGetVertexWorld(1, out Vector3d left));
        Assert.True(second.Mesh.TryGetVertexWorld(0, out Vector3d right));
        Assert.Equal(left, right);
        // Bodyless hosts commit the existing matrix-decomposed pose. Check the
        // authored mapping in that pose, before independent mesh recentering.
        FixedQuaternion committedRotation = first.Mesh.Rotation;
        Assert.Equal(committedRotation, second.Mesh.Rotation);
        Assert.True(committedRotation.TryTransformScaledPoint(Vector3d.Zero,
            new Vector3d(seam, Fixed64.Zero, (Fixed64)(-4)), first.Mesh.OwnerScale, out Vector3d expected));
        Assert.Equal(expected, left);

        var domain = new MeshSurfaceBoundaryDomain();
        domain.Build(new[] { first.Mesh, second.Mesh }, 0, 0);
        int joined = 0;
        foreach (var span in domain.Spans)
            if (span.Kind == MeshSurfaceBoundaryDomain.SpanKind.Joined) joined++;
        Assert.Equal(1, joined);

        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { first, second });
        var cube = new LSCuboidCollider { Size = new Vector3d(4, 1, 2), Material = PhysicsMaterial.Frictionless };
        Vector3d normal = committedRotation.Rotate(Vector3d.Up);
        Vector3d start = normal / (Fixed64)10;
        SolidBody body = scene.CreateBody(cube, start, FixedQuaternion.Identity,
            preventAngularForces: true).Body;
        body.SleepEnabled = false;
        body.UseManualGrounding();
        int rows = 0;
        foreach (LSMeshCollider member in new[] { first, second })
        {
            var pair = scene.CreatePair(member, cube);
            if (CollisionDetection.DoCollisionCheck(pair)) rows += pair.Manifold.Count;
        }
        Assert.True(rows > 0);
        scene.Context.Simulate();
        scene.Context.LateSimulate();
        Assert.True(Vector3d.Dot(body.Position3d - start, normal) > Fixed64.Zero);
    }

    [Fact]
    public void AuthoredAffineMesh_ShouldRetainPhysicalRaySweepProjectionAndConeWitnesses()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        LSMeshCollider floor=CreateFloor(scene,(Fixed64)(-4),(Fixed64)5,FixedQuaternion.Identity,Vector3d.One/4);
        var ray=new RaycastSegmentWorker();var hits=new SwiftList<Vector3d>();
        ray.PrepareSegmentCheck(Vector3d.Up*2,-Vector3d.Up*2);
        Assert.True(floor.ColliderOverlapsRay(ray,ref hits));
        Assert.All(hits,point=>Assert.Equal(Fixed64.Zero,point.Y));
        var sweep=new SweptSphereQueryWorker();
        sweep.Prepare(Vector3d.Up*2,-Vector3d.Up*2,Fixed64.Quarter);
        Assert.True(sweep.TrySweep(floor,out Vector3d center,out Fixed64 distance));
        Fixed64 tolerance=Fixed64.FromFraction(1,1024);
        Assert.InRange(center.Y.m_rawValue,(Fixed64.Quarter-tolerance).m_rawValue,(Fixed64.Quarter+tolerance).m_rawValue);
        Assert.InRange(distance.m_rawValue,((Fixed64)7/4-tolerance).m_rawValue,((Fixed64)7/4+tolerance).m_rawValue);
        Assert.True(ColliderPlanarProjection.TryGetRelation(floor,new Vector2d((Fixed64)3/2,Fixed64.Zero),
            Fixed64.Quarter,out ProjectedSurfaceRelation relation));
        Assert.Equal(Fixed64.Quarter,relation.Distance);
        Assert.True(relation.ContactAnchor.TryGetPoint(out Vector3d projected));
        Assert.Equal(new Vector3d((Fixed64)5/4,Fixed64.Zero,Fixed64.Zero),projected);

        var wall=new LSMeshCollider(new[]{new Vector3d(4,-8,-16),new Vector3d(4,-8,16),new Vector3d(4,24,0)},
            new[]{0,1,2},MeshColliderMode.Concave,MeshInertiaPolicy.SurfaceApproximation);
        wall.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(Vector3d.Zero,
            FixedQuaternion.Identity,Vector3d.One/4)));
        // Move the cone above the floor so the finite wall owns the result.
        Assert.True(scene.Context.Query3D.OverlapCone(Vector3d.Up/4,Vector3d.Up,(Fixed64)4,(Fixed64)2,
            out Physics3DHit hit,PhysicsLayerMask.All));
        Assert.Same(wall,hit.Collider);
        Assert.Equal(new Vector3d(Fixed64.One,(Fixed64)9/4,Fixed64.Zero),hit.Point);
        Assert.Equal(Fixed64.Two,hit.Distance);
    }

    [Fact]
    public void AuthoredNonlatticeMesh_SphereSweep_ShouldRetainTheFiniteRootBeforeDistanceRounding()
    {
        using var scene=PhysicsScenarioBuilder.Create();
        Fixed64 x=Fixed64.MinIncrement;
        var mesh=new LSMeshCollider(new[]{new Vector3d(x,(Fixed64)(-4),(Fixed64)(-4)),
            new Vector3d(x,(Fixed64)4,(Fixed64)(-4)),new Vector3d(x,Fixed64.Zero,(Fixed64)4)},
            new[]{0,1,2},MeshColliderMode.Concave,MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(Vector3d.Zero,
            FixedQuaternion.Identity,new Vector3d((Fixed64)3/2,Fixed64.One,Fixed64.One))));
        var sweep=new SweptSphereQueryWorker();
        sweep.Prepare(-Vector3d.Right*2,Vector3d.Right*2,Fixed64.Half);
        Assert.True(sweep.TrySweep(mesh,out Vector3d center,out Fixed64 distance));
        Assert.Equal(Fixed64.FromFraction(3,2).m_rawValue+2,distance.m_rawValue);
        Assert.Equal(new FixedSegment(-Vector3d.Right*2,Vector3d.Right*2)
            .GetPointAtDistance(distance,(Fixed64)4),center);
        Assert.Equal(1,sweep.LastMeshTriangleCandidateCount);
    }

    [Theory]
    [InlineData(0,true)]
    [InlineData(1,true)]
    [InlineData(2,true)]
    [InlineData(0,false)]
    [InlineData(1,false)]
    [InlineData(2,false)]
    public void AuthoredAffineMesh_PostureAndHostResponse_ShouldRetainFinitePhysicalPenetration(int shape,bool overlaps)
    {
        using var scene=PhysicsScenarioBuilder.Create();
        scene.Context.Environment.Gravity=scene.Context.Environment.AirDensity=scene.Context.Environment.DampingFactor=Fixed64.Zero;
        LSMeshCollider floor=CreateFloor(scene,(Fixed64)(-4),(Fixed64)5,FixedQuaternion.Identity,Vector3d.One/4);
        LSCollider candidate=shape switch
        {
            0=>new LSSphereCollider{Radius=Fixed64.Quarter},
            1=>new LSCapsuleCollider{Radius=Fixed64.Quarter,Size=new Vector3d(1,1,1)},
            _=>new LSCylinderCollider{Radius=Fixed64.Quarter,Size=new Vector3d(1,1,1)}
        };
        // Keep this ingress/host fixture inside one finite face. Circular
        // response at the internal diagonal is a separately reproduced defect.
        Vector3d position=new(-Fixed64.Half,overlaps?Fixed64.FromFraction(1,10):(Fixed64)2,Fixed64.Zero);
        SolidBody body=scene.CreateBody(candidate,position,FixedQuaternion.Identity,preventAngularForces:true).Body;
        body.UseManualGrounding();body.SleepEnabled=false;
        Assert.Equal(overlaps,CollisionDetection.DoesPostureCandidatePenetrate(candidate,floor));
        var pair=scene.CreatePair(floor,candidate);
        Assert.Equal(overlaps,CollisionDetection.DoCollisionCheck(pair));
        scene.Context.Simulate();scene.Context.LateSimulate();
        if(overlaps)Assert.True(body.Position3d.Y>position.Y,
            $"shape {shape}, normal {pair.Manifold.PrimaryContact.Normal}, depth {pair.Manifold.PrimaryContact.Depth}, before {position}, after {body.Position3d}");
        else Assert.Equal(position,body.Position3d);
    }

    [Theory]
    [InlineData(0,false)]
    [InlineData(1,false)]
    [InlineData(2,false)]
    [InlineData(3,false)]
    [InlineData(4,false)]
    [InlineData(0,true)]
    [InlineData(1,true)]
    [InlineData(2,true)]
    [InlineData(3,true)]
    [InlineData(4,true)]
    public void MeshRay_ShouldRetainClosedPointAndCoplanarFiniteAdmissionInEitherGeometryChart(int fixture,bool affine)
    {
        using var scene=PhysicsScenarioBuilder.Create();
        var mesh=new LSMeshCollider(new[]{Vector3d.Zero,new Vector3d(4,0,0),new Vector3d(0,0,4)},
            new[]{0,1,2},MeshColliderMode.Concave,MeshInertiaPolicy.SurfaceApproximation);
        Fixed64 scale=affine?Fixed64.Quarter:Fixed64.One;
        mesh.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(Vector3d.Zero,
            FixedQuaternion.Identity,Vector3d.One*scale)));
        Vector3d start=fixture switch{1=>new(3,0,3),3=>new(3,0,3),4=>new(3,1,3),_=>new(1,0,1)};
        Vector3d end=fixture switch{2=>new(2,0,1),3=>new(1,0,1),4=>new(3,-1,3),_=>start};
        start*=scale;end*=scale;
        var worker=new RaycastSegmentWorker();var hits=new SwiftList<Vector3d>();
        foreach(bool materialize in new[]{false,true})
        {
            hits.FastClear();worker.PrepareSegmentCheck(start,end,materialize);
            bool found=worker.CheckMeshOverlaps(mesh,ref hits);Assert.Equal(fixture!=1&&fixture!=4,found);
            if(!found||!materialize){Assert.Empty(hits);continue;}
            Assert.NotEmpty(hits);
            foreach(var point in hits)
            {
                Assert.Equal(Fixed64.Zero,point.Y);Assert.True(point.X>=Fixed64.Zero&&point.Z>=Fixed64.Zero);
                Assert.True(point.X+point.Z<=(Fixed64)4*scale);
            }
            Assert.Contains(end,hits);
        }
    }

    [Fact]
    public void TiltedMeshRay_ParallelInsideItsBounds_ShouldRejectTheSeparatedPlane()
    {
        using var scene=PhysicsScenarioBuilder.Create();
        var mesh=new LSMeshCollider(new[]{Vector3d.Zero,new Vector3d(4,0,0),new Vector3d(0,4,4)},
            new[]{0,1,2},MeshColliderMode.Concave,MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(mesh,Vector3d.Zero);
        var worker=new RaycastSegmentWorker();var hits=new SwiftList<Vector3d>();
        worker.PrepareSegmentCheck(new Vector3d(Fixed64.Half,Fixed64.One,Fixed64.Half),
            new Vector3d(Fixed64.One,Fixed64.One,Fixed64.Half));
        Assert.False(worker.CheckMeshOverlaps(mesh,ref hits));Assert.Empty(hits);
        Vector3d point=new(Fixed64.Half,Fixed64.One,Fixed64.Half);
        worker.PrepareSegmentCheck(point,point);
        Assert.False(worker.CheckMeshOverlaps(mesh,ref hits));Assert.Empty(hits);
    }

    [Theory]
    [InlineData(0,false)]
    [InlineData(1,false)]
    [InlineData(2,false)]
    [InlineData(3,false)]
    [InlineData(4,false)]
    [InlineData(5,false)]
    [InlineData(6,false)]
    [InlineData(7,false)]
    [InlineData(0,true)]
    [InlineData(1,true)]
    [InlineData(2,true)]
    [InlineData(3,true)]
    [InlineData(4,true)]
    [InlineData(5,true)]
    [InlineData(6,true)]
    [InlineData(7,true)]
    public void AffineTargetPreparedSweep_ShouldRetainFiniteInitialContactAcrossSupportedSources(int sourceKind,bool closed)
    {
        using var scene=PhysicsScenarioBuilder.Create();
        LSMeshCollider target;
        if(closed)
        {
            target=MeshTestFixtures.CreateConvexCube();
            target.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(Vector3d.Zero,
                FixedQuaternion.Identity,new Vector3d((Fixed64)2/3,Fixed64.One,(Fixed64)3/5))));
        }
        else target=CreateFloor(scene,(Fixed64)(-4),(Fixed64)5,FixedQuaternion.Identity,Vector3d.One/4);
        Fixed64 radius=Fixed64.One/8;
        Fixed64 halfHeight=sourceKind==1?Fixed64.Quarter:radius;
        Vector3d center=new(closed?Fixed64.Zero:-Fixed64.Half,
            (closed?Fixed64.Half:Fixed64.Zero)+halfHeight-Fixed64.One/16,Fixed64.Zero);
        var worker=new ConvexSweepQueryWorker();LSCollider? source=null;
        if(sourceKind==6)worker.PrepareSphereSource(center,radius,Vector3d.Down);
        else if(sourceKind==7)worker.PrepareCircleSlabSource(center,radius,halfHeight,Vector3d.Down);
        else
        {
            source=sourceKind switch
            {
                0=>new LSSphereCollider{Radius=radius},
                1=>new LSCapsuleCollider{Radius=radius,Size=new Vector3d(Fixed64.Quarter,Fixed64.Half,Fixed64.Quarter)},
                2=>new LSCuboidCollider{Size=Vector3d.One/4},
                3=>new LSCylinderCollider{Radius=radius,Size=Vector3d.One/4},
                4=>new LSConeCollider{Radius=radius,Size=Vector3d.One/4},
                _=>MeshTestFixtures.CreateConvexCube()
            };
            Vector3d scale=sourceKind==5?new Vector3d((Fixed64)3/5,Fixed64.Quarter,Fixed64.Quarter):Vector3d.One;
            source.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(center,FixedQuaternion.Identity,scale)));
            if(source is LSMeshCollider meshSource)worker.PrepareConvexMeshSource(meshSource,Vector3d.Down);
            else worker.PreparePrimitiveSource(source,Vector3d.Down);
        }
        Assert.True(worker.TrySweepPreparedSource(target,out Physics3DHit hit));
        Assert.Same(target,hit.Collider);Assert.Equal(Fixed64.Zero,hit.Distance);Assert.Equal(Vector3d.Up,hit.Normal);
        Assert.True(hit.TryGetPoint(out Vector3d point));Assert.Equal(closed?Fixed64.Half:Fixed64.Zero,point.Y);
        Assert.InRange(point.X.m_rawValue,(-Fixed64.One).m_rawValue,((Fixed64)5/4).m_rawValue);
        Assert.InRange(point.Z.m_rawValue,(-Fixed64.One).m_rawValue,Fixed64.One.m_rawValue);
        if(source!=null)
        {
            var pair=scene.CreatePair(target,source);Assert.True(CollisionDetection.DoCollisionCheck(pair));
            Assert.True(pair.Manifold.PrimaryContact.Depth>Fixed64.Zero);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AffineMeshMotionCertificate_ShouldRetainFiniteSphereEnclosuresAndDeclineUnboundedTravel(bool overlaps)
    {
        using var scene=PhysicsScenarioBuilder.Create();
        var mesh=new LSMeshCollider(new[]{Vector3d.Zero,new Vector3d(40,0,0),new Vector3d(0,0,40)},
            new[]{0,1,2},MeshColliderMode.Concave,MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(Vector3d.Zero,
            FixedQuaternion.Identity,Vector3d.One/8)));
        Fixed64 coordinate=overlaps?(Fixed64)2:(Fixed64)4;
        var sphere=new LSSphereCollider{Radius=Fixed64.One/8};
        scene.InitializeStaticCollider(sphere,new Vector3d(coordinate,Fixed64.Zero,coordinate));
        Assert.Equal(!overlaps,ContinuousCollisionBounds.AreSeparated(sphere,mesh,Fixed64.Zero));
        Assert.False(ContinuousCollisionBounds.AreSeparated(sphere,mesh,Fixed64.MaxValue));
        var box=new LSCuboidCollider{Size=Vector3d.One/4};
        scene.InitializeStaticCollider(box,new Vector3d(2,0,2));
        Assert.False(ContinuousCollisionBounds.AreSeparated(box,mesh,Fixed64.MaxValue));
        var flat=MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        flat.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(new Vector3d(2,0,2),
            FixedQuaternion.Identity,Vector3d.One/16)));
        Assert.False(ContinuousCollisionBounds.AreSeparated(flat,mesh,Fixed64.Zero));
        var vertical=new LSMeshCollider(new[]{new Vector3d(0,-1,-1),new Vector3d(0,1,-1),new Vector3d(0,0,1)},
            new[]{0,1,2},MeshColliderMode.Concave,MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(vertical,new Vector3d(2,0,2));
        Assert.False(ContinuousCollisionBounds.AreSeparated(vertical,mesh,Fixed64.Zero));
    }

    [Fact]
    public void AffineDisconnectedMeshMotionCertificate_ShouldUseAnEmptyLocalCandidateSetInsideTheWholeMeshBounds()
    {
        using var scene=PhysicsScenarioBuilder.Create();
        var mesh=new LSMeshCollider(new[]{Vector3d.Zero,new Vector3d(8,0,0),new Vector3d(0,0,8),
            new Vector3d(32,0,0),new Vector3d(40,0,0),new Vector3d(32,0,8)},
            new[]{0,1,2,3,4,5},MeshColliderMode.Concave,MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(Vector3d.Zero,
            FixedQuaternion.Identity,Vector3d.One/8)));
        var sphere=new LSSphereCollider{Radius=Fixed64.One/8};
        scene.InitializeStaticCollider(sphere,new Vector3d((Fixed64)5/2,Fixed64.Zero,Fixed64.Quarter));
        Assert.True(mesh.Bounds.Intersects(sphere.Bounds));
        Assert.True(ContinuousCollisionBounds.AreSeparated(sphere,mesh,Fixed64.Zero));
    }

    [Theory]
    [InlineData(false,true)]
    [InlineData(true,false)]
    [InlineData(true,true)]
    public void ConvexMeshPair_ShouldRetainItsSolidExitAcrossIndependentFrameRigidity(bool targetAffine,bool sourceAffine)
    {
        using var scene=PhysicsScenarioBuilder.Create();
        var target=MeshTestFixtures.CreateConvexCube();var source=MeshTestFixtures.CreateConvexCube();
        Vector3d affine=new((Fixed64)2/3,Fixed64.One,(Fixed64)3/5);
        target.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(Vector3d.Zero,
            FixedQuaternion.Identity,targetAffine?affine:Vector3d.One)));
        source.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(Vector3d.Up*((Fixed64)3/4),
            FixedQuaternion.Identity,sourceAffine?affine:Vector3d.One)));
        var pair=scene.CreatePair(target,source);Assert.True(CollisionDetection.DoCollisionCheck(pair));
        Assert.Equal(Fixed64.Quarter,pair.Manifold.PrimaryContact.Depth);
        Assert.Equal(Vector3d.Up,pair.Manifold.PrimaryContact.Normal);
        var sweep=new ConvexSweepQueryWorker();sweep.PrepareConvexMeshSource(source,Vector3d.Down);
        Assert.True(sweep.TrySweepPreparedSource(target,out Physics3DHit hit));
        Assert.Equal(Fixed64.Zero,hit.Distance);Assert.Equal(Vector3d.Up,hit.Normal);
        Assert.True(hit.TryGetPoint(out Vector3d point));Assert.Equal(Fixed64.Half,point.Y);
    }

    [Theory]
    [InlineData(false,false)]
    [InlineData(true,false)]
    [InlineData(false,true)]
    [InlineData(true,true)]
    public void FiniteDiamondSupport_ShouldReduceEightExactCrossingsToFourPhysicalLevers(bool affine,bool affineHull)
    {
        using var scene=PhysicsScenarioBuilder.Create();
        var vertices=new[]{Vector3d.Left*((Fixed64)3/4),Vector3d.Forward*((Fixed64)3/4),
            Vector3d.Right*((Fixed64)3/4),Vector3d.Backward*((Fixed64)3/4)};
        if(affine)
            for(int index=0;index<vertices.Length;index++)vertices[index]=(vertices[index]+Vector3d.Right*7)*4;
        var floor=new LSMeshCollider(vertices,new[]{0,1,2,0,2,3},MeshColliderMode.Concave,MeshInertiaPolicy.SurfaceApproximation);
        floor.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(affine?-Vector3d.Right*7:Vector3d.Zero,
            FixedQuaternion.Identity,affine?Vector3d.One/4:Vector3d.One)));
        var mover=MeshTestFixtures.CreateConvexCube();
        if(affineHull)
        {
            Vector3d[] hullVertices=mover.Mesh.LocalVertices.ToArray();
            for(int index=0;index<hullVertices.Length;index++)hullVertices[index]=(hullVertices[index]+Vector3d.Right*7)*4;
            mover=new LSMeshCollider(hullVertices,mover.Mesh.Triangles.ToArray(),MeshColliderMode.Convex,MeshInertiaPolicy.RequireClosedVolume);
            mover.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(-Vector3d.Right*7+Vector3d.Up*Fixed64.Quarter,
                FixedQuaternion.Identity,Vector3d.One/4)));
        }
        else scene.CreateBody(mover,Vector3d.Up*Fixed64.Quarter,FixedQuaternion.Identity);
        var pair=scene.CreatePair(floor,mover);Assert.True(CollisionDetection.DoCollisionCheck(pair));
        // No original vertex is inside the other support polygon. Their eight
        // crossings are (±1/2, ±1/4) and (±1/4, ±1/2); retain four finite levers.
        Assert.Equal(ContactManifold.MaxContactsPerGroup,pair.Manifold.Count);
        bool sourceA=pair.ColliderA==floor;
        foreach(var contact in pair.Manifold)
        {
            Assert.Equal(Vector3d.Up,sourceA?contact.Normal:-contact.Normal);
            Assert.Equal(Fixed64.Quarter,contact.Depth);
            Assert.True((sourceA?contact.AnchorA:contact.AnchorB).TryGetWorldPoint(out Vector3d source));
            Assert.True((sourceA?contact.AnchorB:contact.AnchorA).TryGetWorldPoint(out Vector3d moving));
            Assert.Equal(Fixed64.Zero,source.Y);Assert.Equal(-Fixed64.Quarter,moving.Y);
            Assert.Equal(source.X,moving.X);Assert.Equal(source.Z,moving.Z);
            Assert.Equal((Fixed64)3/4,source.X.Abs()+source.Z.Abs());
            Assert.True(source.X.Abs()==Fixed64.Half&&source.Z.Abs()==Fixed64.Quarter
                ||source.X.Abs()==Fixed64.Quarter&&source.Z.Abs()==Fixed64.Half);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AffineNonconvexSheet_ShouldRetainFiniteHullTriangleContactsInEitherPairOrder(bool reverse)
    {
        using var scene=PhysicsScenarioBuilder.Create();
        // Register the closed hull first. The nonconvex coplanar owner cannot
        // use a convex-patch support shortcut, and retains its triangle provenance.
        var mover=MeshTestFixtures.CreateConvexCube();
        var body=scene.CreateBody(mover,new Vector3d(-Fixed64.Quarter,Fixed64.Quarter,-Fixed64.Quarter),FixedQuaternion.Identity).Body;
        var vertices=new[]{new Vector3d(-1,0,-1),new(1,0,-1),new(1,0,0),new(0,0,0),new(0,0,1),new(-1,0,1)};
        for(int index=0;index<vertices.Length;index++)vertices[index]=(vertices[index]+Vector3d.Right*7)*4;
        var floor=new LSMeshCollider(vertices,new[]{0,2,1,0,3,2,0,5,3,3,5,4},MeshColliderMode.Concave,MeshInertiaPolicy.SurfaceApproximation);
        floor.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(-Vector3d.Right*7,
            FixedQuaternion.Identity,Vector3d.One/4)));
        var pair=reverse?scene.CreatePair(floor,mover):scene.CreatePair(mover,floor);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));Assert.NotEmpty(pair.Manifold);
        foreach(var contact in pair.Manifold)
        {
            Assert.True((pair.ColliderA==floor?contact.AnchorA:contact.AnchorB).TryGetWorldPoint(out Vector3d onFloor));
            Assert.True((pair.ColliderA==mover?contact.AnchorA:contact.AnchorB).TryGetWorldPoint(out Vector3d onHull));
            Assert.Equal(Fixed64.Zero,onFloor.Y);
            Assert.InRange(onFloor.X.m_rawValue,(-Fixed64.One).m_rawValue,Fixed64.One.m_rawValue);
            Assert.InRange(onFloor.Z.m_rawValue,(-Fixed64.One).m_rawValue,Fixed64.One.m_rawValue);
            Assert.True(onFloor.X<=Fixed64.Zero||onFloor.Z<=Fixed64.Zero,"finite contacts cannot fill the missing quadrant");
            Assert.InRange(onHull.X.m_rawValue,((Fixed64)(-3)/4).m_rawValue,Fixed64.Quarter.m_rawValue);
            Assert.InRange(onHull.Y.m_rawValue,(-Fixed64.Quarter).m_rawValue,((Fixed64)3/4).m_rawValue);
            Assert.InRange(onHull.Z.m_rawValue,((Fixed64)(-3)/4).m_rawValue,Fixed64.Quarter.m_rawValue);
            Assert.True(onHull.X==((Fixed64)(-3)/4)||onHull.X==Fixed64.Quarter
                ||onHull.Y==-Fixed64.Quarter||onHull.Y==((Fixed64)3/4)
                ||onHull.Z==((Fixed64)(-3)/4)||onHull.Z==Fixed64.Quarter);
            Assert.False(contact.DepthIsClamped);
        }
        var opposite=reverse?scene.CreatePair(mover,floor):scene.CreatePair(floor,mover);
        Assert.True(CollisionDetection.DoCollisionCheck(opposite));Assert.Equal(pair.Manifold.Count,opposite.Manifold.Count);
        bool sameOrder=pair.ColliderA==opposite.ColliderA;
        for(int index=0;index<pair.Manifold.Count;index++)
        {
            var first=pair.Manifold[index];var other=opposite.Manifold[index];
            Assert.Equal(first.Depth,other.Depth);Assert.Equal(sameOrder?first.Normal:-first.Normal,other.Normal);
            Assert.Equal(sameOrder?first.AnchorA:first.AnchorB,other.AnchorA);
            Assert.Equal(sameOrder?first.AnchorB:first.AnchorA,other.AnchorB);
        }
        body.SetPosition(body.Position3d+Vector3d.Up*Fixed64.Half);mover.RebuildRuntimeShapeOnly();
        Assert.False(CollisionDetection.DoCollisionCheck(pair));Assert.Empty(pair.Manifold);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void IndependentlyAuthoredOpenSheets_ShouldRetainCanonicalFiniteContactsUnderPairReversal(bool reverse)
    {
        using var scene=PhysicsScenarioBuilder.Create();
        var floor=CreateFloor(scene,(Fixed64)(-4),(Fixed64)5,FixedQuaternion.Identity,Vector3d.One/4);
        var wall=new LSMeshCollider(new[]{new Vector3d(28,-4,-4),new Vector3d(28,4,-4),new Vector3d(28,0,4)},
            new[]{0,1,2},MeshColliderMode.Concave,MeshInertiaPolicy.SurfaceApproximation);
        wall.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(-Vector3d.Right*7,
            FixedQuaternion.Identity,Vector3d.One/4)));
        var pair=reverse?scene.CreatePair(wall,floor):scene.CreatePair(floor,wall);
        Assert.True(CollisionDetection.DoCollisionCheck(pair));Assert.NotEmpty(pair.Manifold);
        bool floorDiagonal=false,edgeCross=false;
        foreach(var contact in pair.Manifold)
        {
            // Crossing sheets have positive minimum exits. The floor's
            // diagonal yields axis (8,0,9), depth 10/sqrt(145); crossing its
            // other triangle with the wall yields (8,±18,9), depth 8/sqrt(469).
            // Verify nearest raw rounding against exact squared rationals.
            Vector3d normal=contact.Normal;
            bool diagonal=normal.Y==Fixed64.Zero;
            floorDiagonal|=diagonal;edgeCross|=!diagonal;
            AssertRoundedSquare(contact.Depth,diagonal?20:64,diagonal?29:469);
            // Solver normals retain the existing bounded common-scale rounding:
            // (8,0,9) first rounds 8/9 to raw3817748708, then its scalar norm
            // to raw5746472768. Final X is one raw above the ideal normalized
            // direction's nearest value; depth selection above stays exact.
            Assert.Equal(diagonal?2_853_420_961L:1_586_585_303L,normal.X.Abs().m_rawValue);
            Assert.Equal(diagonal?3_210_098_580L:1_784_908_466L,normal.Z.Abs().m_rawValue);
            AssertWithinOneRawOfSquare(normal.X.Abs(),64,diagonal?145:469);
            AssertWithinOneRawOfSquare(normal.Z.Abs(),81,diagonal?145:469);
            if(!diagonal)
            {
                Assert.Equal(3_569_816_931L,normal.Y.Abs().m_rawValue);
                AssertWithinOneRawOfSquare(normal.Y.Abs(),324,469);
            }
            Assert.False(contact.DepthIsClamped);
            Assert.True(contact.AnchorA.TryGetWorldPoint(out Vector3d a));
            Assert.True(contact.AnchorB.TryGetWorldPoint(out Vector3d b));
            Vector3d onFloor=pair.ColliderA==floor?a:b,onWall=pair.ColliderA==wall?a:b;
            Assert.Equal(Fixed64.Zero,onFloor.Y);
            Assert.InRange(onFloor.X.m_rawValue,(-Fixed64.One).m_rawValue,((Fixed64)5/4).m_rawValue);
            Assert.InRange(onFloor.Z.m_rawValue,(-Fixed64.One).m_rawValue,Fixed64.One.m_rawValue);
            Assert.Equal(Fixed64.Zero,onWall.X);Assert.Equal(Fixed64.Zero,onWall.Y);
            Assert.InRange(onWall.Z.m_rawValue,(-Fixed64.One).m_rawValue,Fixed64.One.m_rawValue);
            Assert.Equal(onFloor.Z,onWall.Z);
        }
        Assert.True(floorDiagonal&&edgeCross);
        var opposite=reverse?scene.CreatePair(floor,wall):scene.CreatePair(wall,floor);
        Assert.True(CollisionDetection.DoCollisionCheck(opposite));Assert.Equal(pair.Manifold.Count,opposite.Manifold.Count);
        bool sameOrder=opposite.ColliderA==pair.ColliderA;
        for(int index=0;index<pair.Manifold.Count;index++)
        {
            var first=pair.Manifold[index];var other=opposite.Manifold[index];
            Assert.Equal(first.Depth,other.Depth);Assert.Equal(sameOrder?first.Normal:-first.Normal,other.Normal);
            Assert.Equal(sameOrder?first.AnchorA:first.AnchorB,other.AnchorA);
            Assert.Equal(sameOrder?first.AnchorB:first.AnchorA,other.AnchorB);
        }

        static void AssertRoundedSquare(Fixed64 value,int numerator,int denominator)
        {
            Assert.True(value>=Fixed64.Zero);
            BigInteger twiceRaw=(BigInteger)value.m_rawValue*2;
            BigInteger scaled=(BigInteger)numerator<<66;
            BigInteger lower=(twiceRaw-1)*(twiceRaw-1)*denominator;
            BigInteger upper=(twiceRaw+1)*(twiceRaw+1)*denominator;
            Assert.True(scaled>=lower&&scaled<=upper,$"{value} is not the nearest Q32.32 sqrt({numerator}/{denominator})");
            if(scaled==lower||scaled==upper)Assert.True(value.m_rawValue%2==0);
        }
        static void AssertWithinOneRawOfSquare(Fixed64 value,int numerator,int denominator)
        {
            BigInteger raw=value.m_rawValue,scaled=(BigInteger)numerator<<64;
            Assert.True((raw-1)*(raw-1)*denominator<=scaled&&scaled<=(raw+1)*(raw+1)*denominator,
                $"{value} differs by more than one raw unit from sqrt({numerator}/{denominator})");
        }
    }

    [Theory]
    [InlineData(false,1,true)]
    [InlineData(false,5,false)]
    [InlineData(true,1,true)]
    [InlineData(true,5,false)]
    public void AuthoredAffineMesh_CcdCertificate_ShouldExpandPhysicalMotionBeforeInverseScale(bool mixed,int motion,bool separated)
    {
        using var scene=PhysicsScenarioBuilder.Create();
        var mesh=new LSMeshCollider(new[]{Vector3d.Zero,new Vector3d(40,0,0),new Vector3d(0,0,40)},
            new[]{0,1,2},MeshColliderMode.Concave,MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(Vector3d.Zero,
            FixedQuaternion.Identity,Vector3d.One/4)));
        if(mixed)
        {
            scene.Context.Settings.RuntimeMode=PhysicsRuntimeMode.Mixed;
            var circle=new LSCircleCollider2D(Fixed64.FromFraction(1,10));
            circle.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(new Vector3d(8,0,8),
                FixedQuaternion.Identity,Vector3d.One)));
            Assert.Equal(separated,ContinuousCollisionBounds.AreSeparated(mesh,circle,(Fixed64)motion));
        }
        else
        {
            var box=new LSCuboidCollider{Size=Vector3d.One/5};
            scene.InitializeStaticCollider(box,new Vector3d(8,0,8));
            Assert.Equal(separated,ContinuousCollisionBounds.AreSeparated(box,mesh,(Fixed64)motion));
            Assert.Equal(separated,ContinuousCollisionBounds.AreSeparated(mesh,box,(Fixed64)motion));
        }
    }

    private static LSMeshCollider CreateFloor(PhysicsScenarioBuilder scene, Fixed64 min, Fixed64 max,
        FixedQuaternion rotation, Vector3d scale)
    {
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(min, Fixed64.Zero, (Fixed64)(-4)),
            new Vector3d(max, Fixed64.Zero, (Fixed64)(-4)),
            new Vector3d(min, Fixed64.Zero, (Fixed64)4),
            new Vector3d(max, Fixed64.Zero, (Fixed64)4)
        }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
        mesh.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(Vector3d.Zero, rotation, scale)));
        return mesh;
    }
}
