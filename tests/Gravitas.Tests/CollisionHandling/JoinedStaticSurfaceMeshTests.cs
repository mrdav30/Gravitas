using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using System.Numerics;
using Xunit;

namespace Gravitas.Tests;

public sealed class JoinedStaticSurfaceMeshTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OpposingHoleRims_ShouldRetainIndependentDirectedConstraints(bool affine)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var source = Patch(scene, new Fixed64[] { (Fixed64)(-3), (Fixed64)(-1), Fixed64.One, (Fixed64)3 },
            new Fixed64[] { (Fixed64)(-3), (Fixed64)(-2), (Fixed64)3 }, (column, row) => row == 0 || column != 1,
            affine:affine);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { source });
        var mover = ScaledCube(new Vector3d(4, 10, 2));
        Fixed64 height = Fixed64.FromFraction(3, 10);
        scene.CreateBody(mover, new Vector3d(Fixed64.Zero, height, Fixed64.Zero), FixedQuaternion.Identity);
        var pair = scene.CreatePair(source, mover);
        Assert.True(CollisionDetection.DoCollisionCheck(pair)); Assert.NotEmpty(pair.Manifold);
        bool sourceA = pair.ColliderA == source;
        var directions = new HashSet<Vector3d>(); var regions = new HashSet<int>();
        for (int index = 0; index < pair.Manifold.GroupCount; index++)
        {
            ref var group = ref pair.Manifold.GetGroup(index);
            for (int point = 0; point < group.Count; point++)
            {
                var row = group[point]; Vector3d normal = sourceA ? row.Normal : -row.Normal;
                Assert.False(row.DepthIsClamped);
                Assert.True((sourceA ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out var anchor));
                Assert.True((sourceA ? row.AnchorB : row.AnchorA).TryGetWorldPoint(out var moving));
                Assert.Equal(Fixed64.Zero, anchor.Y);
                string detail = $"U contact normal {normal}, depth {row.Depth}, source {anchor}, moving {moving}.";
                Assert.True(normal == Vector3d.Up || normal == Vector3d.Down || normal == Vector3d.Right
                    || normal == Vector3d.Left || normal == Vector3d.Forward || normal == Vector3d.Backward, detail);
                Assert.True(anchor.X >= (Fixed64)(-3) && anchor.X <= (Fixed64)3
                    && anchor.Z >= (Fixed64)(-3) && anchor.Z <= (Fixed64)3
                    && (anchor.Z <= (Fixed64)(-2) || anchor.X.Abs() >= Fixed64.One), detail);
                if (normal.X != Fixed64.Zero)
                {
                    Assert.True(anchor.X.Abs() == (Fixed64)3
                        || anchor.X.Abs() == Fixed64.One && anchor.Z >= (Fixed64)(-2), detail);
                    Assert.Equal(-normal.X * Fixed64.Two, moving.X);
                    if (row.Depth == Fixed64.One)
                    {
                        Assert.Equal(-normal.X, anchor.X);
                        directions.Add(normal); regions.Add(group.Key.Region);
                    }
                }
                else if (normal.Y != Fixed64.Zero)
                {
                    Assert.Equal((Fixed64)5 - normal.Y * height, row.Depth);
                    Assert.Equal(height - normal.Y * (Fixed64)5, moving.Y);
                }
                else
                {
                    Assert.True(anchor.Z.Abs() == (Fixed64)3
                        || anchor.Z == (Fixed64)(-2) && anchor.X.Abs() <= Fixed64.One, detail);
                    Assert.Equal(-normal.Z, moving.Z);
                }
                for (int coordinate = 0; coordinate < 3; coordinate++)
                {
                    Fixed64 delta = anchor[coordinate] - moving[coordinate];
                    if (normal[coordinate] != Fixed64.Zero)
                        Assert.Equal(normal[coordinate] * row.Depth, delta);
                    else
                        // Two half-raw barycentric weight errors plus product
                        // rounding give <= max triangle edge extent + 1 raw;
                        // the largest source/body edge extent here is 10.
                        Assert.InRange(delta.m_rawValue, -11L, 11L);
                    Assert.InRange(moving[coordinate].m_rawValue, mover.BoundsMin[coordinate].m_rawValue - 11,
                        mover.BoundsMax[coordinate].m_rawValue + 11);
                }
            }
        }
        Assert.Equal(2, directions.Count); Assert.Equal(2, regions.Count);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SteppedRim_ShouldRetainDistinctParallelLocalDepths(bool mirrorX, bool mirrorZ)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var vertices = new[] { new Vector3d(-3, 0, -2), new(-1, 0, -2), new(-3, 0, 0), new(-1, 0, 0),
            new(0, 0, 0), new(-3, 0, 2), new(-1, 0, 2), new(0, 0, 2) };
        for (int index = 0; index < vertices.Length; index++)
        {
            if (mirrorX) vertices[index].X = -vertices[index].X;
            if (mirrorZ) vertices[index].Z = -vertices[index].Z;
        }
        var source = new LSMeshCollider(vertices, new[] { 0, 2, 1, 1, 2, 3, 2, 5, 3, 3, 5, 6, 3, 6, 4, 4, 6, 7 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(source, Vector3d.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { source });
        // Tall/deep support exposes two x-rim depths while native interior
        // triangles may also retain genuine plane contacts.
        var mover = ScaledCube(new Vector3d(4, 10, 10));
        Fixed64 height = Fixed64.FromFraction(3, 10);
        scene.CreateBody(mover, new Vector3d(Fixed64.Zero, height, Fixed64.Zero), FixedQuaternion.Identity);
        var pair = scene.CreatePair(source, mover);
        Assert.True(CollisionDetection.DoCollisionCheck(pair)); Assert.NotEmpty(pair.Manifold);
        bool sourceA = pair.ColliderA == source;
        Vector3d expected = mirrorX ? Vector3d.Left : Vector3d.Right;
        var depths = new HashSet<Fixed64>(); var regions = new HashSet<int>();
        for (int index = 0; index < pair.Manifold.GroupCount; index++)
        {
            ref var group = ref pair.Manifold.GetGroup(index);
            for (int point = 0; point < group.Count; point++)
            {
                var row = group[point];
                Vector3d normal = sourceA ? row.Normal : -row.Normal;
                Assert.False(row.DepthIsClamped);
                Assert.True((sourceA ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out var anchor));
                Assert.True((sourceA ? row.AnchorB : row.AnchorA).TryGetWorldPoint(out var moving));
                Assert.Equal(Fixed64.Zero, anchor.Y);
                Fixed64 side = mirrorZ ? -anchor.Z : anchor.Z;
                if (normal == Vector3d.Up || normal == Vector3d.Down)
                {
                    Assert.Equal((Fixed64)5 - normal.Y * height, row.Depth);
                    Assert.Equal(height - normal.Y * (Fixed64)5, moving.Y);
                    Fixed64 x = mirrorX ? -anchor.X : anchor.X;
                    Assert.InRange(side.m_rawValue, ((Fixed64)(-2)).m_rawValue, Fixed64.Two.m_rawValue);
                    Assert.InRange(x.m_rawValue, ((Fixed64)(-2)).m_rawValue, Fixed64.Zero.m_rawValue);
                    Assert.True(side >= Fixed64.Zero || x <= -Fixed64.One,
                        "a plane witness must stay inside the actual stepped domain");
                }
                else
                {
                    Assert.True(normal == expected,
                        $"Unexpected stepped-rim normal {normal}, depth {row.Depth}, source {anchor}.");
                    Assert.True(row.Depth == Fixed64.One || row.Depth == Fixed64.Two);
                    depths.Add(row.Depth); regions.Add(group.Key.Region);
                    Assert.Equal(expected.X * (row.Depth - Fixed64.Two), anchor.X);
                    Assert.InRange(side.m_rawValue, (row.Depth == Fixed64.One ? (Fixed64)(-2) : Fixed64.Zero).m_rawValue,
                        (row.Depth == Fixed64.One ? Fixed64.Zero : Fixed64.Two).m_rawValue);
                }
                for (int coordinate = 0; coordinate < 3; coordinate++)
                {
                    Fixed64 delta = anchor[coordinate] - moving[coordinate];
                    if (normal[coordinate] != Fixed64.Zero)
                        Assert.Equal(normal[coordinate] * row.Depth, delta);
                    else
                        // Native closest-point barycentric weights each round
                        // by <= half a raw unit. Edge extent <=10 plus product
                        // rounding bounds the tangential residual by 11 raw.
                        Assert.InRange(delta.m_rawValue, -11L, 11L);
                }
            }
        }
        Assert.Equal(2, depths.Count); Assert.Equal(2, regions.Count);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AlternatingOwners_ShouldJoinOnlyTheReciprocalSupportWindows(bool bridgeFirst)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        // A | B | A | C: B touches two owners in one collider; the second A
        // owner also touches a different collider on its opposite side.
        var outer = new LSMeshCollider(new[] { new Vector3d(-4, 0, -2), new(-2, 0, -2),
            new(-4, 0, 2), new(-2, 0, 2), new(0, 0, -2), new(2, 0, -2), new(0, 0, 2), new(2, 0, 2) },
            new[] { 0, 2, 1, 1, 2, 3, 4, 6, 5, 5, 6, 7 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        if (!bridgeFirst) scene.InitializeStaticCollider(outer, Vector3d.Zero);
        var bridge = Patch(scene, new Fixed64[] { (Fixed64)(-2), Fixed64.Zero }, new Fixed64[] { (Fixed64)(-2), (Fixed64)2 });
        if (bridgeFirst) scene.InitializeStaticCollider(outer, Vector3d.Zero);
        var end = Patch(scene, new Fixed64[] { (Fixed64)2, (Fixed64)4 }, new Fixed64[] { (Fixed64)(-2), (Fixed64)2 });
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { outer, bridge, end });
        var mover = ScaledCube(new Vector3d(9, 1, 3));
        scene.CreateBody(mover, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity);
        Assert.Equal(new Vector3d(9, 1, 3), mover.BoundsMax - mover.BoundsMin);
        var regions = new HashSet<int>();
        int contacts = 0;
        foreach (var member in new[] { outer, bridge, end })
        {
            var pair = scene.CreatePair(member, mover);
            Assert.True(CollisionDetection.DoCollisionCheck(pair));
            bool sourceA = pair.ColliderA == member;
            for (int index = 0; index < pair.Manifold.GroupCount; index++)
            {
                ref var group = ref pair.Manifold.GetGroup(index);
                regions.Add(group.Key.Region);
                int supplier = sourceA ? group.Key.SourceIdA : group.Key.SourceIdB;
                Assert.True(supplier == outer.Id + 1 || supplier == bridge.Id + 1 || supplier == end.Id + 1);
                for (int point = 0; point < group.Count; point++)
                {
                    var row = group[point]; contacts++;
                    Assert.Equal(Vector3d.Up, sourceA ? row.Normal : -row.Normal);
                    Assert.Equal(Fixed64.Quarter, row.Depth); Assert.False(row.DepthIsClamped);
                    Assert.True((sourceA ? row.AnchorA : row.AnchorB).TryGetWorldPoint(out var source));
                    Assert.Equal(Fixed64.Zero, source.Y);
                    Assert.InRange(source.Z.m_rawValue, (-3 * Fixed64.Half).m_rawValue, (3 * Fixed64.Half).m_rawValue);
                    if (supplier == outer.Id + 1)
                        Assert.True(source.X >= (Fixed64)(-4) && source.X <= (Fixed64)(-2)
                            || source.X >= Fixed64.Zero && source.X <= Fixed64.Two);
                    else if (supplier == bridge.Id + 1)
                        Assert.InRange(source.X.m_rawValue, ((Fixed64)(-2)).m_rawValue, Fixed64.Zero.m_rawValue);
                    else
                        Assert.InRange(source.X.m_rawValue, Fixed64.Two.m_rawValue, ((Fixed64)4).m_rawValue);
                }
            }
        }
        Assert.Equal(4, contacts);
        // Material/provenance groups may differ, but their finite support is
        // one connected region across all four canonical owners.
        Assert.Single(regions);
    }

    [Theory]
    [InlineData(false,0)]
    [InlineData(true,0)]
    [InlineData(false,1)]
    [InlineData(true,1)]
    [InlineData(false,2)]
    [InlineData(true,2)]
    public void AffineTetrahedronEdge_ShouldRetainFiniteRimAndJoinCutConstraints(bool clipped,int cycle)
    {
        using var scene=PhysicsScenarioBuilder.Create();
        var left=Patch(scene,new Fixed64[]{(Fixed64)(-4),Fixed64.Zero},
            new Fixed64[]{(Fixed64)(-4),(Fixed64)4},affine:true,cycle:cycle);
        var members=clipped?new[]{left,Patch(scene,new Fixed64[]{Fixed64.Zero,(Fixed64)4},
            new Fixed64[]{-Fixed64.One,Fixed64.One},affine:true,cycle:cycle)}:new[]{left};
        using var surface=scene.Context.Physics.CreateStaticSurface(members);
        Vector3d a=new(-Fixed64.Quarter,-Fixed64.One/8,clipped?(Fixed64)(-19)/16:(Fixed64)(-2));
        var mover=new LSMeshCollider(new[]{Cycle(a),Cycle(a+Vector3d.One),Cycle(a+Vector3d.Right*2),Cycle(a-Vector3d.Up*2)},
            new[]{0,1,2,0,3,1,0,2,3,1,3,2},MeshColliderMode.Convex,MeshInertiaPolicy.RequireClosedVolume);
        var body=scene.CreateBody(mover,Vector3d.Zero,FixedQuaternion.Identity).Body;
        var rows=Contacts(scene,mover,false,members);Assert.NotEmpty(rows);
        Assert.Equal(rows.ToArray(),Contacts(scene,mover,true,members).ToArray());
        foreach(var row in rows)
        {
            Vector3d source=Uncycle(row.Source),moving=Uncycle(row.Exit),normal=Uncycle(row.Normal);
            Assert.Equal(Fixed64.Zero,source.Y);
            if(row.Supplier==left.Id+1)
            {
                Assert.InRange(source.X.m_rawValue,((Fixed64)(-4)).m_rawValue,Fixed64.Zero.m_rawValue);
                Assert.InRange(source.Z.m_rawValue,((Fixed64)(-4)).m_rawValue,((Fixed64)4).m_rawValue);
            }
            else
            {
                Assert.True(clipped&&row.Supplier==members[1].Id+1);
                Assert.InRange(source.X.m_rawValue,Fixed64.Zero.m_rawValue,((Fixed64)4).m_rawValue);
                Assert.InRange(source.Z.m_rawValue,(-Fixed64.One).m_rawValue,Fixed64.One.m_rawValue);
            }
            // Exact tetra barycentric numerators are z, x-z, z-y and
            // 2-2z-x+y relative to A. Final coordinate rounding contributes
            // at most two raw units to these integer linear forms.
            Vector3d relative=moving-a;
            long x=relative.X.m_rawValue,y=relative.Y.m_rawValue,z=relative.Z.m_rawValue;
            long b=z,c=x-z,d=z-y,first=Fixed64.Two.m_rawValue-2*z-x+y;
            Assert.True(b>=-2&&c>=-2&&d>=-2&&first>=-2);
            Assert.True(Math.Abs(b)<=2||Math.Abs(c)<=2||Math.Abs(d)<=2||Math.Abs(first)<=2);
            if(clipped&&source.X==Fixed64.Zero&&source.Z>-Fixed64.One&&source.Z<Fixed64.One)
            {Assert.Equal(Fixed64.Zero,normal.X);Assert.Equal(Fixed64.Zero,normal.Z);}
            if(clipped)continue;
            Assert.Equal(new Vector3d(1,-1,0).Normalized,normal);
            BigInteger twiceRaw=(BigInteger)row.Depth.m_rawValue*2,scaled=BigInteger.One<<66;
            Assert.True((twiceRaw-1)*(twiceRaw-1)*128<=scaled&&scaled<=(twiceRaw+1)*(twiceRaw+1)*128);
            Assert.Equal(new Vector3d(Fixed64.Zero,Fixed64.Zero,a.Z+(Fixed64)3/16),source);
            Assert.Equal(new Vector3d(-Fixed64.One/16,Fixed64.One/16,source.Z),moving);
        }
        // The exposed-rim exit is exactly(+1/16,-1/16,0). At the joined
        // cut, withdrawing below the whole tetra's maxY=7/8 clears both pieces.
        Vector3d exit=clipped?Vector3d.Down*((Fixed64)7/8+Fixed64.FromRaw(4))
            :new Vector3d(Fixed64.One/16+Fixed64.MinIncrement,-Fixed64.One/16-Fixed64.MinIncrement,Fixed64.Zero);
        surface.Dispose();body.SetPosition(Cycle(exit));mover.RebuildRuntimeShapeOnly();
        foreach(var member in members)Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(member,mover)));
        Vector3d Cycle(Vector3d v)=>cycle==0?v:cycle==1?new(v.Z,v.X,v.Y):new(v.Y,v.Z,v.X);
        Vector3d Uncycle(Vector3d v)=>cycle==0?v:cycle==1?new(v.Y,v.Z,v.X):new(v.Z,v.X,v.Y);
    }

    [Theory]
    [InlineData(false,0)]
    [InlineData(true,0)]
    [InlineData(false,1)]
    [InlineData(true,1)]
    [InlineData(false,2)]
    [InlineData(true,2)]
    public void AffinePerimeterCorner_ShouldRetainExactClosedEdgeTouchAndRejectOneRawWithdrawal(bool lower,int cycle)
    {
        using var scene=PhysicsScenarioBuilder.Create();
        var floor=Patch(scene,new Fixed64[]{(Fixed64)(-4),Fixed64.Zero},
            new Fixed64[]{(Fixed64)(-4),(Fixed64)4},affine:true,cycle:cycle);
        var members=new[]{floor};
        using var surface=scene.Context.Physics.CreateStaticSurface(members);
        Fixed64 z=(Fixed64)(lower?-4:4),outside=z+(lower?-Fixed64.One:Fixed64.One);
        var mover=new LSMeshCollider(new[]{Cycle(new Vector3d(Fixed64.Zero,-Fixed64.One,z)),
            Cycle(new Vector3d(Fixed64.Zero,Fixed64.One,z)),Cycle(new Vector3d(Fixed64.One,Fixed64.Zero,outside))},
            new[]{0,1,2},MeshColliderMode.Concave,MeshInertiaPolicy.SurfaceApproximation);
        var body=scene.CreateBody(mover,Vector3d.Zero,FixedQuaternion.Identity).Body;
        var rows=Contacts(scene,mover,lower,members);Assert.NotEmpty(rows);
        Vector3d outward=Cycle(lower?Vector3d.Backward:Vector3d.Forward),right=Cycle(Vector3d.Right);
        foreach(var row in rows)
        {
            Assert.Equal(Fixed64.Zero,row.Depth);
            Assert.Equal(Cycle(new Vector3d(Fixed64.Zero,Fixed64.Zero,z)),row.Source);
            Assert.Equal(row.Source,row.Exit);
            Assert.True(row.Normal==right||row.Normal==outward);
        }
        // The vertical source edge touches only the floor's perimeter vertex;
        // one raw toward both outward halfplanes opens a genuine finite gap.
        body.SetPosition((right+outward)*Fixed64.MinIncrement);mover.RebuildRuntimeShapeOnly();
        foreach(var member in members)Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(member,mover)));
        Vector3d Cycle(Vector3d v)=>cycle==0?v:cycle==1?new(v.Z,v.X,v.Y):new(v.Y,v.Z,v.X);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AuthoredObliqueSupportEdge_ShouldCrossTheJoinedWindowAndClearBothWholeMembers(bool partial)
    {
        using var scene=PhysicsScenarioBuilder.Create();
        var left=Patch(scene,new Fixed64[]{(Fixed64)(-4),Fixed64.Zero},new Fixed64[]{(Fixed64)(-4),(Fixed64)4},affine:true);
        var right=Patch(scene,new Fixed64[]{Fixed64.Zero,(Fixed64)4},
            partial?new Fixed64[]{Fixed64.Half,Fixed64.Two}:new Fixed64[]{(Fixed64)(-4),(Fixed64)4},affine:true);
        using var surface=scene.Context.Physics.CreateStaticSurface(new[]{left,right});
        var cube=MeshTestFixtures.CreateConvexCube();
        Vector3d[] vertices=cube.Mesh.LocalVertices.ToArray();
        for(int index=0;index<vertices.Length;index++)
        {
            Vector3d v=vertices[index];
            vertices[index]=new(v.X+v.Z,v.Y+v.X/2,v.Z-v.X);
        }
        var mover=new LSMeshCollider(vertices,cube.Mesh.Triangles.ToArray(),MeshColliderMode.Convex,MeshInertiaPolicy.RequireClosedVolume);
        Vector3d start=new(Fixed64.Quarter,Fixed64.Half,Fixed64.Zero);
        var body=scene.CreateBody(mover,start,FixedQuaternion.Identity).Body;
        var rows=Contacts(scene,mover,false,left,right);Assert.Equal(2,rows.Count);
        bool fromLeft=false,fromRight=false;
        foreach(var row in rows)
        {
            Assert.Equal(Vector3d.Up,row.Normal);Assert.Equal(Fixed64.Quarter,row.Depth);
            Assert.Equal(Fixed64.Zero,row.Source.Y);Assert.Equal(-Fixed64.Quarter,row.Exit.Y);
            Assert.Equal(row.Source.X,row.Exit.X);Assert.Equal(row.Source.Z,row.Exit.Z);
            // The exact lowest edge is z=x+3/4, x in[-3/4,1/4]. It
            // intersects x=0 at z=3/4 inside either declared join window.
            Assert.Equal(row.Source.X+(Fixed64)3/4,row.Source.Z);
            Assert.InRange(row.Source.X.m_rawValue,((Fixed64)(-3)/4).m_rawValue,Fixed64.Quarter.m_rawValue);
            fromLeft|=row.Supplier==left.Id+1;fromRight|=row.Supplier==right.Id+1;
        }
        Assert.True(fromLeft&&fromRight);
        surface.Dispose();body.SetPosition(start+Vector3d.Up*(Fixed64.Quarter+Fixed64.FromRaw(4)));mover.RebuildRuntimeShapeOnly();
        Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(left,mover)));
        Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(right,mover)));
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
    public void PartialJoin_ShouldRetainTheActualExposedRim(bool crossesCut, bool reverse,bool affine)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Patch(scene, new Fixed64[] { (Fixed64)(-4), (Fixed64)0 }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 },affine:affine);
        var right = Patch(scene, new Fixed64[] { (Fixed64)0, (Fixed64)4 }, new Fixed64[] { (Fixed64)(-2), (Fixed64)2 },affine:affine);
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        var mover = MeshTestFixtures.CreateConvexCube();
        Vector3d start = new(Fixed64.FromFraction(2, 5), Fixed64.FromFraction(3, 10),
            crossesCut ? Fixed64.Two + Fixed64.FromFraction(3, 10) : (Fixed64)3);
        var body = scene.CreateBody(mover, start, FixedQuaternion.Identity).Body;
        var rows = Contacts(scene, mover, reverse, left, right);
        Assert.NotEmpty(rows);
        Fixed64 exit = Fixed64.Zero;
        bool found = false;
        foreach (var row in rows)
        {
            if (row.Normal != Vector3d.Right) continue;
            found = true; exit = FixedMath.Max(exit, row.Depth);
            Assert.Equal(Fixed64.Half - start.X, row.Depth);
            Assert.Equal(Fixed64.Zero, row.Source.X);
            Assert.True(row.Source.Z >= Fixed64.Two);
            Assert.Equal(left.Id + 1, row.Supplier);
        }
        Assert.True(found, "the exposed x=0 rim above the partial join remains physical");
        surface.Dispose();
        body.SetPosition(start + Vector3d.Right * (exit + Fixed64.FromRaw(2))); mover.RebuildRuntimeShapeOnly();
        Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(left, mover)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Hole_ShouldRemainEmptyAndRetainOnlyItsActualFilledSide(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Fixed64[] z = { (Fixed64)(-2), -Fixed64.Half, Fixed64.Half, (Fixed64)2 };
        var left = Patch(scene, new Fixed64[] { (Fixed64)(-2), -Fixed64.Half, (Fixed64)0 }, z, (column, row) => row != 1 || column != 1);
        var right = Patch(scene, new Fixed64[] { (Fixed64)0, Fixed64.Half, (Fixed64)2 }, z, (column, row) => row != 1 || column != 0);
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        var mover = ScaledCube(Vector3d.One * Fixed64.Half);
        var body = scene.CreateBody(mover, Vector3d.Up * (Fixed64.One / 8), FixedQuaternion.Identity).Body;
        Assert.Equal(Vector3d.One * Fixed64.Half, mover.BoundsMax - mover.BoundsMin);
        foreach (var member in new[] { left, right })
        {
            var pair = scene.CreatePair(member, mover);
            Assert.False(CollisionDetection.DoCollisionCheck(pair)); Assert.Empty(pair.Manifold);
        }
        body.SetPosition(new Vector3d(Fixed64.Half, Fixed64.One / 8, Fixed64.Zero)); mover.RebuildRuntimeShapeOnly();
        var rows = Contacts(scene, mover, reverse, left, right);
        Assert.NotEmpty(rows);
        foreach (var row in rows)
        {
            Assert.Equal(Fixed64.Zero, row.Source.Y);
            Assert.True(row.Source.X.Abs() >= Fixed64.Half || row.Source.Z.Abs() >= Fixed64.Half,
                "support witnesses must lie on the filled ring, including its genuine hole rim");
            Assert.True(row.Depth > Fixed64.Zero);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RemotelyConnectedPatches_ShouldRetainBothDisconnectedSupportBands(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        Fixed64[] z = { (Fixed64)(-2), -Fixed64.Quarter, Fixed64.Quarter, (Fixed64)2 };
        var left = Patch(scene, new Fixed64[] { (Fixed64)(-2), (Fixed64)(-1), (Fixed64)0 }, z, (column, row) => row != 1 || column != 1);
        var right = Patch(scene, new Fixed64[] { (Fixed64)0, (Fixed64)1, (Fixed64)2 }, z, (column, row) => row != 1 || column != 0);
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        var mover = MeshTestFixtures.CreateConvexCube();
        scene.CreateBody(mover, Vector3d.Up * (3 * Fixed64.One / 8), FixedQuaternion.Identity);
        var rows = Contacts(scene, mover, reverse, left, right);
        Assert.Equal(8, rows.Count);
        bool lower = false, upper = false;
        foreach (var row in rows)
        {
            Assert.Equal(Vector3d.Up, row.Normal); Assert.Equal(Fixed64.One / 8, row.Depth);
            Assert.Equal(Fixed64.Zero, row.Source.Y);
            Assert.True(row.Source.Z.Abs() >= Fixed64.Quarter, "a remote spine cannot fill the gap under the hull");
            lower |= row.Source.Z < Fixed64.Zero; upper |= row.Source.Z > Fixed64.Zero;
        }
        Assert.True(lower && upper, "both finite support bands must contribute retained levers");
        var regions = new HashSet<int>();
        foreach (var member in new[] { left, right })
        {
            var pair = reverse ? scene.CreatePair(mover, member) : scene.CreatePair(member, mover);
            CollisionDetection.DoCollisionCheck(pair);
            for (int index = 0; index < pair.Manifold.GroupCount; index++)
            {
                ref var group = ref pair.Manifold.GetGroup(index);
                regions.Add(group.Key.Region);
                int band = 0;
                for (int pointIndex = 0; pointIndex < group.Count; pointIndex++)
                {
                    var row = group[pointIndex];
                    ContactAnchor source = pair.ColliderA == member ? row.AnchorA : row.AnchorB;
                    Assert.True(source.TryGetWorldPoint(out var point));
                    int sign = point.Z < Fixed64.Zero ? -1 : 1;
                    Assert.True(band == 0 || band == sign, "one response region cannot bridge the empty middle of the two U-shaped members");
                    band = sign;
                }
            }
        }
        Assert.Equal(2, regions.Count);
    }

    [Theory]
    [InlineData(30,false,0)]
    [InlineData(45,false,0)]
    [InlineData(30,true,0)]
    [InlineData(45,true,0)]
    [InlineData(30,true,45)]
    public void RotatedHullExit_ShouldClearTheCompleteShapeAgainstBothFiniteMembers(int angle,bool affine,int yaw)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Patch(scene, new Fixed64[] { (Fixed64)(-8), (Fixed64)0 }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 },affine:affine);
        var right = Patch(scene, new Fixed64[] { (Fixed64)0, (Fixed64)8 }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 },affine:affine);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { right, left });
        var mover = MeshTestFixtures.CreateConvexCube();
        Vector3d start = Vector3d.Up * Fixed64.Quarter;
        var body = scene.CreateBody(mover, start,
            PhysicsScenarioBuilder.Yaw(yaw) * FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)angle)).Body;
        var rows = Contacts(scene, mover, false, left, right);
        Assert.NotEmpty(rows);
        Fixed64 depth = Fixed64.Zero;
        foreach (var row in rows)
        {
            Assert.Equal(Vector3d.Up, row.Normal);
            Assert.Equal(Fixed64.Zero, row.Source.Y);
            depth = FixedMath.Max(depth, row.Depth);
        }
        Fixed64 minimumY = Fixed64.MaxValue;
        foreach (Vector3d vertex in mover.Mesh.ScaledLocalVertices)
        {
            Assert.True(new FixedPointAnchor(mover.Mesh.Origin, mover.Mesh.Rotation, vertex).TryGetPoint(out var point));
            minimumY = FixedMath.Min(minimumY, point.Y);
        }
        Assert.InRange(depth.m_rawValue, (-minimumY).m_rawValue - 1, (-minimumY).m_rawValue + 1);
        // Disable continuity for the independent native complete-shape oracle.
        surface.Dispose();
        body.SetPosition(start + Vector3d.Up * (depth + Fixed64.FromRaw(4))); mover.RebuildRuntimeShapeOnly();
        foreach (var member in new[] { left, right })
            Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(member, mover)));
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
    public void IndependentMemberFrames_ShouldRetainSupplierMaterialsAcrossPairAndDeclarationOrder(bool reverseDeclaration, bool reversePair, bool affineLeft)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Patch(scene, new Fixed64[] { (Fixed64)(-4), (Fixed64)0 }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 }, affine:affineLeft);
        var right = new LSMeshCollider(new[] { new Vector3d(-4, 0, 0), new(4, 0, 0), new(-4, 0, 4), new(4, 0, 4) },
            new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One).Normalized;
        right.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(Vector3d.Zero, rotation, Vector3d.One)));
        Assert.Equal(rotation,right.Rotation); Assert.Equal(FixedQuaternion.Identity,left.Rotation);
        left.Material = PhysicsMaterial.Frictionless; right.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics.CreateStaticSurface(reverseDeclaration ? new[] { right, left } : new[] { left, right });
        var mover = MeshTestFixtures.CreateConvexCube();
        scene.CreateBody(mover, new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        var rows = Contacts(scene, mover, reversePair, left, right);
        Assert.NotEmpty(rows);
        bool suppliedLeft = false, suppliedRight = false;
        foreach (var row in rows)
        {
            bool fromLeft = row.Supplier == left.Id + 1;
            Assert.True(fromLeft || row.Supplier == right.Id + 1);
            suppliedLeft |= fromLeft; suppliedRight |= !fromLeft;
            LSMeshCollider supplier = fromLeft ? left : right;
            Assert.Equal(supplier.Material, row.Material); Assert.Equal(supplier.Mesh.Origin, row.Origin);
            Assert.Equal(Vector3d.Up, row.Normal); Assert.Equal(Fixed64.Quarter, row.Depth);
            Assert.Equal(Fixed64.Zero, row.Source.Y);
            Assert.True(fromLeft ? row.Source.X <= Fixed64.Zero : row.Source.X >= Fixed64.Zero);
            Assert.Equal(-Fixed64.Quarter,row.Exit.Y);
            Assert.Equal(row.Source.X,row.Exit.X); Assert.Equal(row.Source.Z,row.Exit.Z);
            Assert.InRange(row.Source.X.m_rawValue,(-Fixed64.Quarter).m_rawValue,((Fixed64)3/4).m_rawValue);
            Assert.InRange(row.Source.Z.m_rawValue,(-Fixed64.Half).m_rawValue,Fixed64.Half.m_rawValue);
        }
        Assert.True(suppliedLeft && suppliedRight);
    }

    [Fact]
    public void EnclosedClosedConcaveMember_ShouldStillOverlapTheConvexVolume()
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var enclosed = MeshTestFixtures.CreateConvexCube(MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(enclosed, Vector3d.Zero);
        var neighbor = Patch(scene, new Fixed64[] { Fixed64.Half, (Fixed64)4 }, new Fixed64[] { -Fixed64.Half, Fixed64.Half }, height: Fixed64.Half);
        using var surface = scene.Context.Physics.CreateStaticSurface(new[] { enclosed, neighbor });
        var mover = ScaledCube(Vector3d.One * (Fixed64)4);
        scene.CreateBody(mover, Vector3d.Zero, FixedQuaternion.Identity);
        Assert.Equal(new Vector3d(-2, -2, -2), mover.BoundsMin);
        Assert.Equal(new Vector3d(2, 2, 2), mover.BoundsMax);
        var rows = Contacts(scene, mover, false, enclosed, neighbor);
        bool found = false;
        foreach (var row in rows)
        {
            if (row.Supplier != enclosed.Id + 1) continue;
            found = true;
            Assert.True(row.Depth > Fixed64.Zero);
            Assert.Equal(Fixed64.Half, FixedMath.Max(row.Source.X.Abs(), FixedMath.Max(row.Source.Y.Abs(), row.Source.Z.Abs())));
        }
        Assert.True(found, "a closed concave surface wholly inside the hull must not disappear through boundary-only admission");
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ProjectedSupplier_ShouldBlockTheHullThroughItsCarrierWithoutFalseEvents(bool obliqueFloor, bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        scene.Context.Environment.Gravity = scene.Context.Environment.AirDensity = scene.Context.Environment.DampingFactor = Fixed64.Zero;
        Vector3d floorNormal = new Vector3d(-1, 1, 0).Normalized;
        Fixed64[] z = { (Fixed64)(-4), (Fixed64)4 };
        var supplier = obliqueFloor
            ? Patch(scene, new Fixed64[] { Fixed64.FromFraction(13, 20), (Fixed64)4 }, z, sloped: true)
            : Patch(scene, new Fixed64[] { -Fixed64.FromFraction(3, 2), -Fixed64.FromFraction(36, 25) }, z);
        var carrier = obliqueFloor
            ? Patch(scene, new Fixed64[] { (Fixed64)(-4), Fixed64.FromFraction(13, 20) }, z, sloped: true)
            : Patch(scene, new Fixed64[] { -Fixed64.FromFraction(36, 25), (Fixed64)2 }, z);
        supplier.Material = carrier.Material = PhysicsMaterial.Frictionless;
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { carrier, supplier } : new[] { supplier, carrier });
        int supplierEnters = 0, carrierEnters = 0;
        supplier.OnContactEnter += _ => supplierEnters++; carrier.OnContactEnter += _ => carrierEnters++;
        var mover = ScaledCube(new Vector3d(4, 1, 2));
        mover.Material = PhysicsMaterial.Frictionless;
        Vector3d start = obliqueFloor ? floorNormal * Fixed64.FromFraction(1, 10) : Vector3d.Up * Fixed64.Quarter;
        var body = scene.CreateBody(mover, start, obliqueFloor ? FixedQuaternion.Identity
            : FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)30)).Body;
        body.UseManualGrounding();
        if (obliqueFloor) Assert.Equal(new Vector3d(4, 1, 2), mover.BoundsMax - mover.BoundsMin);
        else
        {
            Assert.True(mover.BoundsMax.X - mover.BoundsMin.X > (Fixed64)3);
            Assert.True(mover.BoundsMax.Y - mover.BoundsMin.Y > Fixed64.Two);
        }
        if (obliqueFloor) Assert.True(supplier.BoundsMin.Y > mover.BoundsMax.Y);
        Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(supplier, mover)),
            "projected support does not turn its initially separated supplier into a collision event owner");
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.Equal(0, supplierEnters); Assert.Equal(1, carrierEnters);
        Vector3d normal = obliqueFloor ? floorNormal : Vector3d.Up;
        Assert.True(Vector3d.Dot(body.Position3d - start, normal) > Fixed64.Half);
        var pair = scene.Context.Physics.GetCollisionPair(carrier.Id, mover.Id);
        Assert.NotNull(pair); Assert.NotEmpty(pair.Manifold);
        bool supplierA = pair.ColliderA == carrier;
        foreach (var row in pair.Manifold)
        {
            Assert.Equal(PhysicsMaterial.Frictionless, supplierA ? row.MaterialA : row.MaterialB);
            ContactAnchor anchor = supplierA ? row.AnchorA : row.AnchorB;
            Assert.Equal(supplier.Mesh.Origin, anchor.Origin);
            Assert.Equal(supplier.Mesh.Rotation, anchor.Rotation);
            Assert.True(anchor.TryGetWorldPoint(out var source));
            Assert.True(obliqueFloor ? source.Y > Fixed64.Half : source.X < -Fixed64.FromFraction(36, 25));
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CoplanarOpenTriangles_ShouldRetainTheirNativeInPlaneSeparation(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var first = new LSMeshCollider(new[] { new Vector3d(0, 0, 0), new(4, 0, 0), new(0, 0, 4) },
            new[] { 0, 2, 1 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        var neighbor = new LSMeshCollider(new[] { new Vector3d(0, 0, 0), new(0, 0, -4), new(4, 0, 0) },
            new[] { 0, 2, 1 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(first, Vector3d.Zero); scene.InitializeStaticCollider(neighbor, Vector3d.Zero);
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { neighbor, first } : new[] { first, neighbor });
        var mover = new LSMeshCollider(new[] { new Vector3d(3, 0, 3), new(5, 0, 3), new(3, 0, 5) },
            new[] { 0, 2, 1 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.CreateBody(mover, Vector3d.Zero, FixedQuaternion.Identity);
        Assert.True(mover.BoundsMin.X < first.BoundsMax.X && mover.BoundsMin.Z < first.BoundsMax.Z);
        // Face normals and pairwise edge crosses are all vertical here. Only
        // the native normal-cross-own-edge axes prove this coplanar gap.
        foreach (var member in new[] { first, neighbor })
        {
            var pair = reverse ? scene.CreatePair(mover, member) : scene.CreatePair(member, mover);
            Assert.False(CollisionDetection.DoCollisionCheck(pair)); Assert.Empty(pair.Manifold);
        }
    }

    [Theory]
    [InlineData(MeshColliderMode.Concave, false, false)]
    [InlineData(MeshColliderMode.Concave, false, true)]
    [InlineData(MeshColliderMode.Concave, true, false)]
    [InlineData(MeshColliderMode.Concave, true, true)]
    [InlineData(MeshColliderMode.Convex, false, false)]
    [InlineData(MeshColliderMode.Convex, false, true)]
    [InlineData(MeshColliderMode.Convex, true, false)]
    [InlineData(MeshColliderMode.Convex, true, true)]
    public void OpenSheetAcrossJoinedSeam_ShouldRetainItsFiniteVerticalExit(MeshColliderMode mode, bool alternate, bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Patch(scene, new Fixed64[] { (Fixed64)(-4), Fixed64.Zero }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
        var right = Patch(scene, new Fixed64[] { Fixed64.Zero, (Fixed64)4 }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        var mover = new LSMeshCollider(new[] { new Vector3d(-Fixed64.Half, -Fixed64.Quarter, Fixed64.Zero),
            new(Fixed64.Half, -Fixed64.Quarter, Fixed64.Zero), new(-Fixed64.Half, 3 * Fixed64.One / 4, Fixed64.Zero),
            new(Fixed64.Half, 3 * Fixed64.One / 4, Fixed64.Zero) },
            alternate ? new[] { 0, 1, 3, 0, 3, 2 } : new[] { 0, 1, 2, 1, 3, 2 },
            mode, MeshInertiaPolicy.SurfaceApproximation);
        Vector3d start = Vector3d.Right * Fixed64.Quarter;
        var body = scene.CreateBody(mover, start, FixedQuaternion.Identity).Body;
        var rows = Contacts(scene, mover, reverse, left, right);
        Assert.NotEmpty(rows);
        foreach (var row in rows)
        {
            Assert.Equal(Vector3d.Up, row.Normal); Assert.Equal(Fixed64.Quarter, row.Depth);
            Assert.Equal(Fixed64.Zero, row.Source.Y); Assert.Equal(Fixed64.Zero, row.Source.Z);
            Assert.InRange(row.Source.X.m_rawValue, (-Fixed64.Quarter).m_rawValue, (3 * Fixed64.One / 4).m_rawValue);
        }
        surface.Dispose();
        body.SetPosition(start + Vector3d.Up * (Fixed64.Quarter + Fixed64.FromRaw(2))); mover.RebuildRuntimeShapeOnly();
        foreach (var member in new[] { left, right })
            Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(member, mover)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConcaveCubeFreeRotation_ShouldMatchItsAlternateFaceTriangulationThroughTheHostLoop(bool reverse)
    {
        var original = Trace(false);
        var alternate = Trace(true);
        Assert.Equal(original, alternate);

        List<(Vector3d Position, FixedQuaternion Rotation, Vector3d Linear, Vector3d Angular)> Trace(bool changeDiagonals)
        {
            using var scene = PhysicsScenarioBuilder.Create();
            scene.Context.SetFrameRate(16); scene.Context.Environment.Gravity = Fixed64.One;
            scene.Context.Environment.AirDensity = scene.Context.Environment.DampingFactor = scene.Context.Environment.MinSpeed = Fixed64.Zero;
            var left = Patch(scene, new Fixed64[] { (Fixed64)(-8), Fixed64.Zero }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
            var right = Patch(scene, new Fixed64[] { Fixed64.Zero, (Fixed64)8 }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
            using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
            var cube = MeshTestFixtures.CreateConvexCube(MeshColliderMode.Concave, MeshInertiaPolicy.RequireClosedVolume);
            if (changeDiagonals)
            {
                int[] indices = cube.Mesh.Triangles.ToArray();
                for (int face = 0; face < indices.Length; face += 6)
                {
                    int a = indices[face], b = indices[face + 1], c = indices[face + 2], opposite = -1;
                    for (int vertex = face + 3; vertex < face + 6; vertex++)
                        if (indices[vertex] != a && indices[vertex] != b && indices[vertex] != c) opposite = indices[vertex];
                    Assert.True(opposite >= 0);
                    indices[face] = a; indices[face + 1] = b; indices[face + 2] = opposite;
                    indices[face + 3] = a; indices[face + 4] = opposite; indices[face + 5] = c;
                }
                cube = new(cube.Mesh.LocalVertices.ToArray(), indices, MeshColliderMode.Concave, MeshInertiaPolicy.RequireClosedVolume);
            }
            cube.Material = PhysicsMaterial.Frictionless;
            FixedQuaternion initial = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)15);
            var body = scene.CreateBody(cube, new Vector3d(-3 * Fixed64.One / 4, Fixed64.Half, Fixed64.Zero), initial).Body;
            body.UseManualGrounding(); body.SleepEnabled = false; body.AddLinearImpulse(Vector3d.Right);
            var trace = new List<(Vector3d, FixedQuaternion, Vector3d, Vector3d)>();
            bool rotated = false;
            for (int frame = 0; frame < 48; frame++)
            {
                scene.Context.Simulate(); scene.Context.LateSimulate();
                rotated |= body.Rotation != initial;
                Assert.Equal(Fixed64.One, body.LinearVelocity.X); Assert.Equal(Fixed64.Zero, body.LinearVelocity.Z);
                trace.Add((body.Position3d, body.Rotation, body.LinearVelocity, body.AngularVelocity));
            }
            Assert.True(rotated); Assert.True(body.Position3d.X > Fixed64.One);
            return trace;
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OffsetCompoundMeshLeaf_ShouldRetainItsOwnPoseAndBothSupplierMaterials(bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create();
        var left = Patch(scene, new Fixed64[] { (Fixed64)(-4), Fixed64.Zero }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
        var right = Patch(scene, new Fixed64[] { Fixed64.Zero, (Fixed64)4 }, new Fixed64[] { (Fixed64)(-4), (Fixed64)4 });
        left.Material = PhysicsMaterial.Default; right.Material = PhysicsMaterial.Bouncy;
        using var surface = scene.Context.Physics.CreateStaticSurface(reverse ? new[] { right, left } : new[] { left, right });
        Vector3d offset = new(Fixed64.Two, Fixed64.Half, Fixed64.One);
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        var part = new CompoundColliderPart(MeshTestFixtures.CreateConvexCubeDefinition(), offset,
            rotation, Vector3d.One, PhysicsMaterial.Frictionless);
        var compound = new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.Quarter, new Vector3d(-2, 3, 0)), part);
        scene.CreateBody(compound, new Vector3d(-7 * Fixed64.One / 4, -Fixed64.Quarter, -Fixed64.One), FixedQuaternion.Identity);
        var leaf=Assert.IsType<LSMeshCollider>(compound.GetPartCollider(1));
        Assert.Equal(new Vector3d(Fixed64.Quarter,Fixed64.Quarter,Fixed64.Zero),leaf.Center);
        Assert.Equal(rotation,leaf.Rotation);
        var pair = reverse ? scene.CreatePair(compound, left) : scene.CreatePair(left, compound);
        Assert.True(CollisionDetection.DoCollisionCheck(pair)); Assert.Same(compound, pair.ColliderA);
        bool fromLeft = false, fromRight = false;
        for (int groupIndex = 0; groupIndex < pair.Manifold.GroupCount; groupIndex++)
        {
            ref var group = ref pair.Manifold.GetGroup(groupIndex);
            Assert.Equal(2, group.Key.NamespaceA);
            bool suppliedLeft = group.Key.SourceIdB == left.Id + 1;
            Assert.True(suppliedLeft || group.Key.SourceIdB == right.Id + 1);
            fromLeft |= suppliedLeft; fromRight |= !suppliedLeft;
            for (int index = 0; index < group.Count; index++)
            {
                var row = group[index];
                Assert.Equal(PhysicsMaterial.Frictionless, row.MaterialA);
                Assert.Equal(suppliedLeft ? left.Material : right.Material, row.MaterialB);
                Assert.Equal(Vector3d.Down, row.Normal); Assert.Equal(Fixed64.Quarter, row.Depth);
                Assert.Equal(new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), row.AnchorA.Origin);
                Assert.True(row.AnchorA.TryGetWorldPoint(out var moving)); Assert.Equal(-Fixed64.Quarter, moving.Y);
                Assert.True(row.AnchorB.TryGetWorldPoint(out var source)); Assert.Equal(Fixed64.Zero, source.Y);
                // Exact affine witnesses use a translated physical chart;
                // leaf pose and finite contact placement are the contract.
                Assert.InRange(moving.X.m_rawValue,(-Fixed64.Quarter).m_rawValue,((Fixed64)3/4).m_rawValue);
                Assert.InRange(moving.Z.m_rawValue,(-Fixed64.Half).m_rawValue,Fixed64.Half.m_rawValue);
                Assert.Equal(moving.X,source.X);Assert.Equal(moving.Z,source.Z);
                Assert.True(suppliedLeft?source.X<=Fixed64.Zero:source.X>=Fixed64.Zero);
            }
        }
        Assert.True(fromLeft && fromRight);
    }

    [Theory]
    [InlineData(false,false,false)]
    [InlineData(true,false,false)]
    [InlineData(false,true,false)]
    [InlineData(true,true,false)]
    [InlineData(false,false,true)]
    [InlineData(true,false,true)]
    public void YawedHullAgainstAffineExposedRim_ShouldRetainItsFiniteEdgeFootAndClearTheWholeShape(bool reverse,bool endpoint,bool ambiguous)
    {
        using var scene=PhysicsScenarioBuilder.Create();
        var left=Patch(scene,new Fixed64[]{(Fixed64)(-4),Fixed64.Zero},
            new Fixed64[]{(Fixed64)(-4),(Fixed64)4},affine:true);
        var right=Patch(scene,new Fixed64[]{Fixed64.Zero,(Fixed64)4},
            new Fixed64[]{(Fixed64)(-2),(Fixed64)2},affine:true);
        LSMeshCollider? duplicate=ambiguous?Patch(scene,new Fixed64[]{(Fixed64)(-4),Fixed64.Zero},
            new Fixed64[]{(Fixed64)(-4),(Fixed64)4},affine:true):null;
        var members=duplicate==null?new[]{left,right}:new[]{left,right,duplicate!};
        if(reverse)Array.Reverse(members);
        using var surface=scene.Context.Physics.CreateStaticSurface(members);
        var mover=MeshTestFixtures.CreateConvexCube();
        FixedQuaternion yaw=endpoint?FixedQuaternion.Identity:PhysicsScenarioBuilder.Yaw(45);
        if(endpoint)
        {
            // An authored diamond prism makes its extremal vertical edge
            // exactly z=4. A Q32.32 yaw(45) does not have exact cos=sin.
            Vector3d[] vertices=mover.Mesh.LocalVertices.ToArray();
            for(int index=0;index<vertices.Length;index++)
            {
                Vector3d v=vertices[index];
                vertices[index]=new((v.X+v.Z)/2,v.Y,(v.Z-v.X)/2);
            }
            mover=new LSMeshCollider(vertices,mover.Mesh.Triangles.ToArray(),MeshColliderMode.Convex,MeshInertiaPolicy.RequireClosedVolume);
        }
        Vector3d center=new(Fixed64.FromFraction(endpoint?2:3,5),Fixed64.FromFraction(3,10),(Fixed64)(endpoint?4:3));
        SolidBody body=scene.CreateBody(mover,center,yaw).Body;
        var contacts=Contacts(scene,mover,reverse,members);Assert.NotEmpty(contacts);
        bool found=false;Fixed64 depth=Fixed64.Zero;
        Vector3d expectedNormal=endpoint?new Vector3d(1,0,1).Normalized:Vector3d.Right;
        // The exact yaw basis divides by y²+w². Sum its two absolute X
        // coefficients before the only raw rounding; separately rounded
        // Rotate calls and fixed-point additions are not a depth oracle.
        BigInteger y=body.Rotation.Y.m_rawValue,w=body.Rotation.W.m_rawValue;
        BigInteger denominator=y*y+w*w;
        BigInteger numerator=(BigInteger)Fixed64.Half.m_rawValue*(BigInteger.Abs(w*w-y*y)+BigInteger.Abs(2*y*w))
            -(BigInteger)center.X.m_rawValue*denominator;
        BigInteger rounded=BigInteger.DivRem(numerator,denominator,out BigInteger remainder);
        int comparison=(remainder*2).CompareTo(denominator);
        if(comparison>0||comparison==0&&!rounded.IsEven)rounded++;
        Fixed64 expectedDepth=Fixed64.FromRaw((long)rounded);
        foreach(var row in contacts)
        {
            if(row.Normal!=expectedNormal)continue;
            found=true;depth=FixedMath.Max(depth,row.Depth);
            Assert.Equal(Fixed64.Zero,row.Source.X);Assert.Equal(Fixed64.Zero,row.Source.Y);
            Assert.InRange(row.Source.Z.m_rawValue,Fixed64.Two.m_rawValue,((Fixed64)4).m_rawValue);
            Assert.True(row.Supplier==left.Id+1||duplicate!=null&&row.Supplier==duplicate.Id+1);
            if(endpoint)
            {
                // Against x<=0,z<=4, the diamond's shortest exit divides
                // the penetration equally between +X and +Z. Its distance
                // is (1/2-center.X)/sqrt(2), rather than the longer +X exit.
                BigInteger gap=Fixed64.Half.m_rawValue-center.X.m_rawValue;
                BigInteger twiceRaw=(BigInteger)row.Depth.m_rawValue*2;
                Assert.True(gap*gap*4>=(twiceRaw-1)*(twiceRaw-1)*2
                    &&gap*gap*4<=(twiceRaw+1)*(twiceRaw+1)*2);
                Assert.Equal(new Vector3d(0,0,4),row.Source);
                Assert.Equal(Fixed64.Zero,row.Exit.Y);
                Assert.Equal(-Fixed64.FromRaw((long)gap/2),row.Exit.X);
                Assert.Equal((Fixed64)4-Fixed64.FromRaw((long)gap/2),row.Exit.Z);
            }
            else Assert.Equal(expectedDepth,row.Depth);
        }
        Assert.True(found,"the finite exposed rim must retain its minimum support exit");
        surface.Dispose();body.SetPosition(center+expectedNormal*(depth+Fixed64.FromRaw(32)));mover.RebuildRuntimeShapeOnly();
        foreach(var member in members)Assert.False(CollisionDetection.DoCollisionCheck(scene.CreatePair(member,mover)));
    }

    private static List<Contact> Contacts(PhysicsScenarioBuilder scene, LSMeshCollider mover, bool reverse,
        params LSMeshCollider[] members)
    {
        var rows = new List<Contact>();
        foreach (var member in members)
        {
            var pair = reverse ? scene.CreatePair(mover, member) : scene.CreatePair(member, mover);
            if (!CollisionDetection.DoCollisionCheck(pair)) { Assert.Empty(pair.Manifold); continue; }
            bool sourceA = pair.ColliderA == member;
            for (int index = 0; index < pair.Manifold.GroupCount; index++)
            {
                ref var group = ref pair.Manifold.GetGroup(index);
                int supplier = sourceA ? group.Key.SourceIdA : group.Key.SourceIdB;
                for (int point = 0; point < group.Count; point++)
                {
                    var row = group[point];
                    ContactAnchor source = sourceA ? row.AnchorA : row.AnchorB;
                    ContactAnchor exit = sourceA ? row.AnchorB : row.AnchorA;
                    Assert.True(source.TryGetWorldPoint(out var first)); Assert.True(exit.TryGetWorldPoint(out var second));
                    Assert.False(row.DepthIsClamped);
                    rows.Add(new(first, second, sourceA ? row.Normal : -row.Normal, row.Depth, supplier,
                        sourceA ? row.MaterialA : row.MaterialB, source.Origin, source.Rotation));
                }
            }
        }
        return rows;
    }

    private static LSMeshCollider ScaledCube(Vector3d scale)
    {
        var unit = MeshTestFixtures.CreateConvexCube();
        Vector3d[] vertices = unit.Mesh.LocalVertices.ToArray();
        for (int index = 0; index < vertices.Length; index++) vertices[index] *= scale;
        return new(vertices, unit.Mesh.Triangles.ToArray(), MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume);
    }

    private static LSMeshCollider Patch(PhysicsScenarioBuilder scene, Fixed64[] x, Fixed64[] z,
        Func<int, int, bool>? keep = null, Fixed64 height = default, bool sloped = false,bool affine=false,int cycle=0)
    {
        var vertices = new Vector3d[x.Length * z.Length]; var triangles = new List<int>();
        for (int row = 0; row < z.Length; row++)
            for (int column = 0; column < x.Length; column++)
            {
                int vertex = row * x.Length + column; vertices[vertex] = new(x[column], sloped ? x[column] : height, z[row]);
                if (row == z.Length - 1 || column == x.Length - 1 || keep != null && !keep(column, row)) continue;
                triangles.AddRange(new[] { vertex, vertex + x.Length, vertex + 1, vertex + 1, vertex + x.Length, vertex + x.Length + 1 });
            }
        if(cycle!=0)
            for(int index=0;index<vertices.Length;index++)
            {Vector3d v=vertices[index];vertices[index]=cycle==1?new(v.Z,v.X,v.Y):new(v.Y,v.Z,v.X);}
        if(affine)
            for(int index=0;index<vertices.Length;index++)vertices[index]=(vertices[index]+Vector3d.Right*7)*4;
        var mesh = new LSMeshCollider(vertices, triangles.ToArray(), MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        if(affine)
            mesh.InitializeWithNoBody(new TestMatterAgent(scene.Context,new FixedTransform(-Vector3d.Right*7,
                FixedQuaternion.Identity,Vector3d.One/4)));
        else scene.InitializeStaticCollider(mesh, Vector3d.Zero);
        return mesh;
    }

    private readonly record struct Contact(Vector3d Source, Vector3d Exit, Vector3d Normal, Fixed64 Depth,
        int Supplier, PhysicsMaterial Material, Vector3d Origin, FixedQuaternion Rotation);
}
