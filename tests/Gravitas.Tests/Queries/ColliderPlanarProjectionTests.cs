//=======================================================================
// ColliderPlanarProjectionTests.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class ColliderPlanarProjectionTests
{
    [Fact]
    public void PrimitiveReducers_ClassifyTheirCompletePlanarGeometry()
    {
        using GravitasWorldContext context =
            GravitasWorldContext.CreateOwned();
        LSCollider[] colliders =
        {
            new LSSphereCollider { Radius = Fixed64.One },
            new LSCapsuleCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d(
                    Fixed64.Two,
                    (Fixed64)4,
                    Fixed64.Two)
            },
            new LSCylinderCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d(
                    Fixed64.Two,
                    (Fixed64)4,
                    Fixed64.Two)
            },
            new LSConeCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d(
                    Fixed64.Two,
                    (Fixed64)4,
                    Fixed64.Two)
            },
            new LSCuboidCollider
            {
                Size = new Vector3d(
                    (Fixed64)2,
                    (Fixed64)4,
                    (Fixed64)2)
            }
        };
        foreach (LSCollider collider in colliders)
            Initialize(context, collider, FixedQuaternion.Identity);

        foreach (LSCollider collider in colliders)
        {
            ColliderPlanarProjection.TryGetRelation(
                    collider,
                    Vector2d.Zero,
                    Fixed64.Zero,
                    out ProjectedSurfaceRelation relation)
                .Should().BeTrue();
            relation.Distance.Should().Be(Fixed64.Zero);
            relation.Offset.Should().Be(Vector2d.Zero);
        }
    }

    [Fact]
    public void RotatedFiniteAxisReducers_UseTheirCompleteProjection()
    {
        using GravitasWorldContext context =
            GravitasWorldContext.CreateOwned();
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward,
            -Fixed64.PiOver4);
        var cylinder = new LSCylinderCollider
        {
            Radius = Fixed64.One,
            Size = new Vector3d(
                Fixed64.Two,
                (Fixed64)4,
                Fixed64.Two)
        };
        var cone = new LSConeCollider
        {
            Radius = Fixed64.One,
            Size = new Vector3d(
                Fixed64.Two,
                (Fixed64)4,
                Fixed64.Two)
        };
        Initialize(context, cylinder, rotation);
        Initialize(context, cone, rotation);

        ColliderPlanarProjection.TryGetRelation(
                cylinder,
                new Vector2d(Fixed64.Zero, (Fixed64)2),
                Fixed64.One,
                out ProjectedSurfaceRelation cylinderRelation)
            .Should().BeTrue();
        cylinderRelation.Distance.Should().Be(Fixed64.One);

        ColliderPlanarProjection.TryGetRelation(
                cone,
                new Vector2d(Fixed64.Zero, Fixed64.Half),
                Fixed64.Zero,
                out ProjectedSurfaceRelation coneRelation)
            .Should().BeTrue();
        coneRelation.Distance.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void MeshReducer_RetainsTheEarlierTriangleOnAPlanarTie()
    {
        using GravitasWorldContext context =
            GravitasWorldContext.CreateOwned();
        var mesh = new LSMeshCollider(
            new[]
            {
                new Vector3d((Fixed64)(-1), (Fixed64)(-1), (Fixed64)(-1)),
                new Vector3d((Fixed64)1, (Fixed64)(-1), (Fixed64)(-1)),
                new Vector3d(Fixed64.Zero, (Fixed64)(-1), (Fixed64)1),
                new Vector3d((Fixed64)(-1), Fixed64.One, (Fixed64)(-1)),
                new Vector3d((Fixed64)1, Fixed64.One, (Fixed64)(-1)),
                new Vector3d(Fixed64.Zero, Fixed64.One, (Fixed64)1)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        Initialize(context, mesh, FixedQuaternion.Identity);

        ColliderPlanarProjection.TryGetRelation(
                mesh,
                Vector2d.Zero,
                Fixed64.Zero,
                out ProjectedSurfaceRelation relation)
            .Should().BeTrue();
        relation.ContactAnchor.TryGetPoint(out Vector3d point)
            .Should().BeTrue();
        point.Y.Should().Be((Fixed64)(-1));
    }

    [Fact]
    public void CompoundReducer_RetainsTheEarlierPartOnAPlanarTie()
    {
        using GravitasWorldContext context =
            GravitasWorldContext.CreateOwned();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(
                Fixed64.One,
                new Vector3d(
                    Fixed64.Zero,
                    (Fixed64)(-1),
                    Fixed64.Zero)),
            CompoundColliderPart.Sphere(
                Fixed64.One,
                new Vector3d(
                    Fixed64.Zero,
                    Fixed64.One,
                    Fixed64.Zero)));
        Initialize(context, compound, FixedQuaternion.Identity);

        ColliderPlanarProjection.TryGetRelation(
                compound,
                new Vector2d((Fixed64)2, Fixed64.Zero),
                Fixed64.One,
                out ProjectedSurfaceRelation relation)
            .Should().BeTrue();
        relation.ContactAnchor.TryGetPoint(out Vector3d point)
            .Should().BeTrue();
        point.Y.Should().BeLessThan(Fixed64.Zero);
    }

    [Fact]
    public void Reducers_AreAllocationFreeAfterWarmup()
    {
        using GravitasWorldContext context =
            GravitasWorldContext.CreateOwned();
        var mesh = new LSMeshCollider(
            new[]
            {
                new Vector3d((Fixed64)(-1), Fixed64.Zero, (Fixed64)(-1)),
                new Vector3d(Fixed64.One, Fixed64.Zero, (Fixed64)(-1)),
                new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One)
            },
            new[] { 0, 1, 2 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(
                Fixed64.One,
                Vector3d.Zero));
        LSCollider[] colliders =
        {
            new LSSphereCollider { Radius = Fixed64.One },
            new LSCapsuleCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d(
                    Fixed64.Two,
                    (Fixed64)4,
                    Fixed64.Two)
            },
            new LSCylinderCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d(
                    Fixed64.Two,
                    (Fixed64)4,
                    Fixed64.Two)
            },
            new LSConeCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d(
                    Fixed64.Two,
                    (Fixed64)4,
                    Fixed64.Two)
            },
            new LSCuboidCollider
            {
                Size = new Vector3d(
                    Fixed64.Two,
                    (Fixed64)4,
                    Fixed64.Two)
            },
            mesh,
            compound
        };
        foreach (LSCollider collider in colliders)
            Initialize(context, collider, FixedQuaternion.Identity);

        for (int index = 0; index < 32; index++)
        {
            foreach (LSCollider collider in colliders)
            {
                _ = ColliderPlanarProjection.TryGetRelation(
                    collider,
                    Vector2d.Zero,
                    Fixed64.Zero,
                    out _);
            }
        }

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int index = 0; index < 256; index++)
        {
            foreach (LSCollider collider in colliders)
            {
                _ = ColliderPlanarProjection.TryGetRelation(
                    collider,
                    Vector2d.Zero,
                    Fixed64.Zero,
                    out _);
            }
        }
        long allocated =
            GC.GetAllocatedBytesForCurrentThread() - before;

        allocated.Should().Be(0L);
    }

    [Fact]
    public void Reducers_RejectUnsupportedAndSeparatedGeometry()
    {
        using GravitasWorldContext context =
            GravitasWorldContext.CreateOwned();
        var unsupported = new UnsupportedTestCollider3D();
        var sphere = new LSSphereCollider { Radius = Fixed64.One };
        var mesh = new LSMeshCollider(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Forward
            },
            new[] { 0, 1, 2 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        Initialize(context, unsupported, FixedQuaternion.Identity);
        Initialize(context, sphere, FixedQuaternion.Identity);
        Initialize(context, mesh, FixedQuaternion.Identity);

        ColliderPlanarProjection.TryGetRelation(
                unsupported,
                Vector2d.Zero,
                Fixed64.One,
                out _)
            .Should().BeFalse();
        ColliderPlanarProjection.TryGetRelation(
                sphere,
                new Vector2d((Fixed64)3, Fixed64.Zero),
                Fixed64.Half,
                out _)
            .Should().BeFalse();
        ColliderPlanarProjection.TryGetRelation(
                mesh,
                new Vector2d((Fixed64)3, (Fixed64)3),
                Fixed64.Half,
                out _)
            .Should().BeFalse();
    }

    [Fact]
    public void CompoundAndMeshReducers_SkipMissesAndReplaceOnlyWithCloserGeometry()
    {
        using GravitasWorldContext context =
            GravitasWorldContext.CreateOwned();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(
                Fixed64.One,
                new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Sphere(
                Fixed64.One,
                new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Sphere(
                Fixed64.One,
                new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero)));
        var mesh = new LSMeshCollider(
            new[]
            {
                new Vector3d((Fixed64)10, Fixed64.Zero, (Fixed64)(-1)),
                new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.One),
                new Vector3d((Fixed64)11, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)3, Fixed64.Zero, (Fixed64)(-1)),
                new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.One),
                new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)(-1)),
                new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.One),
                new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero)
            },
            new[] { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);
        Initialize(context, compound, FixedQuaternion.Identity);
        Initialize(context, mesh, FixedQuaternion.Identity);

        ColliderPlanarProjection.TryGetRelation(
                compound,
                Vector2d.Zero,
                (Fixed64)4,
                out ProjectedSurfaceRelation compoundRelation)
            .Should().BeTrue();
        compoundRelation.Distance.Should().Be(Fixed64.One);

        ColliderPlanarProjection.TryGetRelation(
                mesh,
                Vector2d.Zero,
                (Fixed64)4,
                out ProjectedSurfaceRelation meshRelation)
            .Should().BeTrue();
        meshRelation.Distance.Should().Be((Fixed64)2);
    }

    private static void Initialize(
        GravitasWorldContext context,
        LSCollider collider,
        FixedQuaternion rotation)
    {
        collider.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(
                Vector3d.Zero,
                rotation,
                Vector3d.One)));
    }
}
