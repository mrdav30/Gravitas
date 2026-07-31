//=======================================================================
// LSMeshCollider.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Queries;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

public sealed class LSMeshCollider : LSCollider
{
    private readonly SwiftList<int> _triangleQueryBuffer = new();

    public override ColliderType Shape => ColliderType.Mesh;

    public override int Priority => ColliderSettings.GetPriority(Shape);

    public override Fixed64 Area => Mesh.TotalArea;

    public PhysicsMesh Mesh { get; private set; }

    public MeshColliderMode Mode => Mesh.Mode;

    /// <summary>
    /// Gets whether the mesh's exact-position-welded triangle topology is one
    /// connected, consistently wound, closed two-manifold surface.
    /// </summary>
    public bool IsClosedSurface => Mesh.IsClosedSurface;

    /// <summary>
    /// Determines how inertia is derived when this mesh collider is attached to a body with angular dynamics.
    /// </summary>
    public MeshInertiaPolicy InertiaPolicy { get; }

    public LSMeshCollider(Vector3d[] vertices, int[] triangles)
        : this(vertices, triangles, MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume)
    {
    }

    public LSMeshCollider(Vector3d[] vertices, int[] triangles, MeshColliderMode mode)
        : this(vertices, triangles, mode, MeshInertiaPolicy.RequireClosedVolume)
    {
    }

    public LSMeshCollider(
        Vector3d[] vertices,
        int[] triangles,
        MeshColliderMode mode,
        MeshInertiaPolicy inertiaPolicy)
    {
        ValidateInertiaPolicy(inertiaPolicy);
        InertiaPolicy = inertiaPolicy;
        Mesh = new PhysicsMesh(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity, mode);
        _offset = Mesh.LocalBounds.Center;
        Mesh.UpdatePosition(_offset, FixedQuaternion.Identity);
        _size = Mesh.LocalBounds.Proportions;
        _radius = _size.Magnitude * Fixed64.Half;
        _bounds = Mesh.Bounds;
    }

    public LSMeshCollider(ColliderShapeDefinition definition)
        : this(
            GetVertices(definition),
            GetTriangles(definition),
            MeshColliderMode.Convex,
            GetInertiaPolicy(definition))
    {
        Material = definition.Material;
    }

    public override Fixed64 ScaledRadius
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            return Mesh.ScaledLocalRadius;
        }
    }

    private protected override void PrepareShape(in ColliderShapeSnapshot snapshot)
    {
        Mesh.PrepareTransformation(
            snapshot.Center,
            snapshot.Rotation,
            snapshot.OwnerScale,
            snapshot.PartScale,
            InertiaPolicy);
        if (CompoundOwner == null
            && !CalculatePreparedLocalMassPoint().TryGetPoint(out _))
        {
            throw new InvalidOperationException(
                "Prepared collider mass-property point is outside the Fixed64 coordinate domain.");
        }
        SetPreparedBounds(Mesh.PreparedBounds);
    }

    private protected override void PublishShape() =>
        Mesh.PublishPreparedTransformation();

    internal override ExactMassWeight CalculateMassPropertyWeight()
    {
        if (InertiaPolicy == MeshInertiaPolicy.SurfaceApproximation)
            return Mesh.SurfaceMassWeight;

        if (Mesh.TryGetClosedVolumeMassProperties(out MeshMassProperties properties, out _))
            return ExactMassWeight.FromMeasure(properties.Volume);

        return ExactMassWeight.Zero;
    }

    internal override ExactMassWeight CalculatePreparedMassPropertyWeight()
    {
        if (InertiaPolicy == MeshInertiaPolicy.SurfaceApproximation)
            return Mesh.PreparedSurfaceMassWeight;

        if (Mesh.TryGetPreparedClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out _))
        {
            return ExactMassWeight.FromMeasure(properties.Volume);
        }

        return ExactMassWeight.Zero;
    }

    internal override bool SupportsMassProperties =>
        InertiaPolicy == MeshInertiaPolicy.SurfaceApproximation
        || Mesh.TryGetClosedVolumeMassProperties(out _, out _);

    internal override ExactMassPoint3D CalculateLocalMassPoint()
    {
        Vector3d meshCenterOfMass;
        if (InertiaPolicy == MeshInertiaPolicy.SurfaceApproximation)
        {
            meshCenterOfMass =
                Mesh.SurfaceMassProperties.CenterOfMass;
        }
        else if (Mesh.TryGetClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out _))
        {
            meshCenterOfMass = properties.CenterOfMass;
        }
        else
        {
            meshCenterOfMass = Vector3d.Zero;
        }

        return TransformRelativeMassPropertyPointExact(
            meshCenterOfMass);
    }

    internal override ExactMassPoint3D CalculatePreparedLocalMassPoint()
    {
        Vector3d meshCenterOfMass;
        if (InertiaPolicy == MeshInertiaPolicy.SurfaceApproximation)
        {
            meshCenterOfMass =
                Mesh.PreparedSurfaceMassProperties.CenterOfMass;
        }
        else if (Mesh.TryGetPreparedClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out _))
        {
            meshCenterOfMass = properties.CenterOfMass;
        }
        else
        {
            meshCenterOfMass = Vector3d.Zero;
        }

        return TransformPreparedRelativeMassPropertyPointExact(
            meshCenterOfMass);
    }

    internal override Fixed3x3 CalculateCenterOfMassInertiaTensor(
        Fixed64 mass)
    {
        Vector3d center;
        if (InertiaPolicy == MeshInertiaPolicy.SurfaceApproximation)
        {
            center = Mesh.SurfaceMassProperties.CenterOfMass;
        }
        else if (Mesh.TryGetClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out _))
        {
            center = properties.CenterOfMass;
        }
        else
        {
            center = Vector3d.Zero;
        }

        return Mesh.CalculateInertiaTensor(
            mass,
            InertiaPolicy,
            center);
    }

    public override Fixed64 GetFrontalArea(Vector3d direction) =>
        Mesh.GetFrontalArea(direction);

    public void GetNearbyTriangles(Vector3d queryPoint, SwiftList<int> result)
    {
        FixedBoundVolume queryBounds = CreateQueryBounds(queryPoint, GetMeshQueryHalfExtent());
        GetTrianglesInBounds(queryBounds, result);
    }

    public void GetTrianglesInBounds(FixedBoundVolume queryBounds, SwiftList<int> result)
    {
        Mesh.GetTrianglesInWorldBounds(queryBounds, result);
    }

    public override Vector3d ClosestPointOnSurface(Vector3d queryPoint)
    {
        FixedPointAnchor closest =
            GetClosestSurfaceAnchor(queryPoint, out _);
        if (closest.TryGetPoint(out Vector3d point))
            return point;

        throw new InvalidOperationException(
            "The closest mesh point is outside the representable coordinate range.");
    }

    internal override FixedPointAnchor GetClosestSurfaceAnchor(
        Vector3d queryPoint,
        out Vector3d normal)
    {
        FindClosestPointAnchor(
            queryPoint,
            _triangleQueryBuffer,
            out FixedPointAnchor closest,
            out normal);
        return closest;
    }

    internal void FindClosestPointAnchor(
        Vector3d queryPoint,
        out FixedPointAnchor closest,
        out Vector3d normal) =>
        FindClosestPointAnchor(
            queryPoint,
            _triangleQueryBuffer,
            out closest,
            out normal);

    internal void FindClosestPointAnchor(
        Vector3d queryPoint,
        SwiftList<int> triangleBuffer,
        out FixedPointAnchor closest,
        out Vector3d normal)
    {
        if (!Mesh.TryConvertWorldToScaledLocal(
                queryPoint,
                out Vector3d localQueryPoint))
        {
            FindClosestPointWithoutLocalQuery(
                queryPoint,
                out closest,
                out normal);
            return;
        }

        Vector3d localClosest = localQueryPoint;
        normal = Vector3d.Zero;
        int closestTriangleIndex = -1;
        _ = KeepClosestPointOnTriangle(
            0,
            localQueryPoint,
            ref closestTriangleIndex,
            ref localClosest,
            ref normal);
        if (localClosest == localQueryPoint)
        {
            closest = Mesh.CreatePointAnchor(localClosest);
            normal = Mesh.Rotation.Rotate(normal).Normalized;
            return;
        }

        if (!TryCreateClosestPointSearchBounds(
                localQueryPoint,
                localClosest,
                out FixedBoundVolume queryBounds))
        {
            FindClosestPointAcrossAllTriangles(
                localQueryPoint,
                1,
                ref closestTriangleIndex,
                ref localClosest,
                ref normal);
        }
        else
        {
            Mesh.GetTrianglesInLocalBounds(queryBounds, triangleBuffer);
            for (int i = 0; i < triangleBuffer.Count; i++)
            {
                _ = KeepClosestPointOnTriangle(
                    triangleBuffer[i],
                    localQueryPoint,
                    ref closestTriangleIndex,
                    ref localClosest,
                    ref normal);
            }
        }

        closest = Mesh.CreatePointAnchor(localClosest);
        normal = Mesh.Rotation.Rotate(normal).Normalized;
    }

    private void FindClosestPointWithoutLocalQuery(
        Vector3d queryPoint,
        out FixedPointAnchor closest,
        out Vector3d normal)
    {
        var queryAnchor = new FixedPointAnchor(
            queryPoint,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        closest = GetClosestPointAnchorOnTriangle(
            0,
            queryAnchor,
            out normal);
        for (int triangleIndex = 1;
            triangleIndex < Mesh.TriangleCount;
            triangleIndex++)
        {
            FixedPointAnchor candidate = GetClosestPointAnchorOnTriangle(
                triangleIndex,
                queryAnchor,
                out Vector3d candidateNormal);
            int comparison =
                queryAnchor.CompareSquaredDistance(candidate, closest);
            if (comparison >= 0)
            {
                continue;
            }

            closest = candidate;
            normal = candidateNormal;
        }

    }

    private FixedPointAnchor GetClosestPointAnchorOnTriangle(
        int triangleIndex,
        in FixedPointAnchor query,
        out Vector3d normal)
    {
        Mesh.GetLocalTriangleVertices(
            triangleIndex,
            out Vector3d first,
            out Vector3d second,
            out Vector3d third);
        var triangle = new FixedTriangle(first, second, third);
        FixedPointAnchor closest = triangle.GetClosestPointAnchor(
            Mesh.Origin,
            Mesh.Rotation,
            query);
        normal = OrientNormalTowardPoint(
            Mesh.GetFaceNormalWorld(triangleIndex),
            query,
            closest);
        return closest;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Fixed64 GetMeshQueryHalfExtent()
    {
        Vector3d extents =
            ColliderCanonicalBounds.GetMaximumAbsoluteExtents(
                this,
                Mesh.Origin);
        return FixedMath.Max(
            FixedMath.Max(extents.X, extents.Y),
            extents.Z);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static FixedBoundVolume CreateQueryBounds(Vector3d center, Fixed64 halfExtent)
    {
        Vector3d extents = Vector3d.One * halfExtent;
        return new FixedBoundVolume(center - extents, center + extents);
    }

    private static void ValidateInertiaPolicy(MeshInertiaPolicy inertiaPolicy)
    {
        SwiftThrowHelper.ThrowIfArgument(
            inertiaPolicy != MeshInertiaPolicy.RequireClosedVolume &&
            inertiaPolicy != MeshInertiaPolicy.SurfaceApproximation,
            nameof(inertiaPolicy),
            "Unsupported mesh inertia policy.");
    }

    private static Vector3d[] GetVertices(ColliderShapeDefinition definition)
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.ConvexMesh);
        return definition.GetMeshVerticesForRuntime();
    }

    private static int[] GetTriangles(ColliderShapeDefinition definition)
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.ConvexMesh);
        return definition.GetMeshTrianglesForRuntime();
    }

    private static MeshInertiaPolicy GetInertiaPolicy(ColliderShapeDefinition definition)
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.ConvexMesh);
        return definition.MeshInertiaPolicy;
    }

    private static bool TryCreateClosestPointSearchBounds(
        Vector3d point,
        Vector3d closest,
        out FixedBoundVolume bounds)
    {
        if (!Vector3d.TrySubtract(point, closest, out Vector3d delta)
            || !delta.TryGetMagnitudeCeiling(out Fixed64 halfExtent)
            || !Fixed64.TryAdd(
                halfExtent,
                Fixed64.Epsilon,
                out halfExtent))
        {
            bounds = default;
            return false;
        }

        Vector3d extents = Vector3d.One * halfExtent;
        Vector3d min = point - extents;
        Vector3d max = point + extents;
        bounds = new FixedBoundVolume(min, max);
        return true;
    }

    private void FindClosestPointAcrossAllTriangles(
        Vector3d point,
        int startTriangleIndex,
        ref int closestTriangleIndex,
        ref Vector3d closest,
        ref Vector3d normal)
    {
        for (int i = startTriangleIndex; i < Mesh.TriangleCount; i++)
        {
            _ = KeepClosestPointOnTriangle(
                i,
                point,
                ref closestTriangleIndex,
                ref closest,
                ref normal);
        }
    }

    private bool KeepClosestPointOnTriangle(
        int triangleIndex,
        Vector3d point,
        ref int closestTriangleIndex,
        ref Vector3d closest,
        ref Vector3d normal)
    {
        Mesh.GetLocalTriangleVertices(
            triangleIndex,
            out Vector3d first,
            out Vector3d second,
            out Vector3d third);
        var triangle = new FixedTriangle(first, second, third);
        Vector3d faceNormal = triangle.Normal;
        Vector3d pointOnTriangle = triangle.ClosestPoint(point);
        if (closestTriangleIndex >= 0)
        {
            int distanceComparison = Vector3d.CompareDistanceSquared(
                point,
                pointOnTriangle,
                point,
                closest);
            if (distanceComparison > 0
                || (distanceComparison == 0 && triangleIndex >= closestTriangleIndex))
            {
                return false;
            }
        }

        closestTriangleIndex = triangleIndex;
        closest = pointOnTriangle;
        normal = OrientNormalTowardPoint(faceNormal, point - pointOnTriangle);
        return true;
    }

    public override Vector3d GetNormalAtPoint(Vector3d point)
    {
        _ = GetClosestSurfaceAnchor(
            point,
            out Vector3d normal);
        return normal;
    }

    internal override bool TryGetPlanarSurfaceNormal(Vector3d point, out Vector3d normal)
    {
        FindClosestPointAnchor(
            point,
            _triangleQueryBuffer,
            out _,
            out normal);
        return true;
    }

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        return worker.CheckMeshOverlaps(this, ref outputIntersectionPoints);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d OrientNormalTowardPoint(Vector3d normal, Vector3d targetDirection)
    {
        if (targetDirection.MagnitudeSquared <= Fixed64.Epsilon)
            return normal;

        return Vector3d.Dot(normal, targetDirection) < Fixed64.Zero ? -normal : normal;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d OrientNormalTowardPoint(
        Vector3d normal,
        in FixedPointAnchor target,
        in FixedPointAnchor surfacePoint)
    {
        if (target.ProjectNonNegativeOffsetFrom(surfacePoint, normal)
            > Fixed64.Zero)
        {
            return normal;
        }

        return surfacePoint.ProjectNonNegativeOffsetFrom(target, normal)
            > Fixed64.Zero
            ? -normal
            : normal;
    }
}
