//=======================================================================
// LSMeshCollider.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Queries;
using SwiftCollections;
using SwiftCollections.Query;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

public class LSMeshCollider : LSCollider
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
        _size = Mesh.LocalBounds.Proportions;
        _radius = _size.Magnitude * Fixed64.Half;
        SetBounds(Mesh.Bounds);
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
        get => Bounds.Scope.Magnitude;
    }

    protected override void RebuildRuntimeShape()
    {
        BuildShape();
        BuildBoundingBox();
    }

    protected override void BuildBoundingBox() =>
        SetBounds(Mesh.Bounds);

    protected override void BuildShape()
    {
        Vector3d scale = LocalScale;
        FixedQuaternion rotation = Rotation;
        Vector3d scaledSourceCenter = Vector3d.Multiply(Mesh.LocalBounds.Center, scale);
        Vector3d meshOrigin = Position + (rotation * (ScaledOffset - scaledSourceCenter));
        Mesh.UpdateTransform(meshOrigin, rotation, scale);
    }

    internal override void ValidateRuntimeTransform(Vector3d scale, FixedQuaternion rotation)
    {
        Mesh.ValidateScale(scale);
        if (InertiaPolicy == MeshInertiaPolicy.RequireClosedVolume)
            Mesh.ValidateClosedVolumeScaleRepresentability(scale);
        else
            Mesh.ValidateSurfaceMassProperties(scale);
        Mesh.ValidateRotation(rotation);
    }

    protected internal override Fixed64 CalculateMassPropertyWeight()
    {
        if (InertiaPolicy == MeshInertiaPolicy.SurfaceApproximation)
            return Mesh.TotalArea;

        if (Mesh.TryGetClosedVolumeMassProperties(out MeshMassProperties properties, out _))
            return properties.Volume;

        return Fixed64.Zero;
    }

    public override Vector3d CalculateLocalCenterOfMassOffset()
    {
        Vector3d meshCenterOfMass;
        if (InertiaPolicy == MeshInertiaPolicy.SurfaceApproximation)
            meshCenterOfMass = Mesh.SurfaceMassProperties.CenterOfMass;
        else if (Mesh.TryGetClosedVolumeMassProperties(out MeshMassProperties properties, out _))
            meshCenterOfMass = properties.CenterOfMass;
        else
            return base.CalculateLocalCenterOfMassOffset();

        Vector3d scaledLocalCenter = ScaledOffset
            + meshCenterOfMass - Mesh.ScaledLocalBounds.Center;
        return TransformMassPropertyPoint(scaledLocalCenter);
    }

    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localCenterOfMassOffset)
    {
        Vector3d partLocalReference = InverseTransformMassPropertyPoint(localCenterOfMassOffset);
        Vector3d scaledMeshReference = Mesh.ScaledLocalBounds.Center
            + partLocalReference - ScaledOffset;
        return Mesh.CalculateInertiaTensor(mass, InertiaPolicy, scaledMeshReference);
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
        FindClosestPointOnSurface(queryPoint, _triangleQueryBuffer, out Vector3d closest, out _);
        return closest;
    }

    internal void FindClosestPointOnSurface(
        Vector3d queryPoint,
        SwiftList<int> triangleBuffer,
        out Vector3d closest,
        out Vector3d normal)
    {
        closest = queryPoint;
        normal = Vector3d.Zero;
        int closestTriangleIndex = -1;
        _ = KeepClosestPointOnTriangle(
            0,
            queryPoint,
            ref closestTriangleIndex,
            ref closest,
            ref normal);
        if (closest == queryPoint)
            return;

        if (!TryCreateClosestPointSearchBounds(queryPoint, closest, out FixedBoundVolume queryBounds))
        {
            FindClosestPointAcrossAllTriangles(
                queryPoint,
                1,
                ref closestTriangleIndex,
                ref closest,
                ref normal);
            return;
        }

        Mesh.GetTrianglesInWorldBounds(queryBounds, triangleBuffer);
        for (int i = 0; i < triangleBuffer.Count; i++)
        {
            _ = KeepClosestPointOnTriangle(
                triangleBuffer[i],
                queryPoint,
                ref closestTriangleIndex,
                ref closest,
                ref normal);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Fixed64 GetMeshQueryHalfExtent() =>
        FixedMath.Max(FixedMath.Max(Bounds.Scope.X, Bounds.Scope.Y), Bounds.Scope.Z);

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
            || delta.X == Fixed64.MinValue
            || delta.Y == Fixed64.MinValue
            || delta.Z == Fixed64.MinValue)
        {
            bounds = default;
            return false;
        }

        Vector3d absoluteDelta = Vector3d.Abs(delta);
        Fixed64 halfExtent = absoluteDelta.X
            + absoluteDelta.Y
            + absoluteDelta.Z
            + Fixed64.Epsilon;
        if (halfExtent == Fixed64.MaxValue)
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
        Mesh.GetTriangleVertices(triangleIndex, out Vector3d first, out Vector3d second, out Vector3d third);
        Vector3d faceNormal = Mesh.GetFaceNormalWorld(triangleIndex);
        Vector3d pointOnTriangle = new FixedTriangle(first, second, third).ClosestPoint(point);
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
        FindClosestPointOnSurface(point, _triangleQueryBuffer, out _, out Vector3d normal);
        return normal;
    }

    internal override bool TryGetPlanarSurfaceNormal(Vector3d point, out Vector3d normal)
    {
        FindClosestPointOnSurface(point, _triangleQueryBuffer, out _, out normal);
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
}
