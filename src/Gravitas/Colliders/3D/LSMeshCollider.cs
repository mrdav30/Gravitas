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

/// <summary>Represents a runtime 3D collider backed by deterministic triangle-mesh geometry.</summary>
public sealed class LSMeshCollider : LSCollider
{
    private readonly SwiftList<int> _triangleQueryBuffer = new();
    private bool _inertiaOriginValid, _inertiaOriginExact;
    private Vector3d _inertiaOriginOffset, _inertiaOriginScale, _inertiaOrigin;

    /// <inheritdoc/>
    public override ColliderType Shape => ColliderType.Mesh;

    /// <inheritdoc/>
    public override int Priority => ColliderSettings.GetPriority(Shape);

    /// <inheritdoc/>
    public override Fixed64 Area => Mesh.TotalArea;

    /// <summary>Gets the deterministic runtime mesh geometry.</summary>
    /// <remarks>
    /// Detached access prepares geometry at the current authored part scale.
    /// Runtime transforms are owned by the collider, body and host transform;
    /// the returned mesh cannot be transformed independently.
    /// </remarks>
    public PhysicsMesh Mesh
    {
        get
        {
            EnsureDetachedMassGeometry();
            return GeometryMesh;
        }
    }

    internal PhysicsMesh GeometryMesh { get; }

    /// <summary>Gets the mesh collision mode.</summary>
    public MeshColliderMode Mode => GeometryMesh.Mode;

    /// <summary>
    /// Gets whether the mesh's exact-position-welded triangle topology is one
    /// connected, consistently wound, closed two-manifold surface.
    /// </summary>
    public bool IsClosedSurface => GeometryMesh.IsClosedSurface;

    /// <summary>
    /// Determines how inertia is derived when this mesh collider is attached to a body with angular dynamics.
    /// </summary>
    public MeshInertiaPolicy InertiaPolicy { get; }

    /// <summary>Creates a convex mesh collider from authored vertices and triangle indices.</summary>
    public LSMeshCollider(Vector3d[] vertices, int[] triangles)
        : this(vertices, triangles, MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume)
    {
    }

    /// <summary>Creates a mesh collider with an explicit collision mode.</summary>
    public LSMeshCollider(Vector3d[] vertices, int[] triangles, MeshColliderMode mode)
        : this(vertices, triangles, mode, MeshInertiaPolicy.RequireClosedVolume)
    {
    }

    /// <summary>Creates a mesh collider with explicit collision and inertia policies.</summary>
    public LSMeshCollider(
        Vector3d[] vertices,
        int[] triangles,
        MeshColliderMode mode,
        MeshInertiaPolicy inertiaPolicy)
    {
        ValidateInertiaPolicy(inertiaPolicy);
        InertiaPolicy = inertiaPolicy;
        GeometryMesh = new PhysicsMesh(vertices, triangles, mode, colliderOwned: true);
        _offset = GeometryMesh.LocalBounds.Center;
        // The inherited authoring summary may saturate; runtime vertices retain exact source differences.
        _size = GeometryMesh.LocalBounds.Max - GeometryMesh.LocalBounds.Min;
        _radius = _size.Magnitude * Fixed64.Half;
    }

    /// <summary>Creates a runtime convex mesh from an authored shape definition.</summary>
    public LSMeshCollider(ColliderShapeDefinition definition)
        : this(
            GetVertices(definition),
            GetTriangles(definition),
            MeshColliderMode.Convex,
            GetInertiaPolicy(definition))
    {
        Material = definition.Material;
    }

    /// <inheritdoc/>
    public override Fixed64 ScaledRadius
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            EnsureDetachedMassGeometry();
            return GeometryMesh.ScaledLocalRadius;
        }
    }

    private protected override void PrepareShape(in ColliderShapeSnapshot snapshot)
    {
        GeometryMesh.PrepareTransformation(
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
        SetPreparedGeometryReach(GeometryMesh.PreparedScaledLocalRadius);
        SetPreparedBounds(GeometryMesh.PreparedBounds);
    }

    private protected override void PublishShape() =>
        GeometryMesh.PublishPreparedTransformation();

    internal override ExactMassWeight CalculateMassPropertyWeight()
    {
        EnsureDetachedMassGeometry();
        if (InertiaPolicy == MeshInertiaPolicy.SurfaceApproximation)
        {
            _ = GeometryMesh.SurfaceMassProperties;
            return GeometryMesh.SurfaceMassWeight;
        }

        if (GeometryMesh.TryGetClosedVolumeMassProperties(out MeshMassProperties properties, out _))
            return GeometryMesh.GetCompoundMassIntegrals(usePrepared: false, InertiaPolicy).Weight;

        return ExactMassWeight.Zero;
    }

    internal override ExactMassWeight CalculatePreparedMassPropertyWeight()
    {
        if (InertiaPolicy == MeshInertiaPolicy.SurfaceApproximation)
            return GeometryMesh.PreparedSurfaceMassWeight;

        if (GeometryMesh.TryGetPreparedClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out _))
        {
            return GeometryMesh.GetCompoundMassIntegrals(usePrepared: true, InertiaPolicy).Weight;
        }

        return ExactMassWeight.Zero;
    }

    internal override bool SupportsMassProperties
    {
        get
        {
            EnsureDetachedMassGeometry();
            return InertiaPolicy == MeshInertiaPolicy.SurfaceApproximation
                || GeometryMesh.TryGetClosedVolumeMassProperties(out _, out _);
        }
    }

    internal override ExactMassPoint3D CalculateLocalMassPoint()
    {
        EnsureDetachedMassGeometry();
        Vector3d meshCenterOfMass;
        if (InertiaPolicy == MeshInertiaPolicy.SurfaceApproximation)
        {
            meshCenterOfMass =
                GeometryMesh.SurfaceMassProperties.CenterOfMass;
        }
        else if (GeometryMesh.TryGetClosedVolumeMassProperties(
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
                GeometryMesh.PreparedSurfaceMassProperties.CenterOfMass;
        }
        else if (GeometryMesh.TryGetPreparedClosedVolumeMassProperties(
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

    /// <inheritdoc/>
    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localCenterOfMassOffset)
    {
        if (mass <= Fixed64.Zero) return Fixed3x3.Zero;
        EnsureDetachedMassGeometry();
        if (CompoundOwner == null)
        {
            GetCurrentShapeScales(out Vector3d ownerScale, out _);
            if (!_inertiaOriginValid || _inertiaOriginOffset != LocalOffset || _inertiaOriginScale != ownerScale)
            {
                ExactMassPoint3D origin = TransformRelativeMassPropertyPointExact(Vector3d.Zero);
                _inertiaOriginExact = origin.TryGetPoint(out _inertiaOrigin);
                if (_inertiaOriginExact)
                {
                    ExactMassPoint3D projected = ExactMassPoint3D.FromPoint(_inertiaOrigin);
                    _inertiaOriginExact = WideArithmetic.SubtractSigned320(origin.XNumerator, projected.XNumerator).IsZero
                        && WideArithmetic.SubtractSigned320(origin.YNumerator, projected.YNumerator).IsZero
                        && WideArithmetic.SubtractSigned320(origin.ZNumerator, projected.ZNumerator).IsZero;
                }
                _inertiaOriginOffset = LocalOffset;
                _inertiaOriginScale = ownerScale;
                _inertiaOriginValid = true;
            }
            if (_inertiaOriginExact && Vector3d.TrySubtract(localCenterOfMassOffset, _inertiaOrigin, out Vector3d reference))
                return GeometryMesh.CalculateInertiaTensor(mass, InertiaPolicy, reference);
        }
        // Keep semantic offsets and first moments until the final mass-applied
        // conversion when a body-local reference cannot be mapped to a scalar.
        if (InertiaPolicy == MeshInertiaPolicy.SurfaceApproximation)
            _ = GeometryMesh.SurfaceMassProperties;
        else if (!GeometryMesh.TryGetClosedVolumeMassProperties(out _, out MeshVolumeValidationResult result))
            throw new InvalidOperationException($"Mesh inertia requires a validated closed volume. Validation result: {result}.");
        MeshMassIntegrals integrals = GeometryMesh.GetCompoundMassIntegrals(false, InertiaPolicy);
        CompoundInertiaTensor exact = default;
        CompoundInertiaMath3D.AddMesh(ref exact, integrals, CompoundLocalRotation,
            TransformRelativeMassPropertyPointExact(integrals.Reference), localCenterOfMassOffset);
        return exact.MaterializeSaturated(mass, integrals.Weight);
    }

    internal override Fixed3x3 CalculateCenterOfMassInertiaTensor(Fixed64 mass) =>
        CalculateInertiaTensor(mass, CalculateLocalCenterOfMassOffset());

    private void EnsureDetachedMassGeometry()
    {
        if (HasCommittedShape) return;
        GetCurrentShapeScales(out Vector3d ownerScale, out Vector3d partScale);
        if (GeometryMesh.HasRuntimeGeometry && GeometryMesh.OwnerScale == ownerScale && GeometryMesh.PartScale == partScale) return;
        // Authored part scale may be compensated by a later host scale; prepare
        // only when current properties are requested, never during construction.
        GeometryMesh.PrepareTransformation(_offset, FixedQuaternion.Identity, ownerScale, partScale, null);
        GeometryMesh.PublishPreparedTransformation();
        _bounds = GeometryMesh.Bounds;
    }

    /// <inheritdoc/>
    public override Fixed64 GetFrontalArea(Vector3d direction) =>
        Mesh.GetFrontalArea(direction);

    /// <summary>Writes triangle indices near a world-space point into a caller-owned buffer.</summary>
    public void GetNearbyTriangles(Vector3d queryPoint, SwiftList<int> result)
    {
        EnsureDetachedMassGeometry();
        FixedBoundVolume queryBounds = CreateQueryBounds(queryPoint, GetMeshQueryHalfExtent());
        GetTrianglesInBounds(queryBounds, result);
    }

    /// <summary>Writes triangle indices overlapping world-space bounds into a caller-owned buffer.</summary>
    public void GetTrianglesInBounds(FixedBoundVolume queryBounds, SwiftList<int> result)
    {
        Mesh.GetTrianglesInWorldBounds(queryBounds, result);
    }

    /// <inheritdoc/>
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

    /// <inheritdoc/>
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

    /// <inheritdoc/>
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
