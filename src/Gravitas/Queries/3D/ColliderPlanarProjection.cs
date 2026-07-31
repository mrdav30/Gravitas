//=======================================================================
// ColliderPlanarProjection.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;

namespace Gravitas.Queries;

/// <summary>
/// Dispatches exact X/Z projected-circle relations for built-in 3D colliders.
/// </summary>
internal static class ColliderPlanarProjection
{
    internal static bool TryGetRelation(
        LSCollider collider,
        Vector2d circleCenter,
        Fixed64 circleRadius,
        out ProjectedSurfaceRelation relation)
    {
        if (collider is LSCompoundCollider compound)
        {
            return TryGetCompoundRelation(
                compound,
                circleCenter,
                circleRadius,
                out relation);
        }
        if (collider is LSMeshCollider mesh)
        {
            return TryGetMeshRelation(
                mesh,
                circleCenter,
                circleRadius,
                out relation);
        }

        bool found;
        PlanarProjectionRelation planar;
        switch (collider)
        {
            case LSSphereCollider sphere:
                found = WidePlanarProjection.TryGetSphereRelation(
                    circleCenter,
                    circleRadius,
                    sphere.Center,
                    sphere.ScaledRadius,
                    out planar);
                break;
            case LSCapsuleCollider capsule:
                found = WidePlanarProjection
                    .TryGetCenteredCapsuleRelation(
                        circleCenter,
                        circleRadius,
                        capsule.Center,
                        capsule.Rotation,
                        Vector3d.Up,
                        capsule.AxisLength,
                        capsule.ScaledRadius,
                        out planar);
                break;
            case LSCylinderCollider cylinder:
                found = WidePlanarProjection
                    .TryGetCenteredCylinderRelation(
                        circleCenter,
                        circleRadius,
                        cylinder.Center,
                        cylinder.Rotation,
                        cylinder.Height,
                        cylinder.ScaledRadius,
                        out planar);
                break;
            case LSConeCollider cone:
                found = WidePlanarProjection
                    .TryGetCenteredConeRelation(
                        circleCenter,
                        circleRadius,
                        cone.Center,
                        cone.Rotation,
                        cone.Height,
                        cone.ScaledRadius,
                        out planar);
                break;
            case LSCuboidCollider cuboid:
                found = WidePlanarProjection
                    .TryGetOrientedBoxRelation(
                        circleCenter,
                        circleRadius,
                        cuboid.OrientedBox,
                        out planar);
                break;
            default:
                relation = default;
                return false;
        }

        if (!found)
        {
            relation = default;
            return false;
        }

        Vector3d queryPoint = GetCanonicalQueryPoint(
            collider,
            circleCenter);
        FixedPointAnchor anchor =
            collider.GetClosestSurfaceAnchor(
                queryPoint,
                out Vector3d normal);
        relation = new ProjectedSurfaceRelation(
            planar.Distance,
            planar.Offset,
            anchor,
            normal);
        return true;
    }

    private static bool TryGetCompoundRelation(
        LSCompoundCollider compound,
        Vector2d circleCenter,
        Fixed64 circleRadius,
        out ProjectedSurfaceRelation relation)
    {
        bool found = false;
        relation = default;
        for (int index = 0; index < compound.PartCount; index++)
        {
            if (!TryGetRelation(
                    compound.GetPartCollider(index),
                    circleCenter,
                    circleRadius,
                    out ProjectedSurfaceRelation candidate)
                || (found
                    && candidate.Distance >= relation.Distance))
            {
                continue;
            }

            found = true;
            relation = candidate;
        }

        return found;
    }

    private static bool TryGetMeshRelation(
        LSMeshCollider collider,
        Vector2d circleCenter,
        Fixed64 circleRadius,
        out ProjectedSurfaceRelation relation)
    {
        PhysicsMesh mesh = collider.Mesh;
        bool found = false;
        PlanarProjectionRelation best = default;
        int bestTriangle = -1;
        FixedTriangle bestGeometry = default;
        for (int index = 0; index < mesh.TriangleCount; index++)
        {
            mesh.GetLocalTriangleVertices(
                index,
                out Vector3d first,
                out Vector3d second,
                out Vector3d third);
            var triangle = new FixedTriangle(first, second, third);
            if (!WidePlanarProjection.TryGetTriangleRelation(
                    circleCenter,
                    circleRadius,
                    triangle,
                    mesh.Origin,
                    mesh.Rotation,
                    out PlanarProjectionRelation candidate)
                || (found && candidate.Distance >= best.Distance))
            {
                continue;
            }

            found = true;
            best = candidate;
            bestTriangle = index;
            bestGeometry = triangle;
        }

        if (!found)
        {
            relation = default;
            return false;
        }

        var queryAnchor = new FixedPointAnchor(
            GetCanonicalQueryPoint(
                collider,
                circleCenter),
            FixedQuaternion.Identity,
            Vector3d.Zero);
        FixedPointAnchor contact = bestGeometry.GetClosestPointAnchor(
            mesh.Origin,
            mesh.Rotation,
            queryAnchor);
        relation = new ProjectedSurfaceRelation(
            best.Distance,
            best.Offset,
            contact,
            mesh.GetFaceNormalWorld(bestTriangle));
        return true;
    }

    private static Vector3d GetCanonicalQueryPoint(
        LSCollider collider,
        Vector2d circleCenter) =>
        new(
            circleCenter.X,
            collider.Center.Y,
            circleCenter.Y);
}
