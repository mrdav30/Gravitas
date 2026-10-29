//=======================================================================
// GravitasQueryMixedService.CircleAgainst3DReducers.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections.Query;
using System;

namespace Gravitas.Queries;

/// <summary>
/// Owns mixed swept-circle reducers against 3D collider targets.
/// </summary>
public sealed partial class GravitasQueryMixedService
{
    internal bool TrySweepCircleAgainst3DCollider(
        LSCollider collider, Vector2d start, Vector2d end, Vector2d direction2D,
        Fixed64 length, Fixed64 radius, Fixed64 slabCenterY, Fixed64 halfThickness,
        Vector3d direction3D, LSCollider2D? sourceCollider, out PhysicsMixedHit hit,
        bool requireClosingContact = false)
    {
        return TrySweepCircleAgainst3DColliderCore(collider, start, end, direction2D, length,
                radius, slabCenterY, halfThickness, direction3D, sourceCollider, requireClosingContact, out hit)
            && (!requireClosingContact
                || MixedContact.IsClosingProjection(ExactContactLever2D.ToSpatial(end - start), hit.Normal3DTo2D));
    }

    private bool TrySweepCircleAgainst3DColliderCore(
        LSCollider collider,
        Vector2d start,
        Vector2d end,
        Vector2d direction2D,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCollider2D? sourceCollider,
        bool requireClosingContact,
        out PhysicsMixedHit hit)
    {
        if (collider is LSSphereCollider sphere)
        {
            return TrySweepCircleAgainstSphere(
                start,
                end,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                sphere,
                sourceCollider,
                out hit);
        }

        if (collider is LSCuboidCollider cuboid)
        {
            return TrySweepCircleAgainstCuboid(
                start,
                end,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                cuboid,
                sourceCollider,
                out hit);
        }

        if (collider is LSCapsuleCollider capsule)
            return TrySweepCircleAgainstCapsule(
                start,
                end,
                direction2D,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                capsule,
                sourceCollider,
                out hit);

        if (collider is LSCylinderCollider cylinder)
            return TrySweepCircleAgainstCylinder(
                start,
                end,
                direction2D,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                cylinder,
                sourceCollider,
                out hit);

        if (collider is LSConeCollider cone)
            return TrySweepCircleAgainstCone(start, end, direction2D, length, radius, slabCenterY, halfThickness, direction3D, cone, sourceCollider, out hit);

        if (collider is LSMeshCollider mesh)
            return TrySweepCircleAgainstMesh(start, end, length, radius, slabCenterY, halfThickness, direction3D, mesh, sourceCollider, requireClosingContact, out hit);

        if (collider is LSCompoundCollider compound)
            return TrySweepCircleAgainstCompound3D(start, end, direction2D, length, radius, slabCenterY, halfThickness, direction3D, compound, sourceCollider, requireClosingContact, out hit);

        hit = default;
        return false;
    }

    private static bool TrySweepCircleAgainstCuboid(
        Vector2d start,
        Vector2d end,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCuboidCollider cuboid,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        if (!cuboid.OrientedBox.TryGetCircleSlabSweepDistance(
            new FixedSegment2d(start, end),
            length,
            slabCenterY,
            halfThickness,
            radius,
            out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        hit = BuildCircleAgainstProjectedFiniteSlabHit(
            start,
            end,
            length,
            distance,
            radius,
            slabCenterY,
            halfThickness,
            direction3D,
            cuboid,
            sourceCollider);
        return true;
    }

    private static bool TrySweepCircleAgainstCapsule(
        Vector2d start,
        Vector2d end,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCapsuleCollider capsule,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        Fixed64 slabMinY = slabCenterY - halfThickness;
        Fixed64 slabMaxY = slabCenterY + halfThickness;
        if (!FiniteSlabProjectionSweep.TrySweepCircleAgainstCapsule(
            start,
            end,
            direction,
            length,
            radius,
            slabMinY,
            slabMaxY,
            capsule,
            out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        hit = BuildCircleAgainstProjectedFiniteSlabHit(
            start,
            end,
            length,
            distance,
            radius,
            slabCenterY,
            halfThickness,
            direction3D,
            capsule,
            sourceCollider);
        return true;
    }

    private static bool TrySweepCircleAgainstCylinder(
        Vector2d start,
        Vector2d end,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCylinderCollider cylinder,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        Fixed64 slabMinY = slabCenterY - halfThickness;
        Fixed64 slabMaxY = slabCenterY + halfThickness;
        if (!FiniteSlabProjectionSweep.TrySweepCircleAgainstCylinder(
            start,
            end,
            direction,
            length,
            radius,
            slabMinY,
            slabMaxY,
            cylinder,
            out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        hit = BuildCircleAgainstProjectedFiniteSlabHit(
            start,
            end,
            length,
            distance,
            radius,
            slabCenterY,
            halfThickness,
            direction3D,
            cylinder,
            sourceCollider);
        return true;
    }

    private static PhysicsMixedHit BuildCircleAgainstProjectedFiniteSlabHit(
        Vector2d start,
        Vector2d end,
        Fixed64 length,
        Fixed64 distance,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCollider collider,
        LSCollider2D? sourceCollider)
    {
        Vector2d center2D = new FixedSegment2d(start, end)
            .GetPointAtDistance(distance, length);
        Vector3d sweepCenter = new(center2D.X, slabCenterY, center2D.Y);
        return BuildCircleAgainst3DHit(
            collider,
            sweepCenter,
            direction3D,
            radius,
            slabCenterY,
            halfThickness,
            PhysicsQueryReducerKind.Exact,
            distance,
            sourceCollider);
    }

    private static bool TrySweepCircleAgainstCone(
        Vector2d start,
        Vector2d end,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSConeCollider cone,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        Fixed64 slabMinY = slabCenterY - halfThickness;
        Fixed64 slabMaxY = slabCenterY + halfThickness;
        if (!FiniteSlabProjectionSweep.TrySweepCircleAgainstCone(
            start,
            end,
            direction,
            length,
            radius,
            slabMinY,
            slabMaxY,
            cone,
            out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        hit = BuildCircleAgainstProjectedFiniteSlabHit(
            start,
            end,
            length,
            distance,
            radius,
            slabCenterY,
            halfThickness,
            direction3D,
            cone,
            sourceCollider);
        return true;
    }

    private bool TrySweepCircleAgainstMesh(
        Vector2d start,
        Vector2d end,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSMeshCollider mesh,
        LSCollider2D? sourceCollider,
        bool requireClosingContact,
        out PhysicsMixedHit hit)
    {
        ContactManifold initial = _context.CollisionScratch.MeshSurfaceManifold;
        initial.BeginUpdate(_context.FrameCount);
        // Initial admission should not pay exact section tests for triangles
        // reached only later in the sweep. Keep the full list for impact search.
        CreateCircleSlabSweepBounds(start, start, radius, slabCenterY, halfThickness,
            out Vector3d min, out Vector3d max);
        var initialTriangles = _context.CollisionScratch.MeshTriangleCandidatesA;
        mesh.GetTrianglesInBounds(new FixedBoundVolume(min, max), initialTriangles);
        bool initiallyOverlapping = CollisionDetectionMixed.AppendMeshCircleSlabContacts(mesh,
            new Vector3d(start.X, slabCenterY, start.Y), radius, halfThickness,
            initialTriangles.AsReadOnlySpan(), initial);
        if (initiallyOverlapping)
        {
            // Initial overlap is geometric, so motion cannot turn a filled face into an authored seam.
            ManifoldContact row = initial.PrimaryContact;
            ContactAnchor center = ContactAnchor.FromWorldPoint(Vector3d.Zero);
            MixedContact closing = default;
            if (!requireClosingContact || MixedContact.TrySelectClosingContact(initial, center,
                    Vector3d.Zero, Vector3d.Zero, center, end - start, Fixed64.Zero, out closing))
            {
                if (requireClosingContact) row = closing.ToManifoldContact();
                hit = new PhysicsMixedHit(mesh, sourceCollider, row.AnchorA, row.AnchorB, row.Normal,
                    PhysicsQueryReducerKind.Exact, Fixed64.Zero, direction3D);
                return true;
            }
        }

        CreateCircleSlabSweepBounds(start, end, radius, slabCenterY, halfThickness, out min, out max);
        mesh.GetTrianglesInBounds(new FixedBoundVolume(min, max), _meshTriangleCandidates);
        LastMeshTriangleCandidateCount += _meshTriangleCandidates.Count;
        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;
        int bestTriangleIndex = int.MaxValue;
        PhysicsMixedHit best = default;

        for (int i = 0; i < _meshTriangleCandidates.Count; i++)
        {
            int triangleIndex = _meshTriangleCandidates[i];
            mesh.Mesh.GetGeometryTriangleVertices(
                triangleIndex,
                out Vector3d first,
                out Vector3d second,
                out Vector3d third);
            bool rigid = mesh.Mesh.GeometryFrame.IsRigid;
            Vector3d triangleOrigin = rigid ? mesh.Mesh.GeometryFrame.RigidOrigin : mesh.Mesh.Origin;
            FixedQuaternion triangleRotation = rigid ? mesh.Mesh.GeometryFrame.Rotation : FixedQuaternion.Identity;
            var triangle = new FixedTriangle(first, second, third);
            bool exactLattice = rigid;
            if (!rigid && mesh.Mesh.GeometryFrame.GetPoint(first).TryGetExactLocalPointIn(triangleOrigin, triangleRotation, out Vector3d localFirst)
                && mesh.Mesh.GeometryFrame.GetPoint(second).TryGetExactLocalPointIn(triangleOrigin, triangleRotation, out Vector3d localSecond)
                && mesh.Mesh.GeometryFrame.GetPoint(third).TryGetExactLocalPointIn(triangleOrigin, triangleRotation, out Vector3d localThird))
            {
                triangle = new FixedTriangle(localFirst, localSecond, localThird);
                exactLattice = true;
            }
            bool swept = exactLattice
                ? triangle.TryGetFiniteSlabProjectedCircleSweep(triangleOrigin, triangleRotation,
                    new FixedSegment2d(start, end), length, radius, slabCenterY, halfThickness,
                    out Fixed64 distance, out FixedPointAnchor triangleContact)
                : WideOrientedBox.TryGetFiniteSlabProjectedCircleSweep(triangle, mesh.Mesh.GeometryFrame, triangleOrigin,
                    new FixedSegment2d(start, end), length, radius, slabCenterY, halfThickness,
                    out distance, out triangleContact);
            if (!swept) continue;

            if (found
                && (distance > bestDistance
                    || (distance == bestDistance && triangleIndex >= bestTriangleIndex)))
            {
                continue;
            }

            // Every genuine initial region was inspected above. An overlapping
            // triangle's arbitrary point normal must not fabricate a closing
            // seam or prevent a later independent wall from being considered.
            if (requireClosingContact && initiallyOverlapping && distance == Fixed64.Zero)
                continue;

            Vector2d center2D = new FixedSegment2d(start, end)
                .GetPointAtDistance(distance, length);
            Vector3d sweepCenter = new(center2D.X, slabCenterY, center2D.Y);
            PhysicsMixedHit candidate = default;
            bool filledHit = false;
            if (requireClosingContact)
            {
                // A triangle can be first reached through an authored diagonal.
                // Certify its candidate against the filled local region before
                // reducing distance, so a nonclosing seam cannot hide a later wall.
                CreateCircleSlabSweepBounds(center2D, center2D, radius, slabCenterY, halfThickness,
                    out Vector3d localMin, out Vector3d localMax);
                var localTriangles = _context.CollisionScratch.MeshTriangleCandidatesA;
                mesh.GetTrianglesInBounds(new FixedBoundVolume(localMin, localMax), localTriangles);
                initial.BeginUpdate(_context.FrameCount);
                if (CollisionDetectionMixed.AppendMeshCircleSlabContacts(mesh, sweepCenter, radius,
                        halfThickness, localTriangles.AsReadOnlySpan(), initial))
                {
                    ContactAnchor center = ContactAnchor.FromWorldPoint(Vector3d.Zero);
                    if (!MixedContact.TrySelectClosingContact(initial, center, Vector3d.Zero,
                            Vector3d.Zero, center, end - start, Fixed64.Zero, out MixedContact closing))
                        continue;
                    candidate = new PhysicsMixedHit(mesh, sourceCollider, closing.Anchor3D,
                        closing.Anchor2D, closing.Normal3DTo2D, PhysicsQueryReducerKind.Exact,
                        distance, direction3D);
                    filledHit = true;
                }
            }
            if (!filledHit)
                candidate = BuildCircleAgainst3DHit(mesh, triangleContact, sweepCenter, direction3D,
                    radius, slabCenterY, halfThickness, PhysicsQueryReducerKind.Exact, distance, sourceCollider);
            if (requireClosingContact && mesh.StaticSurface is { } surface)
            {
                // Internal CCD admission always supplies its registered source.
                System.Diagnostics.Debug.Assert(sourceCollider != null);
                if (!surface.AdmitContinuousWitness(null, sourceCollider, sweepCenter, mesh, triangleIndex,
                    candidate.Anchor3D, circleSlabHalfThickness: filledHit ? null : halfThickness)) continue;
            }
            if (requireClosingContact
                && !MixedContact.IsClosingProjection(ExactContactLever2D.ToSpatial(end - start), candidate.Normal3DTo2D))
                continue;
            best = candidate;
            bestDistance = distance;
            bestTriangleIndex = triangleIndex;
            found = true;
        }

        hit = best;
        return found;
    }

    private bool TrySweepCircleAgainstCompound3D(
        Vector2d start,
        Vector2d end,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCompoundCollider compound,
        LSCollider2D? sourceCollider,
        bool requireClosingContact,
        out PhysicsMixedHit hit)
    {
        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;
        PhysicsMixedHit best = default;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider part = compound.GetPartCollider(i);
            if (!TrySweepCircleAgainst3DCollider(
                part,
                start,
                end,
                direction,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                sourceCollider,
                out PhysicsMixedHit candidate,
                requireClosingContact))
            {
                continue;
            }

            if (!PhysicsHitSelectionPolicy.ShouldReplaceDistance(candidate.Distance, found, bestDistance))
                continue;

            best = candidate;
            bestDistance = candidate.Distance;
            found = true;
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new PhysicsMixedHit(
            compound,
            sourceCollider,
            best.Anchor3D,
            best.Anchor2D,
            best.Normal3DTo2D,
            best.ReducerKind,
            best.Distance,
            best.Direction3D);
        return true;
    }

}
