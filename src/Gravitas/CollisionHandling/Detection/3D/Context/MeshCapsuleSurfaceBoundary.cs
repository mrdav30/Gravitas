//=======================================================================
// MeshCapsuleSurfaceBoundary.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.Materials;
using SwiftCollections;
using System;

namespace Gravitas.CollisionHandling;

/// <summary>Preserves finite radial contacts on a trusted surface's genuine perimeter.</summary>
internal sealed class MeshCapsuleSurfaceBoundary
{
    private readonly SwiftHashSet<int> _visitedCorners = new();
    internal void BuildSurfaceContacts(LSMeshCollider collider, PhysicsMaterial otherMaterial,
        Vector3d center, FixedQuaternion rotation, Fixed64 coreLength, Fixed64 radius,
        int surfaceTriangle, MeshSurfaceConnectivity connectivity, MeshCapsuleFaceRegions faces,
        ContactManifold manifold)
    {
        PhysicsMesh mesh = collider.Mesh;
        int owner = mesh.GetManifoldSurfaceOwner(surfaceTriangle);
        int surface = mesh.GetCanonicalSurfaceOrdinal(surfaceTriangle);
        ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
        _visitedCorners.Clear();
        for (int edge = 0; edge < boundary.Length; edge += 2)
        {
            int first = boundary[edge], second = boundary[edge + 1];
            int feature = checked(3 * (edge / 2));
            // A straight edge's admitted interval needs no subedge routing
            // when the surface has just one connected finite region.
            int region = connectivity.RegionCount == 1
                ? faces.Frame.Finite.IntersectsSegment(new FixedSegment(vertices[first], vertices[second])) ? 0 : -1
                : MeshSurfaceBoundaryTopology.FindEdgeRegion(mesh, owner, first, second,
                    connectivity, faces.Frame.Finite);
            if (region < 0) continue;
            KeepFeature(first, second, feature, region, pointOnly: false);
            if (_visitedCorners.Add(first) && faces.Frame.Finite.ContainsPoint(vertices[first]))
                KeepFeature(first, second, feature + 1, region, pointOnly: true);
            if (_visitedCorners.Add(second) && faces.Frame.Finite.ContainsPoint(vertices[second]))
                KeepFeature(second, first, feature + 2, region, pointOnly: true);
        }

        void KeepFeature(int first, int second, int ordinal, int region, bool pointOnly)
        {
            var triangle = new FixedTriangle(mesh.ScaledLocalVertices[first],
                mesh.ScaledLocalVertices[pointOnly ? first : second], mesh.ScaledLocalVertices[pointOnly ? first : second]);
            Vector3d fallback = faces.GetNormal(region);
            if (!triangle.TryGetCenteredCapsuleContact(mesh.Origin, mesh.Rotation, center, rotation,
                    coreLength, radius, fallback, out FixedContactAnchors contact,
                    out AxisTriangleContactFeature feature, out bool localChart)
                || !pointOnly && feature.TriangleVertex >= 0) return;
            Span<WideAxis3> fan = stackalloc WideAxis3[5];
            int count = MeshSurfaceBoundaryTopology.GetIncidentFan(mesh, owner, first, second, !pointOnly, fan);
            if (!SupportsFan(feature, fan[..count], mesh.Rotation, localChart)
                || !SupportsSelectedHemisphere(feature, mesh.Rotation, localChart,
                    faces.Frame.AuthoredNormal, faces.GetOrientation(region))) return;
            fan[count++] = faces.GetOrientation(region) > 0 ? faces.Frame.AuthoredNormal : -faces.Frame.AuthoredNormal;
            if (coreLength > Fixed64.Zero && feature.IsZeroSeparation
                && TryGetPerpendicularFallback(faces.Frame.Finite, rotation, fan[..count], out fallback))
            {
                // Select a perpendicular direction in the exact capsule chart,
                // then construct the actual ray using its final Q32.32 normal.
                // The inherited closest query can quantize its core direction;
                // do not infer the final ray depth from the ideal radius alone.
                bool found = triangle.TryGetCenteredCapsuleContact(mesh.Origin, mesh.Rotation,
                    center, rotation, coreLength, radius, fallback, out contact);
                System.Diagnostics.Debug.Assert(found);
            }
            var group = new ContactGroupKey(0, 0, surface, 0, checked(-ordinal - 1));
            manifold.AddContact(new ContactAnchor(contact.FirstAnchor), new ContactAnchor(contact.SecondAnchor),
                contact.Depth, contact.Normal, collider.Material, otherMaterial, contact.DepthIsClamped, group: group);
        }
    }

    private static bool SupportsFan(in AxisTriangleContactFeature feature, ReadOnlySpan<WideAxis3> fan,
        FixedQuaternion rotation, bool localChart)
    {
        foreach (WideAxis3 inward in fan)
        {
            WideAxis3 direction = localChart ? inward : WideRigidProjection.TransformLocalAxis(
                new WideRationalBasis3d(rotation), Signed192.NarrowProven(inward.X),
                Signed192.NarrowProven(inward.Y), Signed192.NarrowProven(inward.Z));
            // Inward rays are full-domain 65-bit differences; a normalized
            // rigid basis adds at most 66 bits, still within Signed192.
            if (feature.CompareSeparationProjection(Signed192.NarrowProven(direction.X),
                Signed192.NarrowProven(direction.Y), Signed192.NarrowProven(direction.Z)) > 0) return false;
        }
        return true;
    }

    private static bool SupportsSelectedHemisphere(in AxisTriangleContactFeature feature,
        FixedQuaternion rotation, bool localChart, in WideAxis3 canonicalNormal, int orientation)
    {
        if (localChart)
            return orientation * feature.CompareSeparationProjection(
                Signed192.NarrowProven(canonicalNormal.X), Signed192.NarrowProven(canonicalNormal.Y),
                Signed192.NarrowProven(canonicalNormal.Z)) <= 0;
        WideAxis3 normal = WideRigidProjection.TransformLocalAxis(new WideRationalBasis3d(rotation),
            Signed192.NarrowProven(canonicalNormal.X), Signed192.NarrowProven(canonicalNormal.Y),
            Signed192.NarrowProven(canonicalNormal.Z));
        return orientation * feature.CompareSeparationProjection(normal) <= 0;
    }

    private static bool TryGetPerpendicularFallback(in CapsuleFiniteSectionFrame finite,
        FixedQuaternion rotation, ReadOnlySpan<WideAxis3> authoredFan, out Vector3d fallback)
    {
        fallback = default;
        var core = new WideAxis3(default, Signed320.ExtendValue(Signed192.Signed(1)), default);
        Span<WideAxis3> fan = stackalloc WideAxis3[5];
        for (int index = 0; index < authoredFan.Length; index++)
            fan[index] = WideRigidProjection.TransformLocalAxis(
                finite.ShapeFrame.Basis, Signed192.NarrowProven(authoredFan[index].X),
                Signed192.NarrowProven(authoredFan[index].Y), Signed192.NarrowProven(authoredFan[index].Z));
        ReadOnlySpan<WideAxis3> constraints = fan[..authoredFan.Length];
        FixedQuaternion targetRotation = rotation;
        WideAxis3 selected = -constraints[^1];
        if (KeepDirection(WideAxis3.Cross(core, WideAxis3.Cross(selected, core)), constraints, targetRotation, out fallback)) return true;
        foreach (WideAxis3 wall in constraints)
        {
            WideAxis3 direction = WideAxis3.Cross(core, wall);
            if (KeepDirection(direction, constraints, targetRotation, out fallback)
                || KeepDirection(-direction, constraints, targetRotation, out fallback)) return true;
        }
        // A genuine perimeter fan has two noncollinear rays, so its circle
        // constraints cannot all vanish. Every nonempty clipped circle has
        // an active wall already represented by the signed crosses above.
        return false;
    }

    private static bool KeepDirection(WideAxis3 direction, ReadOnlySpan<WideAxis3> fan,
        FixedQuaternion rotation, out Vector3d result)
    {
        result = default;
        if (direction.IsZero) return false;
        foreach (WideAxis3 inward in fan)
            if (WideAxis3.Dot(direction, inward).Sign > 0) return false;
        // The exact relative basis adds <=130 bits to authored constraints
        // (<=130 bits). Crossing the integer local Up axis stays within
        // Signed320; the final normalized rigid transform fits Signed576.
        WideRigidProjection.TransformLocalAxis(new WideRationalBasis3d(rotation),
            direction, out Signed576 x, out Signed576 y, out Signed576 z);
        result = WideNormalization.GetNormalized(x, y, z);
        return true;
    }

}
