//=======================================================================
// MeshConeSurfaceBoundary.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections;
using System;

namespace Gravitas.CollisionHandling;

/// <summary>Retains finite exposed-feature certificates for one prepared mesh surface.</summary>
internal sealed class MeshConeSurfaceBoundary
{
    private readonly SwiftList<BoundaryCandidate> _isolated = new();
    private int _familyCount;

    internal int IsolatedCount => _isolated.Count;
    // Continuous domains are counted, not retained, until parameter-aware mesh
    // admission consumes them. This count never represents emitted samples.
    internal int FamilyCount => _familyCount;

    internal void Build(PhysicsMesh mesh, int surfaceTriangle, Vector3d coneCenter,
        FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius)
    {
        _isolated.FastClear(); _familyCount = 0;
        ReadOnlySpan<int> boundary = mesh.GetManifoldSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
        int owner = mesh.GetManifoldSurfaceOwner(surfaceTriangle);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        if (boundary.IsEmpty)
        {
            ReadOnlySpan<int> triangle = mesh.GetWeldedTriangleVertexIndices(surfaceTriangle);
            for (int edge = 0; edge < 3; edge++)
                Accumulate(mesh, owner, triangle[edge], triangle[(edge + 1) % 3], vertices,
                    coneCenter, coneRotation, height, radius, storage);
        }
        else
            for (int edge = 0; edge < boundary.Length; edge += 2)
                Accumulate(mesh, owner, boundary[edge], boundary[edge + 1], vertices,
                    coneCenter, coneRotation, height, radius, storage);
    }

    internal FixedContactAnchors GetIsolatedContact(int index) => _isolated[index].Certificate.GetContact();

    internal int GetIsolatedVertex(int index)
    {
        BoundaryCandidate candidate = _isolated[index];
        return candidate.Certificate.PointLocation == ConeSurfacePointLocation.Start ? candidate.First
            : candidate.Certificate.PointLocation == ConeSurfacePointLocation.End ? candidate.Second : -1;
    }

    private void Accumulate(PhysicsMesh mesh, int owner, int first, int second, ReadOnlySpan<Vector3d> vertices,
        Vector3d coneCenter, FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius,
        Span<SegmentConeSurfaceCandidate> storage)
    {
        var selection = new ConeSurfaceSelection(storage);
        SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(
            new FixedSegment(vertices[first], vertices[second]), mesh.Origin, mesh.Rotation,
            coneCenter, coneRotation, height, radius, ref selection);
        for (int index = 0; index < selection.Count; index++)
        {
            var candidate = new BoundaryCandidate(first, second, selection[index]);
            if (candidate.Certificate.Family == ConeSurfaceFamily.None)
            {
                if (SupportsIncidentFan(mesh, owner, candidate, vertices)) _isolated.Add(candidate);
            }
            else
                _familyCount++;
        }
    }

    private static bool SupportsIncidentFan(PhysicsMesh mesh, int owner, BoundaryCandidate candidate,
        ReadOnlySpan<Vector3d> vertices)
    {
        int vertex = candidate.Certificate.PointLocation == ConeSurfacePointLocation.Start ? candidate.First
            : candidate.Certificate.PointLocation == ConeSurfacePointLocation.End ? candidate.Second : -1;
        int origin = vertex < 0 ? candidate.First : vertex;
        foreach (int incident in mesh.GetWeldedVertexTriangleIndices(origin))
        {
            if (mesh.GetManifoldSurfaceOwner(incident) != owner) continue;
            ReadOnlySpan<int> triangle = mesh.GetWeldedTriangleVertexIndices(incident);
            if (vertex < 0 && triangle.IndexOf(candidate.Second) < 0) continue;
            // The local tangent set is the union of the actual incident
            // triangle sectors. Its polar is their intersection: every ray
            // must have nonpositive projection, including reflex/hole fans.
            // At an edge interior, n dot edge = 0, so a ray from either edge
            // endpoint to the third vertex has the same inward projection.
            foreach (int neighbor in triangle)
            {
                if (neighbor == origin) continue;
                Vector3d a = vertices[origin], b = vertices[neighbor];
                var ray = new WideAxis3(
                    Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(b.X), Signed192.Raw(a.X))),
                    Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(b.Y), Signed192.Raw(a.Y))),
                    Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(b.Z), Signed192.Raw(a.Z))));
                bool hasNormal = candidate.Certificate.TryGetNormalDotSign(ray, out int sign);
                System.Diagnostics.Debug.Assert(hasNormal);
                if (sign > 0) return false;
            }
        }
        return true;
    }

    private readonly struct BoundaryCandidate
    {
        internal readonly int First, Second;
        internal readonly SegmentConeSurfaceCandidate Certificate;
        internal BoundaryCandidate(int first, int second, SegmentConeSurfaceCandidate certificate)
        {
            First = first; Second = second; Certificate = certificate;
        }
    }
}
