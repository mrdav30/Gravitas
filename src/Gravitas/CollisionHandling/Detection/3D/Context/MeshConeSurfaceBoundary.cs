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
    private readonly SwiftList<FamilyCandidate> _families = new();
    private readonly SwiftList<FamilySample> _familySamples = new();
    private readonly SwiftList<WideAxis3> _halfspaces = new();
    private ConeSurfaceFamilyEvent[] _familyStorage = Array.Empty<ConeSurfaceFamilyEvent>();

    internal int IsolatedCount => _isolated.Count;
    internal int FamilyCount => _families.Count;
    internal int FamilySampleCount => _familySamples.Count;

    internal void Build(PhysicsMesh mesh, int surfaceTriangle, Vector3d coneCenter,
        FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius)
    {
        _isolated.FastClear(); _families.FastClear(); _familySamples.FastClear(); _halfspaces.FastClear();
        ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
        int owner = mesh.GetManifoldSurfaceOwner(surfaceTriangle);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
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
            else if (candidate.Certificate.Family == ConeSurfaceFamily.SegmentInterval)
            {
                KeepFamily(mesh, owner, candidate, vertices, ConeSurfacePointLocation.Start);
                KeepFamily(mesh, owner, candidate, vertices, ConeSurfacePointLocation.Interior);
                KeepFamily(mesh, owner, candidate, vertices, ConeSurfacePointLocation.End);
            }
            else KeepFamily(mesh, owner, candidate, vertices, candidate.Certificate.PointLocation);
        }
    }

    internal FixedContactAnchors GetFamilySampleContact(int index)
    {
        FamilySample sample = _familySamples[index];
        FamilyCandidate family = _families[sample.Family];
        return sample.Event.GetContact(family.Candidate.Certificate,
            _halfspaces.AsReadOnlySpan().Slice(family.Offset, family.Count));
    }

    internal bool TryGetFamilySampleWorldAnchors(int index, out Vector3d first, out Vector3d second)
    {
        first = second = default;
        if ((uint)index >= (uint)_familySamples.Count) return false;
        FamilySample sample = _familySamples[index];
        FamilyCandidate family = _families[sample.Family];
        return sample.Event.TryGetWorldAnchors(family.Candidate.Certificate,
            _halfspaces.AsReadOnlySpan().Slice(family.Offset, family.Count), out first, out second);
    }

    private void KeepFamily(PhysicsMesh mesh, int owner, BoundaryCandidate candidate,
        ReadOnlySpan<Vector3d> vertices, ConeSurfacePointLocation location)
    {
        Span<WideAxis3> fan = stackalloc WideAxis3[4];
        int count = GetIncidentFan(mesh, owner, candidate, vertices, location, fan);
        int capacity = candidate.Certificate.GetMaximumFamilyEventCount(count);
        if (_familyStorage.Length < capacity) Array.Resize(ref _familyStorage, capacity);
        var selection = new ConeSurfaceFamilySelection(_familyStorage);
        if (!candidate.Certificate.AccumulateFamilyEvents(fan[..count], location, ref selection)) return;
        int family = _families.Count;
        _families.Add(new FamilyCandidate(candidate, _halfspaces.Count, count));
        _halfspaces.AddRange(fan[..count]);
        for (int index = 0; index < selection.Count; index++)
            _familySamples.Add(new FamilySample(family, selection[index]));
    }

    private static bool SupportsIncidentFan(PhysicsMesh mesh, int owner, BoundaryCandidate candidate,
        ReadOnlySpan<Vector3d> vertices)
    {
        Span<WideAxis3> fan = stackalloc WideAxis3[4];
        int count = GetIncidentFan(mesh, owner, candidate, vertices, candidate.Certificate.PointLocation, fan);
        foreach (WideAxis3 ray in fan[..count])
        {
            bool hasNormal = candidate.Certificate.TryGetNormalDotSign(ray, out int sign);
            System.Diagnostics.Debug.Assert(hasNormal);
            if (sign > 0) return false;
        }
        return true;
    }

    private static int GetIncidentFan(PhysicsMesh mesh, int owner, BoundaryCandidate candidate,
        ReadOnlySpan<Vector3d> vertices, ConeSurfacePointLocation location, Span<WideAxis3> fan)
    {
        int origin = location == ConeSurfacePointLocation.End ? candidate.Second : candidate.First;
        int opposite = origin == candidate.First ? candidate.Second : candidate.First;
        Vector3d p = vertices[origin], q = vertices[opposite];
        mesh.GetLocalTriangleVertices(owner, out Vector3d a, out Vector3d b, out Vector3d c);
        new FixedTriangle(a, b, c).GetExactNormal(out Signed192 nx, out Signed192 ny, out _, out _);
        int axis = !nx.IsZero ? 0 : !ny.IsZero ? 1 : 2;
        Vector2d projectedOrigin = Project(p, axis), projectedOpposite = Project(q, axis);
        fan[0] = Ray(p, q);
        if (location == ConeSurfacePointLocation.Interior)
        {
            fan[1] = Negate(fan[0]);
            int inward = -1;
            bool useX = projectedOpposite.X != projectedOrigin.X;
            int direction = useX ? projectedOpposite.X.CompareTo(projectedOrigin.X)
                : projectedOpposite.Y.CompareTo(projectedOrigin.Y);
            foreach (int incident in mesh.GetWeldedVertexTriangleIndices(origin))
            {
                if (mesh.GetManifoldSurfaceOwner(incident) != owner) continue;
                ReadOnlySpan<int> triangle = mesh.GetWeldedTriangleVertexIndices(incident);
                bool onEdge = false;
                foreach (int vertex in triangle)
                {
                    Vector2d v = Project(vertices[vertex], axis);
                    // Only the local ray direction matters. A positive
                    // collinear incident edge supplies the same boundary
                    // half-neighborhood after ±edge enforces tangency.
                    if (v != projectedOrigin && Vector2d.OrientationSign(projectedOrigin, projectedOpposite, v) == 0
                        && (useX ? v.X.CompareTo(projectedOrigin.X) : v.Y.CompareTo(projectedOrigin.Y)) == direction) onEdge = true;
                }
                if (!onEdge) continue;
                foreach (int vertex in triangle)
                    if (Vector2d.OrientationSign(projectedOrigin, projectedOpposite, Project(vertices[vertex], axis)) != 0)
                        inward = vertex;
                break;
            }
            System.Diagnostics.Debug.Assert(inward >= 0);
            fan[2] = Ray(p, vertices[inward]);
            return 3;
        }
        int neighbor = -1;
        ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(owner);
        for (int edge = 0; edge < boundary.Length; edge += 2)
        {
            int first = boundary[edge], second = boundary[edge + 1];
            if (first == origin && second != opposite) neighbor = second;
            else if (second == origin && first != opposite) neighbor = first;
        }
        System.Diagnostics.Debug.Assert(neighbor >= 0);
        fan[1] = Ray(p, vertices[neighbor]);
        Vector2d projectedNeighbor = Project(vertices[neighbor], axis);
        int orientation = Vector2d.OrientationSign(projectedOrigin, projectedOpposite, projectedNeighbor);
        // A certified simple planar boundary has exactly two noncollinear
        // corner rays. Check the actual incident sectors before choosing their
        // minor positive hull: a reflex fan has only plane-normal support.
        foreach (int incident in mesh.GetWeldedVertexTriangleIndices(origin))
        {
            if (mesh.GetManifoldSurfaceOwner(incident) != owner) continue;
            foreach (int vertex in mesh.GetWeldedTriangleVertexIndices(incident))
            {
                Vector2d v = Project(vertices[vertex], axis);
                if (orientation * Vector2d.OrientationSign(projectedOrigin, projectedOpposite, v) >= 0
                    && orientation * Vector2d.OrientationSign(projectedOrigin, v, projectedNeighbor) >= 0) continue;
                fan[2] = Negate(fan[0]); fan[3] = Negate(fan[1]);
                return 4;
            }
        }
        return 2;
    }

    private static Vector2d Project(Vector3d value, int axis) => axis == 0
        ? new Vector2d(value.Y, value.Z) : axis == 1 ? new Vector2d(value.X, value.Z) : new Vector2d(value.X, value.Y);

    private static WideAxis3 Negate(WideAxis3 value) => new(
        WideArithmetic.Negate(value.X), WideArithmetic.Negate(value.Y), WideArithmetic.Negate(value.Z));

    private static WideAxis3 Ray(Vector3d first, Vector3d second) => new(
        Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(second.X), Signed192.Raw(first.X))),
        Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(second.Y), Signed192.Raw(first.Y))),
        Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(second.Z), Signed192.Raw(first.Z))));

    private readonly struct FamilyCandidate
    {
        internal readonly BoundaryCandidate Candidate;
        internal readonly int Offset, Count;
        internal FamilyCandidate(BoundaryCandidate candidate, int offset, int count)
        { Candidate = candidate; Offset = offset; Count = count; }
    }

    private readonly struct FamilySample
    {
        internal readonly int Family;
        internal readonly ConeSurfaceFamilyEvent Event;
        internal FamilySample(int family, ConeSurfaceFamilyEvent descriptor) { Family = family; Event = descriptor; }
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
