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
    private readonly SwiftHashSet<int> _visitedCorners = new();
    private readonly SwiftList<SurfaceSample> _surfacePool = new();

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

    /// <summary>Adds independently reduced true-edge and corner constraints to the unpublished manifold.</summary>
    internal void BuildSurfaceContacts(LSMeshCollider meshCollider, LSConeCollider cone, int surfaceTriangle,
        MeshConeSurfaceConnectivity connectivity, MeshConeFaceRegions faces, ContactManifold manifold)
    {
        PhysicsMesh mesh = meshCollider.Mesh;
        mesh.GetLocalTriangleVertices(surfaceTriangle, out Vector3d a, out Vector3d b, out Vector3d c);
        var triangle = new FixedTriangle(a, b, c);
        var frame = new ConePlaneRayFrame(triangle, mesh.Origin, mesh.Rotation,
            cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius);
        WideAxis3 normal = frame.AuthoredNormal;
        int owner = mesh.GetManifoldSurfaceOwner(surfaceTriangle), surface = mesh.GetCanonicalSurfaceOrdinal(surfaceTriangle);
        ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
        _visitedCorners.Clear();
        for (int edge = 0; edge < boundary.Length; edge += 2)
        {
            int first = boundary[edge], second = boundary[edge + 1];
            int feature = checked(3 * (edge / 2));
            int region = FindEdgeRegion(mesh, owner, first, second, connectivity, frame, vertices);
            if (region >= 0) KeepSurfaceFeature(first, second, feature, region, pointOnly: false);
            if (_visitedCorners.Add(first) && frame.ContainsPoint(vertices[first]))
            {
                // A convex cone cuts this straight edge in one connected
                // interval. Its admitted authored subedges join through their
                // welded endpoints, so this corner shares the edge's region.
                System.Diagnostics.Debug.Assert(region >= 0);
                KeepSurfaceFeature(first, second, feature + 1, region, pointOnly: true);
            }
            if (_visitedCorners.Add(second) && frame.ContainsPoint(vertices[second]))
            {
                System.Diagnostics.Debug.Assert(region >= 0);
                KeepSurfaceFeature(second, first, feature + 2, region, pointOnly: true);
            }
        }

        void KeepSurfaceFeature(int first, int second, int feature, int region, bool pointOnly)
        {
            _isolated.FastClear(); _families.FastClear(); _familySamples.FastClear(); _halfspaces.FastClear();
            WideAxis3 selectedExit = faces.GetOrientation(region) > 0 ? normal : Negate(normal);
            // Use a point segment for each corner exactly once. Adjacent edges
            // must not duplicate its support branches or multiply its pressure.
            Span<SegmentConeSurfaceCandidate> featureStorage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
            Accumulate(mesh, owner, first, second, mesh.ScaledLocalVertices, cone.Center, cone.Rotation,
                cone.Height, cone.ScaledRadius, featureStorage, interiorOnly: !pointOnly, pointOnly, selectedExit);
            ReduceSurfaceFeature(meshCollider, cone, surface, feature, manifold);
        }
    }

    private static int FindEdgeRegion(PhysicsMesh mesh, int owner, int first, int second,
        MeshConeSurfaceConnectivity connectivity, in ConePlaneRayFrame frame, ReadOnlySpan<Vector3d> vertices)
    {
        // Reduced edges can span many authored subedges. Their first corner's
        // triangle need not touch the cone; locate an actually admitted piece.
        ReadOnlySpan<int> perimeter = mesh.GetManifoldSurfaceBoundaryVertexPairs(owner);
        if (perimeter.IsEmpty) return connectivity.GetRegionOrdinal(owner);
        mesh.GetLocalTriangleVertices(owner, out Vector3d a, out Vector3d b, out Vector3d c);
        new FixedTriangle(a, b, c).GetExactNormal(out Signed192 nx, out Signed192 ny, out _, out _);
        int axis = !nx.IsZero ? 0 : !ny.IsZero ? 1 : 2;
        Vector2d p = Project(vertices[first], axis), q = Project(vertices[second], axis);
        bool useX = p.X != q.X;
        Fixed64 low = FixedMath.Min(useX ? p.X : p.Y, useX ? q.X : q.Y);
        Fixed64 high = FixedMath.Max(useX ? p.X : p.Y, useX ? q.X : q.Y);
        for (int edge = 0; edge < perimeter.Length; edge += 2)
        {
            int start = perimeter[edge], end = perimeter[edge + 1];
            Vector2d s = Project(vertices[start], axis), t = Project(vertices[end], axis);
            if (Vector2d.OrientationSign(p, q, s) != 0 || Vector2d.OrientationSign(p, q, t) != 0
                || (useX ? s.X : s.Y) < low || (useX ? s.X : s.Y) > high
                || !frame.IntersectsSegment(new FixedSegment(vertices[start], vertices[end]))) continue;
            // A trusted exposed subedge with its start in this canonical span
            // cannot leave it: reduction merges monotone collinear chains,
            // while the embedding certificate rejects overlap/backtracking.
            foreach (int incident in mesh.GetWeldedVertexTriangleIndices(start))
            {
                if (mesh.GetWeldedTriangleVertexIndices(incident).IndexOf(end) < 0) continue;
                int region = connectivity.GetRegionOrdinal(incident);
                if (region >= 0) return region;
            }
        }
        return -1;
    }

    private void ReduceSurfaceFeature(LSMeshCollider mesh, LSConeCollider cone, int surface, int feature, ContactManifold manifold)
    {
        _surfacePool.FastClear();
        if (_isolated.Count == 0 && _families.Count == 0) return;
        SegmentConeSurfaceCandidate minimum = _isolated.Count > 0
            ? _isolated[0].Certificate : _families[0].Candidate.Certificate;
        foreach (BoundaryCandidate candidate in _isolated.AsReadOnlySpan())
            if (SegmentConeSurfaceCandidates.CompareDepths(candidate.Certificate, minimum) < 0) minimum = candidate.Certificate;
        foreach (FamilyCandidate family in _families.AsReadOnlySpan())
            if (SegmentConeSurfaceCandidates.CompareDepths(family.Candidate.Certificate, minimum) < 0) minimum = family.Candidate.Certificate;
        // A feature's support strata are alternative exits, not simultaneous
        // constraints. Retain its exact lower envelope before rounding; a far
        // apex exit cannot depenetrate a touching or barely penetrating side.
        // Clipped families retain their source certificate's constant depth.
        Span<bool> coMinimalFamilies = stackalloc bool[SegmentConeSurfaceCandidates.MaximumCandidates];
        System.Diagnostics.Debug.Assert(_families.Count <= coMinimalFamilies.Length);
        for (int index = 0; index < _families.Count; index++)
            coMinimalFamilies[index] = SegmentConeSurfaceCandidates.CompareDepths(_families[index].Candidate.Certificate, minimum) == 0;
        for (int index = 0; index < _isolated.Count; index++)
            if (SegmentConeSurfaceCandidates.CompareDepths(_isolated[index].Certificate, minimum) == 0) Keep(GetIsolatedContact(index));
        for (int index = 0; index < _familySamples.Count; index++)
            if (coMinimalFamilies[_familySamples[index].Family]) Keep(GetFamilySampleContact(index));
        // Each retained family emitted at least one admitted sample. A minimum
        // family or isolated certificate therefore seeds this pool before any
        // duplicate can be discarded.
        System.Diagnostics.Debug.Assert(_surfacePool.Count > 0);
        Span<int> chosen = stackalloc int[ContactManifold.MaxContactsPerGroup];
        // All survivors have the same exact depth. Their canonical construction
        // order supplies the first sample; coverage chooses the remaining three.
        chosen[0] = 0;
        int count = Math.Min(ContactManifold.MaxContactsPerGroup, _surfacePool.Count);
        for (int slot = 1; slot < count; slot++)
        {
            int best = -1, bestNearest = 0;
            for (int candidate = 0; candidate < _surfacePool.Count; candidate++)
            {
                if (chosen[..slot].IndexOf(candidate) >= 0) continue;
                int nearest = chosen[0];
                for (int prior = 1; prior < slot; prior++)
                    if (CompareCoverage(candidate, chosen[prior], candidate, nearest) < 0) nearest = chosen[prior];
                if (best < 0 || CompareCoverage(candidate, nearest, best, bestNearest) > 0)
                { best = candidate; bestNearest = nearest; }
            }
            chosen[slot] = best;
        }
        // Negative feature ordinals are disjoint from nonnegative face regions.
        // Four samples approximate this feature's distributed response. Final
        // rounded coverage never feeds back into exact admission or depth selection.
        // Coverage ranks only the admitted finite pool: normal spread first
        // (a touch family can share both anchors), then mesh/cone anchor spread.
        var key = new ContactGroupKey(0, 0, surface, 0, checked(-feature - 1));
        for (int slot = 0; slot < count; slot++)
        {
            FixedContactAnchors contact = _surfacePool[chosen[slot]].Contact;
            manifold.AddContact(new ContactAnchor(contact.FirstAnchor), new ContactAnchor(contact.SecondAnchor),
                contact.Depth, contact.Normal, mesh.Material, cone.Material, contact.DepthIsClamped,
                group: key, sampleIdentity: slot + 1);
        }

        void Keep(FixedContactAnchors contact)
        {
            if (!contact.FirstAnchor.TryGetPoint(out Vector3d p) || !contact.SecondAnchor.TryGetPoint(out Vector3d q))
                throw new OverflowException("An admitted boundary sample is outside the Fixed64 world range.");
            // Distinct exact constructions can round to the same solver row.
            // Collapse that final redundancy after admission; different normals
            // at coincident touch anchors remain independent samples.
            for (int index = 0; index < _surfacePool.Count; index++)
            {
                SurfaceSample existing = _surfacePool[index];
                if (existing.MeshPoint != p || existing.ConePoint != q || existing.Contact.Normal != contact.Normal) continue;
                return;
            }
            _surfacePool.Add(new SurfaceSample(contact, p, q));
        }
    }

    private int CompareCoverage(int first, int firstReference, int second, int secondReference)
    {
        SurfaceSample a = _surfacePool[first], ar = _surfacePool[firstReference];
        SurfaceSample b = _surfacePool[second], br = _surfacePool[secondReference];
        int comparison = Vector3d.CompareDistanceSquared(a.Contact.Normal, ar.Contact.Normal, b.Contact.Normal, br.Contact.Normal);
        if (comparison == 0) comparison = Vector3d.CompareDistanceSquared(a.MeshPoint, ar.MeshPoint, b.MeshPoint, br.MeshPoint);
        return comparison == 0 ? Vector3d.CompareDistanceSquared(a.ConePoint, ar.ConePoint, b.ConePoint, br.ConePoint) : comparison;
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
        Span<SegmentConeSurfaceCandidate> storage, bool interiorOnly = false, bool pointOnly = false, WideAxis3? selectedExit = null)
    {
        var selection = new ConeSurfaceSelection(storage);
        SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(
            new FixedSegment(vertices[first], vertices[pointOnly ? first : second]), mesh.Origin, mesh.Rotation,
            coneCenter, coneRotation, height, radius, ref selection);
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate certificate = selection[index];
            // Both endpoints of a point segment are the same corner. Retain
            // its construction, but use Start for the real incident-edge fan.
            if (pointOnly && certificate.PointLocation != ConeSurfacePointLocation.Start)
                certificate = new SegmentConeSurfaceCandidate(certificate.Input, certificate.Feature, certificate.Family,
                    certificate.Chart, certificate.RootOrdinal, ConeSurfacePointLocation.Start);
            var candidate = new BoundaryCandidate(first, second, certificate);
            if (interiorOnly && candidate.Certificate.Family != ConeSurfaceFamily.SegmentInterval
                && candidate.Certificate.PointLocation != ConeSurfacePointLocation.Interior) continue;
            if (candidate.Certificate.Family == ConeSurfaceFamily.None)
            {
                if (SupportsIncidentFan(mesh, owner, candidate, vertices, selectedExit)) _isolated.Add(candidate);
            }
            else if (candidate.Certificate.Family == ConeSurfaceFamily.SegmentInterval)
            {
                if (!interiorOnly) KeepFamily(mesh, owner, candidate, vertices, ConeSurfacePointLocation.Start, selectedExit);
                if (!pointOnly) KeepFamily(mesh, owner, candidate, vertices, ConeSurfacePointLocation.Interior, selectedExit);
                if (!interiorOnly && !pointOnly) KeepFamily(mesh, owner, candidate, vertices, ConeSurfacePointLocation.End, selectedExit);
            }
            else KeepFamily(mesh, owner, candidate, vertices, candidate.Certificate.PointLocation, selectedExit);
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
        ReadOnlySpan<Vector3d> vertices, ConeSurfacePointLocation location, WideAxis3? selectedExit = null)
    {
        Span<WideAxis3> fan = stackalloc WideAxis3[5];
        int count = GetIncidentFan(mesh, owner, candidate, vertices, location, fan);
        if (selectedExit.HasValue) fan[count++] = selectedExit.Value;
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
        ReadOnlySpan<Vector3d> vertices, WideAxis3? selectedExit = null)
    {
        Span<WideAxis3> fan = stackalloc WideAxis3[5];
        int count = GetIncidentFan(mesh, owner, candidate, vertices, candidate.Certificate.PointLocation, fan);
        if (selectedExit.HasValue) fan[count++] = selectedExit.Value;
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

    private readonly struct SurfaceSample
    {
        internal readonly FixedContactAnchors Contact;
        internal readonly Vector3d MeshPoint, ConePoint;
        internal SurfaceSample(FixedContactAnchors contact, Vector3d meshPoint, Vector3d conePoint)
        { Contact = contact; MeshPoint = meshPoint; ConePoint = conePoint; }
    }
}
