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
        MeshSurfaceConnectivity connectivity, MeshConeFaceRegions faces, ContactManifold manifold)
    {
        PhysicsMesh mesh = meshCollider.Mesh;
        ref readonly ConePlaneRayFrame frame = ref faces.Frame;
        WideAxis3 normal = frame.AuthoredNormal;
        int owner = mesh.GetManifoldSurfaceOwner(surfaceTriangle), surface = mesh.GetCanonicalSurfaceOrdinal(surfaceTriangle);
        ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
        _visitedCorners.Clear();
        for (int edge = 0; edge < boundary.Length; edge += 2)
        {
            int first = boundary[edge], second = boundary[edge + 1];
            int feature = checked(3 * (edge / 2));
            int region = MeshSurfaceBoundaryTopology.FindEdgeRegion(mesh, owner, first, second, connectivity, frame);
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
            ReduceSurfaceFeature(meshCollider, cone, surface, feature, faces.SamplingOrigin, manifold);
        }
    }

    private void ReduceSurfaceFeature(LSMeshCollider mesh, LSConeCollider cone, int surface, int feature,
        Vector3d samplingOrigin, ContactManifold manifold)
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
            if (!contact.FirstAnchor.TryGetPointInFrame(samplingOrigin, out Vector3d p)
                || !contact.SecondAnchor.TryGetPointInFrame(samplingOrigin, out Vector3d q))
                throw new OverflowException("An admitted boundary sample is outside the Fixed64 sampling range.");
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
        int count = MeshSurfaceBoundaryTopology.GetIncidentFan(mesh, owner,
            location == ConeSurfacePointLocation.End ? candidate.Second : candidate.First,
            location == ConeSurfacePointLocation.End ? candidate.First : candidate.Second,
            location == ConeSurfacePointLocation.Interior, fan);
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
        int count = MeshSurfaceBoundaryTopology.GetIncidentFan(mesh, owner,
            candidate.Certificate.PointLocation == ConeSurfacePointLocation.End ? candidate.Second : candidate.First,
            candidate.Certificate.PointLocation == ConeSurfacePointLocation.End ? candidate.First : candidate.Second,
            candidate.Certificate.PointLocation == ConeSurfacePointLocation.Interior, fan);
        if (selectedExit.HasValue) fan[count++] = selectedExit.Value;
        foreach (WideAxis3 ray in fan[..count])
        {
            bool hasNormal = candidate.Certificate.TryGetNormalDotSign(ray, out int sign);
            System.Diagnostics.Debug.Assert(hasNormal);
            if (sign > 0) return false;
        }
        return true;
    }

    private static WideAxis3 Negate(WideAxis3 value) => new(
        WideArithmetic.Negate(value.X), WideArithmetic.Negate(value.Y), WideArithmetic.Negate(value.Z));

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
