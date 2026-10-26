//=======================================================================
// MeshConeFaceRegions.Samples.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using SwiftCollections;
using System;
using System.Collections.Generic;

namespace Gravitas.CollisionHandling;

/// <content>Finite admitted sampling and exact coverage metrics on once-rounded world anchors in one translated frame.</content>
internal sealed partial class MeshConeFaceRegions
{
    private readonly SwiftList<PoolEvent> _poolEvents = new();
    private readonly SwiftList<Sample> _samplePool = new();
    private readonly IComparer<Sample> _sampleComparer;
    private Sample[] _samples = Array.Empty<Sample>();
    private int[] _sampleCounts = Array.Empty<int>();

    /// <summary>Gets the reduced count, throwing if eligible geometry could not be materialized.</summary>
    internal int GetSampleCount(int region)
    {
        int count = _sampleCounts[ValidateRegion(region)];
        if (count < 0) throw new OverflowException("An admitted face sample is outside the Fixed64 sampling range.");
        return count;
    }

    internal bool TryGetSample(int region, int index, out Vector3d meshPoint, out Vector3d conePoint, out Fixed64 depth)
    {
        meshPoint = conePoint = default; depth = default;
        if ((uint)region >= (uint)RegionCount || index < 0 || index >= _sampleCounts[region]) return false;
        Sample sample = _samples[region * ContactManifold.MaxContactsPerGroup + index];
        meshPoint = sample.MeshPoint; conePoint = sample.ConePoint; depth = sample.Depth;
        return true;
    }

    internal ulong GetSampleIdentity(int region, int index)
    {
        ChronicleGuard.ThrowIfListIndexInvalid(index, GetSampleCount(region));
        return _samples[region * ContactManifold.MaxContactsPerGroup + index].Identity;
    }

    private void ReduceSamples(int region, WidePlaneMetrics metrics)
    {
        _samplePool.FastClear();
        _sampleCounts[region] = 0;
        int orientation = _regions[region].Orientation;
        int peakSlot = 2 * region + (orientation > 0 ? 0 : 1);
        Winner winner = _winners[peakSlot];
        _currentMember = winner.Member; _currentSupplier = winner.Supplier; _currentSpan = winner.Span;
        _frame = GetFrame(winner.Member);
        int nativeOrientation = GetNativeOrientation(region, winner.Member, orientation);
        var selected = BorrowWinner(peakSlot);
        if (!ConePlaneRayPointMaterialization.TryGetPointInFrame(_frame, selected.Point, selected.Root,
            _samplingOrigin, out Vector3d maximumPoint)
            || !TryMaterializeSample(selected, nativeOrientation, maximumPoint, out Sample maximum))
        { _sampleCounts[region] = -1; return; }
        // Keep the exact regional maximum first. The rest of this finite pool
        // uses intrinsic and true-boundary constructions, independent of seams.
        _samplePool.Add(maximum);
        foreach (PoolEvent descriptor in _poolEvents.AsReadOnlySpan())
            if (descriptor.Region == region && !KeepSample(descriptor, orientation))
            { _sampleCounts[region] = -1; return; }
        System.Diagnostics.Debug.Assert(_samplePool.Count > 0);
        int count = Math.Min(ContactManifold.MaxContactsPerGroup, _samplePool.Count);
        int offset = region * ContactManifold.MaxContactsPerGroup;
        _samples[offset] = _samplePool[0];
        Span<int> chosen = stackalloc int[ContactManifold.MaxContactsPerGroup]; chosen[0] = 0;
        for (int slot = 1; slot < count; slot++)
        {
            int best = MeshPlaneSampleReduction.ChooseNext(_samplePool.AsReadOnlySpan(), chosen[..slot], metrics, _sampleComparer);
            chosen[slot] = best; _samples[offset + slot] = _samplePool[best];
        }
        // Greedy coverage needs only the once-rounded mesh anchors. Round
        // exits/depths for the selected outputs, after validating every eligible
        // candidate's exact exit range. The exact maximum bounds all depths.
        for (int slot = 1; slot < count; slot++)
            _samples[offset + slot] = MaterializeSelected(_samples[offset + slot], GetNativeOrientation(region, _samples[offset + slot].Member, orientation));
        _sampleCounts[region] = count;
    }

    private bool KeepSample(PoolEvent source, int orientation)
    {
        Ray ray = orientation > 0 ? source.Positive : source.Negative;
        if (!ray.HasValue) return true;
        if (!ray.IsRepresentable) return false;
        Vector3d p = source.MeshPoint;
        var sample = new Sample(ray.Source, ray.Event, p, default, default, member: ray.Member, supplier: source.Supplier, span: ray.Span);
        // Native cohorts and joined members can share an exact point.
        // Deduplicate before coverage, retaining the first canonical supplier.
        foreach (Sample existing in _samplePool.AsReadOnlySpan()[..(source.Member < 0 ? 1 : _samplePool.Count)])
            if (existing.MeshPoint == p && ComparePoints(sample, existing) == 0) return true;
        _samplePool.Add(sample);
        return true;
    }

    private int ComparePoints(Sample first, Sample second)
    {
        // With identity cone rotation, world X is a positive affine image of
        // exact cone-frame X. Distinct rounded X values therefore certify its
        // order. Equal X or a rotated cone still needs exact reconstruction;
        // rounded Y/Z cannot decide before a potentially distinct exact X.
        if (_frame.ConeRotation == FixedQuaternion.Identity)
        {
            int order = first.MeshPoint.X.CompareTo(second.MeshPoint.X);
            if (order != 0) return order;
        }
        if (first.Member >= 0 || second.Member >= 0) return CompareJoinedPoints(first, second);
        return ConePlaneRayEvents.CompareEventAnchors(first.Source, first.Event, second.Source, second.Event,
            _frame, includeCoincidentProvenance: false);
    }

    private Ray CheckRay(scoped in ConePlaneRaySelection selection, int orientation, bool pointRepresentable)
    {
        if (!selection.HasValue) return default;
        // A positive whole-cone certificate covers every admitted finite exit.
        // An inconclusive bound retains the original exact per-ray range test.
        bool representable = pointRepresentable && (_wholeConeSamplingRangeRepresentable || ConePlaneRayPointMaterialization.IsExitPointRepresentable(
            _frame, selection.Point, selection.Root,
            ContactQuadratic.At(selection.Values, selection.Signs, 0, _fieldWords),
            ContactQuadratic.At(selection.Values, selection.Signs, 1, _fieldWords), orientation, _samplingOrigin));
        return new Ray(selection.MaximumSource, selection.MaximumEvent, representable, _currentMember, _currentSpan);
    }

    private Sample MaterializeSelected(Sample sample, int orientation)
    {
        _currentMember = sample.Member; _currentSupplier = sample.Supplier; _currentSpan = sample.Span;
        _frame = GetFrame(sample.Member);
        Span<ulong> pv = stackalloc ulong[SelectionWords];
        Span<ulong> nv = stackalloc ulong[SelectionWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps, _fieldWords, _rootWords); var negative = new ConePlaneRaySelection(nv, ns, _fieldWords, _rootWords);
        Span<ulong> pointValues = stackalloc ulong[PointWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointValues, pointSigns, _fieldWords);
        Span<ulong> root = stackalloc ulong[_rootWords];
        bool exists = ConePlaneRayEvents.TryEvaluateEvent(sample.Source, _frame, sample.Event, GetClip(sample.Member, sample.Span), point, root,
            ref positive, ref negative, requestedOrientation: orientation);
        ConePlaneRaySelection selected = orientation > 0 ? positive : negative;
        System.Diagnostics.Debug.Assert(exists && selected.HasValue);
        bool represented = TryMaterializeSample(selected, orientation, sample.MeshPoint, out Sample result);
        // The source/frame is unchanged and every eligible exit range was
        // checked exactly before reduction; replay preserves that certificate.
        System.Diagnostics.Debug.Assert(represented);
        return result;
    }

    private bool TryMaterializeSample(scoped in ConePlaneRaySelection selection, int orientation,
        Vector3d meshPoint, out Sample sample)
    {
        sample = default;
        // The chosen regional minimum is at most half the cone's projection
        // width. Its enclosing sphere has radius <= max(height,radius), so
        // every selected depth fits Fixed64; the shared getter guards defects.
        Fixed64 depth = selection.GetRoundedMaximumDepth();
        if (!ConePlaneRayPointMaterialization.TryGetExitPointsInFrame(_frame, selection.Point, selection.Root,
                ContactQuadratic.At(selection.Values, selection.Signs, 0, _fieldWords),
                ContactQuadratic.At(selection.Values, selection.Signs, 1, _fieldWords),
                orientation, _samplingOrigin, out Vector3d exit, out Vector3d coneLocal)) return false;
        ref readonly ConePlaneRayFrame supplierFrame = ref GetFrame(_currentSupplier);
        bool meshLocal = ConePlaneRayPointMaterialization.TryGetAuthoredPoint(supplierFrame, selection.Point, selection.Root,
            _frame, out Vector3d p);
        // Round authored identity directly from the exact point. The paired
        // exit owner independently rounds its cone-local anchor; neither is
        // recovered by inverting the once-rounded common-frame outputs.
        System.Diagnostics.Debug.Assert(meshLocal);
        ulong identity = ContactManifold.CreateContactId(ContactAnchor.FromWorldPoint(p), 0, ContactAnchor.FromWorldPoint(coneLocal), 0);
        sample = new Sample(selection.MaximumSource, selection.MaximumEvent, meshPoint, exit, depth, identity, _currentMember, _currentSupplier, _currentSpan, p, coneLocal);
        return true;
    }

    // Availability and exact output range are retained separately. Opposite
    // direction overflow cannot poison the chosen direction's finite pool.
    private readonly struct Ray
    {
        internal readonly ConePlaneRayEventSource Source;
        internal readonly ConePlaneRayEvent Event;
        internal readonly bool HasValue, IsRepresentable;
        internal readonly int Member;
        internal readonly int Span;
        internal Ray(ConePlaneRayEventSource source, ConePlaneRayEvent descriptor, bool representable,
            int member = -1, int span = -1)
        { Source = source; Event = descriptor; HasValue = true; IsRepresentable = representable; Member = member; Span = span; }
    }

    private readonly struct PoolEvent
    {
        internal readonly ConePlaneRayEventSource Source;
        internal readonly ConePlaneRayEvent Event;
        internal readonly int Region;
        internal readonly Vector3d MeshPoint;
        internal readonly Ray Positive, Negative;
        internal readonly int Member, Supplier;
        internal PoolEvent(ConePlaneRayEventSource source, ConePlaneRayEvent descriptor, int region, Vector3d meshPoint, Ray positive, Ray negative, int member = -1, int supplier = -1)
        { Source = source; Event = descriptor; Region = region; MeshPoint = meshPoint; Positive = positive; Negative = negative; Member = member; Supplier = supplier; }
    }

    private readonly struct Sample : IPlaneContactSample
    {
        public Vector3d Point => MeshPoint;
        internal readonly ConePlaneRayEventSource Source;
        internal readonly ConePlaneRayEvent Event;
        internal readonly Vector3d MeshPoint, ConePoint, AuthoredPoint, ConeLocalPoint;
        internal readonly Fixed64 Depth;
        internal readonly ulong Identity;
        internal readonly int Member, Supplier;
        internal readonly int Span;
        internal Sample(ConePlaneRayEventSource source, ConePlaneRayEvent descriptor, Vector3d p, Vector3d q, Fixed64 depth, ulong identity = 0,
            int member = -1, int supplier = -1, int span = -1,
            Vector3d authored = default, Vector3d coneLocal = default)
        { Source = source; Event = descriptor; MeshPoint = p; ConePoint = q; Depth = depth; Identity = identity;
            Member = member; Supplier = supplier; Span = span; AuthoredPoint = authored; ConeLocalPoint = coneLocal; }
    }
}
