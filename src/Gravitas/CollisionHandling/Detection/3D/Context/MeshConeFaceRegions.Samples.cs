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

namespace Gravitas.CollisionHandling;

/// <content>Finite admitted sampling and exact coverage metrics on once-rounded world anchors in one translated frame.</content>
internal sealed partial class MeshConeFaceRegions
{
    private readonly SwiftList<PoolEvent> _poolEvents = new();
    private readonly SwiftList<Sample> _samplePool = new();
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
        SwiftThrowHelper.ThrowIfListIndexInvalid(index, GetSampleCount(region));
        return _samples[region * ContactManifold.MaxContactsPerGroup + index].Identity;
    }

    private void ReduceSamples(int region, WidePlaneMetrics metrics)
    {
        _samplePool.FastClear();
        _sampleCounts[region] = 0;
        int orientation = _regions[region].Orientation;
        var selected = BorrowWinner(2 * region + (orientation > 0 ? 0 : 1));
        if (!ConePlaneRayPointMaterialization.TryGetPointInFrame(_frame, selected.Point, selected.Root,
            _samplingOrigin, out Vector3d maximumPoint)
            || !TryMaterializeSample(selected, orientation, maximumPoint, out Sample maximum))
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
            int best = -1;
            Signed832 bestN = default, bestD = default;
            for (int candidate = 1; candidate < _samplePool.Count; candidate++)
            {
                if (chosen[..slot].IndexOf(candidate) >= 0) continue;
                Vector3d p = _samplePool[candidate].MeshPoint;
                Signed832 n, d = Signed832.ExtendValue(Signed192.Signed(1));
                if (slot == 1) n = metrics.GetSpanSquared(_samples[offset].MeshPoint, p);
                else if (slot == 2) n = metrics.GetAreaSquared(_samples[offset].MeshPoint, _samples[offset + 1].MeshPoint, p);
                else metrics.GetTriangleDistanceSquared(p, _samples[offset].MeshPoint,
                    _samples[offset + 1].MeshPoint, _samples[offset + 2].MeshPoint, out n, out d);
                int comparison = best < 0 ? 1 : WidePlaneMetrics.CompareRatios(n, d, bestN, bestD);
                if (comparison > 0 || comparison == 0 && ComparePoints(_samplePool[candidate], _samplePool[best]) < 0)
                { best = candidate; bestN = n; bestD = d; }
            }
            chosen[slot] = best; _samples[offset + slot] = _samplePool[best];
        }
        // Greedy coverage needs only the once-rounded mesh anchors. Round
        // exits/depths for the selected outputs, after validating every eligible
        // candidate's exact exit range. The exact maximum bounds all depths.
        for (int slot = 1; slot < count; slot++)
            _samples[offset + slot] = MaterializeSelected(_samples[offset + slot], orientation);
        _sampleCounts[region] = count;
    }

    private bool KeepSample(PoolEvent source, int orientation)
    {
        Ray ray = orientation > 0 ? source.Positive : source.Negative;
        if (!ray.HasValue) return true;
        if (!ray.IsRepresentable) return false;
        Vector3d p = source.MeshPoint;
        var sample = new Sample(ray.Source, ray.Event, p, default, default);
        // Pool points are already exactly distinct; only the separately
        // seeded maximum can coincide with a pool entry.
        Sample maximum = _samplePool[0];
        if (maximum.MeshPoint == p && ComparePoints(sample, maximum) == 0) return true;
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
            ContactQuadratic.At(selection.Values, selection.Signs, 0, ConePlaneRaySelection.FieldWords),
            ContactQuadratic.At(selection.Values, selection.Signs, 1, ConePlaneRaySelection.FieldWords), orientation, _samplingOrigin));
        return new Ray(selection.MaximumSource, selection.MaximumEvent, representable);
    }

    private Sample MaterializeSelected(Sample sample, int orientation)
    {
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<ulong> nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount];
        Span<int> ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointValues, pointSigns);
        Span<ulong> root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        bool exists = ConePlaneRayEvents.TryEvaluateEvent(sample.Source, _frame, sample.Event, point, root,
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
                ContactQuadratic.At(selection.Values, selection.Signs, 0, ConePlaneRaySelection.FieldWords),
                ContactQuadratic.At(selection.Values, selection.Signs, 1, ConePlaneRaySelection.FieldWords),
                orientation, _samplingOrigin, out Vector3d exit, out Vector3d coneLocal)) return false;
        bool meshLocal = ConePlaneRayPointMaterialization.TryGetAuthoredPoint(_frame, selection.Point, selection.Root,
            out Vector3d p);
        // Round authored identity directly from the exact point. The paired
        // exit owner independently rounds its cone-local anchor; neither is
        // recovered by inverting the once-rounded common-frame outputs.
        System.Diagnostics.Debug.Assert(meshLocal);
        ulong identity = ContactManifold.CreateContactId(ContactAnchor.FromWorldPoint(p), 0, ContactAnchor.FromWorldPoint(coneLocal), 0);
        sample = new Sample(selection.MaximumSource, selection.MaximumEvent, meshPoint, exit, depth, identity);
        return true;
    }

    // Availability and exact output range are retained separately. Opposite
    // direction overflow cannot poison the chosen direction's finite pool.
    private readonly struct Ray
    {
        internal readonly ConePlaneRayEventSource Source;
        internal readonly ConePlaneRayEvent Event;
        internal readonly bool HasValue, IsRepresentable;
        internal Ray(ConePlaneRayEventSource source, ConePlaneRayEvent descriptor, bool representable)
        { Source = source; Event = descriptor; HasValue = true; IsRepresentable = representable; }
    }

    private readonly struct PoolEvent
    {
        internal readonly ConePlaneRayEventSource Source;
        internal readonly ConePlaneRayEvent Event;
        internal readonly int Region;
        internal readonly Vector3d MeshPoint;
        internal readonly Ray Positive, Negative;
        internal PoolEvent(ConePlaneRayEventSource source, ConePlaneRayEvent descriptor, int region, Vector3d meshPoint, Ray positive, Ray negative)
        { Source = source; Event = descriptor; Region = region; MeshPoint = meshPoint; Positive = positive; Negative = negative; }
    }

    private readonly struct Sample
    {
        internal readonly ConePlaneRayEventSource Source;
        internal readonly ConePlaneRayEvent Event;
        internal readonly Vector3d MeshPoint, ConePoint;
        internal readonly Fixed64 Depth;
        internal readonly ulong Identity;
        internal Sample(ConePlaneRayEventSource source, ConePlaneRayEvent descriptor, Vector3d p, Vector3d q, Fixed64 depth, ulong identity = 0)
        { Source = source; Event = descriptor; MeshPoint = p; ConePoint = q; Depth = depth; Identity = identity; }
    }
}
