//=======================================================================
// DynamicCcdCandidateIndex2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Collections.Generic;
using FixedMathSharp;
using SwiftCollections;
using SwiftCollections.Query;

namespace Gravitas.CollisionHandling;

/// <summary>Indexes planar bounds while preserving CCD and projected-query candidate order.</summary>
internal sealed class DynamicCcdCandidateIndex2D
{
    private readonly SwiftFixedBVH2D<int> _index;
    private readonly bool _supportsUpdates;
    private readonly CandidateComparer _comparer;

    public DynamicCcdCandidateIndex2D(int capacity = 0, bool supportsUpdates = false)
    {
        _index = new SwiftFixedBVH2D<int>(capacity > 0 ? checked(capacity * 2) : 1);
        _supportsUpdates = supportsUpdates;
        _comparer = new CandidateComparer(_index);
    }

    public int Count => _index.Count;

    public void Clear() => _index.Clear();

    public void Add(int dynamicId, DynamicCcdPlanarBounds bounds) =>
        _index.Insert(dynamicId, ToVolume(bounds));

    public void AddOrUpdate(int dynamicId, DynamicCcdPlanarBounds bounds)
    {
        ChronicleGuard.ThrowIfTrue(!_supportsUpdates, nameof(DynamicCcdCandidateIndex2D),
            "Candidate index was not configured for updates.");
        _index.Insert(dynamicId, ToVolume(bounds));
    }

    public bool Remove(int dynamicId) => _supportsUpdates && _index.Remove(dynamicId);

    public void Query(DynamicCcdPlanarBounds queryBounds, SwiftList<int> results)
    {
        results.FastClear();
        _index.Query(ToVolume(queryBounds), results);
        if (results.Count > 1)
            results.SortInPlace(_comparer);
    }

    private static FixedBoundVolume2D ToVolume(DynamicCcdPlanarBounds bounds) =>
        new(new Vector2d(bounds.MinX, bounds.MinZ), new Vector2d(bounds.MaxX, bounds.MaxZ));

    public static DynamicCcdPlanarBounds CreateSweptCircleBounds(Vector2d start, Vector2d displacement, Fixed64 radius)
        => CreateBoundsBetween(start, start + displacement, radius);

    public static DynamicCcdPlanarBounds CreateBoundsBetween(
        Vector2d start,
        Vector2d end,
        Fixed64 radius)
    {
        Fixed64 minX = FixedMath.Min(start.X, end.X) - radius;
        Fixed64 maxX = FixedMath.Max(start.X, end.X) + radius;
        Fixed64 minZ = FixedMath.Min(start.Y, end.Y) - radius;
        Fixed64 maxZ = FixedMath.Max(start.Y, end.Y) + radius;
        return new DynamicCcdPlanarBounds(minX, minZ, maxX, maxZ);
    }

    private readonly struct CandidateComparer : IComparer<int>
    {
        private readonly SwiftFixedBVH2D<int> _index;

        public CandidateComparer(SwiftFixedBVH2D<int> index) => _index = index;

        public int Compare(int first, int second)
        {
            FixedBoundVolume2D a = _index.NodePool[_index.FindEntry(first)].Bounds;
            FixedBoundVolume2D b = _index.NodePool[_index.FindEntry(second)].Bounds;
            int result = a.Min.X.CompareTo(b.Min.X);
            if (result != 0) return result;
            result = a.Min.Y.CompareTo(b.Min.Y);
            if (result != 0) return result;
            result = a.Max.X.CompareTo(b.Max.X);
            if (result != 0) return result;
            result = a.Max.Y.CompareTo(b.Max.Y);
            return result != 0 ? result : first.CompareTo(second);
        }
    }
}
