//=======================================================================
// DynamicCcdCandidateIndex.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using SwiftCollections;
using SwiftCollections.Query;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

internal sealed class DynamicCcdCandidateIndex
{
    private readonly SwiftList<Entry> _entries;
    private Fixed64 _maxExtentX;
    private bool _isSorted = true;

    public DynamicCcdCandidateIndex(int capacity = 0)
    {
        _entries = capacity > 0 ? new SwiftList<Entry>(capacity) : new SwiftList<Entry>();
    }

    public int Count => _entries.Count;

    public void Clear()
    {
        _entries.FastClear();
        _maxExtentX = Fixed64.Zero;
        _isSorted = true;
    }

    public void Add(int dynamicId, FixedBoundVolume bounds)
    {
        _entries.Add(new Entry(dynamicId, bounds));
        Fixed64 extentX = bounds.Max.X - bounds.Min.X;
        if (extentX > _maxExtentX)
            _maxExtentX = extentX;

        _isSorted = false;
    }

    public void Sort()
    {
        if (!_isSorted && _entries.Count > 1)
            HeapSort();

        _isSorted = true;
    }

    public void Query(FixedBoundVolume queryBounds, SwiftList<int> results)
    {
        results.FastClear();
        if (_entries.Count == 0)
            return;

        Sort();
        Fixed64 scanMinX = queryBounds.Min.X - _maxExtentX;
        int index = FindFirstCandidateIndex(scanMinX);
        for (; index < _entries.Count; index++)
        {
            Entry entry = _entries[index];
            if (entry.MinX > queryBounds.Max.X)
                break;

            if (entry.Intersects(queryBounds))
                results.Add(entry.DynamicId);
        }
    }

    public static FixedBoundVolume CreateSweptSphereBounds(Vector3d start, Vector3d displacement, Fixed64 radius)
        => CreateBoundsBetween(start, start + displacement, Vector3d.One * radius);

    public static FixedBoundVolume CreateSweptBounds(
        Vector3d start,
        Vector3d displacement,
        Vector3d extents) =>
        CreateBoundsBetween(start, start + displacement, extents);

    public static FixedBoundVolume CreateBoundsBetween(
        Vector3d start,
        Vector3d end,
        Vector3d extents)
    {
        return new FixedBoundVolume(Vector3d.Min(start, end) - extents, Vector3d.Max(start, end) + extents);
    }

    private int FindFirstCandidateIndex(Fixed64 minX)
    {
        int low = 0;
        int high = _entries.Count;
        while (low < high)
        {
            int middle = low + ((high - low) >> 1);
            if (_entries[middle].MinX < minX)
                low = middle + 1;
            else
                high = middle;
        }

        return low;
    }

    private void HeapSort()
    {
        int count = _entries.Count;
        for (int start = (count >> 1) - 1; start >= 0; start--)
            SiftDown(start, count);

        for (int end = count - 1; end > 0; end--)
        {
            Swap(0, end);
            SiftDown(0, end);
        }
    }

    private void SiftDown(int root, int count)
    {
        while (true)
        {
            int child = (root << 1) + 1;
            if (child >= count)
                return;

            int swapIndex = root;
            if (Compare(_entries[swapIndex], _entries[child]) < 0)
                swapIndex = child;

            int right = child + 1;
            if (right < count && Compare(_entries[swapIndex], _entries[right]) < 0)
                swapIndex = right;

            if (swapIndex == root)
                return;

            Swap(root, swapIndex);
            root = swapIndex;
        }
    }

    private void Swap(int first, int second) =>
        (_entries[second], _entries[first]) = (_entries[first], _entries[second]);

    private static int Compare(Entry x, Entry y)
    {
        int result = x.MinX.CompareTo(y.MinX);
        if (result != 0)
            return result;

        result = x.MinY.CompareTo(y.MinY);
        if (result != 0)
            return result;

        result = x.MinZ.CompareTo(y.MinZ);
        if (result != 0)
            return result;

        result = x.MaxX.CompareTo(y.MaxX);
        if (result != 0)
            return result;

        result = x.MaxY.CompareTo(y.MaxY);
        if (result != 0)
            return result;

        result = x.MaxZ.CompareTo(y.MaxZ);
        if (result != 0)
            return result;

        return x.DynamicId.CompareTo(y.DynamicId);
    }

    private readonly struct Entry
    {
        public Entry(int dynamicId, FixedBoundVolume bounds)
        {
            DynamicId = dynamicId;
            MinX = bounds.Min.X;
            MinY = bounds.Min.Y;
            MinZ = bounds.Min.Z;
            MaxX = bounds.Max.X;
            MaxY = bounds.Max.Y;
            MaxZ = bounds.Max.Z;
        }

        public int DynamicId { get; }
        public Fixed64 MinX { get; }
        public Fixed64 MinY { get; }
        public Fixed64 MinZ { get; }
        public Fixed64 MaxX { get; }
        public Fixed64 MaxY { get; }
        public Fixed64 MaxZ { get; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Intersects(FixedBoundVolume queryBounds)
        {
            return !(MinX > queryBounds.Max.X || MaxX < queryBounds.Min.X ||
                     MinY > queryBounds.Max.Y || MaxY < queryBounds.Min.Y ||
                     MinZ > queryBounds.Max.Z || MaxZ < queryBounds.Min.Z);
        }
    }
}

internal readonly struct DynamicCcdPlanarBounds
{
    public DynamicCcdPlanarBounds(Fixed64 minX, Fixed64 minZ, Fixed64 maxX, Fixed64 maxZ)
    {
        MinX = minX;
        MinZ = minZ;
        MaxX = maxX;
        MaxZ = maxZ;
    }

    public Fixed64 MinX { get; }
    public Fixed64 MinZ { get; }
    public Fixed64 MaxX { get; }
    public Fixed64 MaxZ { get; }
}

internal sealed class DynamicCcdCandidateIndex2D
{
    private readonly SwiftList<Entry> _entries;
    private Fixed64 _maxExtentX;
    private bool _isSorted = true;

    public DynamicCcdCandidateIndex2D(int capacity = 0)
    {
        _entries = capacity > 0 ? new SwiftList<Entry>(capacity) : new SwiftList<Entry>();
    }

    public int Count => _entries.Count;

    public void Clear()
    {
        _entries.FastClear();
        _maxExtentX = Fixed64.Zero;
        _isSorted = true;
    }

    public void Add(int dynamicId, DynamicCcdPlanarBounds bounds)
    {
        _entries.Add(new Entry(dynamicId, bounds));
        Fixed64 extentX = bounds.MaxX - bounds.MinX;
        if (extentX > _maxExtentX)
            _maxExtentX = extentX;

        _isSorted = false;
    }

    public void Sort()
    {
        if (!_isSorted && _entries.Count > 1)
            HeapSort();

        _isSorted = true;
    }

    public void Query(DynamicCcdPlanarBounds queryBounds, SwiftList<int> results)
    {
        results.FastClear();
        if (_entries.Count == 0)
            return;

        Sort();
        Fixed64 scanMinX = queryBounds.MinX - _maxExtentX;
        int index = FindFirstCandidateIndex(scanMinX);
        for (; index < _entries.Count; index++)
        {
            Entry entry = _entries[index];
            if (entry.MinX > queryBounds.MaxX)
                break;

            if (entry.Intersects(queryBounds))
                results.Add(entry.DynamicId);
        }
    }

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

    private int FindFirstCandidateIndex(Fixed64 minX)
    {
        int low = 0;
        int high = _entries.Count;
        while (low < high)
        {
            int middle = low + ((high - low) >> 1);
            if (_entries[middle].MinX < minX)
                low = middle + 1;
            else
                high = middle;
        }

        return low;
    }

    private void HeapSort()
    {
        int count = _entries.Count;
        for (int start = (count >> 1) - 1; start >= 0; start--)
            SiftDown(start, count);

        for (int end = count - 1; end > 0; end--)
        {
            Swap(0, end);
            SiftDown(0, end);
        }
    }

    private void SiftDown(int root, int count)
    {
        while (true)
        {
            int child = (root << 1) + 1;
            if (child >= count)
                return;

            int swapIndex = root;
            if (Compare(_entries[swapIndex], _entries[child]) < 0)
                swapIndex = child;

            int right = child + 1;
            if (right < count && Compare(_entries[swapIndex], _entries[right]) < 0)
                swapIndex = right;

            if (swapIndex == root)
                return;

            Swap(root, swapIndex);
            root = swapIndex;
        }
    }

    private void Swap(int first, int second) =>
        (_entries[second], _entries[first]) = (_entries[first], _entries[second]);

    private static int Compare(Entry x, Entry y)
    {
        int result = x.MinX.CompareTo(y.MinX);
        if (result != 0)
            return result;

        result = x.MinZ.CompareTo(y.MinZ);
        if (result != 0)
            return result;

        result = x.MaxX.CompareTo(y.MaxX);
        if (result != 0)
            return result;

        result = x.MaxZ.CompareTo(y.MaxZ);
        if (result != 0)
            return result;

        return x.DynamicId.CompareTo(y.DynamicId);
    }

    private readonly struct Entry
    {
        public Entry(int dynamicId, DynamicCcdPlanarBounds bounds)
        {
            DynamicId = dynamicId;
            MinX = bounds.MinX;
            MinZ = bounds.MinZ;
            MaxX = bounds.MaxX;
            MaxZ = bounds.MaxZ;
        }

        public int DynamicId { get; }
        public Fixed64 MinX { get; }
        public Fixed64 MinZ { get; }
        public Fixed64 MaxX { get; }
        public Fixed64 MaxZ { get; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Intersects(DynamicCcdPlanarBounds queryBounds)
        {
            return !(MinX > queryBounds.MaxX || MaxX < queryBounds.MinX ||
                     MinZ > queryBounds.MaxZ || MaxZ < queryBounds.MinZ);
        }
    }
}
