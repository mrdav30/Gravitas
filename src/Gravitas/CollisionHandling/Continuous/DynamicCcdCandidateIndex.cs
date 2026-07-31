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
    private readonly SwiftDictionary<int, int>? _entryIndices;
    private Fixed64 _maxExtentX;
    private int _maxExtentXCount;
    private int _unrepresentableExtentXCount;
    private bool _isSorted = true;

    public DynamicCcdCandidateIndex(int capacity = 0, bool supportsUpdates = false)
    {
        _entries = capacity > 0 ? new SwiftList<Entry>(capacity) : new SwiftList<Entry>();
        _entryIndices = !supportsUpdates
            ? null
            : capacity > 0
                ? new SwiftDictionary<int, int>(capacity)
                : new SwiftDictionary<int, int>();
    }

    public int Count => _entries.Count;

    public void Clear()
    {
        _entries.FastClear();
        _entryIndices?.Clear();
        _maxExtentX = Fixed64.Zero;
        _maxExtentXCount = 0;
        _unrepresentableExtentXCount = 0;
        _isSorted = true;
    }

    public void Add(int dynamicId, FixedBoundVolume bounds)
    {
        _entryIndices?.Add(dynamicId, _entries.Count);
        _entries.Add(new Entry(dynamicId, bounds));
        IncludeExtent(bounds.Min.X, bounds.Max.X);

        _isSorted = false;
    }

    public void AddOrUpdate(int dynamicId, FixedBoundVolume bounds)
    {
        SwiftDictionary<int, int>? entryIndices = _entryIndices;
        SwiftThrowHelper.ThrowIfTrue(
            entryIndices == null,
            nameof(DynamicCcdCandidateIndex),
            "Candidate index was not configured for updates.");
        if (entryIndices.TryGetValue(dynamicId, out int index))
        {
            Entry previous = _entries[index];
            _entries[index] = new Entry(dynamicId, bounds);
            bool remainsSorted = _isSorted && IsOrderedAt(index);
            if (!DynamicCcdExtentMetadata.IsEquivalent(
                    previous.MinX,
                    previous.MaxX,
                    bounds.Min.X,
                    bounds.Max.X))
            {
                if (RemoveExtent(previous.MinX, previous.MaxX))
                    RebuildExtents();
                else
                    IncludeExtent(bounds.Min.X, bounds.Max.X);
            }

            _isSorted = remainsSorted;
            return;
        }

        Add(dynamicId, bounds);
    }

    public bool Remove(int dynamicId)
    {
        SwiftDictionary<int, int>? entryIndices = _entryIndices;
        if (entryIndices == null || !entryIndices.TryGetValue(dynamicId, out int index))
            return false;

        Entry removed = _entries[index];
        int lastIndex = _entries.Count - 1;
        if (index != lastIndex)
        {
            Entry moved = _entries[lastIndex];
            _entries[index] = moved;
            entryIndices[moved.DynamicId] = index;
        }

        _entries.RemoveAt(lastIndex);
        entryIndices.Remove(dynamicId);
        if (RemoveExtent(removed.MinX, removed.MaxX))
            RebuildExtents();
        _isSorted = false;
        return true;
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
        Fixed64 scanMinX = _unrepresentableExtentXCount > 0
            || !Fixed64.TrySubtract(queryBounds.Min.X, _maxExtentX, out Fixed64 representableScanMinX)
                ? Fixed64.MinValue
                : representableScanMinX;
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

    private void IncludeExtent(Fixed64 minX, Fixed64 maxX)
    {
        if (!Fixed64.TrySubtract(maxX, minX, out Fixed64 extentX))
        {
            _unrepresentableExtentXCount++;
            return;
        }

        if (extentX > _maxExtentX)
        {
            _maxExtentX = extentX;
            _maxExtentXCount = 1;
        }
        else if (extentX == _maxExtentX)
        {
            _maxExtentXCount++;
        }
    }

    private bool RemoveExtent(Fixed64 minX, Fixed64 maxX)
    {
        if (!Fixed64.TrySubtract(maxX, minX, out Fixed64 extentX))
        {
            _unrepresentableExtentXCount--;
            return false;
        }

        return extentX == _maxExtentX && --_maxExtentXCount == 0;
    }

    private void RebuildExtents()
    {
        _maxExtentX = Fixed64.Zero;
        _maxExtentXCount = 0;
        _unrepresentableExtentXCount = 0;
        for (int i = 0; i < _entries.Count; i++)
            IncludeExtent(_entries[i].MinX, _entries[i].MaxX);
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsOrderedAt(int index)
    {
        Entry entry = _entries[index];
        return (index == 0 || Compare(_entries[index - 1], entry) <= 0)
            && (index == _entries.Count - 1 || Compare(entry, _entries[index + 1]) <= 0);
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

    private void Swap(int first, int second)
    {
        Entry firstEntry = _entries[first];
        Entry secondEntry = _entries[second];
        _entries[first] = secondEntry;
        _entries[second] = firstEntry;
        if (_entryIndices != null)
        {
            _entryIndices[secondEntry.DynamicId] = first;
            _entryIndices[firstEntry.DynamicId] = second;
        }
    }

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
    private readonly SwiftDictionary<int, int>? _entryIndices;
    private Fixed64 _maxExtentX;
    private int _maxExtentXCount;
    private int _unrepresentableExtentXCount;
    private bool _isSorted = true;

    public DynamicCcdCandidateIndex2D(int capacity = 0, bool supportsUpdates = false)
    {
        _entries = capacity > 0 ? new SwiftList<Entry>(capacity) : new SwiftList<Entry>();
        _entryIndices = !supportsUpdates
            ? null
            : capacity > 0
                ? new SwiftDictionary<int, int>(capacity)
                : new SwiftDictionary<int, int>();
    }

    public int Count => _entries.Count;

    public void Clear()
    {
        _entries.FastClear();
        _entryIndices?.Clear();
        _maxExtentX = Fixed64.Zero;
        _maxExtentXCount = 0;
        _unrepresentableExtentXCount = 0;
        _isSorted = true;
    }

    public void Add(int dynamicId, DynamicCcdPlanarBounds bounds)
    {
        _entryIndices?.Add(dynamicId, _entries.Count);
        _entries.Add(new Entry(dynamicId, bounds));
        IncludeExtent(bounds.MinX, bounds.MaxX);

        _isSorted = false;
    }

    public void AddOrUpdate(int dynamicId, DynamicCcdPlanarBounds bounds)
    {
        SwiftDictionary<int, int>? entryIndices = _entryIndices;
        SwiftThrowHelper.ThrowIfTrue(
            entryIndices == null,
            nameof(DynamicCcdCandidateIndex2D),
            "Candidate index was not configured for updates.");
        if (entryIndices.TryGetValue(dynamicId, out int index))
        {
            Entry previous = _entries[index];
            _entries[index] = new Entry(dynamicId, bounds);
            bool remainsSorted = _isSorted && IsOrderedAt(index);
            if (!DynamicCcdExtentMetadata.IsEquivalent(
                    previous.MinX,
                    previous.MaxX,
                    bounds.MinX,
                    bounds.MaxX))
            {
                if (RemoveExtent(previous.MinX, previous.MaxX))
                    RebuildExtents();
                else
                    IncludeExtent(bounds.MinX, bounds.MaxX);
            }

            _isSorted = remainsSorted;
            return;
        }

        Add(dynamicId, bounds);
    }

    public bool Remove(int dynamicId)
    {
        SwiftDictionary<int, int>? entryIndices = _entryIndices;
        if (entryIndices == null || !entryIndices.TryGetValue(dynamicId, out int index))
            return false;

        Entry removed = _entries[index];
        int lastIndex = _entries.Count - 1;
        if (index != lastIndex)
        {
            Entry moved = _entries[lastIndex];
            _entries[index] = moved;
            entryIndices[moved.DynamicId] = index;
        }

        _entries.RemoveAt(lastIndex);
        entryIndices.Remove(dynamicId);
        if (RemoveExtent(removed.MinX, removed.MaxX))
            RebuildExtents();
        _isSorted = false;
        return true;
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
        Fixed64 scanMinX = _unrepresentableExtentXCount > 0
            || !Fixed64.TrySubtract(queryBounds.MinX, _maxExtentX, out Fixed64 representableScanMinX)
                ? Fixed64.MinValue
                : representableScanMinX;
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

    private void IncludeExtent(Fixed64 minX, Fixed64 maxX)
    {
        if (!Fixed64.TrySubtract(maxX, minX, out Fixed64 extentX))
        {
            _unrepresentableExtentXCount++;
            return;
        }

        if (extentX > _maxExtentX)
        {
            _maxExtentX = extentX;
            _maxExtentXCount = 1;
        }
        else if (extentX == _maxExtentX)
        {
            _maxExtentXCount++;
        }
    }

    private bool RemoveExtent(Fixed64 minX, Fixed64 maxX)
    {
        if (!Fixed64.TrySubtract(maxX, minX, out Fixed64 extentX))
        {
            _unrepresentableExtentXCount--;
            return false;
        }

        return extentX == _maxExtentX && --_maxExtentXCount == 0;
    }

    private void RebuildExtents()
    {
        _maxExtentX = Fixed64.Zero;
        _maxExtentXCount = 0;
        _unrepresentableExtentXCount = 0;
        for (int i = 0; i < _entries.Count; i++)
            IncludeExtent(_entries[i].MinX, _entries[i].MaxX);
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsOrderedAt(int index)
    {
        Entry entry = _entries[index];
        return (index == 0 || Compare(_entries[index - 1], entry) <= 0)
            && (index == _entries.Count - 1 || Compare(entry, _entries[index + 1]) <= 0);
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

    private void Swap(int first, int second)
    {
        Entry firstEntry = _entries[first];
        Entry secondEntry = _entries[second];
        _entries[first] = secondEntry;
        _entries[second] = firstEntry;
        if (_entryIndices != null)
        {
            _entryIndices[secondEntry.DynamicId] = first;
            _entryIndices[firstEntry.DynamicId] = second;
        }
    }

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

file static class DynamicCcdExtentMetadata
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsEquivalent(
        Fixed64 previousMinX,
        Fixed64 previousMaxX,
        Fixed64 currentMinX,
        Fixed64 currentMaxX)
    {
        bool previousRepresentable = Fixed64.TrySubtract(previousMaxX, previousMinX, out Fixed64 previousExtentX);
        bool currentRepresentable = Fixed64.TrySubtract(currentMaxX, currentMinX, out Fixed64 currentExtentX);
        return previousRepresentable == currentRepresentable
            && (!previousRepresentable || previousExtentX == currentExtentX);
    }
}
