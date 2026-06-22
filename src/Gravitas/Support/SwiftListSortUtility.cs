//=======================================================================
// SwiftListSortUtility.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using SwiftCollections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Gravitas;

internal static class SwiftListSortUtility
{
    // Keep measured physics hot paths off Array.Sort-backed package sorting until the lower-stack
    // allocation signal is closed.
    private static readonly IntAscendingComparer IntComparer = new();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void SortAscendingInPlace(SwiftList<int> items) =>
        SortInPlace(items, IntComparer);

    public static void SortInPlace<T>(SwiftList<T> items, IComparer<T> comparer)
    {
        int count = items.Count;
        if (count <= 1)
            return;

        for (int start = (count >> 1) - 1; start >= 0; start--)
            SiftDown(items, comparer, start, count);

        for (int end = count - 1; end > 0; end--)
        {
            Swap(items, 0, end);
            SiftDown(items, comparer, 0, end);
        }
    }

    private static void SiftDown<T>(SwiftList<T> items, IComparer<T> comparer, int root, int count)
    {
        while (true)
        {
            int child = (root << 1) + 1;
            if (child >= count)
                return;

            int swapIndex = root;
            if (comparer.Compare(items[swapIndex], items[child]) < 0)
                swapIndex = child;

            int right = child + 1;
            if (right < count && comparer.Compare(items[swapIndex], items[right]) < 0)
                swapIndex = right;

            if (swapIndex == root)
                return;

            Swap(items, root, swapIndex);
            root = swapIndex;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Swap<T>(SwiftList<T> items, int first, int second) =>
        (items[second], items[first]) = (items[first], items[second]);

    private sealed class IntAscendingComparer : IComparer<int>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Compare(int left, int right) => left.CompareTo(right);
    }
}
