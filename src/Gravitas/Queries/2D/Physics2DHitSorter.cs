//=======================================================================
// Physics2DHitSorter.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

internal static class Physics2DHitSorter
{
    public static void SortByDistance(SwiftList<Physics2DHit> hits)
    {
        SortByDistance(hits, 0, hits.Count);
    }

    public static void SortByDistance(SwiftList<Physics2DHit> hits, int start, int count)
    {
        if (count < 2)
            return;

        for (int root = (count / 2) - 1; root >= 0; root--)
            SiftDown(hits, start, root, count);

        for (int end = count - 1; end > 0; end--)
        {
            Swap(hits, start, start + end);
            SiftDown(hits, start, 0, end);
        }
    }

    public static bool ComesBefore(Physics2DHit left, Physics2DHit right) => Compare(left, right) < 0;

    private static void SiftDown(SwiftList<Physics2DHit> hits, int start, int root, int count)
    {
        while (true)
        {
            int child = (root * 2) + 1;
            if (child >= count)
                return;

            int swapIndex = root;
            if (ComesBefore(hits[start + swapIndex], hits[start + child]))
                swapIndex = child;

            int right = child + 1;
            if (right < count && ComesBefore(hits[start + swapIndex], hits[start + right]))
                swapIndex = right;

            if (swapIndex == root)
                return;

            Swap(hits, start + root, start + swapIndex);
            root = swapIndex;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int Compare(Physics2DHit left, Physics2DHit right)
    {
        int distance = left.Distance.CompareTo(right.Distance);
        if (distance != 0)
            return distance;

        return left.Collider.Id.CompareTo(right.Collider.Id);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Swap(SwiftList<Physics2DHit> hits, int left, int right)
    {
        Physics2DHit temp = hits[left];
        hits[left] = hits[right];
        hits[right] = temp;
    }
}
