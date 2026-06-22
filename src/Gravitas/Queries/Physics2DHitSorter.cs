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
        int count = hits.Count;
        if (count < 2)
            return;

        for (int root = (count / 2) - 1; root >= 0; root--)
            SiftDown(hits, root, count);

        for (int end = count - 1; end > 0; end--)
        {
            Swap(hits, 0, end);
            SiftDown(hits, 0, end);
        }
    }

    public static bool ComesBefore(Physics2DHit left, Physics2DHit right) => Compare(left, right) < 0;

    private static void SiftDown(SwiftList<Physics2DHit> hits, int root, int count)
    {
        while (true)
        {
            int child = (root * 2) + 1;
            if (child >= count)
                return;

            int swapIndex = root;
            if (ComesBefore(hits[swapIndex], hits[child]))
                swapIndex = child;

            int right = child + 1;
            if (right < count && ComesBefore(hits[swapIndex], hits[right]))
                swapIndex = right;

            if (swapIndex == root)
                return;

            Swap(hits, root, swapIndex);
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
