//=======================================================================
// Physics3DHitSorter.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

internal static class Physics3DHitSorter
{
    internal static void SortByDistance(SwiftList<Physics3DHit> hits)
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

    private static void SiftDown(SwiftList<Physics3DHit> hits, int root, int count)
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
    private static bool ComesBefore(Physics3DHit left, Physics3DHit right)
    {
        int distanceCompare = left.Distance.CompareTo(right.Distance);
        if (distanceCompare != 0)
            return distanceCompare < 0;

        return (left.Collider?.Id ?? -1) < (right.Collider?.Id ?? -1);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Swap(SwiftList<Physics3DHit> hits, int left, int right)
    {
        Physics3DHit temp = hits[left];
        hits[left] = hits[right];
        hits[right] = temp;
    }
}
