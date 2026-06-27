//=======================================================================
// PhysicsMixedHitSorter.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

internal static class PhysicsMixedHitSorter
{
    internal static void SortByDistance(SwiftList<PhysicsMixedHit> hits)
    {
        SortByDistance(hits, 0, hits.Count);
    }

    internal static void SortByDistance(SwiftList<PhysicsMixedHit> hits, int start, int count)
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

    internal static bool ComesBefore(PhysicsMixedHit left, PhysicsMixedHit right) => Compare(left, right) < 0;

    private static void SiftDown(SwiftList<PhysicsMixedHit> hits, int start, int root, int count)
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
    private static int Compare(PhysicsMixedHit left, PhysicsMixedHit right)
    {
        int distance = left.Distance.CompareTo(right.Distance);
        if (distance != 0)
            return distance;

        int left3D = left.Collider3D?.Id ?? -1;
        int right3D = right.Collider3D?.Id ?? -1;
        int collider3D = left3D.CompareTo(right3D);
        if (collider3D != 0)
            return collider3D;

        int left2D = left.Collider2D?.Id ?? -1;
        int right2D = right.Collider2D?.Id ?? -1;
        return left2D.CompareTo(right2D);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Swap(SwiftList<PhysicsMixedHit> hits, int left, int right)
    {
        PhysicsMixedHit temp = hits[left];
        hits[left] = hits[right];
        hits[right] = temp;
    }
}
