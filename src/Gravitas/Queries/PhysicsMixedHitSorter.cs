//=======================================================================
// PhysicsMixedHitSorter.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using SwiftCollections;

namespace Gravitas.Queries;

internal static class PhysicsMixedHitSorter
{
    internal static void SortByDistance(SwiftList<PhysicsMixedHit> hits)
    {
        for (int i = 1; i < hits.Count; i++)
        {
            PhysicsMixedHit value = hits[i];
            int index = i - 1;
            while (index >= 0 && Compare(hits[index], value) > 0)
            {
                hits[index + 1] = hits[index];
                index--;
            }

            hits[index + 1] = value;
        }
    }

    internal static bool ComesBefore(PhysicsMixedHit left, PhysicsMixedHit right) => Compare(left, right) < 0;

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
}
