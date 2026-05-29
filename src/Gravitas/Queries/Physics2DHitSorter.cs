using SwiftCollections;

namespace Gravitas.Queries;

internal static class Physics2DHitSorter
{
    public static void SortByDistance(SwiftList<Physics2DHit> hits)
    {
        for (int i = 1; i < hits.Count; i++)
        {
            Physics2DHit value = hits[i];
            int index = i - 1;
            while (index >= 0 && Compare(hits[index], value) > 0)
            {
                hits[index + 1] = hits[index];
                index--;
            }

            hits[index + 1] = value;
        }
    }

    public static bool ComesBefore(Physics2DHit left, Physics2DHit right) => Compare(left, right) < 0;

    private static int Compare(Physics2DHit left, Physics2DHit right)
    {
        int distance = left.Distance.CompareTo(right.Distance);
        if (distance != 0)
            return distance;

        return left.Collider.Id.CompareTo(right.Collider.Id);
    }
}
