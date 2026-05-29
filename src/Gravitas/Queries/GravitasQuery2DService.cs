using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Owns pure 2D query buffers and query dispatch for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed class GravitasQuery2DService
{
    private readonly GravitasWorldContext _context;
    private readonly SwiftList<LSCollider2D> _queryCandidates = new();

    /// <summary>
    /// Initializes a pure 2D query service for the supplied context.
    /// </summary>
    /// <param name="context">The owning world context.</param>
    public GravitasQuery2DService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    /// <summary>
    /// Gets the owning world context.
    /// </summary>
    public GravitasWorldContext Context => _context;

    internal int LastQueryCandidateCount { get; private set; }

    /// <summary>
    /// Resets context-local pure 2D query buffers.
    /// </summary>
    public void Reset()
    {
        _queryCandidates.FastClear();
        LastQueryCandidateCount = 0;
    }

    /// <summary>
    /// Writes all active pure 2D colliders overlapping the query circle into <paramref name="results"/>.
    /// </summary>
    /// <returns>The number of hits written to <paramref name="results"/>.</returns>
    public int OverlapCircleAll(Vector2d center, Fixed64 radius, SwiftList<Physics2DHit> results)
    {
        return OverlapCircleAll(center, radius, PhysicsLayerMask.All, results);
    }

    /// <summary>
    /// Writes all active pure 2D colliders on included layers that overlap the query circle.
    /// </summary>
    /// <returns>The number of hits written to <paramref name="results"/>.</returns>
    public int OverlapCircleAll(
        Vector2d center,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics2DHit> results)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        SwiftThrowHelper.ThrowIfArgument(radius < Fixed64.Zero, nameof(radius), "2D query radius cannot be negative.");

        results.FastClear();
        EnsureCandidateCapacity();
        _context.Collisions2D.CollectOverlapCircleCandidates(center, radius, layerMask, _queryCandidates);
        LastQueryCandidateCount = _queryCandidates.Count;
        for (int i = 0; i < _queryCandidates.Count; i++)
        {
            LSCollider2D collider = _queryCandidates[i];
            if (QueryDetection2D.TryOverlapCircle(center, radius, collider, out Physics2DHit hit))
                results.Add(hit);
        }

        Physics2DHitSorter.SortByDistance(results);
        return results.Count;
    }

    /// <summary>
    /// Finds the closest pure 2D collider hit by the segment from <paramref name="start"/> to <paramref name="end"/>.
    /// </summary>
    public bool Raycast(Vector2d start, Vector2d end, out Physics2DHit hit)
    {
        return Raycast(start, end, PhysicsLayerMask.All, out hit);
    }

    /// <summary>
    /// Finds the closest pure 2D collider on an included layer hit by the segment.
    /// </summary>
    public bool Raycast(Vector2d start, Vector2d end, PhysicsLayerMask layerMask, out Physics2DHit hit)
    {
        Vector2d segment = end - start;
        if (segment.SqrMagnitude == Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            hit = default;
            return false;
        }

        EnsureCandidateCapacity();
        _context.Collisions2D.CollectBoundsCandidates(
            CreateMin(start, end),
            CreateMax(start, end),
            layerMask,
            _queryCandidates);

        LastQueryCandidateCount = _queryCandidates.Count;
        bool found = false;
        Physics2DHit closest = default;
        for (int i = 0; i < _queryCandidates.Count; i++)
        {
            if (!QueryDetection2D.TryRaycast(start, end, _queryCandidates[i], out Physics2DHit candidate)
                || (found && !Physics2DHitSorter.ComesBefore(candidate, closest)))
            {
                continue;
            }

            closest = candidate;
            found = true;
        }

        hit = closest;
        return found;
    }

    /// <summary>
    /// Writes all pure 2D colliders hit by the segment into <paramref name="results"/>.
    /// </summary>
    public int RaycastAll(Vector2d start, Vector2d end, SwiftList<Physics2DHit> results)
    {
        return RaycastAll(start, end, PhysicsLayerMask.All, results);
    }

    /// <summary>
    /// Writes all pure 2D colliders on included layers hit by the segment into <paramref name="results"/>.
    /// </summary>
    public int RaycastAll(Vector2d start, Vector2d end, PhysicsLayerMask layerMask, SwiftList<Physics2DHit> results)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));

        results.FastClear();
        Vector2d segment = end - start;
        if (segment.SqrMagnitude == Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            return 0;
        }

        EnsureCandidateCapacity();
        _context.Collisions2D.CollectBoundsCandidates(
            CreateMin(start, end),
            CreateMax(start, end),
            layerMask,
            _queryCandidates);

        LastQueryCandidateCount = _queryCandidates.Count;
        for (int i = 0; i < _queryCandidates.Count; i++)
            if (QueryDetection2D.TryRaycast(start, end, _queryCandidates[i], out Physics2DHit hit))
                results.Add(hit);

        Physics2DHitSorter.SortByDistance(results);
        return results.Count;
    }

    private void EnsureCandidateCapacity()
    {
        int colliderCount = _context.Physics2D.ColliderCount;
        if (colliderCount > 0)
            _queryCandidates.EnsureCapacity(colliderCount);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d CreateMin(Vector2d first, Vector2d second) =>
        new(FixedMath.Min(first.x, second.x), FixedMath.Min(first.y, second.y));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d CreateMax(Vector2d first, Vector2d second) =>
        new(FixedMath.Max(first.x, second.x), FixedMath.Max(first.y, second.y));
}
