using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas.Queries;

public sealed partial class GravitasQuery2DService
{
    /// <summary>Finds the nearest eligible physical support at a stable between-frame boundary.</summary>
    /// <remarks>
    /// Excludes triggers, dynamic carriers, and pairs disabled by physical filters.
    /// Filters compound parts before selecting the nearest surface. Candidate storage may grow;
    /// this synchronous operation has no caller work ceiling. All non-Found results clear support.
    /// </remarks>
    public Physics2DSupportQueryStatus QuerySupport(in Physics2DSupportQuery request, out Physics2DSupportHit support)
    {
        support = default;
        LastQueryCandidateCount = 0;
        SwiftThrowHelper.ThrowIfNull(request.Source, nameof(request));
        SwiftThrowHelper.ThrowIfArgument(!request.Source.IsActive
            || !_context.Physics2D.TryGetColliderById(request.Source.Id, out var registered)
            || !ReferenceEquals(registered, request.Source), nameof(request),
            "Support source must be an active collider registered in this context.");
        if (!_context.RefreshQueryPartitions())
            return Physics2DSupportQueryStatus.WorldNotReady;

        Vector2d direction = -request.Up;
        if (!Fixed64.TryMultiplyAdd(direction.X, request.Distance, request.Center.X, out Fixed64 endX)
            || !Fixed64.TryMultiplyAdd(direction.Y, request.Distance, request.Center.Y, out Fixed64 endY))
            return Physics2DSupportQueryStatus.Unrepresentable;
        Vector2d end = new(endX, endY);
        Vector2d padding = new(request.Radius, request.Radius);
        Vector2d centerMin = new(FixedMath.Min(request.Center.X, end.X), FixedMath.Min(request.Center.Y, end.Y));
        Vector2d centerMax = new(FixedMath.Max(request.Center.X, end.X), FixedMath.Max(request.Center.Y, end.Y));
        if (!Vector2d.TrySubtract(centerMin, padding, out Vector2d min)
            || !Vector2d.TryAdd(centerMax, padding, out Vector2d max))
            return Physics2DSupportQueryStatus.Unrepresentable;

        EnsureCandidateCapacity();
        _context.Collisions2D.CollectBoundsCandidates(min, max, request.Layers,
            NextRaycastVersion(), raycastQuery: true, _queryCandidates);
        LastQueryCandidateCount = _queryCandidates.Count;
        bool found = false;
        Physics2DHit closest = default;
        for (int i = 0; i < _queryCandidates.Count; i++)
        {
            LSCollider2D collider = _queryCandidates[i];
            if (ReferenceEquals(collider, request.Source) || collider.IsTrigger
                || collider.Body?.MotionType == BodyMotionType.Dynamic
                || !_context.Physics2D.RequireCollisionPair(request.Source, collider)
                || !QueryDetection2D.TryFindSupport(request, end, collider, out Physics2DHit candidate)
                || !PhysicsHitSelectionPolicy.ShouldReplace(candidate, found, closest))
                continue;
            closest = candidate;
            found = true;
        }
        if (!found)
            return Physics2DSupportQueryStatus.NoSupport;
        support = new Physics2DSupportHit(closest, _context.FrameCount);
        return Physics2DSupportQueryStatus.Found;
    }
}
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
