//=======================================================================
// QueryDetection2D.Support.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

internal static partial class QueryDetection2D
{
    private static bool TryOffsetPoint(
        Vector2d point,
        Vector2d direction,
        Fixed64 distance,
        out Vector2d result)
    {
        bool representable = Fixed64.TryMultiplyAdd(direction.X, distance, point.X, out Fixed64 x)
            & Fixed64.TryMultiplyAdd(direction.Y, distance, point.Y, out Fixed64 y);
        result = representable ? new Vector2d(x, y) : default;
        return representable;
    }

    private static bool ContainsPointExact(
        LSCollider2D collider,
        Vector2d point)
    {
        if (collider is LSCircleCollider2D circle)
        {
            return FixedSegment2d.ContainsPointInCenteredCapsule(
                point,
                circle.Center,
                Vector2d.Right,
                Fixed64.Zero,
                circle.ScaledRadius,
                Fixed64.Zero);
        }

        if (collider is LSCapsuleCollider2D capsule)
            return capsule.ContainsPoint(point);
        if (collider is not LSAABBoxCollider2D
            && collider is not LSPolygonCollider2D)
        {
            return collider.ContainsPoint(point);
        }

        Span<Vector2d> scratch = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> vertexOffsets =
            GetConvexVertexOffsets(collider, scratch);
        return FixedConvex2dRelations.ContainsPoint(
            point,
            collider.Center,
            collider.ConvexRotation,
            vertexOffsets);
    }

    private static ReadOnlySpan<Vector2d> GetConvexVertexOffsets(
        LSCollider2D collider,
        Span<Vector2d> scratch)
    {
        if (collider is LSPolygonCollider2D polygon)
            return polygon.ScaledLocalVertices;

        int vertexCount = collider.VertexCount;
        for (int i = 0; i < vertexCount; i++)
            scratch[i] = collider.GetScaledLocalVertexUnchecked(i);
        return scratch.Slice(0, vertexCount);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void TryKeepEarlierHit(Physics2DHit candidate, ref bool found, ref Physics2DHit best)
    {
        if (!PhysicsHitSelectionPolicy.ShouldReplace(candidate, found, best))
            return;

        found = true;
        best = candidate;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void TryKeepCloserHit(Physics2DHit candidate, ref bool found, ref Physics2DHit best)
    {
        if (!PhysicsHitSelectionPolicy.ShouldReplaceDistance(candidate.Distance, found, best.Distance))
            return;

        found = true;
        best = candidate;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool SegmentBoundsOverlap(Vector2d start, Vector2d end, LSCollider2D collider)
    {
        Fixed64 minX = FixedMath.Min(start.X, end.X);
        Fixed64 maxX = FixedMath.Max(start.X, end.X);
        Fixed64 minY = FixedMath.Min(start.Y, end.Y);
        Fixed64 maxY = FixedMath.Max(start.Y, end.Y);
        return maxX >= collider.MinX
            && minX <= collider.MaxX
            && maxY >= collider.MinY
            && minY <= collider.MaxY;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool SweepBoundsOverlap(Vector2d start, Vector2d end, Fixed64 radius, LSCollider2D collider)
    {
        Fixed64 minX = FixedMath.Min(start.X, end.X) - radius;
        Fixed64 maxX = FixedMath.Max(start.X, end.X) + radius;
        Fixed64 minY = FixedMath.Min(start.Y, end.Y) - radius;
        Fixed64 maxY = FixedMath.Max(start.Y, end.Y) + radius;
        return maxX >= collider.MinX
            && minX <= collider.MaxX
            && maxY >= collider.MinY
            && minY <= collider.MaxY;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ResolveQueryFallbackNormal(Vector2d center, Vector2d colliderCenter)
    {
        Vector2d direction = center - colliderCenter;
        return direction.MagnitudeSquared > Fixed64.Epsilon
            ? direction.Normalized
            : Vector2d.Right;
    }
}
