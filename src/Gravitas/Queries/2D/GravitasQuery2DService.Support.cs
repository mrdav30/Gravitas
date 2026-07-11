//=======================================================================
// GravitasQuery2DService.Support.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Owns pure 2D query scratch, version, and bounds helpers.
/// </summary>
public sealed partial class GravitasQuery2DService
{
    private void EnsureCandidateCapacity()
    {
        int colliderCount = _context.Physics2D.ColliderCount;
        if (colliderCount > 0)
            _queryCandidates.EnsureCapacity(colliderCount);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private uint NextOverlapQueryVersion()
    {
        OverlapQueryVersion++;
        if (OverlapQueryVersion == 0)
        {
            ResetColliderOverlapQueryVersions();
            OverlapQueryVersion = 1;
        }
        return OverlapQueryVersion;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private uint NextRaycastVersion()
    {
        RaycastVersion++;
        if (RaycastVersion == 0)
        {
            ResetColliderRaycastVersions();
            RaycastVersion = 1;
        }
        return RaycastVersion;
    }

    private void ResetColliderOverlapQueryVersions()
    {
        for (int i = 0; i < _context.Physics2D.ColliderCount; i++)
            _context.Physics2D.GetColliderByServiceIndex(i).CircleQueryVersion = 0;
    }

    private void ResetColliderRaycastVersions()
    {
        for (int i = 0; i < _context.Physics2D.ColliderCount; i++)
            _context.Physics2D.GetColliderByServiceIndex(i).RaycastVersion = 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d CreateMin(Vector2d first, Vector2d second) =>
        new(FixedMath.Min(first.X, second.X), FixedMath.Min(first.Y, second.Y));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d CreateMax(Vector2d first, Vector2d second) =>
        new(FixedMath.Max(first.X, second.X), FixedMath.Max(first.Y, second.Y));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d CreateSweepMin(Vector2d first, Vector2d second, Fixed64 radius) =>
        new(FixedMath.Min(first.X, second.X) - radius, FixedMath.Min(first.Y, second.Y) - radius);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d CreateSweepMax(Vector2d first, Vector2d second, Fixed64 radius) =>
        new(FixedMath.Max(first.X, second.X) + radius, FixedMath.Max(first.Y, second.Y) + radius);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void CreateAabbVertices(Vector2d center, Vector2d halfExtents, Span<Vector2d> vertices)
    {
        vertices[0] = center - halfExtents;
        vertices[1] = new Vector2d(center.X + halfExtents.X, center.Y - halfExtents.Y);
        vertices[2] = center + halfExtents;
        vertices[3] = new Vector2d(center.X - halfExtents.X, center.Y + halfExtents.Y);
    }

    private static void CalculateAreaBounds(ReadOnlySpan<Vector2d> vertices, out Vector2d min, out Vector2d max)
    {
        min = vertices[0];
        max = min;
        for (int i = 1; i < vertices.Length; i++)
        {
            Vector2d vertex = vertices[i];
            min = new Vector2d(FixedMath.Min(min.X, vertex.X), FixedMath.Min(min.Y, vertex.Y));
            max = new Vector2d(FixedMath.Max(max.X, vertex.X), FixedMath.Max(max.Y, vertex.Y));
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsEligibleSweepCandidate(LSCollider2D collider, LSCollider2D? excludedCollider, bool includeTriggers)
    {
        if (!includeTriggers && collider.IsTrigger)
            return false;

        return excludedCollider == null
            || (!ReferenceEquals(collider, excludedCollider)
                && !ReferenceEquals(collider.AgentOrNull, excludedCollider.AgentOrNull)
                && !collider.IsSibling(excludedCollider));
    }
}
