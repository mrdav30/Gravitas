//=======================================================================
// PhysicsMixedHit.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Result from an explicit mixed 3D/2D query.
/// </summary>
public readonly struct PhysicsMixedHit
{
    public PhysicsMixedHit(
        LSCollider? collider3D,
        LSCollider2D? collider2D,
        Vector3d point3D,
        Vector3d point2D,
        Vector3d normal3DTo2D,
        PhysicsQueryReducerKind reducerKind,
        Fixed64 distance,
        Vector3d direction3D)
    {
        Collider3D = collider3D;
        Collider2D = collider2D;
        Body3D = collider3D?.Body;
        Body2D = collider2D?.Body;
        Point3D = point3D;
        Point2D = point2D;
        Normal3DTo2D = normal3DTo2D;
        ReducerKind = reducerKind;
        Distance = distance;
        Direction3D = direction3D;
    }

    public LSCollider? Collider3D { get; }

    public LSCollider2D? Collider2D { get; }

    public StiffBody? Body3D { get; }

    public StiffBody2D? Body2D { get; }

    public Vector3d Point3D { get; }

    public Vector3d Point2D { get; }

    /// <summary>
    /// Normal pointing from the 3D collider or swept 3D source toward the embedded 2D volume.
    /// </summary>
    public Vector3d Normal3DTo2D { get; }

    /// <summary>
    /// Gets whether this mixed query hit was produced by an exact shape reducer or a conservative fallback.
    /// </summary>
    public PhysicsQueryReducerKind ReducerKind { get; }

    /// <summary>
    /// Distance travelled by the swept source center before impact.
    /// </summary>
    public Fixed64 Distance { get; }

    public Vector3d Direction3D { get; }

    /// <summary>
    /// Surface normal suitable for clamping a 3D source moving into an embedded 2D target.
    /// </summary>
    public Vector3d NormalFor3DSource
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => -Normal3DTo2D;
    }

    /// <summary>
    /// Planar normal suitable for clamping a 2D source moving into a 3D target.
    /// </summary>
    public Vector2d NormalFor2DSource
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            Vector2d normal = new(Normal3DTo2D.X, Normal3DTo2D.Z);
            return normal.MagnitudeSquared > Fixed64.Epsilon ? normal.Normalized : Vector2d.Zero;
        }
    }
}
