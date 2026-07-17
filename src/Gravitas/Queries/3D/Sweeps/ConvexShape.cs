//=======================================================================
// ConvexSweepQueryWorker.ConvexShape.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;

namespace Gravitas.Queries;

internal readonly struct ConvexShape
{
    internal enum ConvexShapeKind
    {
        Collider,
        Triangle,
        CircleSlab
    }

    private readonly ConvexShapeKind _kind;
    private readonly LSCollider? _collider;
    private readonly LSMeshCollider? _triangleOwner;
    private readonly int _triangleIndex;
    private readonly Vector3d _offset;
    private readonly Vector3d _triangleA;
    private readonly Vector3d _triangleB;
    private readonly Vector3d _triangleC;
    private readonly Vector3d _center;
    private readonly Fixed64 _radius;
    private readonly Fixed64 _halfHeight;

    public ConvexShape(LSCollider collider, Vector3d offset)
    {
        _kind = ConvexShapeKind.Collider;
        _collider = collider;
        _triangleOwner = null;
        _triangleIndex = -1;
        _offset = offset;
        _triangleA = Vector3d.Zero;
        _triangleB = Vector3d.Zero;
        _triangleC = Vector3d.Zero;
        _center = Vector3d.Zero;
        _radius = Fixed64.Zero;
        _halfHeight = Fixed64.Zero;
    }

    public ConvexShape(
        LSMeshCollider triangleOwner,
        int triangleIndex,
        Vector3d triangleA,
        Vector3d triangleB,
        Vector3d triangleC)
    {
        _kind = ConvexShapeKind.Triangle;
        _collider = null;
        _triangleOwner = triangleOwner;
        _triangleIndex = triangleIndex;
        _offset = Vector3d.Zero;
        _triangleA = triangleA;
        _triangleB = triangleB;
        _triangleC = triangleC;
        _center = Vector3d.Zero;
        _radius = Fixed64.Zero;
        _halfHeight = Fixed64.Zero;
    }

    private ConvexShape(Vector3d center, Fixed64 radius, Fixed64 halfHeight, Vector3d offset)
    {
        _kind = ConvexShapeKind.CircleSlab;
        _collider = null;
        _triangleOwner = null;
        _triangleIndex = -1;
        _offset = offset;
        _triangleA = Vector3d.Zero;
        _triangleB = Vector3d.Zero;
        _triangleC = Vector3d.Zero;
        _center = center;
        _radius = radius;
        _halfHeight = halfHeight;
    }

    public Vector3d Center => _kind switch
    {
        ConvexShapeKind.Triangle => new Vector3d(
            FixedMath.Average(_triangleA.X, _triangleB.X, _triangleC.X),
            FixedMath.Average(_triangleA.Y, _triangleB.Y, _triangleC.Y),
            FixedMath.Average(_triangleA.Z, _triangleB.Z, _triangleC.Z)),
        ConvexShapeKind.CircleSlab => _center + _offset,
        _ => _collider!.Center + _offset
    };

    public bool IsTriangle => _kind == ConvexShapeKind.Triangle;

    public bool ContainsCenter =>
        _kind != ConvexShapeKind.Collider || _collider is not LSMeshCollider;

    public static ConvexShape CreateCircleSlab(Vector3d center, Fixed64 radius, Fixed64 halfHeight) =>
        new(center, radius, halfHeight, Vector3d.Zero);

    public void GetSourceBounds(out Vector3d min, out Vector3d max) =>
        GetBounds(out min, out max);

    public void GetBounds(out Vector3d min, out Vector3d max)
    {
        if (_kind == ConvexShapeKind.CircleSlab)
        {
            Vector3d center = _center + _offset;
            Vector3d extents = new(_radius, _halfHeight, _radius);
            min = center - extents;
            max = center + extents;
            return;
        }

        if (_kind == ConvexShapeKind.Triangle)
        {
            min = new Vector3d(
                FixedMath.Min(_triangleA.X, FixedMath.Min(_triangleB.X, _triangleC.X)),
                FixedMath.Min(_triangleA.Y, FixedMath.Min(_triangleB.Y, _triangleC.Y)),
                FixedMath.Min(_triangleA.Z, FixedMath.Min(_triangleB.Z, _triangleC.Z)));
            max = new Vector3d(
                FixedMath.Max(_triangleA.X, FixedMath.Max(_triangleB.X, _triangleC.X)),
                FixedMath.Max(_triangleA.Y, FixedMath.Max(_triangleB.Y, _triangleC.Y)),
                FixedMath.Max(_triangleA.Z, FixedMath.Max(_triangleB.Z, _triangleC.Z)));
            return;
        }

        min = _collider!.Bounds.Min + _offset;
        max = _collider.Bounds.Max + _offset;
    }

    public ConvexShape WithSourceOffset(Vector3d additionalOffset) =>
        _kind == ConvexShapeKind.CircleSlab
            ? new ConvexShape(_center, _radius, _halfHeight, _offset + additionalOffset)
            : new ConvexShape(_collider!, _offset + additionalOffset);

    public Vector3d Support(Vector3d direction)
    {
        if (_kind == ConvexShapeKind.Triangle)
            return SupportTriangle(direction);

        if (_kind == ConvexShapeKind.CircleSlab)
            return SupportCircleSlab(direction);

        return ConvexColliderSupport.Support(_collider!, direction) + _offset;
    }

    public bool TryGetClosestPointOnSurface(Vector3d point, out Vector3d closest)
    {
        SwiftThrowHelper.ThrowIfTrue(
            _kind == ConvexShapeKind.Triangle,
            nameof(ConvexShape),
            "Triangle shapes are stationary mesh targets and cannot be sweep sources.");

        if (_kind == ConvexShapeKind.Collider)
        {
            if (_collider is LSMeshCollider)
            {
                closest = Vector3d.Zero;
                return false;
            }

            closest = _collider!.ClosestPointOnSurface(point - _offset) + _offset;
            return true;
        }

        Vector3d center = _center + _offset;
        Vector3d local = point - center;
        Vector3d radial = new(local.X, Fixed64.Zero, local.Z);
        Fixed64 radialDistance = radial.Magnitude;
        Vector3d radialDirection = radialDistance > Fixed64.Epsilon
            ? radial / radialDistance
            : Vector3d.Right;
        Fixed64 clampedY = FixedMath.Clamp(local.Y, -_halfHeight, _halfHeight);

        if (radialDistance <= _radius && local.Y >= -_halfHeight && local.Y <= _halfHeight)
        {
            Fixed64 sideDistance = _radius - radialDistance;
            Fixed64 capDistance = _halfHeight - local.Y.Abs();
            closest = sideDistance <= capDistance
                ? center + new Vector3d(radialDirection.X * _radius, local.Y, radialDirection.Z * _radius)
                : center + new Vector3d(local.X, local.Y.Sign() * _halfHeight, local.Z);
            return true;
        }

        Fixed64 surfaceRadius = radialDistance > _radius ? _radius : radialDistance;
        closest = center + new Vector3d(
            radialDirection.X * surfaceRadius,
            clampedY,
            radialDirection.Z * surfaceRadius);
        return true;
    }

    public bool TryGetPlanarSurfaceNormal(Vector3d point, out Vector3d normal)
    {
        SwiftThrowHelper.ThrowIfTrue(
            _kind == ConvexShapeKind.CircleSlab,
            nameof(ConvexShape),
            "Circle slabs are sweep sources and cannot be target shapes.");

        if (_kind == ConvexShapeKind.Triangle)
        {
            normal = _triangleOwner!.Mesh.GetFaceNormalWorld(_triangleIndex);
            return true;
        }

        return _collider!.TryGetPlanarSurfaceNormal(point, out normal);
    }

    private Vector3d SupportCircleSlab(Vector3d direction)
    {
        Vector3d radial = new(direction.X, Fixed64.Zero, direction.Z);
        Fixed64 radialMagnitude = radial.Magnitude;
        Vector3d radialSupport = radialMagnitude > Fixed64.Epsilon
            ? radial / radialMagnitude * _radius
            : Vector3d.Right * _radius;
        Fixed64 y = direction.Y >= Fixed64.Zero ? _halfHeight : -_halfHeight;
        return _center + _offset + new Vector3d(radialSupport.X, y, radialSupport.Z);
    }

    private Vector3d SupportTriangle(Vector3d direction)
    {
        Vector3d best = _triangleA;
        if (Vector3d.CompareProjection(_triangleB, best, direction) > 0)
            best = _triangleB;

        if (Vector3d.CompareProjection(_triangleC, best, direction) > 0)
            best = _triangleC;

        return best;
    }
}
