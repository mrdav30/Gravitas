//=======================================================================
// ConvexSweepQueryWorker.ConvexShape.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;
using System;

namespace Gravitas.Queries;

internal readonly struct ConvexShape
{
    internal enum ConvexShapeKind
    {
        Collider,
        Triangle,
        CircleSlab,
        Sphere
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

    private ConvexShape(
        ConvexShapeKind kind,
        Vector3d center,
        Fixed64 radius,
        Fixed64 halfHeight,
        Vector3d offset)
    {
        _kind = kind;
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

    public bool ContainsCenter =>
        _kind != ConvexShapeKind.Collider || _collider is not LSMeshCollider;

    public FixedPointAnchor GetCenterAnchor()
    {
        if (_kind == ConvexShapeKind.Triangle)
        {
            return new FixedPointAnchor(
                _triangleOwner!.Mesh.Origin,
                _triangleOwner.Mesh.Rotation,
                new Vector3d(
                    FixedMath.Average(
                        _triangleA.X,
                        _triangleB.X,
                        _triangleC.X),
                    FixedMath.Average(
                        _triangleA.Y,
                        _triangleB.Y,
                        _triangleC.Y),
                    FixedMath.Average(
                        _triangleA.Z,
                        _triangleB.Z,
                        _triangleC.Z)));
        }

        return new FixedPointAnchor(
            _kind is ConvexShapeKind.CircleSlab or ConvexShapeKind.Sphere
                ? _center
                : _collider!.Center,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            _offset);
    }

    public static ConvexShape CreateCircleSlab(Vector3d center, Fixed64 radius, Fixed64 halfHeight) =>
        new(ConvexShapeKind.CircleSlab, center, radius, halfHeight, Vector3d.Zero);

    public static ConvexShape CreateSphere(Vector3d center, Fixed64 radius) =>
        new(ConvexShapeKind.Sphere, center, radius, Fixed64.Zero, Vector3d.Zero);

    public void GetSourceBounds(out Vector3d min, out Vector3d max) =>
        GetBounds(out min, out max);

    public void GetBounds(out Vector3d min, out Vector3d max)
    {
        if (_kind is ConvexShapeKind.CircleSlab or ConvexShapeKind.Sphere)
        {
            Vector3d center = _center + _offset;
            Vector3d extents = _kind == ConvexShapeKind.Sphere
                ? Vector3d.One * _radius
                : new Vector3d(_radius, _halfHeight, _radius);
            min = center - extents;
            max = center + extents;
            return;
        }

        if (_kind == ConvexShapeKind.Triangle)
        {
            Vector3d localMin = Vector3d.Min(
                _triangleA,
                Vector3d.Min(_triangleB, _triangleC));
            Vector3d localMax = Vector3d.Max(
                _triangleA,
                Vector3d.Max(_triangleB, _triangleC));
            FixedBoundBox bounds =
                FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
                    _triangleOwner!.Mesh.Origin,
                    _triangleOwner.Mesh.Rotation,
                    localMin,
                    localMax,
                    Vector3d.Zero,
                    FixedQuaternion.Identity);
            min = bounds.Min;
            max = bounds.Max;
            return;
        }

        min = _collider!.Bounds.Min + _offset;
        max = _collider.Bounds.Max + _offset;
    }

    public bool CanTranslateCenter(Vector3d displacement)
    {
        Vector3d canonicalCenter =
            _kind is ConvexShapeKind.CircleSlab or ConvexShapeKind.Sphere
                ? _center
                : _collider!.Center;
        return Vector3d.TryAdd(
                canonicalCenter,
                _offset,
                out Vector3d currentCenter)
            && Vector3d.TryAdd(
                currentCenter,
                displacement,
                out _);
    }

    public bool TryGetBoundsRelativeTo(
        Vector3d referenceOrigin,
        FixedQuaternion referenceRotation,
        out Vector3d min,
        out Vector3d max)
    {
        FixedBoundBox relativeBounds;
        if (_kind is ConvexShapeKind.CircleSlab or ConvexShapeKind.Sphere)
        {
            Vector3d extents = _kind == ConvexShapeKind.Sphere
                ? Vector3d.One * _radius
                : new Vector3d(_radius, _halfHeight, _radius);
            relativeBounds =
                FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
                    _center,
                    FixedQuaternion.Identity,
                    -extents,
                    extents,
                    referenceOrigin,
                    referenceRotation);
        }
        else
        {
            relativeBounds = ColliderCanonicalBounds.GetRelativeBounds(
                _collider!,
                referenceOrigin,
                referenceRotation);
        }

        if (_offset == Vector3d.Zero)
        {
            min = relativeBounds.Min;
            max = relativeBounds.Max;
            return true;
        }

        // Source offsets come from a chord whose magnitude was admitted by
        // Prepare, so a unit rotation cannot move them outside Fixed64.
        _ = referenceRotation.Inverse().TryRotate(
            _offset,
            out Vector3d localOffset);
        if (!Vector3d.TryAdd(relativeBounds.Min, localOffset, out min)
            || !Vector3d.TryAdd(relativeBounds.Max, localOffset, out max))
        {
            min = default;
            max = default;
            return false;
        }

        return true;
    }

    public ConvexShape WithSourceOffset(Vector3d additionalOffset) =>
        _kind is ConvexShapeKind.CircleSlab or ConvexShapeKind.Sphere
            ? new ConvexShape(
                _kind,
                _center,
                _radius,
                _halfHeight,
                _offset + additionalOffset)
            : new ConvexShape(_collider!, _offset + additionalOffset);

    public FixedPointAnchor GetSupportAnchor(Vector3d direction)
    {
        if (_kind == ConvexShapeKind.Triangle)
        {
            return new FixedPointAnchor(
                _triangleOwner!.Mesh.Origin,
                _triangleOwner.Mesh.Rotation,
                GetTriangleSupportLocalPoint(direction));
        }

        if (_kind is ConvexShapeKind.CircleSlab or ConvexShapeKind.Sphere)
        {
            if (_kind == ConvexShapeKind.Sphere)
            {
                return FixedSegment.GetCenteredCapsuleSupportAnchor(
                        _center,
                        FixedQuaternion.Identity,
                        Fixed64.Zero,
                        _radius,
                        direction)
                    .WithLocalTranslation(_offset);
            }

            return new FixedPointAnchor(
                _center,
                FixedQuaternion.Identity,
                GetCircleSlabSupportLocalPoint(direction),
                _offset);
        }

        return ConvexColliderSupport.GetSupportAnchor(
            _collider!,
            direction,
            _offset);
    }

    public FixedPointAnchor GetFallbackSurfaceAnchor(Vector3d direction)
    {
        if (_kind == ConvexShapeKind.Collider
            && _collider is LSCuboidCollider cuboid)
        {
            Vector3d localDirection =
                cuboid.Rotation.Inverse().Rotate(direction);
            Vector3d absoluteDirection = Vector3d.Abs(localDirection);
            Vector3d halfExtents = cuboid.OrientedBox.HalfExtents;
            Vector3d localPoint;
            if (absoluteDirection.X >= absoluteDirection.Y
                && absoluteDirection.X >= absoluteDirection.Z)
            {
                localPoint = new Vector3d(
                    localDirection.X >= Fixed64.Zero
                        ? halfExtents.X
                        : -halfExtents.X,
                    Fixed64.Zero,
                    Fixed64.Zero);
            }
            else if (absoluteDirection.Y >= absoluteDirection.Z)
            {
                localPoint = new Vector3d(
                    Fixed64.Zero,
                    localDirection.Y >= Fixed64.Zero
                        ? halfExtents.Y
                        : -halfExtents.Y,
                    Fixed64.Zero);
            }
            else
            {
                localPoint = new Vector3d(
                    Fixed64.Zero,
                    Fixed64.Zero,
                    localDirection.Z >= Fixed64.Zero
                        ? halfExtents.Z
                        : -halfExtents.Z);
            }

            return new FixedPointAnchor(
                cuboid.Center,
                cuboid.Rotation,
                localPoint,
                _offset);
        }

        return GetSupportAnchor(direction);
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
        if (_kind == ConvexShapeKind.Sphere)
        {
            Vector3d direction = point - center;
            closest = center + GetSphereSupportLocalPoint(direction);
            return true;
        }

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
        if (_kind == ConvexShapeKind.CircleSlab)
        {
            throw new InvalidOperationException(
                "Circle slabs are sweep sources and cannot be target shapes.");
        }
        if (_kind == ConvexShapeKind.Sphere)
        {
            throw new InvalidOperationException(
                "Sphere query sources cannot be target shapes.");
        }

        if (_kind == ConvexShapeKind.Triangle)
        {
            normal = _triangleOwner!.Mesh.GetFaceNormalWorld(_triangleIndex);
            return true;
        }

        return _collider!.TryGetPlanarSurfaceNormal(point, out normal);
    }

    private Vector3d GetCircleSlabSupportLocalPoint(Vector3d direction)
    {
        Vector3d radial = new(direction.X, Fixed64.Zero, direction.Z);
        Fixed64 radialMagnitude = radial.Magnitude;
        Vector3d radialSupport = radialMagnitude > Fixed64.Epsilon
            ? radial / radialMagnitude * _radius
            : Vector3d.Right * _radius;
        Fixed64 y = direction.Y >= Fixed64.Zero ? _halfHeight : -_halfHeight;
        return new Vector3d(radialSupport.X, y, radialSupport.Z);
    }

    private Vector3d GetSphereSupportLocalPoint(Vector3d direction)
    {
        Vector3d normal = direction.IsZero
            ? Vector3d.Right
            : direction.Normalized;
        return normal * _radius;
    }

    private Vector3d GetTriangleSupportLocalPoint(Vector3d direction)
    {
        Vector3d localDirection =
            _triangleOwner!.Mesh.Rotation.Inverse().Rotate(direction);
        Vector3d best = _triangleA;
        if (Vector3d.CompareProjection(_triangleB, best, localDirection) > 0)
            best = _triangleB;

        if (Vector3d.CompareProjection(_triangleC, best, localDirection) > 0)
            best = _triangleC;

        return best;
    }
}
