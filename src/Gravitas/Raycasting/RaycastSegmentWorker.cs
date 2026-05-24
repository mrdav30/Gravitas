using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas.Raycasting;

/// <summary>
/// Stores segment data used by one context-owned raycast service while checking collider overlaps.
/// </summary>
public sealed class RaycastSegmentWorker
{
    private Vector3d _cachedOrigin;
    private Vector3d _cachedEnd;
    private Vector3d _segmentDirection;
    private Fixed64 _segmentLength;
    private Fixed64 _segmentLengthSqr;
    private bool _calculateIntersections;

    /// <summary>
    /// Prepares this worker for overlap checks against the line segment between two points.
    /// </summary>
    public void PrepareSegmentCheck(Vector3d p1, Vector3d p2, bool calculateIntersectionPoints = true)
    {
        _cachedOrigin = p1;
        _cachedEnd = p2;

        Vector3d segment = p2 - p1;
        _segmentLengthSqr = segment.SqrMagnitude;
        _segmentLength = _segmentLengthSqr == Fixed64.Zero ? Fixed64.Zero : segment.Magnitude;
        _segmentDirection = _segmentLength == Fixed64.Zero ? Vector3d.Zero : segment / _segmentLength;
        _calculateIntersections = calculateIntersectionPoints;
    }

    public bool CheckSphereOverlaps(LSSphereCollider sphereCollider, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        CheckSphereOverlaps(sphereCollider.Center, sphereCollider.ScaledRadiusSqr, ref outputIntersectionPoints);

    /// <summary>
    /// Checks whether a sphere overlaps this worker's prepared ray segment.
    /// </summary>
    public bool CheckSphereOverlaps(
        Vector3d position,
        Fixed64 sqrRadius,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (_segmentLengthSqr == Fixed64.Zero)
            return CheckPointInsideSphere(position, sqrRadius, ref outputIntersectionPoints);

        Vector3d originToCenter = position - _cachedOrigin;
        Fixed64 closestParameter = Vector3d.Dot(originToCenter, _segmentDirection);
        closestParameter = FixedMath.Clamp(closestParameter, Fixed64.Zero, _segmentLength);
        Vector3d closestPoint = _cachedOrigin + _segmentDirection * closestParameter;
        if ((closestPoint - position).SqrMagnitude > sqrRadius)
            return false;

        if (!_calculateIntersections)
            return true;

        Vector3d originFromCenter = _cachedOrigin - position;
        Fixed64 c = originFromCenter.SqrMagnitude - sqrRadius;
        if (c <= Fixed64.Zero)
        {
            outputIntersectionPoints.Add(_cachedOrigin);
            return true;
        }

        Fixed64 b = Vector3d.Dot(originFromCenter, _segmentDirection);
        Fixed64 discriminant = b * b - c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 root = FixedMath.Sqrt(discriminant);
        AddIntersectionPointIfOnSegment(-b - root, ref outputIntersectionPoints);
        if (root != Fixed64.Zero)
            AddIntersectionPointIfOnSegment(-b + root, ref outputIntersectionPoints);

        return outputIntersectionPoints.Count > 0;
    }

    public bool CheckCapsuleOverlaps(LSCapsuleCollider capsuleCollider, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        bool intersects = CheckCapsuleCylinderOverlaps(capsuleCollider, ref outputIntersectionPoints);

        if (!intersects)
        {
            intersects = CheckSphereOverlaps(capsuleCollider.LineSegmentEnd, capsuleCollider.ScaledRadiusSqr, ref outputIntersectionPoints)
                         || CheckSphereOverlaps(capsuleCollider.LineSegmentStart, capsuleCollider.ScaledRadiusSqr, ref outputIntersectionPoints);
        }

        return intersects;
    }

    public bool CheckCylinderOverlaps(LSCylinderCollider cylinderCollider, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        CheckFiniteCylinderOverlaps(
            cylinderCollider.Center,
            cylinderCollider.Rotation,
            cylinderCollider.ScaledRadius,
            cylinderCollider.ScaledRadiusSqr,
            cylinderCollider.HalfHeight,
            includeCaps: true,
            ref outputIntersectionPoints);

    private bool CheckCapsuleCylinderOverlaps(LSCapsuleCollider capsuleCollider, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        CheckFiniteCylinderOverlaps(
            capsuleCollider.Center,
            capsuleCollider.Rotation,
            capsuleCollider.ScaledRadius,
            capsuleCollider.ScaledRadiusSqr,
            capsuleCollider.CylinderHeight * Fixed64.Half,
            includeCaps: false,
            ref outputIntersectionPoints);

    private bool CheckFiniteCylinderOverlaps(
        Vector3d center,
        FixedQuaternion rotation,
        Fixed64 radius,
        Fixed64 radiusSqr,
        Fixed64 halfHeight,
        bool includeCaps,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        FixedQuaternion inverseRotation = rotation.Inverse();
        Vector3d localOrigin = (_cachedOrigin - center) * inverseRotation;

        if (_segmentLengthSqr == Fixed64.Zero)
            return CheckPointInsideFiniteCylinder(center, rotation, localOrigin, radiusSqr, halfHeight, ref outputIntersectionPoints);

        Vector3d localDirection = _segmentDirection * inverseRotation;
        if (PointInsideFiniteCylinder(localOrigin, radiusSqr, halfHeight))
        {
            if (_calculateIntersections)
                outputIntersectionPoints.Add(_cachedOrigin);
            return true;
        }

        bool intersects = CheckFiniteCylinderSide(
            center,
            rotation,
            localOrigin,
            localDirection,
            radiusSqr,
            halfHeight,
            ref outputIntersectionPoints);

        if (includeCaps)
        {
            intersects |= CheckFiniteCylinderCap(center, rotation, localOrigin, localDirection, radiusSqr, halfHeight, ref outputIntersectionPoints);
            intersects |= CheckFiniteCylinderCap(center, rotation, localOrigin, localDirection, radiusSqr, -halfHeight, ref outputIntersectionPoints);
        }

        return intersects;
    }

    private bool CheckFiniteCylinderSide(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localOrigin,
        Vector3d localDirection,
        Fixed64 radiusSqr,
        Fixed64 halfHeight,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        Fixed64 a = localDirection.x * localDirection.x + localDirection.z * localDirection.z;
        if (a <= Fixed64.Epsilon)
            return false;

        Fixed64 b = 2 * (localOrigin.x * localDirection.x + localOrigin.z * localDirection.z);
        Fixed64 c = localOrigin.x * localOrigin.x + localOrigin.z * localOrigin.z - radiusSqr;
        Fixed64 discriminant = b * b - 4 * a * c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 root = FixedMath.Sqrt(discriminant);
        Fixed64 denominator = 2 * a;
        Fixed64 t1 = (-b - root) / denominator;
        Fixed64 t2 = (-b + root) / denominator;

        bool intersects = TryAddFiniteCylinderPoint(center, rotation, localOrigin, localDirection, t1, halfHeight, ref outputIntersectionPoints);
        if (t2 != t1)
            intersects |= TryAddFiniteCylinderPoint(center, rotation, localOrigin, localDirection, t2, halfHeight, ref outputIntersectionPoints);

        return intersects;
    }

    private bool CheckFiniteCylinderCap(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localOrigin,
        Vector3d localDirection,
        Fixed64 radiusSqr,
        Fixed64 capY,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (localDirection.y.Abs() <= Fixed64.Epsilon)
            return false;

        Fixed64 distance = (capY - localOrigin.y) / localDirection.y;
        if (distance < Fixed64.Zero || distance > _segmentLength)
            return false;

        Vector3d localPoint = localOrigin + localDirection * distance;
        Fixed64 radialSqr = localPoint.x * localPoint.x + localPoint.z * localPoint.z;
        if (radialSqr > radiusSqr + Fixed64.Epsilon)
            return false;

        AddLocalIntersectionPoint(center, rotation, localPoint, ref outputIntersectionPoints);
        return true;
    }

    public bool CheckAABBoxOverlaps(LSCuboidCollider aabox, ref SwiftList<Vector3d> outputIntersectionPoints) =>
         CheckAABBoxOverlaps(aabox.BoundsMin, aabox.BoundsMax, ref outputIntersectionPoints);

    /// <summary>
    /// Checks whether an axis-aligned bounding box overlaps this worker's prepared ray segment.
    /// </summary>
    public bool CheckAABBoxOverlaps(Vector3d min, Vector3d max, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (_segmentLengthSqr == Fixed64.Zero)
            return CheckPointInsideBox(min, max, ref outputIntersectionPoints);

        Fixed64 entry = Fixed64.Zero;
        Fixed64 exit = _segmentLength;

        if (!ClipSegmentAxis(_cachedOrigin.x, _segmentDirection.x, min.x, max.x, ref entry, ref exit)
            || !ClipSegmentAxis(_cachedOrigin.y, _segmentDirection.y, min.y, max.y, ref entry, ref exit)
            || !ClipSegmentAxis(_cachedOrigin.z, _segmentDirection.z, min.z, max.z, ref entry, ref exit))
        {
            return false;
        }

        if (_calculateIntersections)
        {
            outputIntersectionPoints.Add(_cachedOrigin + _segmentDirection * entry);
            if (exit != entry)
                outputIntersectionPoints.Add(_cachedOrigin + _segmentDirection * exit);
        }

        return true;
    }

    /// <summary>
    /// Checks whether an oriented bounding box overlaps this worker's prepared ray segment.
    /// </summary>
    public bool CheckOBBoxOverlaps(LSCuboidCollider oobox, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!CheckAABBoxOverlaps(oobox.BoundsMin, oobox.BoundsMax, ref outputIntersectionPoints))
            return false;

        for (int i = 0; i < outputIntersectionPoints.Count; i++)
        {
            Vector3d worldSpaceIntersection = outputIntersectionPoints[i].Rotate(oobox.Position, oobox.Rotation);
            outputIntersectionPoints[i] = worldSpaceIntersection;
        }

        return true;
    }

    private bool TryAddFiniteCylinderPoint(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localOrigin,
        Vector3d localDirection,
        Fixed64 distance,
        Fixed64 halfHeight,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (distance < Fixed64.Zero || distance > _segmentLength)
            return false;

        Vector3d localPoint = localOrigin + localDirection * distance;
        if (localPoint.y < -halfHeight || localPoint.y > halfHeight)
            return false;

        AddLocalIntersectionPoint(center, rotation, localPoint, ref outputIntersectionPoints);
        return true;
    }

    private bool CheckPointInsideFiniteCylinder(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localPoint,
        Fixed64 radiusSqr,
        Fixed64 halfHeight,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!PointInsideFiniteCylinder(localPoint, radiusSqr, halfHeight))
            return false;

        AddLocalIntersectionPoint(center, rotation, localPoint, ref outputIntersectionPoints);
        return true;
    }

    private static bool PointInsideFiniteCylinder(Vector3d localPoint, Fixed64 radiusSqr, Fixed64 halfHeight) =>
        localPoint.y >= -halfHeight
        && localPoint.y <= halfHeight
        && localPoint.x * localPoint.x + localPoint.z * localPoint.z <= radiusSqr;

    private void AddLocalIntersectionPoint(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localPoint,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!_calculateIntersections)
            return;

        Vector3d worldPoint = center + rotation * localPoint;
        for (int i = 0; i < outputIntersectionPoints.Count; i++)
        {
            if (Vector3d.SqrDistance(outputIntersectionPoints[i], worldPoint) <= Fixed64.Epsilon)
                return;
        }

        outputIntersectionPoints.Add(worldPoint);
    }

    private bool CheckPointInsideSphere(
        Vector3d position,
        Fixed64 sqrRadius,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if ((_cachedOrigin - position).SqrMagnitude > sqrRadius)
            return false;

        if (_calculateIntersections)
            outputIntersectionPoints.Add(_cachedOrigin);

        return true;
    }

    private bool CheckPointInsideBox(
        Vector3d min,
        Vector3d max,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (_cachedOrigin.x < min.x || _cachedOrigin.x > max.x
            || _cachedOrigin.y < min.y || _cachedOrigin.y > max.y
            || _cachedOrigin.z < min.z || _cachedOrigin.z > max.z)
        {
            return false;
        }

        if (_calculateIntersections)
            outputIntersectionPoints.Add(_cachedOrigin);

        return true;
    }

    private void AddIntersectionPointIfOnSegment(Fixed64 distance, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (distance < Fixed64.Zero || distance > _segmentLength)
            return;

        outputIntersectionPoints.Add(_cachedOrigin + _segmentDirection * distance);
    }

    private static bool ClipSegmentAxis(
        Fixed64 position,
        Fixed64 direction,
        Fixed64 min,
        Fixed64 max,
        ref Fixed64 entry,
        ref Fixed64 exit)
    {
        if (direction.Abs() <= Fixed64.Epsilon)
            return position >= min && position <= max;

        Fixed64 t1 = (min - position) / direction;
        Fixed64 t2 = (max - position) / direction;

        if (t1 > t2)
            (t1, t2) = (t2, t1);

        if (t1 > entry)
            entry = t1;

        if (t2 < exit)
            exit = t2;

        return entry <= exit;
    }
}
