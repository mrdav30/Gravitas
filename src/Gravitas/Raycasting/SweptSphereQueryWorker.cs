using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Raycasting;

/// <summary>
/// Performs deterministic swept-sphere checks for one prepared segment.
/// </summary>
public sealed class SweptSphereQueryWorker
{
    private Vector3d _start;
    private Vector3d _end;
    private Vector3d _direction;
    private Fixed64 _length;
    private Fixed64 _lengthSqr;
    private Fixed64 _radius;

    /// <summary>
    /// Prepares this worker for a swept sphere from <paramref name="start"/> to <paramref name="end"/>.
    /// </summary>
    public void Prepare(Vector3d start, Vector3d end, Fixed64 radius)
    {
        _start = start;
        _end = end;
        _radius = radius;

        Vector3d segment = end - start;
        _lengthSqr = segment.SqrMagnitude;
        _length = _lengthSqr <= Fixed64.Epsilon ? Fixed64.Zero : segment.Magnitude;
        _direction = _length <= Fixed64.Epsilon ? Vector3d.Zero : segment / _length;
    }

    /// <summary>
    /// Checks a collider against the prepared swept sphere.
    /// </summary>
    public bool TrySweep(
        LSCollider collider,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        if (_length <= Fixed64.Epsilon)
            return false;

        return collider switch
        {
            LSSphereCollider sphere => TrySweepSphere(sphere.Center, sphere.ScaledRadius + _radius, out sphereCenterAtImpact, out impactDistance),
            LSCapsuleCollider capsule => TrySweepCapsule(capsule, out sphereCenterAtImpact, out impactDistance),
            LSCuboidCollider cuboid => TrySweepCuboid(cuboid, out sphereCenterAtImpact, out impactDistance),
            LSCylinderCollider cylinder => TrySweepCylinder(cylinder, out sphereCenterAtImpact, out impactDistance),
            _ => false
        };
    }

    private bool TrySweepSphere(
        Vector3d center,
        Fixed64 radius,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        Fixed64 radiusSqr = radius * radius;
        Vector3d startToCenter = _start - center;
        if (startToCenter.SqrMagnitude <= radiusSqr)
        {
            sphereCenterAtImpact = _start;
            return true;
        }

        Fixed64 b = Vector3d.Dot(startToCenter, _direction);
        Fixed64 c = startToCenter.SqrMagnitude - radiusSqr;
        Fixed64 discriminant = b * b - c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 root = FixedMath.Sqrt(discriminant);
        Fixed64 distance = -b - root;
        if (distance < Fixed64.Zero)
            distance = -b + root;

        if (distance < Fixed64.Zero || distance > _length)
            return false;

        impactDistance = distance;
        sphereCenterAtImpact = _start + _direction * distance;
        return true;
    }

    private bool TrySweepCapsule(
        LSCapsuleCollider capsule,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.MAX_VALUE;

        Fixed64 radius = capsule.ScaledRadius + _radius;
        bool found = false;

        found |= TryKeepEarlierSweep(
            TrySweepSphere(capsule.LineSegmentStart, radius, out Vector3d startHit, out Fixed64 startDistance),
            startHit,
            startDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepSphere(capsule.LineSegmentEnd, radius, out Vector3d endHit, out Fixed64 endDistance),
            endHit,
            endDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);

        Fixed64 halfHeight = capsule.CylinderHeight * Fixed64.Half;
        if (halfHeight > Fixed64.Epsilon)
        {
            found |= TryKeepEarlierSweep(
                TrySweepFiniteCylinder(
                    capsule.Center,
                    capsule.Rotation,
                    radius,
                    halfHeight,
                    includeCaps: false,
                    out Vector3d cylinderHit,
                    out Fixed64 cylinderDistance),
                cylinderHit,
                cylinderDistance,
                ref sphereCenterAtImpact,
                ref impactDistance);
        }

        return found;
    }

    private bool TrySweepCylinder(
        LSCylinderCollider cylinder,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        return TrySweepFiniteCylinder(
            cylinder.Center,
            cylinder.Rotation,
            cylinder.ScaledRadius + _radius,
            cylinder.HalfHeight + _radius,
            includeCaps: true,
            out sphereCenterAtImpact,
            out impactDistance);
    }

    private bool TrySweepFiniteCylinder(
        Vector3d center,
        FixedQuaternion rotation,
        Fixed64 radius,
        Fixed64 halfHeight,
        bool includeCaps,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.MAX_VALUE;

        FixedQuaternion inverseRotation = rotation.Inverse();
        Vector3d localStart = (_start - center) * inverseRotation;
        Vector3d localDirection = _direction * inverseRotation;
        Fixed64 radiusSqr = radius * radius;

        if (IsPointInsideFiniteCylinder(localStart, radiusSqr, halfHeight))
        {
            sphereCenterAtImpact = _start;
            impactDistance = Fixed64.Zero;
            return true;
        }

        bool found = false;
        found |= TryKeepEarlierSweep(
            TrySweepCylinderSide(center, rotation, localStart, localDirection, radiusSqr, halfHeight, out Vector3d sideHit, out Fixed64 sideDistance),
            sideHit,
            sideDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);

        if (includeCaps)
        {
            found |= TryKeepEarlierSweep(
                TrySweepCylinderCap(center, rotation, localStart, localDirection, radiusSqr, halfHeight, out Vector3d topHit, out Fixed64 topDistance),
                topHit,
                topDistance,
                ref sphereCenterAtImpact,
                ref impactDistance);
            found |= TryKeepEarlierSweep(
                TrySweepCylinderCap(center, rotation, localStart, localDirection, radiusSqr, -halfHeight, out Vector3d bottomHit, out Fixed64 bottomDistance),
                bottomHit,
                bottomDistance,
                ref sphereCenterAtImpact,
                ref impactDistance);
        }

        return found;
    }

    private bool TrySweepCylinderSide(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localStart,
        Vector3d localDirection,
        Fixed64 radiusSqr,
        Fixed64 halfHeight,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.MAX_VALUE;

        Fixed64 a = localDirection.x * localDirection.x + localDirection.z * localDirection.z;
        if (a <= Fixed64.Epsilon)
            return false;

        Fixed64 b = 2 * (localStart.x * localDirection.x + localStart.z * localDirection.z);
        Fixed64 c = localStart.x * localStart.x + localStart.z * localStart.z - radiusSqr;
        Fixed64 discriminant = b * b - 4 * a * c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 root = FixedMath.Sqrt(discriminant);
        Fixed64 denominator = 2 * a;
        Fixed64 first = (-b - root) / denominator;
        Fixed64 second = (-b + root) / denominator;

        bool found = false;
        found |= TryKeepEarlierSweep(
            TryBuildCylinderLocalHit(center, rotation, localStart, localDirection, first, halfHeight, out Vector3d firstHit, out Fixed64 firstDistance),
            firstHit,
            firstDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TryBuildCylinderLocalHit(center, rotation, localStart, localDirection, second, halfHeight, out Vector3d secondHit, out Fixed64 secondDistance),
            secondHit,
            secondDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);

        return found;
    }

    private bool TrySweepCylinderCap(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localStart,
        Vector3d localDirection,
        Fixed64 radiusSqr,
        Fixed64 capY,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        if (localDirection.y.Abs() <= Fixed64.Epsilon)
            return false;

        Fixed64 distance = (capY - localStart.y) / localDirection.y;
        if (distance < Fixed64.Zero || distance > _length)
            return false;

        Vector3d localPoint = localStart + localDirection * distance;
        Fixed64 radialSqr = localPoint.x * localPoint.x + localPoint.z * localPoint.z;
        if (radialSqr > radiusSqr + Fixed64.Epsilon)
            return false;

        sphereCenterAtImpact = center + rotation * localPoint;
        impactDistance = distance;
        return true;
    }

    private bool TrySweepCuboid(
        LSCuboidCollider cuboid,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        FixedQuaternion inverseRotation = cuboid.Rotation.Inverse();
        Vector3d localStart = (_start - cuboid.Center) * inverseRotation;
        Vector3d localDirection = _direction * inverseRotation;
        Vector3d halfExtents = cuboid.ScaledSize * Fixed64.Half;

        Vector3d min = -halfExtents - Vector3d.One * _radius;
        Vector3d max = halfExtents + Vector3d.One * _radius;
        return TrySweepLocalBox(
            cuboid.Center,
            cuboid.Rotation,
            localStart,
            localDirection,
            min,
            max,
            out sphereCenterAtImpact,
            out impactDistance);
    }

    private bool TrySweepLocalBox(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localStart,
        Vector3d localDirection,
        Vector3d min,
        Vector3d max,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        if (IsPointInsideBox(localStart, min, max))
        {
            sphereCenterAtImpact = _start;
            return true;
        }

        Fixed64 entry = Fixed64.Zero;
        Fixed64 exit = _length;
        if (!ClipSegmentAxis(localStart.x, localDirection.x, min.x, max.x, ref entry, ref exit)
            || !ClipSegmentAxis(localStart.y, localDirection.y, min.y, max.y, ref entry, ref exit)
            || !ClipSegmentAxis(localStart.z, localDirection.z, min.z, max.z, ref entry, ref exit))
        {
            return false;
        }

        impactDistance = entry;
        sphereCenterAtImpact = center + rotation * (localStart + localDirection * entry);
        return true;
    }

    private bool TryBuildCylinderLocalHit(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localStart,
        Vector3d localDirection,
        Fixed64 distance,
        Fixed64 halfHeight,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        if (distance < Fixed64.Zero || distance > _length)
            return false;

        Vector3d localPoint = localStart + localDirection * distance;
        if (localPoint.y < -halfHeight || localPoint.y > halfHeight)
            return false;

        sphereCenterAtImpact = center + rotation * localPoint;
        impactDistance = distance;
        return true;
    }

    private static bool TryKeepEarlierSweep(
        bool candidateFound,
        Vector3d candidateCenter,
        Fixed64 candidateDistance,
        ref Vector3d sphereCenterAtImpact,
        ref Fixed64 impactDistance)
    {
        if (!candidateFound || candidateDistance >= impactDistance)
            return false;

        sphereCenterAtImpact = candidateCenter;
        impactDistance = candidateDistance;
        return true;
    }

    private static bool IsPointInsideFiniteCylinder(Vector3d localPoint, Fixed64 radiusSqr, Fixed64 halfHeight) =>
        localPoint.y >= -halfHeight
        && localPoint.y <= halfHeight
        && localPoint.x * localPoint.x + localPoint.z * localPoint.z <= radiusSqr;

    private static bool IsPointInsideBox(Vector3d localPoint, Vector3d min, Vector3d max) =>
        localPoint.x >= min.x && localPoint.x <= max.x
        && localPoint.y >= min.y && localPoint.y <= max.y
        && localPoint.z >= min.z && localPoint.z <= max.z;

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

        Fixed64 first = (min - position) / direction;
        Fixed64 second = (max - position) / direction;
        if (first > second)
            (first, second) = (second, first);

        if (first > entry)
            entry = first;

        if (second < exit)
            exit = second;

        return entry <= exit;
    }
}
