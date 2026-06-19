using FixedMathSharp;
using Gravitas.Queries;
using SwiftCollections;

namespace Gravitas.Colliders;

public class LSCapsuleCollider : LSCollider
{
    public LSCapsuleCollider() { }

    public LSCapsuleCollider(ColliderShapeDefinition definition)
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.Capsule);
        Radius = definition.Radius;
        Size = definition.Size;
    }

    public override ColliderType Shape => ColliderType.Capsule;
    public override int Priority => ColliderSettings.GetPriority(Shape);

    public override Fixed64 ScaledRadius => _radius * FixedMath.Max(LocalScale.X, LocalScale.Z);

    // The local top and bottom center points that define the hemispheres of the capsule
    public Vector3d HemisphereCenterTop { get; private set; }

    public Vector3d HemisphereCenterBottom { get; private set; }

    public Fixed64 CylinderHeight { get; private set; }

    public Vector3d LineSegmentStart { get; private set; }

    public Vector3d LineSegmentEnd { get; private set; }

    public Vector3d LineDirection => (LineSegmentEnd - LineSegmentStart).Normalized;

    protected override void OnInitialize()
    {
        Fixed64 diameter = _radius * 2;
        _size = new Vector3d(diameter, _size.Y, diameter);
        base.OnInitialize();
    }

    protected override void OnRadiusChanged()
    {
        Fixed64 diameter = _radius * 2;
        _size = new Vector3d(diameter, _size.Y, diameter);
    }

    protected override Vector3d NormalizeSize(Vector3d value) =>
        new(_radius * 2, value.Y, _radius * 2);

    protected override void BuildShape()
    {
        Fixed64 halfCylinderHeight = FixedMath.Max(Fixed64.Zero, (ScaledSize.Y * Fixed64.Half) - ScaledRadius);
        HemisphereCenterBottom = new Vector3d(Fixed64.Zero, -halfCylinderHeight, Fixed64.Zero);
        HemisphereCenterTop = new Vector3d(Fixed64.Zero, halfCylinderHeight, Fixed64.Zero);
        CylinderHeight = halfCylinderHeight * 2;

        // Area calculation: A = 2πrh + 2πr^2
        Area = 2 * Fixed64.Pi * ScaledRadius * CylinderHeight + 2 * Fixed64.Pi * ScaledRadiusSqr;
        UpdateLineSegment();
    }

    private void UpdateLineSegment()
    {
        // Convert local start and end positions to world positions and add capsule position
        LineSegmentStart = Center + (Rotation * HemisphereCenterBottom);
        LineSegmentEnd = Center + (Rotation * HemisphereCenterTop);
    }

    // The inertia tensor for a homogeneous solid cylinder of radius r and height h,
    // about an axis through its center (which is the case for the cylindrical part of the capsule), is given by:
    //     I_cylinder = (1/12) * mass * (3*r*r + h*h)
    // And the inertia tensor for a solid sphere of radius r (which applies to the hemispherical ends of the capsule) is:
    //     I_sphere = (2/5) * mass * r*r
    // Therefore, for the capsule (assuming it's made of the same material throughout),
    // we'll add the inertia of the two hemispheres and the cylinder to get:
    //     I_capsule = I_cylinder + 2 * I_sphere
    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localCenterOfMassOffset)
    {
        if (CylinderHeight <= Fixed64.Epsilon)
        {
            Fixed64 sphereDiagonal = Fixed64.FromFraction(2, 5) * mass * ScaledRadiusSqr;
            Fixed3x3 sphereTensor = new(
                sphereDiagonal, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, sphereDiagonal, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, sphereDiagonal
            );
            return ShiftInertiaTensorFromLocalCenterOfMass(sphereTensor, mass, localCenterOfMassOffset);
        }

        // Masses of the cylinder and spheres (proportional to their volumes)
        Fixed64 cylinderVolume = Fixed64.Pi * ScaledRadiusSqr * CylinderHeight;
        Fixed64 sphereVolume = Fixed64.FromFraction(4, 3) * Fixed64.Pi * ScaledRadiusSqr * ScaledRadius;
        Fixed64 totalVolume = cylinderVolume + sphereVolume;
        Fixed64 cylinderMass = mass * (cylinderVolume / totalVolume);
        Fixed64 sphereMass = mass - cylinderMass;

        // Distance from the center of the hemisphere to the center of the capsule
        Fixed64 d = CylinderHeight / 2;

        // Calculating the inertia tensors for the cylinder and the spheres
        Fixed64 cylinderInertiaY = Fixed64.FromFraction(1, 2) * cylinderMass * ScaledRadiusSqr;
        Fixed64 cylinderInertiaXZ = Fixed64.FromFraction(1, 12) * cylinderMass * ((3 * ScaledRadiusSqr) + (CylinderHeight * CylinderHeight));
        // Multiply by 2 because there are two hemispheres and apply parallel axis theorem
        Fixed64 sphereInertiaXZ = (Fixed64.FromFraction(2, 5) * sphereMass * ScaledRadiusSqr) + sphereMass * d * d;
        Fixed64 sphereInertiaY = Fixed64.FromFraction(2, 5) * sphereMass * ScaledRadiusSqr;

        // The total inertia tensor for the capsule
        Fixed64 totalInertia_xz = cylinderInertiaXZ + sphereInertiaXZ;
        Fixed64 totalInertia_y = cylinderInertiaY + sphereInertiaY;

        Fixed3x3 tensor = new(
            totalInertia_xz, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, totalInertia_y, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, totalInertia_xz
        );
        return ShiftInertiaTensorFromLocalCenterOfMass(tensor, mass, localCenterOfMassOffset);
    }

    // If the capsule is moving in the direction of its main axis,
    // the frontal area would be a circle (the end cap of the capsule).
    // Therefore, the frontal area would be πr^2, where r is the radius of the capsule.
    // If it's moving perpendicular to its main axis,
    // then the frontal area would be a rectangle with a semicircle on either end,
    // which would be (2r)*h + πr^2, where h is the height of the cylindrical part of the capsule.
    public override Fixed64 GetFrontalArea(Vector3d direction) =>
        2 * ScaledRadius * CylinderHeight + Fixed64.Pi * ScaledRadiusSqr;

    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        Vector3d directionToStart = other - LineSegmentStart;
        Fixed64 distanceToStart = directionToStart.Magnitude;
        Vector3d directionToEnd = other - LineSegmentEnd;
        Fixed64 distanceToEnd = directionToEnd.Magnitude;

        // If the point is within the bottom hemisphere
        if (distanceToStart < ScaledRadius && distanceToStart > Fixed64.Zero)
        {
            directionToStart /= distanceToStart; // normalize
            directionToStart *= ScaledRadius;
            return LineSegmentStart + directionToStart;
        }

        // If the point is within the top hemisphere
        if (distanceToEnd < ScaledRadius && distanceToEnd > Fixed64.Zero)
        {
            directionToEnd /= distanceToEnd; // normalize
            directionToEnd *= ScaledRadius;
            return LineSegmentEnd + directionToEnd;
        }

        // If the point is along the length of the cylinder
        Vector3d lineDirection = LineSegmentEnd - LineSegmentStart;
        Fixed64 lineLength = lineDirection.Magnitude;

        if (lineLength > Fixed64.Epsilon)
            lineDirection /= lineLength;  // normalize

        // Compute the t value for the line equation
        Fixed64 t = Vector3d.Dot(other - LineSegmentStart, lineDirection);

        // Clamp to the segment extents
        t = FixedMath.Max(Fixed64.Zero, FixedMath.Min(lineLength, t));

        // Calculate the projection of 'other' onto the line to find the closest point on the line segment
        Vector3d projection = LineSegmentStart + t * lineDirection;

        // Now find the direction from the closest point on the line segment to 'other'
        Vector3d direction = other - projection;

        Fixed64 distance = direction.Magnitude;

        // If the point is inside the capsule, return the point itself
        if (distance <= ScaledRadius)
            return other;

        if (distance > Fixed64.Zero)
        {
            direction /= distance; // normalize
            direction *= ScaledRadius;
        }

        return projection + direction;
    }

    public override Vector3d GetNormalAtPoint(Vector3d point)
    {
        Vector3d rotatedPoint = Rotation.Inverse() * (point - Center);

        // If the point is on the top hemisphere
        if (rotatedPoint.Y > HemisphereCenterTop.Y)
        {
            Vector3d topDirection = rotatedPoint - HemisphereCenterTop;
            Fixed64 topDistance = topDirection.Magnitude;
            return topDistance > Fixed64.Zero
                ? Rotation * (topDirection / topDistance)
                : Rotation * Vector3d.Up;
        }

        // If the point is on the bottom hemisphere
        if (rotatedPoint.Y < HemisphereCenterBottom.Y)
        {
            Vector3d bottomDirection = rotatedPoint - HemisphereCenterBottom;
            Fixed64 bottomDistance = bottomDirection.Magnitude;
            return bottomDistance > Fixed64.Zero
                ? Rotation * (bottomDirection / bottomDistance)
                : Rotation * -Vector3d.Up;
        }

        // If the point is along the length of the cylinder
        Vector3d direction = new(rotatedPoint.X, Fixed64.Zero, rotatedPoint.Z);
        Fixed64 distance = direction.Magnitude;
        return distance > Fixed64.Zero
            ? Rotation * (direction / distance)
            : Rotation * Vector3d.Right;
    }

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        worker.CheckCapsuleOverlaps(this, ref outputIntersectionPoints);
}
