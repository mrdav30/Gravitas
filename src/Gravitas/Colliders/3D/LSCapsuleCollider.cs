//=======================================================================
// LSCapsuleCollider.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

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
        Material = definition.Material;
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

    protected internal override Fixed64 CalculateMassPropertyWeight() =>
        Fixed64.Pi * ScaledRadiusSqr * CylinderHeight
        + Fixed64.FromFraction(4, 3) * Fixed64.Pi * ScaledRadiusSqr * ScaledRadius;

    private void UpdateLineSegment()
    {
        // Convert local start and end positions to world positions and add capsule position
        LineSegmentStart = Center + (Rotation * HemisphereCenterBottom);
        LineSegmentEnd = Center + (Rotation * HemisphereCenterTop);
    }

    // The capsule is split into a cylinder and a pair of solid hemispheres with masses
    // proportional to their volumes. Each hemisphere's centroid lies 3r/8 outward from
    // its sphere center, which contributes the 3dr/4 cross term to transverse cap inertia.
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
        if (totalVolume <= Fixed64.Zero)
        {
            // Fixed-point scaling can quantize a positive radius and both volumes to zero.
            // The remaining shape is the zero-radius limit: a thin rod along local Y.
            Fixed64 rodInertiaXZ = Fixed64.FromFraction(1, 12) * mass * CylinderHeight * CylinderHeight;
            Fixed3x3 rodTensor = new(
                rodInertiaXZ, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, rodInertiaXZ
            );
            return ShiftInertiaTensorFromLocalCenterOfMass(rodTensor, mass, localCenterOfMassOffset);
        }

        Fixed64 cylinderMass = mass * (cylinderVolume / totalVolume);
        Fixed64 sphereMass = mass - cylinderMass;

        // Distance from the center of the hemisphere to the center of the capsule
        Fixed64 d = CylinderHeight / 2;

        // Calculating the inertia tensors for the cylinder and the spheres
        Fixed64 cylinderInertiaY = Fixed64.FromFraction(1, 2) * cylinderMass * ScaledRadiusSqr;
        Fixed64 cylinderInertiaXZ = Fixed64.FromFraction(1, 12) * cylinderMass * ((3 * ScaledRadiusSqr) + (CylinderHeight * CylinderHeight));
        Fixed64 sphereInertiaXZ = sphereMass
            * (Fixed64.FromFraction(2, 5) * ScaledRadiusSqr
                + d * d
                + Fixed64.FromFraction(3, 4) * d * ScaledRadius);
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
    public override Fixed64 GetFrontalArea(Vector3d direction)
    {
        Fixed64 directionMagnitude = direction.Magnitude;
        if (directionMagnitude <= Fixed64.Epsilon)
            return Area;

        Vector3d normalizedDirection = direction / directionMagnitude;
        Vector3d axis = Rotation * Vector3d.Up;
        Fixed64 axial = Vector3d.Dot(normalizedDirection, axis).Abs();
        Fixed64 radialFactorSqr = Fixed64.One - axial * axial;
        Fixed64 radialFactor = radialFactorSqr <= Fixed64.Zero
            ? Fixed64.Zero
            : FixedMath.Sqrt(radialFactorSqr);

        Fixed64 capArea = Fixed64.Pi * ScaledRadiusSqr;
        Fixed64 sideProfile = 2 * ScaledRadius * CylinderHeight;
        return capArea + radialFactor * sideProfile;
    }

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

        // The preceding inside test proves distance > ScaledRadius >= 0 here.
        direction /= distance;
        direction *= ScaledRadius;

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
