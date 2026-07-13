//=======================================================================
// LSConeCollider.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Queries;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Represents a finite circular cone whose local Y axis runs from base plane
/// to apex and whose local origin is the cone's bounding center.
/// </summary>
public sealed class LSConeCollider : LSCollider
{
    public LSConeCollider() { }

    public LSConeCollider(ColliderShapeDefinition definition)
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.Cone);
        Material = definition.Material;
        Radius = definition.Radius;
        Size = definition.Size;
    }

    public override ColliderType Shape => ColliderType.Cone;

    public override int Priority => ColliderSettings.GetPriority(Shape);

    public override Fixed64 ScaledRadius => _radius * FixedMath.Max(LocalScale.X, LocalScale.Z);

    /// <summary>
    /// Gets the local base-center point before world rotation and translation.
    /// </summary>
    public Vector3d BaseCenter { get; private set; }

    /// <summary>
    /// Gets the local apex point before world rotation and translation.
    /// </summary>
    public Vector3d Apex { get; private set; }

    /// <summary>
    /// Gets the world-space base-center point.
    /// </summary>
    public Vector3d WorldBaseCenter { get; private set; }

    /// <summary>
    /// Gets the world-space apex point.
    /// </summary>
    public Vector3d WorldApex { get; private set; }

    /// <summary>
    /// Gets the world-space base-to-apex axis.
    /// </summary>
    public Vector3d Axis { get; private set; } = Vector3d.Up;

    public Fixed64 Height { get; private set; }

    public Fixed64 HalfHeight { get; private set; }

    /// <summary>
    /// Gets the solid cone volume used for mass weighting and diagnostics.
    /// </summary>
    public Fixed64 Volume { get; private set; }

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
        Height = ScaledSize.Y;
        HalfHeight = Height * Fixed64.Half;
        BaseCenter = new Vector3d(Fixed64.Zero, -HalfHeight, Fixed64.Zero);
        Apex = new Vector3d(Fixed64.Zero, HalfHeight, Fixed64.Zero);
        WorldBaseCenter = Center + Rotation * BaseCenter;
        WorldApex = Center + Rotation * Apex;
        Axis = (WorldApex - WorldBaseCenter).Normalized;
        Volume = Fixed64.Pi * ScaledRadiusSqr * Height / (Fixed64)3;
        Area = Volume;
    }

    protected override void BuildBoundingBox()
    {
        Fixed64 height = ScaledSize.Y;
        Fixed64 halfHeight = height * Fixed64.Half;
        Vector3d axis = Rotation * Vector3d.Up;
        Vector3d baseCenter = Center + Rotation * new Vector3d(Fixed64.Zero, -halfHeight, Fixed64.Zero);
        Vector3d apex = Center + Rotation * new Vector3d(Fixed64.Zero, halfHeight, Fixed64.Zero);
        ConeGeometry.CreateFiniteConeBounds(apex, baseCenter, axis, ScaledRadius, out Vector3d min, out Vector3d max);
        SetBoundsMinMax(min, max);
    }

    protected internal override Fixed64 CalculateMassPropertyWeight() => Volume;

    public override Vector3d CalculateLocalCenterOfMassOffset()
    {
        Vector3d localCom = new(Fixed64.Zero, -Height * Fixed64.FromFraction(1, 4), Fixed64.Zero);
        return TransformMassPropertyPoint(ScaledOffset + localCom);
    }

    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localCenterOfMassOffset)
    {
        Fixed64 radiusSqr = ScaledRadiusSqr;
        Fixed64 heightSqr = Height * Height;
        Fixed64 inertiaXZ = mass
            * ((Fixed64.FromFraction(3, 20) * radiusSqr)
                + (Fixed64.FromFraction(3, 80) * heightSqr));
        Fixed64 inertiaY = Fixed64.FromFraction(3, 10) * mass * radiusSqr;

        Fixed3x3 tensor = new(
            inertiaXZ, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, inertiaY, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, inertiaXZ);
        return ShiftInertiaTensorFromLocalCenterOfMass(tensor, mass, localCenterOfMassOffset);
    }

    public override Fixed64 GetFrontalArea(Vector3d direction)
    {
        Fixed64 directionMagnitude = direction.Magnitude;
        if (directionMagnitude <= Fixed64.Epsilon)
            return Area;

        Vector3d normalizedDirection = direction / directionMagnitude;
        Fixed64 axial = Vector3d.Dot(normalizedDirection, Axis).Abs();
        Fixed64 radialFactorSqr = Fixed64.One - axial * axial;
        Fixed64 radialFactor = radialFactorSqr <= Fixed64.Zero ? Fixed64.Zero : FixedMath.Sqrt(radialFactorSqr);

        Fixed64 baseArea = Fixed64.Pi * ScaledRadiusSqr;
        Fixed64 triangularProfile = ScaledRadius * Height;
        return axial * baseArea + radialFactor * triangularProfile;
    }

    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        Vector3d local = Rotation.Inverse() * (other - Center);
        Fixed64 radialDistance = FixedMath.Sqrt(local.X * local.X + local.Z * local.Z);
        Vector3d radialDirection = radialDistance > Fixed64.Epsilon
            ? new Vector3d(local.X / radialDistance, Fixed64.Zero, local.Z / radialDistance)
            : Vector3d.Right;

        ProjectPointOntoConeSide(radialDistance, local.Y, out Fixed64 sideRho, out Fixed64 sideY);
        ProjectPointOntoBase(radialDistance, local.Y, out Fixed64 baseRho, out Fixed64 baseY);

        Fixed64 sideDistanceSqr = Square(radialDistance - sideRho) + Square(local.Y - sideY);
        Fixed64 baseDistanceSqr = Square(radialDistance - baseRho) + Square(local.Y - baseY);

        Fixed64 rho = sideDistanceSqr <= baseDistanceSqr ? sideRho : baseRho;
        Fixed64 y = sideDistanceSqr <= baseDistanceSqr ? sideY : baseY;
        Vector3d localSurface = new(radialDirection.X * rho, y, radialDirection.Z * rho);
        return Center + Rotation * localSurface;
    }

    public override Vector3d GetNormalAtPoint(Vector3d point)
    {
        Vector3d local = Rotation.Inverse() * (point - Center);
        Fixed64 radialDistance = FixedMath.Sqrt(local.X * local.X + local.Z * local.Z);
        Fixed64 baseDistance = (local.Y + HalfHeight).Abs();
        Fixed64 sideRadius = RadiusAtLocalY(local.Y);
        Fixed64 sideDistance = (radialDistance - sideRadius).Abs();

        if (baseDistance <= sideDistance && local.Y <= -HalfHeight + Fixed64.Epsilon)
            return Rotation * -Vector3d.Up;

        if (radialDistance <= Fixed64.Epsilon)
        {
            return local.Y >= HalfHeight - Fixed64.Epsilon
                ? Rotation * Vector3d.Up
                : Rotation * Vector3d.Right;
        }

        Vector3d radialDirection = new(local.X / radialDistance, Fixed64.Zero, local.Z / radialDistance);
        Vector3d localNormal = (radialDirection * Height + Vector3d.Up * ScaledRadius).Normalized;
        return Rotation * localNormal;
    }

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        worker.CheckConeOverlaps(this, ref outputIntersectionPoints);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 RadiusAtLocalY(Fixed64 localY)
    {
        if (localY <= -HalfHeight)
            return ScaledRadius;
        if (localY >= HalfHeight)
            return Fixed64.Zero;

        return ScaledRadius * ((HalfHeight - localY) / Height);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool ContainsWorldPoint(Vector3d point, Fixed64 tolerance = default)
    {
        Vector3d local = Rotation.Inverse() * (point - Center);
        Fixed64 radialSqr = local.X * local.X + local.Z * local.Z;
        Fixed64 radius = RadiusAtLocalY(local.Y);
        return local.Y >= -HalfHeight - tolerance
            && local.Y <= HalfHeight + tolerance
            && radialSqr <= (radius + tolerance) * (radius + tolerance);
    }

    private void ProjectPointOntoConeSide(
        Fixed64 rho,
        Fixed64 y,
        out Fixed64 projectedRho,
        out Fixed64 projectedY)
    {
        Fixed64 segmentRho = -ScaledRadius;
        Fixed64 segmentY = Height;
        Fixed64 denominator = segmentRho * segmentRho + segmentY * segmentY;
        Fixed64 t = denominator <= Fixed64.Epsilon
            ? Fixed64.Zero
            : ((rho - ScaledRadius) * segmentRho + (y + HalfHeight) * segmentY) / denominator;
        t = FixedMath.Clamp01(t);
        projectedRho = ScaledRadius + segmentRho * t;
        projectedY = -HalfHeight + segmentY * t;
    }

    private void ProjectPointOntoBase(
        Fixed64 rho,
        Fixed64 y,
        out Fixed64 projectedRho,
        out Fixed64 projectedY)
    {
        projectedRho = FixedMath.Clamp(rho, Fixed64.Zero, ScaledRadius);
        projectedY = -HalfHeight;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 Square(Fixed64 value) => value * value;
}
