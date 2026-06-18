using FixedMathSharp;
using Gravitas.Queries;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

public class LSCylinderCollider : LSCollider
{
    public LSCylinderCollider() { }

    public LSCylinderCollider(ColliderShapeDefinition definition)
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.Cylinder);
        Radius = definition.Radius;
        Size = definition.Size;
    }

    public override ColliderType Shape => ColliderType.Cylinder;
    public override int Priority => ColliderSettings.GetPriority(Shape);

    public override Fixed64 ScaledRadius => _radius * FixedMath.Max(LocalScale.X, LocalScale.Z);

    /// <summary>
    /// Local bottom cap center before world rotation and translation.
    /// </summary>
    public Vector3d CapCenterBottom { get; private set; }

    /// <summary>
    /// Local top cap center before world rotation and translation.
    /// </summary>
    public Vector3d CapCenterTop { get; private set; }

    public Fixed64 Height { get; private set; }

    public Fixed64 HalfHeight { get; private set; }

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

    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass)
    {
        Fixed64 radiusSqr = ScaledRadiusSqr;
        Fixed64 heightSqr = Height * Height;
        Fixed64 inertiaXZ = Fixed64.FromFraction(1, 12) * mass * ((3 * radiusSqr) + heightSqr);
        Fixed64 inertiaY = Fixed64.Half * mass * radiusSqr;

        return new Fixed3x3(
            inertiaXZ, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, inertiaY, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, inertiaXZ
        );
    }

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        return worker.CheckCylinderOverlaps(this, ref outputIntersectionPoints);
    }

    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        Vector3d local = Rotation.Inverse() * (other - Center);
        Vector3d radial = new(local.X, Fixed64.Zero, local.Z);
        Fixed64 radialDistance = radial.Magnitude;
        Fixed64 clampedY = FixedMath.Clamp(local.Y, -HalfHeight, HalfHeight);

        if (IsInsideFiniteCylinder(local, radialDistance))
        {
            Fixed64 sideDistance = ScaledRadius - radialDistance;
            Fixed64 capDistance = HalfHeight - local.Y.Abs();

            if (sideDistance <= capDistance)
            {
                Vector3d direction = radialDistance > Fixed64.Epsilon
                    ? radial / radialDistance
                    : Vector3d.Right;
                return Center + Rotation * new Vector3d(direction.X * ScaledRadius, local.Y, direction.Z * ScaledRadius);
            }

            return Center + Rotation * new Vector3d(local.X, local.Y.Sign() * HalfHeight, local.Z);
        }

        Vector3d radialDirection;
        if (radialDistance > Fixed64.Epsilon)
            radialDirection = radial / radialDistance;
        else
            radialDirection = Vector3d.Right;

        Fixed64 surfaceRadius = radialDistance > ScaledRadius ? ScaledRadius : radialDistance;
        Vector3d surfaceLocal = new(
            radialDirection.X * surfaceRadius,
            clampedY,
            radialDirection.Z * surfaceRadius);

        return Center + Rotation * surfaceLocal;
    }

    public override Vector3d GetNormalAtPoint(Vector3d point)
    {
        Vector3d local = Rotation.Inverse() * (point - Center);
        Fixed64 radialDistance = FixedMath.Sqrt(local.X * local.X + local.Z * local.Z);
        Fixed64 sideDistance = (radialDistance - ScaledRadius).Abs();
        Fixed64 capDistance = (local.Y.Abs() - HalfHeight).Abs();

        if (capDistance <= sideDistance && local.Y.Abs() >= HalfHeight - Fixed64.Epsilon)
            return Rotation * new Vector3d(Fixed64.Zero, local.Y >= Fixed64.Zero ? Fixed64.One : -Fixed64.One, Fixed64.Zero);

        if (radialDistance <= Fixed64.Epsilon)
            return Rotation * Vector3d.Right;

        return Rotation * new Vector3d(local.X / radialDistance, Fixed64.Zero, local.Z / radialDistance);
    }

    protected override void BuildShape()
    {
        Height = ScaledSize.Y;
        HalfHeight = Height * Fixed64.Half;
        CapCenterBottom = new Vector3d(Fixed64.Zero, -HalfHeight, Fixed64.Zero);
        CapCenterTop = new Vector3d(Fixed64.Zero, HalfHeight, Fixed64.Zero);
        LineSegmentStart = Center + Rotation * CapCenterBottom;
        LineSegmentEnd = Center + Rotation * CapCenterTop;

        Area = 2 * Fixed64.Pi * ScaledRadius * (Height + ScaledRadius);
    }

    public override Fixed64 GetFrontalArea(Vector3d direction)
    {
        Fixed64 directionMagnitude = direction.Magnitude;
        if (directionMagnitude <= Fixed64.Epsilon)
            return Area;

        Vector3d normalizedDirection = direction / directionMagnitude;
        Fixed64 axial = Vector3d.Dot(normalizedDirection, LineDirection).Abs();
        Fixed64 radialFactorSqr = Fixed64.One - axial * axial;
        Fixed64 radialFactor = radialFactorSqr <= Fixed64.Zero ? Fixed64.Zero : FixedMath.Sqrt(radialFactorSqr);

        return axial * Fixed64.Pi * ScaledRadiusSqr
            + radialFactor * 2 * ScaledRadius * Height;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsInsideFiniteCylinder(Vector3d local, Fixed64 radialDistance) =>
        radialDistance <= ScaledRadius && local.Y >= -HalfHeight && local.Y <= HalfHeight;
}
