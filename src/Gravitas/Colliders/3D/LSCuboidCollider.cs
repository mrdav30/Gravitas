//=======================================================================
// LSCuboidCollider.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Queries;
using SwiftCollections;
using System;

namespace Gravitas.Colliders;

public sealed class LSCuboidCollider : LSCollider
{
    private FixedOrientedBox _orientedBox;
    private FixedOrientedBox _preparedOrientedBox;
    private Fixed64 _preparedArea;

    public override ColliderType Shape => Rotation == FixedQuaternion.Identity ? ColliderType.AABox : ColliderType.OBBox;

    public override int Priority => ColliderSettings.GetPriority(Shape);

    /// <summary>
    /// Gets the canonical committed oriented-box geometry.
    /// </summary>
    public FixedOrientedBox OrientedBox => _orientedBox;

    public override Fixed64 ScaledRadius =>
        HasCommittedShape
            ? CanonicalCenteredProxyRadius
            : ColliderCanonicalBounds
                .GetCurrentCenteredProxyRadius(this);

    protected internal override FixedMassWeight CalculateMassPropertyWeight()
    {
        Vector3d halfExtents;
        if (HasCommittedShape)
        {
            halfExtents = _orientedBox.HalfExtents;
        }
        else
        {
            GetCurrentShapeScales(
                out Vector3d ownerScale,
                out Vector3d partScale);
            halfExtents = ColliderScalePolicy.ScalePositive(
                Size,
                ownerScale,
                partScale,
                Fixed64.Two);
        }
        return FixedMassWeight.FromProduct(
            halfExtents.X,
            halfExtents.Y,
            halfExtents.Z,
            (Fixed64)8);
    }

    internal override FixedMassWeight CalculatePreparedMassPropertyWeight()
    {
        Vector3d halfExtents = _preparedOrientedBox.HalfExtents;
        return FixedMassWeight.FromProduct(
            halfExtents.X,
            halfExtents.Y,
            halfExtents.Z,
            (Fixed64)8);
    }

    public LSCuboidCollider() { }

    public LSCuboidCollider(ColliderShapeDefinition definition)
        : this()
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.Cuboid);
        Material = definition.Material;
        Size = definition.Size;
    }

    private protected override void PrepareShape(in ColliderShapeSnapshot snapshot)
    {
        Vector3d halfExtents = ColliderScalePolicy.ScalePositive(
            snapshot.Size,
            snapshot.OwnerScale,
            snapshot.PartScale,
            Fixed64.Two);
        _preparedOrientedBox = new FixedOrientedBox(
            snapshot.Center,
            snapshot.Rotation,
            halfExtents);
        _preparedArea = GetSurfaceArea(halfExtents);
        SetPreparedBounds(_preparedOrientedBox.GetBoundsClippedToDomain());
    }

    private protected override void PublishShape()
    {
        _orientedBox = _preparedOrientedBox;
        Area = _preparedArea;
    }

    internal override Fixed3x3 CalculateCenterOfMassInertiaTensor(Fixed64 mass)
    {
        Vector3d halfExtents = _orientedBox.HalfExtents;
        Fixed64 xContribution = GetInertiaContribution(mass, halfExtents.X);
        Fixed64 yContribution = GetInertiaContribution(mass, halfExtents.Y);
        Fixed64 zContribution = GetInertiaContribution(mass, halfExtents.Z);
        Fixed64 xx = yContribution + zContribution;
        Fixed64 yy = xContribution + zContribution;
        Fixed64 zz = xContribution + yContribution;

        Fixed3x3 tensor = new(
            xx, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, yy, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, zz
        );
        return tensor;
    }

    public override Fixed64 GetFrontalArea(Vector3d direction)
    {
        if (direction.MagnitudeSquared <= Fixed64.Epsilon)
            return Area;

        direction = direction.Normalized;
        _orientedBox.GetAxes(out Vector3d axisX, out Vector3d axisY, out Vector3d axisZ);
        Fixed64 dotX = Vector3d.Dot(direction, axisX).Abs();
        Fixed64 dotY = Vector3d.Dot(direction, axisY).Abs();
        Fixed64 dotZ = Vector3d.Dot(direction, axisZ).Abs();
        Vector3d halfExtents = _orientedBox.HalfExtents;

        // The orthographic projection of a box is the sum of each face area's
        // contribution along the view direction.
        return GetProjectedFaceArea(halfExtents.Y, halfExtents.Z, dotX)
            + GetProjectedFaceArea(halfExtents.X, halfExtents.Z, dotY)
            + GetProjectedFaceArea(halfExtents.X, halfExtents.Y, dotZ);
    }

    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        if (_orientedBox.TryGetClosestPointOnSurface(other, out Vector3d closestPoint))
            return closestPoint;

        throw new InvalidOperationException(
            "The selected cuboid surface point is outside the representable coordinate domain.");
    }

    public override Vector3d GetNormalAtPoint(Vector3d point) =>
        _orientedBox.GetNearestFaceNormal(point);

    internal override bool TryGetPlanarSurfaceNormal(Vector3d point, out Vector3d normal)
    {
        normal = GetNormalAtPoint(point);
        return true;
    }

    private static Fixed64 GetSurfaceArea(Vector3d halfExtents) =>
        GetScaledProduct(halfExtents.X, halfExtents.Y, (Fixed64)8)
        + GetScaledProduct(halfExtents.X, halfExtents.Z, (Fixed64)8)
        + GetScaledProduct(halfExtents.Y, halfExtents.Z, (Fixed64)8);

    private static Fixed64 GetProjectedFaceArea(
        Fixed64 firstHalfExtent,
        Fixed64 secondHalfExtent,
        Fixed64 alignment) =>
        Fixed64.TryMultiplyDivide(
            firstHalfExtent,
            secondHalfExtent,
            alignment,
            Fixed64.One / (Fixed64)4,
            out Fixed64 area)
            ? area
            : Fixed64.MaxValue;

    private static Fixed64 GetInertiaContribution(Fixed64 mass, Fixed64 halfExtent) =>
        Fixed64.TryMultiplyDivide(
            mass,
            halfExtent,
            halfExtent,
            (Fixed64)3,
            out Fixed64 contribution)
            ? contribution
            : Fixed64.MaxValue;

    private static Fixed64 GetScaledProduct(
        Fixed64 first,
        Fixed64 second,
        Fixed64 multiplier) =>
        Fixed64.TryMultiplyDivide(
            first,
            second,
            multiplier,
            Fixed64.One,
            out Fixed64 product)
            ? product
            : Fixed64.MaxValue;

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (Shape == ColliderType.AABox)
            return worker.CheckAABBoxOverlaps(this, ref outputIntersectionPoints);

        return worker.CheckOBBoxOverlaps(this, ref outputIntersectionPoints);
    }
}
