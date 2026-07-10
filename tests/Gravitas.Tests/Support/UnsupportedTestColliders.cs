//=======================================================================
// UnsupportedTestColliders.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Queries;
using SwiftCollections;

namespace Gravitas.Tests.Support;

internal sealed class UnsupportedTestCollider2D : LSCollider2D
{
    private readonly bool _containsPoints;

    public UnsupportedTestCollider2D(bool containsPoints = false)
    {
        _containsPoints = containsPoints;
    }

    public override ColliderType2D Shape => (ColliderType2D)byte.MaxValue;

    public override bool ContainsPoint(Vector2d point) => _containsPoints;

    public override Vector2d GetClosestPoint(Vector2d point) =>
        new(
            FixedMath.Clamp(point.X, Center.X - Fixed64.One, Center.X + Fixed64.One),
            FixedMath.Clamp(point.Y, Center.Y - Fixed64.One, Center.Y + Fixed64.One));

    public override Vector2d GetSupportPoint(Vector2d direction) => Center;

    internal override int VertexCount => 0;

    internal override Vector2d GetVertexUnchecked(int index) => Center;

    public override Fixed64 CalculateMomentOfInertia(Fixed64 mass, Vector2d localReferencePoint) => Fixed64.Zero;

    internal override Fixed64 CalculateAreaForMassProperties() => Fixed64.Zero;

    protected override void RebuildShape() =>
        SetBoundsFromMinMax(Center - Vector2d.One, Center + Vector2d.One);
}

internal sealed class UnsupportedTestCollider3D : LSCollider
{
    public override ColliderType Shape => (ColliderType)byte.MaxValue;

    public override int Priority => 0;

    protected override void BuildShape()
    {
        Area = Fixed64.One;
        SetBoundsMinMax(Center - Vector3d.One, Center + Vector3d.One);
    }

    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localCenterOfMassOffset) =>
        Fixed3x3.Zero;

    public override Vector3d ClosestPointOnSurface(Vector3d other) => Center;

    public override Vector3d GetNormalAtPoint(Vector3d point) => Vector3d.Up;

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        false;
}
