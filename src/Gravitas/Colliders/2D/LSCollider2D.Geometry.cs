//=======================================================================
// LSCollider2D.Geometry.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

public abstract partial class LSCollider2D
{
    public Vector2d Position => _compoundOwner != null
        ? _compoundOwner.Center
        : ResolveStandalonePosition();

    public Fixed64 Rotation
    {
        get
        {
            if (_hasCommittedShape)
                return _committedRotation;
            if (_compoundOwner == null)
                return ResolveStandaloneRotation();

            return PlanarRotation.Combine(
                _compoundOwner.Rotation,
                _compoundLocalRotation);
        }
    }

    public virtual Vector2d LocalScale
    {
        get
        {
            bool representable = TryGetLocalScale(out Vector2d scale);
            SwiftThrowHelper.ThrowIfTrue(
                !representable,
                nameof(LocalScale),
                "The combined 2D collider scale is outside the Fixed64 coordinate domain.");
            return scale;
        }
    }

    /// <summary>
    /// Attempts to materialize the combined owner and compound-part scale.
    /// Shape preparation retains those factors separately and does not require
    /// their product to be representable.
    /// </summary>
    public bool TryGetLocalScale(out Vector2d scale)
    {
        Vector2d ownerScale = GetCurrentOwnerScale();
        Vector2d partScale = _hasCommittedShape
            ? _committedPartScale
            : _compoundOwner != null
                ? _compoundLocalScale
                : Vector2d.One;

        bool representable = Fixed64.TryMultiplyDivide(
                ownerScale.X,
                partScale.X,
                Fixed64.One,
                out Fixed64 x)
            & Fixed64.TryMultiplyDivide(
                ownerScale.Y,
                partScale.Y,
                Fixed64.One,
                out Fixed64 y);
        scale = representable ? new Vector2d(x, y) : default;
        return representable;
    }

    /// <summary>
    /// Gets the component-scaled local offset.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// The scaled offset itself is outside the Fixed64 coordinate domain. The
    /// collider may still have a representable world center through exact
    /// transform cancellation.
    /// </exception>
    public Vector2d ScaledLocalOffset => GetCurrentScaledOffset();

    /// <summary>
    /// Attempts to materialize the component-scaled local offset independently
    /// from the collider's complete world transform.
    /// </summary>
    public bool TryGetScaledLocalOffset(out Vector2d scaledLocalOffset) =>
        TryGetCurrentScaledOffset(out scaledLocalOffset);

    public Vector2d Center => _hasCommittedShape
        ? _committedCenter
        : GetUncommittedCenter();

    public abstract bool ContainsPoint(Vector2d point);

    public abstract Vector2d GetClosestPoint(Vector2d point);

    public abstract Vector2d GetSupportPoint(Vector2d direction);

    internal int VertexCount
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this is IConvexVertexSource2D source ? source.VertexCount : 0;
    }

    internal Fixed64 ConvexRotation =>
        ((IConvexVertexSource2D)this).Rotation;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Vector2d GetScaledLocalVertexUnchecked(int index) =>
        ((IConvexVertexSource2D)this).GetScaledLocalVertexUnchecked(index);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal FixedPointAnchor2d GetConvexSupportAnchor(Vector2d direction) =>
        ((IConvexVertexSource2D)this).GetSupportAnchor(direction);

    internal bool TryGetClosestBoundaryAnchor(
        Vector2d point,
        out FixedPointAnchor2d boundary,
        out Fixed64 distance)
    {
        switch (this)
        {
            case LSCircleCollider2D circle:
                return TryGetRoundBoundaryAnchor(
                    circle.Center,
                    circle.Rotation,
                    Vector2d.Forward,
                    Fixed64.Zero,
                    circle.ScaledRadius,
                    point,
                    Vector2d.Right,
                    out boundary,
                    out distance);
            case LSCapsuleCollider2D capsule:
                return TryGetRoundBoundaryAnchor(
                    capsule.Center,
                    capsule.Rotation,
                    Vector2d.Forward,
                    capsule.AxisLength,
                    capsule.ScaledRadius,
                    point,
                    capsule.GetNormalFromCenteredAxis(point),
                    out boundary,
                    out distance);
            case LSAABBoxCollider2D box:
                return TryGetAABoxBoundaryAnchor(
                    box,
                    point,
                    out boundary,
                    out distance);
            case LSPolygonCollider2D polygon:
                return TryGetConvexBoundaryAnchor(
                    polygon.Center,
                    polygon.Rotation,
                    polygon.ScaledLocalVertices,
                    point,
                    out boundary,
                    out distance);
            case LSCompoundCollider2D compound:
                return TryGetCompoundBoundaryAnchor(
                    compound,
                    point,
                    out boundary,
                    out distance);
            default:
                boundary = default;
                distance = default;
                return false;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetVertex(int index, out Vector2d vertex) =>
        Vector2d.TryTransformPoint(
            Center,
            GetScaledLocalVertexUnchecked(index),
            ConvexRotation,
            out vertex);

    private static bool TryGetRoundBoundaryAnchor(
        Vector2d center,
        Fixed64 rotation,
        Vector2d localAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d point,
        Vector2d fallbackNormal,
        out FixedPointAnchor2d boundary,
        out Fixed64 distance)
    {
        Vector2d normal = FixedSegment2d.GetDirectionFromCenteredAxis(
            point,
            center,
            rotation,
            axisLength);
        if (normal == Vector2d.Zero)
            normal = fallbackNormal;

        if (Vector2d.TryRotate(
                normal,
                -rotation,
                out Vector2d localNormal))
        {
            localNormal = localNormal.Normalized;
            FixedPointAnchor2d candidate =
                FixedSegment2d.GetSurfaceAnchorOnCenteredCapsule(
                    point,
                    center,
                    rotation,
                    localAxis,
                    axisLength,
                    radius,
                    localNormal);
            if (TryGetAnchorDistance(
                    point,
                    candidate,
                    out distance))
            {
                boundary = candidate;
                return true;
            }
        }

        boundary = default;
        distance = default;
        return false;
    }

    private static bool TryGetAABoxBoundaryAnchor(
        LSAABBoxCollider2D box,
        Vector2d point,
        out FixedPointAnchor2d boundary,
        out Fixed64 distance)
    {
        Vector2d halfExtents = box.ScaledHalfExtents;
        Span<Vector2d> offsets = stackalloc Vector2d[4];
        offsets[0] = new Vector2d(-halfExtents.X, halfExtents.Y);
        offsets[1] = new Vector2d(-halfExtents.X, -halfExtents.Y);
        offsets[2] = new Vector2d(halfExtents.X, -halfExtents.Y);
        offsets[3] = new Vector2d(halfExtents.X, halfExtents.Y);
        return TryGetConvexBoundaryAnchor(
            box.Center,
            Fixed64.Zero,
            offsets,
            point,
            out boundary,
            out distance);
    }

    private static bool TryGetConvexBoundaryAnchor(
        Vector2d center,
        Fixed64 rotation,
        ReadOnlySpan<Vector2d> offsets,
        Vector2d point,
        out FixedPointAnchor2d boundary,
        out Fixed64 distance)
    {
        FixedPointAnchor2d candidate =
            FixedConvex2dRelations.GetClosestPointAnchor(
                point,
                center,
                rotation,
                offsets);
        if (TryGetAnchorDistance(
                point,
                candidate,
                out distance))
        {
            boundary = candidate;
            return true;
        }

        boundary = default;
        distance = default;
        return false;
    }

    private static bool TryGetCompoundBoundaryAnchor(
        LSCompoundCollider2D compound,
        Vector2d point,
        out FixedPointAnchor2d boundary,
        out Fixed64 distance)
    {
        if (TryGetCompoundBoundaryAnchor(
                compound,
                point,
                containingPartsOnly: true,
                out boundary,
                out distance))
        {
            return true;
        }

        return TryGetCompoundBoundaryAnchor(
            compound,
            point,
            containingPartsOnly: false,
            out boundary,
            out distance);
    }

    private static bool TryGetCompoundBoundaryAnchor(
        LSCompoundCollider2D compound,
        Vector2d point,
        bool containingPartsOnly,
        out FixedPointAnchor2d boundary,
        out Fixed64 distance)
    {
        boundary = default;
        distance = default;
        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;
        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            if ((containingPartsOnly && !part.ContainsPoint(point))
                || !part.TryGetClosestBoundaryAnchor(
                    point,
                    out FixedPointAnchor2d candidate,
                    out Fixed64 candidateDistance)
                || (found && candidateDistance >= bestDistance))
            {
                continue;
            }

            boundary = candidate;
            distance = candidateDistance;
            bestDistance = candidateDistance;
            found = true;
        }

        return found;
    }

    private static bool TryGetAnchorDistance(
        Vector2d point,
        in FixedPointAnchor2d anchor,
        out Fixed64 distance)
    {
        if (anchor.TryGetOffsetFrom(
                new FixedPointAnchor2d(
                    point,
                    Fixed64.Zero,
                    Vector2d.Zero),
                out Vector2d difference)
            && Vector2d.TryGetMagnitude(difference, out distance))
        {
            return true;
        }

        distance = default;
        return false;
    }

    /// <summary>
    /// Calculates the body-local center of mass implied by this 2D collider's current shape state.
    /// </summary>
    public virtual Vector2d CalculateLocalCenterOfMassOffset()
    {
        if (!_hasCommittedShape)
            return TransformRelativeMassPropertyPoint(Vector2d.Zero);

        SwiftThrowHelper.ThrowIfTrue(
            !_hasCommittedDefaultCenterOfMassOffset,
            nameof(CalculateLocalCenterOfMassOffset),
            "The 2D collider's body-local center of mass is outside the Fixed64 coordinate domain.");
        return _defaultCenterOfMassOffset;
    }

    /// <summary>
    /// Calculates the scalar moment of inertia about a requested body-local reference point.
    /// </summary>
    public abstract Fixed64 CalculateMomentOfInertia(Fixed64 mass, Vector2d localReferencePoint);

    internal abstract Fixed64 CalculateAreaForMassProperties();

    protected virtual void OnMaterialChanged() { }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    protected void MarkShapeDirty()
    {
        _shapeVersion++;
        _runtimeShapeState.MarkDirty();
        if (_compoundOwner != null)
        {
            _compoundOwner.MarkShapeDirty();
            return;
        }

        _body?.Wake();
    }

    protected static Fixed64 ApplyParallelAxis(
        Fixed64 momentAboutCenterOfMass,
        Fixed64 mass,
        Vector2d centerOfMass,
        Vector2d localReferencePoint)
    {
        Vector2d delta = localReferencePoint - centerOfMass;
        return momentAboutCenterOfMass + mass * delta.MagnitudeSquared;
    }

    private bool RebuildRuntimeShapeState()
    {
        ColliderShapeSnapshot2D snapshot = CaptureShapeSnapshot();
        if (!_runtimeShapeState.ShouldRebuild(snapshot))
            return false;

        PrepareRuntimeShape(
            snapshot,
            requireRepresentableMassPoint:
                _body != null || _compoundOwner != null);
        PublishPreparedShape();
        _body?.RefreshMassPropertiesFromColliderShape();
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Vector2d ResolveStandalonePosition() =>
        _body?.Position
        ?? _agent?.Transform.WorldPositionXZ
        ?? Vector2d.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Fixed64 ResolveStandaloneRotation() =>
        _body?.Rotation ?? ResolveAgentRotation();
}
