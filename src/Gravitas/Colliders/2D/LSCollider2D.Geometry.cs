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
    /// <summary>Gets the planar runtime position.</summary>
    public Vector2d Position => _compoundOwner != null
        ? _compoundOwner.Center
        : ResolveStandalonePosition();

    /// <summary>Gets the committed planar rotation in radians.</summary>
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

    /// <summary>Gets the combined planar owner and compound-part scale.</summary>
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
        GetCurrentScaleFactors(
            out Vector2d ownerScale,
            out Vector2d partScale);

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

    /// <summary>Gets the collider center in planar world coordinates.</summary>
    public Vector2d Center => _hasCommittedShape
        ? _committedCenter
        : GetUncommittedCenter();

    /// <summary>Gets whether a planar world-space point lies inside this collider.</summary>
    public abstract bool ContainsPoint(Vector2d point);

    /// <summary>Gets the closest representable point on this collider to a planar world-space point.</summary>
    public abstract Vector2d GetClosestPoint(Vector2d point);

    /// <summary>Gets the farthest representable point along a planar world-space direction.</summary>
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
        out FixedPointAnchor2d boundary)
    {
        switch (this)
        {
            case LSCircleCollider2D circle:
                boundary = GetRoundBoundaryAnchor(
                    circle.Center,
                    circle.Rotation,
                    Vector2d.Forward,
                    Fixed64.Zero,
                    circle.ScaledRadius,
                    point,
                    Vector2d.Right);
                return true;
            case LSCapsuleCollider2D capsule:
                boundary = GetRoundBoundaryAnchor(
                    capsule.Center,
                    capsule.Rotation,
                    Vector2d.Forward,
                    capsule.AxisLength,
                    capsule.ScaledRadius,
                    point,
                    capsule.GetNormalFromCenteredAxis(point));
                return true;
            case LSAABBoxCollider2D box:
                return TryGetAABoxBoundaryAnchor(
                    box,
                    point,
                    out boundary);
            case LSPolygonCollider2D polygon:
                return TryGetConvexBoundaryAnchor(
                    polygon.Center,
                    polygon.Rotation,
                    polygon.ScaledLocalVertices,
                    point,
                    out boundary);
            case LSCompoundCollider2D compound:
                return TryGetCompoundBoundaryAnchor(
                    compound,
                    point,
                    out boundary);
            default:
                boundary = default;
                return false;
        }
    }

    internal bool TryGetClosestBoundaryAnchor(
        Vector2d point,
        out FixedPointAnchor2d boundary,
        out Fixed64 distance)
    {
        if (TryGetClosestBoundaryAnchor(
                point,
                out boundary)
            && TryGetAnchorDistance(
                point,
                boundary,
                out distance))
        {
            return true;
        }

        boundary = default;
        distance = default;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetVertex(int index, out Vector2d vertex) =>
        Vector2d.TryTransformPoint(
            Center,
            GetScaledLocalVertexUnchecked(index),
            ConvexRotation,
            out vertex);

    private static FixedPointAnchor2d GetRoundBoundaryAnchor(
        Vector2d center,
        Fixed64 rotation,
        Vector2d localAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d point,
        Vector2d fallbackNormal)
    {
        Vector2d normal = FixedSegment2d.GetDirectionFromCenteredAxis(
            point,
            center,
            rotation,
            axisLength);
        if (normal == Vector2d.Zero)
            normal = fallbackNormal;

        Vector2d localNormal =
            Vector2d.Rotate(normal, -rotation).Normalized;
        return FixedSegment2d.GetSurfaceAnchorOnCenteredCapsule(
            point,
            center,
            rotation,
            localAxis,
            axisLength,
            radius,
            localNormal);
    }

    private static bool TryGetAABoxBoundaryAnchor(
        LSAABBoxCollider2D box,
        Vector2d point,
        out FixedPointAnchor2d boundary)
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
            out boundary);
    }

    private static bool TryGetConvexBoundaryAnchor(
        Vector2d center,
        Fixed64 rotation,
        ReadOnlySpan<Vector2d> offsets,
        Vector2d point,
        out FixedPointAnchor2d boundary)
    {
        boundary =
            FixedConvex2dRelations.GetClosestPointAnchor(
                point,
                center,
                rotation,
                offsets);
        return true;
    }

    private static bool TryGetCompoundBoundaryAnchor(
        LSCompoundCollider2D compound,
        Vector2d point,
        out FixedPointAnchor2d boundary)
    {
        var reference = new FixedPointAnchor2d(
            point,
            Fixed64.Zero,
            Vector2d.Zero);
        if (TryGetCompoundBoundaryAnchor(
                compound,
                point,
                reference,
                containingPartsOnly: true,
                out boundary))
        {
            return true;
        }

        return TryGetCompoundBoundaryAnchor(
            compound,
            point,
            reference,
            containingPartsOnly: false,
            out boundary);
    }

    private static bool TryGetCompoundBoundaryAnchor(
        LSCompoundCollider2D compound,
        Vector2d point,
        in FixedPointAnchor2d reference,
        bool containingPartsOnly,
        out FixedPointAnchor2d boundary)
    {
        boundary = default;
        bool found = false;
        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            if (containingPartsOnly && !part.ContainsPoint(point))
                continue;

            // Compound definitions materialize only built-in semantic shapes.
            _ = part.TryGetClosestBoundaryAnchor(
                point,
                out FixedPointAnchor2d candidate);
            if (found
                && reference.CompareSquaredDistance(
                    candidate,
                    boundary) >= 0)
            {
                continue;
            }

            boundary = candidate;
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
        ExactMassPoint2D point = CalculateLocalMassPoint();
        SwiftThrowHelper.ThrowIfTrue(
            !point.TryGetPoint(out Vector2d center),
            nameof(CalculateLocalCenterOfMassOffset),
            "The 2D collider's body-local center of mass is outside the Fixed64 coordinate domain.");
        return center;
    }

    internal virtual ExactMassPoint2D CalculateLocalMassPoint() =>
        TransformRelativeMassPropertyPointExact(Vector2d.Zero);

    internal virtual ExactMassPoint2D CalculatePreparedLocalMassPoint() =>
        TransformPreparedRelativeMassPropertyPointExact(Vector2d.Zero);

    /// <summary>
    /// Calculates the scalar moment of inertia about a requested body-local reference point.
    /// </summary>
    public virtual Fixed64 CalculateMomentOfInertia(
        Fixed64 mass,
        Vector2d localReferencePoint)
    {
        if (mass <= Fixed64.Zero)
            return Fixed64.Zero;

        Fixed64 centerMoment =
            CalculateCenterOfMassMoment(mass);
        ExactMassPoint2D massPoint = CalculateLocalMassPoint();
        if (massPoint.TryAddParallelAxisMoment(
                centerMoment,
                mass,
                localReferencePoint,
                out Fixed64 moment))
        {
            return moment;
        }

        if (!massPoint.TryGetPoint(out Vector2d center))
        {
            SwiftThrowHelper.ThrowIfTrue(
                true,
                nameof(localReferencePoint),
                "The requested moment of inertia is outside the Fixed64 scalar domain.");
        }

        Vector2d delta = localReferencePoint - center;
        return centerMoment
            + mass * delta.MagnitudeSquared;
    }

    internal abstract Fixed64 CalculateCenterOfMassMoment(Fixed64 mass);

    internal abstract ExactMassWeight CalculateAreaForMassProperties();

    internal abstract ExactMassWeight CalculatePreparedAreaForMassProperties();

    /// <summary>Updates derived material state after <see cref="Material"/> changes.</summary>
    protected virtual void OnMaterialChanged() { }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal ExactMassPoint2D TransformPreparedRelativeMassPropertyPointExact(
        Vector2d partRelativePoint) =>
        _compoundOwner == null
            ? ExactMassPoint2D.CreateScaledLocalComposition(
                _preparedSnapshot.LocalOffset,
                _preparedSnapshot.OwnerScale,
                Vector2d.Zero,
                Vector2d.One,
                partRelativePoint,
                Fixed64.Zero)
            : ExactMassPoint2D.CreateScaledLocalComposition(
                _compoundOwner._preparedSnapshot.LocalOffset,
                _preparedSnapshot.OwnerScale,
                _preparedSnapshot.LocalOffset,
                _preparedSnapshot.OwnerScale,
                partRelativePoint,
                _preparedCompoundLocalRotation);

    /// <summary>Marks derived runtime shape state for deterministic rebuilding.</summary>
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
