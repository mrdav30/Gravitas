//=======================================================================
// LSCollider.ShapeTransaction.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.Colliders;

public abstract partial class LSCollider
{
    private Vector3d _committedCenter;
    private FixedQuaternion _committedRotation = FixedQuaternion.Identity;
    private Vector3d _committedOwnerScale = Vector3d.One;
    private Vector3d _committedPartScale = Vector3d.One;
    private bool _hasCommittedShape;

    /// <summary>Gets whether this collider has committed runtime shape state.</summary>
    protected bool HasCommittedShape => _hasCommittedShape;

    private ColliderShapeSnapshot _preparedSnapshot;
    private FixedBoundBox _preparedBounds;
    private Fixed64 _canonicalCenteredProxyRadius = Fixed64.MaxValue;
    private Fixed64 _canonicalGroundProbeRadius;
    private GravitasWorldContext? _preparedContext;
    private FixedQuaternion _preparedCompoundLocalRotation = FixedQuaternion.Identity;

    private protected GravitasWorldContext PreparedContext => _preparedContext!;

    internal FixedBoundBox PreparedShapeBounds => _preparedBounds;

    internal Fixed64 CanonicalCenteredProxyRadius =>
        _canonicalCenteredProxyRadius;

    internal Fixed64 CanonicalGroundProbeRadius =>
        _canonicalGroundProbeRadius;

    internal Vector3d CanonicalCenter => _committedCenter;

    internal FixedQuaternion CanonicalRotation => _committedRotation;

    internal bool TryGetCommittedOwnerScale(out Vector3d scale)
    {
        if (_hasCommittedShape)
        {
            scale = _committedOwnerScale;
            return true;
        }

        scale = default;
        return false;
    }

    private Vector3d GetCurrentOwnerScale()
    {
        if (_hasCommittedShape)
            return _committedOwnerScale;
        if (_compoundOwner != null)
            return _compoundOwner.GetCurrentOwnerScale();
        return Vector3d.One;
    }

    private Vector3d GetCurrentPartScale() =>
        _hasCommittedShape
            ? _committedPartScale
            : _compoundOwner == null
                ? Vector3d.One
                : _compoundLocalScale;

    internal void GetCurrentShapeScales(
        out Vector3d ownerScale,
        out Vector3d partScale)
    {
        ownerScale = GetCurrentOwnerScale();
        partScale = GetCurrentPartScale();
    }

    internal bool TryGetCurrentScaledOffset(out Vector3d scaledOffset)
    {
        Vector3d ownerScale = GetCurrentOwnerScale();
        bool representable = Fixed64.TryMultiplyDivide(
                _offset.X,
                ownerScale.X,
                Fixed64.One,
                out Fixed64 x)
            & Fixed64.TryMultiplyDivide(
                _offset.Y,
                ownerScale.Y,
                Fixed64.One,
                out Fixed64 y)
            & Fixed64.TryMultiplyDivide(
                _offset.Z,
                ownerScale.Z,
                Fixed64.One,
                out Fixed64 z);
        scaledOffset = representable
            ? new Vector3d(x, y, z)
            : default;
        return representable;
    }

    internal Vector3d GetCurrentScaledOffset()
    {
        bool representable =
            TryGetCurrentScaledOffset(out Vector3d scaledOffset);
        SwiftThrowHelper.ThrowIfTrue(
            !representable,
            nameof(ScaledOffset),
            "The scaled collider offset is outside the Fixed64 coordinate domain.");
        return scaledOffset;
    }

    internal Fixed64 GetCurrentScaledRadius()
    {
        Vector3d ownerScale = GetCurrentOwnerScale();
        Vector3d partScale = GetCurrentPartScale();
        return FixedMath.Max(
            ColliderScalePolicy.ScalePositive(
                _radius,
                ownerScale.X,
                partScale.X),
            FixedMath.Max(
                ColliderScalePolicy.ScalePositive(
                    _radius,
                    ownerScale.Y,
                    partScale.Y),
                ColliderScalePolicy.ScalePositive(
                    _radius,
                    ownerScale.Z,
                    partScale.Z)));
    }

    private ColliderShapeSnapshot CreateStandaloneSnapshot(
        IMatterAgent agent,
        bool useRequestedPose,
        Vector3d requestedPosition,
        FixedQuaternion requestedRotation)
    {
        Vector3d ownerScale;
        Vector3d center;
        FixedQuaternion rotation;
        if (useRequestedPose)
        {
            ownerScale = ColliderScalePolicy.CaptureScale(agent.Transform);
            rotation = requestedRotation;
            SwiftThrowHelper.ThrowIfArgument(
                !rotation.TryTransformScaledPoint(
                    requestedPosition,
                    _offset,
                    ownerScale,
                    out center),
                nameof(requestedPosition),
                "Collider center must be representable after applying the requested body pose.");
        }
        else
        {
            ownerScale = ColliderScalePolicy.Capture(
                agent.Transform,
                out Fixed4x4 worldMatrix,
                out FixedQuaternion matrixRotation);
            rotation = matrixRotation;
            SwiftThrowHelper.ThrowIfArgument(
                !Fixed4x4.TryTransformAffinePoint(worldMatrix, _offset, out center),
                nameof(agent),
                "Collider center must be representable after applying the host transform.");
        }

        return new ColliderShapeSnapshot(
            center,
            rotation,
            ownerScale,
            Vector3d.One,
            _offset,
            _size,
            _radius);
    }

    private ColliderShapeSnapshot CaptureShapeSnapshot()
    {
        IMatterAgent agent = _agent!;
        return _body == null
            ? CreateStandaloneSnapshot(
                agent,
                useRequestedPose: false,
                default,
                default)
            : CreateStandaloneSnapshot(
                agent,
                useRequestedPose: true,
                _body.Position3d,
                _body.Rotation);
    }

    private ColliderShapeSnapshot CreateCompoundPartSnapshot(
        in ColliderShapeSnapshot ownerSnapshot,
        FixedQuaternion localRotation,
        Vector3d localScale)
    {
        ColliderScalePolicy.Validate(localScale);
        SwiftThrowHelper.ThrowIfArgument(
            !ownerSnapshot.Rotation.TryTransformScaledPoint(
                ownerSnapshot.Center,
                _offset,
                ownerSnapshot.OwnerScale,
                out Vector3d center),
            nameof(ownerSnapshot),
            "Compound collider center must be representable after applying its scaled local offset.");
        FixedQuaternion rotation = (ownerSnapshot.Rotation * localRotation).Normalized;
        return new ColliderShapeSnapshot(
            center,
            rotation,
            ownerSnapshot.OwnerScale,
            localScale,
            _offset,
            _size,
            _radius);
    }

    private void PrepareRuntimeShape(
        in ColliderShapeSnapshot snapshot,
        bool requireRepresentableMassPoint)
    {
        _preparedContext ??= _context;
        ValidateShapeCandidate(snapshot);
        _preparedSnapshot = snapshot;
        PrepareShape(snapshot);
        if (requireRepresentableMassPoint
            && !CalculatePreparedLocalMassPoint().TryGetPoint(out _))
        {
            throw new System.InvalidOperationException(
                "The collider's body-local center of mass is outside the Fixed64 coordinate domain.");
        }
    }

    private void PublishPreparedShape()
    {
        _committedCenter = _preparedSnapshot.Center;
        _committedRotation = _preparedSnapshot.Rotation;
        _committedOwnerScale = _preparedSnapshot.OwnerScale;
        _committedPartScale = _preparedSnapshot.PartScale;
        _bounds = _preparedBounds;
        PublishShape();
        _canonicalCenteredProxyRadius =
            ColliderCanonicalBounds.GetCenteredProxyRadius(this);
        _canonicalGroundProbeRadius =
            ColliderCanonicalBounds.GetGroundProbeRadius(this);
        _runtimeShapeState.Commit(_preparedSnapshot);
        _hasCommittedShape = true;
    }

    internal bool TryPrepareBodyPose(
        Vector3d position,
        FixedQuaternion rotation)
    {
        try
        {
            ColliderShapeSnapshot snapshot = CreateStandaloneSnapshot(
                _agent!,
                useRequestedPose: true,
                position,
                rotation);
            PrepareRuntimeShape(
                snapshot,
                requireRepresentableMassPoint: true);
            return true;
        }
        catch (System.ArgumentException)
        {
            return false;
        }
        catch (System.InvalidOperationException)
        {
            return false;
        }
    }

    internal void PublishPreparedBodyPose()
    {
        PublishPreparedShape();
    }

    internal void PublishPreparedExplicitBodyPose()
    {
        PublishPreparedShape();
        _body!.RefreshMassPropertiesFromColliderShape();
    }

    private void PrepareStandaloneInitialization(
        IMatterAgent agent,
        bool useRequestedPose,
        Vector3d requestedPosition,
        FixedQuaternion requestedRotation)
    {
        ColliderShapeSnapshot snapshot = CreateStandaloneSnapshot(
            agent,
            useRequestedPose,
            requestedPosition,
            requestedRotation);
        _preparedContext = agent.Context;
        PrepareRuntimeShape(
            snapshot,
            requireRepresentableMassPoint: useRequestedPose);
    }

    internal void PrepareCompoundPart(
        in ColliderShapeSnapshot ownerSnapshot,
        FixedQuaternion localRotation,
        Vector3d localScale,
        GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfArgument(
            _context != null && !ReferenceEquals(_context, context),
            nameof(context),
            "Compound collider part is already bound to a different GravitasWorldContext.");
        ColliderShapeSnapshot snapshot = CreateCompoundPartSnapshot(
            ownerSnapshot,
            localRotation,
            localScale);
        _preparedContext = context;
        _preparedCompoundLocalRotation = localRotation;
        PrepareRuntimeShape(
            snapshot,
            requireRepresentableMassPoint: false);
    }

    internal void PublishCompoundPart(
        FixedQuaternion localRotation,
        Vector3d localScale,
        GravitasWorldContext context)
    {
        _compoundLocalRotation = localRotation;
        _compoundLocalScale = localScale;
        _context = context;
        PublishPreparedShape();
    }

    /// <summary>Sets the bounds for the shape transaction being prepared.</summary>
    protected void SetPreparedBounds(FixedBoundBox bounds) =>
        _preparedBounds = bounds;

    private protected virtual void ValidateShapeCandidate(in ColliderShapeSnapshot snapshot) { }

    private protected abstract void PrepareShape(in ColliderShapeSnapshot snapshot);

    private protected abstract void PublishShape();
}
