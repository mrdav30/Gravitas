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
    private Fixed64 _canonicalPivotProxyRadius = Fixed64.MaxValue;
    private Fixed64 _preparedAttachmentReach;
    private Fixed64 _preparedGeometryReach = Fixed64.MaxValue;

    internal Fixed64 PreparedGeometryReach => _preparedGeometryReach;
    internal Fixed64 PreparedAttachmentReach => _preparedAttachmentReach;
    private Fixed64 _canonicalGroundProbeRadius;
    private GravitasWorldContext? _preparedContext;
    private FixedQuaternion _preparedCompoundLocalRotation = FixedQuaternion.Identity;

    private protected GravitasWorldContext PreparedContext => _preparedContext!;

    internal FixedBoundBox PreparedShapeBounds => _preparedBounds;

    internal Fixed64 CanonicalCenteredProxyRadius =>
        _canonicalCenteredProxyRadius;

    internal Fixed64 CanonicalPivotProxyRadius => _canonicalPivotProxyRadius;

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
        bool includeBodyAttachment)
    {
        _preparedContext ??= _context;
        ValidateShapeCandidate(snapshot);
        _preparedSnapshot = snapshot;
        _preparedGeometryReach = Fixed64.MaxValue;
        PrepareShape(snapshot);
        _preparedAttachmentReach = ColliderGeometryPolicy.Attachment(snapshot.LocalOffset, snapshot.OwnerScale);
        Fixed64 completeReach = _preparedGeometryReach;
        if (includeBodyAttachment)
            completeReach = ColliderGeometryPolicy.Add(
                _preparedAttachmentReach, completeReach);
        ColliderGeometryPolicy.ValidateReach(completeReach, nameof(snapshot));
    }

    // Broad-phase geometry also advances for accepted dynamic solver poses; those are not new contact stimuli.
    internal uint ContactDependencyVersion { get; private set; }

    private void MarkContactDependencyChanged()
    {
        LSCollider owner = this;
        while (owner._compoundOwner != null)
            owner = owner._compoundOwner;
        owner.ContactDependencyVersion++;
    }

    private void PublishPreparedShape(bool contactDependencyChanged = true)
    {
        _committedCenter = _preparedSnapshot.Center;
        _committedRotation = _preparedSnapshot.Rotation;
        _committedOwnerScale = _preparedSnapshot.OwnerScale;
        _committedPartScale = _preparedSnapshot.PartScale;
        _bounds = _preparedBounds;
        PublishShape();
        Fixed64 intrinsic;
        if (this is LSCompoundCollider compound)
            _canonicalCenteredProxyRadius = ColliderCanonicalBounds.GetCompoundCenteredProxyRadius(compound, out intrinsic);
        else
        {
            _canonicalCenteredProxyRadius = _preparedGeometryReach;
            intrinsic = this is LSSphereCollider ? _canonicalCenteredProxyRadius
                : ColliderGeometryPolicy.RotatedReach(_canonicalCenteredProxyRadius, planar: false);
        }
        _canonicalPivotProxyRadius = ColliderGeometryPolicy.Add(
            ColliderGeometryPolicy.RotatedReach(_preparedAttachmentReach, planar: false), intrinsic);
        _canonicalGroundProbeRadius =
            ColliderCanonicalBounds.GetGroundProbeRadius(this);
        _runtimeShapeState.Commit(_preparedSnapshot);
        _hasCommittedShape = true;
        if (contactDependencyChanged)
            ContactDependencyVersion++;
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
                includeBodyAttachment: true);
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
        PublishPreparedShape(contactDependencyChanged: false);
    }

    internal void PublishPreparedExplicitBodyPose(bool poseChanged)
    {
        PublishPreparedShape(contactDependencyChanged: poseChanged || _runtimeShapeState.IsDirty
            || _preparedSnapshot.OwnerScale != _committedOwnerScale);
        _body!.RefreshMassPropertiesFromColliderShape();
    }

    internal void PrepareDetachedBodyReconfigurationCandidate(
        SolidBody body,
        Vector3d localOffset,
        Vector3d position,
        FixedQuaternion rotation)
    {
        _body = body;
        _agent = body.Agent;
        _context = body.Context;
        _offset = localOffset;
        PrepareStandaloneInitialization(
            body.Agent,
            useRequestedPose: true,
            position,
            rotation);
        PublishPreparedShape();
    }

    internal bool MatchesCommittedReconfigurationCandidate(LSCollider candidate) =>
        !_runtimeShapeState.ShouldRebuild(candidate._preparedSnapshot);

    internal void PrepareBodyReconfiguration(
        ColliderShapeDefinition definition,
        Vector3d localOffset,
        Vector3d position,
        FixedQuaternion rotation)
    {
        Vector3d ownerScale = ColliderScalePolicy.CaptureScale(_agent!.Transform);
        SwiftThrowHelper.ThrowIfArgument(
            !rotation.TryTransformScaledPoint(
                position,
                localOffset,
                ownerScale,
                out Vector3d center),
            nameof(position),
            "Collider center must be representable after applying the requested body pose.");
        var snapshot = new ColliderShapeSnapshot(
            center,
            rotation,
            ownerScale,
            Vector3d.One,
            localOffset,
            definition.Size,
            definition.Radius);
        PrepareRuntimeShape(
            snapshot,
            includeBodyAttachment: true);
    }

    internal void PublishPreparedBodyReconfiguration(
        ColliderShapeDefinition definition,
        Vector3d localOffset)
    {
        _offset = localOffset;
        _radius = definition.Radius;
        _size = definition.Size;
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
            includeBodyAttachment: useRequestedPose);
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
            includeBodyAttachment: false);
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

    private protected void SetPreparedGeometryReach(Fixed64 reach) =>
        _preparedGeometryReach = reach;

    /// <summary>Sets the bounds for the shape transaction being prepared.</summary>
    protected void SetPreparedBounds(FixedBoundBox bounds) =>
        _preparedBounds = bounds;

    private protected virtual void ValidateShapeCandidate(in ColliderShapeSnapshot snapshot) { }

    private protected abstract void PrepareShape(in ColliderShapeSnapshot snapshot);

    private protected abstract void PublishShape();
}
