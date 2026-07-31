//=======================================================================
// LSCollider2D.ShapeTransaction.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.Colliders;

public abstract partial class LSCollider2D
{
    private Vector2d _committedCenter;
    private Fixed64 _committedRotation;
    private Vector2d _committedOwnerScale = Vector2d.One;
    private Vector2d _committedPartScale = Vector2d.One;
    private bool _hasCommittedShape;

    protected bool HasCommittedShape => _hasCommittedShape;

    private ColliderShapeSnapshot2D _preparedSnapshot;
    private FixedBoundArea _preparedBounds;
    private FixedBoundBox _preparedMixedBounds;
    private Fixed64 _canonicalCenteredProxyRadius = Fixed64.MaxValue;
    private Fixed64 _canonicalGroundProbeRadius;
    private GravitasWorldContext? _preparedContext;
    private Fixed64 _preparedCompoundLocalRotation;

    private protected GravitasWorldContext PreparedContext => _preparedContext!;

    internal FixedBoundArea PreparedShapeBounds => _preparedBounds;

    internal Fixed64 CanonicalCenteredProxyRadius =>
        _canonicalCenteredProxyRadius;

    internal Fixed64 CanonicalGroundProbeRadius =>
        _canonicalGroundProbeRadius;

    internal Vector2d CanonicalCenter => _committedCenter;

    internal bool TryGetCommittedOwnerScale(out Vector2d scale)
    {
        if (_hasCommittedShape)
        {
            scale = _committedOwnerScale;
            return true;
        }

        scale = default;
        return false;
    }

    private Vector2d GetCurrentOwnerScale()
    {
        if (_hasCommittedShape)
            return _committedOwnerScale;
        if (_compoundOwner != null)
            return _compoundOwner.GetCurrentOwnerScale();
        if (_agent == null)
            return Vector2d.One;

        return ColliderScalePolicy.CapturePlanar(
            _agent.Transform,
            out _,
            out _);
    }

    protected void GetCurrentScaleFactors(
        out Vector2d ownerScale,
        out Vector2d partScale)
    {
        ownerScale = GetCurrentOwnerScale();
        partScale = _hasCommittedShape
            ? _committedPartScale
            : _compoundOwner != null
                ? _compoundLocalScale
                : Vector2d.One;
    }

    internal bool TryGetCurrentScaledOffset(out Vector2d scaledOffset)
    {
        Vector2d ownerScale = GetCurrentOwnerScale();
        bool representable = Fixed64.TryMultiplyDivide(
                _localOffset.X,
                ownerScale.X,
                Fixed64.One,
                out Fixed64 x)
            & Fixed64.TryMultiplyDivide(
                _localOffset.Y,
                ownerScale.Y,
                Fixed64.One,
                out Fixed64 y);
        scaledOffset = representable ? new Vector2d(x, y) : default;
        return representable;
    }

    internal Vector2d GetCurrentScaledOffset()
    {
        bool representable =
            TryGetCurrentScaledOffset(out Vector2d scaledOffset);
        SwiftThrowHelper.ThrowIfTrue(
            !representable,
            nameof(ScaledLocalOffset),
            "The scaled 2D collider offset is outside the Fixed64 coordinate domain.");
        return scaledOffset;
    }

    private Vector2d GetUncommittedCenter()
    {
        Vector2d ownerScale = GetCurrentOwnerScale();
        bool representable = Vector2d.TryTransformScaledPoint(
            Position,
            _localOffset,
            ownerScale,
            _compoundOwner?.Rotation ?? Rotation,
            out Vector2d center);
        SwiftThrowHelper.ThrowIfTrue(
            !representable,
            nameof(Center),
            "The 2D collider center is outside the Fixed64 coordinate domain.");
        return center;
    }

    private ColliderShapeSnapshot2D CreateStandaloneSnapshot(
        IMatterAgent agent,
        bool useRequestedPose,
        Vector2d requestedPosition,
        Fixed64 requestedRotation)
    {
        Vector2d ownerScale = ColliderScalePolicy.CapturePlanar(
            agent.Transform,
            out Fixed4x4 worldMatrix,
            out Fixed64 matrixRotation);
        Vector2d center;
        Fixed64 rotation;
        if (useRequestedPose)
        {
            rotation = PlanarRotation.Canonicalize(requestedRotation);
            bool representable = Vector2d.TryTransformScaledPoint(
                requestedPosition,
                _localOffset,
                ownerScale,
                rotation,
                out center);
            SwiftThrowHelper.ThrowIfArgument(
                !representable,
                nameof(requestedPosition),
                "2D collider center must be representable after applying the requested body pose.");
        }
        else
        {
            rotation = matrixRotation;
            SwiftThrowHelper.ThrowIfArgument(
                !Fixed4x4.TryTransformAffinePoint(
                    worldMatrix,
                    _localOffset.ToVector3d(Fixed64.Zero),
                    out Vector3d transformed),
                nameof(agent),
                "2D collider center must be representable after applying the host transform.");
            center = transformed.ToVector2d();
        }

        return new ColliderShapeSnapshot2D(
            center,
            rotation,
            ownerScale,
            Vector2d.One,
            _localOffset,
            _shapeVersion,
            worldMatrix.M42,
            _mixedHalfThicknessOverride ?? agent.Context.Settings.Mixed2DHalfThickness);
    }

    private ColliderShapeSnapshot2D CaptureShapeSnapshot()
    {
        if (_compoundOwner != null)
        {
            return CreateCompoundPartSnapshot(
                _compoundOwner._preparedSnapshot,
                _compoundLocalRotation,
                _compoundLocalScale);
        }

        IMatterAgent? agent = _agent;
        if (agent == null)
        {
            Vector2d ownerScale = Vector2d.One;
            Fixed64 rotation = PlanarRotation.Canonicalize(
                ResolveStandaloneRotation());
            _ = Vector2d.TryTransformScaledPoint(
                ResolveStandalonePosition(),
                _localOffset,
                ownerScale,
                rotation,
                out Vector2d center);
            return new ColliderShapeSnapshot2D(
                center,
                rotation,
                ownerScale,
                Vector2d.One,
                _localOffset,
                _shapeVersion,
                Fixed64.Zero,
                _mixedHalfThicknessOverride ?? PhysicsSettings.DefaultMixed2DHalfThickness);
        }

        return _body == null
            ? CreateStandaloneSnapshot(
                agent,
                useRequestedPose: false,
                default,
                default)
            : CreateStandaloneSnapshot(
                agent,
                useRequestedPose: true,
                _body.Position,
                _body.Rotation);
    }

    private ColliderShapeSnapshot2D CreateCompoundPartSnapshot(
        in ColliderShapeSnapshot2D ownerSnapshot,
        Fixed64 localRotation,
        Vector2d localScale)
    {
        ColliderScalePolicy.Validate(localScale);
        bool representable = Vector2d.TryTransformScaledPoint(
            ownerSnapshot.Center,
            _localOffset,
            ownerSnapshot.OwnerScale,
            ownerSnapshot.Rotation,
            out Vector2d center);
        SwiftThrowHelper.ThrowIfArgument(
            !representable,
            nameof(_localOffset),
            "Compound-part center must be representable in the owner's world frame.");
        return new ColliderShapeSnapshot2D(
            center,
            PlanarRotation.Combine(ownerSnapshot.Rotation, localRotation),
            ownerSnapshot.OwnerScale,
            localScale,
            _localOffset,
            _shapeVersion,
            ownerSnapshot.MixedSlabCenterY,
            ownerSnapshot.MixedHalfThickness);
    }

    private void PrepareRuntimeShape(
        in ColliderShapeSnapshot2D snapshot,
        bool requireRepresentableMassPoint)
    {
        _preparedContext ??= _context;
        _preparedSnapshot = snapshot;
        PrepareShape(snapshot);
        if (requireRepresentableMassPoint
            && !CalculatePreparedLocalMassPoint().TryGetPoint(out _))
        {
            throw new System.InvalidOperationException(
                "The 2D collider's body-local center of mass is outside the Fixed64 coordinate domain.");
        }
        Vector2d min = _preparedBounds.Min;
        Vector2d max = _preparedBounds.Max;
        _preparedMixedBounds = FixedBoundBox.FromMinMax(
            new Vector3d(
                min.X,
                snapshot.MixedSlabCenterY - snapshot.MixedHalfThickness,
                min.Y),
            new Vector3d(
                max.X,
                snapshot.MixedSlabCenterY + snapshot.MixedHalfThickness,
                max.Y));
    }

    private void PublishPreparedShape()
    {
        _committedCenter = _preparedSnapshot.Center;
        _committedRotation = _preparedSnapshot.Rotation;
        _committedOwnerScale = _preparedSnapshot.OwnerScale;
        _committedPartScale = _preparedSnapshot.PartScale;
        _bounds = _preparedBounds;
        _mixedBounds3D = _preparedMixedBounds;
        _mixedSlabCenterY = _preparedSnapshot.MixedSlabCenterY;
        _mixedHalfThickness = _preparedSnapshot.MixedHalfThickness;
        PublishShape();
        _canonicalCenteredProxyRadius =
            ColliderCanonicalBounds2D.GetCenteredProxyRadius(this);
        _canonicalGroundProbeRadius =
            ColliderCanonicalBounds2D.GetGroundProbeRadius(this);
        _runtimeShapeState.Commit(_preparedSnapshot);
        _hasCommittedShape = true;
    }

    internal bool TryPrepareBodyPose(
        Vector2d position,
        Fixed64 rotation)
    {
        try
        {
            ColliderShapeSnapshot2D snapshot = _agent != null
                ? CreateStandaloneSnapshot(
                    _agent,
                    useRequestedPose: true,
                    position,
                    rotation)
                : CreateDetachedBodySnapshot(position, rotation);
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

    private ColliderShapeSnapshot2D CreateDetachedBodySnapshot(
        Vector2d position,
        Fixed64 rotation)
    {
        bool representable = Vector2d.TryTransformScaledPoint(
            position,
            _localOffset,
            _committedOwnerScale,
            rotation,
            out Vector2d center);
        SwiftThrowHelper.ThrowIfArgument(
            !representable,
            nameof(position),
            "2D collider center must be representable after applying the detached body pose.");
        return new ColliderShapeSnapshot2D(
            center,
            rotation,
            _committedOwnerScale,
            Vector2d.One,
            _localOffset,
            _shapeVersion,
            _mixedSlabCenterY,
            _mixedHalfThickness);
    }

    internal void PublishPreparedBodyPose()
    {
        PublishPreparedShape();
    }

    internal void PublishPreparedExplicitBodyPose()
    {
        PublishPreparedShape();
        _body?.RefreshMassPropertiesFromColliderShape();
        if (_id >= 0)
            _context!.Collisions2D.RefreshColliderPartitionAfterShapeChange(this);
    }

    private void PrepareStandaloneInitialization(
        IMatterAgent agent,
        bool useRequestedPose,
        Vector2d requestedPosition,
        Fixed64 requestedRotation)
    {
        ColliderShapeSnapshot2D snapshot = CreateStandaloneSnapshot(
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
        in ColliderShapeSnapshot2D ownerSnapshot,
        Fixed64 localRotation,
        Vector2d localScale,
        GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfArgument(
            _context != null && !ReferenceEquals(_context, context),
            nameof(context),
            "2D compound collider part is already bound to a different GravitasWorldContext.");
        ColliderShapeSnapshot2D snapshot = CreateCompoundPartSnapshot(
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
        Fixed64 localRotation,
        Vector2d localScale,
        GravitasWorldContext context)
    {
        _compoundLocalRotation = localRotation;
        _compoundLocalScale = localScale;
        _context = context;
        PublishPreparedShape();
    }

    protected void SetPreparedBounds(FixedBoundArea bounds) =>
        _preparedBounds = bounds;

    private protected abstract void PrepareShape(in ColliderShapeSnapshot2D snapshot);

    private protected abstract void PublishShape();
}
