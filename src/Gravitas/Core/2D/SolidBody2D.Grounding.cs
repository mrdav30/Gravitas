//=======================================================================
// SolidBody2D.Grounding.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas;

public sealed partial class SolidBody2D
{
    private static readonly Vector2d DefaultGroundUpDirection = Vector2d.Forward;

    private GroundingMode _groundingMode = GroundingMode.Automatic;
    private GroundProbeMode2D _groundProbeMode = GroundProbeMode2D.Auto;
    private bool _useGravityDerivedGroundUpDirection = true;
    private Vector2d _groundUpDirection = DefaultGroundUpDirection;
    private Fixed64 _groundProbeRadius;
    private int _lastGroundCheckFrame = int.MinValue;
    private const int GroundCheckFrameThreshold = 10;
    private readonly Fixed64 _groundCheckPositionThreshold = Fixed64.FromFraction(1, 100);
    private readonly SwiftList<Physics2DHit> _groundProbeHits = new();

    private bool _isGrounded;
    private bool _wasGrounded;
    private bool _groundedTransitionCapturedForStep;
    private Vector2d _groundNormal;
    private Vector2d _groundPoint;
    private Vector2d _lastGroundedPosition;
    private LSCollider2D? _groundCollider;
    private uint _groundColliderBroadPhaseVersion;

    private bool _hasGroundContactCandidate;
    private Fixed64 _groundContactCandidateUpDot;
    private Fixed64 _groundContactCandidateDepth;
    private int _groundContactCandidateColliderId;
    private ulong _groundContactCandidateId;
    private Vector2d _groundContactCandidateNormal;
    private Vector2d _groundContactCandidatePoint;
    private LSCollider2D? _groundContactCandidateCollider;

    /// <summary>
    /// Selects whether Gravitas owns planar support detection or preserves host-supplied state.
    /// </summary>
    public GroundingMode GroundingMode => _groundingMode;

    /// <summary>
    /// Selects the pure 2D query primitive used when probing for planar support.
    /// </summary>
    public GroundProbeMode2D GroundProbeMode
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _groundProbeMode;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _groundProbeMode = value;
    }

    /// <summary>
    /// Gets or sets whether non-zero planar gravity defines the support up direction.
    /// </summary>
    public bool UseGravityDerivedGroundUpDirection
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _useGravityDerivedGroundUpDirection;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _useGravityDerivedGroundUpDirection = value;
    }

    /// <summary>
    /// Gets or sets the fallback support up direction in the X/Z simulation plane.
    /// </summary>
    public Vector2d GroundUpDirection
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _groundUpDirection;
        set
        {
            SwiftThrowHelper.ThrowIfArgument(
                value.MagnitudeSquared <= Fixed64.Epsilon,
                nameof(value),
                "2D ground up direction must be non-zero.");
            _groundUpDirection = value.Normalized;
        }
    }

    /// <summary>
    /// Optional explicit radius for swept-circle support probes. A zero value derives the radius from the collider.
    /// </summary>
    public Fixed64 GroundProbeRadius
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _groundProbeRadius;
        set
        {
            SwiftThrowHelper.ThrowIfArgument(value < Fixed64.Zero, nameof(value), "2D ground probe radius cannot be negative.");
            _groundProbeRadius = value;
        }
    }

    /// <summary>
    /// Probe distance used while the body is already grounded.
    /// </summary>
    public Fixed64 GroundedDistanceRay { get; set; } = Fixed64.Half;

    /// <summary>
    /// Probe distance used while the body is airborne.
    /// </summary>
    public Fixed64 GroundDownDistanceOnAir { get; set; } = Fixed64.Half;

    /// <summary>
    /// Minimum dot product between a candidate support normal and resolved up direction.
    /// </summary>
    public Fixed64 GroundMinNormalDot { get; set; } = Fixed64.Half;

    /// <summary>
    /// Gets whether this body currently has planar support.
    /// </summary>
    public bool IsGrounded
    {
        get => _isGrounded;
        private set
        {
            if (_isGrounded == value)
                return;

            _isGrounded = value;
            OnGrounded?.Invoke(value);
        }
    }

    /// <summary>
    /// Gets whether this body was grounded before the latest authoritative grounding refresh or manual grounding change.
    /// </summary>
    public bool WasGrounded => _wasGrounded;

    /// <summary>
    /// Gets the current support normal in the X/Z simulation plane.
    /// </summary>
    public Vector2d GroundNormal => _groundNormal;

    /// <summary>
    /// Gets the current support point in the X/Z simulation plane.
    /// </summary>
    public Vector2d GroundPoint => _groundPoint;

    /// <summary>
    /// Gets the body position captured when the current support state was accepted.
    /// </summary>
    public Vector2d LastGroundedPosition => _lastGroundedPosition;

    /// <summary>
    /// Raised when grounded state changes.
    /// </summary>
    public Action<bool>? OnGrounded;

    /// <summary>
    /// Gives the host ownership of grounded state and disables automatic support refresh.
    /// </summary>
    public void UseManualGrounding(bool clearGrounding = true)
    {
        _groundingMode = GroundingMode.Manual;
        if (!clearGrounding)
            return;

        CaptureGroundedTransitionState();
        ClearGrounding();
    }

    /// <summary>
    /// Returns grounded-state ownership to Gravitas and optionally refreshes support immediately.
    /// </summary>
    public void UseAutomaticGrounding(bool checkGroundImmediately = true)
    {
        _groundingMode = GroundingMode.Automatic;
        if (checkGroundImmediately && Active)
        {
            CaptureGroundedTransitionState();
            CheckGround(force: true);
        }
    }

    /// <summary>
    /// Sets host-owned planar grounded state for manual support sources such as deterministic tilemaps.
    /// </summary>
    public void SetManualGrounding(Vector2d groundPoint, Vector2d groundNormal)
    {
        SwiftThrowHelper.ThrowIfArgument(
            groundNormal.MagnitudeSquared <= Fixed64.Epsilon,
            nameof(groundNormal),
            "Manual 2D grounding normal must be non-zero.");

        _groundingMode = GroundingMode.Manual;
        CaptureGroundedTransitionState();
        SetGroundingState(groundPoint, groundNormal.Normalized, null);
    }

    /// <summary>
    /// Clears host-owned planar grounded state while leaving automatic support disabled.
    /// </summary>
    public void ClearManualGrounding()
    {
        _groundingMode = GroundingMode.Manual;
        CaptureGroundedTransitionState();
        ClearGrounding();
    }

    /// <summary>
    /// Forces an immediate automatic support probe when Gravitas owns grounding.
    /// </summary>
    public void CheckGround()
    {
        CaptureGroundedTransitionState();
        CheckGround(force: true);
    }

    internal void BeginAutomaticGroundingRefresh()
    {
        if (!CanUseAutomaticGrounding)
        {
            ClearGroundingForAutomaticRefresh();
            return;
        }

        CaptureGroundedStepState();
        ClearGroundContactCandidate();
    }

    internal void TryAcceptContactGroundCandidate(
        LSCollider2D ownCollider,
        LSCollider2D otherCollider,
        ManifoldContact2D contact,
        bool ownColliderIsA)
    {
        if (!CanUseAutomaticGrounding || !ReferenceEquals(ownCollider, Collider))
            return;

        if (!IsValidGroundCollider(otherCollider))
            return;

        Vector2d normal = ownColliderIsA ? -contact.Normal : contact.Normal;
        if (normal.MagnitudeSquared <= Fixed64.Epsilon)
            return;

        normal = normal.Normalized;
        Fixed64 upDot = Vector2d.Dot(normal, ResolveGroundUpDirection());
        if (upDot < GroundMinNormalDot)
            return;

        Vector2d point = ownColliderIsA ? contact.PointA : contact.PointB;
        int otherId = otherCollider.Id;
        if (_hasGroundContactCandidate)
        {
            int compare = CompareGroundContactCandidate(
                upDot,
                contact.Depth,
                otherId,
                contact.ContactId);
            if (compare >= 0)
                return;
        }

        _hasGroundContactCandidate = true;
        _groundContactCandidateUpDot = upDot;
        _groundContactCandidateDepth = contact.Depth;
        _groundContactCandidateColliderId = otherId;
        _groundContactCandidateId = contact.ContactId;
        _groundContactCandidateNormal = normal;
        _groundContactCandidatePoint = point;
        _groundContactCandidateCollider = otherCollider;
    }

    internal void CompleteAutomaticGroundingRefresh()
    {
        if (!CanUseAutomaticGrounding)
        {
            CompleteGroundedStepState();
            return;
        }

        if (_hasGroundContactCandidate)
        {
            SetGroundingState(
                _groundContactCandidatePoint,
                _groundContactCandidateNormal,
                _groundContactCandidateCollider);
            ClearGroundContactCandidate();
            CompleteGroundedStepState();
            return;
        }

        CheckGround(force: false);
        CompleteGroundedStepState();
    }

    internal void CheckGroundForSimulation() => CheckGround(force: false);

    internal Vector2d RemoveIntoGroundComponent(Vector2d value)
    {
        value = ProjectLinearMotion(value);
        if (!_isGrounded || _groundNormal.MagnitudeSquared <= Fixed64.Epsilon)
            return value;

        Fixed64 intoGround = Vector2d.Dot(value, _groundNormal);
        return intoGround < Fixed64.Zero
            ? ProjectLinearMotion(value - _groundNormal * intoGround)
            : value;
    }

    private bool CanUseAutomaticGrounding =>
        Active && CanTranslate && _groundingMode == GroundingMode.Automatic;

    private void CheckGround(bool force)
    {
        if (_groundingMode == GroundingMode.Manual)
            return;

        if (!CanUseAutomaticGrounding)
        {
            ClearGrounding();
            return;
        }

        LSCollider2D? cachedGroundCollider = _groundCollider;
        bool frameGuard = !force
            && _isGrounded
            && cachedGroundCollider != null
            && IsValidGroundCollider(cachedGroundCollider)
            && cachedGroundCollider.BroadPhaseVersion == _groundColliderBroadPhaseVersion
            && Vector2d.Distance(_lastGroundedPosition, _position) < _groundCheckPositionThreshold
            && Context.FrameCount - _lastGroundCheckFrame < GroundCheckFrameThreshold;
        if (frameGuard)
            return;

        _lastGroundCheckFrame = Context.FrameCount;
        Vector2d up = ResolveGroundUpDirection();
        Vector2d down = -up;
        Fixed64 distance = _isGrounded ? GroundedDistanceRay : GroundDownDistanceOnAir;
        if (distance <= Fixed64.Zero)
        {
            ClearGrounding();
            return;
        }

        GroundProbeMode2D mode = ResolveGroundProbeMode();
        Fixed64 radius = mode == GroundProbeMode2D.SweptCircle
            ? ResolveGroundProbeRadius()
            : Fixed64.Zero;
        Vector2d start = _position;
        Vector2d end = start + down * distance;
        bool foundGround = TryFindGroundHit(mode, radius, start, end, out Physics2DHit hit);
        Context.Diagnostics.EmitGroundProbe(this, mode, start, end, radius, foundGround, hit);

        if (!foundGround)
        {
            ClearGrounding();
            return;
        }

        SetGroundingState(hit.Point, hit.Normal.Normalized, hit.Collider);
    }

    private bool TryFindGroundHit(
        GroundProbeMode2D mode,
        Fixed64 radius,
        Vector2d start,
        Vector2d end,
        out Physics2DHit hit)
    {
        if (mode == GroundProbeMode2D.SweptCircle && radius > Fixed64.Epsilon)
            return TryFindGroundHitWithSweptCircle(start, end, radius, out hit);

        return TryFindGroundHitWithRay(start, end, out hit);
    }

    private bool TryFindGroundHitWithRay(Vector2d start, Vector2d end, out Physics2DHit hit)
    {
        int hitCount = Context.Query2D.RaycastAll(start, end, Context.Settings.GroundCheckLayerMask, _groundProbeHits);
        for (int i = 0; i < hitCount; i++)
        {
            Physics2DHit current = _groundProbeHits[i];
            if (!IsValidGroundHit(current))
                continue;

            hit = current;
            return true;
        }

        hit = default;
        return false;
    }

    private bool TryFindGroundHitWithSweptCircle(Vector2d start, Vector2d end, Fixed64 radius, out Physics2DHit hit)
    {
        int hitCount = Context.Query2D.SweepCircleAgainstStaticAll(
            start,
            end,
            radius,
            Context.Settings.GroundCheckLayerMask,
            _groundProbeHits,
            Collider,
            includeTriggers: false);

        for (int i = 0; i < hitCount; i++)
        {
            Physics2DHit current = _groundProbeHits[i];
            if (!IsValidGroundHit(current))
                continue;

            hit = current;
            return true;
        }

        hit = default;
        return false;
    }

    private bool IsValidGroundHit(Physics2DHit hit)
    {
        LSCollider2D? hitCollider = hit.Collider;
        if (hitCollider == null || ReferenceEquals(hitCollider, Collider))
            return false;

        if (!IsValidGroundCollider(hitCollider))
            return false;

        Vector2d up = ResolveGroundUpDirection();
        Vector2d normal = hit.Normal.MagnitudeSquared > Fixed64.Epsilon
            ? hit.Normal.Normalized
            : up;
        return Vector2d.Dot(normal, up) >= GroundMinNormalDot;
    }

    private bool IsValidGroundCollider(LSCollider2D collider)
    {
        if (!collider.IsActive
            || collider.IsTrigger
            || ReferenceEquals(collider, Collider)
            || !Context.Settings.GroundCheckLayerMask.Includes(collider.Layer)
            || !ColliderCollisionFilter.AllowsPhysicalPair(Collider, collider))
        {
            return false;
        }

        SolidBody2D? body = collider.Body;
        return body == null || body.IsPositionFullyFrozen || body.IsKinematic;
    }

    private GroundProbeMode2D ResolveGroundProbeMode()
    {
        if (_groundProbeMode != GroundProbeMode2D.Auto)
            return _groundProbeMode;

        return ResolveGroundProbeRadius() > Fixed64.Epsilon
            ? GroundProbeMode2D.SweptCircle
            : GroundProbeMode2D.Ray;
    }

    private Fixed64 ResolveGroundProbeRadius()
    {
        if (_groundProbeRadius > Fixed64.Zero)
            return _groundProbeRadius;

        return Collider switch
        {
            LSCircleCollider2D circle => circle.ScaledRadius,
            LSCapsuleCollider2D capsule => capsule.ScaledRadius,
            LSAABBoxCollider2D box => FixedMath.Min(box.ScaledSize.X, box.ScaledSize.Y) * Fixed64.Half,
            LSPolygonCollider2D polygon => FixedMath.Min(polygon.Bounds.Width, polygon.Bounds.Height) * Fixed64.Half,
            LSCompoundCollider2D compound => FixedMath.Min(compound.Bounds.Width, compound.Bounds.Height) * Fixed64.Half,
            _ => Fixed64.Zero
        };
    }

    private Vector2d ResolveGroundUpDirection()
    {
        if (_useGravityDerivedGroundUpDirection && Gravity.MagnitudeSquared > Fixed64.Epsilon)
            return (-Gravity).Normalized;

        return _groundUpDirection;
    }

    private void CaptureGroundedTransitionState()
    {
        _wasGrounded = _isGrounded;
        _groundedTransitionCapturedForStep = true;
    }

    private void CaptureGroundedStepState()
    {
        if (!_groundedTransitionCapturedForStep)
            CaptureGroundedTransitionState();
    }

    private void CompleteGroundedStepState() => _groundedTransitionCapturedForStep = false;

    private void ClearGrounding()
    {
        IsGrounded = false;
        _groundNormal = Vector2d.Zero;
        _groundPoint = Vector2d.Zero;
        _groundCollider = null;
        _groundColliderBroadPhaseVersion = 0;
    }

    private void ClearGroundingForAutomaticRefresh()
    {
        if (_groundingMode != GroundingMode.Automatic)
            return;

        CaptureGroundedStepState();
        ClearGrounding();
        CompleteGroundedStepState();
    }

    private void SetGroundingState(Vector2d groundPoint, Vector2d groundNormal, LSCollider2D? groundCollider)
    {
        _groundPoint = groundPoint;
        _groundNormal = groundNormal.MagnitudeSquared > Fixed64.Epsilon
            ? groundNormal.Normalized
            : ResolveGroundUpDirection();
        _lastGroundedPosition = _position;
        _groundCollider = groundCollider;
        _groundColliderBroadPhaseVersion = groundCollider?.BroadPhaseVersion ?? 0;
        IsGrounded = true;
    }

    private void ResetGroundingForInitialize(Vector2d position)
    {
        _isGrounded = false;
        _wasGrounded = false;
        _groundedTransitionCapturedForStep = false;
        _groundNormal = Vector2d.Zero;
        _groundPoint = Vector2d.Zero;
        _lastGroundedPosition = position;
        _groundCollider = null;
        _groundColliderBroadPhaseVersion = 0;
        _lastGroundCheckFrame = int.MinValue;
        ClearGroundContactCandidate();
    }

    private void ClearGroundContactCandidate()
    {
        _hasGroundContactCandidate = false;
        _groundContactCandidateUpDot = Fixed64.Zero;
        _groundContactCandidateDepth = Fixed64.Zero;
        _groundContactCandidateColliderId = int.MaxValue;
        _groundContactCandidateId = ulong.MaxValue;
        _groundContactCandidateNormal = Vector2d.Zero;
        _groundContactCandidatePoint = Vector2d.Zero;
        _groundContactCandidateCollider = null;
    }

    private int CompareGroundContactCandidate(
        Fixed64 upDot,
        Fixed64 depth,
        int colliderId,
        ulong contactId)
    {
        if (upDot != _groundContactCandidateUpDot)
            return upDot > _groundContactCandidateUpDot ? -1 : 1;

        if (depth != _groundContactCandidateDepth)
            return depth > _groundContactCandidateDepth ? -1 : 1;

        if (colliderId != _groundContactCandidateColliderId)
            return colliderId < _groundContactCandidateColliderId ? -1 : 1;

        if (contactId != _groundContactCandidateId)
            return contactId < _groundContactCandidateId ? -1 : 1;

        return 0;
    }
}
