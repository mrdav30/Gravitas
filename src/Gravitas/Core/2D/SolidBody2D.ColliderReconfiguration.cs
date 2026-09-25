// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;
using System;

namespace Gravitas;

/// <content>Owns transactional native 2D collider replacement.</content>
public sealed partial class SolidBody2D
{
    /// <summary>
    /// Attempts to publish replacement native 2D geometry and a body-root pose
    /// without replacing body, collider, or host identity.
    /// </summary>
    /// <remarks>
    /// Supports circle-to-circle and capsule-to-capsule replacement only in
    /// <see cref="PhysicsRuntimeMode.TwoD"/>. The definition contributes geometry;
    /// material, filters, hierarchy, events, rotation, motion, and host world Y
    /// are retained. Exact touching is allowed; any positive penetration,
    /// even below one raw unit, against an eligible registered collider rejects
    /// the change, including outside grid coverage. Triggers do not block it.
    /// This synchronous operation may scan the registry and allocate. It makes
    /// no work, timing, or allocation-failure recovery guarantee.
    /// The publisher runs once after physical publication or confirmation of
    /// unchanged state, and before pair separation. Rejection never invokes it.
    /// Notification failures do not roll back accepted state; pair retirement
    /// continues and failures are returned separately in stable order.
    /// </remarks>
    /// <param name="definition">The complete replacement geometry in the existing family.</param>
    /// <param name="localOffset">The unscaled collider offset from the body root.</param>
    /// <param name="position">The authoritative X/Z body-root position.</param>
    /// <param name="blocker">The first eligible penetrating collider in stable service order.</param>
    /// <param name="notificationException">Post-commit publisher or separation failures, aggregated when multiple.</param>
    /// <param name="publishSynchronizedState">Optional once-only publication of already prepared dependent state.</param>
    /// <returns>Whether the change was applied, already matched, or was blocked.</returns>
    /// <exception cref="ArgumentException">The definition, family, or candidate geometry is invalid.</exception>
    /// <exception cref="InvalidOperationException">The runtime mode, registration, fixed-step boundary, or host pose is unsafe.</exception>
    public ColliderReconfigurationStatus TryReconfigureCollider(
        ColliderShapeDefinition2D definition, Vector2d localOffset, Vector2d position,
        out LSCollider2D? blocker, out Exception? notificationException,
        Action? publishSynchronizedState = null)
    {
        blocker = null;
        notificationException = null;
        definition.EnsureDefined();
        ThrowIfRuntimeRegistrationMissing();
        Context.ThrowIfFixedStepMutationNotAllowed();
        SwiftThrowHelper.ThrowIfTrue(
            Context.Settings.RuntimeMode != PhysicsRuntimeMode.TwoD,
            nameof(Context.Settings.RuntimeMode),
            "Native 2D collider reconfiguration requires pure TwoD runtime mode.");
        ColliderType2D expectedShape = definition.Kind switch
        {
            ColliderShapeDefinition2DKind.Circle => ColliderType2D.Circle,
            ColliderShapeDefinition2DKind.Capsule => ColliderType2D.Capsule,
            _ => ColliderType2D.None
        };
        SwiftThrowHelper.ThrowIfArgument(
            expectedShape == ColliderType2D.None || Collider.Shape != expectedShape,
            nameof(definition),
            "2D collider reconfiguration requires the existing circle or capsule family.");

        FixedTransform transform = Agent.Transform;
        Vector3d worldPosition = new(position.X, transform.WorldPosition.Y, position.Y);
        FixedTransform? parent = transform.Parent;
        SwiftThrowHelper.ThrowIfTrue(
            parent != null && (!parent.TryInverseTransformPoint(worldPosition, out Vector3d localPosition)
                || !parent.TryTransformPoint(localPosition, out Vector3d roundTrip)
                || !roundTrip.FuzzyEqualAbsolute(worldPosition, Fixed64.Epsilon)),
            nameof(position),
            "The host transform cannot represent the requested 2D body-root position.");

        LSCollider2D candidate = definition.CreateRuntimeCollider();
        candidate.PrepareDetachedBodyReconfigurationCandidate(this, localOffset, position);
        if (_position == position && Collider.MatchesCommittedReconfigurationCandidate(candidate))
        {
            SwiftList<Exception>? unchangedFailures = null;
            CaptureSynchronizedStatePublication(publishSynchronizedState, ref unchangedFailures);
            notificationException = CollisionNotificationExceptions.ToException(unchangedFailures);
            return ColliderReconfigurationStatus.Unchanged;
        }

        // The grid-backed query index excludes unpartitioned colliders. A rare
        // explicit posture transaction must not omit those registered blockers.
        int count = Context.Physics2D.ColliderCount;
        for (int i = 0; i < count; i++)
        {
            LSCollider2D other = Context.Physics2D.GetColliderByServiceIndex(i);
            if (ReferenceEquals(other, Collider) || other.IsTrigger
                || !Context.Physics2D.RequireCollisionPair(Collider, other)
                || !CollisionDetection2D.DoesPostureCandidatePenetrate(candidate, other))
                continue;

            blocker = other;
            return ColliderReconfigurationStatus.Blocked;
        }

        Vector2d centerOfMass = _centerOfMassOffsetExplicit
            ? _localCenterOfMassOffset
            : candidate.CalculateLocalCenterOfMassOffset();
        Fixed64 moment = _mass > Fixed64.Zero
            ? candidate.CalculateMomentOfInertia(_mass, centerOfMass)
            : Fixed64.Zero;
        Fixed64 inverseMoment = moment > Fixed64.Zero ? Fixed64.One / moment : Fixed64.Zero;

        // All geometry and mass work has succeeded on detached state. There are
        // no host callbacks until every authoritative view has been published.
        SwiftThrowHelper.ThrowIfTrue(!transform.TrySetWorldPosition(worldPosition), nameof(position),
            "The host transform cannot represent the requested 2D body-root position.");
        _position = position;
        Collider.PublishBodyReconfigurationCandidate(candidate);
        _localCenterOfMassOffset = centerOfMass;
        _momentOfInertia = moment;
        _inverseMomentOfInertia = inverseMoment;
        Context.Constraints2D.ClearSolverCachesForBody(this);
        Context.Physics2D.InvalidateContinuousCollisionStateForMotionTypeChange(this, DynamicId);
        InvalidateContinuousCollisionFrame();
        Context.Collisions2D.RefreshColliderPartitionAfterShapeChange(Collider);
        Wake();

        var registration = new ColliderLifetimeToken2D(Collider);
        SwiftList<Exception>? failures = null;
        CaptureSynchronizedStatePublication(publishSynchronizedState, ref failures);
        if (registration.IsCurrentLifetime)
            Context.Physics2D.InvalidatePairsForColliderReconfiguration(Collider, ref failures);
        notificationException = CollisionNotificationExceptions.ToException(failures);
        return ColliderReconfigurationStatus.Applied;
    }

    private static void CaptureSynchronizedStatePublication(
        Action? publisher, ref SwiftList<Exception>? failures)
    {
        if (publisher == null)
            return;
        try
        {
            publisher();
        }
        catch (Exception exception)
        {
            CollisionNotificationExceptions.Capture(ref failures, exception);
        }
    }
}
