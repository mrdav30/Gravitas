//=======================================================================
// SolidBody2D.ColliderReconfiguration.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using SwiftCollections;

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
        ChronicleGuard.ThrowIfTrue(
            Context.Settings.RuntimeMode != PhysicsRuntimeMode.TwoD,
            nameof(Context.Settings.RuntimeMode),
            "Native 2D collider reconfiguration requires pure TwoD runtime mode.");
        ValidateReconfigurationFamily(definition);
        bool representable = TryGetReconfigurationWorldPosition(position, out Vector3d worldPosition);
        ChronicleGuard.ThrowIfTrue(!representable, nameof(position),
            "The host transform cannot represent the requested 2D body-root position.");

        LSCollider2D candidate = definition.CreateRuntimeCollider();
        candidate.PrepareDetachedBodyReconfigurationCandidate(this, localOffset, position);
        if (MatchesReconfiguration(candidate, position))
        {
            notificationException = CompleteColliderReconfiguration(ColliderReconfigurationStatus.Unchanged, publishSynchronizedState);
            return ColliderReconfigurationStatus.Unchanged;
        }

        PhysicsQueryMeter meter = default;
        FindColliderReconfigurationBlocker(candidate, ref meter, bounded: false, out blocker);
        if (blocker != null)
            return ColliderReconfigurationStatus.Blocked;

        ChronicleGuard.ThrowIfTrue(!TryCommitReconfigurationGeometry(candidate, position, worldPosition),
            nameof(position), "The host transform cannot represent the requested 2D body-root position.");
        notificationException = CompleteColliderReconfiguration(ColliderReconfigurationStatus.Applied, publishSynchronizedState);
        return ColliderReconfigurationStatus.Applied;
    }

    /// <summary>Attempts to replace registered circle or capsule geometry and its root pose within a bounded clearance query.</summary>
    /// <remarks>
    /// The caller serializes this operation with world and collider mutation. An incomplete query leaves the
    /// live state unchanged and clears outcome outputs. Work limits cover detached preparation and clearance.
    /// Successful publication, mass properties, pair cleanup, and host notifications follow clearance.
    /// </remarks>
    public PhysicsQueryExecutionStatus TryReconfigureCollider(
        ColliderShapeDefinition2D definition, Vector2d localOffset, Vector2d position,
        in PhysicsQueryBudget budget, PhysicsQueryScratch scratch,
        out ColliderReconfigurationStatus? result, out LSCollider2D? blocker,
        out Exception? notificationException, out PhysicsQueryWork work,
        Action? publishSynchronizedState = null)
    {
        result = null;
        blocker = null;
        notificationException = null;
        work = default;
        ChronicleGuard.ThrowIfNull(scratch, nameof(scratch));
        definition.EnsureDefined();
        ThrowIfRuntimeRegistrationMissing();
        if (Context.Settings.RuntimeMode != PhysicsRuntimeMode.TwoD)
            return PhysicsQueryExecutionStatus.UnsupportedRuntimeMode;
        ValidateReconfigurationFamily(definition);

        var meter = new PhysicsQueryMeter(budget);
        LSCollider2D? candidate = null;
        scratch.Borrow();
        try
        {
            ulong sequence = Context.ObservedGridChangeSequence;
            var status = PhysicsQueryScratch.CheckView(Context, sequence, ref meter, PhysicsQueryExecutionStatus.WorldNotReady);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
            if (!PhysicsQueryScratch.ChargePosturePreparation(Agent.Transform, ref meter))
                return PhysicsQueryExecutionStatus.WorkBudgetExceeded;
            if (!TryGetReconfigurationWorldPosition(position, out Vector3d worldPosition))
                return PhysicsQueryExecutionStatus.Unrepresentable;

            candidate = scratch.GetPostureCandidate(definition);
            try
            {
                candidate.PrepareDetachedBodyReconfigurationCandidate(this, localOffset, position);
            }
            catch (Exception exception) when (exception is ArgumentException or InvalidOperationException or OverflowException)
            {
                return PhysicsQueryExecutionStatus.Unrepresentable;
            }

            bool unchanged = MatchesReconfiguration(candidate, position);
            LSCollider2D? found = null;
            if (!unchanged)
            {
                status = FindColliderReconfigurationBlocker(candidate, ref meter, bounded: true, out found);
                if (status != PhysicsQueryExecutionStatus.Completed)
                    return status;
            }
            status = PhysicsQueryScratch.CheckView(Context, sequence, ref meter);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
            if (found != null)
            {
                blocker = found;
                result = ColliderReconfigurationStatus.Blocked;
                return PhysicsQueryExecutionStatus.Completed;
            }

            if (!unchanged && !TryCommitReconfigurationGeometry(candidate, position, worldPosition))
                return PhysicsQueryExecutionStatus.Unrepresentable;
            result = unchanged ? ColliderReconfigurationStatus.Unchanged : ColliderReconfigurationStatus.Applied;
        }
        finally
        {
            candidate?.ClearBindingState();
            scratch.Release(0);
            work = meter.Work;
        }

        notificationException = CompleteColliderReconfiguration(result.Value, publishSynchronizedState);
        return PhysicsQueryExecutionStatus.Completed;
    }

    private void ValidateReconfigurationFamily(ColliderShapeDefinition2D definition)
    {
        ColliderType2D expectedShape = definition.Kind switch
        {
            ColliderShapeDefinition2DKind.Circle => ColliderType2D.Circle,
            ColliderShapeDefinition2DKind.Capsule => ColliderType2D.Capsule,
            _ => ColliderType2D.None
        };
        ChronicleGuard.ThrowIfArgument(
            expectedShape == ColliderType2D.None || Collider.Shape != expectedShape,
            nameof(definition),
            "2D collider reconfiguration requires the existing circle or capsule family.");
    }

    private bool TryGetReconfigurationWorldPosition(Vector2d position, out Vector3d worldPosition)
    {
        FixedTransform transform = Agent.Transform;
        worldPosition = new(position.X, transform.WorldPosition.Y, position.Y);
        FixedTransform? parent = transform.Parent;
        return parent == null || (parent.TryInverseTransformPoint(worldPosition, out Vector3d localPosition)
            && parent.TryTransformPoint(localPosition, out Vector3d roundTrip)
            && roundTrip.FuzzyEqualAbsolute(worldPosition, Fixed64.Epsilon));
    }

    private bool MatchesReconfiguration(LSCollider2D candidate, Vector2d position) =>
        _position == position && Collider.MatchesCommittedReconfigurationCandidate(candidate);

    private PhysicsQueryExecutionStatus FindColliderReconfigurationBlocker(
        LSCollider2D candidate, ref PhysicsQueryMeter meter, bool bounded, out LSCollider2D? blocker)
    {
        blocker = null;
        // The grid-backed query index excludes unpartitioned colliders. A rare
        // explicit posture transaction must not omit those registered blockers.
        var physics = Context.Physics2D;
        int count = physics.ColliderCount;
        int cursor = bounded ? physics.FirstColliderRegistrationId : 0;
        while (bounded ? cursor >= 0 : cursor < count)
        {
            if (bounded && !meter.ChargeCandidate())
                return PhysicsQueryExecutionStatus.WorkBudgetExceeded;
            LSCollider2D other = bounded ? physics.GetColliderByRegistrationId(cursor, out cursor)
                : physics.GetColliderByServiceIndex(cursor++);
            if (ReferenceEquals(other, Collider) || other.IsTrigger)
                continue;
            if (bounded && !meter.ChargeCandidates(Context.Constraints2D.LinkedCollisionProbeLimit))
                return PhysicsQueryExecutionStatus.WorkBudgetExceeded;
            if (!physics.RequireCollisionPair(Collider, other))
                continue;

            var status = CollisionDetection2D.CheckPostureCandidatePenetration(candidate, other, ref meter, bounded, out bool penetrates);
            if (status != PhysicsQueryExecutionStatus.Completed)
                return status;
            if (!penetrates)
                continue;

            blocker = other;
            break;
        }
        return PhysicsQueryExecutionStatus.Completed;
    }

    private bool TryCommitReconfigurationGeometry(LSCollider2D candidate, Vector2d position, Vector3d worldPosition)
    {
        Vector2d centerOfMass = _centerOfMassOffsetExplicit
            ? _localCenterOfMassOffset
            : candidate.CalculateLocalCenterOfMassOffset();
        Fixed64 moment = _mass > Fixed64.Zero
            ? candidate.CalculateMomentOfInertia(_mass, centerOfMass)
            : Fixed64.Zero;
        Fixed64 inverseMoment = moment > Fixed64.Zero ? Fixed64.One / moment : Fixed64.Zero;

        // All geometry and mass work has succeeded on detached state. There are
        // no host callbacks until every authoritative view has been published.
        if (!Agent.Transform.TrySetWorldPosition(worldPosition))
            return false;
        _position = position;
        Collider.PublishBodyReconfigurationCandidate(candidate);
        _localCenterOfMassOffset = centerOfMass;
        _momentOfInertia = moment;
        _inverseMomentOfInertia = inverseMoment;
        return true;
    }

    private Exception? CompleteColliderReconfiguration(ColliderReconfigurationStatus result, Action? publisher)
    {
        if (result == ColliderReconfigurationStatus.Applied)
        {
            Context.Constraints2D.ClearSolverCachesForBody(this);
            Context.Physics2D.InvalidateContinuousCollisionStateForBodyMutation(this, DynamicId);
            InvalidateContinuousCollisionFrame();
            Context.Collisions2D.RefreshColliderPartitionAfterShapeChange(Collider);
            Wake();
        }

        var registration = result == ColliderReconfigurationStatus.Applied ? new ColliderLifetimeToken2D(Collider) : default;
        SwiftList<Exception>? failures = null;
        CaptureSynchronizedStatePublication(publisher, ref failures);
        if (result == ColliderReconfigurationStatus.Applied && registration.IsCurrentLifetime)
            Context.Physics2D.InvalidatePairsForColliderReconfiguration(Collider, ref failures);
        return CollisionNotificationExceptions.ToException(failures);
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
