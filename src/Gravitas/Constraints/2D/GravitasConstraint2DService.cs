//=======================================================================
// GravitasConstraint2DService.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Constraints;
using SwiftCollections;
using System;

namespace Gravitas;

/// <summary>
/// Owns deterministic pure 2D joints and ragdoll articulation state for one world context.
/// </summary>
public sealed class GravitasConstraint2DService
{
    private const int DefaultJointCapacity = 64;

    private readonly GravitasWorldContext _context;
    private readonly ConstraintEndpointJointIndex<SolidBody2D> _jointIdsByBody = new();
    private readonly SwiftDictionary<SolidBody2D, RagdollRuntime2D> _ragdollByBody = new();
    private readonly SwiftDictionary<int, RagdollRuntime2D> _ragdollsById = new();
    private readonly SwiftDictionary<ulong, int> _suppressedColliderPairs = new();
    private Joint2D?[] _joints = new Joint2D?[DefaultJointCapacity];
    private RagdollRuntime2D? _firstRagdoll;
    private RagdollRuntime2D? _lastRagdoll;
    private int _nextRagdollId;
    private int _registeredRagdollCount;
    private int _enabledJointCount;

    internal GravitasConstraint2DService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    /// <summary>
    /// Gets the owning world context.
    /// </summary>
    public GravitasWorldContext Context => _context;

    /// <summary>
    /// Gets the highest joint ID allocated in this context.
    /// </summary>
    public int PeakJointCount { get; private set; }

    /// <summary>
    /// Gets the number of registered active joints.
    /// </summary>
    public int RegisteredJointCount { get; private set; }

    /// <summary>
    /// Gets the number of registered ragdoll runtimes.
    /// </summary>
    public int RegisteredRagdollCount => _registeredRagdollCount;

    internal int EnabledJointCount => _enabledJointCount;

    internal bool HasActiveJoints => _enabledJointCount > 0;

    internal void ClearSolverCachesForBody(SolidBody2D body)
    {
        if (!_jointIdsByBody.TryGetLast(body, out int jointId))
            return;

        while (true)
        {
            _joints[jointId]!.ClearSolverCache();
            if (!_jointIdsByBody.TryGetPrevious(body, jointId, out int previousJointId))
                return;

            jointId = previousJointId;
        }
    }

    /// <summary>
    /// Registers a context-owned deterministic pure 2D joint.
    /// </summary>
    public Joint2D RegisterJoint(in JointDefinition2D definition)
    {
        _context.ThrowIfDisposed();
        ValidateDefinition(definition);

        int id = ++PeakJointCount;
        EnsureJointCapacity(id + 1);
        var joint = new Joint2D(this, id, definition);
        _joints[id] = joint;
        _jointIdsByBody.Add(joint.BodyA, joint.BodyB, id);
        RegisteredJointCount++;
        _enabledJointCount++;

        if (joint.CollisionPolicy == JointCollisionPolicy.SuppressLinked)
            AddSuppressedColliderPair(joint.BodyA.Collider, joint.BodyB.Collider);

        _context.Diagnostics.EmitJointRegistered(joint);
        return joint;
    }

    /// <summary>
    /// Removes a context-owned pure 2D joint and releases its filter state.
    /// </summary>
    public bool RemoveJoint(int jointId)
    {
        _context.ThrowIfDisposed();
        if (!TryGetJoint(jointId, out Joint2D? joint))
            return false;

        SwiftThrowHelper.ThrowIfTrue(
            joint!.OwningRagdoll?.IsRegistered == true,
            nameof(jointId),
            "Ragdoll-owned joints cannot be removed independently. Remove the owning ragdoll instead.");
        return RemoveJointCore(joint);
    }

    private bool RemoveJointCore(Joint2D joint)
    {
        int jointId = joint.Id;
        _joints[jointId] = null;
        _jointIdsByBody.Remove(jointId);
        RegisteredJointCount--;
        if (joint.IsEnabled)
            _enabledJointCount--;
        joint.IsActive = false;
        joint.ClearSolverCache();
        if (joint.CollisionPolicy == JointCollisionPolicy.SuppressLinked)
            RemoveSuppressedColliderPair(joint.BodyA.Collider, joint.BodyB.Collider);

        _context.Diagnostics.EmitJointRemoved(joint);
        return true;
    }

    /// <summary>
    /// Gets a registered pure 2D joint by context-local ID.
    /// </summary>
    public Joint2D GetJoint(int jointId)
    {
        if (!TryGetJoint(jointId, out Joint2D? joint))
            throw new ArgumentOutOfRangeException(nameof(jointId), "No active 2D joint exists with the supplied ID.");

        return joint!;
    }

    /// <summary>
    /// Tries to get a registered pure 2D joint by context-local ID.
    /// </summary>
    public bool TryGetJoint(int jointId, out Joint2D? joint)
    {
        if ((uint)jointId >= (uint)_joints.Length)
        {
            joint = null;
            return false;
        }

        joint = _joints[jointId];
        return joint != null && joint.IsActive;
    }

    /// <summary>
    /// Registers a pure 2D ragdoll articulation and its owned joints.
    /// </summary>
    public RagdollRuntime2D RegisterRagdoll(RagdollDefinition2D definition)
    {
        _context.ThrowIfDisposed();
        ValidateRagdollDefinition(definition);

        int linkCount = definition.Links.Length;
        var links = new SolidBody2D[linkCount];
        for (int i = 0; i < linkCount; i++)
            links[i] = definition.Links[i].Body;

        int jointCount = definition.Joints.Length;
        var joints = new Joint2D[jointCount];
        for (int i = 0; i < jointCount; i++)
        {
            RagdollJointDefinition2D authoredJoint = definition.Joints[i];
            SolidBody2D bodyA = ResolveRagdollLink(definition.Links, authoredJoint.LinkAId);
            SolidBody2D bodyB = ResolveRagdollLink(definition.Links, authoredJoint.LinkBId);
            JointCollisionPolicy collisionPolicy = definition.SelfCollisionPolicy == RagdollSelfCollisionPolicy.CollideAllLinks
                ? JointCollisionPolicy.Collide
                : authoredJoint.CollisionPolicy;

            joints[i] = RegisterJoint(new JointDefinition2D(
                bodyA,
                bodyB,
                authoredJoint.LocalFrameA,
                authoredJoint.LocalFrameB,
                authoredJoint.Type,
                authoredJoint.Limits,
                authoredJoint.Motor,
                collisionPolicy));
        }

        if (definition.SelfCollisionPolicy == RagdollSelfCollisionPolicy.SuppressAllLinks)
            AddAllRagdollPairSuppressions(links);

        var runtime = new RagdollRuntime2D(
            this,
            ++_nextRagdollId,
            links,
            joints,
            definition.SelfCollisionPolicy,
            startsActive: !HasKinematicLink(links));
        AppendRagdoll(runtime);
        _ragdollsById.Add(runtime.Id, runtime);
        for (int i = 0; i < links.Length; i++)
            _ragdollByBody.Add(links[i], runtime);
        for (int i = 0; i < joints.Length; i++)
            joints[i].OwningRagdoll = runtime;
        return runtime;
    }

    /// <summary>
    /// Removes a registered pure 2D ragdoll and every joint and collision suppression it owns.
    /// </summary>
    public bool RemoveRagdoll(int ragdollId)
    {
        _context.ThrowIfDisposed();
        if (!_ragdollsById.TryGetValue(ragdollId, out RagdollRuntime2D? ragdoll))
            return false;

        RemoveRagdollCore(ragdoll);
        return true;
    }

    /// <summary>
    /// Updates a pure 2D joint motor target while preserving the motor's gains and impulse cap.
    /// </summary>
    public bool SetJointMotorTarget(int jointId, Fixed64 target)
    {
        _context.ThrowIfDisposed();
        if (!TryGetJoint(jointId, out Joint2D? joint))
            return false;

        JointMotor2D current = joint!.Motor;
        JointMotor2D replacement = current.Kind == JointMotorKind2D.Linear
            ? JointMotor2D.Linear(target, current.DriveStrength, current.Damping, current.MaximumMotorImpulse)
            : JointMotor2D.Angular(target, current.DriveStrength, current.Damping, current.MaximumMotorImpulse);
        joint.SetMotor(replacement);
        return true;
    }

    /// <summary>
    /// Disables a pure 2D joint motor target and clears its cached solver impulses.
    /// </summary>
    public bool ClearJointMotorTarget(int jointId)
    {
        _context.ThrowIfDisposed();
        if (!TryGetJoint(jointId, out Joint2D? joint))
            return false;

        joint!.ClearMotor();
        return true;
    }

    /// <summary>
    /// Applies caller-owned motor payloads to every joint in a pure 2D ragdoll runtime.
    /// </summary>
    public void SetRagdollPoseTargets(RagdollRuntime2D ragdoll, ReadOnlySpan<JointMotor2D> motors)
    {
        _context.ThrowIfDisposed();
        SwiftThrowHelper.ThrowIfNull(ragdoll, nameof(ragdoll));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(ragdoll.Service, this),
            nameof(ragdoll),
            "Ragdoll targets must be applied through the owning 2D constraint service.");
        SwiftThrowHelper.ThrowIfTrue(
            !ragdoll.IsRegistered,
            nameof(ragdoll),
            "Removed ragdolls cannot mutate simulation state.");
        SwiftThrowHelper.ThrowIfArgument(
            motors.Length != ragdoll.JointCount,
            nameof(motors),
            "Ragdoll pose target count must match the ragdoll joint count.");

        for (int i = 0; i < motors.Length; i++)
        {
            Joint2D joint = ragdoll.GetJoint(i);
            joint.SetMotor(motors[i]);
        }
    }

    /// <summary>
    /// Gets whether the supplied pure 2D colliders should be excluded because a registered articulation links them.
    /// </summary>
    public bool ShouldExcludeLinkedCollision(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        if (colliderA == null || colliderB == null)
            return false;
        if (colliderA.Id < 0 || colliderB.Id < 0 || colliderA.Id == colliderB.Id)
            return false;

        return _suppressedColliderPairs.ContainsKey(CreateColliderPairKey(colliderA.Id, colliderB.Id));
    }

    internal bool TryGetJointForSolver(int jointId, out Joint2D? joint)
    {
        if (!TryGetJoint(jointId, out joint))
            return false;

        if (!joint!.IsEnabled || !joint.HasSolverParticipant())
        {
            joint = null;
            return false;
        }

        return true;
    }

    internal void RemoveJointsForBody(SolidBody2D body)
    {
        if (_ragdollByBody.TryGetValue(body, out RagdollRuntime2D? ragdoll))
            RemoveRagdollCore(ragdoll);

        while (_jointIdsByBody.TryGetLast(body, out int jointId))
            RemoveJointCore(GetJoint(jointId));
    }

    internal void UpdateJointCollisionPolicy(
        Joint2D joint,
        JointCollisionPolicy oldPolicy,
        JointCollisionPolicy newPolicy)
    {
        if (!joint.IsActive || oldPolicy == newPolicy)
            return;

        if (oldPolicy == JointCollisionPolicy.SuppressLinked)
            RemoveSuppressedColliderPair(joint.BodyA.Collider, joint.BodyB.Collider);
        if (newPolicy == JointCollisionPolicy.SuppressLinked)
            AddSuppressedColliderPair(joint.BodyA.Collider, joint.BodyB.Collider);
    }

    internal void UpdateJointEnabledState(
        Joint2D joint,
        bool oldEnabled,
        bool newEnabled)
    {
        if (!joint.IsActive || oldEnabled == newEnabled)
            return;

        _enabledJointCount += newEnabled ? 1 : -1;
    }

    internal void Reset()
    {
        RagdollRuntime2D? ragdoll = _firstRagdoll;
        while (ragdoll != null)
        {
            RagdollRuntime2D? next = ragdoll.NextRegistered;
            ragdoll.Invalidate();
            ragdoll = next;
        }

        for (int i = 1; i <= PeakJointCount && i < _joints.Length; i++)
        {
            Joint2D? joint = _joints[i];
            if (joint == null)
                continue;

            joint.IsActive = false;
            joint.ClearSolverCache();
            joint.OwningRagdoll = null;
            _joints[i] = null;
        }

        _suppressedColliderPairs.Clear();
        _jointIdsByBody.Clear();
        _ragdollByBody.Clear();
        _ragdollsById.Clear();
        _firstRagdoll = null;
        _lastRagdoll = null;
        PeakJointCount = 0;
        RegisteredJointCount = 0;
        _registeredRagdollCount = 0;
        _enabledJointCount = 0;
        _nextRagdollId = 0;
    }

    internal void ContributeReplayHash(
        ref ChronicleHashWriter writer,
        GravitasReplayHashMode mode)
    {
        writer.WriteSection("constraints.2d", 1);
        writer.WriteInt32(PeakJointCount);
        writer.WriteInt32(RegisteredJointCount);
        for (int jointId = 1; jointId <= PeakJointCount; jointId++)
        {
            bool hasJoint = TryGetJoint(jointId, out Joint2D? joint);
            writer.WriteBool(hasJoint);
            if (hasJoint)
                joint!.ContributeReplayHash(ref writer, mode);
        }

        writer.WriteInt32(_registeredRagdollCount);
        for (RagdollRuntime2D? ragdoll = _firstRagdoll;
             ragdoll != null;
             ragdoll = ragdoll.NextRegistered)
        {
            writer.WriteInt32(ragdoll.Id);
            writer.WriteEnum(ragdoll.SelfCollisionPolicy);
            writer.WriteBool(ragdoll.IsActive);
            writer.WriteInt32(ragdoll.LinkCount);
            writer.WriteInt32(ragdoll.JointCount);
        }
    }

    internal static ulong CreateColliderPairKey(int colliderAId, int colliderBId)
    {
        uint min = (uint)(colliderAId <= colliderBId ? colliderAId : colliderBId);
        uint max = (uint)(colliderAId <= colliderBId ? colliderBId : colliderAId);
        return ((ulong)min << 32) | max;
    }

    private void EnsureJointCapacity(int required)
    {
        if (required <= _joints.Length)
            return;

        int newSize = _joints.Length;
        while (newSize < required)
            newSize *= 2;

        Array.Resize(ref _joints, newSize);
    }

    private void ValidateDefinition(in JointDefinition2D definition)
    {
        SwiftThrowHelper.ThrowIfNull(definition.BodyA, nameof(definition.BodyA));
        SwiftThrowHelper.ThrowIfNull(definition.BodyB, nameof(definition.BodyB));
        SwiftThrowHelper.ThrowIfArgument(
            ReferenceEquals(definition.BodyA, definition.BodyB),
            nameof(definition),
            "A 2D joint cannot link a body to itself.");
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(definition.BodyA.Context, _context) || !ReferenceEquals(definition.BodyB.Context, _context),
            nameof(definition),
            "Both 2D joint bodies must belong to this constraint service context.");
        SwiftThrowHelper.ThrowIfArgument(
            !definition.BodyA.Active || !definition.BodyB.Active,
            nameof(definition),
            "2D joint bodies must be active before registration.");
        SwiftThrowHelper.ThrowIfArgument(
            !Joint2D.IsSupportedType(definition.Type),
            nameof(definition.Type),
            "Unsupported 2D joint type.");
        SwiftThrowHelper.ThrowIfArgument(
            definition.CollisionPolicy != JointCollisionPolicy.SuppressLinked
                && definition.CollisionPolicy != JointCollisionPolicy.Collide,
            nameof(definition.CollisionPolicy),
            "Unsupported joint collision policy.");
        definition.Limits.Validate();
        definition.Motor.Validate();
        Joint2D.ValidatePayload(definition.Type, definition.Limits, definition.Motor);
    }

    private void ValidateRagdollDefinition(RagdollDefinition2D definition)
    {
        SwiftThrowHelper.ThrowIfNull(definition, nameof(definition));
        SwiftThrowHelper.ThrowIfNull(definition.Links, nameof(definition.Links));
        SwiftThrowHelper.ThrowIfNull(definition.Joints, nameof(definition.Joints));
        SwiftThrowHelper.ThrowIfArgument(definition.Links.Length == 0, nameof(definition.Links), "A 2D ragdoll must contain at least one link.");
        SwiftThrowHelper.ThrowIfArgument(
            definition.SelfCollisionPolicy != RagdollSelfCollisionPolicy.SuppressAdjacentLinks
                && definition.SelfCollisionPolicy != RagdollSelfCollisionPolicy.CollideAllLinks
                && definition.SelfCollisionPolicy != RagdollSelfCollisionPolicy.SuppressAllLinks,
            nameof(definition.SelfCollisionPolicy),
            "Unsupported ragdoll self-collision policy.");

        for (int i = 0; i < definition.Links.Length; i++)
        {
            RagdollLinkDefinition2D link = definition.Links[i];
            SwiftThrowHelper.ThrowIfNull(link.Body, nameof(definition.Links));
            SwiftThrowHelper.ThrowIfNull(link.Collider, nameof(definition.Links));
            SwiftThrowHelper.ThrowIfArgument(
                !ReferenceEquals(link.Body.Context, _context),
                nameof(definition.Links),
                "All 2D ragdoll links must belong to this context.");
            SwiftThrowHelper.ThrowIfArgument(
                !link.Body.Active,
                nameof(definition.Links),
                "2D ragdoll links must have active 2D colliders.");
            SwiftThrowHelper.ThrowIfArgument(
                !ReferenceEquals(link.Body.Collider, link.Collider),
                nameof(definition.Links),
                "2D ragdoll link collider must match the link body's collider.");
            SwiftThrowHelper.ThrowIfArgument(
                _ragdollByBody.ContainsKey(link.Body),
                nameof(definition.Links),
                "A 2D body cannot belong to more than one registered ragdoll.");

            for (int j = i + 1; j < definition.Links.Length; j++)
            {
                SwiftThrowHelper.ThrowIfArgument(
                    link.LinkId == definition.Links[j].LinkId,
                    nameof(definition.Links),
                    "2D ragdoll link IDs must be unique.");
                SwiftThrowHelper.ThrowIfArgument(
                    ReferenceEquals(link.Body, definition.Links[j].Body),
                    nameof(definition.Links),
                    "2D ragdoll bodies must be unique.");
            }
        }

        for (int i = 0; i < definition.Joints.Length; i++)
        {
            RagdollJointDefinition2D joint = definition.Joints[i];
            SolidBody2D bodyA = ResolveRagdollLink(definition.Links, joint.LinkAId);
            SolidBody2D bodyB = ResolveRagdollLink(definition.Links, joint.LinkBId);
            SwiftThrowHelper.ThrowIfArgument(
                joint.LinkAId == joint.LinkBId,
                nameof(definition.Joints),
                "A 2D ragdoll joint cannot link a body to itself.");
            JointCollisionPolicy collisionPolicy = definition.SelfCollisionPolicy == RagdollSelfCollisionPolicy.CollideAllLinks
                ? JointCollisionPolicy.Collide
                : joint.CollisionPolicy;
            ValidateDefinition(new JointDefinition2D(
                bodyA,
                bodyB,
                joint.LocalFrameA,
                joint.LocalFrameB,
                joint.Type,
                joint.Limits,
                joint.Motor,
                collisionPolicy));
        }
    }

    private static bool HasKinematicLink(SolidBody2D[] links)
    {
        for (int i = 0; i < links.Length; i++)
        {
            if (links[i].IsKinematic)
                return true;
        }

        return false;
    }

    private static SolidBody2D ResolveRagdollLink(RagdollLinkDefinition2D[] links, int linkId)
    {
        for (int i = 0; i < links.Length; i++)
        {
            if (links[i].LinkId == linkId)
                return links[i].Body;
        }

        throw new ArgumentException("2D ragdoll joint references an unknown link ID.", nameof(linkId));
    }

    private void AddAllRagdollPairSuppressions(SolidBody2D[] links)
    {
        for (int i = 0; i < links.Length; i++)
        {
            for (int j = i + 1; j < links.Length; j++)
                AddSuppressedColliderPair(links[i].Collider, links[j].Collider);
        }
    }

    private void RemoveRagdollCore(RagdollRuntime2D ragdoll)
    {
        _ragdollsById.Remove(ragdoll.Id);
        for (int i = 0; i < ragdoll.LinkCount; i++)
            _ragdollByBody.Remove(ragdoll.GetLink(i));

        if (ragdoll.SelfCollisionPolicy == RagdollSelfCollisionPolicy.SuppressAllLinks)
        {
            for (int i = 0; i < ragdoll.LinkCount; i++)
            {
                for (int j = i + 1; j < ragdoll.LinkCount; j++)
                {
                    RemoveSuppressedColliderPair(
                        ragdoll.GetLink(i).Collider,
                        ragdoll.GetLink(j).Collider);
                }
            }
        }

        UnlinkRagdoll(ragdoll);
        ragdoll.Invalidate();

        for (int i = 0; i < ragdoll.JointCount; i++)
        {
            Joint2D joint = ragdoll.GetJoint(i);
            joint.OwningRagdoll = null;
            RemoveJointCore(joint);
        }
    }

    private void AppendRagdoll(RagdollRuntime2D ragdoll)
    {
        ragdoll.PreviousRegistered = _lastRagdoll;
        if (_lastRagdoll == null)
            _firstRagdoll = ragdoll;
        else
            _lastRagdoll.NextRegistered = ragdoll;

        _lastRagdoll = ragdoll;
        _registeredRagdollCount++;
    }

    private void UnlinkRagdoll(RagdollRuntime2D ragdoll)
    {
        if (ragdoll.PreviousRegistered == null)
            _firstRagdoll = ragdoll.NextRegistered;
        else
            ragdoll.PreviousRegistered.NextRegistered = ragdoll.NextRegistered;

        if (ragdoll.NextRegistered == null)
            _lastRagdoll = ragdoll.PreviousRegistered;
        else
            ragdoll.NextRegistered.PreviousRegistered = ragdoll.PreviousRegistered;

        _registeredRagdollCount--;
    }

    private void AddSuppressedColliderPair(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        ulong key = CreateColliderPairKey(colliderA.Id, colliderB.Id);
        if (_suppressedColliderPairs.TryGetValue(key, out int count))
            _suppressedColliderPairs[key] = count + 1;
        else
            _suppressedColliderPairs.Add(key, 1);
    }

    private void RemoveSuppressedColliderPair(LSCollider2D colliderA, LSCollider2D colliderB)
    {
        ulong key = CreateColliderPairKey(colliderA.Id, colliderB.Id);
        if (!_suppressedColliderPairs.TryGetValue(key, out int count))
            return;

        if (count <= 1)
            _suppressedColliderPairs.Remove(key);
        else
            _suppressedColliderPairs[key] = count - 1;
    }
}
