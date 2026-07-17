//=======================================================================
// GravitasConstraint3DService.cs
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
/// Owns deterministic 3D joints and ragdoll articulation state for one world context.
/// </summary>
public sealed class GravitasConstraint3DService
{
    private const int DefaultJointCapacity = 64;

    private readonly GravitasWorldContext _context;
    private readonly ConstraintEndpointJointIndex<SolidBody> _jointIdsByBody = new();
    private readonly SwiftDictionary<SolidBody, RagdollRuntime3D> _ragdollByBody = new();
    private readonly SwiftDictionary<int, RagdollRuntime3D> _ragdollsById = new();
    private readonly SwiftDictionary<ulong, int> _suppressedColliderPairs = new();
    private Joint3D?[] _joints = new Joint3D?[DefaultJointCapacity];
    private RagdollRuntime3D? _firstRagdoll;
    private RagdollRuntime3D? _lastRagdoll;
    private int _nextRagdollId;
    private int _registeredRagdollCount;
    private int _enabledJointCount;

    internal GravitasConstraint3DService(GravitasWorldContext context)
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

    /// <summary>
    /// Registers a context-owned deterministic 3D joint.
    /// </summary>
    public Joint3D RegisterJoint(in JointDefinition3D definition)
    {
        _context.ThrowIfDisposed();
        ValidateDefinition(definition);
        JointFrame3D frameA = JointFrame3D.FromTransform(definition.LocalFrameA, nameof(definition.LocalFrameA));
        JointFrame3D frameB = JointFrame3D.FromTransform(definition.LocalFrameB, nameof(definition.LocalFrameB));

        int id = ++PeakJointCount;
        EnsureJointCapacity(id + 1);
        var joint = new Joint3D(this, id, definition, frameA, frameB);
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
    /// Removes a context-owned joint and releases its filter state.
    /// </summary>
    public bool RemoveJoint(int jointId)
    {
        _context.ThrowIfDisposed();
        if (!TryGetJoint(jointId, out Joint3D? joint))
            return false;

        SwiftThrowHelper.ThrowIfTrue(
            joint!.OwningRagdoll?.IsRegistered == true,
            nameof(jointId),
            "Ragdoll-owned joints cannot be removed independently. Remove the owning ragdoll instead.");
        return RemoveJointCore(joint);
    }

    private bool RemoveJointCore(Joint3D joint)
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
    /// Gets a registered joint by context-local ID.
    /// </summary>
    public Joint3D GetJoint(int jointId)
    {
        if (!TryGetJoint(jointId, out Joint3D? joint))
            throw new ArgumentOutOfRangeException(nameof(jointId), "No active joint exists with the supplied ID.");

        return joint!;
    }

    /// <summary>
    /// Tries to get a registered joint by context-local ID.
    /// </summary>
    public bool TryGetJoint(int jointId, out Joint3D? joint)
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
    /// Registers a ragdoll articulation and its owned joints.
    /// </summary>
    public RagdollRuntime3D RegisterRagdoll(RagdollDefinition3D definition)
    {
        _context.ThrowIfDisposed();
        ValidateRagdollDefinition(definition);

        int linkCount = definition.Links.Length;
        var links = new SolidBody[linkCount];
        for (int i = 0; i < linkCount; i++)
            links[i] = definition.Links[i].Body;

        int jointCount = definition.Joints.Length;
        var joints = new Joint3D[jointCount];
        for (int i = 0; i < jointCount; i++)
        {
            RagdollJointDefinition3D authoredJoint = definition.Joints[i];
            SolidBody bodyA = ResolveRagdollLink(definition.Links, authoredJoint.LinkAId);
            SolidBody bodyB = ResolveRagdollLink(definition.Links, authoredJoint.LinkBId);
            JointCollisionPolicy collisionPolicy = definition.SelfCollisionPolicy == RagdollSelfCollisionPolicy.CollideAllLinks
                ? JointCollisionPolicy.Collide
                : authoredJoint.CollisionPolicy;

            joints[i] = RegisterJoint(new JointDefinition3D(
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

        var runtime = new RagdollRuntime3D(
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
    /// Removes a registered ragdoll and every joint and collision suppression it owns.
    /// </summary>
    public bool RemoveRagdoll(int ragdollId)
    {
        _context.ThrowIfDisposed();
        if (!_ragdollsById.TryGetValue(ragdollId, out RagdollRuntime3D? ragdoll))
            return false;

        RemoveRagdollCore(ragdoll);
        return true;
    }

    /// <summary>
    /// Updates a joint motor target while preserving the joint's authored motor gains and impulse cap.
    /// </summary>
    public bool SetJointMotorTarget(int jointId, FixedQuaternion targetLocalRotation)
    {
        _context.ThrowIfDisposed();
        if (!TryGetJoint(jointId, out Joint3D? joint))
            return false;

        JointMotor3D current = joint!.Motor;
        joint.SetMotor(new JointMotor3D(
            targetLocalRotation,
            current.AngularDriveStrength,
            current.AngularDriveDamping,
            current.MaximumMotorImpulse));
        return true;
    }

    /// <summary>
    /// Disables a joint motor target and clears its cached solver impulses.
    /// </summary>
    public bool ClearJointMotorTarget(int jointId)
    {
        _context.ThrowIfDisposed();
        if (!TryGetJoint(jointId, out Joint3D? joint))
            return false;

        joint!.ClearMotor();
        return true;
    }

    /// <summary>
    /// Applies caller-owned motor payloads to every joint in a ragdoll runtime.
    /// </summary>
    public void SetRagdollPoseTargets(RagdollRuntime3D ragdoll, ReadOnlySpan<JointMotor3D> motors)
    {
        _context.ThrowIfDisposed();
        SwiftThrowHelper.ThrowIfNull(ragdoll, nameof(ragdoll));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(ragdoll.Service, this),
            nameof(ragdoll),
            "Ragdoll targets must be applied through the owning constraint service.");
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
            Joint3D joint = ragdoll.GetJoint(i);
            joint.SetMotor(motors[i]);
        }
    }

    /// <summary>
    /// Gets whether the supplied 3D colliders should be excluded because a registered articulation links them.
    /// </summary>
    public bool ShouldExcludeLinkedCollision(LSCollider colliderA, LSCollider colliderB)
    {
        if (colliderA == null || colliderB == null)
            return false;
        if (colliderA.Id < 0 || colliderB.Id < 0 || colliderA.Id == colliderB.Id)
            return false;

        return _suppressedColliderPairs.ContainsKey(CreateColliderPairKey(colliderA.Id, colliderB.Id));
    }

    internal bool TryGetJointForSolver(int jointId, out Joint3D? joint)
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

    internal void RemoveJointsForBody(SolidBody body)
    {
        if (_ragdollByBody.TryGetValue(body, out RagdollRuntime3D? ragdoll))
            RemoveRagdollCore(ragdoll);

        while (_jointIdsByBody.TryGetLast(body, out int jointId))
            RemoveJointCore(GetJoint(jointId));
    }

    internal void UpdateJointCollisionPolicy(
        Joint3D joint,
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
        Joint3D joint,
        bool oldEnabled,
        bool newEnabled)
    {
        if (!joint.IsActive || oldEnabled == newEnabled)
            return;

        _enabledJointCount += newEnabled ? 1 : -1;
    }

    internal void Reset()
    {
        RagdollRuntime3D? ragdoll = _firstRagdoll;
        while (ragdoll != null)
        {
            RagdollRuntime3D? next = ragdoll.NextRegistered;
            ragdoll.Invalidate();
            ragdoll = next;
        }

        for (int i = 1; i <= PeakJointCount && i < _joints.Length; i++)
        {
            Joint3D? joint = _joints[i];
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
        writer.WriteSection("constraints.3d", 1);
        writer.WriteInt32(PeakJointCount);
        writer.WriteInt32(RegisteredJointCount);
        for (int jointId = 1; jointId <= PeakJointCount; jointId++)
        {
            bool hasJoint = TryGetJoint(jointId, out Joint3D? joint);
            writer.WriteBool(hasJoint);
            if (hasJoint)
                joint!.ContributeReplayHash(ref writer, mode);
        }

        writer.WriteInt32(_registeredRagdollCount);
        for (RagdollRuntime3D? ragdoll = _firstRagdoll;
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

    private void ValidateDefinition(in JointDefinition3D definition)
    {
        SwiftThrowHelper.ThrowIfNull(definition.BodyA, nameof(definition.BodyA));
        SwiftThrowHelper.ThrowIfNull(definition.BodyB, nameof(definition.BodyB));
        SwiftThrowHelper.ThrowIfNull(definition.LocalFrameA, nameof(definition.LocalFrameA));
        SwiftThrowHelper.ThrowIfNull(definition.LocalFrameB, nameof(definition.LocalFrameB));
        SwiftThrowHelper.ThrowIfArgument(
            ReferenceEquals(definition.BodyA, definition.BodyB),
            nameof(definition),
            "A joint cannot link a body to itself.");
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(definition.BodyA.Context, _context) || !ReferenceEquals(definition.BodyB.Context, _context),
            nameof(definition),
            "Both joint bodies must belong to this constraint service context.");
        SwiftThrowHelper.ThrowIfArgument(
            !definition.BodyA.Active || !definition.BodyB.Active,
            nameof(definition),
            "Joint bodies must be active before registration.");
        SwiftThrowHelper.ThrowIfArgument(
            !Joint3D.IsSupportedType(definition.Type),
            nameof(definition.Type),
            "Unsupported 3D joint type.");
        SwiftThrowHelper.ThrowIfArgument(
            definition.CollisionPolicy != JointCollisionPolicy.SuppressLinked
                && definition.CollisionPolicy != JointCollisionPolicy.Collide,
            nameof(definition.CollisionPolicy),
            "Unsupported joint collision policy.");
        definition.Limits.Validate();
        definition.Motor.Validate();
        Joint3D.ValidatePayload(definition.Type, definition.Limits);
    }

    private void ValidateRagdollDefinition(RagdollDefinition3D definition)
    {
        SwiftThrowHelper.ThrowIfNull(definition, nameof(definition));
        SwiftThrowHelper.ThrowIfNull(definition.Links, nameof(definition.Links));
        SwiftThrowHelper.ThrowIfNull(definition.Joints, nameof(definition.Joints));
        SwiftThrowHelper.ThrowIfArgument(definition.Links.Length == 0, nameof(definition.Links), "A ragdoll must contain at least one link.");
        SwiftThrowHelper.ThrowIfArgument(
            definition.SelfCollisionPolicy != RagdollSelfCollisionPolicy.SuppressAdjacentLinks
                && definition.SelfCollisionPolicy != RagdollSelfCollisionPolicy.CollideAllLinks
                && definition.SelfCollisionPolicy != RagdollSelfCollisionPolicy.SuppressAllLinks,
            nameof(definition.SelfCollisionPolicy),
            "Unsupported ragdoll self-collision policy.");

        for (int i = 0; i < definition.Links.Length; i++)
        {
            RagdollLinkDefinition3D link = definition.Links[i];
            SwiftThrowHelper.ThrowIfNull(link.Body, nameof(definition.Links));
            SwiftThrowHelper.ThrowIfNull(link.Collider, nameof(definition.Links));
            SwiftThrowHelper.ThrowIfArgument(
                !ReferenceEquals(link.Body.Context, _context),
                nameof(definition.Links),
                "All ragdoll links must belong to this context.");
            SwiftThrowHelper.ThrowIfArgument(
                !link.Body.Active,
                nameof(definition.Links),
                "Ragdoll links must have active 3D colliders.");
            SwiftThrowHelper.ThrowIfArgument(
                !ReferenceEquals(link.Body.Collider, link.Collider),
                nameof(definition.Links),
                "Ragdoll link collider must match the link body's collider.");
            SwiftThrowHelper.ThrowIfArgument(
                _ragdollByBody.ContainsKey(link.Body),
                nameof(definition.Links),
                "A body cannot belong to more than one registered ragdoll.");

            for (int j = i + 1; j < definition.Links.Length; j++)
            {
                SwiftThrowHelper.ThrowIfArgument(
                    link.LinkId == definition.Links[j].LinkId,
                    nameof(definition.Links),
                    "Ragdoll link IDs must be unique.");
                SwiftThrowHelper.ThrowIfArgument(
                    ReferenceEquals(link.Body, definition.Links[j].Body),
                    nameof(definition.Links),
                    "Ragdoll bodies must be unique.");
            }
        }

        for (int i = 0; i < definition.Joints.Length; i++)
        {
            RagdollJointDefinition3D joint = definition.Joints[i];
            SolidBody bodyA = ResolveRagdollLink(definition.Links, joint.LinkAId);
            SolidBody bodyB = ResolveRagdollLink(definition.Links, joint.LinkBId);
            SwiftThrowHelper.ThrowIfArgument(
                joint.LinkAId == joint.LinkBId,
                nameof(definition.Joints),
                "A ragdoll joint cannot link a body to itself.");
            JointCollisionPolicy collisionPolicy = definition.SelfCollisionPolicy == RagdollSelfCollisionPolicy.CollideAllLinks
                ? JointCollisionPolicy.Collide
                : joint.CollisionPolicy;
            ValidateDefinition(new JointDefinition3D(
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

    private static SolidBody ResolveRagdollLink(RagdollLinkDefinition3D[] links, int linkId)
    {
        for (int i = 0; i < links.Length; i++)
        {
            if (links[i].LinkId == linkId)
                return links[i].Body;
        }

        throw new ArgumentException("Ragdoll joint references an unknown link ID.", nameof(linkId));
    }

    private static bool HasKinematicLink(SolidBody[] links)
    {
        for (int i = 0; i < links.Length; i++)
        {
            if (links[i].IsKinematic)
                return true;
        }

        return false;
    }

    private void AddAllRagdollPairSuppressions(SolidBody[] links)
    {
        for (int i = 0; i < links.Length; i++)
        {
            for (int j = i + 1; j < links.Length; j++)
                AddSuppressedColliderPair(links[i].Collider, links[j].Collider);
        }
    }

    private void RemoveRagdollCore(RagdollRuntime3D ragdoll)
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
            Joint3D joint = ragdoll.GetJoint(i);
            joint.OwningRagdoll = null;
            RemoveJointCore(joint);
        }
    }

    private void AppendRagdoll(RagdollRuntime3D ragdoll)
    {
        ragdoll.PreviousRegistered = _lastRagdoll;
        if (_lastRagdoll == null)
            _firstRagdoll = ragdoll;
        else
            _lastRagdoll.NextRegistered = ragdoll;

        _lastRagdoll = ragdoll;
        _registeredRagdollCount++;
    }

    private void UnlinkRagdoll(RagdollRuntime3D ragdoll)
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

    private void AddSuppressedColliderPair(LSCollider colliderA, LSCollider colliderB)
    {
        ulong key = CreateColliderPairKey(colliderA.Id, colliderB.Id);
        if (_suppressedColliderPairs.TryGetValue(key, out int count))
            _suppressedColliderPairs[key] = count + 1;
        else
            _suppressedColliderPairs.Add(key, 1);
    }

    private void RemoveSuppressedColliderPair(LSCollider colliderA, LSCollider colliderB)
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
