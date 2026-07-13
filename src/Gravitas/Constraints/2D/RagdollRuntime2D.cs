//=======================================================================
// RagdollRuntime2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;

namespace Gravitas.Constraints;

/// <summary>
/// Runtime handle for a context-owned pure 2D ragdoll articulation.
/// </summary>
public sealed class RagdollRuntime2D : IRecordable
{
    private readonly SolidBody2D[] _links;
    private readonly Joint2D[] _joints;
    private bool _isActive;

    internal RagdollRuntime2D(
        int id,
        SolidBody2D[] links,
        Joint2D[] joints,
        RagdollSelfCollisionPolicy selfCollisionPolicy,
        bool startsActive)
    {
        Id = id;
        _links = links;
        _joints = joints;
        SelfCollisionPolicy = selfCollisionPolicy;
        ApplyActivationState(startsActive, emitDiagnostics: false);
    }

    /// <summary>
    /// Gets this context-local ragdoll ID.
    /// </summary>
    public int Id { get; }

    /// <summary>
    /// Gets ragdoll-internal collision filtering behavior.
    /// </summary>
    public RagdollSelfCollisionPolicy SelfCollisionPolicy { get; }

    /// <summary>
    /// Gets whether the ragdoll links are currently dynamic.
    /// </summary>
    public bool IsActive => _isActive;

    /// <summary>
    /// Gets the number of links in this ragdoll.
    /// </summary>
    public int LinkCount => _links.Length;

    /// <summary>
    /// Gets the number of joints in this ragdoll.
    /// </summary>
    public int JointCount => _joints.Length;

    /// <summary>
    /// Gets a ragdoll link by runtime index.
    /// </summary>
    public SolidBody2D GetLink(int index) => _links[index];

    /// <summary>
    /// Gets a ragdoll joint by runtime index.
    /// </summary>
    public Joint2D GetJoint(int index) => _joints[index];

    /// <summary>
    /// Switches all links to dynamic simulation and enables ragdoll joints.
    /// </summary>
    public void ActivateDynamic()
    {
        ApplyActivationState(isActive: true, emitDiagnostics: true);
    }

    /// <summary>
    /// Switches all links to kinematic host control and disables ragdoll joints.
    /// </summary>
    public void DeactivateToKinematic()
    {
        ApplyActivationState(isActive: false, emitDiagnostics: true);
    }

    /// <summary>
    /// Records runtime activation state for deterministic continuation.
    /// </summary>
    public void RecordData(IChronicler chronicler)
    {
        bool isActive = _isActive;
        RecordValues.Look(chronicler, ref isActive, "IsActive", false);
        if (chronicler.Mode != SerializationMode.Loading)
            return;

        ApplyActivationState(isActive, emitDiagnostics: true);
    }

    private void ApplyActivationState(bool isActive, bool emitDiagnostics)
    {
        if (isActive)
        {
            for (int i = 0; i < _links.Length; i++)
            {
                _links[i].IsKinematic = false;
                _links[i].Wake();
            }

            for (int i = 0; i < _joints.Length; i++)
                _joints[i].IsEnabled = true;
        }
        else
        {
            for (int i = 0; i < _joints.Length; i++)
                _joints[i].IsEnabled = false;

            for (int i = 0; i < _links.Length; i++)
                _links[i].IsKinematic = true;
        }

        _isActive = isActive;
        if (emitDiagnostics)
            _links[0].Context.Diagnostics.EmitRagdollActivated(Id, LinkCount, JointCount, isActive);
    }
}
