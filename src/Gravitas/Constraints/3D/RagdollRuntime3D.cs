//=======================================================================
// RagdollRuntime3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;

namespace Gravitas.Constraints;

/// <summary>
/// Runtime handle for a context-owned 3D ragdoll articulation.
/// </summary>
public sealed class RagdollRuntime3D : IRecordable
{
    private readonly SolidBody[] _links;
    private readonly Joint3D[] _joints;
    private bool _isActive;

    internal RagdollRuntime3D(
        int id,
        SolidBody[] links,
        Joint3D[] joints,
        RagdollSelfCollisionPolicy selfCollisionPolicy)
    {
        Id = id;
        _links = links;
        _joints = joints;
        SelfCollisionPolicy = selfCollisionPolicy;
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
    public SolidBody GetLink(int index) => _links[index];

    /// <summary>
    /// Gets a ragdoll joint by runtime index.
    /// </summary>
    public Joint3D GetJoint(int index) => _joints[index];

    /// <summary>
    /// Switches all links to dynamic simulation and enables ragdoll joints.
    /// </summary>
    public void ActivateDynamic()
    {
        for (int i = 0; i < _links.Length; i++)
        {
            _links[i].IsKinematic = false;
            _links[i].Wake();
        }

        for (int i = 0; i < _joints.Length; i++)
            _joints[i].IsEnabled = true;

        _isActive = true;
        if (_joints.Length > 0)
            _joints[0].Context.Diagnostics.EmitRagdollActivated(Id, LinkCount, JointCount, true);
    }

    /// <summary>
    /// Switches all links to kinematic host control and disables ragdoll joints.
    /// </summary>
    public void DeactivateToKinematic()
    {
        for (int i = 0; i < _joints.Length; i++)
            _joints[i].IsEnabled = false;

        for (int i = 0; i < _links.Length; i++)
            _links[i].IsKinematic = true;

        _isActive = false;
        if (_joints.Length > 0)
            _joints[0].Context.Diagnostics.EmitRagdollActivated(Id, LinkCount, JointCount, false);
    }

    /// <summary>
    /// Records runtime activation state for deterministic continuation.
    /// </summary>
    public void RecordData(IChronicler chronicler)
    {
        bool isActive = _isActive;
        RecordValues.Look(chronicler, ref isActive, "IsActive", false);
        if (chronicler.Mode != SerializationMode.Loading || isActive == _isActive)
            return;

        if (isActive)
            ActivateDynamic();
        else
            DeactivateToKinematic();
    }
}
