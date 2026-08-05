//=======================================================================
// GravitasDiagnosticEventKind.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Diagnostics;

/// <summary>
/// Identifies deterministic physics diagnostic event payloads.
/// </summary>
public enum GravitasDiagnosticEventKind : byte
{
    /// <summary>A force changed a body's acceleration.</summary>
    ForceDelta = 1,

    /// <summary>A torque was applied to a body.</summary>
    TorqueDelta = 2,

    /// <summary>A body's linear velocity changed.</summary>
    LinearVelocityDelta = 3,

    /// <summary>A body's angular velocity changed.</summary>
    AngularVelocityDelta = 4,

    /// <summary>A body performed a ground probe.</summary>
    GroundProbe = 5,

    /// <summary>A 3D ray or swept-sphere query completed.</summary>
    RayQuery = 6,

    /// <summary>A 3D X/Z circle query completed.</summary>
    CircleQuery = 7,

    /// <summary>A 3D collision contact was evaluated.</summary>
    Contact = 8,

    /// <summary>A 3D collision response impulse was applied.</summary>
    ResponseImpulse = 9,

    /// <summary>A mixed 3D/2D query completed.</summary>
    MixedQuery = 10,

    /// <summary>A mixed 3D/2D collision contact was evaluated.</summary>
    MixedContact = 11,

    /// <summary>A mixed 3D/2D response impulse was applied.</summary>
    MixedResponseImpulse = 12,

    /// <summary>A mixed response island was solved.</summary>
    MixedResponseIsland = 13,

    /// <summary>A query reducer reported its quality counters.</summary>
    QuerySummary = 14,

    /// <summary>A joint was registered with its owning service.</summary>
    JointRegistered = 15,

    /// <summary>A joint was removed from its owning service.</summary>
    JointRemoved = 16,

    /// <summary>A joint solver applied impulses.</summary>
    JointImpulse = 17,

    /// <summary>A joint reached a configured limit.</summary>
    JointLimitReached = 18,

    /// <summary>A ragdoll runtime changed active state.</summary>
    RagdollActivated = 19
}
