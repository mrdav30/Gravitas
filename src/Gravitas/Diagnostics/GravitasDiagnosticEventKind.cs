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
    ForceDelta = 1,
    TorqueDelta = 2,
    LinearVelocityDelta = 3,
    AngularVelocityDelta = 4,
    GroundProbe = 5,
    RayQuery = 6,
    CircleQuery = 7,
    Contact = 8,
    ResponseImpulse = 9,
    MixedQuery = 10,
    MixedContact = 11,
    MixedResponseImpulse = 12
}
