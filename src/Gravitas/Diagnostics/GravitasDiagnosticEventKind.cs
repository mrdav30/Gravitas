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
    ResponseImpulse = 9
}
