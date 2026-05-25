namespace Gravitas.Diagnostics;

/// <summary>
/// Identifies engine-agnostic diagnostic draw command payloads.
/// </summary>
public enum GravitasDebugDrawKind : byte
{
    Line = 1,
    Ray = 2,
    Point = 3,
    WireSphere = 4,
    WireBox = 5,
    WireCapsule = 6,
    WireCylinder = 7,
    WireTriangle = 8
}
