//=======================================================================
// GravitasDebugDrawKind.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Diagnostics;

/// <summary>
/// Identifies engine-agnostic diagnostic draw command payloads.
/// </summary>
public enum GravitasDebugDrawKind : byte
{
    /// <summary>A line segment.</summary>
    Line = 1,

    /// <summary>A directed line segment.</summary>
    Ray = 2,

    /// <summary>A point marker with a radius.</summary>
    Point = 3,

    /// <summary>A wireframe sphere.</summary>
    WireSphere = 4,

    /// <summary>A wireframe oriented box.</summary>
    WireBox = 5,

    /// <summary>A wireframe capsule.</summary>
    WireCapsule = 6,

    /// <summary>A wireframe cylinder.</summary>
    WireCylinder = 7,

    /// <summary>A wireframe triangle.</summary>
    WireTriangle = 8,

    /// <summary>A wireframe cone.</summary>
    WireCone = 9
}
