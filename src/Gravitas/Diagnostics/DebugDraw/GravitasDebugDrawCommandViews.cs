//=======================================================================
// GravitasDebugDrawCommandViews.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Diagnostics;

/// <summary>
/// Typed read-only view over a line draw command.
/// </summary>
public readonly struct GravitasLineDebugDrawView
{
    internal GravitasLineDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    /// <summary>Gets the underlying draw command.</summary>
    public GravitasDebugDrawCommand Command { get; }

    /// <summary>Gets the simulation frame in which the command was captured.</summary>
    public int Frame => Command.Frame;

    /// <summary>Gets the command's sequence within its capture buffer.</summary>
    public int Sequence => Command.Sequence;

    /// <summary>Gets the associated context-local collider identifier.</summary>
    public int ColliderId => Command.ColliderId;

    /// <summary>Gets the associated collider dimension.</summary>
    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    /// <summary>Gets the associated 3D collider type.</summary>
    public ColliderType ColliderType => Command.ColliderType;

    /// <summary>Gets the associated 2D collider type.</summary>
    public ColliderType2D Collider2DType => Command.Collider2DType;

    /// <summary>Gets the line start point.</summary>
    public Vector3d Start => Command.Start;

    /// <summary>Gets the line end point.</summary>
    public Vector3d End => Command.End;

    /// <summary>Gets the line color.</summary>
    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a ray draw command.
/// </summary>
public readonly struct GravitasRayDebugDrawView
{
    internal GravitasRayDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    /// <summary>Gets the underlying draw command.</summary>
    public GravitasDebugDrawCommand Command { get; }

    /// <summary>Gets the simulation frame in which the command was captured.</summary>
    public int Frame => Command.Frame;

    /// <summary>Gets the command's sequence within its capture buffer.</summary>
    public int Sequence => Command.Sequence;

    /// <summary>Gets the associated context-local collider identifier.</summary>
    public int ColliderId => Command.ColliderId;

    /// <summary>Gets the associated collider dimension.</summary>
    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    /// <summary>Gets the associated 3D collider type.</summary>
    public ColliderType ColliderType => Command.ColliderType;

    /// <summary>Gets the associated 2D collider type.</summary>
    public ColliderType2D Collider2DType => Command.Collider2DType;

    /// <summary>Gets the ray origin.</summary>
    public Vector3d Start => Command.Start;

    /// <summary>Gets the ray endpoint.</summary>
    public Vector3d End => Command.End;

    /// <summary>Gets the ray color.</summary>
    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a point draw command.
/// </summary>
public readonly struct GravitasPointDebugDrawView
{
    internal GravitasPointDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    /// <summary>Gets the underlying draw command.</summary>
    public GravitasDebugDrawCommand Command { get; }

    /// <summary>Gets the simulation frame in which the command was captured.</summary>
    public int Frame => Command.Frame;

    /// <summary>Gets the command's sequence within its capture buffer.</summary>
    public int Sequence => Command.Sequence;

    /// <summary>Gets the associated context-local collider identifier.</summary>
    public int ColliderId => Command.ColliderId;

    /// <summary>Gets the associated collider dimension.</summary>
    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    /// <summary>Gets the associated 3D collider type.</summary>
    public ColliderType ColliderType => Command.ColliderType;

    /// <summary>Gets the associated 2D collider type.</summary>
    public ColliderType2D Collider2DType => Command.Collider2DType;

    /// <summary>Gets the marker center.</summary>
    public Vector3d Center => Command.Center;

    /// <summary>Gets the marker radius.</summary>
    public Fixed64 Radius => Command.Radius;

    /// <summary>Gets the marker color.</summary>
    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a wire-sphere draw command.
/// </summary>
public readonly struct GravitasWireSphereDebugDrawView
{
    internal GravitasWireSphereDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    /// <summary>Gets the underlying draw command.</summary>
    public GravitasDebugDrawCommand Command { get; }

    /// <summary>Gets the simulation frame in which the command was captured.</summary>
    public int Frame => Command.Frame;

    /// <summary>Gets the command's sequence within its capture buffer.</summary>
    public int Sequence => Command.Sequence;

    /// <summary>Gets the associated context-local collider identifier.</summary>
    public int ColliderId => Command.ColliderId;

    /// <summary>Gets the associated collider dimension.</summary>
    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    /// <summary>Gets the associated 3D collider type.</summary>
    public ColliderType ColliderType => Command.ColliderType;

    /// <summary>Gets the associated 2D collider type.</summary>
    public ColliderType2D Collider2DType => Command.Collider2DType;

    /// <summary>Gets the sphere center.</summary>
    public Vector3d Center => Command.Center;

    /// <summary>Gets the sphere radius.</summary>
    public Fixed64 Radius => Command.Radius;

    /// <summary>Gets the sphere color.</summary>
    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a wire-box draw command.
/// </summary>
public readonly struct GravitasWireBoxDebugDrawView
{
    internal GravitasWireBoxDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    /// <summary>Gets the underlying draw command.</summary>
    public GravitasDebugDrawCommand Command { get; }

    /// <summary>Gets the simulation frame in which the command was captured.</summary>
    public int Frame => Command.Frame;

    /// <summary>Gets the command's sequence within its capture buffer.</summary>
    public int Sequence => Command.Sequence;

    /// <summary>Gets the associated context-local collider identifier.</summary>
    public int ColliderId => Command.ColliderId;

    /// <summary>Gets the associated collider dimension.</summary>
    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    /// <summary>Gets the associated 3D collider type.</summary>
    public ColliderType ColliderType => Command.ColliderType;

    /// <summary>Gets the associated 2D collider type.</summary>
    public ColliderType2D Collider2DType => Command.Collider2DType;

    /// <summary>Gets the box center.</summary>
    public Vector3d Center => Command.Center;

    /// <summary>Gets the box half-extents.</summary>
    public Vector3d HalfExtents => Command.HalfExtents;

    /// <summary>Gets the box orientation.</summary>
    public FixedQuaternion Rotation => Command.Rotation;

    /// <summary>Gets the box color.</summary>
    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a wire-capsule draw command.
/// </summary>
public readonly struct GravitasWireCapsuleDebugDrawView
{
    internal GravitasWireCapsuleDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    /// <summary>Gets the underlying draw command.</summary>
    public GravitasDebugDrawCommand Command { get; }

    /// <summary>Gets the simulation frame in which the command was captured.</summary>
    public int Frame => Command.Frame;

    /// <summary>Gets the command's sequence within its capture buffer.</summary>
    public int Sequence => Command.Sequence;

    /// <summary>Gets the associated context-local collider identifier.</summary>
    public int ColliderId => Command.ColliderId;

    /// <summary>Gets the associated collider dimension.</summary>
    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    /// <summary>Gets the associated 3D collider type.</summary>
    public ColliderType ColliderType => Command.ColliderType;

    /// <summary>Gets the associated 2D collider type.</summary>
    public ColliderType2D Collider2DType => Command.Collider2DType;

    /// <summary>Gets the capsule center.</summary>
    public Vector3d Center => Command.Center;

    /// <summary>Gets the capsule orientation.</summary>
    public FixedQuaternion Rotation => Command.Rotation;

    /// <summary>Gets the capsule radius.</summary>
    public Fixed64 Radius => Command.Radius;

    /// <summary>
    /// Gets the full distance between the capsule's hemisphere centers.
    /// </summary>
    public Fixed64 AxisLength => Command.AxisLength;

    /// <summary>Gets the capsule color.</summary>
    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a wire-cylinder draw command.
/// </summary>
public readonly struct GravitasWireCylinderDebugDrawView
{
    internal GravitasWireCylinderDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    /// <summary>Gets the underlying draw command.</summary>
    public GravitasDebugDrawCommand Command { get; }

    /// <summary>Gets the simulation frame in which the command was captured.</summary>
    public int Frame => Command.Frame;

    /// <summary>Gets the command's sequence within its capture buffer.</summary>
    public int Sequence => Command.Sequence;

    /// <summary>Gets the associated context-local collider identifier.</summary>
    public int ColliderId => Command.ColliderId;

    /// <summary>Gets the associated collider dimension.</summary>
    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    /// <summary>Gets the associated 3D collider type.</summary>
    public ColliderType ColliderType => Command.ColliderType;

    /// <summary>Gets the associated 2D collider type.</summary>
    public ColliderType2D Collider2DType => Command.Collider2DType;

    /// <summary>Gets the cylinder center.</summary>
    public Vector3d Center => Command.Center;

    /// <summary>Gets the cylinder orientation.</summary>
    public FixedQuaternion Rotation => Command.Rotation;

    /// <summary>Gets the cylinder radius.</summary>
    public Fixed64 Radius => Command.Radius;

    /// <summary>Gets the cylinder height.</summary>
    public Fixed64 Height => Command.Height;

    /// <summary>Gets the cylinder color.</summary>
    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a wire-triangle draw command.
/// </summary>
public readonly struct GravitasWireTriangleDebugDrawView
{
    internal GravitasWireTriangleDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    /// <summary>Gets the underlying draw command.</summary>
    public GravitasDebugDrawCommand Command { get; }

    /// <summary>Gets the simulation frame in which the command was captured.</summary>
    public int Frame => Command.Frame;

    /// <summary>Gets the command's sequence within its capture buffer.</summary>
    public int Sequence => Command.Sequence;

    /// <summary>Gets the associated context-local collider identifier.</summary>
    public int ColliderId => Command.ColliderId;

    /// <summary>Gets the associated collider dimension.</summary>
    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    /// <summary>Gets the associated 3D collider type.</summary>
    public ColliderType ColliderType => Command.ColliderType;

    /// <summary>Gets the associated 2D collider type.</summary>
    public ColliderType2D Collider2DType => Command.Collider2DType;

    /// <summary>Gets the first triangle vertex.</summary>
    public Vector3d PointA => Command.PointA;

    /// <summary>Gets the second triangle vertex.</summary>
    public Vector3d PointB => Command.PointB;

    /// <summary>Gets the third triangle vertex.</summary>
    public Vector3d PointC => Command.PointC;

    /// <summary>Gets the triangle color.</summary>
    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a wire-cone draw command.
/// </summary>
public readonly struct GravitasWireConeDebugDrawView
{
    internal GravitasWireConeDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    /// <summary>Gets the underlying draw command.</summary>
    public GravitasDebugDrawCommand Command { get; }

    /// <summary>Gets the simulation frame in which the command was captured.</summary>
    public int Frame => Command.Frame;

    /// <summary>Gets the command's sequence within its capture buffer.</summary>
    public int Sequence => Command.Sequence;

    /// <summary>Gets the associated context-local collider identifier.</summary>
    public int ColliderId => Command.ColliderId;

    /// <summary>Gets the associated collider dimension.</summary>
    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    /// <summary>Gets the associated 3D collider type.</summary>
    public ColliderType ColliderType => Command.ColliderType;

    /// <summary>Gets the associated 2D collider type.</summary>
    public ColliderType2D Collider2DType => Command.Collider2DType;

    /// <summary>Gets the cone center.</summary>
    public Vector3d Center => Command.Center;

    /// <summary>Gets the cone orientation.</summary>
    public FixedQuaternion Rotation => Command.Rotation;

    /// <summary>Gets the cone base radius.</summary>
    public Fixed64 Radius => Command.Radius;

    /// <summary>Gets the cone height.</summary>
    public Fixed64 Height => Command.Height;

    /// <summary>Gets the cone color.</summary>
    public GravitasDiagnosticColor Color => Command.Color;
}
