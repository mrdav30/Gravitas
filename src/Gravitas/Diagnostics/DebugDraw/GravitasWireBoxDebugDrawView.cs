//=======================================================================
// GravitasWireBoxDebugDrawView.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Diagnostics;

/// <summary>
/// Typed read-only view over a wire-box draw command.
/// </summary>
public readonly struct GravitasWireBoxDebugDrawView
{
    internal GravitasWireBoxDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    /// <summary>Gets the underlying draw command.</summary>
    public GravitasDebugDrawCommand Command { get; }

    /// <summary>Gets the simulation frame in which the command was captured.</summary>
    public long Frame => Command.Frame;

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
