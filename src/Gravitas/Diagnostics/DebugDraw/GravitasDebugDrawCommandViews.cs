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

    public GravitasDebugDrawCommand Command { get; }

    public int Frame => Command.Frame;

    public int Sequence => Command.Sequence;

    public int ColliderId => Command.ColliderId;

    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    public ColliderType ColliderType => Command.ColliderType;

    public ColliderType2D Collider2DType => Command.Collider2DType;

    public Vector3d Start => Command.Start;

    public Vector3d End => Command.End;

    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a ray draw command.
/// </summary>
public readonly struct GravitasRayDebugDrawView
{
    internal GravitasRayDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    public GravitasDebugDrawCommand Command { get; }

    public int Frame => Command.Frame;

    public int Sequence => Command.Sequence;

    public int ColliderId => Command.ColliderId;

    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    public ColliderType ColliderType => Command.ColliderType;

    public ColliderType2D Collider2DType => Command.Collider2DType;

    public Vector3d Start => Command.Start;

    public Vector3d End => Command.End;

    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a point draw command.
/// </summary>
public readonly struct GravitasPointDebugDrawView
{
    internal GravitasPointDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    public GravitasDebugDrawCommand Command { get; }

    public int Frame => Command.Frame;

    public int Sequence => Command.Sequence;

    public int ColliderId => Command.ColliderId;

    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    public ColliderType ColliderType => Command.ColliderType;

    public ColliderType2D Collider2DType => Command.Collider2DType;

    public Vector3d Center => Command.Center;

    public Fixed64 Radius => Command.Radius;

    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a wire-sphere draw command.
/// </summary>
public readonly struct GravitasWireSphereDebugDrawView
{
    internal GravitasWireSphereDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    public GravitasDebugDrawCommand Command { get; }

    public int Frame => Command.Frame;

    public int Sequence => Command.Sequence;

    public int ColliderId => Command.ColliderId;

    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    public ColliderType ColliderType => Command.ColliderType;

    public ColliderType2D Collider2DType => Command.Collider2DType;

    public Vector3d Center => Command.Center;

    public Fixed64 Radius => Command.Radius;

    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a wire-box draw command.
/// </summary>
public readonly struct GravitasWireBoxDebugDrawView
{
    internal GravitasWireBoxDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    public GravitasDebugDrawCommand Command { get; }

    public int Frame => Command.Frame;

    public int Sequence => Command.Sequence;

    public int ColliderId => Command.ColliderId;

    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    public ColliderType ColliderType => Command.ColliderType;

    public ColliderType2D Collider2DType => Command.Collider2DType;

    public Vector3d Center => Command.Center;

    public Vector3d Size => Command.Size;

    public FixedQuaternion Rotation => Command.Rotation;

    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a wire-capsule draw command.
/// </summary>
public readonly struct GravitasWireCapsuleDebugDrawView
{
    internal GravitasWireCapsuleDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    public GravitasDebugDrawCommand Command { get; }

    public int Frame => Command.Frame;

    public int Sequence => Command.Sequence;

    public int ColliderId => Command.ColliderId;

    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    public ColliderType ColliderType => Command.ColliderType;

    public ColliderType2D Collider2DType => Command.Collider2DType;

    public Vector3d Center => Command.Center;

    public FixedQuaternion Rotation => Command.Rotation;

    public Fixed64 Radius => Command.Radius;

    public Fixed64 Height => Command.Height;

    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a wire-cylinder draw command.
/// </summary>
public readonly struct GravitasWireCylinderDebugDrawView
{
    internal GravitasWireCylinderDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    public GravitasDebugDrawCommand Command { get; }

    public int Frame => Command.Frame;

    public int Sequence => Command.Sequence;

    public int ColliderId => Command.ColliderId;

    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    public ColliderType ColliderType => Command.ColliderType;

    public ColliderType2D Collider2DType => Command.Collider2DType;

    public Vector3d Center => Command.Center;

    public FixedQuaternion Rotation => Command.Rotation;

    public Fixed64 Radius => Command.Radius;

    public Fixed64 Height => Command.Height;

    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a wire-triangle draw command.
/// </summary>
public readonly struct GravitasWireTriangleDebugDrawView
{
    internal GravitasWireTriangleDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    public GravitasDebugDrawCommand Command { get; }

    public int Frame => Command.Frame;

    public int Sequence => Command.Sequence;

    public int ColliderId => Command.ColliderId;

    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    public ColliderType ColliderType => Command.ColliderType;

    public ColliderType2D Collider2DType => Command.Collider2DType;

    public Vector3d PointA => Command.PointA;

    public Vector3d PointB => Command.PointB;

    public Vector3d PointC => Command.PointC;

    public GravitasDiagnosticColor Color => Command.Color;
}

/// <summary>
/// Typed read-only view over a wire-cone draw command.
/// </summary>
public readonly struct GravitasWireConeDebugDrawView
{
    internal GravitasWireConeDebugDrawView(GravitasDebugDrawCommand command) => Command = command;

    public GravitasDebugDrawCommand Command { get; }

    public int Frame => Command.Frame;

    public int Sequence => Command.Sequence;

    public int ColliderId => Command.ColliderId;

    public GravitasColliderDimension ColliderDimension => Command.ColliderDimension;

    public ColliderType ColliderType => Command.ColliderType;

    public ColliderType2D Collider2DType => Command.Collider2DType;

    public Vector3d Center => Command.Center;

    public FixedQuaternion Rotation => Command.Rotation;

    public Fixed64 Radius => Command.Radius;

    public Fixed64 Height => Command.Height;

    public GravitasDiagnosticColor Color => Command.Color;
}
