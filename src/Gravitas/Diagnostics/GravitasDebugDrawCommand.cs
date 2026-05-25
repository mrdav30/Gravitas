using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Diagnostics;

/// <summary>
/// Engine-agnostic debug draw command that host adapters can translate into renderer calls.
/// </summary>
public readonly struct GravitasDebugDrawCommand
{
    internal GravitasDebugDrawCommand(
        int frame,
        int sequence,
        GravitasDebugDrawKind kind,
        int colliderId,
        ColliderType colliderType,
        Vector3d start,
        Vector3d end,
        Vector3d center,
        Vector3d size,
        Vector3d pointA,
        Vector3d pointB,
        Vector3d pointC,
        FixedQuaternion rotation,
        Fixed64 radius,
        Fixed64 height,
        GravitasDiagnosticColor color)
    {
        Frame = frame;
        Sequence = sequence;
        Kind = kind;
        ColliderId = colliderId;
        ColliderType = colliderType;
        Start = start;
        End = end;
        Center = center;
        Size = size;
        PointA = pointA;
        PointB = pointB;
        PointC = pointC;
        Rotation = rotation;
        Radius = radius;
        Height = height;
        Color = color;
    }

    public int Frame { get; }

    public int Sequence { get; }

    public GravitasDebugDrawKind Kind { get; }

    public int ColliderId { get; }

    public ColliderType ColliderType { get; }

    public Vector3d Start { get; }

    public Vector3d End { get; }

    public Vector3d Center { get; }

    public Vector3d Size { get; }

    public Vector3d PointA { get; }

    public Vector3d PointB { get; }

    public Vector3d PointC { get; }

    public FixedQuaternion Rotation { get; }

    public Fixed64 Radius { get; }

    public Fixed64 Height { get; }

    public GravitasDiagnosticColor Color { get; }
}
