//=======================================================================
// GravitasDebugDrawCommand.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using System;

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
        GravitasColliderDimension colliderDimension,
        ColliderType colliderType,
        ColliderType2D collider2DType,
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
        ColliderDimension = colliderDimension;
        ColliderType = colliderType;
        Collider2DType = collider2DType;
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

    public GravitasColliderDimension ColliderDimension { get; }

    public ColliderType ColliderType { get; }

    public ColliderType2D Collider2DType { get; }

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

    /// <summary>
    /// Dispatches this draw command to a typed debug draw visitor based on <see cref="Kind"/>.
    /// </summary>
    public void DispatchTo(GravitasDebugDrawCommandVisitor visitor)
    {
        if (visitor == null)
            throw new ArgumentNullException(nameof(visitor));

        switch (Kind)
        {
            case GravitasDebugDrawKind.Line:
                visitor.VisitLine(new GravitasLineDebugDrawView(this));
                break;
            case GravitasDebugDrawKind.Ray:
                visitor.VisitRay(new GravitasRayDebugDrawView(this));
                break;
            case GravitasDebugDrawKind.Point:
                visitor.VisitPoint(new GravitasPointDebugDrawView(this));
                break;
            case GravitasDebugDrawKind.WireSphere:
                visitor.VisitWireSphere(new GravitasWireSphereDebugDrawView(this));
                break;
            case GravitasDebugDrawKind.WireBox:
                visitor.VisitWireBox(new GravitasWireBoxDebugDrawView(this));
                break;
            case GravitasDebugDrawKind.WireCapsule:
                visitor.VisitWireCapsule(new GravitasWireCapsuleDebugDrawView(this));
                break;
            case GravitasDebugDrawKind.WireCylinder:
                visitor.VisitWireCylinder(new GravitasWireCylinderDebugDrawView(this));
                break;
            case GravitasDebugDrawKind.WireTriangle:
                visitor.VisitWireTriangle(new GravitasWireTriangleDebugDrawView(this));
                break;
            default:
                visitor.VisitUnknown(this);
                break;
        }
    }
}
