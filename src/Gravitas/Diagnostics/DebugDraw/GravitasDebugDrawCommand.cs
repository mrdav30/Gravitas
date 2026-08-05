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
        Vector3d halfExtents,
        Vector3d pointA,
        Vector3d pointB,
        Vector3d pointC,
        FixedQuaternion rotation,
        Fixed64 radius,
        Fixed64 axisLength,
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
        HalfExtents = halfExtents;
        PointA = pointA;
        PointB = pointB;
        PointC = pointC;
        Rotation = rotation;
        Radius = radius;
        AxisLength = axisLength;
        Height = height;
        Color = color;
    }

    /// <summary>Gets the simulation frame in which the command was captured.</summary>
    public int Frame { get; }

    /// <summary>Gets the command's sequence within its capture buffer.</summary>
    public int Sequence { get; }

    /// <summary>Gets the command kind that determines how its payload is interpreted.</summary>
    public GravitasDebugDrawKind Kind { get; }

    /// <summary>Gets the context-local collider identifier, or <c>-1</c> when unassociated.</summary>
    public int ColliderId { get; }

    /// <summary>Gets the dimensional runtime surface of the associated collider.</summary>
    public GravitasColliderDimension ColliderDimension { get; }

    /// <summary>Gets the associated 3D collider type.</summary>
    public ColliderType ColliderType { get; }

    /// <summary>Gets the associated 2D collider type.</summary>
    public ColliderType2D Collider2DType { get; }

    /// <summary>Gets the start point for line and ray commands.</summary>
    public Vector3d Start { get; }

    /// <summary>Gets the end point for line and ray commands.</summary>
    public Vector3d End { get; }

    /// <summary>Gets the center of a point or wireframe volume.</summary>
    public Vector3d Center { get; }

    /// <summary>
    /// Gets the center-relative half-extents for
    /// <see cref="GravitasDebugDrawKind.WireBox"/> commands.
    /// </summary>
    public Vector3d HalfExtents { get; }

    /// <summary>Gets the first vertex of a wireframe triangle.</summary>
    public Vector3d PointA { get; }

    /// <summary>Gets the second vertex of a wireframe triangle.</summary>
    public Vector3d PointB { get; }

    /// <summary>Gets the third vertex of a wireframe triangle.</summary>
    public Vector3d PointC { get; }

    /// <summary>Gets the orientation of a wireframe volume.</summary>
    public FixedQuaternion Rotation { get; }

    /// <summary>Gets the point or wireframe volume radius.</summary>
    public Fixed64 Radius { get; }

    /// <summary>
    /// Gets the full distance between hemisphere centers for
    /// <see cref="GravitasDebugDrawKind.WireCapsule"/> commands.
    /// </summary>
    public Fixed64 AxisLength { get; }

    /// <summary>Gets the full height of a cylinder or cone.</summary>
    public Fixed64 Height { get; }

    /// <summary>Gets the command color.</summary>
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
            case GravitasDebugDrawKind.WireCone:
                visitor.VisitWireCone(new GravitasWireConeDebugDrawView(this));
                break;
            default:
                visitor.VisitUnknown(this);
                break;
        }
    }
}
