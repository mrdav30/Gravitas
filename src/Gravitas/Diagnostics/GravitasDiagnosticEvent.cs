using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Diagnostics;

/// <summary>
/// Deterministic physics diagnostic event data captured from one world context.
/// </summary>
public readonly struct GravitasDiagnosticEvent
{
    internal GravitasDiagnosticEvent(
        int frame,
        int sequence,
        GravitasDiagnosticEventKind kind,
        int bodyId,
        int colliderAId,
        int colliderBId,
        ColliderType colliderAType,
        ColliderType colliderBType,
        Vector3d start,
        Vector3d end,
        Vector3d pointA,
        Vector3d pointB,
        Vector3d vector,
        Fixed64 scalarA,
        Fixed64 scalarB,
        int dataA,
        int dataB,
        bool hit)
    {
        Frame = frame;
        Sequence = sequence;
        Kind = kind;
        BodyId = bodyId;
        ColliderAId = colliderAId;
        ColliderBId = colliderBId;
        ColliderAType = colliderAType;
        ColliderBType = colliderBType;
        Start = start;
        End = end;
        PointA = pointA;
        PointB = pointB;
        Vector = vector;
        ScalarA = scalarA;
        ScalarB = scalarB;
        DataA = dataA;
        DataB = dataB;
        Hit = hit;
    }

    public int Frame { get; }

    public int Sequence { get; }

    public GravitasDiagnosticEventKind Kind { get; }

    public int BodyId { get; }

    public int ColliderAId { get; }

    public int ColliderBId { get; }

    public ColliderType ColliderAType { get; }

    public ColliderType ColliderBType { get; }

    public Vector3d Start { get; }

    public Vector3d End { get; }

    public Vector3d PointA { get; }

    public Vector3d PointB { get; }

    public Vector3d Vector { get; }

    public Fixed64 ScalarA { get; }

    public Fixed64 ScalarB { get; }

    public int DataA { get; }

    public int DataB { get; }

    public bool Hit { get; }
}
