//=======================================================================
// GravitasQuerySummaryDiagnosticView.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Diagnostics;

/// <summary>
/// Typed read-only view over query reducer quality counters.
/// </summary>
public readonly struct GravitasQuerySummaryDiagnosticView
{
    internal GravitasQuerySummaryDiagnosticView(GravitasDiagnosticEvent diagnosticEvent) => Event = diagnosticEvent;

    /// <summary>Gets the underlying diagnostic event.</summary>
    public GravitasDiagnosticEvent Event { get; }

    /// <summary>Gets the simulation frame in which the summary was captured.</summary>
    public long Frame => Event.Frame;

    /// <summary>Gets the event's sequence within its capture buffer.</summary>
    public int Sequence => Event.Sequence;

    /// <summary>Gets the query source dimension.</summary>
    public GravitasColliderDimension SourceDimension => Event.ColliderADimension;

    /// <summary>Gets the query target dimension.</summary>
    public GravitasColliderDimension TargetDimension => Event.ColliderBDimension;

    /// <summary>Gets the query start point.</summary>
    public Vector3d Start => Event.Start;

    /// <summary>Gets the query endpoint.</summary>
    public Vector3d End => Event.End;

    /// <summary>Gets the number of exact reducer attempts.</summary>
    public int ExactReducerAttempts => Event.DataA;

    /// <summary>Gets the number of accepted hits.</summary>
    public int AcceptedHits => Event.DataB;

    /// <summary>Gets the number of conservative fallback hits.</summary>
    public int FallbackHits => (int)Event.ScalarA;

    /// <summary>Gets the number of rejected conservative candidates.</summary>
    public int RejectedConservativeCandidates => (int)Event.ScalarB;

    /// <summary>Gets whether conservative fallback affected the query.</summary>
    public bool HasConservativeFallback => FallbackHits > 0 || RejectedConservativeCandidates > 0;
}
