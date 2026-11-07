using System;

namespace Gravitas.Tests.Determinism;

internal sealed class ReplayFrame
{
    public long FrameCount { get; set; }
    public long LateToken { get; set; }
    public string Authoritative { get; set; } = string.Empty;
    public string? SolverCaches { get; set; }
    public ReplayBodyState[] Bodies { get; set; } = Array.Empty<ReplayBodyState>();
    public string[] Events { get; set; } = Array.Empty<string>();
    public string[] Queries { get; set; } = Array.Empty<string>();
}
