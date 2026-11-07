using System;

namespace Gravitas.Tests.Determinism;

internal sealed class ReplayBodyState
{
    public int Actor { get; set; }
    public long[] PositionRaw { get; set; } = Array.Empty<long>();
    public long[] RotationRaw { get; set; } = Array.Empty<long>();
    public long[] VelocityRaw { get; set; } = Array.Empty<long>();
    public long[] AngularVelocityRaw { get; set; } = Array.Empty<long>();
    public bool Sleeping { get; set; }
    public int ToiIterations { get; set; }
}
