using System;

namespace Gravitas.Tests.Determinism;

internal sealed class ReplayCommand
{
    public int Frame { get; set; }
    public string Operation { get; set; } = string.Empty;
    public int Actor { get; set; }
    public int OtherActor { get; set; }
    public long[] ValueRaw { get; set; } = Array.Empty<long>();
}
