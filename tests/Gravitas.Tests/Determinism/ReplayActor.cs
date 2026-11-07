using System;

namespace Gravitas.Tests.Determinism;

internal sealed class ReplayActor
{
    public int Id { get; set; }
    public string Dimension { get; set; } = string.Empty;
    public string Shape { get; set; } = string.Empty;
    public long[] PositionRaw { get; set; } = Array.Empty<long>();
    public long[] SizeRaw { get; set; } = Array.Empty<long>();
    public long MassRaw { get; set; }
    public BodyMotionType MotionType { get; set; }
    public bool Continuous { get; set; }
    public bool StartActive { get; set; } = true;
    public bool Frictionless { get; set; }
    public bool PreventAngularForces { get; set; }
    public int SurfaceGroup { get; set; }
    public long[][] VerticesRaw { get; set; } = Array.Empty<long[]>();
    public int[] Triangles { get; set; } = Array.Empty<int>();
}
