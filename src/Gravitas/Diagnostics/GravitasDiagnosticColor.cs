namespace Gravitas.Diagnostics;

/// <summary>
/// Engine-agnostic 8-bit RGBA color for diagnostic draw commands.
/// </summary>
public readonly struct GravitasDiagnosticColor
{
    public GravitasDiagnosticColor(byte r, byte g, byte b, byte a = byte.MaxValue)
    {
        R = r;
        G = g;
        B = b;
        A = a;
    }

    public byte R { get; }

    public byte G { get; }

    public byte B { get; }

    public byte A { get; }

    public uint Rgba =>
        ((uint)R << 24)
        | ((uint)G << 16)
        | ((uint)B << 8)
        | A;

    public static GravitasDiagnosticColor White => new(byte.MaxValue, byte.MaxValue, byte.MaxValue);

    public static GravitasDiagnosticColor Red => new(byte.MaxValue, 0, 0);

    public static GravitasDiagnosticColor Green => new(0, byte.MaxValue, 0);

    public static GravitasDiagnosticColor Blue => new(0, 0, byte.MaxValue);

    public static GravitasDiagnosticColor Yellow => new(byte.MaxValue, byte.MaxValue, 0);

    public static GravitasDiagnosticColor Cyan => new(0, byte.MaxValue, byte.MaxValue);
}
