//=======================================================================
// GravitasDiagnosticColor.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Diagnostics;

/// <summary>
/// Engine-agnostic 8-bit RGBA color for diagnostic draw commands.
/// </summary>
public readonly struct GravitasDiagnosticColor
{
    /// <summary>Creates a color from red, green, blue, and alpha components.</summary>
    public GravitasDiagnosticColor(byte r, byte g, byte b, byte a = byte.MaxValue)
    {
        R = r;
        G = g;
        B = b;
        A = a;
    }

    /// <summary>Gets the red component.</summary>
    public byte R { get; }

    /// <summary>Gets the green component.</summary>
    public byte G { get; }

    /// <summary>Gets the blue component.</summary>
    public byte B { get; }

    /// <summary>Gets the alpha component.</summary>
    public byte A { get; }

    /// <summary>Gets the packed RGBA value with red in the most significant byte.</summary>
    public uint Rgba =>
        ((uint)R << 24)
        | ((uint)G << 16)
        | ((uint)B << 8)
        | A;

    /// <summary>Gets opaque white.</summary>
    public static GravitasDiagnosticColor White => new(byte.MaxValue, byte.MaxValue, byte.MaxValue);

    /// <summary>Gets opaque red.</summary>
    public static GravitasDiagnosticColor Red => new(byte.MaxValue, 0, 0);

    /// <summary>Gets opaque green.</summary>
    public static GravitasDiagnosticColor Green => new(0, byte.MaxValue, 0);

    /// <summary>Gets opaque blue.</summary>
    public static GravitasDiagnosticColor Blue => new(0, 0, byte.MaxValue);

    /// <summary>Gets opaque yellow.</summary>
    public static GravitasDiagnosticColor Yellow => new(byte.MaxValue, byte.MaxValue, 0);

    /// <summary>Gets opaque cyan.</summary>
    public static GravitasDiagnosticColor Cyan => new(0, byte.MaxValue, byte.MaxValue);
}
