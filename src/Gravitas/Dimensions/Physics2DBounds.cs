using FixedMathSharp;
using SwiftCollections;
using SwiftCollections.Query;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Represents pure 2D broad-phase bounds and their deterministic storage slab.
/// </summary>
public readonly struct Physics2DBounds
{
    private Physics2DBounds(BoundingArea area, Fixed64 planeZ, Fixed64 halfThickness)
    {
        Area = area;
        PlaneZ = planeZ;
        HalfThickness = halfThickness;
    }

    /// <summary>
    /// Gets the normalized 2D area stored in the X/Y axes. The Z component is not part of the 2D shape.
    /// </summary>
    public BoundingArea Area { get; }

    /// <summary>
    /// Gets the storage-plane Z coordinate used when projecting into current fixed broad-phase volumes.
    /// </summary>
    public Fixed64 PlaneZ { get; }

    /// <summary>
    /// Gets the deterministic half thickness used only for broad-phase storage.
    /// </summary>
    public Fixed64 HalfThickness { get; }

    /// <summary>
    /// Creates normalized pure 2D bounds from minimum and maximum corners.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Physics2DBounds FromMinMax(
        Vector2d first,
        Vector2d second,
        Fixed64 planeZ,
        Fixed64 halfThickness)
    {
        ValidateHalfThickness(halfThickness);
        return new Physics2DBounds(
            new BoundingArea(
                new Vector3d(first.x, first.y, Fixed64.Zero),
                new Vector3d(second.x, second.y, Fixed64.Zero)),
            planeZ,
            halfThickness);
    }

    /// <summary>
    /// Creates normalized pure 2D bounds from a center point and non-negative extents.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Physics2DBounds FromCenterExtents(
        Vector2d center,
        Vector2d extents,
        Fixed64 planeZ,
        Fixed64 halfThickness)
    {
        SwiftThrowHelper.ThrowIfArgument(
            extents.x < Fixed64.Zero || extents.y < Fixed64.Zero,
            nameof(extents),
            "2D bounds extents cannot be negative.");
        return FromMinMax(center - extents, center + extents, planeZ, halfThickness);
    }

    /// <summary>
    /// Projects the pure 2D area into the current fixed broad-phase volume shape.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedBoundVolume ToFixedBoundVolume()
    {
        return new FixedBoundVolume(
            new Vector3d(Area.MinX, Area.MinY, PlaneZ - HalfThickness),
            new Vector3d(Area.MaxX, Area.MaxY, PlaneZ + HalfThickness));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ValidateHalfThickness(Fixed64 halfThickness)
    {
        SwiftThrowHelper.ThrowIfArgument(
            halfThickness < Fixed64.Zero,
            nameof(halfThickness),
            "2D bounds halfThickness cannot be negative.");
    }
}
