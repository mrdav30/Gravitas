//=======================================================================
// FiniteSlabProjectionSweep.State.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Queries;

internal static partial class FiniteSlabProjectionSweep
{
    private readonly struct PlanarSupportPoint
    {
        public PlanarSupportPoint(Vector2d point)
        {
            Point = point;
        }

        public Vector2d Point { get; }
    }

    private readonly struct PlanarGjkResult
    {
        public PlanarGjkResult(Fixed64 distance, Vector2d normal)
        {
            Distance = distance;
            Normal = normal;
        }

        public Fixed64 Distance { get; }

        public Vector2d Normal { get; }

        public static PlanarGjkResult Intersection => new(Fixed64.Zero, Vector2d.Zero);
    }

    private readonly struct ClosestPlanarSimplexResult
    {
        private ClosestPlanarSimplexResult(bool intersects, Vector2d point)
        {
            Intersects = intersects;
            Point = point;
            DistanceSqr = point.MagnitudeSquared;
        }

        public bool Intersects { get; }

        public Vector2d Point { get; }

        public Fixed64 DistanceSqr { get; }

        public static ClosestPlanarSimplexResult Intersection => new(true, Vector2d.Zero);

        public static ClosestPlanarSimplexResult FromPoint(Vector2d point) => new(false, point);
    }
}
