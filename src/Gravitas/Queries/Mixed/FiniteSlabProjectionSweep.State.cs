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
        private ClosestPlanarSimplexResult(bool intersects, Vector2d point, Vector2d searchDirection)
        {
            Intersects = intersects;
            Point = point;
            SearchDirection = searchDirection;
        }

        public bool Intersects { get; }

        public Vector2d Point { get; }

        public Vector2d SearchDirection { get; }

        public static ClosestPlanarSimplexResult Intersection => new(true, Vector2d.Zero, Vector2d.Zero);

        public static ClosestPlanarSimplexResult FromPoint(Vector2d point) => new(false, point, -point);

        public static ClosestPlanarSimplexResult FromSegment(Vector2d point, Vector2d a, Vector2d b)
        {
            int side = Vector2d.OrientationSign(a, b, Vector2d.Zero);
            if (side == 0)
                return Intersection;
            // A rounded near-zero closest point can erase a normal component;
            // the edge's perpendicular retains its direction independently.
            Vector2d edgeDirection = Vector2d.TrySubtract(b, a, out Vector2d edge)
                ? edge : Vector2d.GetDirection(a, b);
            Vector2d searchDirection = side > 0
                ? new Vector2d(-edgeDirection.Y, edgeDirection.X)
                : new Vector2d(edgeDirection.Y, -edgeDirection.X);
            return new ClosestPlanarSimplexResult(false, point, searchDirection);
        }
    }
}
