//=======================================================================
// SmallContactNormalSolver3D.cs
//=======================================================================
//
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//
//=======================================================================

using System;
using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>Solves a bounded unilateral normal response for a small 3D contact island.</summary>
internal static class SmallContactNormalSolver3D
{
    internal const int MaxRows = 8;
    private const int Satisfied = -1;
    private const int Invalid = -2;

    // Q32.32 residual allowance in the units of the supplied right-hand side.
    private static readonly Signed192 WideResidualTolerance = WideArithmetic.ShiftLeftSigned192By32(
        Signed192.Raw(Fixed64.FromRaw(1L << 12)));

    internal static bool TrySolve(int rowCount, ReadOnlySpan<Fixed64> matrix,
        ReadOnlySpan<Fixed64> freeSpeeds, Span<Fixed64> impulses)
    {
        if (rowCount is < 1 or > MaxRows || matrix.Length < rowCount * rowCount
            || freeSpeeds.Length < rowCount || impulses.Length < rowCount)
        {
            return false;
        }

        matrix = matrix[..(rowCount * rowCount)];
        freeSpeeds = freeSpeeds[..rowCount];
        Fixed64 maxDiagonal = Fixed64.Zero;
        for (int row = 0; row < rowCount; row++)
        {
            Fixed64 diagonal = matrix[row * rowCount + row];
            if (diagonal <= Fixed64.Zero)
                return false;
            if (diagonal > maxDiagonal)
                maxDiagonal = diagonal;
            for (int column = 0; column < row; column++)
                if (matrix[row * rowCount + column] != matrix[column * rowCount + row])
                    return false;
        }

        // Reject unresolved pivots instead of perturbing the effective mass matrix.
        Fixed64 pivotTolerance = Fixed64.FromRaw(Math.Max(FixedMath.DEFAULT_TOLERANCE_L,
            maxDiagonal.m_rawValue >> 24));
        int setCount = 1 << rowCount;
        Span<bool> visited = stackalloc bool[setCount];
        visited.Clear();
        Span<Fixed64> candidate = stackalloc Fixed64[rowCount];
        Span<Fixed64> working = stackalloc Fixed64[rowCount * rowCount];
        Span<Fixed64> speeds = stackalloc Fixed64[rowCount];
        Span<int> order = stackalloc int[rowCount];

        int mask = 0;
        for (int attempt = 0; attempt < 2 * rowCount && !visited[mask]; attempt++)
        {
            visited[mask] = true;
            if (!TrySolveActiveRows(matrix, freeSpeeds, mask, pivotTolerance, candidate, working, speeds, order))
                break;

            int leaving = FindNegativeImpulse(candidate);
            if (leaving >= 0)
            {
                mask &= ~(1 << leaving);
                continue;
            }

            int entering = FindViolatedRow(matrix, freeSpeeds, candidate);
            if (entering == Satisfied)
            {
                candidate.CopyTo(impulses);
                return true;
            }
            if (entering == Invalid)
                break;
            mask |= 1 << entering;
        }

        // A semidefinite system can have several valid impulse distributions.
        // Try the remaining independent bases in deterministic row-mask order.
        for (mask = 1; mask < setCount; mask++)
        {
            if (visited[mask]
                || !TrySolveActiveRows(matrix, freeSpeeds, mask, pivotTolerance, candidate, working, speeds, order)
                || FindNegativeImpulse(candidate) >= 0
                || FindViolatedRow(matrix, freeSpeeds, candidate) != Satisfied)
            {
                continue;
            }

            candidate.CopyTo(impulses);
            return true;
        }
        return false;
    }

    private static bool TrySolveActiveRows(ReadOnlySpan<Fixed64> matrix, ReadOnlySpan<Fixed64> freeSpeeds,
        int mask, Fixed64 pivotTolerance, Span<Fixed64> candidate, Span<Fixed64> working,
        Span<Fixed64> speeds, Span<int> order)
    {
        candidate.Clear();
        if (mask == 0)
            return true;

        int stride = candidate.Length;
        matrix.CopyTo(working);
        freeSpeeds.CopyTo(speeds);
        int count = 0;
        for (int row = 0; row < stride; row++)
            if ((mask & (1 << row)) != 0)
                order[count++] = row;

        for (int step = 0; step < count; step++)
        {
            int best = step;
            for (int i = step + 1; i < count; i++)
                if (working[order[i] * (stride + 1)] > working[order[best] * (stride + 1)])
                    best = i;
            (order[step], order[best]) = (order[best], order[step]);

            int pivot = order[step];
            int pivotBase = pivot * stride;
            Fixed64 diagonal = working[pivotBase + pivot];
            if (diagonal <= pivotTolerance)
                return false;

            for (int i = step + 1; i < count; i++)
            {
                int row = order[i];
                int rowBase = row * stride;
                if (!Fixed64.TryMultiplyDivide(-Fixed64.One, working[rowBase + pivot], diagonal, out Fixed64 factor)
                    || !Fixed64.TryMultiplyAdd(factor, speeds[pivot], speeds[row], out speeds[row]))
                {
                    return false;
                }

                for (int j = i; j < count; j++)
                {
                    int column = order[j];
                    if (!Fixed64.TryMultiplyAdd(factor, working[pivotBase + column],
                            working[rowBase + column], out Fixed64 reduced))
                    {
                        return false;
                    }
                    working[rowBase + column] = working[column * stride + row] = reduced;
                }
            }
        }

        for (int step = count - 1; step >= 0; step--)
        {
            int row = order[step];
            int rowBase = row * stride;
            Fixed64 speed = speeds[row];
            for (int i = step + 1; i < count; i++)
            {
                int column = order[i];
                if (!Fixed64.TryMultiplyAdd(working[rowBase + column], candidate[column], speed, out speed))
                    return false;
            }
            if (!Fixed64.TryMultiplyDivide(-Fixed64.One, speed, working[rowBase + row], out candidate[row]))
                return false;
        }
        return true;
    }

    private static int FindNegativeImpulse(ReadOnlySpan<Fixed64> candidate)
    {
        Fixed64 lowest = Fixed64.Zero;
        int leaving = -1;
        for (int row = 0; row < candidate.Length; row++)
        {
            if (candidate[row] >= lowest)
                continue;
            lowest = candidate[row];
            leaving = row;
        }
        return leaving;
    }

    private static int FindViolatedRow(ReadOnlySpan<Fixed64> matrix, ReadOnlySpan<Fixed64> freeSpeeds,
        ReadOnlySpan<Fixed64> impulses)
    {
        int stride = freeSpeeds.Length;
        Signed192 negativeTolerance = WideArithmetic.SubtractSigned192(default, WideResidualTolerance);
        Signed192 lowest = negativeTolerance;
        int entering = Satisfied;
        for (int row = 0; row < stride; row++)
        {
            // One exact sum keeps rounding in the solve from masking a closing row.
            Signed192 speed = WideArithmetic.ShiftLeftSigned192By32(Signed192.Raw(freeSpeeds[row]));
            for (int column = 0; column < stride; column++)
                speed = WideArithmetic.AddSigned192(speed,
                    Fixed64.GetExactRawProduct(matrix[row * stride + column], impulses[column]));

            if (impulses[row] > Fixed64.Zero)
            {
                if (WideArithmetic.SubtractSigned192(speed, negativeTolerance).Sign < 0
                    || WideArithmetic.SubtractSigned192(speed, WideResidualTolerance).Sign > 0)
                {
                    return Invalid;
                }
            }
            else if (WideArithmetic.SubtractSigned192(speed, lowest).Sign < 0)
            {
                lowest = speed;
                entering = row;
            }
        }
        return entering;
    }
}
