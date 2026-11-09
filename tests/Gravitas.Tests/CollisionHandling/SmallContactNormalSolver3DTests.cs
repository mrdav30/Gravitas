using System;
using FixedMathSharp;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class SmallContactNormalSolver3DTests
{
    [Fact]
    public void TrySolve_WithCapturedRankSixSupportIsland_ShouldBalanceTheHeavyLoad()
    {
        // Frozen centered 1:100 stack at frame 7. The upper triangle is J M^-1 J^T.
        long[] upper =
        {
            24910867822, 4295040419, 4294951682, -16320875721, -20616661321, -4295032431, 5679304811, -24910777824,
            24910899050, -16320875721, 4294982910, -8589107178, -24910760791, -14936538206, -4294934808,
            24910721576, 4294894173, -891428, 16320737462, 6346585633, -4295057293,
            24910752804, 12026662715, -4294990898, -14269257384, 16320785723,
            18292445849, 8674962776, -1399022273, 20822733945,
            25159728603, 15085835237, 4337876128,
            14759824556, -5736108920,
            25159794938
        };
        long[] free = { -700917714, -700526534, -701855885, -701464704, -4209743016, -4210052715, -4209598816, -4209661535 };
        const int count = 8;
        var matrix = new Fixed64[count * count];
        var speeds = new Fixed64[count];
        int entry = 0;
        for (int row = 0; row < count; row++)
        {
            speeds[row] = Fixed64.FromRaw(free[row]);
            for (int column = row; column < count; column++)
                matrix[row * count + column] = matrix[column * count + row] = Fixed64.FromRaw(upper[entry++]);
        }

        var impulses = new Fixed64[count];
        Assert.True(SmallContactNormalSolver3D.TrySolve(count, matrix, speeds, impulses));

        decimal floorImpulse = 0m;
        decimal interBoxImpulse = 0m;
        for (int row = 0; row < count; row++)
        {
            Assert.True(impulses[row] >= Fixed64.Zero);
            decimal normalSpeed = (decimal)speeds[row];
            for (int column = 0; column < count; column++)
                normalSpeed += (decimal)matrix[row * count + column] * (decimal)impulses[column];
            Assert.True(normalSpeed >= -0.000001m, $"Row {row} still closes at {normalSpeed}.");
            if (impulses[row] > Fixed64.Zero)
                Assert.InRange(normalSpeed, -0.000001m, 0.000001m);

            if (row < 4)
                floorImpulse += (decimal)impulses[row];
            else
                interBoxImpulse += (decimal)impulses[row];
        }
        Assert.InRange(floorImpulse, 75.47319m, 75.47322m);
        Assert.InRange(interBoxImpulse, 75.30993m, 75.30996m);
    }

    [Fact]
    public void TrySolve_WithDuplicateRows_ShouldLeaveTheAlreadySeparatingRowUnloaded()
    {
        Fixed64 one = Fixed64.One;
        Fixed64[] matrix = { one, one, one, one };
        Fixed64[] speeds = { -one, Fixed64.Zero };
        var impulses = new Fixed64[2];

        Assert.True(SmallContactNormalSolver3D.TrySolve(2, matrix, speeds, impulses));

        Assert.Equal(one, impulses[0]);
        Assert.Equal(Fixed64.Zero, impulses[1]);
    }

    [Fact]
    public void TrySolve_WhenTheGreedyRowsBecomeDependent_ShouldFindAnIndependentBasis()
    {
        Fixed64 one = Fixed64.One;
        Fixed64[] matrix =
        {
            (Fixed64)8, (Fixed64)(-4), (Fixed64)4,
            (Fixed64)(-4), (Fixed64)4, (Fixed64)(-2),
            (Fixed64)4, (Fixed64)(-2), (Fixed64)2
        };
        Fixed64[] speeds = { one, (Fixed64)(-2), Fixed64.Zero };
        var impulses = new Fixed64[3];

        Assert.True(SmallContactNormalSolver3D.TrySolve(3, matrix, speeds, impulses));

        Assert.Equal(new[] { Fixed64.Zero, one, one }, impulses);
    }

    [Fact]
    public void TrySolve_WhenGreedyRowChangesReachTheirBudget_ShouldFindTheFeasibleBasis()
    {
        // Gram matrix whose greedy path visits ten distinct bases before
        // reaching the feasible subset. This exercises bounded basis search.
        int[] coefficients =
        {
             31, -12, -19, -19, -3,
            -12,  32,   1,   8, 16,
            -19,   1,  27,   2, 13,
            -19,   8,   2,  21, -6,
             -3,  16,  13,  -6, 26
        };
        Fixed64[] matrix = Array.ConvertAll(coefficients, value => (Fixed64)value);
        Fixed64[] speeds = { (Fixed64)13, (Fixed64)(-18), (Fixed64)(-12), (Fixed64)(-13), (Fixed64)(-18) };
        var impulses = new Fixed64[5];

        Assert.True(SmallContactNormalSolver3D.TrySolve(5, matrix, speeds, impulses));

        Fixed64 tolerance = Fixed64.FromFraction(1, 1_000_000);
        Fixed64[] expected =
        {
            Fixed64.FromFraction(20_984, 5_577), Fixed64.FromFraction(5_836, 5_577),
            Fixed64.FromFraction(474, 169), Fixed64.FromFraction(18_725, 5_577)
        };
        for (int row = 0; row < expected.Length; row++)
            Assert.InRange(impulses[row].m_rawValue, (expected[row] - tolerance).m_rawValue,
                (expected[row] + tolerance).m_rawValue);
        Assert.Equal(Fixed64.Zero, impulses[4]);
    }

    [Fact]
    public void TrySolve_WithOpposingIncompatibleRows_ShouldLeaveTheDestinationUntouched()
    {
        Fixed64 one = Fixed64.One;
        Fixed64[] matrix = { one, -one, -one, one };
        Fixed64[] speeds = { -one, -one };
        Fixed64[] impulses = { one, one };

        Assert.False(SmallContactNormalSolver3D.TrySolve(2, matrix, speeds, impulses));

        Assert.Equal(new[] { one, one }, impulses);
    }

    [Theory]
    [InlineData(-32_769L)]
    [InlineData(-98_303L)]
    public void TrySolve_WhenImpulseRoundingViolatesComplementarity_ShouldLeaveTheDestinationUntouched(long speedRaw)
    {
        // One raw impulse changes normal speed by 65,536 raw units. The nearest
        // impulse misses the allowed speed by more than the tolerance in either direction.
        Fixed64[] matrix = { (Fixed64)65_536 };
        Fixed64[] speeds = { Fixed64.FromRaw(speedRaw) };
        Fixed64[] impulses = { Fixed64.One };

        Assert.False(SmallContactNormalSolver3D.TrySolve(1, matrix, speeds, impulses));

        Assert.Equal(Fixed64.One, impulses[0]);
    }

    [Fact]
    public void TrySolve_WhenRequiredImpulseExceedsFixedPointRange_ShouldLeaveTheDestinationUntouched()
    {
        Fixed64[] matrix = { Fixed64.FromFraction(1, 1024) };
        Fixed64[] speeds = { Fixed64.MinValue };
        Fixed64[] impulses = { Fixed64.One };

        Assert.False(SmallContactNormalSolver3D.TrySolve(1, matrix, speeds, impulses));

        Assert.Equal(Fixed64.One, impulses[0]);
    }

    [Theory]
    [InlineData(-1_500_000_000)]
    [InlineData(-1_000_000_000)]
    public void TrySolve_WhenCoupledIntermediateExceedsFixedPointRange_ShouldLeaveTheDestinationUntouched(int secondSpeed)
    {
        // Both exact impulses fit Q32.32. The first case exceeds its range
        // during elimination and the second during back substitution.
        Fixed64 one = Fixed64.One;
        Fixed64[] matrix = { Fixed64.Two, -one, -one, Fixed64.Two };
        Fixed64[] speeds = { (Fixed64)(-1_500_000_000), (Fixed64)secondSpeed };
        Fixed64[] impulses = { one, Fixed64.Two };

        Assert.False(SmallContactNormalSolver3D.TrySolve(2, matrix, speeds, impulses));

        Assert.Equal(new[] { one, Fixed64.Two }, impulses);
    }

    [Fact]
    public void TrySolve_WithUnresponsiveClosingRow_ShouldLeaveTheDestinationUntouched()
    {
        Fixed64[] matrix = { Fixed64.Zero };
        Fixed64[] speeds = { -Fixed64.One };
        Fixed64[] impulses = { Fixed64.One };

        Assert.False(SmallContactNormalSolver3D.TrySolve(1, matrix, speeds, impulses));

        Assert.Equal(Fixed64.One, impulses[0]);
    }

    [Theory]
    [InlineData(0, 0, 0, 0)]
    [InlineData(9, 81, 9, 9)]
    [InlineData(2, 3, 2, 2)]
    [InlineData(2, 4, 1, 2)]
    [InlineData(2, 4, 2, 1)]
    public void TrySolve_WithUnsupportedRowOrBufferSize_ShouldLeaveTheDestinationUntouched(
        int rowCount, int matrixLength, int speedLength, int impulseLength)
    {
        var matrix = new Fixed64[81];
        for (int row = 0; row < rowCount; row++)
            matrix[row * (rowCount + 1)] = Fixed64.One;
        var speeds = new Fixed64[9];
        var impulses = new Fixed64[9];
        Array.Fill(impulses, Fixed64.One);

        Assert.False(SmallContactNormalSolver3D.TrySolve(rowCount,
            matrix.AsSpan(0, matrixLength), speeds.AsSpan(0, speedLength), impulses.AsSpan(0, impulseLength)));

        Assert.All(impulses, value => Assert.Equal(Fixed64.One, value));
    }

    [Fact]
    public void TrySolve_WithAnAsymmetricMatrix_ShouldLeaveTheDestinationUntouched()
    {
        Fixed64 one = Fixed64.One;
        Fixed64[] matrix = { one, Fixed64.Zero, one, one };
        Fixed64[] speeds = { Fixed64.Zero, Fixed64.Zero };
        Fixed64[] impulses = { one, Fixed64.Two };

        Assert.False(SmallContactNormalSolver3D.TrySolve(2, matrix, speeds, impulses));

        Assert.Equal(new[] { one, Fixed64.Two }, impulses);
    }

    [Theory]
    [InlineData(1024, -10_000_000)]
    [InlineData(1, -65_536)]
    public void TrySolve_WhenIndefiniteMatrixEliminationExceedsRange_ShouldLeaveTheDestinationUntouched(
        int diagonalDivisor, int crossCoefficient)
    {
        Fixed64 diagonal = Fixed64.FromFraction(1, diagonalDivisor);
        Fixed64 cross = (Fixed64)crossCoefficient;
        Fixed64[] matrix = { diagonal, cross, cross, diagonal };
        Fixed64[] speeds = { -Fixed64.One, -Fixed64.One };
        Fixed64[] impulses = { Fixed64.One, Fixed64.Two };

        Assert.False(SmallContactNormalSolver3D.TrySolve(2, matrix, speeds, impulses));

        Assert.Equal(new[] { Fixed64.One, Fixed64.Two }, impulses);
    }
}
