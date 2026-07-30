using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

/// <content>
/// Verifies exact cached two-axis Coulomb-disk accumulation and removal.
/// </content>
public sealed partial class ExactContactResponseKernelCoulombTests
{
    [Fact]
    public void CoulombDiskCompositeMaterialization_ShouldHonorHalfEvenLimits()
    {
        ulong[] radicand = { 1UL };
        ulong[] rationalNumerator = { 0UL };
        ulong[] rationalDenominator = { 1UL };

        Assert.True(
            ExactContactResponseKernel.TryGetSignedRadicalAndRationalSum(
                new[] { ulong.MaxValue - 2UL, 1UL },
                new[] { 4UL },
                radicand,
                radicalSign: 1,
                rationalNumerator,
                rationalDenominator,
                rationalSign: 0,
                out Fixed64 positiveBelowMidpoint));
        Assert.Equal(Fixed64.MaxValue, positiveBelowMidpoint);

        Assert.False(
            ExactContactResponseKernel.TryGetSignedRadicalAndRationalSum(
                new[] { ulong.MaxValue },
                new[] { 2UL },
                radicand,
                radicalSign: 1,
                rationalNumerator,
                rationalDenominator,
                rationalSign: 0,
                out _));

        Assert.True(
            ExactContactResponseKernel.TryGetSignedRadicalAndRationalSum(
                new[] { 1UL, 1UL },
                new[] { 2UL },
                radicand,
                radicalSign: -1,
                rationalNumerator,
                rationalDenominator,
                rationalSign: 0,
                out Fixed64 negativeAtMidpoint));
        Assert.Equal(Fixed64.MinValue, negativeAtMidpoint);

        Assert.False(
            ExactContactResponseKernel.TryGetSignedRadicalAndRationalSum(
                new[] { 3UL, 2UL },
                new[] { 4UL },
                radicand,
                radicalSign: -1,
                rationalNumerator,
                rationalDenominator,
                rationalSign: 0,
                out _));
    }

    [Fact]
    public void CoulombDiskResponse_ShouldCancelStaticCacheWithoutAllocating()
    {
        ExactLever3D lever = CreateLever();
        Vector3d velocity =
            (Vector3d.Up * (Fixed64)6) + Vector3d.Right;
        ExactNormalConstraint3D constraint = CreateNormalConstraint(
            lever,
            Fixed64.One,
            Vector3d.Up,
            velocity);
        ExactContactResponseOperand3D primaryFirst = CreateTangentOperand(
            lever,
            velocity,
            Vector3d.Left,
            Fixed64.One);
        ExactContactResponseOperand3D primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One);
        ExactContactResponseOperand3D secondaryFirst = CreateTangentOperand(
            lever,
            velocity,
            -Vector3d.Forward,
            Fixed64.One);
        ExactContactResponseOperand3D secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One);

        ExactCoulombResponse3D response = Solve();
        Assert.True(response.HasAppliedImpulse);
        Assert.Equal(
            Vector3d.Left * Fixed64.Half,
            response.FirstLinearVelocityDelta);
        Assert.True(response.TryGetPrimaryAccumulatedImpulse(
            out Fixed64 primaryAccumulated));
        Assert.Equal(Fixed64.Zero, primaryAccumulated);
        Assert.True(response.TryGetSecondaryAccumulatedImpulse(
            out Fixed64 secondaryAccumulated));
        Assert.Equal(Fixed64.Zero, secondaryAccumulated);

        _ = Solve();
        long before = System.GC.GetAllocatedBytesForCurrentThread();
        for (int iteration = 0; iteration < 8; iteration++)
            _ = Solve();
        Assert.Equal(
            before,
            System.GC.GetAllocatedBytesForCurrentThread());

        ExactCoulombResponse3D Solve()
        {
            Assert.True(ExactContactResponseKernel.TryGetCoulombDiskResponse(
                constraint,
                primaryFirst,
                primarySecond,
                Vector3d.Right,
                -Fixed64.Half,
                secondaryFirst,
                secondarySecond,
                Vector3d.Forward,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out ExactCoulombResponse3D solved));
            return solved;
        }
    }

    [Fact]
    public void CoulombDiskResponse_ShouldRemoveExtremeCachesBeforeNarrowing()
    {
        ExactLever3D lever = CreateLever();
        Fixed64 inverseMass = Fixed64.MinIncrement;
        ExactNormalConstraint3D constraint = CreateNormalConstraint(
            lever,
            inverseMass,
            Vector3d.Up,
            Vector3d.Zero);
        ExactContactResponseOperand3D primaryFirst = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Left,
            inverseMass);
        ExactContactResponseOperand3D primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            inverseMass);
        ExactContactResponseOperand3D secondaryFirst = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            -Vector3d.Forward,
            inverseMass);
        ExactContactResponseOperand3D secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            inverseMass);

        AssertRemoval(Fixed64.MinValue, -Vector3d.Right * Fixed64.Half);
        AssertRemoval(Fixed64.MaxValue, Vector3d.Right * Fixed64.Half);

        void AssertRemoval(Fixed64 cached, Vector3d expectedFirstDelta)
        {
            Assert.True(ExactContactResponseKernel.TryGetCoulombDiskResponse(
                constraint,
                primaryFirst,
                primarySecond,
                Vector3d.Right,
                cached,
                secondaryFirst,
                secondarySecond,
                Vector3d.Forward,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                out ExactCoulombResponse3D response));

            Assert.True(response.HasAppliedImpulse);
            Assert.Equal(
                expectedFirstDelta,
                response.FirstLinearVelocityDelta);
            Assert.Equal(
                -expectedFirstDelta,
                response.SecondLinearVelocityDelta);
            Assert.True(response.TryGetPrimaryAccumulatedImpulse(
                out Fixed64 primaryAccumulated));
            Assert.Equal(Fixed64.Zero, primaryAccumulated);
            Assert.True(response.TryGetSecondaryAccumulatedImpulse(
                out Fixed64 secondaryAccumulated));
            Assert.Equal(Fixed64.Zero, secondaryAccumulated);
        }
    }

    [Fact]
    public void CoulombDiskResponse_ShouldAccumulateAndClampCachedDiskExactly()
    {
        ExactLever3D lever = CreateLever();
        Vector3d velocity =
            (Vector3d.Up * (Fixed64)6)
            + (Vector3d.Right * (Fixed64)8)
            + (Vector3d.Forward * (Fixed64)8);
        ExactNormalConstraint3D constraint = CreateNormalConstraint(
            lever,
            Fixed64.One,
            Vector3d.Up,
            velocity);
        ExactContactResponseOperand3D primaryFirst = CreateTangentOperand(
            lever,
            velocity,
            Vector3d.Left,
            Fixed64.One);
        ExactContactResponseOperand3D primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One);
        ExactContactResponseOperand3D secondaryFirst = CreateTangentOperand(
            lever,
            velocity,
            -Vector3d.Forward,
            Fixed64.One);
        ExactContactResponseOperand3D secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One);
        Fixed64 cachedPrimary = Fixed64.One;
        Fixed64 cachedSecondary = -Fixed64.One;

        Assert.True(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            constraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            cachedPrimary,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            cachedSecondary,
            Fixed64.Zero,
            Fixed64.One / (Fixed64)4,
            out ExactCoulombResponse3D response));

        Assert.True(response.HasAppliedImpulse);
        Assert.True(response.TryGetPrimaryAccumulatedImpulse(
            out Fixed64 primaryAccumulated));
        Assert.True(response.TryGetSecondaryAccumulatedImpulse(
            out Fixed64 secondaryAccumulated));
        Fixed64 accumulatedMagnitude = FixedMath.Sqrt(
            primaryAccumulated * primaryAccumulated
            + secondaryAccumulated * secondaryAccumulated);
        Assert.True(accumulatedMagnitude >= (Fixed64)0.74999999m);
        Assert.True(accumulatedMagnitude <= (Fixed64)0.75000001m);
        Assert.Equal(
            primaryAccumulated,
            cachedPrimary - response.FirstLinearVelocityDelta.X);
        Assert.Equal(
            secondaryAccumulated,
            cachedSecondary - response.FirstLinearVelocityDelta.Z);
        Assert.Equal(
            -response.FirstLinearVelocityDelta,
            response.SecondLinearVelocityDelta);
    }

    [Fact]
    public void CoulombDiskResponse_ShouldDetectSecondaryOnlyDynamicChange()
    {
        ExactLever3D lever = CreateLever();
        Vector3d velocity =
            (Vector3d.Up * (Fixed64)6)
            + (Vector3d.Forward * (Fixed64)8);
        ExactNormalConstraint3D constraint = CreateNormalConstraint(
            lever,
            Fixed64.One,
            Vector3d.Up,
            velocity);
        ExactContactResponseOperand3D primaryFirst = CreateTangentOperand(
            lever,
            velocity,
            Vector3d.Left,
            Fixed64.One);
        ExactContactResponseOperand3D primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One);
        ExactContactResponseOperand3D secondaryFirst = CreateTangentOperand(
            lever,
            velocity,
            -Vector3d.Forward,
            Fixed64.One);
        ExactContactResponseOperand3D secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One);

        Assert.True(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            constraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            Fixed64.Zero,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            -Fixed64.One,
            Fixed64.Zero,
            Fixed64.One / (Fixed64)4,
            out ExactCoulombResponse3D response));

        Assert.True(response.HasAppliedImpulse);
        Assert.True(response.TryGetPrimaryAccumulatedImpulse(
            out Fixed64 primaryAccumulated));
        Assert.Equal(Fixed64.Zero, primaryAccumulated);
        Assert.True(response.TryGetSecondaryAccumulatedImpulse(
            out Fixed64 secondaryAccumulated));
        Assert.Equal((Fixed64)0.75m, secondaryAccumulated);
        Assert.Equal(
            -Vector3d.Forward * (Fixed64)1.75m,
            response.FirstLinearVelocityDelta);
    }

    [Fact]
    public void CoulombDiskResponse_ShouldCombineOpposingCacheComponents()
    {
        ExactLever3D lever = CreateLever();
        Fixed64 threeFifths = (Fixed64)3 / (Fixed64)5;
        Fixed64 fourFifths = (Fixed64)4 / (Fixed64)5;
        Vector3d primaryTangent =
            new(threeFifths, Fixed64.Zero, fourFifths);
        Vector3d secondaryTangent =
            new(fourFifths, Fixed64.Zero, -threeFifths);
        Vector3d velocity =
            (Vector3d.Up * (Fixed64)6)
            + (primaryTangent * (Fixed64)8)
            + (secondaryTangent * (Fixed64)8);
        ExactNormalConstraint3D constraint = CreateNormalConstraint(
            lever,
            Fixed64.One,
            Vector3d.Up,
            velocity);
        ExactContactResponseOperand3D primaryFirst = CreateTangentOperand(
            lever,
            velocity,
            -primaryTangent,
            Fixed64.One);
        ExactContactResponseOperand3D primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            primaryTangent,
            Fixed64.One);
        ExactContactResponseOperand3D secondaryFirst = CreateTangentOperand(
            lever,
            velocity,
            -secondaryTangent,
            Fixed64.One);
        ExactContactResponseOperand3D secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            secondaryTangent,
            Fixed64.One);
        Fixed64 cachedPrimary = Fixed64.One;
        Fixed64 cachedSecondary = -Fixed64.One;

        Assert.True(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            constraint,
            primaryFirst,
            primarySecond,
            primaryTangent,
            cachedPrimary,
            secondaryFirst,
            secondarySecond,
            secondaryTangent,
            cachedSecondary,
            Fixed64.Zero,
            Fixed64.One / (Fixed64)4,
            out ExactCoulombResponse3D response));

        Assert.True(response.HasAppliedImpulse);
        Assert.True(response.TryGetPrimaryAccumulatedImpulse(
            out Fixed64 primaryAccumulated));
        Assert.True(response.TryGetSecondaryAccumulatedImpulse(
            out Fixed64 secondaryAccumulated));
        Fixed64 accumulatedMagnitude = FixedMath.Sqrt(
            primaryAccumulated * primaryAccumulated
            + secondaryAccumulated * secondaryAccumulated);
        Assert.True(accumulatedMagnitude >= (Fixed64)0.74999999m);
        Assert.True(accumulatedMagnitude <= (Fixed64)0.75000001m);
        Assert.Equal(
            -response.FirstLinearVelocityDelta,
            response.SecondLinearVelocityDelta);
    }

    [Fact]
    public void CoulombDiskResponse_ShouldRetainUnrepresentableProjections()
    {
        ExactLever3D lever = CreateLever();
        Fixed64 inverseMass = Fixed64.MinIncrement * Fixed64.Two;
        Vector3d velocity =
            (Vector3d.Up * (Fixed64)6)
            + (Vector3d.Right * (Fixed64)8)
            + (Vector3d.Forward * (Fixed64)8);
        ExactNormalConstraint3D constraint = CreateNormalConstraint(
            lever,
            inverseMass,
            Vector3d.Up,
            velocity);
        ExactContactResponseOperand3D primaryFirst = CreateTangentOperand(
            lever,
            velocity,
            Vector3d.Left,
            inverseMass);
        ExactContactResponseOperand3D primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            inverseMass);
        ExactContactResponseOperand3D secondaryFirst = CreateTangentOperand(
            lever,
            velocity,
            -Vector3d.Forward,
            inverseMass);
        ExactContactResponseOperand3D secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            inverseMass);

        Assert.True(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            constraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            Fixed64.Zero,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            out ExactCoulombResponse3D response));

        Assert.True(response.HasAppliedImpulse);
        Assert.False(response.TryGetPrimaryAccumulatedImpulse(out _));
        Assert.False(response.TryGetSecondaryAccumulatedImpulse(out _));
        Assert.True(
            response.FirstLinearVelocityDelta.Magnitude
            >= (Fixed64)2.99999999m);
        Assert.True(
            response.FirstLinearVelocityDelta.Magnitude
            <= (Fixed64)3.00000001m);
        Assert.Equal(
            -response.FirstLinearVelocityDelta,
            response.SecondLinearVelocityDelta);
    }

    [Fact]
    public void CoulombDiskResponse_ShouldNarrowAfterExactCacheRemoval()
    {
        ExactLever3D lever = CreateLever();
        Fixed64 inverseMass = Fixed64.MaxValue;
        ExactNormalConstraint3D constraint = CreateNormalConstraint(
            lever,
            inverseMass,
            Vector3d.Up,
            Vector3d.Zero,
            accumulatedImpulse: (Fixed64)7);
        ExactContactResponseOperand3D primaryFirst = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Left,
            inverseMass);
        ExactContactResponseOperand3D primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            inverseMass);
        ExactContactResponseOperand3D secondaryFirst = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            -Vector3d.Forward,
            inverseMass);
        ExactContactResponseOperand3D secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            inverseMass);

        Assert.True(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            constraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            (Fixed64)2,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One / (Fixed64)4,
            out ExactCoulombResponse3D response));

        Assert.True(response.HasAppliedImpulse);
        Assert.True(response.TryGetPrimaryAccumulatedImpulse(
            out Fixed64 primaryAccumulated));
        Assert.Equal((Fixed64)1.75m, primaryAccumulated);
        Assert.True(response.TryGetSecondaryAccumulatedImpulse(
            out Fixed64 secondaryAccumulated));
        Assert.Equal(Fixed64.Zero, secondaryAccumulated);
        Assert.Equal(
            Vector3d.Right * (Fixed64)536870912,
            response.FirstLinearVelocityDelta);
        Assert.Equal(
            -response.FirstLinearVelocityDelta,
            response.SecondLinearVelocityDelta);
        Assert.Equal(Vector3d.Zero, response.FirstAngularVelocityDelta);
        Assert.Equal(Vector3d.Zero, response.SecondAngularVelocityDelta);
    }

    [Fact]
    public void CoulombDiskResponse_ShouldRejectOnlyTrueFinalCacheRemovalOverflow()
    {
        ExactLever3D lever = CreateLever();
        ExactNormalConstraint3D constraint = CreateNormalConstraint(
            lever,
            Fixed64.One,
            Vector3d.Up,
            Vector3d.Zero);
        ExactContactResponseOperand3D primaryFirst = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Left,
            Fixed64.One);
        ExactContactResponseOperand3D primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One);
        ExactContactResponseOperand3D secondaryFirst = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            -Vector3d.Forward,
            Fixed64.One);
        ExactContactResponseOperand3D secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One);

        Assert.False(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            constraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            Fixed64.MinValue,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            out _));
    }
}
