using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

/// <content>Checks retained contact impulses at materialization and final-motion boundaries.</content>
public sealed partial class ExactContactResponseKernelCoulombTests
{
    [Fact]
    public void RetainedNormalResponse_ShouldPreserveRestingImpulseAndExposeItsReleasedZero()
    {
        ExactContactImpulseState? state = null;
        Fixed64 inverse = Fixed64.MinIncrement * Fixed64.Two;
        ExactNormalResponse3D impact = Solve(Vector3d.Right * (Fixed64)8, Vector3d.Zero);
        Assert.False(impact.TryGetAccumulatedImpulse(out _));
        ulong[] retained = state!.NormalNumerator.ToArray();

        ExactNormalResponse3D resting = Solve(Vector3d.Zero, Vector3d.Zero);
        Assert.False(resting.HasAppliedImpulse);
        Assert.False(resting.TryGetAccumulatedImpulse(out _));
        Assert.Equal(retained, state.NormalNumerator.ToArray());

        ExactNormalResponse3D release = Solve(Vector3d.Left * (Fixed64)4, Vector3d.Right * (Fixed64)4);
        Assert.Equal(Vector3d.Right * (Fixed64)4, release.FirstLinearVelocityDelta);
        Assert.Equal(Vector3d.Left * (Fixed64)4, release.SecondLinearVelocityDelta);
        resting = Solve(Vector3d.Zero, Vector3d.Zero);
        Assert.False(resting.HasAppliedImpulse);
        Assert.True(resting.TryGetAccumulatedImpulse(out Fixed64 remaining));
        Assert.Equal(Fixed64.Zero, remaining);

        ExactNormalResponse3D Solve(Vector3d firstVelocity, Vector3d secondVelocity)
        {
            Assert.True(ExactContactResponseKernel.TryGetAccumulatedNormalResponse(
                CreateTangentOperand(CreateLever(), firstVelocity, Vector3d.Left, inverse),
                CreateTangentOperand(CreateLever(), secondVelocity, Vector3d.Right, inverse),
                Vector3d.Right, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.One, Fixed64.One, out ExactNormalResponse3D response, default, ref state));
            return response;
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void RetainedNormalResponse_ShouldRejectFinalMotionOverflowBeforeCreatingState(int boundary)
    {
        Fixed64 inverse = Fixed64.MinIncrement;
        Fixed3x3 inertia = new(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, inverse);
        bool angular = (boundary & 1) != 0;
        bool secondBoundary = boundary >= 2;
        ExactLever3D lever = CreateLever(angular ? Vector3d.Up : Vector3d.Zero);
        Vector3d firstVelocity = Vector3d.Right * (secondBoundary
            ? Fixed64.MaxValue : Fixed64.MinValue + Fixed64.One);
        Vector3d secondVelocity = Vector3d.Right * (secondBoundary
            ? Fixed64.MaxValue - Fixed64.One : Fixed64.MinValue);
        Vector3d firstAngular = Vector3d.Zero;
        Vector3d secondAngular = Vector3d.Zero;
        if (angular)
        {
            firstVelocity = secondBoundary ? Vector3d.Zero : Vector3d.Right * Fixed64.MaxValue;
            secondVelocity = secondBoundary ? Vector3d.Left * Fixed64.MaxValue : Vector3d.Zero;
            firstAngular = secondBoundary ? Vector3d.Zero : Vector3d.Forward * (Fixed64.MaxValue - (Fixed64)8);
            secondAngular = secondBoundary ? -Vector3d.Forward * (Fixed64.MaxValue - (Fixed64)8) : Vector3d.Zero;
        }
        var first = new ExactContactResponseOperand3D(lever, firstVelocity, firstAngular,
            Vector3d.Left, angular && secondBoundary ? Fixed64.Zero : inverse,
            angular && !secondBoundary ? inertia : Fixed3x3.Zero);
        var second = new ExactContactResponseOperand3D(lever, secondVelocity, secondAngular,
            Vector3d.Right, angular && !secondBoundary ? Fixed64.Zero : inverse,
            angular && secondBoundary ? inertia : Fixed3x3.Zero);
        ExactContactImpulseState? state = null;

        // The impulse and each delta fit their own wide/materialized lanes;
        // restitution makes exactly one participant's final velocity overflow.
        Assert.False(ExactContactResponseKernel.TryGetAccumulatedNormalResponse(first, second,
            Vector3d.Right, (Fixed64)3, Fixed64.Zero, Fixed64.Zero,
            Fixed64.One, Fixed64.One, out _, default, ref state));
        Assert.Null(state);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void RetainedDiskResponse_ShouldRejectOverflowWithoutRemovingItsPreviousImpulse(int boundary)
    {
        bool angular = (boundary & 1) != 0;
        bool secondBoundary = boundary >= 2;
        Fixed64 inverse = Fixed64.MinIncrement * Fixed64.Two;
        Fixed3x3 inertia = new(inverse, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
        ExactLever3D lever = CreateLever(angular ? Vector3d.Up : Vector3d.Zero);
        ExactContactImpulseState? state = null;
        Assert.True(Solve(angular ? Vector3d.Zero : Vector3d.Forward * (Fixed64)12,
            Vector3d.Zero, angular && !secondBoundary ? Vector3d.Right * (Fixed64)8 : Vector3d.Zero,
            angular && secondBoundary ? Vector3d.Right * (Fixed64)8 : Vector3d.Zero,
            Fixed64.MaxValue, out _));
        Assert.True(state!.HasTangents);
        ulong[] previous = state.PrimaryMagnitude.ToArray();
        int previousSign = state.PrimarySign;
        Fixed64 linearBoundary = secondBoundary ? Fixed64.MinValue : Fixed64.MaxValue;
        Assert.False(Solve(angular ? Vector3d.Zero : Vector3d.Forward * linearBoundary,
            angular ? Vector3d.Zero : Vector3d.Forward * linearBoundary,
            angular && !secondBoundary ? Vector3d.Right * Fixed64.MaxValue : Vector3d.Zero,
            angular && secondBoundary ? Vector3d.Right * Fixed64.MaxValue : Vector3d.Zero,
            Fixed64.Zero, out _));
        Assert.Equal(previous, state.PrimaryMagnitude.ToArray());
        Assert.Equal(previousSign, state.PrimarySign);
        Assert.True(Solve(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero, Vector3d.Zero,
            Fixed64.Zero, out ExactCoulombResponse3D released));
        Assert.True(released.HasAppliedImpulse);
        Assert.True(released.TryGetPrimaryAccumulatedImpulse(out Fixed64 remaining));
        Assert.Equal(Fixed64.Zero, remaining);
        Assert.Equal(0UL, state.PrimaryMagnitude[0]);
        Assert.Equal(0UL, state.SecondaryMagnitude[0]);

        bool Solve(Vector3d firstLinear, Vector3d secondLinear,
            Vector3d firstAngular, Vector3d secondAngular, Fixed64 friction, out ExactCoulombResponse3D response)
        {
            var first = new ExactContactResponseOperand3D(lever, firstLinear, firstAngular,
                -Vector3d.Forward, angular ? Fixed64.Zero : inverse,
                angular && !secondBoundary ? inertia : Fixed3x3.Zero);
            var second = new ExactContactResponseOperand3D(lever, secondLinear, secondAngular,
                Vector3d.Forward, angular ? Fixed64.Zero : inverse,
                angular && secondBoundary ? inertia : Fixed3x3.Zero);
            var secondaryFirst = new ExactContactResponseOperand3D(lever, firstLinear, firstAngular,
                -Vector3d.Up, first.InverseMass, first.InverseInertia);
            var secondarySecond = new ExactContactResponseOperand3D(lever, secondLinear, secondAngular,
                Vector3d.Up, second.InverseMass, second.InverseInertia);
            return ExactContactResponseKernel.TryGetCoulombDiskResponse(Vector3d.Right, Fixed64.MaxValue,
                first, second, Vector3d.Forward, Fixed64.Zero,
                secondaryFirst, secondarySecond, Vector3d.Up, Fixed64.Zero,
                friction, friction, out response, ref state);
        }
    }

    [Fact]
    public void RetainedStaticDiskResponse_ShouldRoundThirdsOnEitherSideOfTheMidpoint()
    {
        ExactContactImpulseState? state = null;
        Assert.True(Solve((Fixed64)8, out ExactCoulombResponse3D seeded));
        Assert.False(seeded.TryGetPrimaryAccumulatedImpulse(out _));
        Assert.Equal(0x80000002aaaaaaaaUL, state!.PrimaryMagnitude[0]);
        Assert.True(Solve(Fixed64.FromRaw(1048579L), out ExactCoulombResponse3D response));
        Assert.Equal(0x80000002aab00000UL, state.PrimaryMagnitude[0]);
        Assert.Equal(-Vector3d.Forward * Fixed64.FromRaw(349526L), response.FirstLinearVelocityDelta);
        Assert.Equal(Vector3d.Forward * Fixed64.FromRaw(699053L), response.SecondLinearVelocityDelta);

        bool Solve(Fixed64 velocity, out ExactCoulombResponse3D response) =>
            ExactContactResponseKernel.TryGetCoulombDiskResponse(Vector3d.Right, Fixed64.MaxValue,
                CreateTangentOperand(CreateLever(), Vector3d.Forward * velocity, -Vector3d.Forward, Fixed64.One),
                CreateTangentOperand(CreateLever(), Vector3d.Zero, Vector3d.Forward, Fixed64.Two),
                Vector3d.Forward, Fixed64.MaxValue,
                CreateTangentOperand(CreateLever(), Vector3d.Forward * velocity, -Vector3d.Up, Fixed64.One),
                CreateTangentOperand(CreateLever(), Vector3d.Zero, Vector3d.Up, Fixed64.Two),
                Vector3d.Up, Fixed64.Zero, Fixed64.MaxValue, Fixed64.MaxValue, out response, ref state);
    }

    [Theory]
    [InlineData(1048577L, 0x8000000080080000UL)]
    [InlineData(1048579L, 0x8000000080080002UL)]
    public void RetainedStaticDiskResponse_ShouldRoundHalfEvenAboveTheSignedRange(long velocityRaw, ulong expected)
    {
        ExactContactImpulseState? state = null;
        Fixed64 cached = Fixed64.MaxValue;
        Assert.True(Solve(Fixed64.FromRaw(4294967298L), out ExactCoulombResponse3D seeded));
        Assert.False(seeded.TryGetPrimaryAccumulatedImpulse(out _));
        Assert.Equal(0x8000000080000000UL, state!.PrimaryMagnitude[0]);
        Assert.True(Solve(Fixed64.FromRaw(velocityRaw), out ExactCoulombResponse3D response));
        Assert.False(response.TryGetPrimaryAccumulatedImpulse(out _));
        Assert.Equal(expected, state.PrimaryMagnitude[0]);
        Assert.Equal(-response.FirstLinearVelocityDelta, response.SecondLinearVelocityDelta);

        bool Solve(Fixed64 velocity, out ExactCoulombResponse3D response) =>
            ExactContactResponseKernel.TryGetCoulombDiskResponse(Vector3d.Right, Fixed64.MaxValue,
                CreateTangentOperand(CreateLever(), Vector3d.Forward * velocity, -Vector3d.Forward, Fixed64.One),
                CreateTangentOperand(CreateLever(), Vector3d.Zero, Vector3d.Forward, Fixed64.One),
                Vector3d.Forward, cached,
                CreateTangentOperand(CreateLever(), Vector3d.Forward * velocity, -Vector3d.Up, Fixed64.One),
                CreateTangentOperand(CreateLever(), Vector3d.Zero, Vector3d.Up, Fixed64.One),
                Vector3d.Up, Fixed64.Zero, Fixed64.MaxValue, Fixed64.MaxValue, out response, ref state);
    }

    [Theory]
    [InlineData(-1L, 0xfffffffe80000000UL)]
    [InlineData(1L, 0xffffffff80000000UL)]
    public void RetainedSlidingDiskResponse_ShouldRoundRadialMidpointsAndClearItsZeroAxis(long normalOffset, ulong expected)
    {
        foreach (int direction in new[] { -1, 1 })
        {
            ExactContactImpulseState? state = null;
            Fixed64 friction = Fixed64.FromRaw(0x7fffffff80000000L);
            Fixed64 completedNormal = Fixed64.FromRaw(8589934592L + normalOffset);
            Fixed64 inverse = Fixed64.MinIncrement;
            Vector3d velocity = Vector3d.Forward * (Fixed64)(12 * direction);
            Assert.True(ExactContactResponseKernel.TryGetCoulombDiskResponse(Vector3d.Right, completedNormal,
                CreateTangentOperand(CreateLever(), velocity, -Vector3d.Forward, inverse),
                CreateTangentOperand(CreateLever(), Vector3d.Zero, Vector3d.Forward, inverse),
                Vector3d.Forward, Fixed64.Zero,
                CreateTangentOperand(CreateLever(), velocity, -Vector3d.Up, inverse),
                CreateTangentOperand(CreateLever(), Vector3d.Zero, Vector3d.Up, inverse),
                Vector3d.Up, Fixed64.Zero, Fixed64.Zero, friction, out ExactCoulombResponse3D response, ref state));
            Assert.False(response.TryGetPrimaryAccumulatedImpulse(out _));
            Assert.Equal(expected, state!.PrimaryMagnitude[0]);
            Assert.Equal(direction, state.PrimarySign);
            Assert.Equal(0UL, state.SecondaryMagnitude[0]);
            Assert.True(response.TryGetSecondaryAccumulatedImpulse(out Fixed64 secondary));
            Assert.Equal(Fixed64.Zero, secondary);
            Assert.Equal(-response.FirstLinearVelocityDelta, response.SecondLinearVelocityDelta);
            Assert.Equal(Vector3d.Zero, response.FirstAngularVelocityDelta);
        }
    }

    [Fact]
    public void PlanarAngularOperand_ShouldRejectBothDeltaAndFinalYawOverflow()
    {
        var minimum = new ExactContactResponseOperand3D(CreateLever(), Vector3d.Zero,
            Vector3d.Zero, Vector3d.Zero, Fixed64.Zero, Fixed3x3.Zero, Fixed64.MinValue);
        var maximum = new ExactContactResponseOperand3D(CreateLever(), Vector3d.Zero,
            Vector3d.Zero, Vector3d.Zero, Fixed64.Zero, Fixed3x3.Zero, Fixed64.MaxValue);
        Assert.False(minimum.CanApplyAngularDelta(new Vector3d(Fixed64.Zero, Fixed64.MinValue, Fixed64.Zero)));
        Assert.False(minimum.CanApplyAngularDelta(Vector3d.Up * Fixed64.MinIncrement));
        Assert.False(maximum.CanApplyAngularDelta(-Vector3d.Up * Fixed64.MinIncrement));
        Assert.True(minimum.CanApplyAngularDelta(-Vector3d.Up * Fixed64.MinIncrement));
        Assert.True(maximum.CanApplyAngularDelta(Vector3d.Up * Fixed64.MinIncrement));
    }
}
