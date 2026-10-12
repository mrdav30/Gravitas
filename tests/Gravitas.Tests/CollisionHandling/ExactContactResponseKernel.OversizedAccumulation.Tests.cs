using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

/// <summary>Checks retained impulse rows when mass hides a large impulse behind a small velocity change.</summary>
public sealed class ExactContactResponseKernelOversizedTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PlanarNormalResponse_ShouldRejectAnUnrepresentableAngularDelta(bool planarFirst)
    {
        ExactLever3D lever = Lever(Vector3d.Right);
        Vector3d normal = planarFirst ? Vector3d.Backward : Vector3d.Forward;
        Fixed3x3 inertia = new(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
        var first = new ExactContactResponseOperand3D(lever, Vector3d.Zero, Vector3d.Zero,
            -normal, Fixed64.Zero, planarFirst ? inertia : Fixed3x3.Zero,
            planarFirst ? Fixed64.MinValue : null);
        var second = new ExactContactResponseOperand3D(lever, Vector3d.Zero, Vector3d.Zero,
            normal, Fixed64.Zero, planarFirst ? Fixed3x3.Zero : inertia,
            planarFirst ? null : Fixed64.MinValue);

        // Stopping minimum yaw needs positive 2^31 planar delta. The final yaw
        // fits, but the existing delta API must reject it rather than saturate.
        Assert.False(ExactContactResponseKernel.TryGetNormalResponse(first, second,
            normal, Fixed64.Zero, Fixed64.Zero, out _));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void PlanarFrictionResponse_ShouldRejectAnUnrepresentableAngularDelta(bool planarFirst, bool disk)
    {
        ExactLever3D lever = Lever(Vector3d.Right);
        Vector3d tangent = planarFirst ? Vector3d.Backward : Vector3d.Forward;
        Vector3d velocity = Vector3d.Right * Fixed64.MaxValue;
        Fixed3x3 inertia = new(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
        foreach (int offset in new[] { 0, 1 })
        {
            Fixed64 yaw = Fixed64.FromRaw(long.MinValue + offset);
            var first = new ExactContactResponseOperand3D(lever, velocity, Vector3d.Zero,
                Vector3d.Zero, planarFirst ? Fixed64.One : Fixed64.Zero,
                planarFirst ? inertia : Fixed3x3.Zero, planarFirst ? yaw : null);
            var second = new ExactContactResponseOperand3D(lever, Vector3d.Zero, Vector3d.Zero,
                Vector3d.Zero, planarFirst ? Fixed64.Zero : Fixed64.One,
                planarFirst ? Fixed3x3.Zero : inertia, planarFirst ? null : yaw);
            if (disk)
            {
                Assert.Equal(offset != 0, ExactContactResponseKernel.TryGetCoulombDiskResponse(Vector3d.Right, Fixed64.MaxValue,
                    first, second, tangent, Fixed64.Zero, first, second, Vector3d.Up, Fixed64.Zero,
                    Fixed64.Two, Fixed64.Two, out _));
            }
            else
            {
                var normalFirst = new ExactContactResponseOperand3D(lever, velocity, Vector3d.Zero,
                    Vector3d.Left, first.InverseMass, first.InverseInertia, first.PlanarAngularVelocity);
                var normalSecond = new ExactContactResponseOperand3D(lever, Vector3d.Zero, Vector3d.Zero,
                    Vector3d.Right, second.InverseMass, second.InverseInertia, second.PlanarAngularVelocity);
                var normal = new ExactNormalConstraint3D(normalFirst, normalSecond, Vector3d.Right,
                    Fixed64.Zero, Fixed64.Zero, Fixed64.MaxValue, Fixed64.One, Fixed64.One);
                Assert.Equal(offset != 0, ExactContactResponseKernel.TryGetCoulombLineResponse(normal, first, second,
                    tangent, Fixed64.Zero, Fixed64.Two, Fixed64.Two, out _));
            }
        }
    }

    [Fact]
    public void RepresentableRows_ShouldKeepRetainedStateNullAndAllocateNothingAfterWarmup()
    {
        ExactContactImpulseState? state = null;
        ExactLever3D lever = Lever(Vector3d.Zero);
        var first = new ExactContactResponseOperand3D(lever, Vector3d.Right + Vector3d.Forward,
            Vector3d.Zero, Vector3d.Left, Fixed64.One, Fixed3x3.Zero);
        var second = new ExactContactResponseOperand3D(lever, Vector3d.Zero, Vector3d.Zero,
            Vector3d.Right, Fixed64.One, Fixed3x3.Zero);
        var primaryFirst = new ExactContactResponseOperand3D(lever, first.LinearVelocity,
            Vector3d.Zero, -Vector3d.Forward, Fixed64.One, Fixed3x3.Zero);
        var primarySecond = new ExactContactResponseOperand3D(lever, Vector3d.Zero,
            Vector3d.Zero, Vector3d.Forward, Fixed64.One, Fixed3x3.Zero);
        var secondaryFirst = new ExactContactResponseOperand3D(lever, first.LinearVelocity,
            Vector3d.Zero, -Vector3d.Up, Fixed64.One, Fixed3x3.Zero);
        var secondarySecond = new ExactContactResponseOperand3D(lever, Vector3d.Zero,
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed3x3.Zero);
        for (int i = 0; i < 16; i++) Assert.True(Solve());
        Assert.Null(state);
        long before = System.GC.GetAllocatedBytesForCurrentThread();
        bool resolved = true;
        for (int i = 0; i < 64; i++) resolved &= Solve();
        long allocated = System.GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.True(resolved);
        Assert.Null(state);
        Assert.Equal(0, allocated);

        bool Solve() => ExactContactResponseKernel.TryGetAccumulatedNormalResponse(first, second,
            Vector3d.Right, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One,
            out _, default, ref state)
            && ExactContactResponseKernel.TryGetCoulombDiskResponse(Vector3d.Right, Fixed64.Half,
                primaryFirst, primarySecond, Vector3d.Forward, Fixed64.Zero,
                secondaryFirst, secondarySecond, Vector3d.Up, Fixed64.Zero,
                Fixed64.One, Fixed64.One, out _, ref state);
    }

    [Fact]
    public void OversizedNormal_ShouldRetractAndClampItsPreviousRowImpulse()
    {
        ExactContactImpulseState? state = null;
        ExactLever3D lever = Lever(Vector3d.Zero);
        var impact = Impact(Vector3d.Right * (Fixed64)8, Vector3d.Zero);
        ExactNormalResponse3D first = Solve(Vector3d.Right * (Fixed64)8, Vector3d.Zero);
        Assert.False(first.TryGetAccumulatedImpulse(out _));
        Assert.True(state!.HasNormal);
        Assert.Equal(Vector3d.Left * (Fixed64)4, first.FirstLinearVelocityDelta);
        Assert.Equal(Vector3d.Right * (Fixed64)4, first.SecondLinearVelocityDelta);

        // A different constraint can make this row separate; retract its own
        // previous impulse instead of treating an unprojectable accumulator as zero.
        ExactNormalResponse3D retract = Solve(Vector3d.Left * Fixed64.Two, Vector3d.Right * Fixed64.Two);
        Assert.Equal(Vector3d.Right * Fixed64.Two, retract.FirstLinearVelocityDelta);
        Assert.Equal(Vector3d.Left * Fixed64.Two, retract.SecondLinearVelocityDelta);
        ExactNormalResponse3D clamp = Solve(Vector3d.Left * (Fixed64)4, Vector3d.Right * (Fixed64)4);
        Assert.Equal(Vector3d.Right * Fixed64.Two, clamp.FirstLinearVelocityDelta);
        Assert.Equal(Vector3d.Left * Fixed64.Two, clamp.SecondLinearVelocityDelta);
        Assert.True(clamp.TryGetAccumulatedImpulse(out Fixed64 remaining));
        Assert.Equal(Fixed64.Zero, remaining);
        state!.Clear();
        Assert.False(state!.HasNormal);
        Assert.False(state!.HasTangents);
        Assert.True(state.NormalNumerator.IsEmpty);
        Assert.True(state.NormalDenominator.IsEmpty);
        Assert.True(state.PrimaryMagnitude.IsEmpty);
        Assert.True(state.SecondaryMagnitude.IsEmpty);

        ExactNormalResponse3D Solve(Vector3d a, Vector3d b)
        {
            Assert.True(ExactContactResponseKernel.TryGetAccumulatedNormalResponse(
                Operand(lever, a, -Vector3d.Right), Operand(lever, b, Vector3d.Right),
                Vector3d.Right, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.One, Fixed64.One, out ExactNormalResponse3D response, impact, ref state));
            return response;
        }
    }

    [Fact]
    public void OversizedStaticFriction_ShouldRetainBothTangentsWhenAnotherRowReversesMotion()
    {
        ExactContactImpulseState? state = null;
        SeedNormal(ref state);
        ExactCoulombResponse3D first = SolveDisk(ref state,
            Vector3d.Forward * (Fixed64)12 + Vector3d.Up * (Fixed64)8, Vector3d.Zero,
            (Fixed64)4, (Fixed64)4);
        Assert.False(first.TryGetPrimaryAccumulatedImpulse(out _));
        Assert.False(first.TryGetSecondaryAccumulatedImpulse(out _));
        Assert.True(state!.HasTangents);
        Assert.Equal(-Vector3d.Forward * (Fixed64)6 - Vector3d.Up * (Fixed64)4,
            first.FirstLinearVelocityDelta);
        Assert.Equal(Vector3d.Forward * (Fixed64)6 + Vector3d.Up * (Fixed64)4,
            first.SecondLinearVelocityDelta);

        ExactCoulombResponse3D retract = SolveDisk(ref state,
            -Vector3d.Forward * Fixed64.Two - Vector3d.Up * Fixed64.Two,
            Vector3d.Forward * Fixed64.Two + Vector3d.Up * Fixed64.Two, (Fixed64)4, (Fixed64)4);
        Assert.Equal(Vector3d.Forward * Fixed64.Two + Vector3d.Up * Fixed64.Two,
            retract.FirstLinearVelocityDelta);
        Assert.Equal(-Vector3d.Forward * Fixed64.Two - Vector3d.Up * Fixed64.Two,
            retract.SecondLinearVelocityDelta);
    }

    [Fact]
    public void OversizedSlidingDisk_ShouldKeepItsRoundedCacheAcrossRepeatedRadialProjection()
    {
        ExactContactImpulseState? state = null;
        SeedNormal(ref state);
        Vector3d velocity = Vector3d.Forward * (Fixed64)12 + Vector3d.Up * (Fixed64)5;
        ExactCoulombResponse3D first = SolveDisk(ref state, velocity, Vector3d.Zero, Fixed64.Half, Fixed64.Half);
        Assert.False(first.TryGetPrimaryAccumulatedImpulse(out _));
        Assert.True(first.TryGetSecondaryAccumulatedImpulse(out _));
        Assert.True(state!.HasTangents);
        Vector3d delta = -Vector3d.Forward * Fixed64.FromFraction(24, 13)
            - Vector3d.Up * Fixed64.FromFraction(10, 13);
        Assert.Equal(delta, first.FirstLinearVelocityDelta);
        Assert.Equal(-delta, first.SecondLinearVelocityDelta);

        ExactCoulombResponse3D repeated = SolveDisk(ref state, velocity + delta, -delta, Fixed64.Half, Fixed64.Half);
        Assert.Equal(Vector3d.Zero, repeated.FirstLinearVelocityDelta);
        Assert.Equal(Vector3d.Zero, repeated.SecondLinearVelocityDelta);
    }

    [Fact]
    public void OversizedAngularSlidingRow_ShouldSubtractItsCachedTorqueWithFrozenLinearAxes()
    {
        ExactLever3D lever = Lever(Vector3d.Up);
        Fixed64 inverse = Fixed64.MinIncrement;
        var inertia = new Fixed3x3(inverse, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, inverse);
        ExactContactImpulseState? state = null;
        Vector3d angular = Vector3d.Right * (Fixed64)8 - Vector3d.Forward * (Fixed64)8;
        ExactContactResponseOperand3D first = Angular(angular);
        var second = new ExactContactResponseOperand3D(lever, Vector3d.Zero, Vector3d.Zero,
            Vector3d.Zero, Fixed64.Zero, Fixed3x3.Zero);
        Assert.True(ExactContactResponseKernel.TryGetAccumulatedNormalResponse(first, second,
            Vector3d.Right, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One,
            out ExactNormalResponse3D normal, Impact(Vector3d.Zero, angular), ref state));
        Assert.Equal(Vector3d.Forward * (Fixed64)8, normal.FirstAngularVelocityDelta);
        Assert.True(state!.HasNormal);
        ExactCoulombResponse3D friction = Solve(Angular(Vector3d.Right * (Fixed64)8));
        Assert.Equal(Vector3d.Left * (Fixed64)4, friction.FirstAngularVelocityDelta);
        Assert.True(state!.HasTangents);
        ExactCoulombResponse3D repeated = Solve(Angular(Vector3d.Right * (Fixed64)3));
        Assert.Equal(Vector3d.Zero, repeated.FirstAngularVelocityDelta);
        Assert.Equal(Vector3d.Zero, repeated.FirstLinearVelocityDelta);

        ExactContactResponseOperand3D Angular(Vector3d value) =>
            new(lever, Vector3d.Zero, value, Vector3d.Zero, Fixed64.Zero, inertia);
        ExactCoulombResponse3D Solve(ExactContactResponseOperand3D operand)
        {
            Assert.True(ExactContactResponseKernel.TryGetCoulombDiskResponse(Vector3d.Right, Fixed64.Zero,
                operand, second, Vector3d.Forward, Fixed64.Zero,
                operand, second, Vector3d.Up, Fixed64.Zero,
                Fixed64.Quarter, Fixed64.Half, out ExactCoulombResponse3D response, ref state));
            return response;
        }
    }

    private static void SeedNormal(ref ExactContactImpulseState? state)
    {
        ExactLever3D lever = Lever(Vector3d.Zero);
        Vector3d incoming = Vector3d.Right * (Fixed64)8;
        Assert.True(ExactContactResponseKernel.TryGetAccumulatedNormalResponse(
            Operand(lever, incoming, Vector3d.Left), Operand(lever, Vector3d.Zero, Vector3d.Right),
            Vector3d.Right, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One,
            out _, Impact(incoming, Vector3d.Zero), ref state));
    }

    private static ExactCoulombResponse3D SolveDisk(ref ExactContactImpulseState? state,
        Vector3d first, Vector3d second, Fixed64 staticFriction, Fixed64 dynamicFriction)
    {
        ExactLever3D lever = Lever(Vector3d.Zero);
        Assert.True(ExactContactResponseKernel.TryGetCoulombDiskResponse(Vector3d.Right, Fixed64.Zero,
            Operand(lever, first, -Vector3d.Forward), Operand(lever, second, Vector3d.Forward),
            Vector3d.Forward, Fixed64.Zero,
            Operand(lever, first, -Vector3d.Up), Operand(lever, second, Vector3d.Up),
            Vector3d.Up, Fixed64.Zero, staticFriction, dynamicFriction,
            out ExactCoulombResponse3D response, ref state));
        return response;
    }

    private static ExactContactResponseOperand3D Operand(ExactLever3D lever, Vector3d velocity, Vector3d axis) =>
        new(lever, velocity, Vector3d.Zero, axis, Fixed64.MinIncrement * Fixed64.Two, Fixed3x3.Zero);

    private static ContactResponseSnapshot Impact(Vector3d linear, Vector3d angular) =>
        new(Vector3d.Zero, Vector3d.Zero, ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero), linear, angular, Vector3d.Zero, Vector3d.Zero);

    private static ExactLever3D Lever(Vector3d point) =>
        ExactLever3D.Create(new FixedPointAnchor(point, FixedQuaternion.Identity, Vector3d.Zero),
            new FixedPointAnchor(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero));
}
