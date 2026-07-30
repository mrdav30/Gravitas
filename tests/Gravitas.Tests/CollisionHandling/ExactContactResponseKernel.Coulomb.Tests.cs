using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.CollisionHandling;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

/// <summary>
/// Verifies exact line and uncached disk Coulomb response.
/// </summary>
public sealed partial class ExactContactResponseKernelCoulombTests
{
    [Fact]
    public void CoulombLineResponse_ShouldRetainUnrepresentableNormalAndTangentImpulses()
    {
        ExactLever3D lever = CreateLever();
        Fixed64 inverseMass = Fixed64.MinIncrement * Fixed64.Two;
        Vector3d firstVelocity =
            (Vector3d.Right * (Fixed64)6)
            + (Vector3d.Forward * (Fixed64)4);
        var normalFirst = new ExactContactResponseOperand3D(
            lever,
            firstVelocity,
            Vector3d.Zero,
            Vector3d.Left,
            inverseMass,
            Fixed3x3.Zero);
        var normalSecond = new ExactContactResponseOperand3D(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Right,
            inverseMass,
            Fixed3x3.Zero);
        var constraint = new ExactNormalConstraint3D(
            normalFirst,
            normalSecond,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One);
        var tangentFirst = new ExactContactResponseOperand3D(
            lever,
            firstVelocity,
            Vector3d.Zero,
            -Vector3d.Forward,
            inverseMass,
            Fixed3x3.Zero);
        var tangentSecond = new ExactContactResponseOperand3D(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Forward,
            inverseMass,
            Fixed3x3.Zero);

        bool resolved = ExactContactResponseKernel.TryGetCoulombLineResponse(
            constraint,
            tangentFirst,
            tangentSecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out ExactCoulombResponse3D response);

        Assert.True(resolved);
        Assert.True(response.HasAppliedImpulse);
        Assert.Equal(
            -Vector3d.Forward * Fixed64.Two,
            response.FirstLinearVelocityDelta);
        Assert.Equal(
            Vector3d.Forward * Fixed64.Two,
            response.SecondLinearVelocityDelta);
        Assert.False(response.TryGetPrimaryAccumulatedImpulse(out _));
        Assert.False(response.TryGetSecondaryAccumulatedImpulse(out _));
    }

    [Fact]
    public void CoulombLineResponse_ShouldClampWideDesiredImpulseToDynamicLimit()
    {
        ExactLever3D lever = CreateLever();
        Fixed64 inverseMass = Fixed64.MinIncrement * Fixed64.Two;
        Vector3d firstVelocity =
            (Vector3d.Up * (Fixed64)6)
            + (Vector3d.Forward * (Fixed64)8);
        ExactNormalConstraint3D constraint =
            CreateNormalConstraint(
                lever,
                inverseMass,
                Vector3d.Up,
                firstVelocity);
        var tangentFirst = new ExactContactResponseOperand3D(
            lever,
            firstVelocity,
            Vector3d.Zero,
            -Vector3d.Forward,
            inverseMass,
            Fixed3x3.Zero);
        var tangentSecond = new ExactContactResponseOperand3D(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Forward,
            inverseMass,
            Fixed3x3.Zero);

        Assert.True(ExactContactResponseKernel.TryGetCoulombLineResponse(
            constraint,
            tangentFirst,
            tangentSecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.Half,
            Fixed64.One / (Fixed64)4,
            out ExactCoulombResponse3D response));

        Assert.Equal(
            -Vector3d.Forward * (Fixed64)0.75m,
            response.FirstLinearVelocityDelta);
        Assert.Equal(
            Vector3d.Forward * (Fixed64)0.75m,
            response.SecondLinearVelocityDelta);
        Assert.True(response.TryGetPrimaryAccumulatedImpulse(
            out Fixed64 accumulated));
        Assert.True(accumulated > Fixed64.Zero);
    }

    [Fact]
    public void CoulombLineResponse_ShouldApplyWarmAccumulatorChangesExactly()
    {
        ExactLever3D lever = CreateLever();
        ExactNormalConstraint3D constraint = CreateNormalConstraint(
            lever,
            Fixed64.One,
            Vector3d.Up,
            (Vector3d.Up * (Fixed64)6) + Vector3d.Forward);
        ExactContactResponseOperand3D first = CreateTangentOperand(
            lever,
            constraint.First.LinearVelocity,
            -Vector3d.Forward,
            Fixed64.One);
        ExactContactResponseOperand3D second = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One);

        AssertAccumulatedImpulse(-Fixed64.Half, Fixed64.Zero);
        AssertAccumulatedImpulse(-(Fixed64)0.25m, Fixed64.One / (Fixed64)4);
        AssertAccumulatedImpulse(-Fixed64.One, -Fixed64.Half);
        AssertAccumulatedImpulse(Fixed64.One, (Fixed64)1.5m);

        void AssertAccumulatedImpulse(
            Fixed64 previous,
            Fixed64 expected)
        {
            Assert.True(ExactContactResponseKernel.TryGetCoulombLineResponse(
                constraint,
                first,
                second,
                Vector3d.Forward,
                previous,
                Fixed64.One,
                Fixed64.One,
                out ExactCoulombResponse3D response));
            Assert.True(response.HasAppliedImpulse);
            Assert.True(response.TryGetPrimaryAccumulatedImpulse(
                out Fixed64 accumulated));
            Assert.Equal(expected, accumulated);
        }
    }

    [Fact]
    public void CoulombLineResponse_ShouldUseCompletedNormalLoad()
    {
        ExactLever3D lever = CreateLever();
        Vector3d tangentVelocity = Vector3d.Forward;
        ExactContactResponseOperand3D tangentFirst = CreateTangentOperand(
            lever,
            tangentVelocity,
            -Vector3d.Forward,
            Fixed64.One);
        ExactContactResponseOperand3D tangentSecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One);

        ExactNormalConstraint3D resting = CreateNormalConstraint(
            lever,
            Fixed64.One,
            Vector3d.Up,
            tangentVelocity,
            accumulatedImpulse: Fixed64.One);
        Assert.True(ExactContactResponseKernel.TryGetCoulombLineResponse(
            resting,
            tangentFirst,
            tangentSecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out ExactCoulombResponse3D restingResponse));
        Assert.True(restingResponse.HasAppliedImpulse);

        ExactNormalConstraint3D partiallyReleased =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                -Vector3d.Up + tangentVelocity,
                accumulatedImpulse: Fixed64.One);
        Assert.True(ExactContactResponseKernel.TryGetCoulombLineResponse(
            partiallyReleased,
            tangentFirst,
            tangentSecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out ExactCoulombResponse3D partialResponse));
        Assert.True(partialResponse.HasAppliedImpulse);

        ExactNormalConstraint3D fullyReleased =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                -Vector3d.Up + tangentVelocity,
                accumulatedImpulse: Fixed64.One / (Fixed64)4);
        Assert.True(ExactContactResponseKernel.TryGetCoulombLineResponse(
            fullyReleased,
            tangentFirst,
            tangentSecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out ExactCoulombResponse3D releasedResponse));
        Assert.False(releasedResponse.HasAppliedImpulse);

        ExactNormalConstraint3D belowRestitutionThreshold =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                (Vector3d.Up * Fixed64.MinIncrement) + tangentVelocity,
                restitution: Fixed64.One,
                restitutionVelocityThreshold: Fixed64.One);
        Assert.True(ExactContactResponseKernel.TryGetCoulombLineResponse(
            belowRestitutionThreshold,
            tangentFirst,
            tangentSecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void CoulombLineResponse_ShouldClampAgainstSignedWarmCacheAndRejectOverflow()
    {
        ExactLever3D lever = CreateLever();
        Vector3d movingVelocity =
            (Vector3d.Up * (Fixed64)6)
            + (Vector3d.Forward * (Fixed64)8);
        ExactNormalConstraint3D constraint =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                movingVelocity);
        ExactContactResponseOperand3D first = CreateTangentOperand(
            lever,
            movingVelocity,
            -Vector3d.Forward,
            Fixed64.One);
        ExactContactResponseOperand3D second = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One);

        AssertClamped(Fixed64.One);
        AssertClamped(-Fixed64.One);

        ExactNormalConstraint3D overflowConstraint =
            CreateNormalConstraint(
                lever,
                Fixed64.Two,
                Vector3d.Up,
                Vector3d.Forward * Fixed64.MinIncrement,
                accumulatedImpulse: Fixed64.MaxValue);
        first = CreateTangentOperand(
            lever,
            overflowConstraint.First.LinearVelocity,
            -Vector3d.Forward,
            Fixed64.Two);
        second = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.Two);
        Assert.False(ExactContactResponseKernel.TryGetCoulombLineResponse(
            overflowConstraint,
            first,
            second,
            Vector3d.Forward,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero,
            out _));

        void AssertClamped(Fixed64 accumulated)
        {
            Assert.True(ExactContactResponseKernel.TryGetCoulombLineResponse(
                constraint,
                first,
                second,
                Vector3d.Forward,
                accumulated,
                Fixed64.Half,
                Fixed64.One / (Fixed64)4,
                out ExactCoulombResponse3D response));
            Assert.True(response.TryGetPrimaryAccumulatedImpulse(
                out Fixed64 completed));
            Assert.Equal((Fixed64)0.75m, completed);
        }
    }

    [Fact]
    public void CoulombDiskResponse_ShouldClampUnrepresentableTangentsWithoutScalarProjection()
    {
        ExactLever3D lever = CreateLever();
        Fixed64 inverseMass = Fixed64.MinIncrement * Fixed64.Two;
        Vector3d tangentVelocity =
            (Vector3d.Right + Vector3d.Forward) * (Fixed64)8;
        Vector3d firstVelocity =
            (Vector3d.Up * (Fixed64)6) + tangentVelocity;
        ExactNormalConstraint3D constraint =
            CreateNormalConstraint(
                lever,
                inverseMass,
                Vector3d.Up,
                firstVelocity);
        ExactContactResponseOperand3D primaryFirst = CreateTangentOperand(
            lever,
            firstVelocity,
            Vector3d.Left,
            inverseMass);
        ExactContactResponseOperand3D primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            inverseMass);
        ExactContactResponseOperand3D secondaryFirst = CreateTangentOperand(
            lever,
            firstVelocity,
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
            Fixed64.Half,
            Fixed64.One / (Fixed64)4,
            out ExactCoulombResponse3D response));

        Assert.True(response.HasAppliedImpulse);
        Assert.Equal(
            response.FirstLinearVelocityDelta.X,
            response.FirstLinearVelocityDelta.Z);
        Assert.True(response.FirstLinearVelocityDelta.X < Fixed64.Zero);
        Assert.Equal(
            -response.FirstLinearVelocityDelta,
            response.SecondLinearVelocityDelta);
        Fixed64 magnitude = response.FirstLinearVelocityDelta.Magnitude;
        Assert.True(magnitude >= (Fixed64)0.74999999m);
        Assert.True(magnitude <= (Fixed64)0.75000001m);
    }

    [Fact]
    public void CoulombDiskResponse_ShouldHandleStaticRestingAndZeroDynamicLimits()
    {
        ExactLever3D lever = CreateLever();
        Vector3d movingVelocity =
            (Vector3d.Up * (Fixed64)6)
            + Vector3d.Right
            + Vector3d.Forward;
        ExactNormalConstraint3D movingConstraint =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                movingVelocity);
        ExactContactResponseOperand3D primaryFirst = CreateTangentOperand(
            lever,
            movingVelocity,
            Vector3d.Left,
            Fixed64.One);
        ExactContactResponseOperand3D primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One);
        ExactContactResponseOperand3D secondaryFirst = CreateTangentOperand(
            lever,
            movingVelocity,
            -Vector3d.Forward,
            Fixed64.One);
        ExactContactResponseOperand3D secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One);

        Assert.True(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            movingConstraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            Fixed64.Zero,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out ExactCoulombResponse3D staticResponse));
        Assert.Equal(
            (Vector3d.Left - Vector3d.Forward) * Fixed64.Half,
            staticResponse.FirstLinearVelocityDelta);
        Assert.True(staticResponse.TryGetPrimaryAccumulatedImpulse(
            out Fixed64 primaryAccumulated));
        Assert.Equal(Fixed64.Half, primaryAccumulated);
        Assert.True(staticResponse.TryGetSecondaryAccumulatedImpulse(
            out Fixed64 secondaryAccumulated));
        Assert.Equal(Fixed64.Half, secondaryAccumulated);

        Vector3d restingVelocity = Vector3d.Up * (Fixed64)6;
        ExactNormalConstraint3D restingConstraint =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                restingVelocity);
        ExactContactResponseOperand3D restingFirst = CreateTangentOperand(
            lever,
            restingVelocity,
            Vector3d.Left,
            Fixed64.One);
        Assert.True(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            restingConstraint,
            restingFirst,
            primarySecond,
            Vector3d.Right,
            Fixed64.Zero,
            restingFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            out ExactCoulombResponse3D restingResponse));
        Assert.False(restingResponse.HasAppliedImpulse);
        Assert.Equal(Vector3d.Zero, restingResponse.FirstLinearVelocityDelta);

        Assert.True(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            movingConstraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            Fixed64.Zero,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            out ExactCoulombResponse3D zeroDynamicResponse));
        Assert.False(zeroDynamicResponse.HasAppliedImpulse);
        Assert.Equal(Vector3d.Zero, zeroDynamicResponse.FirstLinearVelocityDelta);
    }

    [Fact]
    public void CoulombDiskResponse_ShouldCombineAngularTangentChanges()
    {
        ExactLever3D lever = CreateLever(Vector3d.Up);
        Vector3d staticVelocity =
            (Vector3d.Up * (Fixed64)6)
            + Vector3d.Right
            + Vector3d.Forward;
        ExactNormalConstraint3D staticConstraint =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                staticVelocity,
                Fixed3x3.Identity);
        ExactContactResponseOperand3D primaryFirst = CreateTangentOperand(
            lever,
            staticVelocity,
            Vector3d.Left,
            Fixed64.One,
            Fixed3x3.Identity);
        ExactContactResponseOperand3D primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One,
            Fixed3x3.Identity);
        ExactContactResponseOperand3D secondaryFirst = CreateTangentOperand(
            lever,
            staticVelocity,
            -Vector3d.Forward,
            Fixed64.One,
            Fixed3x3.Identity);
        ExactContactResponseOperand3D secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One,
            Fixed3x3.Identity);

        Assert.True(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            staticConstraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            Fixed64.Zero,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out ExactCoulombResponse3D staticResponse));
        Assert.NotEqual(Vector3d.Zero, staticResponse.FirstAngularVelocityDelta);
        Assert.Equal(
            -staticResponse.FirstAngularVelocityDelta,
            staticResponse.SecondAngularVelocityDelta);

        Vector3d dynamicVelocity =
            (Vector3d.Up * (Fixed64)6)
            + ((Vector3d.Right + Vector3d.Forward) * (Fixed64)8);
        ExactNormalConstraint3D dynamicConstraint =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                dynamicVelocity,
                Fixed3x3.Identity);
        primaryFirst = CreateTangentOperand(
            lever,
            dynamicVelocity,
            Vector3d.Left,
            Fixed64.One,
            Fixed3x3.Identity);
        secondaryFirst = CreateTangentOperand(
            lever,
            dynamicVelocity,
            -Vector3d.Forward,
            Fixed64.One,
            Fixed3x3.Identity);
        Assert.True(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            dynamicConstraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            Fixed64.Zero,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One / (Fixed64)4,
            out ExactCoulombResponse3D dynamicResponse));
        Assert.NotEqual(Vector3d.Zero, dynamicResponse.FirstAngularVelocityDelta);
        Assert.Equal(
            -dynamicResponse.FirstAngularVelocityDelta,
            dynamicResponse.SecondAngularVelocityDelta);
    }

    [Fact]
    public void CoulombDiskResponse_ShouldResolveOneTangentAndRejectOverflow()
    {
        ExactLever3D lever = CreateLever();
        Vector3d movingVelocity =
            (Vector3d.Up * (Fixed64)6)
            + (Vector3d.Right * (Fixed64)8);
        ExactNormalConstraint3D constraint =
            CreateNormalConstraint(
                lever,
                Fixed64.One,
                Vector3d.Up,
                movingVelocity);
        ExactContactResponseOperand3D primaryFirst = CreateTangentOperand(
            lever,
            movingVelocity,
            Vector3d.Left,
            Fixed64.One);
        ExactContactResponseOperand3D primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.One);
        ExactContactResponseOperand3D secondaryFirst = CreateTangentOperand(
            lever,
            movingVelocity,
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
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One / (Fixed64)4,
            out ExactCoulombResponse3D response));
        Assert.True(response.HasAppliedImpulse);
        Assert.True(response.TryGetSecondaryAccumulatedImpulse(
            out Fixed64 secondaryAccumulated));
        Assert.Equal(Fixed64.Zero, secondaryAccumulated);

        ExactNormalConstraint3D overflowConstraint =
            CreateNormalConstraint(
                lever,
                Fixed64.MaxValue,
                Vector3d.Up,
                Vector3d.Right,
                accumulatedImpulse: Fixed64.MaxValue);
        primaryFirst = CreateTangentOperand(
            lever,
            overflowConstraint.First.LinearVelocity,
            Vector3d.Left,
            Fixed64.MaxValue);
        primarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.MaxValue);
        secondaryFirst = CreateTangentOperand(
            lever,
            overflowConstraint.First.LinearVelocity,
            -Vector3d.Forward,
            Fixed64.MaxValue);
        secondarySecond = CreateTangentOperand(
            lever,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.MaxValue);
        Assert.False(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            overflowConstraint,
            primaryFirst,
            primarySecond,
            Vector3d.Right,
            Fixed64.Zero,
            secondaryFirst,
            secondarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.MaxValue,
            out _));

        ExactLever3D angularLever = CreateLever(Vector3d.Up);
        var angularOverflowInertia = new Fixed3x3(
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.MaxValue, Fixed64.Zero, Fixed64.MinIncrement);
        var angularNormalFirst = new ExactContactResponseOperand3D(
            angularLever,
            Vector3d.Right,
            Vector3d.Zero,
            -Vector3d.Up,
            Fixed64.Zero,
            angularOverflowInertia);
        var angularNormalSecond = new ExactContactResponseOperand3D(
            angularLever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed3x3.Zero);
        var angularConstraint = new ExactNormalConstraint3D(
            angularNormalFirst,
            angularNormalSecond,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.MaxValue,
            Fixed64.One,
            Fixed64.One);
        var angularPrimaryFirst = new ExactContactResponseOperand3D(
            angularLever,
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Zero,
            angularOverflowInertia);
        var angularPrimarySecond = new ExactContactResponseOperand3D(
            angularLever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed3x3.Zero);
        Assert.False(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            angularConstraint,
            angularPrimaryFirst,
            angularPrimarySecond,
            Vector3d.Right,
            Fixed64.Zero,
            angularPrimaryFirst,
            angularPrimarySecond,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void CoulombResponses_ShouldValidateInputsAndRemainAllocationFree()
    {
        ExactLever3D lever = CreateLever();
        ExactNormalConstraint3D constraint =
            CreateNormalConstraint(lever, Fixed64.One);
        var first = new ExactContactResponseOperand3D(
            lever,
            constraint.First.LinearVelocity,
            Vector3d.Zero,
            -Vector3d.Forward,
            Fixed64.One,
            Fixed3x3.Zero);
        var second = new ExactContactResponseOperand3D(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Forward,
            Fixed64.One,
            Fixed3x3.Zero);

        Assert.False(ExactContactResponseKernel.TryGetCoulombLineResponse(
            constraint,
            first,
            second,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));
        Assert.False(ExactContactResponseKernel.TryGetCoulombLineResponse(
            constraint,
            first,
            second,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            -Fixed64.One,
            out _));

        var mismatched = new ExactContactResponseOperand3D(
            CreateLever(Vector3d.Up),
            first.LinearVelocity,
            first.AngularVelocity,
            first.LinearImpulseAxis,
            first.InverseMass,
            first.InverseInertia);
        Assert.False(ExactContactResponseKernel.TryGetCoulombLineResponse(
            constraint,
            mismatched,
            second,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));

        var invalidConstraint = new ExactNormalConstraint3D(
            constraint.First,
            constraint.Second,
            Vector3d.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One);
        Assert.False(ExactContactResponseKernel.TryGetCoulombLineResponse(
            invalidConstraint,
            first,
            second,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));

        Assert.False(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            constraint,
            first,
            second,
            Vector3d.Forward,
            Fixed64.Zero,
            first,
            second,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));
        Assert.False(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            constraint,
            mismatched,
            second,
            Vector3d.Forward,
            Fixed64.Zero,
            first,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));
        Assert.False(ExactContactResponseKernel.TryGetCoulombDiskResponse(
            invalidConstraint,
            first,
            second,
            Vector3d.Forward,
            Fixed64.Zero,
            first,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));
        Assert.False(ExactContactResponseKernel.TryGetCoulombLineResponse(
            constraint,
            first,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            -Fixed64.One,
            Fixed64.One,
            out _));
        Assert.False(ExactContactResponseKernel.TryGetCoulombLineResponse(
            constraint,
            first,
            second,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));

        Assert.True(ExactContactResponseKernel.TryGetCoulombLineResponse(
            constraint,
            first,
            second,
            Vector3d.Forward,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));
        long before = System.GC.GetAllocatedBytesForCurrentThread();
        for (int iteration = 0; iteration < 8; iteration++)
        {
            Assert.True(ExactContactResponseKernel.TryGetCoulombLineResponse(
                constraint,
                first,
                second,
                Vector3d.Forward,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out _));
        }
        Assert.Equal(before, System.GC.GetAllocatedBytesForCurrentThread());
    }

    private static ExactNormalConstraint3D CreateNormalConstraint(
        ExactLever3D lever,
        Fixed64 inverseMass,
        Vector3d? normal = null,
        Vector3d? firstVelocity = null,
        Fixed3x3? inverseInertia = null,
        Fixed64? restitution = null,
        Fixed64? restitutionVelocityThreshold = null,
        Fixed64? accumulatedImpulse = null)
    {
        Vector3d resolvedNormal = normal ?? Vector3d.Right;
        Fixed3x3 resolvedInverseInertia = inverseInertia ?? Fixed3x3.Zero;
        var first = new ExactContactResponseOperand3D(
            lever,
            firstVelocity ?? resolvedNormal * (Fixed64)6,
            Vector3d.Zero,
            -resolvedNormal,
            inverseMass,
            resolvedInverseInertia);
        var second = new ExactContactResponseOperand3D(
            lever,
            Vector3d.Zero,
            Vector3d.Zero,
            resolvedNormal,
            inverseMass,
            resolvedInverseInertia);
        return new ExactNormalConstraint3D(
            first,
            second,
            resolvedNormal,
            restitution ?? Fixed64.Zero,
            restitutionVelocityThreshold ?? Fixed64.Zero,
            accumulatedImpulse ?? Fixed64.Zero,
            Fixed64.One,
            Fixed64.One);
    }

    private static ExactContactResponseOperand3D CreateTangentOperand(
        ExactLever3D lever,
        Vector3d velocity,
        Vector3d signedAxis,
        Fixed64 inverseMass,
        Fixed3x3? inverseInertia = null) =>
        new(
            lever,
            velocity,
            Vector3d.Zero,
            signedAxis,
            inverseMass,
            inverseInertia ?? Fixed3x3.Zero);

    private static ExactLever3D CreateLever() =>
        CreateLever(Vector3d.Zero);

    private static ExactLever3D CreateLever(Vector3d point)
    {
        var pointAnchor = new FixedPointAnchor(
            point,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        var centerAnchor = new FixedPointAnchor(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        return ExactLever3D.Create(pointAnchor, centerAnchor);
    }
}
