using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContactGroupQualityTests
{
    private readonly ITestOutputHelper _output;

    public ContactGroupQualityTests(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CenteredFaceSequentialResponse_ShouldConvergeToSymmetricImpact(bool frictionless)
    {
        PhysicsMaterial material = frictionless ? PhysicsMaterial.Frictionless : PhysicsMaterial.Default;
        Vector3d expectedVelocity = Vector3d.Up * (material.Restitution * (Fixed64)2);
        Fixed64 tolerance = Fixed64.FromFraction(1, 1_000_000);

        foreach (int iterations in new[] { 1, 6, 12, 32, 64 })
        {
            using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
            var floor = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
            var box = scenario.CreateCuboid(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));
            floor.Collider.Material = box.Collider.Material = material;
            box.Body.AddLinearImpulse(Vector3d.Down * (Fixed64)2 * box.Body.Mass);
            CollisionPair pair = scenario.CreatePair(floor.Collider, box.Collider);
            CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
            pair.Manifold.Count.Should().Be(ContactManifold.MaxContactsPerGroup);

            CollisionResponse.PrepareSolve(pair);
            for (int iteration = 0; iteration < iterations; iteration++)
                CollisionResponse.CalculateImpulse(pair, applyCachedImpulse: iteration == 0,
                    applyPositionCorrection: iteration == 0);

            Vector3d linearError = box.Body.LinearVelocity - expectedVelocity;
            Fixed64 linearResidual = ComponentMagnitudeSum(linearError);
            Fixed64 angularResidual = ComponentMagnitudeSum(box.Body.AngularVelocity);
            _output.WriteLine($"{iterations} sweeps: linear={box.Body.LinearVelocity}, angular={box.Body.AngularVelocity}, "
                + $"linear residual={linearResidual}, angular residual={angularResidual}");

            if (iterations >= 32)
            {
                // Check components rather than squared magnitudes: Q32.32 squaring
                // would round tiny, physically relevant residuals down to zero.
                linearResidual.Should().BeLessThan(tolerance,
                    $"a centered four-corner impact should converge to its symmetric restitution target after {iterations} sweeps");
                angularResidual.Should().BeLessThan(tolerance,
                    $"a centered impact should converge without spurious spin after {iterations} sweeps");
            }
        }
    }

    private static Fixed64 ComponentMagnitudeSum(Vector3d vector) =>
        vector.X.Abs() + vector.Y.Abs() + vector.Z.Abs();
}
