using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

/// <content>Requires repeated mixed CCD previews to reuse compact and oversized normal-impulse state.</content>
public sealed partial class MixedResponseTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MixedCcdManifoldPreview_ShouldReuseAccumulationWithoutAllocating(bool oversized)
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.DiscreteSolverIterations = 3;
        ScenarioBody<LSSphereCollider> first = CreateSphere3D(context,
            Vector3d.Left * Fixed64.FromFraction(3, 4), preventAngularForces: true);
        SolidBody2D second = CreateCircle2D(context, Vector2d.Zero);
        second.FreezeAxes = BodyFreezeAxes2D.Rotation;
        first.Collider.Material = second.Collider.Material = PhysicsMaterial.Frictionless;
        if (oversized) first.Body.Mass = second.Mass = Fixed64.MaxValue;
        CollisionPairMixed pair = BuildMixedManifoldPair(first.Collider, second.Collider);
        pair.Manifold.AddContact(new ContactGroupKey(1, 0), pair.Manifold.PrimaryContact);
        var impact = new ContactResponseSnapshot(first.Body.Position3d, Vector3d.Zero,
            first.Body.GetCenterOfMassAnchor(), ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right * (Fixed64)4 + Vector3d.Forward * (Fixed64)2, Vector3d.Zero,
            Vector3d.Zero, Vector3d.Zero, Fixed64.Zero);
        ContactWarmStartCache cache = default;
        ExactContactImpulseState?[]? exactImpulses = null;
        ContactResponseSnapshot solved = default;
        bool allSolved = true;

        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            allSolved &= CollisionResponseMixed.TrySolveManifoldVelocity(first.Collider, second.Collider,
                pair.Manifold, impact, ref cache, ref exactImpulses, out solved);
        }, warmupIterations: 16, stabilizationIterations: 4, measurementIterations: 16);

        allSolved.Should().BeTrue();
        allocated.Should().Be(0);
        // CCD preserves its normal-only policy: equal masses split X closing
        // speed while tangent Z motion remains on the source. λ=2m exceeds
        // Fixed64 for the large-mass case, but every final velocity fits.
        solved.LinearA.Should().Be(new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2));
        solved.LinearB.Should().Be(Vector3d.Right * (Fixed64)2);
        first.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        second.LinearVelocity.Should().Be(Vector2d.Zero);
    }
}
