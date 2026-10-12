using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

/// <content>Checks independent mixed constraints through the complete host step.</content>
public sealed partial class MixedResponseTests
{
    [Fact]
    public void MixedManifoldRestitution_ShouldUseExactMinimumYawAtTheIncomingLever()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> first = CreateSphere3D(context, Vector3d.Zero, preventAngularForces: true);
        var box = new LSAABBoxCollider2D(new Vector2d((Fixed64)2048, Fixed64.One));
        var second = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), box) { Mass = Fixed64.One };
        second.Initialize(Vector2d.Zero);
        second.FreezeAxes = BodyFreezeAxes2D.Rotation;
        var material = new PhysicsMaterial(Fixed64.Zero, Fixed64.Zero, Fixed64.One);
        first.Collider.Material = second.Collider.Material = material;
        ContactAnchor center = ContactAnchor.FromWorldPoint(Vector3d.Zero);
        ContactAnchor point = ContactAnchor.FromWorldPoint(Vector3d.Right * (Fixed64)1024);
        var manifold = new ContactManifold();
        manifold.AddContact(default, new MixedContact(point, point, Vector3d.Forward, Fixed64.Zero).ToManifoldContact());
        var impact = new ContactResponseSnapshot(Vector3d.Zero, Vector3d.Zero, center, center,
            Vector3d.Zero, new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero),
            Vector3d.Zero, new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero), Fixed64.MinValue);
        ContactWarmStartCache cache = default;
        ExactContactImpulseState?[]? exact = null;

        CollisionResponseMixed.TrySolveManifoldVelocity(first.Collider, second.Collider, manifold,
            impact, ref cache, ref exact, out ContactResponseSnapshot solved).Should().BeTrue();

        // Equal frozen-rotation masses split the 1024-raw closing speed and
        // its elastic bounce. Incoming angular motion is not changed by torque.
        solved.LinearA.Should().Be(Vector3d.Backward * Fixed64.FromRaw(1024));
        solved.LinearB.Should().Be(Vector3d.Forward * Fixed64.FromRaw(1024));
        solved.AngularA.Should().Be(impact.AngularA);
        solved.PlanarAngularB.Should().Be(Fixed64.MinValue);
    }

    [Fact]
    public void MixedCompoundCorner_ShouldResolveBothWallsInOnePair()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 64);
        context.Environment.AirDensity = Fixed64.Zero;
        var corner = new LSCompoundCollider(
            CompoundColliderPart.Cuboid(new Vector3d(Fixed64.FromFraction(1, 5), (Fixed64)2, (Fixed64)4),
                Vector3d.Right * Fixed64.FromFraction(2, 5), PhysicsMaterial.Frictionless),
            CompoundColliderPart.Cuboid(new Vector3d((Fixed64)4, (Fixed64)2, Fixed64.FromFraction(1, 5)),
                Vector3d.Forward * Fixed64.FromFraction(2, 5), PhysicsMaterial.Frictionless));
        corner.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        SolidBody2D body = CreateCircle2D(context, Vector2d.Zero);
        body.FreezeAxes = BodyFreezeAxes2D.Rotation;
        body.Collider.Material = PhysicsMaterial.Frictionless;
        body.ApplyCollisionLinearVelocityDelta(Vector2d.Right + Vector2d.Forward);
        int entered = 0;
        corner.OnMixedContactEnter += _ => entered++;

        Step(context);

        body.LinearVelocity.Should().Be(Vector2d.Zero);
        body.Position.X.Should().BeLessThan(Fixed64.Zero);
        body.Position.Y.Should().BeLessThan(Fixed64.Zero);
        body.Agent.Transform.LocalPosition.Y.Should().Be(Fixed64.Zero);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
        entered.Should().Be(1);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(6)]
    public void MixedCompoundCorner_ShouldReflectIncomingMotionOnceAcrossIterations(int iterations)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 64);
        context.Environment.AirDensity = Fixed64.Zero;
        context.Settings.DiscreteSolverIterations = iterations;
        context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        var material = new PhysicsMaterial(Fixed64.Zero, Fixed64.Zero, Fixed64.One);
        LSCompoundCollider corner = CreateMixedManifoldCorner(context, material);
        SolidBody2D body = CreateCircle2D(context, Vector2d.Zero);
        body.FreezeAxes = BodyFreezeAxes2D.Rotation;
        body.Collider.Material = material;
        body.ApplyCollisionLinearVelocityDelta(Vector2d.Right + Vector2d.Forward);

        Step(context);

        // Orthogonal frictionless walls reverse each incoming component independently.
        // Later iterations must retain that incoming restitution target, rather than erase the bounce.
        body.LinearVelocity.Should().Be(Vector2d.Left - Vector2d.Forward);
        body.AngularVelocity.Should().Be(Fixed64.Zero);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
    }

    [Fact]
    public void MixedEqualNormalGroups_ShouldCorrectOnlyTheDeepestDirectionOnce()
    {
        using GravitasWorldContext singleContext = CreateMixedContext();
        using GravitasWorldContext groupedContext = CreateMixedContext();
        CollisionPairMixed single = CreateMixedParallelWallPair(singleContext, Fixed64.Half);
        CollisionPairMixed grouped = CreateMixedParallelWallPair(groupedContext,
            Fixed64.FromFraction(3, 4), Fixed64.Half, Fixed64.Half);
        grouped.Manifold.Count.Should().Be(3);
        grouped.Manifold[0].Depth.Should().Be(Fixed64.FromFraction(1, 4));
        grouped.Manifold.PrimaryContact.Depth.Should().Be(Fixed64.Half);
        for (int i = 0; i < grouped.Manifold.Count; i++)
            grouped.Manifold[i].Normal.Should().Be(Vector3d.Right);

        CollisionResponseMixed.ResolveManifold(single);
        CollisionResponseMixed.ResolveManifold(grouped);

        // A shallower row authored first and a duplicate deepest row cannot multiply translation.
        grouped.Collider2D.Body!.Position.Should().Be(single.Collider2D.Body!.Position);
        grouped.Collider2D.Body.Position.X.Should().BeGreaterThan(Fixed64.Zero);
        grouped.ResponseSnapshot.IsValid.Should().BeFalse();
    }

    [Fact]
    public void MixedVerticalPrimaryWithPlanarSecondary_ShouldWakeAndResolveThePlanarParticipant()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 64);
        context.Environment.AirDensity = Fixed64.Zero;
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Cuboid(new Vector3d((Fixed64)4, Fixed64.FromFraction(1, 5), (Fixed64)4),
                Vector3d.Down * Fixed64.FromFraction(1, 10), PhysicsMaterial.Frictionless),
            CompoundColliderPart.Cuboid(new Vector3d(Fixed64.FromFraction(1, 5), (Fixed64)2, (Fixed64)4),
                Vector3d.Right * Fixed64.FromFraction(2, 5), PhysicsMaterial.Frictionless));
        var body3D = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), compound)
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes3D.Rotation
        };
        body3D.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        body3D.ApplyCollisionLinearVelocityDelta(Vector3d.Left);
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero);
        body2D.FreezeAxes = BodyFreezeAxes2D.Rotation;
        body2D.Collider.Material = PhysicsMaterial.Frictionless;
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(compound, body2D.Collider, manifold).Should().BeTrue();
        manifold.Count.Should().Be(2);
        manifold.PrimaryContact.Normal.Should().Be(Vector3d.Up);
        manifold[1].Normal.Should().Be(Vector3d.Left);
        body2D.Sleep();
        body2D.IsSleeping.Should().BeTrue();

        Step(context);

        body2D.IsSleeping.Should().BeFalse();
        body2D.LinearVelocity.X.Should().BeLessThan(Fixed64.Zero);
        body2D.Agent.Transform.LocalPosition.Y.Should().Be(Fixed64.Zero);
        body3D.Position3d.Y.Should().BeLessThan(Fixed64.Zero);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
    }

    [Fact]
    public void MixedManifoldPublication_ShouldDiscardPreviousSolveImpulsesAndIncomingSnapshot()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCompoundCollider corner = CreateMixedManifoldCorner(context, PhysicsMaterial.Frictionless);
        SolidBody2D body = CreateCircle2D(context, Vector2d.Zero);
        body.FreezeAxes = BodyFreezeAxes2D.Rotation;
        body.ApplyCollisionLinearVelocityDelta(Vector2d.Right + Vector2d.Forward);
        CollisionPairMixed pair = BuildMixedManifoldPair(corner, body.Collider);

        CollisionResponseMixed.PrepareSolve(pair);
        pair.ResponseSnapshot.IsValid.Should().BeTrue();
        CollisionResponseMixed.ResolveManifold(pair, 0, 2, correctPosition: false);
        pair.SolveImpulses.Count.Should().Be(2);
        pair.ResponseSnapshot.LinearB.Should().Be(Vector3d.Right + Vector3d.Forward);

        pair.MarkResting(1, pair.Manifold);

        pair.Manifold.Count.Should().Be(2);
        pair.SolveImpulses.Count.Should().Be(0);
        pair.ResponseSnapshot.IsValid.Should().BeFalse();
        CollisionResponseMixed.PrepareSolve(pair);
        pair.ResponseSnapshot.LinearB.Should().Be(Vector3d.Zero);
        pair.Initialize(corner, body.Collider);
        pair.ResponseSnapshot.IsValid.Should().BeFalse();
    }

    [Theory]
    [InlineData(1)]
    [InlineData(6)]
    public void MixedDuplicateNormalRows_ShouldKeepAccumulatedFrictionIndependentOfIterations(int iterations)
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.DiscreteSolverIterations = iterations;
        var material = new PhysicsMaterial(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4), Fixed64.Zero);
        var walls = new LSCompoundCollider(
            CompoundColliderPart.Cuboid(new Vector3d(Fixed64.One, (Fixed64)2, (Fixed64)2), Vector3d.Left, material),
            CompoundColliderPart.Cuboid(new Vector3d(Fixed64.One, (Fixed64)2, (Fixed64)2), Vector3d.Left, material));
        walls.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        SolidBody2D body = CreateCircle2D(context, Vector2d.Zero);
        body.FreezeAxes = BodyFreezeAxes2D.Rotation;
        body.Collider.Material = material;
        CollisionPairMixed pair = BuildMixedManifoldPair(walls, body.Collider);
        body.ApplyCollisionLinearVelocityDelta(Vector2d.Left + Vector2d.Forward);

        CollisionResponseMixed.ResolveManifold(pair);

        // Unit mass closing at unit speed needs unit normal impulse. μ=1/4 removes 1/4 tangent speed.
        body.LinearVelocity.Should().Be(Vector2d.Forward * Fixed64.FromFraction(3, 4));
        body.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MixedFullRangeMassFriction_ShouldPreserveRepresentableVelocityWithRepeatedRows(bool repeated)
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.DiscreteSolverIterations = 3;
        ScenarioBody<LSSphereCollider> first = CreateSphere3D(context,
            Vector3d.Left * Fixed64.FromFraction(3, 4), preventAngularForces: true);
        SolidBody2D second = CreateCircle2D(context, Vector2d.Zero);
        second.FreezeAxes = BodyFreezeAxes2D.Rotation;
        first.Body.Mass = second.Mass = Fixed64.MaxValue;
        var material = new PhysicsMaterial(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4), Fixed64.Zero);
        first.Collider.Material = second.Collider.Material = material;
        CollisionPairMixed pair = BuildMixedManifoldPair(first.Collider, second.Collider);
        if (repeated)
        {
            ManifoldContact row = pair.Manifold.PrimaryContact;
            pair.Manifold.AddContact(new ContactGroupKey(1, 0), row);
        }
        first.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)4 + Vector3d.Forward * (Fixed64)2);

        CollisionResponseMixed.ResolveManifold(pair);

        // Equal masses share the closing speed. λ=2m exceeds Q32.32, while these velocity changes fit.
        first.Body.LinearVelocity.Should().Be(new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.FromFraction(3, 2)));
        second.LinearVelocity.Should().Be(new Vector2d((Fixed64)2, Fixed64.Half));
        if (repeated)
        {
            ExactContactImpulseState?[] states = pair.ExactSolveImpulses!;
            states.Should().NotBeNull();
            states.Should().Contain(state => state != null && state.HasNormal);
            pair.MarkResting(1, pair.Manifold);
            pair.ExactSolveImpulses.Should().BeSameAs(states);
            states.Should().OnlyContain(state => state == null || !state.HasNormal && !state.HasTangents);
        }
    }

    private static LSCompoundCollider CreateMixedManifoldCorner(GravitasWorldContext context, PhysicsMaterial material)
    {
        var collider = new LSCompoundCollider(
            CompoundColliderPart.Cuboid(new Vector3d(Fixed64.FromFraction(1, 5), (Fixed64)2, (Fixed64)4),
                Vector3d.Right * Fixed64.FromFraction(2, 5), material),
            CompoundColliderPart.Cuboid(new Vector3d((Fixed64)4, (Fixed64)2, Fixed64.FromFraction(1, 5)),
                Vector3d.Forward * Fixed64.FromFraction(2, 5), material));
        collider.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        return collider;
    }

    private static CollisionPairMixed CreateMixedParallelWallPair(GravitasWorldContext context, params Fixed64[] distances)
    {
        var parts = new CompoundColliderPart[distances.Length];
        for (int i = 0; i < parts.Length; i++)
            parts[i] = CompoundColliderPart.Cuboid(new Vector3d(Fixed64.One, (Fixed64)2, (Fixed64)2),
                Vector3d.Left * distances[i], PhysicsMaterial.Frictionless);
        var collider = new LSCompoundCollider(parts);
        collider.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        SolidBody2D body = CreateCircle2D(context, Vector2d.Zero);
        body.FreezeAxes = BodyFreezeAxes2D.Rotation;
        body.Collider.Material = PhysicsMaterial.Frictionless;
        return BuildMixedManifoldPair(collider, body.Collider);
    }

    private static CollisionPairMixed BuildMixedManifoldPair(LSCollider collider3D, LSCollider2D collider2D)
    {
        var pair = new CollisionPairMixed(collider3D, collider2D);
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(collider3D, collider2D, manifold).Should().BeTrue();
        pair.MarkResting(0, manifold);
        return pair;
    }
}
