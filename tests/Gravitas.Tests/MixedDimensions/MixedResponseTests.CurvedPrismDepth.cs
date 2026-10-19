using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedResponseTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void Step_WithConeGeneratorContact_ShouldRespectExactDepthAndReplay(bool inward, bool moving3D)
    {
        var first = RunConeGeneratorHostSteps(inward, moving3D);
        var replay = RunConeGeneratorHostSteps(inward, moving3D);

        replay.Should().Be(first);
    }

    private static (Vector3d First3D, Vector2d First2D, Vector3d Final3D, Vector2d Final2D,
        Vector3d Velocity3D, Vector2d Velocity2D, (int Enter, int Stay, int Exit) Events)
        RunConeGeneratorHostSteps(bool inward, bool moving3D)
    {
        using GravitasWorldContext context = CreateMixedContext();
        Fixed64 delta = inward ? Fixed64.FromFraction(1, 64) : Fixed64.Zero;
        Vector3d slabOrigin = new(Fixed64.One - Fixed64.Two * delta, Fixed64.One - delta, Fixed64.Zero);
        var cone = new LSConeCollider
        {
            Radius = Fixed64.One,
            Size = new Vector3d(2, 2, 2),
            Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero)
        };
        var spatial = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), cone)
        {
            Mass = Fixed64.One,
            SleepEnabled = false,
            FreezeAxes = BodyFreezeAxes3D.Rotation,
            LinearDragCoefficient = Fixed64.Zero
        };
        spatial.Initialize(Vector3d.Zero, FixedQuaternion.Identity,
            moving3D ? BodyMotionType.Dynamic : BodyMotionType.Static);
        var slab = new LSAABBoxCollider2D(new Vector2d(Fixed64.One, Fixed64.Two))
        {
            MixedHalfThicknessOverride = Fixed64.One,
            Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero)
        };
        var planar = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(slabOrigin, FixedQuaternion.Identity, Vector3d.One)), slab)
        {
            Mass = Fixed64.One,
            SleepEnabled = false,
            FreezeAxes = BodyFreezeAxes2D.Rotation
        };
        Vector2d initial2D = slabOrigin.ToVector2d();
        planar.Initialize(initial2D, motionType: moving3D ? BodyMotionType.Static : BodyMotionType.Dynamic);
        int entered = 0, stayed = 0, exited = 0;
        cone.OnMixedContactEnter += _ => entered++;
        slab.OnMixedContactEnter += _ => entered++;
        cone.OnMixedContact += _ => stayed++;
        slab.OnMixedContact += _ => stayed++;
        cone.OnMixedContactExit += _ => exited++;
        slab.OnMixedContactExit += _ => exited++;

        CollisionDetectionMixed.TryCollide(cone, slab, out MixedContact contact).Should().BeTrue();
        // 2x+y<=1 supports the entire H=2,R=1 cone. The slab's lower
        // near edge is (1/2,0,0)-delta*(2,1,0), giving the exact seam
        // tangency or minimum sqrt(5)/64 already proved in the core tests.
        Fixed64 expectedDepth = Fixed64.FromRaw(inward ? 150_059_982L : 0);
        contact.Depth.Should().Be(expectedDepth);
        contact.DepthIsClamped.Should().BeFalse();
        contact.Normal3DTo2D.Should().Be(new Vector3d(
            Fixed64.FromRaw(3_841_535_534L), Fixed64.FromRaw(1_920_767_767L), Fixed64.Zero));
        contact.Point3D.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        contact.Point2D.Should().Be(new Vector3d(Fixed64.Half - Fixed64.Two * delta, -delta, Fixed64.Zero));

        Step(context);

        Assert.Equal(1, context.MixedCollisions.LastBroadPhaseCandidateCount);
        Assert.Equal(1, context.MixedCollisions.ActivePairCount);
        entered.Should().Be(2);
        stayed.Should().Be(2);
        exited.Should().Be(0);
        spatial.Rotation.Should().Be(FixedQuaternion.Identity);
        planar.Rotation.Should().Be(Fixed64.Zero);
        spatial.LinearVelocity.Should().Be(Vector3d.Zero);
        spatial.AngularVelocity.Should().Be(Vector3d.Zero);
        planar.LinearVelocity.Should().Be(Vector2d.Zero);
        planar.AngularVelocity.Should().Be(Fixed64.Zero);
        Vector3d first3D = spatial.Position3d;
        Vector2d first2D = planar.Position;
        if (!inward)
        {
            first3D.Should().Be(Vector3d.Zero);
            first2D.Should().Be(initial2D);
        }
        else
        {
            if (moving3D)
            {
                first3D.X.Should().BeLessThan(Fixed64.Zero);
                first3D.Y.Should().BeLessThan(Fixed64.Zero);
                (-first3D.X).Should().BeLessThan(expectedDepth);
                first2D.Should().Be(initial2D);
                spatial.AddLinearImpulse(-contact.Normal3DTo2D);
            }
            else
            {
                first3D.Should().Be(Vector3d.Zero);
                first2D.X.Should().BeGreaterThan(initial2D.X);
                (first2D.X - initial2D.X).Should().BeLessThan(expectedDepth * Fixed64.Two);
                planar.Agent.Transform.WorldPosition.Y.Should().Be(slabOrigin.Y);
                planar.AddLinearImpulse(Vector2d.Right);
            }
            // Positional response deliberately retains penetration slop.
            // An authored outward velocity takes the next full host step beyond
            // that resting region, where an independent support plane proves
            // that the complete shapes, rather than only their anchors, clear.
            Step(context);
            Fixed64 coneMaximum = spatial.Position3d.X * Fixed64.Two + spatial.Position3d.Y + Fixed64.One;
            Fixed64 prismMinimum = (planar.Position.X - Fixed64.Half) * Fixed64.Two
                + planar.Agent.Transform.WorldPosition.Y - Fixed64.One;
            (prismMinimum - coneMaximum).Should().BeGreaterThan(Fixed64.FromFraction(1, 8));
            CollisionDetectionMixed.TryCollide(cone, slab, out _).Should().BeFalse();
            Assert.Equal(0, context.MixedCollisions.ActivePairCount);
            entered.Should().Be(2);
            stayed.Should().Be(2);
            exited.Should().Be(2);
        }

        return (first3D, first2D, spatial.Position3d, planar.Position,
            spatial.LinearVelocity, planar.LinearVelocity, (entered, stayed, exited));
    }
}
