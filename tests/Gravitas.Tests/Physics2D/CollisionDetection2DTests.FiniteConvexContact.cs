using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using System;
using System.Reflection;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class CollisionDetection2DTests
{
    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, false)]
    [InlineData(false, true, true)]
    [InlineData(true, false, false)]
    [InlineData(true, false, true)]
    [InlineData(true, true, false)]
    [InlineData(true, true, true)]
    public void ConvexSingleContact_AndInitialOverlapSweep_ShouldRetainFinitePairedWitnesses(
        bool polygonMover, bool polygonFloor, bool reverse)
    {
        using GravitasWorldContext context = Create2DContext();
        // Translate the same exact geometry as well: anchors must retain the
        // finite overlap rather than substituting an arbitrary support endpoint.
        foreach (Vector2d origin in new[] { Vector2d.Zero, new Vector2d(1_000_000_000, -1_000_000_000) })
        {
            LSCollider2D floor = CreateContactRectangle(polygonFloor, new Vector2d(8, 4));
            var floorTransform = new FixedTransform(
                new Vector3d(origin.X, Fixed64.Zero, origin.Y - (Fixed64)2),
                FixedQuaternion.Identity, Vector3d.One);
            floor.InitializeWithNoBody(new TestMatterAgent(context, floorTransform));
            LSCollider2D mover = CreateContactRectangle(polygonMover, new Vector2d(1, 2));
            SolidBody2D body = CreateBody(context, mover,
                origin + new Vector2d(Fixed64.Quarter, Fixed64.FromFraction(3, 4)));
            LSCollider2D first = reverse ? floor : mover;
            LSCollider2D second = reverse ? mover : floor;

            CollisionDetection2D.TryCollide(first, second, out Contact2D contact).Should().BeTrue();
            contact.Normal.Should().Be(reverse ? Vector2d.Forward : -Vector2d.Forward);
            contact.Depth.Should().Be(Fixed64.Quarter);
            contact.DepthIsClamped.Should().BeFalse();
            Vector2d floorPoint = (reverse ? contact.PointA : contact.PointB) - origin;
            Vector2d moverPoint = (reverse ? contact.PointB : contact.PointA) - origin;
            floorPoint.Y.Should().Be(Fixed64.Zero);
            floorPoint.X.Should().BeInRange(-Fixed64.Quarter, Fixed64.FromFraction(3, 4));
            moverPoint.Should().Be(floorPoint - Vector2d.Forward * Fixed64.Quarter);

            (bool collided, ContactManifold2D manifold) = BuildManifold(first, second);
            collided.Should().BeTrue();
            manifold.Should().Contain(row => row.PointA == contact.PointA && row.PointB == contact.PointB
                && row.Normal == contact.Normal && row.Depth == contact.Depth);
            CollisionDetection2D.TryCollide(first, second, out Contact2D repeated).Should().BeTrue();
            repeated.Should().Be(contact);

            QueryDetection2D.TrySweepMoverShape(first, Vector2d.Right, second,
                out Physics2DHit hit).Should().BeTrue();
            hit.Collider.Should().BeSameAs(second);
            hit.Distance.Should().Be(Fixed64.Zero);
            hit.Normal.Should().Be(-contact.Normal);
            hit.Point.Should().Be(contact.PointB);

            // This convex face exit applies to the complete rectangle, not only
            // the selected witness: touching at depth, separated one raw farther.
            Vector2d start = body.Position;
            body.SetPosition(start + Vector2d.Forward * Fixed64.Quarter);
            CollisionDetection2D.TryCollide(mover, floor, out Contact2D touching).Should().BeTrue();
            touching.Depth.Should().Be(Fixed64.Zero);
            body.SetPosition(start + Vector2d.Forward * (Fixed64.Quarter + Fixed64.FromRaw(1)));
            CollisionDetection2D.TryCollide(mover, floor, out _).Should().BeFalse();
            body.Deactivate();
            floor.Deactivate();
        }
    }

    [Fact]
    public void RotationalContactSearch_ShouldRetainFiniteWitnessesAtItsReportedSamplePose()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        LSCollider2D floor = CreateContactRectangle(false, new Vector2d(8, 4));
        var floorTransform = new FixedTransform(new Vector3d(0, 0, -2),
            FixedQuaternion.Identity, Vector3d.One);
        floor.InitializeWithNoBody(new TestMatterAgent(context, floorTransform));
        var body = new SolidBody2D(new TestMatterAgent(context),
            CreateContactRectangle(true, new Vector2d(1, 2)));
        body.Initialize(new Vector2d(Fixed64.Quarter, Fixed64.FromFraction(3, 4)),
            motionType: BodyMotionType.Kinematic);
        Fixed64 angularDelta = Fixed64.FromFraction(1, 100);
        // Inspect the actual CCD search result without adding runtime diagnostics
        // solely to expose its private sampled contact to this regression.
        MethodInfo search = typeof(SolidBody2D).GetMethod(
            "TryFindEarliestRotationalContinuousCollisionAgainstTarget",
            BindingFlags.Instance | BindingFlags.NonPublic)!;
        object[] arguments = { floor, body.Position, Vector2d.Zero, Fixed64.Zero,
            angularDelta, angularDelta, body.ResolveContinuousCollisionProxyRadius(),
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, default(Contact2D), false, Fixed64.Zero };

        search.Invoke(body, arguments).Should().Be(true);

        ((bool)arguments[11]).Should().BeTrue();
        Fixed64 contactTime = (Fixed64)arguments[12];
        contactTime.Should().BeInRange(Fixed64.Zero, Fixed64.One);
        Contact2D contact = (Contact2D)arguments[10];
        body.SetRotation(angularDelta * contactTime);
        contact.PointB.Y.Should().Be(Fixed64.Zero);
        contact.PointB.X.Should().BeInRange(body.Collider.Bounds.Min.X, body.Collider.Bounds.Max.X);
        (bool collided, ContactManifold2D manifold) = BuildManifold(body.Collider, floor);
        collided.Should().BeTrue();
        manifold.Should().Contain(row => row.PointA == contact.PointA && row.PointB == contact.PointB
            && row.Normal == contact.Normal && row.Depth == contact.Depth);
        // Continue through the host phases to check sampled shape restoration
        // and a genuine witnessed rotational stop against the static floor.
        body.SetRotation(Fixed64.Zero);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        body.Agent.Transform.LocalRotationXZRadians = angularDelta;
        context.Simulate();
        context.LateSimulate();
        body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        body.Rotation.Should().BeInRange(Fixed64.Zero, angularDelta);
        body.Collider.Center.Should().Be(body.Position);
    }

    [Fact]
    public void ConvexSingleContact_AndInitialOverlapSweep_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D mover = CreateContactRectangle(true, new Vector2d(1, 2));
        LSCollider2D floor = CreateContactRectangle(false, new Vector2d(8, 4));
        _ = CreateBody(context, mover, new Vector2d(Fixed64.Quarter, Fixed64.FromFraction(3, 4)));
        _ = CreateBody(context, floor, new Vector2d(0, -2));
        Action collideAndSweep = () =>
        {
            _ = CollisionDetection2D.TryCollide(mover, floor, out _);
            _ = QueryDetection2D.TrySweepMoverShape(mover, Vector2d.Right, floor, out _);
        };

        AllocationTestHelper.MeasureSteadyState(collideAndSweep).Should().Be(0);
    }

    private static LSCollider2D CreateContactRectangle(bool polygon, Vector2d size)
    {
        Vector2d half = size * Fixed64.Half;
        return polygon
            ? new LSPolygonCollider2D(new Vector2d(-half.X, -half.Y), new Vector2d(half.X, -half.Y),
                new Vector2d(half.X, half.Y), new Vector2d(-half.X, half.Y))
            : new LSAABBoxCollider2D(size);
    }
}
