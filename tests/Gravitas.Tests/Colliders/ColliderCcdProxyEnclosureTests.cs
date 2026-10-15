using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderCcdProxyEnclosureTests
{
    [Theory]
    [InlineData(false, 0)]
    [InlineData(false, 1)]
    [InlineData(true, 0)]
    [InlineData(true, 1)]
    [InlineData(false, 2)]
    [InlineData(false, 3)]
    [InlineData(true, 2)]
    [InlineData(true, 3)]
    public void PlanarBodyProxy_ShouldEncloseEveryRoundedRotationOfPublishedLeaves(
        bool compound, int placement)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        Vector2d attachment = new(Fixed64.FromRaw(3), Fixed64.Zero);
        LSCollider2D collider = compound
            ? new LSCompoundCollider2D(
                CompoundColliderPart2D.Circle((Fixed64)400, attachment),
                CompoundColliderPart2D.Circle((Fixed64)399, Vector2d.Zero))
            : new LSCircleCollider2D((Fixed64)400);
        collider.LocalOffset = attachment;
        var body = new SolidBody2D(new TestMatterAgent(context), collider);
        body.Initialize(Vector2d.Zero);
        Fixed64 frameRadius = body.ResolveContinuousCollisionProxyRadius();
        Vector2d publishedPivot = GetPublishedPivot(placement).ToVector2d();

        for (int step = -12; step <= 12; step++)
        {
            Fixed64.TryMultiplyDivide(
                Fixed64.Pi, (Fixed64)step, (Fixed64)12, out Fixed64 angle).Should().BeTrue();
            collider.TryPrepareBodyPose(publishedPivot, angle).Should().BeTrue();
            collider.PublishPreparedBodyPose();
            body.Position.Should().Be(Vector2d.Zero);

            int count = compound ? ((LSCompoundCollider2D)collider).PartCount : 1;
            for (int index = 0; index < count; index++)
            {
                var leaf = (LSCircleCollider2D)(compound
                    ? ((LSCompoundCollider2D)collider).GetPartCollider(index)
                    : collider);
                (leaf.Center - publishedPivot).TryGetMagnitudeCeiling(
                    out Fixed64 centerReach).Should().BeTrue();
                // A circle's farthest point has radius |center - pivot| + r;
                // its ceiling is exact because r is already an integer raw value.
                Fixed64 required = centerReach + leaf.ScaledRadius;
                frameRadius.Should().BeGreaterThanOrEqualTo(required,
                    "the frame proxy must enclose leaf {0} at angle step {1}", index, step);
                body.ResolveContinuousCollisionProxyRadius().Should().BeGreaterThanOrEqualTo(required,
                    "a published pose must use its own pivot even when the body pose lags");
            }
        }
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(false, 1)]
    [InlineData(true, 0)]
    [InlineData(true, 1)]
    [InlineData(false, 2)]
    [InlineData(false, 3)]
    [InlineData(true, 2)]
    [InlineData(true, 3)]
    public void SpatialBodyProxy_ShouldEncloseEveryRoundedRotationOfPublishedLeaves(
        bool compound, int placement)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector3d attachment = new(Fixed64.FromRaw(3), Fixed64.Zero, Fixed64.Zero);
        LSCollider collider = compound
            ? new LSCompoundCollider(
                CompoundColliderPart.Sphere((Fixed64)400, attachment),
                CompoundColliderPart.Sphere((Fixed64)399, Vector3d.Zero))
            : new LSSphereCollider { Radius = (Fixed64)400 };
        collider.LocalOffset = attachment;
        var body = new SolidBody(new TestMatterAgent(context), collider);
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        Fixed64 frameRadius = body.ResolveContinuousCollisionProxyRadius();
        Vector3d publishedPivot = GetPublishedPivot(placement);

        for (int axis = 0; axis < 2; axis++)
        for (int step = -12; step <= 12; step++)
        {
            Fixed64.TryMultiplyDivide(
                Fixed64.Pi, (Fixed64)step, (Fixed64)12, out Fixed64 angle).Should().BeTrue();
            Vector3d rotationAxis = axis == 0 ? Vector3d.Up : Vector3d.One.Normalized;
            FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(rotationAxis, angle);
            collider.TryPrepareBodyPose(publishedPivot, rotation).Should().BeTrue();
            collider.PublishPreparedBodyPose();
            body.Position3d.Should().Be(Vector3d.Zero);

            int count = compound ? ((LSCompoundCollider)collider).PartCount : 1;
            for (int index = 0; index < count; index++)
            {
                var leaf = (LSSphereCollider)(compound
                    ? ((LSCompoundCollider)collider).GetPartCollider(index)
                    : collider);
                (leaf.Center - publishedPivot).TryGetMagnitudeCeiling(
                    out Fixed64 centerReach).Should().BeTrue();
                Fixed64 required = centerReach + leaf.ScaledRadius;
                frameRadius.Should().BeGreaterThanOrEqualTo(required,
                    "the frame proxy must enclose leaf {0} at axis {1}, angle step {2}", index, axis, step);
                body.ResolveContinuousCollisionProxyRadius().Should().BeGreaterThanOrEqualTo(required,
                    "a published pose must use its own pivot even when the body pose lags");
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PlanarBodyProxy_ShouldEncloseEveryPublishedPolygonVertex(bool compound)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        Vector2d attachment = new(Fixed64.FromRaw(3), Fixed64.Zero);
        Vector2d[] vertices =
        {
            new(-2400, -3200), new(2400, -3200),
            new(2400, 3200), new(-2400, 3200)
        };
        LSCollider2D collider = compound
            ? new LSCompoundCollider2D(CompoundColliderPart2D.ConvexPolygon(vertices, attachment))
            : new LSPolygonCollider2D(vertices);
        collider.LocalOffset = attachment;
        var body = new SolidBody2D(new TestMatterAgent(context), collider);
        body.Initialize(Vector2d.Zero);
        Fixed64 frameRadius = body.ResolveContinuousCollisionProxyRadius();
        var polygon = (LSPolygonCollider2D)(compound
            ? ((LSCompoundCollider2D)collider).GetPartCollider(0) : collider);
        Vector2d publishedPivot = new(10, -7);

        for (int step = -12; step <= 12; step++)
        {
            Fixed64.TryMultiplyDivide(
                Fixed64.Pi, (Fixed64)step, (Fixed64)12, out Fixed64 angle).Should().BeTrue();
            collider.TryPrepareBodyPose(publishedPivot, angle).Should().BeTrue();
            collider.PublishPreparedBodyPose();
            for (int index = 0; index < polygon.Count; index++)
            {
                (polygon.GetWorldVertex(index) - publishedPivot).TryGetMagnitudeCeiling(
                    out Fixed64 required).Should().BeTrue();
                frameRadius.Should().BeGreaterThanOrEqualTo(required,
                    "the frame proxy must enclose polygon vertex {0} at angle step {1}", index, step);
                body.ResolveContinuousCollisionProxyRadius().Should().BeGreaterThanOrEqualTo(required);
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SpatialBodyProxy_ShouldEncloseEveryPublishedCuboidCorner(bool compound)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector3d attachment = new(Fixed64.FromRaw(3), Fixed64.Zero, Fixed64.Zero);
        Vector3d size = new(4800, 6400, 2);
        LSCollider collider = compound
            ? new LSCompoundCollider(CompoundColliderPart.Cuboid(size, attachment))
            : new LSCuboidCollider { Size = size };
        collider.LocalOffset = attachment;
        var body = new SolidBody(new TestMatterAgent(context), collider);
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        Fixed64 frameRadius = body.ResolveContinuousCollisionProxyRadius();
        var cuboid = (LSCuboidCollider)(compound
            ? ((LSCompoundCollider)collider).GetPartCollider(0) : collider);
        Vector3d publishedPivot = new(10, -7, 5);

        for (int axis = 0; axis < 2; axis++)
        for (int step = -12; step <= 12; step++)
        {
            Fixed64.TryMultiplyDivide(
                Fixed64.Pi, (Fixed64)step, (Fixed64)12, out Fixed64 angle).Should().BeTrue();
            Vector3d rotationAxis = axis == 0 ? Vector3d.Up : Vector3d.One.Normalized;
            FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(rotationAxis, angle);
            collider.TryPrepareBodyPose(publishedPivot, rotation).Should().BeTrue();
            collider.PublishPreparedBodyPose();
            Vector3d halfExtents = cuboid.OrientedBox.HalfExtents;
            for (int corner = 0; corner < 8; corner++)
            {
                Vector3d localCorner = new(
                    (corner & 1) == 0 ? -halfExtents.X : halfExtents.X,
                    (corner & 2) == 0 ? -halfExtents.Y : halfExtents.Y,
                    (corner & 4) == 0 ? -halfExtents.Z : halfExtents.Z);
                cuboid.CanonicalRotation.TryTransformScaledPoint(
                    cuboid.CanonicalCenter, localCorner, Vector3d.One, out Vector3d witness).Should().BeTrue();
                (witness - publishedPivot).TryGetMagnitudeCeiling(out Fixed64 required).Should().BeTrue();
                frameRadius.Should().BeGreaterThanOrEqualTo(required,
                    "the frame proxy must enclose cuboid corner {0} at axis {1}, angle step {2}", corner, axis, step);
                body.ResolveContinuousCollisionProxyRadius().Should().BeGreaterThanOrEqualTo(required);
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PlanarBodyProxy_ShouldEnclosePublishedCapsuleSupport(bool compound)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        Vector2d attachment = new(Fixed64.FromRaw(3), Fixed64.Zero);
        LSCollider2D collider = compound
            ? new LSCompoundCollider2D(CompoundColliderPart2D.Capsule((Fixed64)1000, (Fixed64)6000, attachment))
            : new LSCapsuleCollider2D((Fixed64)1000, (Fixed64)6000);
        collider.LocalOffset = attachment;
        var body = new SolidBody2D(new TestMatterAgent(context), collider);
        body.Initialize(Vector2d.Zero);
        Fixed64 frameRadius = body.ResolveContinuousCollisionProxyRadius();
        var capsule = (LSCapsuleCollider2D)(compound
            ? ((LSCompoundCollider2D)collider).GetPartCollider(0) : collider);
        Vector2d publishedPivot = new(10, -7);
        Vector2d[] directions = { Vector2d.Right, -Vector2d.Right, Vector2d.Forward, -Vector2d.Forward, Vector2d.One };
        for (int step = -12; step <= 12; step++)
        {
            Fixed64.TryMultiplyDivide(Fixed64.Pi, (Fixed64)step, (Fixed64)12,
                out Fixed64 angle).Should().BeTrue();
            collider.TryPrepareBodyPose(publishedPivot, angle).Should().BeTrue();
            collider.PublishPreparedBodyPose();
            foreach (Vector2d direction in directions)
            {
                (capsule.GetSupportPoint(direction) - publishedPivot).TryGetMagnitudeCeiling(
                    out Fixed64 required).Should().BeTrue();
                frameRadius.Should().BeGreaterThanOrEqualTo(required);
                body.ResolveContinuousCollisionProxyRadius().Should().BeGreaterThanOrEqualTo(required);
            }
        }
    }

    private static Vector3d GetPublishedPivot(int placement) => placement switch
    {
        0 => Vector3d.Zero,
        1 => new Vector3d(10, -7, 5),
        2 => Vector3d.One * (Fixed64.MaxValue - (Fixed64)4096),
        _ => Vector3d.One * (Fixed64.MinValue + (Fixed64)4096)
    };
}
