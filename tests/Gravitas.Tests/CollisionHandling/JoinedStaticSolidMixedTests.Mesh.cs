using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests;

/// <content>Checks finite moving mesh patches against declared static prism boundaries.</content>
public sealed partial class JoinedStaticSolidMixedTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void OpenMeshAcrossJoinedSolids_ShouldKeepTheRealOuterExit(bool convex, bool reverse)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, 0, -2, 2, Fixed64.Two),
            Rectangle(scene, 0, 2, -2, 2, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 h = Fixed64.Half;
        var mesh = new LSMeshCollider(new[] { new Vector3d(-h, Fixed64.Zero, -h), new(h, Fixed64.Zero, -h),
            new(-h, Fixed64.Zero, h), new(h, Fixed64.Zero, h) }, reverse ? new[] { 0, 3, 2, 0, 1, 3 } : new[] { 0, 2, 1, 1, 2, 3 },
            convex ? MeshColliderMode.Convex : MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
        { Material = PhysicsMaterial.Frictionless };
        Vector3d start = new(Fixed64.Quarter, 3 * h, Fixed64.Zero);
        SolidBody body = scene.CreateBody(mesh, start, FixedQuaternion.Identity).Body;
        var rows = Rows(mesh, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.Equal(Vector3d.Down, sample.Row.Normal);
            Assert.Equal(h, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
            Assert.Equal(Fixed64.Two, point.Y);
        }
        body.FreezeAxes = BodyFreezeAxes3D.Rotation; body.SleepEnabled = false; body.UseManualGrounding();
        scene.Context.Simulate(); scene.Context.LateSimulate();
        Assert.Equal(start.X, body.Position3d.X); Assert.Equal(start.Z, body.Position3d.Z);
        Assert.InRange(body.Position3d.Y.m_rawValue, start.Y.m_rawValue, Fixed64.Two.m_rawValue);
        AssertWholeShapeExit(mesh, members, new(start.X, Fixed64.Two, start.Z), Vector3d.Up);
    }

    [Fact]
    public void OpenMeshSlidingAcrossTheJoin_ShouldRetainTangentialVelocity()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -8, 0, -2, 0, Fixed64.Two),
            Rectangle(scene, 0, 8, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 h = Fixed64.Half;
        var mesh = new LSMeshCollider(new[] { new Vector3d(-h, Fixed64.Zero, -h), new(h, Fixed64.Zero, -h),
            new(-h, Fixed64.Zero, h), new(h, Fixed64.Zero, h) }, new[] { 0, 2, 1, 1, 2, 3 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation) { Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(mesh, new(-Fixed64.Two, Fixed64.Zero, h), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.Rotation;
        body.SleepEnabled = false; body.UseManualGrounding();
        for (int frame = 0; frame < 32; frame++) Step();
        Fixed64 direction = Fixed64.One;
        for (int frame = 0; frame < 256; frame++)
        {
            if (body.Position3d.X >= Fixed64.Two) direction = -Fixed64.One;
            else if (body.Position3d.X <= -Fixed64.Two) direction = Fixed64.One;
            body.AddLinearImpulse(Vector3d.Right * (direction - body.LinearVelocity.X));
            Step();
            if (body.LinearVelocity.X != direction) Assert.Fail(
                $"Frame {frame}: {body.Position3d}, {body.LinearVelocity}; rows " +
                string.Join("; ", Rows(mesh, members).ConvertAll(row => $"{row.Supplier}/{row.Row.Normal}/{row.Row.Depth}")));
            Assert.InRange(body.Position3d.Z.m_rawValue, Fixed64.Quarter.m_rawValue, Fixed64.Half.m_rawValue);
        }
        void Step()
        {
            body.AddLinearImpulse(Vector3d.Backward * Fixed64.FromFraction(1, 16));
            scene.Context.Simulate(); scene.Context.LateSimulate();
        }
    }
}
