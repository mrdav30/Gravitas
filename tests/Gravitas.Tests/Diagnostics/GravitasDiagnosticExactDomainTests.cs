using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Diagnostics;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Diagnostics;

public sealed class GravitasDiagnosticExactDomainTests
{
    [Fact]
    public void CaptureCollider_WithUnmaterializableMeshTriangles_ShouldOmitInvalidCommands()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector3d[] vertices =
        {
            new(-Fixed64.Two, Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.One, Fixed64.One, Fixed64.Zero),
            new(Fixed64.One, Fixed64.Zero, Fixed64.One)
        };
        int[] triangles = { 0, 1, 2 };
        var standalone = new LSMeshCollider(
            vertices,
            triangles,
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
        var compound = new LSCompoundCollider(
            CompoundColliderPart.ConvexMesh(
                vertices,
                triangles,
                Vector3d.Zero,
                MeshInertiaPolicy.SurfaceApproximation));
        InitializeAtScalarFace(context, standalone);
        InitializeAtScalarFace(context, compound);
        var compoundMesh = (LSMeshCollider)compound.GetPartCollider(0);
        standalone.Mesh.TryGetTriangleVertices(0, out _, out _, out _)
            .Should().BeFalse();
        compoundMesh.Mesh.TryGetTriangleVertices(0, out _, out _, out _)
            .Should().BeFalse();

        context.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 2);
        context.Diagnostics.CaptureCollider(
            standalone,
            GravitasDiagnosticColor.Cyan);
        context.Diagnostics.CaptureCollider(
            compound,
            GravitasDiagnosticColor.Cyan);

        context.Diagnostics.DrawCommands.Length.Should().Be(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CaptureMixedCollider_WithOneRepresentableSlabFace_ShouldEmitThatFace(
        bool positiveFace)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        Fixed64 centerY = positiveFace
            ? Fixed64.MaxValue - Fixed64.FromFraction(1, 4)
            : Fixed64.MinValue + Fixed64.FromFraction(1, 4);
        var polygon = new LSPolygonCollider2D(
            Vector2d.Forward,
            Vector2d.Right,
            Vector2d.Backward,
            Vector2d.Left);
        polygon.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(
                new Vector3d(Fixed64.Zero, centerY, Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.One)));
        Fixed64 expectedY = positiveFace
            ? centerY - polygon.MixedHalfThickness
            : centerY + polygon.MixedHalfThickness;

        context.Diagnostics.Enable(eventCapacity: 0, drawCommandCapacity: 4);
        context.Diagnostics.CaptureMixedCollider(
            polygon,
            GravitasDiagnosticColor.Cyan);

        context.Diagnostics.DrawCommands.Length.Should().Be(polygon.VertexCount);
        for (int i = 0; i < context.Diagnostics.DrawCommands.Length; i++)
        {
            GravitasDebugDrawCommand command =
                context.Diagnostics.DrawCommands[i];
            command.Kind.Should().Be(GravitasDebugDrawKind.Line);
            command.Start.Y.Should().Be(expectedY);
            command.End.Y.Should().Be(expectedY);
        }
    }

    private static void InitializeAtScalarFace(
        GravitasWorldContext context,
        LSCollider collider)
    {
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        transform.LocalPosition = new Vector3d(
            Fixed64.MaxValue - Fixed64.FromFraction(1, 8),
            Fixed64.Zero,
            Fixed64.Zero);
        collider.RebuildRuntimeShapeOnly(refreshMassProperties: false);
    }
}
