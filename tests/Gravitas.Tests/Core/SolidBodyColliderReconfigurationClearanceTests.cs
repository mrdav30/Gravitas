using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyColliderReconfigurationClearanceTests
{
    [Theory]
    [InlineData(-1L, ColliderReconfigurationStatus.Blocked)]
    [InlineData(0L, ColliderReconfigurationStatus.Applied)]
    [InlineData(1L, ColliderReconfigurationStatus.Applied)]
    public void CapsuleEndpointAtCylinderRim_ShouldUseExactDistance(long offset, ColliderReconfigurationStatus expected)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector3d position = new(Fixed64.FromFraction(83, 4), Fixed64.Two + Fixed64.FromRaw(offset), Fixed64.Zero);
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, -Fixed64.One, Fixed64.One).Normalized;
        var source = ColliderShapeDefinition.Capsule(Fixed64.Half, (Fixed64)21).CreateCollider();
        var body = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(position, rotation, Vector3d.One)), source) { Mass = Fixed64.One };
        body.Initialize(position, rotation, BodyMotionType.Kinematic);
        LSCollider obstacle = ColliderShapeDefinition.Cylinder((Fixed64)10, Fixed64.Two).CreateCollider();
        obstacle.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        var before = context.ComputeReplayHash();

        var result = body.TryReconfigureCollider(
            ColliderShapeDefinition.Capsule(Fixed64.FromFraction(5, 4), Fixed64.FromFraction(45, 2)),
            Vector3d.Zero, position, out var blocker, out var failure);

        // Nearest endpoint is (10.75,2+e,0), nearest rim is (10,1,0).
        // Its squared distance equals (5/4)^2 + 2e + e^2.
        result.Should().Be(expected);
        failure.Should().BeNull();
        if (expected == ColliderReconfigurationStatus.Blocked)
        {
            blocker.Should().BeSameAs(obstacle);
            context.ComputeReplayHash().Should().Be(before);
        }
        else blocker.Should().BeNull();
    }

    [Theory]
    [InlineData(-1L, ColliderReconfigurationStatus.Blocked)]
    [InlineData(0L, ColliderReconfigurationStatus.Applied)]
    [InlineData(1L, ColliderReconfigurationStatus.Applied)]
    public void PerpendicularCylinderRims_ShouldNotUseSampledContactDepth(long offset, ColliderReconfigurationStatus expected)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var source = ColliderShapeDefinition.Cylinder(Fixed64.Half, Fixed64.Two).CreateCollider();
        var body = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), source) { Mass = Fixed64.One };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, BodyMotionType.Kinematic);
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        LSCollider obstacle = ColliderShapeDefinition.Cylinder(Fixed64.FromFraction(5, 4), Fixed64.Two).CreateCollider();
        obstacle.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(Fixed64.FromFraction(7, 4), Fixed64.FromFraction(7, 4),
                Fixed64.Two + Fixed64.FromRaw(offset)), rotation, Vector3d.One)));
        var before = context.ComputeReplayHash();

        var result = body.TryReconfigureCollider(
            ColliderShapeDefinition.Cylinder(Fixed64.FromFraction(5, 4), Fixed64.Two),
            Vector3d.Zero, Vector3d.Zero, out var blocker, out var failure);

        // Each disk has one unit of Z reach at x/y offset3/4. At zero
        // offset the sole common point is (3/4,1,1), not positive overlap.
        result.Should().Be(expected);
        failure.Should().BeNull();
        if (expected == ColliderReconfigurationStatus.Blocked)
        {
            blocker.Should().BeSameAs(obstacle);
            context.ComputeReplayHash().Should().Be(before);
        }
        else blocker.Should().BeNull();
    }

    [Fact]
    public void CylinderCrossingTriangleBelowCap_ShouldBlockEvenWhenCenterWitnessMisses()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var source = ColliderShapeDefinition.Cylinder(Fixed64.One, Fixed64.Two).CreateCollider();
        var body = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), source) { Mass = Fixed64.One };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, BodyMotionType.Kinematic);
        var obstacle = new LSMeshCollider(new[] { new Vector3d(0, 7, -5), new Vector3d(10, 1, -5), new Vector3d(5, 4, 5) },
            new[] { 0, 1, 2 }, MeshColliderMode.Concave);
        obstacle.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        var before = context.ComputeReplayHash();

        var result = body.TryReconfigureCollider(ColliderShapeDefinition.Cylinder((Fixed64)5, (Fixed64)10),
            Vector3d.Zero, Vector3d.Zero, out var blocker, out var failure);

        // (4,23/5,0) is inside the triangle and the replacement cylinder.
        result.Should().Be(ColliderReconfigurationStatus.Blocked);
        blocker.Should().BeSameAs(obstacle);
        failure.Should().BeNull();
        context.ComputeReplayHash().Should().Be(before);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(2, true)]
    public void EnclosedCandidate_ShouldRespectVolumeVersusSurfaceMeshSemantics(int sourceFamily, bool concave)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var source = SourceShape(sourceFamily, Fixed64.Half).CreateCollider();
        var body = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), source)
        { Mass = Fixed64.One };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, BodyMotionType.Kinematic);
        LSCollider shell = CreateTarget(concave ? 8 : 5, compound: false);
        shell.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, new Vector3d(10, 10, 10))));
        var before = context.ComputeReplayHash();

        var result = body.TryReconfigureCollider(SourceShape(sourceFamily, Fixed64.One),
            Vector3d.Zero, Vector3d.Zero, out var blocker, out var failure);

        // Every triangle is at least eight units from the candidate. Convex
        // mode fills the volume; concave mode collides with its surface only.
        result.Should().Be(concave ? ColliderReconfigurationStatus.Applied : ColliderReconfigurationStatus.Blocked);
        failure.Should().BeNull();
        if (concave)
            blocker.Should().BeNull();
        else
        {
            blocker.Should().BeSameAs(shell);
            context.ComputeReplayHash().Should().Be(before);
        }
    }

    public static IEnumerable<object[]> TargetFamilies()
    {
        for (int source = 0; source < 3; source++)
        for (int target = 0; target < 8; target++)
            yield return new object[] { source, target };
    }

    [Theory]
    [MemberData(nameof(TargetFamilies))]
    public void Clearance_ShouldDistinguishTouchingFromPenetrationAcrossSupportedFamilies(
        int sourceFamily, int targetFamily)
    {
        foreach (bool compound in new[] { false, true })
        foreach (int offset in new[] { -1, 0, 1 })
        {
            // Concave mesh is a standalone surface, not an authored compound part.
            if (compound && targetFamily == 7) continue;
            using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
            Fixed64 targetTop = targetFamily >= 6 ? Fixed64.Zero
                : targetFamily == 1 ? Fixed64.Two : Fixed64.One;
            Fixed64 sourceHalfHeight = sourceFamily == 1 ? Fixed64.Two : Fixed64.One;
            Vector3d position = new(Fixed64.Zero,
                targetTop + sourceHalfHeight + Fixed64.FromRaw(offset), Fixed64.Zero);
            ColliderShapeDefinition initial = SourceShape(sourceFamily, Fixed64.Half);
            LSCollider source = initial.CreateCollider();
            var body = new SolidBody(new TestMatterAgent(context,
                new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One)), source)
            { Mass = Fixed64.One };
            body.Initialize(position, FixedQuaternion.Identity, BodyMotionType.Kinematic);
            LSCollider obstacle = CreateTarget(targetFamily, compound);
            obstacle.InitializeWithNoBody(new TestMatterAgent(context,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
            var before = context.ComputeReplayHash();
            int publications = 0;

            ColliderReconfigurationStatus result = body.TryReconfigureCollider(
                SourceShape(sourceFamily, Fixed64.One), Vector3d.Zero, position,
                out var blocker, out var failure, () => publications++);

            // Every target has a known top support point; source bottom is
            // exactly targetTop+offset. No contact result defines this oracle.
            bool penetrates = offset < 0;
            result.Should().Be(penetrates ? ColliderReconfigurationStatus.Blocked
                : ColliderReconfigurationStatus.Applied,
                $"source {sourceFamily}, target {targetFamily}, compound {compound}, offset {offset}");
            failure.Should().BeNull();
            publications.Should().Be(penetrates ? 0 : 1);
            if (penetrates)
            {
                blocker.Should().BeSameAs(obstacle);
                context.ComputeReplayHash().Should().Be(before);
                body.Collider.Should().BeSameAs(source);
            }
            else
            {
                blocker.Should().BeNull();
                source.ScaledRadius.Should().Be(Fixed64.One);
            }
        }
    }

    private static ColliderShapeDefinition SourceShape(int family, Fixed64 radius) => family switch
    {
        0 => ColliderShapeDefinition.Sphere(radius),
        1 => ColliderShapeDefinition.Capsule(radius, Fixed64.Two + radius * Fixed64.Two),
        _ => ColliderShapeDefinition.Cylinder(radius, radius * Fixed64.Two)
    };

    private static LSCollider CreateTarget(int family, bool compound)
    {
        Vector3d[] cube = {
            new(-1,-1,-1), new(1,-1,-1), new(1,1,-1), new(-1,1,-1),
            new(-1,-1,1), new(1,-1,1), new(1,1,1), new(-1,1,1)
        };
        int[] faces = { 0,2,1, 0,3,2, 4,5,6, 4,6,7, 0,1,5, 0,5,4,
            3,7,6, 3,6,2, 0,4,7, 0,7,3, 1,2,6, 1,6,5 };
        if (family == 8)
            return new LSMeshCollider(cube, faces, MeshColliderMode.Concave);
        Vector3d[] plane = { new(-2,0,-2), new(2,0,-2), new(0,0,2) };
        int[] triangle = { 0,2,1 };
        if (family == 7)
            return new LSMeshCollider(plane, triangle, MeshColliderMode.Concave);
        ColliderShapeDefinition shape = family switch
        {
            0 => ColliderShapeDefinition.Sphere(Fixed64.One),
            1 => ColliderShapeDefinition.Capsule(Fixed64.One, (Fixed64)4),
            2 => ColliderShapeDefinition.Cuboid(new Vector3d(2,2,2)),
            3 => ColliderShapeDefinition.Cylinder(Fixed64.One, Fixed64.Two),
            4 => ColliderShapeDefinition.Cone(Fixed64.One, Fixed64.Two),
            5 => ColliderShapeDefinition.ConvexMesh(cube, faces),
            _ => ColliderShapeDefinition.ConvexMesh(plane, triangle)
        };
        return compound ? new LSCompoundCollider(new CompoundColliderPart(shape)) : shape.CreateCollider();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SubRawCornerPenetration_ShouldBlockDespiteRoundedZeroContactDepth(bool capsule)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 raw = Fixed64.FromRaw(1);
        Vector3d position = new((Fixed64)3 + raw, Fixed64.Zero, (Fixed64)4 - raw);
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        LSCollider source = capsule
            ? new LSCapsuleCollider(ColliderShapeDefinition.Capsule((Fixed64)4, (Fixed64)8))
            : new LSSphereCollider(ColliderShapeDefinition.Sphere((Fixed64)4));
        var body = new SolidBody(new TestMatterAgent(context, transform), source) { Mass = Fixed64.One };
        body.Initialize(position, FixedQuaternion.Identity, BodyMotionType.Kinematic);
        var obstacle = new LSCuboidCollider(ColliderShapeDefinition.Cuboid(new Vector3d(2, 2, 2)));
        obstacle.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(-1, 0, -1), FixedQuaternion.Identity, Vector3d.One)));
        var definition = capsule
            ? ColliderShapeDefinition.Capsule((Fixed64)5, (Fixed64)10)
            : ColliderShapeDefinition.Sphere((Fixed64)5);
        var hash = context.ComputeReplayHash();

        var result = body.TryReconfigureCollider(definition, Vector3d.Zero, position, out var blocker, out var failure);

        result.Should().Be(ColliderReconfigurationStatus.Blocked);
        blocker.Should().BeSameAs(obstacle);
        failure.Should().BeNull();
        context.ComputeReplayHash().Should().Be(hash);
    }
}
