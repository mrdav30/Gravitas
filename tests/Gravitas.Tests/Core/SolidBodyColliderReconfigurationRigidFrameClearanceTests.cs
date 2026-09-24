using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyColliderReconfigurationRigidFrameClearanceTests
{
    // These exact unit quaternions have representable permutation matrices.
    // The source maps local +Y to world +Z; the target root flips Y and Z.
    private static readonly FixedQuaternion SourceRotation = new(
        Fixed64.Half, Fixed64.Half, Fixed64.Half, Fixed64.Half);
    private static readonly FixedQuaternion TargetRotation = new(
        Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

    public static IEnumerable<object[]> SourceCases()
    {
        for (int family = 0; family < 3; family++)
        for (long rawOffset = -1; rawOffset <= 1; rawOffset++)
            yield return new object[] { family, rawOffset };
    }

    [Theory]
    [MemberData(nameof(SourceCases))]
    public void ScaledOffsetRotatedCompound_ShouldUsePartFrameAndReturnRootBlocker(
        int sourceFamily, long rawOffset)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var part = new CompoundColliderPart(
            ColliderShapeDefinition.Cuboid(new Vector3d(2, 4, 8)),
            new Vector3d(1, 2, 3), SourceRotation, new Vector3d(3, 2, 1));
        var obstacle = new LSCompoundCollider(part);
        obstacle.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(new Vector3d(10, 20, 30), TargetRotation, new Vector3d(2, 3, 4))));

        // Scaled offset (2,6,12), followed by the root half-turn, gives
        // part center (12,14,18). Its local half-extents are (6,12,16).
        // The composed frame maps local X/Y/Z to world -Y/-Z/+X, so
        // its upper Y face is exactly 14+6=20. Every source has world-Y
        // radius 1, while capsule/cylinder cores extend along world Z.
        Vector3d sourceCenter = new((Fixed64)12,
            (Fixed64)21 + Fixed64.FromRaw(rawOffset), (Fixed64)18);
        SolidBody body = CreateSource(context, sourceFamily, sourceCenter);

        AssertReconfiguration(context, body, sourceFamily, sourceCenter, rawOffset, obstacle);
    }

    [Theory]
    [MemberData(nameof(SourceCases))]
    public void TranslatedOffCenterMesh_ShouldUseCenteredLocalVerticesAndMeshFrame(
        int sourceFamily, long rawOffset)
    {
        foreach (MeshColliderMode mode in new[] { MeshColliderMode.Convex, MeshColliderMode.Concave })
        {
            using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
            Vector3d[] points =
            {
                new(2,4,5), new(6,4,5), new(6,10,5), new(2,10,5),
                new(2,4,13), new(6,4,13), new(6,10,13), new(2,10,13)
            };
            int[] faces =
            {
                0,2,1, 0,3,2, 4,5,6, 4,6,7, 0,1,5, 0,5,4,
                3,7,6, 3,6,2, 0,4,7, 0,7,3, 1,2,6, 1,6,5
            };
            var obstacle = new LSMeshCollider(points, faces, mode);
            obstacle.InitializeWithNoBody(new TestMatterAgent(context,
                new FixedTransform(new Vector3d(30, -20, 40), TargetRotation, new Vector3d(2, 3, 4))));

            // Authored bounds center (4,7,9) is scaled to (8,21,36)
            // and half-turned to (8,-21,-36), yielding mesh origin
            // (38,-41,4). Half-extents (4,9,16) put its upper face
            // at Y=-32, independently of the mesh's recentering cache.
            obstacle.LocalOffset.Should().Be(new Vector3d(4, 7, 9));
            obstacle.Mesh.Origin.Should().Be(new Vector3d(38, -41, 4));
            Vector3d sourceCenter = new((Fixed64)38,
                (Fixed64)(-31) + Fixed64.FromRaw(rawOffset), (Fixed64)4);
            SolidBody body = CreateSource(context, sourceFamily, sourceCenter);

            AssertReconfiguration(context, body, sourceFamily, sourceCenter, rawOffset, obstacle);
        }
    }

    private static SolidBody CreateSource(GravitasWorldContext context, int family, Vector3d position)
    {
        LSCollider collider = SourceShape(family, Fixed64.Half).CreateCollider();
        var body = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(position, SourceRotation, Vector3d.One)), collider)
        { Mass = Fixed64.One };
        body.Initialize(position, SourceRotation, BodyMotionType.Kinematic);
        return body;
    }

    private static void AssertReconfiguration(GravitasWorldContext context, SolidBody body,
        int family, Vector3d position, long rawOffset, LSCollider obstacle)
    {
        var before = context.ComputeReplayHash();
        LSCollider originalCollider = body.Collider;
        int publications = 0;
        ColliderReconfigurationStatus result = body.TryReconfigureCollider(
            SourceShape(family, Fixed64.One), Vector3d.Zero, position,
            out var blocker, out var failure, () => publications++);

        bool penetrates = rawOffset < 0;
        result.Should().Be(penetrates ? ColliderReconfigurationStatus.Blocked : ColliderReconfigurationStatus.Applied);
        failure.Should().BeNull();
        publications.Should().Be(penetrates ? 0 : 1);
        body.Collider.Should().BeSameAs(originalCollider);
        body.Rotation.Should().Be(SourceRotation);
        if (penetrates)
        {
            blocker.Should().BeSameAs(obstacle);
            context.ComputeReplayHash().Should().Be(before);
            body.Collider.ScaledRadius.Should().Be(Fixed64.Half);
        }
        else
        {
            blocker.Should().BeNull();
            body.Collider.ScaledRadius.Should().Be(Fixed64.One);
        }
    }

    private static ColliderShapeDefinition SourceShape(int family, Fixed64 radius) => family switch
    {
        0 => ColliderShapeDefinition.Sphere(radius),
        1 => ColliderShapeDefinition.Capsule(radius, Fixed64.Two + Fixed64.Two * radius),
        _ => ColliderShapeDefinition.Cylinder(radius, (Fixed64)4 * radius)
    };
}
