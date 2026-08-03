using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class PhysicsMeshScaleTests
{
    [Fact]
    public void AreaAdmittedMicroTriangle_ShouldRetainAUsableFaceNormal()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right * Fixed64.Half,
                Vector3d.Up * Fixed64.FromFraction(1, 65536)
            },
            new[] { 0, 1, 2 },
            Vector3d.Zero,
            FixedQuaternion.Identity);

        mesh.TotalArea.Should().Be(Fixed64.FromFraction(1, 262144));
        mesh.GetFaceNormalWorld(0).Should().Be(Vector3d.Forward);
    }

    [Fact]
    public void CompoundMesh_WithUnrepresentableCombinedScale_ShouldCommitExactCenteredVertices()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector3d[] vertices =
        {
            Vector3d.Zero,
            Vector3d.Right * Fixed64.Half,
            Vector3d.Up * Fixed64.FromFraction(1, 65536)
        };
        var compound = new LSCompoundCollider(
            CompoundColliderPart.ConvexMesh(
                vertices,
                new[] { 0, 1, 2 },
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d((Fixed64)2, Fixed64.One, Fixed64.One),
                MeshInertiaPolicy.RequireClosedVolume));
        Fixed64 ownerX = (Fixed64)1500000000;
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(ownerX, Fixed64.One, Fixed64.One));

        compound.InitializeWithNoBody(new TestMatterAgent(context, transform));

        var meshPart = (LSMeshCollider)compound.GetPartCollider(0);
        meshPart.Mesh.OwnerScale.X.Should().Be(ownerX);
        meshPart.Mesh.PartScale.X.Should().Be((Fixed64)2);
        meshPart.Mesh.GetVertexWorld(0).X.Should().Be((Fixed64)(-750000000));
        meshPart.Mesh.GetVertexWorld(1).X.Should().Be((Fixed64)750000000);
        meshPart.Area.Should().Be(Fixed64.FromFraction(1500000000, 131072));
    }

    [Fact]
    public void ClosedMesh_WithUnrepresentableScaledSourceCenter_ShouldUseCenteredCandidateMass()
    {
        Vector3d translation = Vector3d.Right * (Fixed64)1200000000;
        Vector3d[] vertices = CreateTetrahedronVertices();
        for (int i = 0; i < vertices.Length; i++)
            vertices[i] += translation;

        var mesh = new PhysicsMesh(
            vertices,
            TetrahedronTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        mesh.UpdateTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d((Fixed64)2, Fixed64.One, Fixed64.One));

        mesh.TryGetClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out MeshVolumeValidationResult result).Should().BeTrue();
        result.Should().Be(MeshVolumeValidationResult.Valid);
        AssertNear(properties.Volume, Fixed64.FromFraction(1, 3));
        AssertVectorNear(
            properties.CenterOfMass,
            new Vector3d(
                -Fixed64.Half,
                -Fixed64.Quarter,
                -Fixed64.Quarter));
    }

    [Fact]
    public void PreparedMeshScaleFailure_ShouldPreserveCommittedBuffersAndCaches()
    {
        PhysicsMesh mesh = CreateOffsetTriangleMesh();
        Vector3d[] vertices = GetWorldVertices(mesh);
        FixedBoundBox bounds = mesh.Bounds;
        Fixed64 area = mesh.TotalArea;
        int bvhBuildCount = mesh.TriangleBvhBuildCount;

        Action prepare = () => mesh.PrepareTransformation(
            new Vector3d((Fixed64)9, (Fixed64)8, (Fixed64)7),
            FixedQuaternion.Identity,
            new Vector3d((Fixed64)65536, (Fixed64)65536, Fixed64.One),
            Vector3d.One,
            MeshInertiaPolicy.SurfaceApproximation);

        prepare.Should().Throw<ArgumentException>();
        GetWorldVertices(mesh).Should().Equal(vertices);
        mesh.Bounds.Should().Be(bounds);
        mesh.TotalArea.Should().Be(area);
        mesh.OwnerScale.Should().Be(Vector3d.One);
        mesh.PartScale.Should().Be(Vector3d.One);
        mesh.TriangleBvhBuildCount.Should().Be(bvhBuildCount);
    }

    [Fact]
    public void DenseSupportTree_FailedPreparedScale_ShouldPreserveCommittedAndLaterRefittedResults()
    {
        CreateSubdividedClosedCubeTopology(8, out Vector3d[] vertices, out int[] triangles);
        for (int i = 0; i < vertices.Length; i++)
            vertices[i] *= (Fixed64)100;

        var mesh = new PhysicsMesh(
            vertices,
            triangles,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        Vector3d tiedDirection = Vector3d.Right;
        AssertSupportMatchesAuthoredOrderBruteForce(mesh, tiedDirection);
        Vector3d committedSupport = mesh.GetSupportVertexWorld(tiedDirection);

        Action prepare = () => mesh.PrepareTransformation(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One * (Fixed64)16,
            Vector3d.One,
            MeshInertiaPolicy.RequireClosedVolume);

        prepare.Should().Throw<ArgumentException>().WithMessage("*NonRepresentableVolume*");
        mesh.GetSupportVertexWorld(tiedDirection).Should().Be(committedSupport);
        mesh.OwnerScale.Should().Be(Vector3d.One);

        PrepareAndPublish(mesh, new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.One));
        AssertSupportMatchesAuthoredOrderBruteForce(mesh, tiedDirection);
        PrepareAndPublish(mesh, new Vector3d(Fixed64.One, Fixed64.Half, Fixed64.One));
        AssertSupportMatchesAuthoredOrderBruteForce(mesh, tiedDirection);
    }

    [Fact]
    public void ChangedMeshPreparation_AfterWarmup_ShouldAllocateZeroBytes()
    {
        PhysicsMesh mesh = CreateOffsetTriangleMesh();
        Vector3d firstScale = new((Fixed64)2, (Fixed64)3, Fixed64.One);
        Vector3d secondScale = new((Fixed64)3, (Fixed64)2, Fixed64.One);
        PrepareAndPublish(mesh, firstScale);
        PrepareAndPublish(mesh, secondScale);

        long before = GC.GetAllocatedBytesForCurrentThread();
        PrepareAndPublish(mesh, firstScale);
        PrepareAndPublish(mesh, secondScale);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        allocated.Should().Be(0);
    }

    [Fact]
    public void DenseConvexMeshScaleChange_AfterWarmup_ShouldAllocateZeroBytes()
    {
        CreateSubdividedClosedCubeTopology(8, out Vector3d[] vertices, out int[] triangles);
        var mesh = new PhysicsMesh(
            vertices,
            triangles,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        Vector3d firstScale = new((Fixed64)2, (Fixed64)3, Fixed64.One);
        Vector3d secondScale = new((Fixed64)3, (Fixed64)2, Fixed64.One);
        PrepareAndPublish(mesh, firstScale);
        PrepareAndPublish(mesh, secondScale);

        long before = GC.GetAllocatedBytesForCurrentThread();
        PrepareAndPublish(mesh, firstScale);
        PrepareAndPublish(mesh, secondScale);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        allocated.Should().Be(0);
    }

    [Fact]
    public void ClosedCube_WithNonUniformScale_ShouldUpdateGeometryAndMassProperties()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider collider = MeshTestFixtures.CreateConvexCube();
        Vector3d scale = new((Fixed64)2, (Fixed64)3, (Fixed64)4);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, scale));

        collider.InitializeWithNoBody(agent);

        AssertVectorNear(collider.Bounds.Proportions, scale);
        AssertNear(collider.Area, (Fixed64)52);
        collider.CalculateMassPropertyWeight()
            .TryGetMeasure(out Fixed64 weight)
            .Should()
            .BeTrue();
        AssertNear(weight, (Fixed64)24);
        AssertVectorNear(collider.CalculateLocalCenterOfMassOffset(), Vector3d.Zero);

        Fixed3x3 tensor = collider.CalculateInertiaTensor((Fixed64)12, Vector3d.Zero);
        AssertNear(tensor.M11, (Fixed64)25);
        AssertNear(tensor.M22, (Fixed64)20);
        AssertNear(tensor.M33, (Fixed64)13);
        AssertNear(tensor.M12, Fixed64.Zero);
        AssertNear(tensor.M13, Fixed64.Zero);
        AssertNear(tensor.M23, Fixed64.Zero);
    }

    [Fact]
    public void ClosedVolumeMassProperties_ShouldCacheEachCommittedScale()
    {
        var mesh = new PhysicsMesh(
            CreateTetrahedronVertices(),
            TetrahedronTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        Vector3d firstScale = new((Fixed64)2, (Fixed64)3, (Fixed64)4);
        mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, firstScale);

        mesh.TryGetClosedVolumeMassProperties(out MeshMassProperties first, out _).Should().BeTrue();
        mesh.TryGetClosedVolumeMassProperties(out MeshMassProperties repeated, out _).Should().BeTrue();
        mesh.ValidateClosedVolumeScaleRepresentability(firstScale);

        repeated.Should().Be(first);
        mesh.ClosedVolumeScaleEvaluationCount.Should().Be(1);

        Vector3d secondScale = new((Fixed64)3, (Fixed64)2, (Fixed64)4);
        mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, secondScale);
        mesh.ClosedVolumeScaleEvaluationCount.Should().Be(1);
        mesh.TryGetClosedVolumeMassProperties(out _, out _).Should().BeTrue();
        mesh.TryGetClosedVolumeMassProperties(out _, out _).Should().BeTrue();
        mesh.ClosedVolumeScaleEvaluationCount.Should().Be(2);
    }

    [Fact]
    public void ClosedVolumeColliderScaleValidation_ShouldPromoteCandidateIntoCommittedCache()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider collider = MeshTestFixtures.CreateConvexCube();
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4));

        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));

        collider.Mesh.ClosedVolumeScaleEvaluationCount.Should().Be(1);
        collider.Mesh.TryGetClosedVolumeMassProperties(out _, out _).Should().BeTrue();
        collider.CalculateInertiaTensor(Fixed64.One, Vector3d.Zero);
        collider.Mesh.ClosedVolumeScaleEvaluationCount.Should().Be(1);

        transform.LocalScale = new Vector3d((Fixed64)3, (Fixed64)2, (Fixed64)4);
        collider.Simulate();

        collider.Mesh.ClosedVolumeScaleEvaluationCount.Should().Be(2);
        collider.Mesh.TryGetClosedVolumeMassProperties(out _, out _).Should().BeTrue();
        collider.CalculateInertiaTensor(Fixed64.One, Vector3d.Zero);
        collider.Mesh.ClosedVolumeScaleEvaluationCount.Should().Be(2);
    }

    [Fact]
    public void ClosedVolumeColliderAtDefaultScale_ShouldPromoteValidatedCandidate()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider collider = MeshTestFixtures.CreateConvexCube();
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);

        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        collider.CalculateInertiaTensor(Fixed64.One, Vector3d.Zero);
        collider.Mesh.ClosedVolumeScaleEvaluationCount.Should().Be(1);

        transform.LocalPosition = new Vector3d((Fixed64)3, (Fixed64)4, (Fixed64)5);
        collider.Simulate();
        collider.CalculateInertiaTensor(Fixed64.One, Vector3d.Zero);

        collider.Mesh.ClosedVolumeScaleEvaluationCount.Should().Be(1);
    }

    [Fact]
    public void SlantedTriangle_WithNonUniformScale_ShouldUseScaledNormalAreaAndProjection()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new LSMeshCollider(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.One)
            },
            new[] { 0, 1, 2 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4)));

        collider.InitializeWithNoBody(agent);

        Vector3d expectedNormal = new(
            Fixed64.Zero,
            -Fixed64.FromFraction(4, 5),
            Fixed64.FromFraction(3, 5));
        AssertVectorNear(collider.Mesh.GetFaceNormalWorld(0), expectedNormal);
        AssertNear(collider.Area, (Fixed64)5);
        AssertNear(collider.GetFrontalArea(expectedNormal), (Fixed64)5);
        AssertNear(collider.GetFrontalArea(expectedNormal * (Fixed64)7), (Fixed64)5);
        AssertNear(collider.GetFrontalArea(-expectedNormal), Fixed64.Zero);
    }

    [Fact]
    public void SurfaceApproximation_ForRightTriangle_ShouldUsePhysicalThinShellTensor()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right * (Fixed64)2,
                Vector3d.Up * (Fixed64)2
            },
            new[] { 0, 1, 2 },
            Vector3d.Zero,
            FixedQuaternion.Identity);

        Fixed3x3 tensor = mesh.CalculateInertiaTensor(
            (Fixed64)12,
            MeshInertiaPolicy.SurfaceApproximation,
            new Vector3d(-Fixed64.One, -Fixed64.One, Fixed64.Zero));
        MeshSurfaceMassProperties cached = mesh.SurfaceMassProperties;
        mesh.SurfaceMassProperties.Should().Be(cached);

        AssertNear(cached.Area, (Fixed64)2);
        AssertNear(tensor.M11, (Fixed64)8);
        AssertNear(tensor.M22, (Fixed64)8);
        AssertNear(tensor.M33, (Fixed64)16);
        AssertNear(tensor.M12, (Fixed64)(-4));
        AssertNear(tensor.M21, (Fixed64)(-4));
        AssertNear(tensor.M13, Fixed64.Zero);
        AssertNear(tensor.M31, Fixed64.Zero);
        AssertNear(tensor.M23, Fixed64.Zero);
        AssertNear(tensor.M32, Fixed64.Zero);
    }

    [Fact]
    public void UpdateTransform_WithInvalidScale_ShouldRetainPriorTransformAndCaches()
    {
        PhysicsMesh mesh = CreateOffsetTriangleMesh();
        Vector3d priorPosition = new((Fixed64)3, (Fixed64)(-2), (Fixed64)5);
        Vector3d priorScale = new((Fixed64)2, (Fixed64)3, (Fixed64)4);
        mesh.UpdateTransform(priorPosition, FixedQuaternion.Identity, priorScale);
        Vector3d priorVertex = mesh.GetVertexWorld(0);
        FixedBoundBox priorBounds = mesh.Bounds;
        Fixed64 priorArea = mesh.TotalArea;

        Vector3d[] invalidScales =
        {
            new(Fixed64.Zero, Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One, Fixed64.One),
            new(Fixed64.FromRaw(1), Fixed64.One, Fixed64.One)
        };

        for (int i = 0; i < invalidScales.Length; i++)
        {
            Vector3d invalidScale = invalidScales[i];
            Action update = () => mesh.UpdateTransform(
                new Vector3d((Fixed64)99, (Fixed64)98, (Fixed64)97),
                FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)20, (Fixed64)30, (Fixed64)40),
                invalidScale);

            update.Should().Throw<ArgumentException>();
            mesh.OwnerScale.Should().Be(priorScale);
            mesh.PartScale.Should().Be(Vector3d.One);
            mesh.GetVertexWorld(0).Should().Be(priorVertex);
            mesh.Bounds.Should().Be(priorBounds);
            mesh.TotalArea.Should().Be(priorArea);
        }
    }

    [Fact]
    public void UpdateTransform_WhenScaleCollapsesSourceTriangle_ShouldRetainPriorState()
    {
        Fixed64 edge = Fixed64.FromFraction(1, 10);
        var mesh = new PhysicsMesh(
            new[] { Vector3d.Zero, Vector3d.Right * edge, Vector3d.Up * edge },
            new[] { 0, 1, 2 },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        FixedBoundBox priorBounds = mesh.Bounds;
        Vector3d priorVertex = mesh.GetVertexWorld(0);

        Action update = () => mesh.UpdateTransform(
            Vector3d.One,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.FromFraction(1, 1000), Fixed64.FromFraction(1, 1000), Fixed64.One));

        update.Should().Throw<ArgumentException>().WithMessage("*nondegenerate*");
        mesh.OwnerScale.Should().Be(Vector3d.One);
        mesh.Bounds.Should().Be(priorBounds);
        mesh.GetVertexWorld(0).Should().Be(priorVertex);
    }

    [Fact]
    public void RotationValidation_ShouldRejectNonUnitBeforeMutationAndRoundTripNormalizedPose()
    {
        var mesh = new PhysicsMesh(
            new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Up },
            new[] { 0, 1, 2 },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        FixedQuaternion invalidRotation = new(
            Fixed64.Quarter,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Half);
        Vector3d priorVertex = mesh.GetVertexWorld(0);
        FixedBoundBox priorBounds = mesh.Bounds;
        Action invalidUpdate = () => mesh.UpdateTransform(
            new Vector3d((Fixed64)3, (Fixed64)(-2), (Fixed64)5),
            invalidRotation,
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4));
        invalidUpdate.Should().Throw<ArgumentException>().WithMessage("*normalized*");
        mesh.GetVertexWorld(0).Should().Be(priorVertex);
        mesh.Bounds.Should().Be(priorBounds);

        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)15,
            (Fixed64)25,
            (Fixed64)35);
        mesh.UpdateTransform(
            new Vector3d((Fixed64)3, (Fixed64)(-2), (Fixed64)5),
            rotation,
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4));
        Vector3d localPoint = new(Fixed64.Quarter, Fixed64.Half, Fixed64.FromFraction(3, 4));

        Vector3d worldPoint = mesh.ConvertScaledLocalToWorld(localPoint);
        Vector3d roundTrip = mesh.ConvertWorldToScaledLocal(worldPoint);
        mesh.GetTriangleVertices(0, out Vector3d first, out Vector3d second, out Vector3d third);
        Vector3d normal = mesh.GetFaceNormalWorld(0);

        AssertVectorNear(roundTrip, localPoint);
        AssertNear(Vector3d.Dot(normal, second - first), Fixed64.Zero);
        AssertNear(Vector3d.Dot(normal, third - first), Fixed64.Zero);
        AssertNear(normal.Magnitude, Fixed64.One);
    }

    [Fact]
    public void WorldDirectionConversion_ShouldExposeRigidRotationOverflow()
    {
        var mesh = new PhysicsMesh(
            new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Up },
            new[] { 0, 1, 2 },
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                Fixed64.PiOver4));
        var direction = new Vector3d(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.MaxValue);

        mesh.TryConvertWorldDirectionToLocal(
                direction,
                out Vector3d localDirection)
            .Should()
            .BeFalse();
        localDirection.Should().Be(Vector3d.Zero);
        Action materialize = () =>
            mesh.ConvertWorldDirectionToLocal(direction);
        materialize.Should().Throw<InvalidOperationException>()
            .WithMessage("*cannot be represented*");
    }

    [Fact]
    public void TriangleWorldView_ShouldExposeUnmaterializableVerticesExplicitly()
    {
        var mesh = new PhysicsMesh(
            new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Up },
            new[] { 0, 1, 2 },
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);

        mesh.TryGetTriangleVertices(0, out Vector3d first, out Vector3d second, out Vector3d third)
            .Should().BeFalse();
        first.Should().Be(Vector3d.Zero);
        second.Should().Be(Vector3d.Zero);
        third.Should().Be(Vector3d.Zero);
        Action materialize = () => mesh.GetTriangleVertices(0, out _, out _, out _);
        materialize.Should().Throw<InvalidOperationException>()
            .WithMessage("*outside the Fixed64 world-coordinate domain*");
    }

    [Fact]
    public void ExactWorldViews_ShouldExposeUnrepresentableSupportAndConversionsExplicitly()
    {
        var mesh = new PhysicsMesh(
            new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Up },
            new[] { 0, 1, 2 },
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity);

        mesh.TryGetSupportVertexWorld(Vector3d.Right, out Vector3d support)
            .Should().BeFalse();
        support.Should().Be(Vector3d.Zero);
        Action getSupport = () => mesh.GetSupportVertexWorld(Vector3d.Right);
        getSupport.Should().Throw<InvalidOperationException>()
            .WithMessage("*outside the Fixed64 world-coordinate domain*");
        mesh.TryGetVertexWorld(1, out Vector3d vertex)
            .Should().BeFalse();
        vertex.Should().Be(Vector3d.Zero);
        Action getVertex = () => mesh.GetVertexWorld(1);
        getVertex.Should().Throw<InvalidOperationException>()
            .WithMessage("*outside the Fixed64 world-coordinate domain*");

        mesh.TryConvertScaledLocalToWorld(
                Vector3d.Right,
                out Vector3d worldPoint)
            .Should().BeFalse();
        worldPoint.Should().Be(Vector3d.Zero);
        Action toWorld = () => mesh.ConvertScaledLocalToWorld(Vector3d.Right);
        toWorld.Should().Throw<InvalidOperationException>()
            .WithMessage("*outside the Fixed64 world-coordinate domain*");

        mesh.TryConvertWorldToScaledLocal(
                new Vector3d(
                    Fixed64.MinValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                out Vector3d localPoint)
            .Should().BeFalse();
        localPoint.Should().Be(Vector3d.Zero);
        Action toLocal = () => mesh.ConvertWorldToScaledLocal(
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero));
        toLocal.Should().Throw<InvalidOperationException>()
            .WithMessage("*scaled-local frame*");
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ExtremeRotatedWorldBoundsAndQueries_ShouldUseExactRelativeFrames(
        bool positive)
    {
        Fixed64 edge = positive
            ? Fixed64.MaxValue - (Fixed64)4
            : Fixed64.MinValue + (Fixed64)4;
        var mesh = new PhysicsMesh(
            new[]
            {
                new Vector3d(-Fixed64.One, -Fixed64.One, Fixed64.Zero),
                new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.Zero),
                new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero),
            },
            new[] { 0, 1, 2 },
            new Vector3d(edge, edge, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                Fixed64.Zero,
                (Fixed64)45));

        var candidates = new SwiftList<int>();
        for (int i = 0; i < mesh.VertexCount; i++)
        {
            mesh.TryGetVertexWorld(i, out Vector3d vertex).Should().BeTrue();
            mesh.Bounds.Contains(vertex).Should().BeTrue();
            mesh.GetTrianglesInWorldBounds(
                new FixedBoundVolume(vertex, vertex),
                candidates);
            candidates.Should().Contain(0);
        }
    }

    [Fact]
    public void FullSpanAuthoredMesh_ShouldDownscaleWithoutIntermediateSubtractionLoss()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
                new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
                new Vector3d(Fixed64.MaxValue, Fixed64.One, Fixed64.Zero),
            },
            new[] { 0, 1, 2 },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        FixedBoundBox priorBounds = mesh.Bounds;

        mesh.UpdateTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Quarter, Fixed64.One, Fixed64.One));

        mesh.ScaledLocalBounds.Min.X.Should().Be(Fixed64.MinValue / (Fixed64)4);
        mesh.ScaledLocalBounds.Max.X.Should().Be(Fixed64.MaxValue / (Fixed64)4);
        mesh.Bounds.Should().NotBe(priorBounds);
    }

    [Fact]
    public void ScaleChange_ShouldRebuildCenteredTriangleBvhAndQueryScaledWorldBounds()
    {
        PhysicsMesh mesh = CreateOffsetTriangleMesh();
        var hits = new SwiftList<int>();
        mesh.GetTrianglesInWorldBounds(
            new FixedBoundVolume(
                new Vector3d(Fixed64.Half, -Fixed64.Half, Fixed64.Zero),
                new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Half, Fixed64.Zero)),
            hits);
        int buildCount = mesh.TriangleBvhBuildCount;

        mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4));
        mesh.GetTrianglesInWorldBounds(
            new FixedBoundVolume(
                new Vector3d(Fixed64.One, Fixed64.FromFraction(-3, 2), Fixed64.Zero),
                new Vector3d((Fixed64)3, Fixed64.FromFraction(3, 2), Fixed64.Zero)),
            hits);

        hits.Should().Contain(1);
        mesh.TriangleBvhBuildCount.Should().Be(buildCount + 1);
    }

    [Fact]
    public void ScaledSupportTree_ShouldMatchAuthoredOrderBruteForce()
    {
        PhysicsMesh mesh = MeshTestFixtures.CreateConvexPolygonFan(40, (Fixed64)4).Mesh;
        mesh.UpdateTransform(
            new Vector3d((Fixed64)4, (Fixed64)(-3), (Fixed64)2),
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)15, (Fixed64)25, (Fixed64)35),
            new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4));
        Vector3d direction = new((Fixed64)7, (Fixed64)(-3), (Fixed64)5);

        AssertSupportMatchesAuthoredOrderBruteForce(mesh, direction);
    }

    [Fact]
    public void SurfaceApproximation_ShouldBeTriangulationAndTranslationStable()
    {
        Vector3d[] centeredVertices =
        {
            new(-Fixed64.One, -Fixed64.One, Fixed64.Zero),
            new(Fixed64.One, -Fixed64.One, Fixed64.Zero),
            new(Fixed64.One, Fixed64.One, Fixed64.Zero),
            new(-Fixed64.One, Fixed64.One, Fixed64.Zero)
        };
        Vector3d translation = new((Fixed64)1000000000, (Fixed64)(-900000000), (Fixed64)100000000);
        var translatedVertices = new Vector3d[centeredVertices.Length];
        for (int i = 0; i < centeredVertices.Length; i++)
            translatedVertices[i] = centeredVertices[i] + translation;

        var first = new PhysicsMesh(centeredVertices, new[] { 0, 1, 2, 0, 2, 3 }, Vector3d.Zero, FixedQuaternion.Identity);
        var second = new PhysicsMesh(translatedVertices, new[] { 0, 1, 3, 1, 2, 3 }, Vector3d.Zero, FixedQuaternion.Identity);
        Fixed3x3 firstTensor = first.CalculateInertiaTensor((Fixed64)12, MeshInertiaPolicy.SurfaceApproximation, Vector3d.Zero);
        Fixed3x3 secondTensor = second.CalculateInertiaTensor(
            (Fixed64)12,
            MeshInertiaPolicy.SurfaceApproximation,
            Vector3d.Zero);

        AssertMatrixNear(firstTensor, secondTensor);
        AssertNear(firstTensor.M11, (Fixed64)4);
        AssertNear(firstTensor.M22, (Fixed64)4);
        AssertNear(firstTensor.M33, (Fixed64)8);
    }

    [Fact]
    public void SurfaceApproximation_WithNonUniformScale_ShouldUseScaledThinShellTensor()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                new Vector3d(-Fixed64.One, -Fixed64.One, Fixed64.Zero),
                new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.Zero),
                new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero),
                new Vector3d(-Fixed64.One, Fixed64.One, Fixed64.Zero)
            },
            new[] { 0, 1, 2, 0, 2, 3 },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4));

        Fixed3x3 tensor = mesh.CalculateInertiaTensor((Fixed64)12, MeshInertiaPolicy.SurfaceApproximation, Vector3d.Zero);

        AssertNear(tensor.M11, (Fixed64)36);
        AssertNear(tensor.M22, (Fixed64)16);
        AssertNear(tensor.M33, (Fixed64)52);
    }

    [Fact]
    public void TranslatedClosedVolume_ShouldFallBackToCenteredScalingWhenAbsoluteCenterOverflows()
    {
        Vector3d translation = Vector3d.Up * (Fixed64.MaxValue - (Fixed64)4);
        Vector3d[] vertices = CreateTetrahedronVertices();
        for (int i = 0; i < vertices.Length; i++)
            vertices[i] += translation;
        var mesh = new PhysicsMesh(
            vertices,
            TetrahedronTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        mesh.UpdateTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.One, Fixed64.Two, Fixed64.One));

        mesh.TryGetClosedVolumeMassProperties(
                out MeshMassProperties properties,
                out MeshVolumeValidationResult result)
            .Should().BeTrue();
        result.Should().Be(MeshVolumeValidationResult.Valid);
        properties.CenterOfMass.Y.Abs().Should().BeLessThan((Fixed64)4);
    }

    [Fact]
    public void CompoundClosedVolume_ShouldUsePreparedVerticesWhenCombinedScaleIsUnrepresentable()
    {
        Fixed64 narrow = Fixed64.FromFraction(1, 65536);
        var mesh = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                new Vector3d(narrow, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Up,
                Vector3d.Forward
            },
            TetrahedronTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        Vector3d ownerScale = new((Fixed64)50000, Fixed64.One, Fixed64.One);
        Vector3d partScale = ownerScale;

        mesh.PrepareTransformation(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            ownerScale,
            partScale,
            MeshInertiaPolicy.RequireClosedVolume);
        mesh.PublishPreparedTransformation();

        mesh.TryGetClosedVolumeMassProperties(
                out MeshMassProperties properties,
                out MeshVolumeValidationResult result)
            .Should().BeTrue();
        result.Should().Be(MeshVolumeValidationResult.Valid);
        properties.Volume.Should().BeGreaterThan(Fixed64.Epsilon);
    }

    [Fact]
    public void StandaloneRequireClosed_WithCollapsedScaledVolume_ShouldRejectBeforeRegistration()
    {
        Vector3d[] vertices = CreateTetrahedronVertices();
        int[] triangles = TetrahedronTriangles();
        Vector3d scale = Vector3d.One * Fixed64.FromFraction(1, 200);
        var directMesh = new PhysicsMesh(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity);
        directMesh.TryGetClosedVolumeMassProperties(out _, out MeshVolumeValidationResult rawResult).Should().BeTrue();
        rawResult.Should().Be(MeshVolumeValidationResult.Valid);
        directMesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, scale);
        directMesh.TryGetClosedVolumeMassProperties(out _, out MeshVolumeValidationResult scaledResult).Should().BeFalse();
        scaledResult.Should().Be(MeshVolumeValidationResult.ZeroVolume);
        Action validateCachedScale = () => directMesh.ValidateClosedVolumeScaleRepresentability(scale);
        validateCachedScale.Should().Throw<ArgumentException>().WithMessage("*ZeroVolume*");

        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new LSMeshCollider(
            vertices,
            triangles,
            MeshColliderMode.Convex,
            MeshInertiaPolicy.RequireClosedVolume);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, scale));

        Action initialize = () => collider.InitializeWithNoBody(agent);

        initialize.Should().Throw<ArgumentException>().WithMessage("*ZeroVolume*");
        collider.Id.Should().Be(-1);
        collider.HasHostBinding.Should().BeFalse();
        context.Physics.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void BodyInitialize_WithCollapsedScaledMeshVolume_ShouldRejectBeforeRuntimeMutation()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector3d scale = Vector3d.One * Fixed64.FromFraction(1, 200);
        var collider = new LSMeshCollider(
            CreateTetrahedronVertices(),
            TetrahedronTriangles(),
            MeshColliderMode.Convex,
            MeshInertiaPolicy.RequireClosedVolume);
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, scale);
        var body = new SolidBody(new TestMatterAgent(context, transform), collider);

        Action initialize = () => body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        initialize.Should().Throw<ArgumentException>().WithMessage("*ZeroVolume*");
        body.Active.Should().BeFalse();
        body.DynamicId.Should().Be(-1);
        collider.Id.Should().Be(-1);
        collider.Body.Should().BeNull();
        collider.HasHostBinding.Should().BeFalse();
        context.Physics.BodyCount.Should().Be(0);
        context.Physics.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void RequireClosed_WithSaturatedScaledVolume_ShouldReportAndRejectNonRepresentableVolume()
    {
        CreateSubdividedClosedCubeTopology(8, out Vector3d[] vertices, out int[] triangles);
        for (int i = 0; i < vertices.Length; i++)
            vertices[i] *= (Fixed64)100;

        Vector3d scale = Vector3d.One * (Fixed64)16;
        var directMesh = new PhysicsMesh(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity);
        directMesh.TryGetClosedVolumeMassProperties(out _, out _).Should().BeTrue();
        directMesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, scale);
        directMesh.TryGetClosedVolumeMassProperties(out _, out MeshVolumeValidationResult result).Should().BeFalse();
        result.Should().Be(MeshVolumeValidationResult.NonRepresentableVolume);

        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new LSMeshCollider(
            vertices,
            triangles,
            MeshColliderMode.Convex,
            MeshInertiaPolicy.RequireClosedVolume);
        Action initialize = () => collider.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, scale)));

        initialize.Should().Throw<ArgumentException>().WithMessage("*NonRepresentableVolume*");
        collider.Id.Should().Be(-1);
        collider.HasHostBinding.Should().BeFalse();
        context.Physics.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void RequireClosed_WithFiniteVolumeAndSaturatedTensor_ShouldReportNonRepresentableMassProperties()
    {
        CreateSubdividedClosedCubeTopology(32, out Vector3d[] vertices, out int[] triangles);
        for (int i = 0; i < vertices.Length; i++)
            vertices[i] *= (Fixed64)100;

        Vector3d scale = new((Fixed64)2000, Fixed64.One, Fixed64.One);
        var directMesh = new PhysicsMesh(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity);
        directMesh.TryGetClosedVolumeMassProperties(out _, out _).Should().BeTrue();
        directMesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, scale);

        directMesh.TryGetClosedVolumeMassProperties(out _, out MeshVolumeValidationResult result).Should().BeFalse();
        result.Should().Be(MeshVolumeValidationResult.NonRepresentableMassProperties);

        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new LSMeshCollider(vertices, triangles, MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume);
        Action initialize = () => collider.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, scale)));

        initialize.Should().Throw<ArgumentException>().WithMessage("*NonRepresentableMassProperties*");
        collider.Id.Should().Be(-1);
        collider.HasHostBinding.Should().BeFalse();
        context.Physics.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void SurfaceApproximation_WithLongThinTriangle_ShouldRetainRepresentableFinalShell()
    {
        Vector3d[] vertices =
        {
            new((Fixed64)(-50000), Fixed64.Zero, Fixed64.Zero),
            new((Fixed64)50000, Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.Zero, Fixed64.FromFraction(1, 10), Fixed64.Zero)
        };
        int[] triangles = { 0, 1, 2 };
        var directMesh = new PhysicsMesh(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity);
        MeshSurfaceMassProperties properties =
            directMesh.SurfaceMassProperties;

        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var collider = new LSMeshCollider(vertices, triangles, MeshColliderMode.Convex, MeshInertiaPolicy.SurfaceApproximation);
        collider.InitializeWithNoBody(new TestMatterAgent(context));

        properties.UnitMassInertiaTensor.M22
            .Should()
            .BeGreaterThan(Fixed64.Zero);
        properties.UnitMassInertiaTensor.M33
            .Should()
            .BeGreaterThan(properties.UnitMassInertiaTensor.M22);
        collider.Id.Should().BeGreaterThanOrEqualTo(0);
        collider.HasHostBinding.Should().BeTrue();
        context.Physics.ColliderCount.Should().Be(1);
    }

    [Fact]
    public void StandaloneSurfaceMesh_ShouldRejectAnUnrepresentableAuthoredCenterOfMass()
    {
        using GravitasWorldContext context =
            GravitasWorldContext.CreateOwned();
        var collider = new LSMeshCollider(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.One
            },
            new[] { 0, 1, 2 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation)
        {
            LocalOffset = new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero)
        };
        var transform = new FixedTransform(
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);

        Action initialize = () =>
            collider.InitializeWithNoBody(
                new TestMatterAgent(context, transform));

        initialize.Should().Throw<InvalidOperationException>()
            .WithMessage("*Prepared collider mass-property point*");
        collider.Id.Should().Be(-1);
        collider.HasHostBinding.Should().BeFalse();
    }

    [Fact]
    public void CompoundScaleFailure_ShouldPrevalidateAllMeshPartsBeforeAnyPartRebuild()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider cube = MeshTestFixtures.CreateConvexCube();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, -Vector3d.Right),
            CompoundColliderPart.ConvexMesh(
                cube.Mesh.LocalVertices.ToArray(),
                cube.Mesh.Triangles.ToArray(),
                Vector3d.Right,
                MeshInertiaPolicy.RequireClosedVolume));
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        compound.InitializeWithNoBody(new TestMatterAgent(context, transform));
        LSCollider spherePart = compound.GetPartCollider(0);
        var meshPart = (LSMeshCollider)compound.GetPartCollider(1);
        uint sphereVersion = spherePart.RuntimeShapeVersion;
        uint compoundVersion = compound.RuntimeShapeVersion;
        FixedBoundBox sphereBounds = spherePart.Bounds;
        FixedBoundBox compoundBounds = compound.Bounds;

        transform.LocalScale = new Vector3d((Fixed64)65536, (Fixed64)65536, Fixed64.Half);
        Action simulate = compound.Simulate;

        simulate.Should().Throw<ArgumentException>();
        spherePart.RuntimeShapeVersion.Should().Be(sphereVersion);
        spherePart.Bounds.Should().Be(sphereBounds);
        meshPart.Mesh.OwnerScale.Should().Be(Vector3d.One);
        meshPart.Mesh.PartScale.Should().Be(Vector3d.One);
        compound.Bounds.Should().Be(compoundBounds);
        context.Physics.ColliderCount.Should().Be(1);

        transform.LocalScale = new Vector3d((Fixed64)2, Fixed64.One, Fixed64.One);
        compound.Simulate();

        spherePart.RuntimeShapeVersion.Should().BeGreaterThan(sphereVersion);
        compound.RuntimeShapeVersion.Should().BeGreaterThan(compoundVersion);
        meshPart.Mesh.OwnerScale.Should().Be(transform.LossyScale);
        meshPart.Mesh.PartScale.Should().Be(Vector3d.One);
        compound.Bounds.Should().NotBe(compoundBounds);

    }

    [Fact]
    public void RuntimeMeshScaleFailure_ShouldPreserveCommittedShapeAndRecoverOnNextValidScale()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider collider = MeshTestFixtures.CreateConvexCube();
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        FixedBoundBox colliderBounds = collider.Bounds;
        FixedBoundBox meshBounds = collider.Mesh.Bounds;
        Vector3d meshScale = collider.Mesh.OwnerScale;
        Vector3d firstVertex = collider.Mesh.GetVertexWorld(0);
        Fixed64 area = collider.Area;
        uint version = collider.RuntimeShapeVersion;
        bool wasPartitioned = collider.IsPartitioned;
        int partitionCoordinateCount = collider.PartitionCoordinates?.Count ?? 0;

        transform.LocalScale = new Vector3d((Fixed64)65536, (Fixed64)65536, Fixed64.Half);
        Action rebuild = collider.Simulate;

        rebuild.Should().Throw<ArgumentException>();
        collider.Bounds.Should().Be(colliderBounds);
        collider.Mesh.Bounds.Should().Be(meshBounds);
        collider.Mesh.OwnerScale.Should().Be(meshScale);
        collider.Mesh.GetVertexWorld(0).Should().Be(firstVertex);
        collider.Area.Should().Be(area);
        collider.RuntimeShapeVersion.Should().Be(version);
        collider.IsPartitioned.Should().Be(wasPartitioned);
        (collider.PartitionCoordinates?.Count ?? 0).Should().Be(partitionCoordinateCount);

        transform.LocalScale = new Vector3d((Fixed64)2, Fixed64.One, Fixed64.One);
        collider.Simulate();

        collider.RuntimeShapeVersion.Should().Be(version + 1);
        collider.Mesh.OwnerScale.Should().Be(transform.LossyScale);
        collider.Bounds.Should().NotBe(colliderBounds);
    }

    [Fact]
    public void CompoundWithInvalidMeshScale_ShouldRejectBeforeOwnerRegistration()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider cube = MeshTestFixtures.CreateConvexCube();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Zero),
            CompoundColliderPart.ConvexMesh(
                cube.Mesh.LocalVertices.ToArray(),
                cube.Mesh.Triangles.ToArray(),
                Vector3d.Right,
                MeshInertiaPolicy.RequireClosedVolume));
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero));

        Action initialize = () => compound.InitializeWithNoBody(agent);

        initialize.Should().Throw<ArgumentException>();
        compound.Id.Should().Be(-1);
        compound.HasHostBinding.Should().BeFalse();
        context.Physics.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void CompoundWithNonNormalizedMeshPartRotation_ShouldNormalizeBeforeOwnerRegistration()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        FixedQuaternion authoredRotation = new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero, Fixed64.Half);
        LSMeshCollider cube = MeshTestFixtures.CreateConvexCube();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.ConvexMesh(
                cube.Mesh.LocalVertices.ToArray(),
                cube.Mesh.Triangles.ToArray(),
                Vector3d.Zero,
                authoredRotation,
                Vector3d.One,
                MeshInertiaPolicy.RequireClosedVolume));

        compound.InitializeWithNoBody(new TestMatterAgent(context));

        compound.Parts[0].LocalRotation.Should().Be(authoredRotation.Normalized);
        compound.Id.Should().BeGreaterThanOrEqualTo(0);
        compound.HasHostBinding.Should().BeTrue();
        context.Physics.ColliderCount.Should().Be(1);
    }

    [Fact]
    public void ScaleAdmission_ShouldUseFinalCanonicalGeometryInsteadOfIntermediateAggregates()
    {
        PhysicsMesh mesh = CreateOffsetTriangleMesh();
        Action determinantOverflow = () => mesh.UpdateTransform(
            Vector3d.One,
            FixedQuaternion.Identity,
            new Vector3d((Fixed64)65536, (Fixed64)65536, Fixed64.Half));
        determinantOverflow.Should().NotThrow();
        mesh.OwnerScale.Should().Be(
            new Vector3d((Fixed64)65536, (Fixed64)65536, Fixed64.Half));

        var sameSignBounds = new PhysicsMesh(
            new[]
            {
                new Vector3d((Fixed64)1200000000, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)1200000001, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)1200000000, Fixed64.One, Fixed64.Zero)
            },
            new[] { 0, 1, 2 },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        sameSignBounds.LocalBounds.Center.X.Should().Be((Fixed64)1200000000 + Fixed64.Half);

        var wideBounds = new PhysicsMesh(
            new[]
            {
                new Vector3d((Fixed64)(-1000000000), Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)1000000000, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)(-1000000000), Fixed64.MinIncrement, Fixed64.Zero)
            },
            new[] { 0, 1, 2 },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        Action boundsOverflow = () => wideBounds.UpdateTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.FromFraction(11, 10), Fixed64.One, Fixed64.One));
        boundsOverflow.Should().NotThrow();
        wideBounds.OwnerScale.X.Should().Be(Fixed64.FromFraction(11, 10));

        PhysicsMesh? crossProductMesh = null;
        Action crossProductOverflow = () => crossProductMesh = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                new Vector3d(Fixed64.Zero, (Fixed64)100000, Fixed64.Zero),
                new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)100000)
            },
            new[] { 0, 1, 2 },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        crossProductOverflow.Should().NotThrow();
        crossProductMesh!.TotalArea.Should().Be(Fixed64.MaxValue);

        PhysicsMesh? crossDifferenceMesh = null;
        Action crossDifferenceOverflow = () => crossDifferenceMesh = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                new Vector3d(Fixed64.Zero, (Fixed64)50000, (Fixed64)50000),
                new Vector3d(Fixed64.Zero, (Fixed64)30000, (Fixed64)(-30000))
            },
            new[] { 0, 1, 2 },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        crossDifferenceOverflow.Should().NotThrow();
        crossDifferenceMesh!.TotalArea.Should().Be((Fixed64)1500000000);

        PhysicsMesh? largeTriangleMesh = null;
        Action magnitudeOverflow = () => largeTriangleMesh = new PhysicsMesh(
            new[] { Vector3d.Zero, Vector3d.Right * (Fixed64)50000, Vector3d.Up },
            new[] { 0, 1, 2 },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        magnitudeOverflow.Should().NotThrow();
        largeTriangleMesh!.TotalArea.Should().Be((Fixed64)25000);
    }

    [Fact]
    public void CenteredScale_ShouldIgnoreAuthoredTranslationUnderflowAndSaturateAggregateArea()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                new Vector3d(Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right,
                Vector3d.Up
            },
            new[] { 0, 1, 2 },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);
        Action centeredScale = () => mesh.UpdateTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One * Fixed64.Half);
        centeredScale.Should().NotThrow();
        mesh.OwnerScale.Should().Be(Vector3d.One * Fixed64.Half);

        const int triangleCount = 100000;
        var triangles = new int[triangleCount * 3];
        for (int i = 0; i < triangles.Length; i += 3)
        {
            triangles[i] = 0;
            triangles[i + 1] = 1;
            triangles[i + 2] = 2;
        }

        PhysicsMesh? repeatedTriangleMesh = null;
        Action totalAreaOverflow = () => repeatedTriangleMesh = new PhysicsMesh(
            new[] { Vector3d.Zero, Vector3d.Right * (Fixed64)45000, Vector3d.Up },
            triangles,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);
        totalAreaOverflow.Should().NotThrow();
        repeatedTriangleMesh!.TotalArea.Should().Be(Fixed64.MaxValue);
        repeatedTriangleMesh.TotalArea
            .Should()
            .Be(repeatedTriangleMesh.SurfaceMassProperties.Area);
        repeatedTriangleMesh.SurfaceMassWeight
            .TryGetMeasure(out _)
            .Should().BeFalse();
        repeatedTriangleMesh.SurfaceMassProperties.CenterOfMass
            .Should().Be(new Vector3d(
                (Fixed64)(-7500),
                -Fixed64.FromFraction(1, 6),
                Fixed64.Zero));
    }

    [Fact]
    public void CheckedClosedMassScaling_ShouldDistinguishIntermediateMomentFailures()
    {
        var covarianceRecoveryOverflow = new MeshMassProperties(
            Fixed64.One,
            Vector3d.Zero,
            Vector3d.Zero,
            new Fixed3x3(
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, (Fixed64)1500000000, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, (Fixed64)1500000000));
        covarianceRecoveryOverflow.TryScale(Vector3d.One, out _)
            .Should().Be(MeshMassScaleResult.NonRepresentableMassProperties);

        var inertiaReconstructionOverflow = new MeshMassProperties(
            Fixed64.One,
            Vector3d.Zero,
            Vector3d.Zero,
            new Fixed3x3(
                (Fixed64)300000000, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, (Fixed64)300000000, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, (Fixed64)600000000));
        inertiaReconstructionOverflow.TryScale(new Vector3d((Fixed64)2, (Fixed64)2, Fixed64.One), out _)
            .Should().Be(MeshMassScaleResult.NonRepresentableMassProperties);

        var referenceShiftOverflow = new MeshMassProperties(
            Fixed64.One,
            Vector3d.Zero,
            new Vector3d((Fixed64)50000, Fixed64.Zero, Fixed64.Zero),
            Fixed3x3.Identity);
        referenceShiftOverflow.TryScale(Vector3d.One, out _)
            .Should().Be(MeshMassScaleResult.NonRepresentableMassProperties);
    }

    [Fact]
    public void SurfaceApproximation_WithLargeAsymmetricTriangle_ShouldRejectFirstMomentOverflow()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right * (Fixed64)2000000000,
                Vector3d.Up * Fixed64.FromFraction(1, 50000)
            },
            new[] { 0, 1, 2 },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);

        Action read = () => _ = mesh.SurfaceMassProperties;

        read.Should().Throw<InvalidOperationException>().WithMessage("*not representable*");
    }

    [Fact]
    public void SurfaceApproximation_WithSaturatedAreaNearScalarFace_ShouldPreserveSemanticCenter()
    {
        Fixed64 edge = Fixed64.MaxValue;
        var mesh = new PhysicsMesh(
            new[]
            {
                new Vector3d(edge - Fixed64.One, Fixed64.Zero, Fixed64.Zero),
                new Vector3d(edge, (Fixed64)(-25000), (Fixed64)(-25000)),
                new Vector3d(edge, (Fixed64)25000, (Fixed64)(-25000)),
                new Vector3d(edge, (Fixed64)(-25000), (Fixed64)25000)
            },
            new[]
            {
                0, 1, 2,
                1, 2, 3,
                1, 2, 3
            },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);

        // Exact round-to-even centroid of the three canonical shell faces.
        var expectedCenter = new Vector3d(
            Fixed64.FromRaw(1_861_152_495L),
            Fixed64.FromRaw(-28_633_115_302_085L),
            Fixed64.FromRaw(-42_949_672_964_581L));
        mesh.SurfaceMassWeight.TryGetMeasure(out _)
            .Should().BeFalse();
        mesh.SurfaceMassProperties.CenterOfMass
            .Should().Be(expectedCenter);
    }

    [Fact]
    public void SurfaceApproximation_WithDistantBalancedTriangles_ShouldRejectParallelAxisOverflow()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                new Vector3d((Fixed64)(-50001), Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)(-50000), Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)(-50001), Fixed64.One, Fixed64.Zero),
                new Vector3d((Fixed64)50000, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)50001, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)50000, Fixed64.One, Fixed64.Zero)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);

        Action read = () => _ = mesh.SurfaceMassProperties;

        read.Should().Throw<InvalidOperationException>().WithMessage("*not representable*");
    }

    [Fact]
    public void CheckedMeshMath_ShouldRejectOverflowAndLeaveDefaultOutputs()
    {
        MeshCheckedMath.TryCreateParallelAxisTensor(
            new Vector3d((Fixed64)50000, Fixed64.Zero, Fixed64.Zero),
            out Fixed3x3 parallelAxisTensor).Should().BeFalse();
        parallelAxisTensor.Should().Be(default(Fixed3x3));

        MeshCheckedMath.TryMultiply(
            new Vector3d((Fixed64)50000, Fixed64.One, Fixed64.One),
            new Vector3d((Fixed64)50000, Fixed64.One, Fixed64.One),
            out Vector3d vectorProduct).Should().BeFalse();
        vectorProduct.Should().Be(Vector3d.Zero);

        MeshCheckedMath.TryAdd(
            new Vector3d((Fixed64)1500000000, Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)1000000000, Fixed64.Zero, Fixed64.Zero),
            out Vector3d vectorSum).Should().BeFalse();
        vectorSum.Should().Be(Vector3d.Zero);

        MeshCheckedMath.TryAdd(
            Fixed3x3.Identity * (Fixed64)1500000000,
            Fixed3x3.Identity * (Fixed64)1000000000,
            out Fixed3x3 matrixSum).Should().BeFalse();
        matrixSum.Should().Be(default(Fixed3x3));
    }

    [Fact]
    public void CheckedMeshMath_ShouldAdmitExactFixed64DomainEndpoints()
    {
        Vector3d endpoint = new(
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.Zero);

        MeshCheckedMath.TrySubtract(
                endpoint,
                Vector3d.Zero,
                out Vector3d difference)
            .Should()
            .BeTrue();
        difference.Should().Be(endpoint);

    }

    private static PhysicsMesh CreateOffsetTriangleMesh() =>
        new(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)2, Fixed64.One, Fixed64.Zero)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);

    private static Vector3d[] CreateTetrahedronVertices() =>
        new[]
        {
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Up,
            Vector3d.Forward
        };

    private static int[] TetrahedronTriangles() =>
        new[]
        {
            1, 2, 3,
            0, 2, 1,
            0, 1, 3,
            0, 3, 2
        };

    private static void CreateSubdividedClosedCubeTopology(
        int subdivision,
        out Vector3d[] vertices,
        out int[] triangles)
    {
        var lookup = new Dictionary<long, int>();
        var vertexList = new List<Vector3d>();
        var triangleList = new List<int>();
        for (int x = 0; x < subdivision; x++)
        {
            for (int y = 0; y < subdivision; y++)
            {
                AddCubeQuad(lookup, vertexList, triangleList, 2, x, y, 0, subdivision, false);
                AddCubeQuad(lookup, vertexList, triangleList, 2, x, y, subdivision, subdivision, true);
            }
        }

        for (int y = 0; y < subdivision; y++)
        {
            for (int z = 0; z < subdivision; z++)
            {
                AddCubeQuad(lookup, vertexList, triangleList, 0, y, z, 0, subdivision, false);
                AddCubeQuad(lookup, vertexList, triangleList, 0, y, z, subdivision, subdivision, true);
            }
        }

        for (int x = 0; x < subdivision; x++)
        {
            for (int z = 0; z < subdivision; z++)
            {
                AddCubeQuad(lookup, vertexList, triangleList, 1, x, z, 0, subdivision, false);
                AddCubeQuad(lookup, vertexList, triangleList, 1, x, z, subdivision, subdivision, true);
            }
        }

        vertices = vertexList.ToArray();
        triangles = triangleList.ToArray();
    }

    private static void AddCubeQuad(
        Dictionary<long, int> lookup,
        List<Vector3d> vertices,
        List<int> triangles,
        int fixedAxis,
        int first,
        int second,
        int fixedCoordinate,
        int subdivision,
        bool positive)
    {
        int a = GetCubeVertex(lookup, vertices, fixedAxis, first, second, fixedCoordinate, subdivision);
        int b = GetCubeVertex(lookup, vertices, fixedAxis, first + 1, second, fixedCoordinate, subdivision);
        int c = GetCubeVertex(lookup, vertices, fixedAxis, first, second + 1, fixedCoordinate, subdivision);
        int d = GetCubeVertex(lookup, vertices, fixedAxis, first + 1, second + 1, fixedCoordinate, subdivision);
        bool reverse = fixedAxis == 1 ? positive : !positive;
        if (reverse)
        {
            triangles.Add(a); triangles.Add(c); triangles.Add(b);
            triangles.Add(b); triangles.Add(c); triangles.Add(d);
        }
        else
        {
            triangles.Add(a); triangles.Add(b); triangles.Add(c);
            triangles.Add(b); triangles.Add(d); triangles.Add(c);
        }
    }

    private static int GetCubeVertex(
        Dictionary<long, int> lookup,
        List<Vector3d> vertices,
        int fixedAxis,
        int first,
        int second,
        int fixedCoordinate,
        int subdivision)
    {
        int x = fixedAxis == 0 ? fixedCoordinate : first;
        int y = fixedAxis == 1 ? fixedCoordinate : fixedAxis == 0 ? first : second;
        int z = fixedAxis == 2 ? fixedCoordinate : second;
        long key = ((long)x << 42) | ((long)y << 21) | (uint)z;
        if (lookup.TryGetValue(key, out int index))
            return index;

        index = vertices.Count;
        vertices.Add(new Vector3d(
            Fixed64.FromFraction(x, subdivision) - Fixed64.Half,
            Fixed64.FromFraction(y, subdivision) - Fixed64.Half,
            Fixed64.FromFraction(z, subdivision) - Fixed64.Half));
        lookup.Add(key, index);
        return index;
    }

    private static void AssertMatrixNear(Fixed3x3 actual, Fixed3x3 expected)
    {
        AssertNear(actual.M11, expected.M11);
        AssertNear(actual.M12, expected.M12);
        AssertNear(actual.M13, expected.M13);
        AssertNear(actual.M21, expected.M21);
        AssertNear(actual.M22, expected.M22);
        AssertNear(actual.M23, expected.M23);
        AssertNear(actual.M31, expected.M31);
        AssertNear(actual.M32, expected.M32);
        AssertNear(actual.M33, expected.M33);
    }

    private static void AssertNear(Fixed64 actual, Fixed64 expected) =>
        (actual - expected).Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon * (Fixed64)128);

    private static void AssertVectorNear(Vector3d actual, Vector3d expected)
    {
        AssertNear(actual.X, expected.X);
        AssertNear(actual.Y, expected.Y);
        AssertNear(actual.Z, expected.Z);
    }

    private static void PrepareAndPublish(PhysicsMesh mesh, Vector3d scale)
    {
        mesh.PrepareTransformation(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            scale,
            Vector3d.One,
            null);
        mesh.PublishPreparedTransformation();
    }

    private static void AssertSupportMatchesAuthoredOrderBruteForce(
        PhysicsMesh mesh,
        Vector3d direction)
    {
        Vector3d expected = mesh.GetVertexWorld(0);
        for (int i = 1; i < mesh.VertexCount; i++)
        {
            Vector3d candidate = mesh.GetVertexWorld(i);
            if (Vector3d.CompareProjection(candidate, expected, direction) <= 0)
                continue;

            expected = candidate;
        }

        mesh.GetSupportVertexWorld(direction).Should().Be(expected);
    }

    private static Vector3d[] GetWorldVertices(PhysicsMesh mesh)
    {
        var vertices = new Vector3d[mesh.VertexCount];
        for (int index = 0; index < vertices.Length; index++)
            vertices[index] = mesh.GetVertexWorld(index);
        return vertices;
    }
}
