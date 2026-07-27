using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class PhysicsMeshTests
{
    [Fact]
    public void Constructor_ShouldRejectInvalidMeshInput()
    {
        Action nullVertices = () => _ = new PhysicsMesh(null!, ValidTriangles(), Vector3d.Zero, FixedQuaternion.Identity);
        Action nullTriangles = () => _ = new PhysicsMesh(ValidVertices(), null!, Vector3d.Zero, FixedQuaternion.Identity);
        Action emptyTriangleCount = () => _ = new PhysicsMesh(ValidVertices(), Array.Empty<int>(), Vector3d.Zero, FixedQuaternion.Identity);
        Action invalidTriangleCount = () => _ = new PhysicsMesh(ValidVertices(), new[] { 0, 1 }, Vector3d.Zero, FixedQuaternion.Identity);
        Action outOfRangeTriangle = () => _ = new PhysicsMesh(ValidVertices(), new[] { 0, 1, 3 }, Vector3d.Zero, FixedQuaternion.Identity);
        Action duplicateTriangleIndex = () => _ = new PhysicsMesh(ValidVertices(), new[] { 0, 1, 1 }, Vector3d.Zero, FixedQuaternion.Identity);
        Action degenerateTriangle = () => _ = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Right * 2
            },
            ValidTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        nullVertices.Should().Throw<ArgumentNullException>();
        nullTriangles.Should().Throw<ArgumentNullException>();
        emptyTriangleCount.Should().Throw<ArgumentException>();
        invalidTriangleCount.Should().Throw<ArgumentException>();
        outOfRangeTriangle.Should().Throw<ArgumentOutOfRangeException>();
        duplicateTriangleIndex.Should().Throw<ArgumentException>();
        degenerateTriangle.Should().Throw<ArgumentException>();
    }

    [Fact]
    public void Constructor_ShouldRejectMeshesBeyondDeterministicLimits()
    {
        Vector3d[] tooManyVertices = new Vector3d[PhysicsMesh.MaxVertexCount + 1];
        int[] triangles = ValidTriangles();

        Action create = () => _ = new PhysicsMesh(
            tooManyVertices,
            triangles,
            Vector3d.Zero,
            FixedQuaternion.Identity);

        create.Should().Throw<ArgumentOutOfRangeException>();
    }

    [Fact]
    public void Constructor_ShouldAdmitFullDomainTriangleArea()
    {
        Vector3d[] vertices =
        {
            new(Fixed64.MinValue, Fixed64.MinValue, Fixed64.Zero),
            new(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.Zero),
            new(Fixed64.MinValue, Fixed64.MaxValue, Fixed64.Zero)
        };

        Action create = () => _ = new PhysicsMesh(
            vertices,
            ValidTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);

        create.Should().NotThrow();
    }

    [Fact]
    public void Constructor_ShouldRejectFullDomainCollinearTriangle()
    {
        Vector3d[] vertices =
        {
            new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Zero
        };

        Action create = () => _ = new PhysicsMesh(
            vertices,
            ValidTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);

        create.Should().Throw<ArgumentException>()
            .WithParameterName("triangles")
            .WithMessage("*Degenerate triangles*");
    }

    [Fact]
    public void SameSignExtremeBounds_UseExactCenterDuringScaleAndSurfaceValidation()
    {
        Fixed64 coordinate = Fixed64.MaxValue - (Fixed64)4;
        var mesh = new PhysicsMesh(
            new[]
            {
                new Vector3d(coordinate, Fixed64.Zero, Fixed64.Zero),
                new Vector3d(coordinate + Fixed64.One, Fixed64.Zero, Fixed64.Zero),
                new Vector3d(coordinate, Fixed64.One, Fixed64.Zero)
            },
            ValidTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        mesh.ValidateSurfaceMassProperties(Vector3d.One);

        Fixed64 expectedCenter = FixedMath.Midpoint(coordinate, coordinate + Fixed64.One);
        mesh.LocalBounds.Center.X.Should().Be(expectedCenter);
        mesh.ScaledLocalBounds.Center.X.Should().Be(Fixed64.Zero);
        mesh.SurfaceMassProperties.CenterOfMass.X.Should().BeGreaterThanOrEqualTo(-Fixed64.Half);
        mesh.SurfaceMassProperties.CenterOfMass.X.Should().BeLessThanOrEqualTo(Fixed64.Half);
    }

    [Fact]
    public void OpenSurface_ShouldReportItsClosureFailureForVolumeMassProperties()
    {
        var mesh = new PhysicsMesh(
            ValidVertices(),
            ValidTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        mesh.TryGetClosedVolumeMassProperties(
                out _,
                out MeshVolumeValidationResult result)
            .Should().BeFalse();
        result.Should().NotBe(MeshVolumeValidationResult.Valid);
    }

    [Fact]
    public void ScaleValidators_ShouldRejectMismatchedOwnerAndCompoundPartScale()
    {
        var standalone = new PhysicsMesh(
            ValidVertices(),
            ValidTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        Action invalidSurfaceOwner =
            () => standalone.ValidateSurfaceMassProperties(Vector3d.One * Fixed64.Two);
        Action invalidVolumeOwner =
            () => standalone.ValidateClosedVolumeScaleRepresentability(
                Vector3d.One * Fixed64.Two);

        invalidSurfaceOwner.Should().Throw<ArgumentException>()
            .WithParameterName("scale");
        invalidVolumeOwner.Should().Throw<ArgumentException>()
            .WithParameterName("scale");

        var compoundPart = new PhysicsMesh(
            ValidVertices(),
            ValidTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);
        compoundPart.PrepareTransformation(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One,
            Vector3d.One * Fixed64.Two,
            inertiaPolicy: null);
        compoundPart.PublishPreparedTransformation();
        Action invalidSurfacePart =
            () => compoundPart.ValidateSurfaceMassProperties(Vector3d.One);
        Action invalidVolumePart =
            () => compoundPart.ValidateClosedVolumeScaleRepresentability(Vector3d.One);

        invalidSurfacePart.Should().Throw<ArgumentException>()
            .WithParameterName("scale");
        invalidVolumePart.Should().Throw<ArgumentException>()
            .WithParameterName("scale");
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void ScaleAdmission_ShouldRejectCenteredVertexOverflowOnEveryAxis(
        int axis)
    {
        Vector3d extreme = axis switch
        {
            0 => new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            1 => new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero),
            _ => new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.MaxValue)
        };
        Vector3d transverse = axis switch
        {
            0 => Vector3d.Up,
            1 => Vector3d.Forward,
            _ => Vector3d.Right
        };
        var mesh = new PhysicsMesh(
            new[] { Vector3d.Zero, extreme, transverse },
            ValidTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);
        Vector3d scale = Vector3d.One * (Fixed64)3;

        mesh.GetScaledLocalRadius(scale, Vector3d.One)
            .Should().Be(Fixed64.MaxValue);
        Action prepare = () => mesh.PrepareTransformation(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            scale,
            Vector3d.One,
            inertiaPolicy: null);

        prepare.Should().Throw<ArgumentException>()
            .WithParameterName("ownerScale");
    }

    [Fact]
    public void Constructor_ConvexMode_ShouldRejectDisconnectedTriangleTopology()
    {
        Action create = () => _ = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)4, Fixed64.One, Fixed64.Zero)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Convex);

        create.Should().Throw<ArgumentException>()
            .WithParameterName("triangles")
            .WithMessage("*connected*");
    }

    [Fact]
    public void Constructor_ConcaveMode_ShouldAllowDisconnectedTriangleTopology()
    {
        Action create = () => _ = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)4, Fixed64.One, Fixed64.Zero)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);

        create.Should().NotThrow();
    }

    [Fact]
    public void Constructor_ConvexMode_ShouldRejectBoundaryBranches()
    {
        Action create = () => _ = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                Vector3d.Left,
                Vector3d.Down
            },
            new[] { 0, 1, 2, 0, 3, 4 },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Convex);

        create.Should().Throw<ArgumentException>()
            .WithParameterName("triangles")
            .WithMessage("*boundary loop*");
    }

    [Fact]
    public void Constructor_ConvexMode_ShouldRejectConcavePlanarSurface()
    {
        Action create = () => _ = new PhysicsMesh(
            new[]
            {
                new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)2, Fixed64.One, Fixed64.Zero),
                new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero),
                new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.Zero),
                new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero)
            },
            new[]
            {
                0, 1, 3,
                1, 2, 3,
                0, 3, 5,
                3, 4, 5
            },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Convex);

        create.Should().Throw<ArgumentException>()
            .WithParameterName("triangles")
            .WithMessage("*convex boundary*");
    }

    [Fact]
    public void Constructor_ConvexMode_ShouldRejectSelfOverlappingPlanarBoundary()
    {
        Action create = () => _ = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
                new Vector3d((Fixed64)2, (Fixed64)(-3), Fixed64.Zero),
                new Vector3d((Fixed64)(-3), Fixed64.One, Fixed64.Zero),
                new Vector3d((Fixed64)3, Fixed64.One, Fixed64.Zero),
                new Vector3d((Fixed64)(-2), (Fixed64)(-3), Fixed64.Zero)
            },
            new[]
            {
                0, 1, 2,
                0, 2, 3,
                0, 3, 4,
                0, 4, 5,
                0, 5, 1
            },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Convex);

        create.Should().Throw<ArgumentException>()
            .WithParameterName("triangles")
            .WithMessage("*convex boundary*");
    }

    [Fact]
    public void Constructor_ConvexMode_ShouldRejectFoldedFillInsideConvexBoundary()
    {
        Action create = () => _ = new PhysicsMesh(
            new[]
            {
                new Vector3d((Fixed64)3, Fixed64.One, Fixed64.Zero),
                Vector3d.Zero,
                new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)2, (Fixed64)2, Fixed64.Zero),
                new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero)
            },
            new[]
            {
                0, 1, 2,
                0, 2, 3,
                0, 3, 4,
                0, 4, 1
            },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Convex);

        create.Should().Throw<ArgumentException>()
            .WithParameterName("triangles")
            .WithMessage("*without holes, overlap, or folds*");
    }

    [Fact]
    public void IsClosedSurface_WithPinchedVertexLink_ShouldRejectClosedVolume()
    {
        Fixed64 half = Fixed64.Half;
        var mesh = new PhysicsMesh(
            new[]
            {
                new Vector3d(-half, -half, -half),
                new Vector3d(half, -half, -half),
                new Vector3d(-half, half, -half),
                new Vector3d(half, half, -half),
                new Vector3d(-half, -half, half),
                new Vector3d(half, -half, half),
                new Vector3d(-half, half, half),
                new Vector3d(-half, -half, -half)
            },
            CubeTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);

        bool valid = mesh.TryGetClosedVolumeMassProperties(
            out _,
            out MeshVolumeValidationResult result);

        mesh.IsClosedSurface.Should().BeFalse();
        valid.Should().BeFalse();
        result.Should().Be(MeshVolumeValidationResult.NonManifoldVertex);
    }

    [Fact]
    public void Constructor_ConvexMode_ShouldRejectDentedClosedShell()
    {
        Vector3d[] vertices = CubeVertices();
        vertices[7] = Vector3d.Zero;

        Action create = () => _ = new PhysicsMesh(
            vertices,
            CubeTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Convex);

        create.Should().Throw<ArgumentException>()
            .WithParameterName("triangles")
            .WithMessage("*reflex edges*");
    }

    [Fact]
    public void Constructor_ConvexMode_ShouldRejectBentOpenSurface()
    {
        Action create = () => _ = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One)
            },
            new[] { 0, 1, 2, 1, 3, 2 },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Convex);

        create.Should().Throw<ArgumentException>()
            .WithParameterName("triangles")
            .WithMessage("*coplanar*");
    }

    [Fact]
    public void Constructor_ConvexMode_ShouldRejectOneRawBentOpenSurface()
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        Action create = () => _ = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                new Vector3d(Fixed64.One, Fixed64.One, oneRaw)
            },
            new[] { 0, 1, 2, 1, 3, 2 },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Convex);

        create.Should().Throw<ArgumentException>()
            .WithParameterName("triangles")
            .WithMessage("*coplanar*");
    }

    [Fact]
    public void Constructor_ConvexMode_ShouldAllowCollinearBoundarySubdivisions()
    {
        Action create = () => _ = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Right * 2,
                new Vector3d((Fixed64)2, (Fixed64)2, Fixed64.Zero),
                Vector3d.Up * 2
            },
            new[]
            {
                0, 1, 4,
                1, 3, 4,
                1, 2, 3
            },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Convex);

        create.Should().NotThrow();
    }

    [Fact]
    public void Constructor_ConvexMode_ShouldRejectInconsistentSharedEdgeWinding()
    {
        Action create = () => _ = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                Vector3d.Right + Vector3d.Up
            },
            new[] { 0, 1, 2, 1, 2, 3 },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Convex);

        create.Should().Throw<ArgumentException>()
            .WithParameterName("triangles")
            .WithMessage("*winding*");
    }

    [Theory]
    [InlineData(MeshColliderMode.Convex)]
    [InlineData(MeshColliderMode.Concave)]
    public void Constructor_ShouldRejectUnreferencedVertices(MeshColliderMode mode)
    {
        Action create = () => _ = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                new Vector3d((Fixed64)100, (Fixed64)100, (Fixed64)100)
            },
            ValidTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity,
            mode);

        create.Should().Throw<ArgumentException>()
            .WithParameterName("vertices")
            .WithMessage("*referenced*");
    }

    [Fact]
    public void TriangleBVH_ShouldStoreTriangleBoundsAsMinMax()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)11, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)10, Fixed64.One, Fixed64.Zero)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);
        var hits = new SwiftList<int>();

        mesh.TriangleBVH.Query(
            new FixedBoundVolume(
                new Vector3d(Fixed64.FromFraction(9, 2), -Fixed64.Half, Fixed64.Zero),
                new Vector3d(Fixed64.FromFraction(11, 2), Fixed64.Half, Fixed64.Zero)),
            hits);

        hits.Should().Contain(1);
        hits.Should().NotContain(0);
    }

    [Fact]
    public void Constructor_WithNonNormalizedRotation_ShouldRejectBeforeStateCreation()
    {
        Action create = () => _ = new PhysicsMesh(
            ValidVertices(),
            ValidTriangles(),
            Vector3d.Zero,
            new FixedQuaternion(Fixed64.Half, Fixed64.Half, Fixed64.Zero, Fixed64.Zero));

        create.Should().Throw<ArgumentException>()
            .WithMessage("*normalized*");
    }

    [Fact]
    public void TriangleBVH_ShouldStayLocalWhenRigidMeshMoves()
    {
        var mesh = new PhysicsMesh(
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
        var hits = new SwiftList<int>();

        mesh.TriangleBVH.Query(
            new FixedBoundVolume(
                new Vector3d(Fixed64.FromFraction(-3, 2), -Fixed64.Half, Fixed64.Zero),
                new Vector3d(-Fixed64.Half, Fixed64.Half, Fixed64.Zero)),
            hits);
        int buildCount = mesh.TriangleBvhBuildCount;

        mesh.UpdatePosition(
            new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));
        mesh.TriangleBVH.Query(
            new FixedBoundVolume(
                new Vector3d(Fixed64.FromFraction(-3, 2), -Fixed64.Half, Fixed64.Zero),
                new Vector3d(-Fixed64.Half, Fixed64.Half, Fixed64.Zero)),
            hits);

        mesh.TriangleBvhBuildCount.Should().Be(buildCount);
        hits.Should().Contain(0);
    }

    [Fact]
    public void GetSupportVertexWorld_ShouldMatchStableWorldVertexScan()
    {
        PhysicsMesh mesh = MeshTestFixtures.CreateConvexCube().Mesh;
        PhysicsMesh denseMesh = MeshTestFixtures.CreateConvexPolygonFan(
            40,
            (Fixed64)4).Mesh;
        Vector3d[] directions =
        {
            Vector3d.Right,
            Vector3d.Left,
            Vector3d.Up,
            Vector3d.Down,
            Vector3d.Forward,
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized
        };

        AssertSupportMatchesStableScan(mesh, directions);
        AssertSupportMatchesStableScan(denseMesh, directions);

        mesh.UpdatePosition(
            new Vector3d((Fixed64)3, Fixed64.One, (Fixed64)(-2)),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));
        denseMesh.UpdatePosition(
            new Vector3d((Fixed64)(-2), (Fixed64)3, Fixed64.One),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)45, Fixed64.Zero));

        AssertSupportMatchesStableScan(mesh, directions);
        AssertSupportMatchesStableScan(denseMesh, directions);
    }

    [Fact]
    public void GetSupportVertexWorld_WithZeroDirection_ShouldUseStableRightAxisFallback()
    {
        PhysicsMesh mesh = MeshTestFixtures.CreateConvexCube().Mesh;

        mesh.GetSupportVertexWorld(Vector3d.Zero)
            .Should().Be(ScanSupportWithGetVertexWorld(mesh, Vector3d.Right));
    }

    [Fact]
    public void GetSupportVertexWorld_WithAcceleratedExtremeTranslatedMesh_ShouldCompareLocalFeatures()
    {
        PhysicsMesh mesh = MeshTestFixtures.CreateConvexPolygonFan(
            40,
            (Fixed64)4).Mesh;
        Vector3d direction = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized;
        Vector3d untranslatedSupport = mesh.GetSupportVertexWorld(direction);
        Fixed64 offset = new(2_000_000_000);
        mesh.UpdatePosition(
            new Vector3d(offset, offset, Fixed64.Zero),
            FixedQuaternion.Identity);

        Vector3d support = mesh.GetSupportVertexWorld(direction);

        support.Should().Be(untranslatedSupport + new Vector3d(offset, offset, Fixed64.Zero));
    }

    [Fact]
    public void GetSupportVertexWorld_WithDenseFlatFace_ShouldUseStableAuthoredOrder()
    {
        const int verticesPerEdge = 10;
        const int boundaryVertexCount = verticesPerEdge * 4;
        var vertices = new Vector3d[boundaryVertexCount + 1];
        var triangles = new int[boundaryVertexCount * 3];
        vertices[0] = Vector3d.Zero;
        for (int i = 0; i < boundaryVertexCount; i++)
        {
            Fixed64 edgeOffset = Fixed64.FromFraction(i % verticesPerEdge, verticesPerEdge) * 2;
            vertices[i + 1] = (i / verticesPerEdge) switch
            {
                0 => new Vector3d(-Fixed64.One + edgeOffset, -Fixed64.One, Fixed64.Zero),
                1 => new Vector3d(Fixed64.One, -Fixed64.One + edgeOffset, Fixed64.Zero),
                2 => new Vector3d(Fixed64.One - edgeOffset, Fixed64.One, Fixed64.Zero),
                _ => new Vector3d(-Fixed64.One, Fixed64.One - edgeOffset, Fixed64.Zero),
            };

            int triangleIndex = i * 3;
            triangles[triangleIndex] = 0;
            triangles[triangleIndex + 1] = i + 1;
            triangles[triangleIndex + 2] = ((i + 1) % boundaryVertexCount) + 1;
        }

        var mesh = new PhysicsMesh(
            vertices,
            triangles,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Convex);

        mesh.GetSupportVertexWorld(Vector3d.Right)
            .Should().Be(new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.Zero));
    }

    [Fact]
    public void GetFrontalArea_ShouldSumOnlyFacingWorldTriangles()
    {
        PhysicsMesh mesh = MeshTestFixtures.CreateConvexQuadFloor().Mesh;

        mesh.GetFrontalArea(Vector3d.Up).Should().Be(mesh.TotalArea);
        mesh.GetFrontalArea(-Vector3d.Up).Should().Be(Fixed64.Zero);
        mesh.GetFrontalArea(Vector3d.Zero).Should().Be(Fixed64.Zero);

        mesh.UpdatePosition(Vector3d.Zero, FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));

        mesh.GetFrontalArea(Vector3d.Left).Should().Be(mesh.TotalArea);
        mesh.GetFrontalArea(Vector3d.Up).Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void MeshColliderAreaAndFrontalArea_ShouldExposeMeshGeometry()
    {
        LSMeshCollider collider = MeshTestFixtures.CreateConvexQuadFloor();

        collider.Area.Should().Be(collider.Mesh.TotalArea);
        collider.GetFrontalArea(Vector3d.Up).Should().Be(collider.Mesh.TotalArea);
        collider.GetFrontalArea(-Vector3d.Up).Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Constructor_ShouldStoreExplicitMeshColliderMode()
    {
        var collider = new LSMeshCollider(
            ValidVertices(),
            ValidTriangles(),
            MeshColliderMode.Concave);

        collider.Mode.Should().Be(MeshColliderMode.Concave);
        collider.Mesh.Mode.Should().Be(MeshColliderMode.Concave);
        collider.IsClosedSurface.Should().BeFalse();
    }

    [Fact]
    public void IsClosedSurface_ShouldDescribeAuthoredTopologyIndependentOfMode()
    {
        var closedConcave = new PhysicsMesh(
            CubeVertices(),
            CubeTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);
        var openConvex = new PhysicsMesh(
            ValidVertices(),
            ValidTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Convex);
        var disconnectedConcave = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
                new Vector3d((Fixed64)4, Fixed64.One, Fixed64.Zero)
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);

        closedConcave.IsClosedSurface.Should().BeTrue();
        openConvex.IsClosedSurface.Should().BeFalse();
        disconnectedConcave.IsClosedSurface.Should().BeFalse();
    }

    [Theory]
    [InlineData(MeshColliderMode.Convex)]
    [InlineData(MeshColliderMode.Concave)]
    public void IsClosedSurface_WithExactPositionSeams_ShouldRecognizeClosedShell(MeshColliderMode mode)
    {
        CreateSeamedCube(out Vector3d[] vertices, out int[] triangles);

        var mesh = new PhysicsMesh(
            vertices,
            triangles,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            mode);

        mesh.IsClosedSurface.Should().BeTrue();
    }

    [Fact]
    public void Constructor_ConvexMode_ShouldAcceptOpenPlanarSurfaceWithExactPositionSeam()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                Vector3d.Right,
                Vector3d.Right + Vector3d.Up,
                Vector3d.Up
            },
            new[] { 0, 1, 2, 3, 4, 5 },
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Convex);

        mesh.IsClosedSurface.Should().BeFalse();
    }

    [Fact]
    public void ConcaveMesh_WithSurfaceApproximation_ShouldAllowDynamicBodyInitialization()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new LSMeshCollider(
            ValidVertices(),
            ValidTriangles(),
            MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation);

        ScenarioBody<LSMeshCollider> body = scenario.CreateBody(
            collider,
            Vector3d.Zero,
            FixedQuaternion.Identity);

        body.Collider.Mode.Should().Be(MeshColliderMode.Concave);
        body.Body.IsKinematic.Should().BeFalse();
    }

    [Fact]
    public void ConcaveMesh_ShouldAllowKinematicBodyInitialization()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var collider = new LSMeshCollider(
            ValidVertices(),
            ValidTriangles(),
            MeshColliderMode.Concave);

        ScenarioBody<LSMeshCollider> body = scenario.CreateBody(
            collider,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            isKinematic: true);

        body.Collider.Mode.Should().Be(MeshColliderMode.Concave);
    }

    [Fact]
    public void CalculateInertiaTensor_ShouldUseLocalGeometryForRigidMovement()
    {
        PhysicsMesh mesh = MeshTestFixtures.CreateConvexCube().Mesh;

        Fixed3x3 originTensor = mesh.CalculateInertiaTensor((Fixed64)3);
        mesh.UpdatePosition(new Vector3d((Fixed64)10, (Fixed64)4, (Fixed64)(-2)), FixedQuaternion.Identity);

        mesh.CalculateInertiaTensor((Fixed64)3).Should().Be(originTensor);
    }

    [Fact]
    public void CalculateInertiaTensor_ForClosedUnitCube_ShouldMatchSolidBox()
    {
        PhysicsMesh mesh = MeshTestFixtures.CreateConvexCube().Mesh;

        Fixed3x3 tensor = mesh.CalculateInertiaTensor((Fixed64)3);

        Fixed64 expected = Fixed64.Half;
        AssertNear(tensor.M11, expected);
        AssertNear(tensor.M22, expected);
        AssertNear(tensor.M33, expected);
        tensor.M12.Should().Be(Fixed64.Zero);
        tensor.M13.Should().Be(Fixed64.Zero);
        tensor.M21.Should().Be(Fixed64.Zero);
        tensor.M23.Should().Be(Fixed64.Zero);
        tensor.M31.Should().Be(Fixed64.Zero);
        tensor.M32.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void MeshCollider_WithZeroScaleAxes_ShouldRejectBeforeRegistration()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider collider = MeshTestFixtures.CreateConvexCube();
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Zero));
        Action initialize = () => collider.InitializeWithNoBody(agent);

        initialize.Should().Throw<ArgumentException>().WithMessage("*greater than zero*");
        collider.Id.Should().Be(-1);
        collider.HasHostBinding.Should().BeFalse();
        context.Physics.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void CalculateInertiaTensor_ForReversedClosedVolume_ShouldMatchForwardWinding()
    {
        LSMeshCollider forward = MeshTestFixtures.CreateConvexCube();
        LSMeshCollider reversed = CreateReversedCube();

        Fixed3x3 forwardTensor = forward.Mesh.CalculateInertiaTensor((Fixed64)3);
        Fixed3x3 reversedTensor = reversed.Mesh.CalculateInertiaTensor((Fixed64)3);

        reversedTensor.Should().Be(forwardTensor);
    }

    [Fact]
    public void CalculateInertiaTensor_ForRectangularPrism_ShouldMatchSolidBox()
    {
        var mesh = new PhysicsMesh(
            BoxVertices(Fixed64.One, Fixed64.Half, (Fixed64)2),
            CubeTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        Fixed3x3 tensor = mesh.CalculateInertiaTensor((Fixed64)6);

        AssertNear(tensor.M11, Fixed64.FromFraction(17, 2));
        AssertNear(tensor.M22, (Fixed64)10);
        AssertNear(tensor.M33, Fixed64.FromFraction(5, 2));
        tensor.M12.Should().Be(Fixed64.Zero);
        tensor.M13.Should().Be(Fixed64.Zero);
        tensor.M21.Should().Be(Fixed64.Zero);
        tensor.M23.Should().Be(Fixed64.Zero);
        tensor.M31.Should().Be(Fixed64.Zero);
        tensor.M32.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void TryGetClosedVolumeMassProperties_ForTetrahedron_ShouldCalculateVolumeAndCenterOfMass()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                Vector3d.Forward
            },
            new[]
            {
                1, 2, 3,
                0, 2, 1,
                0, 1, 3,
                0, 3, 2
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);

        bool valid = mesh.TryGetClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out MeshVolumeValidationResult result);

        valid.Should().BeTrue();
        result.Should().Be(MeshVolumeValidationResult.Valid);
        AssertNear(properties.Volume, Fixed64.FromFraction(1, 6));
        AssertNear(properties.CenterOfMass.X, Fixed64.FromFraction(-1, 4));
        AssertNear(properties.CenterOfMass.Y, Fixed64.FromFraction(-1, 4));
        AssertNear(properties.CenterOfMass.Z, Fixed64.FromFraction(-1, 4));
    }

    [Fact]
    public void CalculateInertiaTensor_ForOffCenterClosedVolume_ShouldShiftReferenceTensorToCenterOfMass()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                Vector3d.Forward
            },
            new[]
            {
                1, 2, 3,
                0, 2, 1,
                0, 1, 3,
                0, 3, 2
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        mesh.TryGetClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out _).Should().BeTrue();
        Fixed64 mass = (Fixed64)6;
        Fixed3x3 referenceTensor = properties.UnitMassInertiaTensor * mass;
        Vector3d referenceOffset = properties.InertiaReferencePoint - properties.CenterOfMass;

        Fixed3x3 tensor = mesh.CalculateInertiaTensor(mass);

        AssertNear(
            referenceTensor.M11,
            tensor.M11 + mass * ((referenceOffset.Y * referenceOffset.Y) + (referenceOffset.Z * referenceOffset.Z)));
        AssertNear(
            referenceTensor.M22,
            tensor.M22 + mass * ((referenceOffset.X * referenceOffset.X) + (referenceOffset.Z * referenceOffset.Z)));
        AssertNear(
            referenceTensor.M33,
            tensor.M33 + mass * ((referenceOffset.X * referenceOffset.X) + (referenceOffset.Y * referenceOffset.Y)));
    }

    [Fact]
    public void CalculateInertiaTensor_ForClosedTetrahedron_ShouldPreserveProductsOfInertia()
    {
        var mesh = new PhysicsMesh(
            new[]
            {
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Up,
                Vector3d.Forward
            },
            new[]
            {
                1, 2, 3,
                0, 2, 1,
                0, 1, 3,
                0, 3, 2
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);

        mesh.TryGetClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out _).Should().BeTrue();
        Fixed3x3 centerTensor = mesh.CalculateInertiaTensor(Fixed64.One);

        AssertNear(properties.UnitMassInertiaTensor.M12, Fixed64.FromFraction(-1, 20));
        AssertNear(properties.UnitMassInertiaTensor.M13, Fixed64.FromFraction(-1, 20));
        AssertNear(properties.UnitMassInertiaTensor.M23, Fixed64.FromFraction(-1, 20));
        AssertNear(centerTensor.M11, Fixed64.FromFraction(3, 40));
        AssertNear(centerTensor.M22, Fixed64.FromFraction(3, 40));
        AssertNear(centerTensor.M33, Fixed64.FromFraction(3, 40));
        AssertNear(centerTensor.M12, Fixed64.FromFraction(1, 80));
        AssertNear(centerTensor.M13, Fixed64.FromFraction(1, 80));
        AssertNear(centerTensor.M23, Fixed64.FromFraction(1, 80));
        centerTensor.M21.Should().Be(centerTensor.M12);
        centerTensor.M31.Should().Be(centerTensor.M13);
        centerTensor.M32.Should().Be(centerTensor.M23);
    }

    [Fact]
    public void CalculateInertiaTensor_WithSurfaceApproximationOptIn_ShouldAllowOpenMesh()
    {
        var mesh = new PhysicsMesh(
            ValidVertices(),
            ValidTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        Action calculate = () => _ = mesh.CalculateInertiaTensor((Fixed64)3, MeshInertiaPolicy.SurfaceApproximation);

        calculate.Should().NotThrow();
    }

    [Fact]
    public void CalculateInertiaTensor_WithUnknownPolicy_ShouldRejectPolicy()
    {
        var mesh = new PhysicsMesh(
            ValidVertices(),
            ValidTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        Action calculate = () => _ = mesh.CalculateInertiaTensor(
            Fixed64.One,
            (MeshInertiaPolicy)byte.MaxValue,
            Vector3d.Zero);

        calculate.Should().Throw<ArgumentOutOfRangeException>()
            .WithParameterName("policy");
    }

    [Fact]
    public void CalculateInertiaTensor_WithRequireClosedVolume_ShouldRejectOpenMesh()
    {
        var mesh = new PhysicsMesh(
            ValidVertices(),
            ValidTriangles(),
            Vector3d.Zero,
            FixedQuaternion.Identity);

        Action calculate = () => _ = mesh.CalculateInertiaTensor((Fixed64)3, MeshInertiaPolicy.RequireClosedVolume);

        calculate.Should().Throw<InvalidOperationException>()
            .WithMessage("*BoundaryEdge*");
    }

    [Theory]
    [MemberData(nameof(InvalidClosedVolumeMeshes))]
    public void TryGetClosedVolumeMassProperties_ShouldReturnExplicitFailureReasons(
        Vector3d[] vertices,
        int[] triangles,
        MeshVolumeValidationResult expectedResult)
    {
        var mesh = new PhysicsMesh(
            vertices,
            triangles,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            MeshColliderMode.Concave);

        bool valid = mesh.TryGetClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out MeshVolumeValidationResult result);

        valid.Should().BeFalse();
        properties.Should().Be(default(MeshMassProperties));
        result.Should().Be(expectedResult);
    }

    [Fact]
    public void MeshColliderRotation_ShouldRefreshWorldBoundsFromTransformedVertices()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var meshCollider = new LSMeshCollider(
            new[]
            {
                new Vector3d((Fixed64)(-2), (Fixed64)(-1), Fixed64.Zero),
                new Vector3d((Fixed64)2, (Fixed64)(-1), Fixed64.Zero),
                new Vector3d((Fixed64)(-2), Fixed64.One, Fixed64.Zero),
                new Vector3d((Fixed64)2, Fixed64.One, Fixed64.Zero)
            },
            new[] { 0, 1, 2, 1, 3, 2 },
            MeshColliderMode.Convex,
            MeshInertiaPolicy.SurfaceApproximation);
        ScenarioBody<LSMeshCollider> body = scenario.CreateBody(
            meshCollider,
            Vector3d.Zero,
            FixedQuaternion.Identity);

        body.Collider.Bounds.Proportions.X.Should().Be((Fixed64)4);
        body.Collider.Bounds.Proportions.Y.Should().Be((Fixed64)2);

        body.Body.SetRotation(FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90));
        body.Collider.Simulate();

        AssertNear(body.Collider.Bounds.Proportions.X, (Fixed64)2);
        AssertNear(body.Collider.Bounds.Proportions.Y, (Fixed64)4);
    }

    private static Vector3d[] ValidVertices() =>
        new[]
        {
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Up
        };

    private static int[] ValidTriangles() => new[] { 0, 1, 2 };

    private static void AssertSupportMatchesStableScan(PhysicsMesh mesh, Vector3d[] directions)
    {
        foreach (Vector3d direction in directions)
            mesh.GetSupportVertexWorld(direction).Should().Be(ScanSupportWithGetVertexWorld(mesh, direction));
    }

    private static Vector3d ScanSupportWithGetVertexWorld(PhysicsMesh mesh, Vector3d direction)
    {
        Vector3d best = mesh.GetVertexWorld(0);
        Fixed64 bestProjection = Vector3d.Dot(best, direction);
        for (int i = 1; i < mesh.VertexCount; i++)
        {
            Vector3d vertex = mesh.GetVertexWorld(i);
            Fixed64 projection = Vector3d.Dot(vertex, direction);
            if (projection <= bestProjection)
                continue;

            bestProjection = projection;
            best = vertex;
        }

        return best;
    }

    public static TheoryData<Vector3d[], int[], MeshVolumeValidationResult> InvalidClosedVolumeMeshes()
    {
        TheoryData<Vector3d[], int[], MeshVolumeValidationResult> data = new();
        data.Add(ValidVertices(), ValidTriangles(), MeshVolumeValidationResult.BoundaryEdge);
        data.Add(CubeVertices(), CubeTrianglesWithDuplicateFace(), MeshVolumeValidationResult.DuplicateTriangle);
        data.Add(CubeVertices(), CubeTrianglesWithInconsistentWinding(), MeshVolumeValidationResult.InconsistentWinding);
        data.Add(NonManifoldEdgeVertices(), NonManifoldEdgeTriangles(), MeshVolumeValidationResult.NonManifoldEdge);
        data.Add(DisconnectedCubeVertices(), DisconnectedCubeTriangles(), MeshVolumeValidationResult.DisconnectedShell);
        data.Add(PlanarClosedVolumeVertices(), PlanarClosedVolumeTriangles(), MeshVolumeValidationResult.ZeroVolume);
        return data;
    }

    private static LSMeshCollider CreateReversedCube()
    {
        Vector3d[] vertices = CubeVertices();
        int[] triangles = CubeTriangles();
        for (int i = 0; i < triangles.Length; i += 3)
            (triangles[i + 1], triangles[i + 2]) = (triangles[i + 2], triangles[i + 1]);

        return new LSMeshCollider(vertices, triangles);
    }

    private static Vector3d[] CubeVertices()
    {
        Fixed64 half = Fixed64.Half;
        return BoxVertices(half, half, half);
    }

    private static Vector3d[] BoxVertices(Fixed64 halfX, Fixed64 halfY, Fixed64 halfZ)
    {
        return new[]
        {
            new Vector3d(-halfX, -halfY, -halfZ),
            new Vector3d(halfX, -halfY, -halfZ),
            new Vector3d(-halfX, halfY, -halfZ),
            new Vector3d(halfX, halfY, -halfZ),
            new Vector3d(-halfX, -halfY, halfZ),
            new Vector3d(halfX, -halfY, halfZ),
            new Vector3d(-halfX, halfY, halfZ),
            new Vector3d(halfX, halfY, halfZ)
        };
    }

    private static int[] CubeTriangles() =>
        new[]
        {
            0, 2, 1, 1, 2, 3,
            4, 5, 6, 5, 7, 6,
            0, 4, 2, 2, 4, 6,
            1, 3, 5, 5, 3, 7,
            0, 1, 4, 1, 5, 4,
            2, 6, 3, 3, 6, 7
        };

    private static void CreateSeamedCube(out Vector3d[] vertices, out int[] triangles)
    {
        Vector3d[] cubeVertices = CubeVertices();
        int[] cubeTriangles = CubeTriangles();
        vertices = new Vector3d[cubeTriangles.Length];
        triangles = new int[cubeTriangles.Length];
        for (int i = 0; i < cubeTriangles.Length; i++)
        {
            vertices[i] = cubeVertices[cubeTriangles[i]];
            triangles[i] = i;
        }
    }

    private static int[] CubeTrianglesWithDuplicateFace()
    {
        int[] triangles = CubeTriangles();
        int[] result = new int[triangles.Length + 3];
        Array.Copy(triangles, result, triangles.Length);
        result[^3] = triangles[0];
        result[^2] = triangles[1];
        result[^1] = triangles[2];
        return result;
    }

    private static int[] CubeTrianglesWithInconsistentWinding()
    {
        int[] triangles = CubeTriangles();
        (triangles[1], triangles[2]) = (triangles[2], triangles[1]);
        return triangles;
    }

    private static Vector3d[] NonManifoldEdgeVertices() =>
        new[]
        {
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Up,
            Vector3d.Forward,
            new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero)
        };

    private static int[] NonManifoldEdgeTriangles() =>
        new[]
        {
            0, 1, 2,
            1, 0, 3,
            0, 1, 4
        };

    private static Vector3d[] DisconnectedCubeVertices()
    {
        Vector3d[] first = CubeVertices();
        Vector3d[] result = new Vector3d[first.Length * 2];
        Array.Copy(first, result, first.Length);
        for (int i = 0; i < first.Length; i++)
            result[first.Length + i] = first[i] + new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero);

        return result;
    }

    private static int[] DisconnectedCubeTriangles()
    {
        int[] first = CubeTriangles();
        int[] result = new int[first.Length * 2];
        Array.Copy(first, result, first.Length);
        for (int i = 0; i < first.Length; i++)
            result[first.Length + i] = first[i] + 8;

        return result;
    }

    private static Vector3d[] PlanarClosedVolumeVertices() =>
        new[]
        {
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Forward,
            Vector3d.Right + Vector3d.Forward
        };

    private static int[] PlanarClosedVolumeTriangles() =>
        new[]
        {
            1, 2, 3,
            0, 2, 1,
            0, 1, 3,
            0, 3, 2
        };

    private static void AssertNear(Fixed64 actual, Fixed64 expected) =>
        (actual - expected).Abs().Should().BeLessThan(Fixed64.FromFraction(1, 1000));
}
