using System.Collections.Generic;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class CertifiedMeshSweepTests
{
    private static readonly PhysicsLayerMask LayerZero = PhysicsLayerMask.FromLayer(0);
    private static readonly Fixed64 Tolerance = ConvexSweepQueryWorker.ContactTolerance * 2;

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AlmostAlignedCircleCast_ShouldRetainItsFiniteHitOrMiss(bool misses)
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        scene.InitializeStaticCollider(Quad(false, false), Vector3d.Zero);
        Fixed64 z = misses ? Fixed64.FromFraction(3, 2) : Fixed64.Zero;
        var start = new Vector2d((Fixed64)(-3), misses ? z + Fixed64.MinIncrement : z);
        var end = new Vector2d((Fixed64)3, misses ? z : z + Fixed64.MinIncrement);
        var hits = new SwiftList<PhysicsMixedHit>();
        scene.Context.QueryMixed.SweepCircleAgainst3DAll(start, end, Fixed64.Half,
            Fixed64.Zero, Fixed64.Half, LayerZero, hits).Should().Be(misses ? 0 : 1);
        if (!misses)
        {
            (hits[0].Distance - Fixed64.FromFraction(5, 2)).Abs().Should().BeLessThanOrEqualTo(Tolerance);
            hits[0].Point3D.X.Abs().Should().BeLessThanOrEqualTo(Tolerance);
        }
    }

    [Fact]
    public void CircleCastEndingBeforeTheCenterCrossing_ShouldRetainTheRadiusHit()
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        scene.InitializeStaticCollider(Quad(false, false), Vector3d.Zero);
        var hits = new SwiftList<PhysicsMixedHit>();
        scene.Context.QueryMixed.SweepCircleAgainst3DAll(Vector2d.Left * (Fixed64)3,
            Vector2d.Left * Fixed64.FromFraction(1, 4), Fixed64.Half, Fixed64.Zero,
            Fixed64.Half, LayerZero, hits).Should().Be(1);
        hits[0].Distance.Should().Be(Fixed64.FromFraction(5, 2));
        hits[0].Point3D.X.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(-45)]
    [InlineData(45)]
    public void PosedTiltedFace_ShouldPublishTheMatchingSlabCap(int roll)
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSMeshCollider target = Quad(false, false);
        target.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)roll), Vector3d.One)));
        Sweep(scene.Context, 2, null, Vector3d.Left * 3, Vector3d.Right * 6, out Fixed64 distance,
            out Vector3d point, out Vector3d normal, out Vector3d companion).Should().BeTrue();
        (distance - Fixed64.Two).Abs().Should().BeLessThanOrEqualTo(Tolerance);
        Vector3d expectedNormal = new Vector3d(-1, roll > 0 ? -1 : 1, 0).Normalized;
        Vector3d.Dot(normal, expectedNormal).Should().BeGreaterThan(Fixed64.FromFraction(999, 1000));
        companion.Y.Should().Be(roll > 0 ? Fixed64.Half : -Fixed64.Half);
        (point - companion).Magnitude.Should().BeLessThanOrEqualTo(Tolerance);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AlmostVerticalFace_ShouldKeepTheExactCapSign(bool mirror)
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        Fixed64 offset = Fixed64.MinIncrement;
        Fixed64 slope = Fixed64.MinIncrement * (mirror ? -2 : 2);
        // The first cap contact is exactly X=0 at distance 2.5, despite the sub-raw slope.
        var target = new LSMeshCollider(new[] { new Vector3d(offset - slope, -Fixed64.One, -Fixed64.One),
            new Vector3d(offset + slope, Fixed64.One, -Fixed64.One), new Vector3d(offset + slope, Fixed64.One, Fixed64.One),
            new Vector3d(offset - slope, -Fixed64.One, Fixed64.One), new Vector3d(offset, Fixed64.Zero, Fixed64.Zero) },
            new[] { 0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        target.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, new Vector3d(Fixed64.FromFraction(1, 8), Fixed64.One, Fixed64.One))));
        Sweep(scene.Context, 2, null, Vector3d.Left * 3, Vector3d.Right * 6, out Fixed64 distance,
            out Vector3d point, out Vector3d normal, out Vector3d companion).Should().BeTrue();
        (distance - Fixed64.FromFraction(5, 2)).Abs().Should().BeLessThanOrEqualTo(Fixed64.MinIncrement * 2);
        normal.Should().Be(Vector3d.Left);
        companion.Y.Should().Be(mirror ? Fixed64.Half : -Fixed64.Half);
        (point - companion).Magnitude.Should().BeLessThanOrEqualTo(Tolerance);
    }

    [Fact]
    public void AlmostHorizontalFace_ShouldKeepItsFiniteSlabEntry()
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        Fixed64 tiny = Fixed64.MinIncrement;
        var target = new LSMeshCollider(new[] { new Vector3d(-Fixed64.One, -tiny, -Fixed64.One),
            new Vector3d(Fixed64.One, tiny, -Fixed64.One), new Vector3d(Fixed64.One, tiny, Fixed64.One),
            new Vector3d(-Fixed64.One, -tiny, Fixed64.One), Vector3d.Zero },
            new[] { 0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        target.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, new Vector3d(Fixed64.One, Fixed64.Half, Fixed64.One))));
        Sweep(scene.Context, 2, null, new Vector3d((Fixed64)(-3), Fixed64.Half, Fixed64.Zero), Vector3d.Right * 6,
            out Fixed64 distance, out Vector3d point, out _, out Vector3d companion).Should().BeTrue();
        (distance - Fixed64.FromFraction(5, 2)).Abs().Should().BeLessThanOrEqualTo(Tolerance);
        point.X.Abs().Should().BeLessThanOrEqualTo(Tolerance);
        point.Y.Abs().Should().BeLessThanOrEqualTo(Fixed64.MinIncrement);
        (point - companion).Magnitude.Should().BeLessThanOrEqualTo(Tolerance);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PartialOrInitiallyIntersectingBounds_ShouldRetainAuthoredFiniteEntry(bool tilted)
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var vertices = new List<Vector3d>();
        var triangles = new List<int>();
        if (tilted)
        {
            vertices.AddRange(new[] { new Vector3d(1, -1, -1), new Vector3d(-1, 1, -1),
                new Vector3d(-1, 1, 1), new Vector3d(1, -1, 1), Vector3d.Zero });
            triangles.AddRange(new[] { 0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4 });
        }
        else
        {
            for (int y = -2; y <= 2; y++)
                for (int z = -2; z <= 2; z++) vertices.Add(new Vector3d(0, y, z));
            for (int y = 0; y < 4; y++)
                for (int z = 0; z < 4; z++)
                {
                    int first = y * 5 + z;
                    triangles.AddRange(new[] { first, first + 5, first + 1, first + 1, first + 5, first + 6 });
                }
        }
        var target = new LSMeshCollider(vertices.ToArray(), triangles.ToArray(),
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(target, Vector3d.Zero);
        Vector3d start = Vector3d.Left * (tilted ? Fixed64.FromFraction(5, 4) : (Fixed64)3);
        Sweep(scene.Context, 2, null, start, Vector3d.Right * 6, out Fixed64 distance,
            out Vector3d point, out _, out Vector3d companion).Should().BeTrue();
        (distance - (tilted ? Fixed64.Quarter : Fixed64.FromFraction(5, 2))).Abs()
            .Should().BeLessThanOrEqualTo(Tolerance);
        (point - companion).Magnitude.Should().BeLessThanOrEqualTo(Tolerance);
        (point.X + (tilted ? point.Y : Fixed64.Zero)).Abs().Should().BeLessThanOrEqualTo(Tolerance);
        if (!tilted)
            scene.Context.QueryMixed.LastMeshTriangleCandidateCount.Should().BeInRange(3, 31);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MixedEntry_ShouldDistinguishARealRimFromATiltedSupportingFace(bool tilted)
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        Vector3d[] vertices = tilted
            ? new[] { new Vector3d(1, -1, -1), new Vector3d(-1, 1, -1), new Vector3d(-1, 1, 1),
                new Vector3d(1, -1, 1), Vector3d.Zero }
            : new[] { new Vector3d(-1, 0, -1), new Vector3d(1, 0, -1), new Vector3d(1, 0, 1),
                new Vector3d(-1, 0, 1), Vector3d.Zero };
        var target = new LSMeshCollider(vertices, new[] { 0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(target, Vector3d.Zero);
        Sweep(scene.Context, 2, null, Vector3d.Left * 3, Vector3d.Right * 6, out Fixed64 distance,
            out Vector3d point, out Vector3d normal, out Vector3d companion).Should().BeTrue();
        (distance - (tilted ? Fixed64.Two : Fixed64.FromFraction(3, 2))).Abs()
            .Should().BeLessThanOrEqualTo(Tolerance);
        (point - companion).Magnitude.Should().BeLessThanOrEqualTo(Tolerance);
        if (tilted)
        {
            (point.X + point.Y).Abs().Should().BeLessThanOrEqualTo(Tolerance);
            Vector3d.Dot(normal, new Vector3d(-1, -1, 0).Normalized)
                .Should().BeGreaterThan(Fixed64.FromFraction(999, 1000));
        }
        else
        {
            point.Y.Should().Be(Fixed64.Zero);
            (point.X + Fixed64.One).Abs().Should().BeLessThanOrEqualTo(Tolerance);
            normal.Should().Be(Vector3d.Left);
        }
    }

    [Fact]
    public void TangentialCircleSlabEntry_ShouldRetainTheFiniteRimHit()
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        scene.InitializeStaticCollider(Quad(false, false), Vector3d.Zero);
        var start = new Vector3d(-Fixed64.Half, Fixed64.Zero, (Fixed64)(-3));
        Sweep(scene.Context, 2, null, start, Vector3d.Forward * 6, out Fixed64 distance,
            out Vector3d point, out _, out _).Should().BeTrue();
        (distance - Fixed64.Two).Abs().Should().BeLessThanOrEqualTo(Tolerance);
        point.X.Abs().Should().BeLessThanOrEqualTo(Tolerance);
        (point.Z + Fixed64.One).Abs().Should().BeLessThanOrEqualTo(Tolerance);
    }

    [Theory]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(16, false)]
    [InlineData(16, true)]
    public void RepeatedMissTriangles_ShouldNotHideADifferentHitOrRetainSourceInputs(int repeats, bool includeHit)
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var vertices = new List<Vector3d>();
        var indices = new List<int>();
        for (int i = 0; i < repeats; i++) AddMiss();
        for (int i = 0; includeHit && i < repeats; i++)
        {
            int first = vertices.Count;
            vertices.AddRange(new[] { new Vector3d(0, -1, -1), new Vector3d(0, 1, -1), new Vector3d(0, 0, 1) });
            indices.AddRange(new[] { first, first + 1, first + 2 });
        }
        AddMiss();
        var target = new LSMeshCollider(vertices.ToArray(), indices.ToArray(),
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(target, Vector3d.Zero);
        var hits = new SwiftList<PhysicsMixedHit>();
        PhysicsMixedHit firstHit = default;
        int query = 0;
        foreach (Fixed64 radius in new[] { Fixed64.Half, Fixed64.FromFraction(1, 8), Fixed64.Half })
        {
            int count = scene.Context.QueryMixed.SweepCircleAgainst3DAll(Vector2d.Left * (Fixed64)3, Vector2d.Right * (Fixed64)3,
                radius, Fixed64.Zero, Fixed64.Half, LayerZero, hits);
            count.Should().Be(includeHit ? 1 : 0);
            if (includeHit)
            {
                hits[0].Distance.Should().Be((Fixed64)3 - radius);
                hits[0].Point3D.X.Should().Be(Fixed64.Zero);
                if (query == 0) firstHit = hits[0];
                if (query == 2) hits[0].Should().Be(firstHit);
            }
            query++;
        }

        void AddMiss()
        {
            int first = vertices.Count;
            vertices.AddRange(new[] {
                new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.FromFraction(49, 100)),
                new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.FromFraction(71, 100)),
                new Vector3d(0, -1, 1) });
            indices.AddRange(new[] { first, first + 1, first + 2 });
        }
    }

    [Fact]
    public void AdmittedMicroPatch_ShouldRetainItsSphereFaceHit()
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Quad(false, false);
        target.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * Fixed64.FromFraction(1, 200))));
        target.Mesh.GetScaledLocalFaceNormal(0).Should().NotBe(Vector3d.Zero);
        Vector3d start = new(-Fixed64.FromFraction(1, 100), Fixed64.FromFraction(1, 5000), -Fixed64.FromFraction(1, 5000));
        Fixed64 radius = Fixed64.FromFraction(1, 4000);
        var hits = new SwiftList<Physics3DHit>();
        scene.Context.Query3D.SweepSphereAll(start, start + Vector3d.Right * Fixed64.FromFraction(1, 50),
            radius, LayerZero, hits).Should().Be(1);
        (hits[0].Distance - (Fixed64.FromFraction(1, 100) - radius)).Abs()
            .Should().BeLessThanOrEqualTo(Fixed64.MinIncrement * 2);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void OffsetSlabAgainstVerticalPatch_ShouldKeepASupportingFaceNormal(int side)
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        var vertices = new Vector3d[9];
        for (int y = 0; y < 3; y++)
            for (int z = 0; z < 3; z++)
                vertices[y * 3 + z] = new Vector3d(Fixed64.Zero, Fixed64.FromFraction(y - 1, 2), Fixed64.FromFraction(z - 1, 2));
        var target = new LSMeshCollider(vertices, new[] { 0, 3, 1, 1, 3, 4, 1, 4, 2, 2, 4, 5,
            3, 6, 4, 4, 6, 7, 4, 7, 5, 5, 7, 8 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scene.InitializeStaticCollider(target, Vector3d.Zero);
        Vector3d start = new((Fixed64)(-3), Fixed64.Half * side, Fixed64.Zero);
        Sweep(scene.Context, 2, null, start, Vector3d.Right * 6, out Fixed64 distance, out _,
            out Vector3d normal, out _).Should().BeTrue();
        distance.Should().Be(Fixed64.FromFraction(5, 2));
        // A vertical filled face can only support this side impact with a horizontal normal.
        normal.Should().Be(Vector3d.Left);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void OrdinaryFace_ShouldRetainAnalyticTimeAndFiniteWitnesses(int kind, bool affine)
    {
        foreach (bool reverse in new[] { false, true })
            foreach (FixedQuaternion rotation in new[] { FixedQuaternion.Identity, PhysicsScenarioBuilder.Yaw(31),
            new FixedQuaternion(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One).Normalized })
                foreach (Fixed64 z in new[] { Fixed64.Zero, Fixed64.MinValue + (Fixed64)32 })
                    foreach (int side in new[] { -1, 1 })
                    {
                        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
                        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
                        Vector3d origin = Vector3d.Forward * z;
                        if (z != Fixed64.Zero)
                            scene.Context.World.TryAddGrid(new GridConfiguration(origin - Vector3d.One * 8,
                                origin + Vector3d.One * 8), out _).Should().BeTrue();
                        LSMeshCollider target = Quad(affine, reverse);
                        target.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(origin, rotation,
                            new Vector3d(affine ? Fixed64.Half : Fixed64.One, Fixed64.One, Fixed64.One))));
                        target.Mesh.HasSingleConvexCoplanarPatch.Should().BeTrue();
                        Vector3d start = origin + rotation * Vector3d.Right * (side * 3);
                        Vector3d travel = rotation * Vector3d.Left * (side * 6);
                        LSMeshCollider? source = CreateSource(scene, kind, start, rotation);

                        Sweep(scene.Context, kind, source, start, travel, out Fixed64 distance, out Vector3d point,
                            out Vector3d normal, out Vector3d companion).Should().BeTrue();
                        (distance - Fixed64.FromFraction(5, 2)).Abs().Should().BeLessThanOrEqualTo(Tolerance);
                        Vector3d local = rotation.Inverse() * (point - origin);
                        local.X.Abs().Should().BeLessThanOrEqualTo(Tolerance);
                        local.Y.Abs().Should().BeLessThanOrEqualTo(kind == 2 ? Fixed64.Half + Tolerance : Fixed64.One + Tolerance);
                        local.Z.Abs().Should().BeLessThanOrEqualTo(Fixed64.One + Tolerance);
                        Vector3d.Dot(normal, travel).Should().BeLessThan(Fixed64.Zero);
                        Vector3d center = start + travel.Normalized * distance;
                        if (kind == 0)
                            ((point - center).Magnitude - Fixed64.Half).Abs().Should().BeLessThanOrEqualTo(Tolerance);
                        else if (kind == 1)
                        {
                            Vector3d onSource = rotation.Inverse() * (point - center);
                            (onSource.X.Abs() - Fixed64.Half).Abs().Should().BeLessThanOrEqualTo(Tolerance);
                            onSource.Y.Abs().Should().BeLessThanOrEqualTo(Fixed64.Half + Tolerance);
                            onSource.Z.Abs().Should().BeLessThanOrEqualTo(Fixed64.Half + Tolerance);
                        }
                        else
                        {
                            (point - companion).Magnitude.Should().BeLessThanOrEqualTo(Tolerance);
                            ((companion.ToVector2d() - center.ToVector2d()).Magnitude - Fixed64.Half).Abs()
                                .Should().BeLessThanOrEqualTo(Tolerance);
                        }
                    }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void RimAndEmptyCorner_ShouldKeepTheFiniteSurface(int kind)
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        scene.InitializeStaticCollider(Quad(false, false), Vector3d.Zero);
        Vector3d start = new((Fixed64)(-3), Fixed64.Zero, -Fixed64.FromFraction(1, 5));
        Vector3d travel = new(6, 0, 3);
        LSMeshCollider? source = CreateSource(scene, kind, start, FixedQuaternion.Identity);
        Sweep(scene.Context, kind, source, start, travel, out Fixed64 distance, out Vector3d point,
            out _, out _).Should().BeTrue();
        Fixed64 expected;
        if (kind == 1) expected = Fixed64.FromFraction(5, 2) * travel.Magnitude / (Fixed64)6;
        else
        {
            // Circle against the genuine rim at X=0,Z=1: |start + t*travel - rim|² = 1/4.
            Vector3d offset = start - Vector3d.Forward;
            Fixed64 a = travel.MagnitudeSquared;
            Fixed64 b = Vector3d.Dot(offset, travel) * 2;
            Fixed64 c = offset.MagnitudeSquared - Fixed64.Quarter;
            expected = (-b - FixedMath.Sqrt(b * b - (Fixed64)4 * a * c)) / (2 * a) * travel.Magnitude;
        }
        (distance - expected).Abs().Should().BeLessThanOrEqualTo(Tolerance);
        point.X.Abs().Should().BeLessThanOrEqualTo(Tolerance);
        point.Z.Abs().Should().BeLessThanOrEqualTo(Fixed64.One + Tolerance);

        start = new Vector3d(-3, 0, -3);
        travel = new Vector3d(6, 0, 12);
        if (source != null) { source.Position = start; source.Simulate(); }
        Sweep(scene.Context, kind, source, start, travel, out _, out _, out _, out _).Should().BeFalse();
    }

    [Fact]
    public void ConvexMeshInspectionCenterChord_ShouldNotInventAHullContact()
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = Quad(false, false);
        target.InitializeWithNoBody(new TestMatterAgent(scene.Context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * Fixed64.FromFraction(1, 8))));
        var source = new LSMeshCollider(new[] { new Vector3d(-1, -1, -1), new Vector3d(1, -1, -1),
            new Vector3d(-1, 1, -1), new Vector3d(-1, -1, 1) },
            new[] { 0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3 }, MeshColliderMode.Convex, MeshInertiaPolicy.RequireClosedVolume);
        scene.InitializeStaticCollider(source, new Vector3d(-3, 0, 0));
        new ConvexShape(source, Vector3d.Zero).ContainsCenter.Should().BeFalse();
        // At the endpoint the hull has X+Y+Z <= -1/2; the finite target has Y+Z >= -1/4.
        scene.Context.Query3D.SweepConvexMesh(source, new Vector3d(Fixed64.FromFraction(7, 2), Fixed64.Zero, Fixed64.Zero),
            LayerZero, out _).Should().BeFalse();
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void EqualDistanceTargets_ShouldKeepColliderOrderAndRepeatExactly(int kind)
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        LSMeshCollider first = Quad(false, false), second = Quad(false, true);
        scene.InitializeStaticCollider(first, Vector3d.Zero);
        scene.InitializeStaticCollider(second, Vector3d.Zero);
        Vector3d start = Vector3d.Left * 3, travel = Vector3d.Right * 6;
        LSMeshCollider? source = CreateSource(scene, kind, start, FixedQuaternion.Identity);
        var spatial = new SwiftList<Physics3DHit>();
        var mixed = new SwiftList<PhysicsMixedHit>();
        Physics3DHit expectedSpatial = default;
        PhysicsMixedHit expectedMixed = default;
        for (int repeat = 0; repeat < 4; repeat++)
        {
            if (kind == 2)
            {
                scene.Context.QueryMixed.SweepCircleAgainst3DAll(start.ToVector2d(), (start + travel).ToVector2d(),
                    Fixed64.Half, Fixed64.Zero, Fixed64.Half, LayerZero, mixed).Should().Be(2);
                mixed[0].Collider3D.Should().BeSameAs(first);
                mixed[1].Collider3D.Should().BeSameAs(second);
                mixed[0].Distance.Should().Be(mixed[1].Distance);
                if (repeat == 0) expectedMixed = mixed[0];
                else mixed[0].Should().Be(expectedMixed);
            }
            else
            {
                int count = kind == 0
                    ? scene.Context.Query3D.SweepSphereAll(start, start + travel, Fixed64.Half, LayerZero, spatial)
                    : scene.Context.Query3D.SweepConvexMeshAll(source!, travel, LayerZero, spatial);
                count.Should().Be(2);
                spatial[0].Collider.Should().BeSameAs(first);
                spatial[1].Collider.Should().BeSameAs(second);
                spatial[0].Distance.Should().Be(spatial[1].Distance);
                if (repeat == 0) expectedSpatial = spatial[0];
                else spatial[0].Should().Be(expectedSpatial);
            }
        }
    }

    private static LSMeshCollider? CreateSource(PhysicsScenarioBuilder scene, int kind, Vector3d start,
        FixedQuaternion rotation)
    {
        if (kind != 1) return null;
        LSMeshCollider source = MeshTestFixtures.CreateConvexCube();
        source.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(start, rotation, Vector3d.One)));
        return source;
    }

    private static bool Sweep(GravitasWorldContext context, int kind, LSMeshCollider? source,
        Vector3d start, Vector3d travel, out Fixed64 distance, out Vector3d point,
        out Vector3d normal, out Vector3d companion)
    {
        companion = default;
        if (kind == 2)
        {
            bool found = context.QueryMixed.SweepCircleAgainst3D(start.ToVector2d(), (start + travel).ToVector2d(),
                Fixed64.Half, start.Y, Fixed64.Half, LayerZero, out PhysicsMixedHit mixed);
            distance = mixed.Distance; point = found ? mixed.Point3D : default;
            normal = mixed.Normal3DTo2D; companion = found ? mixed.Point2D : default;
            return found;
        }
        var hits = new SwiftList<Physics3DHit>();
        bool result = kind == 0
            ? context.Query3D.SweepSphereAll(start, start + travel, Fixed64.Half, LayerZero, hits) > 0
            : context.Query3D.SweepConvexMeshAll(source!, travel, LayerZero, hits) > 0;
        Physics3DHit hit = result ? hits[0] : default;
        distance = hit.Distance; point = result ? hit.Point : default; normal = hit.Normal;
        return result;
    }

    private static LSMeshCollider Quad(bool affine, bool reverse)
    {
        Fixed64 x = affine ? Fixed64.MinIncrement : Fixed64.Zero;
        return new LSMeshCollider(new[] { new Vector3d(x, -Fixed64.One, -Fixed64.One), new Vector3d(x, Fixed64.One, -Fixed64.One),
            new Vector3d(x, Fixed64.One, Fixed64.One), new Vector3d(x, -Fixed64.One, Fixed64.One), new Vector3d(x, Fixed64.Zero, Fixed64.Zero) },
            reverse ? new[] { 4, 1, 0, 4, 2, 1, 4, 3, 2, 4, 0, 3 }
                : new[] { 0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4 },
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
    }
}
