using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests;

/// <content>Checks connected physical curved patches before bounded manifold reduction.</content>
public sealed partial class JoinedStaticSolidMixedTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedGatheredDisjointComponent_ShouldNotCreateAResponse(bool cone)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -4, 0, -2, 0, Fixed64.Two);
        var right = Rectangle(scene, 0, 4, -2, 0, Fixed64.Two);
        var remoteFirst = Rectangle(scene, 5, 6, -2, 0, Fixed64.Two);
        var remoteSecond = Rectangle(scene, 6, 7, -2, 0, Fixed64.Two);
        var members = new[] { left, right, remoteFirst, remoteSecond };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = cone ? new LSConeCollider { Radius = Fixed64.Half, Size = new(1, 6, 1) }
            : new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 6, 1) };
        var horizontal = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        scene.CreateBody(mover, Vector3d.Forward * Fixed64.Quarter, horizontal);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.True(sample.Supplier == left.Id + 1 || sample.Supplier == right.Id + 1);
            Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
        }
    }

    [Fact]
    public void CurvedPhysicalAndEmbeddedComponents_ShouldKeepIndependentSuppliers()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var wall = Rectangle(scene, -6, 5, -2, 0, Fixed64.Two);
        var wallJoin = Rectangle(scene, 5, 6, -2, 0, Fixed64.Two);
        var enclosing = Rectangle(scene, -4, 3, -4, 4, (Fixed64)10);
        var enclosingJoin = Rectangle(scene, 3, 4, -4, 4, (Fixed64)10);
        var members = new[] { wall, wallJoin, enclosing, enclosingJoin };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 6, 1) };
        scene.CreateBody(mover, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity);
        bool physical = false, far = false;
        foreach (var sample in Rows(mover, members))
        {
            Assert.Equal(Vector3d.Backward, sample.Row.Normal);
            if (sample.Supplier == wall.Id + 1) { physical = true; Assert.Equal(Fixed64.Quarter, sample.Row.Depth); }
            else
            { far = true; Assert.Equal(enclosing.Id + 1, sample.Supplier); Assert.Equal(17 * Fixed64.Quarter, sample.Row.Depth); }
        }
        // These overlapping components have no trusted join near the body.
        // A's chosen physical wall cannot borrow B's embedded recovery policy.
        Assert.True(physical && far);
    }

    [Fact]
    public void CurvedOuterRimAndSeparateFace_ShouldRetainBothFiniteSources()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var rim = Rectangle(scene, 3, 4, -2, 0, Fixed64.Two);
        var rimJoin = Rectangle(scene, 2, 3, -2, 0, Fixed64.Two);
        var face = Rectangle(scene, 4, 6, 0, 2, Fixed64.Two);
        var faceJoin = Rectangle(scene, 6, 8, 0, 2, Fixed64.Two);
        var members = new[] { rim, rimJoin, face, faceJoin };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) };
        var rotation = new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Two).Normalized;
        scene.CreateBody(mover, new(17 * Fixed64.Quarter, Fixed64.Zero, Fixed64.One), rotation);
        bool outer = false, flat = false;
        foreach (var sample in Rows(mover, members))
        {
            if (sample.Supplier == rim.Id + 1)
            { outer = true; Assert.True(sample.Row.Normal.X < Fixed64.Zero && sample.Row.Normal.Z < Fixed64.Zero); }
            else
            { flat = true; Assert.Equal(face.Id + 1, sample.Supplier); Assert.Equal(Vector3d.Right, sample.Row.Normal); Assert.Equal(3 * Fixed64.Quarter, sample.Row.Depth); }
        }
        Assert.True(outer && flat);
    }

    [Fact]
    public void CurvedPointTouchWithRemoteConnectivity_ShouldKeepTheFrontPatchesSeparate()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var first = Piece(new(-2, -2), new(-Fixed64.Half, -Fixed64.Two), new(0, 0), new(-2, 0));
        var second = Piece(new(Fixed64.Half, -Fixed64.Two), new(2, -2), new(2, 0), new(0, 0));
        var bridge = Rectangle(scene, -2, 2, -4, -2, Fixed64.Two);
        var members = new[] { first, second, bridge };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 3, 1) };
        scene.CreateBody(mover, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity);
        var regions = new HashSet<int>(); bool a = false, b = false;
        foreach (var sample in Rows(mover, members))
        {
            regions.Add(sample.Region); Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            a |= sample.Supplier == first.Id + 1; b |= sample.Supplier == second.Id + 1;
        }
        Assert.True(a && b); Assert.Equal(2, regions.Count);

        LSCollider2D Piece(params Vector2d[] points)
        {
            var polygon = new LSPolygonCollider2D(points) { MixedHalfThicknessOverride = Fixed64.Two };
            polygon.InitializeWithNoBody(new TestMatterAgent(scene.Context)); return polygon;
        }
    }

    [Fact]
    public void CylinderDiagonalSupportFamily_ShouldNotConnectThroughADisjointWindowCorner()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var wall = Rectangle(scene, -2, 2, -2, 0, Fixed64.Two);
        var lowerLeft = Rectangle(scene, -2, 0, 0, 2, Fixed64.One, -Fixed64.One);
        var upperRight = Rectangle(scene, 0, 2, 0, 2, Fixed64.One, Fixed64.One);
        var members = new[] { wall, lowerLeft, upperRight };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 6, 1) };
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)45);
        scene.CreateBody(mover, new(Fixed64.Zero, Fixed64.One, Fixed64.Quarter), rotation);
        var regions = new HashSet<int>(); bool upper = false, lower = false;
        foreach (var sample in Rows(mover, members))
        {
            if (sample.Supplier != wall.Id + 1) continue;
            regions.Add(sample.Region); Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
            upper |= point.X <= Fixed64.Zero && point.Y > Fixed64.Zero;
            lower |= point.X > Fixed64.Zero && point.Y <= Fixed64.Zero;
        }
        // The axial support follows X+Y approximately 1. It intersects both
        // half-windows but the shared source corner (0,0,0) lies outside the body.
        Assert.True(upper && lower); Assert.Equal(2, regions.Count);
    }
    [Fact]
    public void CylinderDeepOpposedMemberPlacement_ShouldKeepFiniteProvenanceAndRepeatableLocalResponse()
    {
        Assert.Equal(Trace(), Trace());

        List<(Vector3d Position, Vector3d Velocity, FixedQuaternion Rotation, int Entered, int Exited)> Trace()
        {
            using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
            var members = ClippedCapSources(scene);
            using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
            int entered = 0, exited = 0;
            foreach (var member in members)
            { member.OnMixedContactEnter += _ => entered++; member.OnMixedContactExit += _ => exited++; }
            var mover = new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 6, 1), Material = Gravitas.Materials.PhysicsMaterial.Frictionless };
            FixedQuaternion horizontal = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
            SolidBody body = scene.CreateBody(mover, Vector3d.Zero, horizontal).Body;
            body.FreezeAxes = BodyFreezeAxes3D.Rotation; body.SleepEnabled = false; body.UseManualGrounding();
            bool forward = false, backward = false;
            var zRegions = new HashSet<int>(); var zSuppliers = new HashSet<int>(); var rowDetails = new List<string>();
            foreach (var sample in Rows(mover, members))
            {
                Assert.True(sample.Row.Depth > Fixed64.Zero);
                Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out _));
                Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
                var supplier = System.Array.Find(members, member => member.Id + 1 == sample.Supplier);
                Assert.NotNull(supplier); Assert.True(supplier.ContainsPoint(new(source.X, source.Z)));
                Assert.True(FixedMath.Abs(source.Y - supplier.MixedSlabCenterY) <= supplier.MixedHalfThickness);
                rowDetails.Add($"supplier={sample.Supplier},region={sample.Region},normal={sample.Row.Normal},depth={sample.Row.Depth},source={source}");
                if (sample.Row.Normal == Vector3d.Forward)
                { forward = true; zRegions.Add(sample.Region); zSuppliers.Add(sample.Supplier); }
                else if (sample.Row.Normal == Vector3d.Backward)
                { backward = true; zRegions.Add(sample.Region); zSuppliers.Add(sample.Supplier); }
            }
            Assert.True(forward && backward, string.Join("; ", rowDetails));
            Assert.True(zRegions.Count >= 2 && zSuppliers.Count >= 2, string.Join("; ", rowDetails));
            var trace = new List<(Vector3d, Vector3d, FixedQuaternion, int, int)>();
            for (int frame = 0; frame < 32; frame++)
            {
                scene.Context.Simulate(); scene.Context.LateSimulate();
                trace.Add((body.Position3d, body.LinearVelocity, body.Rotation, entered, exited));
            }
            Assert.True(entered > 0);
            return trace;
        }
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CylinderComplementaryHeightCuts_ShouldNotTreatAPartialParentAsTheWholeLine(bool positiveX)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var wall = Rectangle(scene, -2, 2, -2, 0, Fixed64.Two);
        var lowerLeft = Rectangle(scene, -2, 0, 0, 2, Fixed64.One, -Fixed64.One);
        var upperRight = Rectangle(scene, 0, 2, 0, 2, Fixed64.One, Fixed64.One);
        var members = new[] { wall, lowerLeft, upperRight };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 x = positiveX ? Fixed64.Quarter : -Fixed64.Quarter;
        var mover = new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) };
        scene.CreateBody(mover, new(x, Fixed64.Zero, Fixed64.Quarter), FixedQuaternion.Identity);
        bool support = false;
        foreach (var sample in Rows(mover, members))
        {
            if (sample.Supplier != wall.Id + 1) continue;
            support = true; Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(x, source.X); Assert.Equal(Fixed64.Zero, source.Z);
            Assert.True(positiveX ? source.Y <= Fixed64.Zero : source.Y >= Fixed64.Zero);
        }
        // Complementary halfspans meet at Y=0 and coalesce into one full
        // horizontal line. Neither half-face certificate contains that line;
        // its exact tied MAX must compete with the valid partial native face.
        Assert.True(support);
    }
    [Fact]
    public void CylinderClippedCapIsolation_ShouldResolveAllOrdinaryPenetrations()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = ClippedCapSources(scene, wide: true);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 6, 1), Material = Gravitas.Materials.PhysicsMaterial.Frictionless };
        FixedQuaternion horizontal = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        SolidBody body = scene.CreateBody(mover, Vector3d.Zero, horizontal).Body;
        body.FreezeAxes = BodyFreezeAxes3D.Rotation; body.SleepEnabled = false; body.UseManualGrounding();
        for (int frame = 0; frame < 32; frame++) { scene.Context.Simulate(); scene.Context.LateSimulate(); }
        // Clone only after the final step, at the authoritative body pose.
        // This oracle does not depend on the moving collider's refresh timing.
        var probe = new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 6, 1) };
        probe.InitializeWithNoBody(new TestMatterAgent(scene.Context, new FixedTransform(body.Position3d, body.Rotation, Vector3d.One)));
        try
        {
            Fixed64 limit = CollisionResponseMixed.PenetrationSlop + Fixed64.FromRaw(8);
            foreach (var member in members)
                if (CollisionDetectionMixed.TryCollide(probe, member, out var contact))
                    Assert.True(contact.Depth <= limit,
                        $"member={member.Id},depth={contact.Depth},position={body.Position3d},normal={contact.Normal3DTo2D}");
        }
        finally { probe.Deactivate(); }
    }

    [Fact]
    public void CylinderZeroDepthCutRegions_ShouldKeepTheirExactTangentRims()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var source = Rectangle(scene, -2, 0, -2, 2, Fixed64.Two);
        var middle = Rectangle(scene, 0, 2, -1, 1, Fixed64.Two);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { source, middle });
        var mover = new LSCylinderCollider { Radius = 5 * Fixed64.Quarter, Size = new(5 * Fixed64.Half, (Fixed64)6, 5 * Fixed64.Half) };
        scene.CreateBody(mover, Vector3d.Right * (3 * Fixed64.Quarter), FixedQuaternion.Identity);
        var regions = new HashSet<int>(); bool lower = false, upper = false;
        foreach (var sample in Rows(mover, source, middle))
        {
            if (sample.Supplier != source.Id + 1) continue;
            regions.Add(sample.Region); Assert.Equal(Vector3d.Left, sample.Row.Normal); Assert.Equal(Fixed64.Zero, sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point));
            Assert.Equal(moving, point); Assert.Equal(Fixed64.Zero, point.X); Assert.Equal(Fixed64.One, FixedMath.Abs(point.Z));
            lower |= point.Z == -Fixed64.One; upper |= point.Z == Fixed64.One;
        }
        // Both tangent points lie on exact 3-4-5 radial sections. The middle
        // face is covered, so these two zero rows remain separate real regions.
        Assert.True(lower && upper); Assert.Equal(2, regions.Count);
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedDeepVerticalOverlap_ShouldGatherItsPhysicalNativeExit(bool cone)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, 0, -4, 4, Fixed64.Two), Rectangle(scene, 0, 4, -4, 4, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = cone ? new LSConeCollider { Radius = Fixed64.Half, Size = new(1, 6, 1) }
            : new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 6, 1) };
        scene.CreateBody(mover, Vector3d.Zero, FixedQuaternion.Identity);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        foreach (var sample in rows)
        {
            Assert.True(sample.Row.Normal == Vector3d.Up || sample.Row.Normal == Vector3d.Down);
            Assert.Equal((Fixed64)5, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Two, FixedMath.Abs(source.Y));
        }
        // The five-unit physical cap exit exceeds the initial proxy reach;
        // gathering must expand without discarding its original finite source.
        var first = rows[0].Row;
        AssertWholeShapeExit(mover, members, -first.Normal * (Fixed64)5, -first.Normal);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CylinderClippedCapLines_ShouldConnectWithinAndAcrossFiniteSourcePatches(bool above)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        Fixed64 quarter = Fixed64.Quarter;
        var members = ClippedCapSources(scene); var first = members[0]; var second = members[1];
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 6, 1) };
        FixedQuaternion horizontal = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        scene.CreateBody(mover, Vector3d.Up * (above ? Fixed64.Half : -Fixed64.Half), horizontal);
        var regions = new HashSet<int>(); int count = 0;
        Fixed64 expected = FixedMath.Sqrt((Fixed64)3) / (Fixed64)4 - quarter;
        foreach (var sample in Rows(mover, members))
        {
            if (sample.Supplier != first.Id + 1 && sample.Supplier != second.Id + 1) continue;
            regions.Add(sample.Region); count++;
            Assert.Equal(above ? Vector3d.Down : Vector3d.Up, sample.Row.Normal);
            Assert.InRange(sample.Row.Depth.m_rawValue, expected.m_rawValue - 4, expected.m_rawValue + 4);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(above ? quarter : -quarter, source.Y); Assert.Equal(-quarter, source.Z);
        }
        // The full cap support is at Z=0, outside both small slabs. Their
        // front and side line maxima still belong to one convex cap section,
        // joined through the actually intersected finite wall at X=0.
        Assert.True(count >= 2); Assert.Single(regions);

    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedTangentCornerCuts_ShouldNotEraseThePenetratingExposedFaceSupports(bool cone)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, 0, -2, 2, Fixed64.Two), Rectangle(scene, 0, 2, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 radius = Fixed64.FromFraction(5, 8), x = 3 * Fixed64.FromFraction(1, 8);
        Vector3d size = new(radius * Fixed64.Two, Fixed64.Two, radius * Fixed64.Two);
        LSCollider mover = cone ? new LSConeCollider { Radius = radius, Size = size }
            : new LSCylinderCollider { Radius = radius, Size = size };
        mover.Material = Gravitas.Materials.PhysicsMaterial.Frictionless;
        SolidBody body = scene.CreateBody(mover, new(x, Fixed64.Zero, Fixed64.Half), FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.Rotation; body.SleepEnabled = false; body.UseManualGrounding();
        bool left = false, back = false;
        foreach (var sample in Rows(mover, members))
        {
            if (sample.Row.Normal == Vector3d.Left)
            { left = true; Assert.Equal(Fixed64.Quarter, sample.Row.Depth); }
            else
            { Assert.Equal(Vector3d.Backward, sample.Row.Normal); back = true; Assert.Equal(Fixed64.FromFraction(1, 8), sample.Row.Depth); }
        }
        // The corner lies on an exact 3-4-5 radial section. Its zero-depth
        // subset lines cannot replace the penetrating parent face supports.
        Assert.True(left && back);
        for (int frame = 0; frame < 16; frame++) { scene.Context.Simulate(); scene.Context.LateSimulate(); }
        Fixed64 allowance = CollisionResponseMixed.PenetrationSlop + Fixed64.FromRaw(4);
        Assert.True(body.Position3d.X >= radius - allowance); Assert.True(body.Position3d.Z >= radius - allowance);
        Assert.Equal(Fixed64.Zero, body.Position3d.Y);
    }

    [Fact]
    public void CylinderThreeSurroundingCutNormals_ShouldUseTheCompatibleNativeAlternative()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var source = Polygon(new(-2, -2), new(2, -2), new(0, 2));
        var bottom = Rectangle(scene, -1, 1, -4, -2, Fixed64.Half);
        var right = Polygon(new(3 * Fixed64.Half, -Fixed64.One), new(7 * Fixed64.Half, Fixed64.Zero),
            new(5 * Fixed64.Half, Fixed64.Two), new(Fixed64.Half, Fixed64.One));
        var left = Polygon(new(-Fixed64.Half, Fixed64.One), new(-5 * Fixed64.Half, Fixed64.Two),
            new(-7 * Fixed64.Half, Fixed64.Zero), new(-3 * Fixed64.Half, -Fixed64.One));
        var members = new[] { source, bottom, right, left };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCylinderCollider { Radius = 5 * Fixed64.Half, Size = new(5, 8, 5) };
        // A small Y offset makes the upper cap uniquely cheaper than the
        // admitted top vertex, while remaining deeper than every finite cut.
        scene.CreateBody(mover, Vector3d.Up * Fixed64.FromFraction(1, 16), FixedQuaternion.Identity);
        bool native = false;
        foreach (var sample in Rows(mover, members))
        {
            if (sample.Supplier != source.Id + 1) continue;
            Assert.Equal(Vector3d.Down, sample.Row.Normal); Assert.Equal(Fixed64.FromFraction(71, 16), sample.Row.Depth);
            native = true;
        }
        Assert.True(native);

        LSCollider2D Polygon(params Vector2d[] points)
        {
            var polygon = new LSPolygonCollider2D(points)
            { MixedHalfThicknessOverride = Fixed64.Half, Material = Gravitas.Materials.PhysicsMaterial.Frictionless };
            polygon.InitializeWithNoBody(new TestMatterAgent(scene.Context)); return polygon;
        }
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TallCylinderConcaveCorner_ShouldRetainBothShallowCutsInsteadOfTheDistantCap(bool asymmetric)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -2, 0, -2, 2, Fixed64.Two), Rectangle(scene, 0, 2, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 x = asymmetric ? Fixed64.FromFraction(1, 8) : Fixed64.Quarter;
        var mover = new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 6, 1) };
        scene.CreateBody(mover, new(-x, Fixed64.Zero, -Fixed64.Quarter), FixedQuaternion.Identity);
        bool left = false, back = false;
        foreach (var sample in Rows(mover, members))
        {
            Fixed64 expected;
            if (sample.Row.Normal == Vector3d.Left)
            { left = true; expected = x + FixedMath.Sqrt(Fixed64.Quarter - Fixed64.Quarter * Fixed64.Quarter); }
            else
            {
                Assert.Equal(Vector3d.Backward, sample.Row.Normal); back = true;
                expected = Fixed64.Quarter + FixedMath.Sqrt(Fixed64.Quarter - x * x);
            }
            Assert.InRange(sample.Row.Depth.m_rawValue, expected.m_rawValue - 4, expected.m_rawValue + 4);
        }
        // Caps at Y=+/-2 intersect this body, but their five-unit whole-source
        // exit must not hide either independently required lateral cut.
        Assert.True(left && back);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void CylinderNativeAlternative_ShouldCompeteWithTheCompleteAsymmetricSourceCutSet(int heightCase)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var source = Rectangle(scene, -2, 2, -2, 2, Fixed64.Half);
        var right = Rectangle(scene, 2, 4, 0, 2, Fixed64.Half);
        var front = Rectangle(scene, 0, 2, 2, 4, Fixed64.Half);
        var members = new[] { source, right, front };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 height = heightCase == 2 ? (Fixed64)6 : heightCase == 1 ? (Fixed64)5 : 9 * Fixed64.Half;
        var mover = new LSCylinderCollider { Radius = 5 * Fixed64.Half, Size = new((Fixed64)5, height, (Fixed64)5) };
        scene.CreateBody(mover, new(Fixed64.Half, Fixed64.Zero, Fixed64.Two), FixedQuaternion.Identity);
        bool left = false, back = false, cap = false;
        foreach (var sample in Rows(mover, members))
        {
            if (sample.Supplier != source.Id + 1) continue;
            if (sample.Row.Normal == Vector3d.Left)
            { left = true; Assert.Equal((Fixed64)3, sample.Row.Depth); }
            else if (sample.Row.Normal == Vector3d.Backward)
            {
                back = true; Fixed64 expected = FixedMath.Sqrt((Fixed64)6);
                Assert.InRange(sample.Row.Depth.m_rawValue, expected.m_rawValue - 4, expected.m_rawValue + 4);
            }
            else if (sample.Row.Normal == Vector3d.Up || sample.Row.Normal == Vector3d.Down)
            { cap = true; Assert.Equal(height * Fixed64.Half + Fixed64.Half, sample.Row.Depth); }
        }
        // The original source's two independent cuts have depths sqrt(6) and
        // 3. Native 2.75 and the exact tie at 3 beat the set; native 3.5 loses.
        // Other sources may borrow this source's cap witnesses independently.
        if (heightCase == 2) Assert.True(left && back);
        else { Assert.False(left || back); Assert.True(cap); }
    }

    [Fact]
    public void CylinderZeroAndPositiveOpposedCuts_ShouldKeepTheWholeSourceExit()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        // Start with the left edge so its exact zero-depth pole precedes the
        // opposite positive 3-4-5 cut in the source's boundary inventory.
        var source = new LSPolygonCollider2D(new(-2, 6), new(-2, -6), new(2, -6), new(2, 6))
        { MixedHalfThicknessOverride = Fixed64.Half };
        source.InitializeWithNoBody(new TestMatterAgent(scene.Context));
        var left = Rectangle(scene, -4, -2, -6, 5, Fixed64.Two);
        var right = Rectangle(scene, 2, 4, -6, 3, Fixed64.Two);
        var members = new LSCollider2D[] { source, left, right };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCylinderCollider { Radius = (Fixed64)5, Size = new(10, 20, 10) };
        scene.CreateBody(mover, Vector3d.Left * Fixed64.Two, FixedQuaternion.Identity);
        bool cap = false;
        foreach (var sample in Rows(mover, members))
        {
            if (sample.Supplier != source.Id + 1) continue;
            Assert.True(sample.Row.Normal == Vector3d.Up || sample.Row.Normal == Vector3d.Down);
            Assert.Equal(21 * Fixed64.Half, sample.Row.Depth); cap = true;
        }
        // A zero row still requires nonnegative translation on its side. It
        // cannot coexist with an opposed positive row; the native cap is valid.
        Assert.True(cap);
    }

    [Fact]
    public void CylinderOpposedSourceCuts_ShouldRetainTheFeasibleWholeSourceExit()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var source = Rectangle(scene, -2, 2, -2, 2, Fixed64.Half);
        var members = new[] { source, Rectangle(scene, -4, -2, 0, 2, Fixed64.Half),
            Rectangle(scene, 2, 4, 0, 2, Fixed64.Half), Rectangle(scene, -2, 2, 2, 4, Fixed64.Half) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCylinderCollider { Radius = 5 * Fixed64.Half, Size = new(5, 8, 5) };
        scene.CreateBody(mover, new(Fixed64.Zero, Fixed64.Zero, 3 * Fixed64.Half), FixedQuaternion.Identity);
        bool cap = false;
        foreach (var sample in Rows(mover, members))
        {
            if (sample.Supplier != source.Id + 1) continue;
            Assert.True(sample.Row.Normal == Vector3d.Up || sample.Row.Normal == Vector3d.Down);
            Assert.Equal(9 * Fixed64.Half, sample.Row.Depth); cap = true;
        }
        // This complete equal-height union has a known Y exit. Opposed cuts
        // +/-X cannot be satisfied together by translating a rigid body.
        Assert.True(cap);
        AssertWholeShapeExit(mover, members, Vector3d.Up * (9 * Fixed64.Half) + Vector3d.Forward * (3 * Fixed64.Half), Vector3d.Up);
    }
    [Fact]
    public void CylinderUnequalHeightFrontWall_ShouldNotPushTheBodyVertically()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -2, 0, -2, 0, (Fixed64)3);
        var right = Rectangle(scene, 0, 2, -2, 0, Fixed64.One, 3 * Fixed64.Half);
        var remote = Rectangle(scene, -2, 0, -4, -2, Fixed64.One, -Fixed64.Two);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right, remote });
        var mover = new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 6, 1), Material = Gravitas.Materials.PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(mover, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity).Body;
        body.FreezeAxes = BodyFreezeAxes3D.Rotation; body.SleepEnabled = false; body.UseManualGrounding();
        for (int frame = 0; frame < 16; frame++)
        {
            body.AddLinearImpulse(Vector3d.Backward * Fixed64.FromFraction(1, 16));
            scene.Context.Simulate(); scene.Context.LateSimulate();
            string state = $"frame={frame},position={body.Position3d},velocity={body.LinearVelocity}";
            Assert.True(body.Position3d.Y == Fixed64.Zero, state);
            Assert.True(body.LinearVelocity.Y == Fixed64.Zero, state);
        }
        Assert.True(body.Position3d.Z > Fixed64.Quarter);
    }
    [Fact]
    public void CylinderUnequalHeightWalls_ShouldJoinOnlyThroughTheirActualSharedBand()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -2, 0, -2, 0, (Fixed64)3);
        var right = Rectangle(scene, 0, 2, -2, 0, Fixed64.One, 3 * Fixed64.Half);
        // A body-disjoint back piece adds lower bands before the genuine
        // shared front-wall height [1/2,5/2], without changing its continuity.
        var remote = Rectangle(scene, -2, 0, -4, -2, Fixed64.One, -Fixed64.Two);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right, remote });
        var mover = new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 6, 1) };
        scene.CreateBody(mover, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity);
        var rows = Rows(mover, left, right, remote); Assert.NotEmpty(rows);
        var regions = new HashSet<int>();
        foreach (var sample in rows)
        {
            regions.Add(sample.Region);
            Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.X); Assert.Equal(Fixed64.Zero, source.Z);
            Assert.NotEqual(remote.Id + 1, sample.Supplier);
        }
        Assert.Single(regions);
    }
    [Fact]
    public void CylinderFreeAxialFamily_ShouldKeepDisconnectedCoplanarWallComponentsSeparate()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -4, -3, -2, 0, Fixed64.Two), Rectangle(scene, -3, -2, -2, 0, Fixed64.Two),
            Rectangle(scene, 2, 3, -2, 0, Fixed64.Two), Rectangle(scene, 3, 4, -2, 0, Fixed64.Two) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        var mover = new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 6, 1) };
        FixedQuaternion horizontal = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        scene.CreateBody(mover, Vector3d.Forward * Fixed64.Quarter, horizontal);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        var regions = new HashSet<int>(); bool left = false, right = false;
        foreach (var sample in rows)
        {
            regions.Add(sample.Region);
            Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.Z);
            Assert.True(source.X <= -(Fixed64)2 || source.X >= (Fixed64)2);
            left |= source.X <= -(Fixed64)2; right |= source.X >= (Fixed64)2;
        }
        // Coplanarity and one moving support family cannot connect terrain
        // pieces across an empty gap in the explicitly declared surface.
        Assert.True(left && right); Assert.Equal(2, regions.Count);
    }
    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void CurvedContainedAcrossTheSeam_ShouldChooseTheNearestRetainedOuterBoundary(bool cone, bool offCenter)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new[] { Rectangle(scene, -10, 0, -10, 10, (Fixed64)10), Rectangle(scene, 0, 10, -10, 10, (Fixed64)10) };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        Fixed64 x = offCenter ? Fixed64.Quarter : Fixed64.Zero;
        LSCollider mover = CurvedMover(scene, cone, Vector3d.Right * x);
        var rows = Rows(mover, members); Assert.NotEmpty(rows);
        Fixed64 expectedDepth = (Fixed64)10 + Fixed64.Half - x;
        foreach (var sample in rows)
        {
            Assert.Equal(expectedDepth, sample.Row.Depth); Assert.False(sample.Row.DepthIsClamped);
            if (offCenter) Assert.Equal(Vector3d.Left, sample.Row.Normal);
            Assert.Equal(Fixed64.Zero, sample.Row.Normal.Y);
            Assert.Equal(Fixed64.One, FixedMath.Abs(sample.Row.Normal.X) + FixedMath.Abs(sample.Row.Normal.Z));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(-(Fixed64)10, Vector3d.Dot(source, sample.Row.Normal));
        }
        // This convex union has an independently known complete exit; the
        // joined member seam at X=0 must never become the recovery surface.
        var first = rows[0].Row;
        AssertWholeShapeExit(mover, members, Vector3d.Right * x - first.Normal * first.Depth, -first.Normal);
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedCoplanarFrontWalls_ShouldConnectThroughTheirFiniteSharedEdge(bool cone)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -2, 0, -2, 0, Fixed64.Two);
        var right = Rectangle(scene, 0, 2, -2, 0, Fixed64.Two);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right });
        LSCollider mover = CurvedMover(scene, cone, Vector3d.Forward * Fixed64.Quarter);
        var rows = Rows(mover, left, right); Assert.NotEmpty(rows);
        var regions = new HashSet<int>();
        foreach (var sample in rows)
        {
            regions.Add(sample.Region);
            Assert.Equal(Vector3d.Backward, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.X); Assert.Equal(Fixed64.Zero, source.Z);
            Assert.InRange(source.Y.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
        }
        // The boundary sources meet only on X=Z=0. This exact shared vertical
        // stratum intersects the body and joins their two face certificates.
        Assert.Single(regions);
    }
    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void CurvedCapSections_ShouldConnectOnlyAcrossAnAdmittedSharedWall(bool cone, bool ambiguous)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -2, 0, -2, 2, Fixed64.Two);
        var right = Rectangle(scene, 0, 2, -2, 2, Fixed64.Two);
        LSCollider2D[] members = ambiguous
            ? new[] { left, right, Rectangle(scene, -1, 1, -1, 1, Fixed64.Two) } : new[] { left, right };
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members);
        LSCollider mover = CurvedMover(scene, cone, new(Fixed64.Zero, 11 * Fixed64.Quarter, Fixed64.Zero));
        var rows = Rows(mover, left, right); Assert.NotEmpty(rows);
        var regions = new HashSet<int>();
        foreach (var sample in rows)
        {
            regions.Add(sample.Region);
            Assert.Equal(Vector3d.Down, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorA.TryGetWorldPoint(out Vector3d moving));
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(7 * Fixed64.Quarter, moving.Y); Assert.Equal(Fixed64.Two, source.Y);
            Assert.True(source.X * source.X + source.Z * source.Z <= Fixed64.Quarter);
        }
        // The ambiguous middle overlap hides the declared join within the
        // whole contact disk. A join elsewhere in the component cannot merge it.
        Assert.Equal(ambiguous ? 2 : 1, regions.Count);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedConnectedCapTiles_ShouldReduceThePhysicalRegionToFourSamples(bool cone)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var members = new List<LSCollider2D>();
        for (int x = -3; x < 3; x += 2)
            for (int z = -3; z < 3; z += 2) members.Add(Rectangle(scene, x, x + 2, z, z + 2, Fixed64.Two));
        using var surface = scene.Context.Physics2D.CreateStaticSurface(members.ToArray());
        LSCollider mover = cone ? new LSConeCollider { Radius = Fixed64.Two, Size = new(4, 2, 4) }
            : new LSCylinderCollider { Radius = Fixed64.Two, Size = new(4, 2, 4) };
        scene.CreateBody(mover, new(Fixed64.Zero, 11 * Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity);
        var rows = Rows(mover, members.ToArray()); Assert.InRange(rows.Count, 1, ContactManifold.MaxContactsPerGroup);
        var regions = new HashSet<int>();
        foreach (var sample in rows)
        {
            regions.Add(sample.Region); Assert.Equal(Vector3d.Down, sample.Row.Normal); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source)); Assert.Equal(Fixed64.Two, source.Y);
            LSCollider2D? supplier = members.Find(member => member.Id + 1 == sample.Supplier);
            Assert.NotNull(supplier); Assert.True(supplier.ContainsPoint(new(source.X, source.Z)));
        }
        Assert.Single(regions);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CurvedCapBoundaryLine_ShouldPreserveTheClippedDeclaredJoin(bool cone)
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var left = Rectangle(scene, -2, 0, -2, 0, Fixed64.One);
        var right = Rectangle(scene, 0, 2, -2, 0, Fixed64.One);
        var taller = Rectangle(scene, -2, 0, 0, 2, Fixed64.Two);
        var ambiguous = Rectangle(scene, 0, 1, -1, 0, Fixed64.One);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { left, right, taller, ambiguous });
        LSCollider mover = CurvedMover(scene, cone, new(Fixed64.Quarter, 7 * Fixed64.Quarter, -Fixed64.Quarter));
        var regions = new HashSet<int>(); bool fromLeft = false, fromRight = false;
        foreach (var sample in Rows(mover, left, right, taller))
        {
            if (sample.Row.Normal != Vector3d.Down || sample.Supplier != left.Id + 1 && sample.Supplier != right.Id + 1) continue;
            regions.Add(sample.Region); fromLeft |= sample.Supplier == left.Id + 1; fromRight |= sample.Supplier == right.Id + 1;
            Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d point)); Assert.Equal(Fixed64.One, point.Y);
        }
        // The taller front neighbor supplies a left-cap boundary line at Z=0.
        // Left/right continuity exists only at X=0,Z=[-2,-1], outside this body;
        // the nearer shared side is ambiguous. Mapping that retained join must
        // not clear its clip and join the cap patches through the front corner.
        Assert.True(fromLeft && fromRight); Assert.Equal(2, regions.Count);
    }

    [Fact]
    public void CylinderFreeAxialFamily_ShouldRetainDisconnectedExposedHeightBands()
    {
        using var scene = PhysicsScenarioBuilder.Create(); Configure(scene.Context);
        var wall = Rectangle(scene, -2, 2, -2, 0, (Fixed64)4);
        var tab = Rectangle(scene, -1, 1, 0, 2, Fixed64.One);
        using var surface = scene.Context.Physics2D.CreateStaticSurface(new[] { wall, tab });
        var mover = new LSCylinderCollider { Radius = Fixed64.Half, Size = new(1, 6, 1) };
        scene.CreateBody(mover, Vector3d.Forward * Fixed64.Quarter, FixedQuaternion.Identity);
        var regions = new HashSet<int>(); bool upper = false, lower = false;
        foreach (var sample in Rows(mover, wall, tab))
        {
            if (sample.Supplier != wall.Id + 1 || sample.Row.Normal != Vector3d.Backward) continue;
            regions.Add(sample.Region); Assert.Equal(Fixed64.Quarter, sample.Row.Depth);
            Assert.True(sample.Row.AnchorB.TryGetWorldPoint(out Vector3d source));
            Assert.Equal(Fixed64.Zero, source.X); Assert.Equal(Fixed64.Zero, source.Z);
            Assert.True(source.Y <= -Fixed64.One || source.Y >= Fixed64.One);
            upper |= source.Y >= Fixed64.One; lower |= source.Y <= -Fixed64.One;
        }
        Assert.True(upper && lower); Assert.Equal(2, regions.Count);
    }
    private static LSCollider2D[] ClippedCapSources(PhysicsScenarioBuilder scene, bool wide = false)
    {
        Fixed64 quarter = Fixed64.Quarter;
        if (wide)
            return new[] { Prism(-(Fixed64)4, Fixed64.Zero, -Fixed64.Two, -quarter, quarter),
                Prism(Fixed64.Zero, (Fixed64)4, -Fixed64.Two, -quarter, quarter),
                Prism(-(Fixed64)4, (Fixed64)4, -quarter, Fixed64.Two, Fixed64.Two) };
        return new[] { Prism(-Fixed64.One, Fixed64.Zero, -Fixed64.Two, -quarter, quarter),
            Prism(Fixed64.Zero, Fixed64.One, -Fixed64.Two, -quarter, quarter),
            Prism(-Fixed64.One, Fixed64.One, -quarter, Fixed64.Two, Fixed64.Two),
            Prism(-(Fixed64)3, -Fixed64.One, -Fixed64.Two, -quarter, Fixed64.Two),
            Prism(Fixed64.One, (Fixed64)3, -Fixed64.Two, -quarter, Fixed64.Two),
            Prism(-Fixed64.One, Fixed64.Zero, -(Fixed64)4, -Fixed64.Two, Fixed64.FromFraction(1, 8)) };

        LSCollider2D Prism(Fixed64 minX, Fixed64 maxX, Fixed64 minZ, Fixed64 maxZ, Fixed64 half)
        {
            var polygon = new LSPolygonCollider2D(new(minX, minZ), new(maxX, minZ), new(maxX, maxZ), new(minX, maxZ))
            { MixedHalfThicknessOverride = half, Material = Gravitas.Materials.PhysicsMaterial.Frictionless };
            polygon.InitializeWithNoBody(new TestMatterAgent(scene.Context)); return polygon;
        }
    }
}
