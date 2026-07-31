using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using SwiftCollections;
using SwiftCollections.Query;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class DynamicCcdCandidateIndexTests
{
    [Fact]
    public void Query_ShouldIncludeTargetsWhoseSweptBoundsOverlapAfterMovement()
    {
        var index = new DynamicCcdCandidateIndex(4);
        index.Add(
            dynamicId: 7,
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(
                new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.Zero),
                -Vector3d.Right * (Fixed64)5,
                Fixed64.Half));
        index.Add(
            dynamicId: 11,
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(
                new Vector3d((Fixed64)5, Fixed64.Zero, (Fixed64)16),
                -Vector3d.Right * (Fixed64)5,
                Fixed64.Half));
        index.Sort();

        var results = new SwiftList<int>(4);
        index.Query(
            DynamicCcdCandidateIndex.CreateSweptSphereBounds(
                new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
                Vector3d.Right * (Fixed64)5,
                Fixed64.Half),
            results);

        results.Count.Should().Be(1);
        results[0].Should().Be(7);
    }

    [Fact]
    public void Query_ShouldUseStableBoundsThenDynamicIdOrdering()
    {
        var index = new DynamicCcdCandidateIndex(4);
        index.Add(30, CreateBounds(Fixed64.One, Fixed64.Zero));
        index.Add(10, CreateBounds(Fixed64.One, Fixed64.Zero));
        index.Add(20, CreateBounds(-Fixed64.One, Fixed64.Zero));
        index.Sort();

        var results = new SwiftList<int>(4);
        index.Query(new FixedBoundVolume(new Vector3d((Fixed64)(-4), -Fixed64.One, -Fixed64.One), new Vector3d((Fixed64)4, Fixed64.One, Fixed64.One)), results);

        results.Count.Should().Be(3);
        results[0].Should().Be(20);
        results[1].Should().Be(10);
        results[2].Should().Be(30);
    }

    [Fact]
    public void Query_ShouldUseFullBoundsTupleOrdering()
    {
        var index = new DynamicCcdCandidateIndex(8);
        index.Add(70, new FixedBoundVolume(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero), new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One)));
        index.Add(60, new FixedBoundVolume(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero), new Vector3d(Fixed64.One, Fixed64.One, (Fixed64)2)));
        index.Add(50, new FixedBoundVolume(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero), new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One)));
        index.Add(40, new FixedBoundVolume(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero), new Vector3d((Fixed64)2, Fixed64.One, Fixed64.One)));
        index.Add(30, new FixedBoundVolume(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half), new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One)));
        index.Add(20, new FixedBoundVolume(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero), new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One)));
        index.Add(10, new FixedBoundVolume(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero), new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One)));
        index.Sort();

        var results = new SwiftList<int>(8);
        index.Query(
            new FixedBoundVolume(
                new Vector3d(-Fixed64.One, -Fixed64.One, -Fixed64.One),
                new Vector3d((Fixed64)3, (Fixed64)3, (Fixed64)3)),
            results);

        results.Should().Equal(10, 70, 60, 50, 40, 30, 20);
    }

    [Fact]
    public void Query2D_ShouldIncludeTargetsWhoseSweptBoundsOverlapAfterMovement()
    {
        var index = new DynamicCcdCandidateIndex2D(4);
        index.Add(
            dynamicId: 7,
            DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
                new Vector2d((Fixed64)5, Fixed64.Zero),
                -Vector2d.Right * (Fixed64)5,
                Fixed64.Half));
        index.Add(
            dynamicId: 11,
            DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
                new Vector2d((Fixed64)5, (Fixed64)16),
                -Vector2d.Right * (Fixed64)5,
                Fixed64.Half));
        index.Sort();

        var results = new SwiftList<int>(4);
        index.Query(
            DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
                new Vector2d((Fixed64)(-5), Fixed64.Zero),
                Vector2d.Right * (Fixed64)5,
                Fixed64.Half),
            results);

        results.Count.Should().Be(1);
        results[0].Should().Be(7);
    }

    [Fact]
    public void Query2D_ShouldUseStableBoundsThenDynamicIdOrdering()
    {
        var index = new DynamicCcdCandidateIndex2D(4);
        index.Add(30, CreatePlanarBounds(Fixed64.One, Fixed64.Zero));
        index.Add(10, CreatePlanarBounds(Fixed64.One, Fixed64.Zero));
        index.Add(20, CreatePlanarBounds(-Fixed64.One, Fixed64.Zero));
        index.Sort();

        var results = new SwiftList<int>(4);
        index.Query(new DynamicCcdPlanarBounds((Fixed64)(-4), -Fixed64.One, (Fixed64)4, Fixed64.One), results);

        results.Count.Should().Be(3);
        results[0].Should().Be(20);
        results[1].Should().Be(10);
        results[2].Should().Be(30);
    }

    [Fact]
    public void Query2D_ShouldUseFullBoundsTupleOrdering()
    {
        var index = new DynamicCcdCandidateIndex2D(6);
        index.Add(50, new DynamicCcdPlanarBounds(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One));
        index.Add(40, new DynamicCcdPlanarBounds(Fixed64.Zero, Fixed64.Zero, Fixed64.One, (Fixed64)2));
        index.Add(30, new DynamicCcdPlanarBounds(Fixed64.Zero, Fixed64.Zero, (Fixed64)2, Fixed64.One));
        index.Add(20, new DynamicCcdPlanarBounds(Fixed64.Zero, Fixed64.Half, Fixed64.One, Fixed64.One));
        index.Add(10, new DynamicCcdPlanarBounds(-Fixed64.Half, Fixed64.Zero, Fixed64.One, Fixed64.One));
        index.Sort();

        var results = new SwiftList<int>(6);
        index.Query(
            new DynamicCcdPlanarBounds(-Fixed64.One, -Fixed64.One, (Fixed64)3, (Fixed64)3),
            results);

        results.Should().Equal(10, 50, 40, 30, 20);
    }

    [Fact]
    public void AddOrUpdate_ShouldPreserveIdentityAcrossOrderedAndCrossingUpdates()
    {
        var index3D = new DynamicCcdCandidateIndex(2, supportsUpdates: true);
        index3D.Add(7, CreateBounds(Fixed64.Zero, Fixed64.Zero));
        index3D.Add(9, CreateBounds((Fixed64)4, Fixed64.Zero));
        index3D.Sort();
        var results3D = new SwiftList<int>(2);
        index3D.AddOrUpdate(7, CreateBounds(Fixed64.One, Fixed64.Zero));
        index3D.Query(CreateBounds(Fixed64.One, Fixed64.Zero), results3D);
        results3D.Should().Equal(7);
        index3D.AddOrUpdate(7, CreateBounds((Fixed64)8, Fixed64.Zero));
        index3D.Query(CreateBounds((Fixed64)8, Fixed64.Zero), results3D);
        results3D.Should().Equal(7);
        index3D.AddOrUpdate(
            7,
            new FixedBoundVolume(
                new Vector3d((Fixed64)6, -Fixed64.Half, -Fixed64.Half),
                new Vector3d((Fixed64)10, Fixed64.Half, Fixed64.Half)));
        index3D.Query(CreateBounds((Fixed64)8, Fixed64.Zero), results3D);
        results3D.Should().Equal(7);
        index3D.AddOrUpdate(7, CreateBounds(Fixed64.One, Fixed64.Zero));
        index3D.Query(CreateBounds(Fixed64.One, Fixed64.Zero), results3D);

        var index2D = new DynamicCcdCandidateIndex2D(2, supportsUpdates: true);
        index2D.Add(11, CreatePlanarBounds(Fixed64.Zero, Fixed64.Zero));
        index2D.Add(13, CreatePlanarBounds((Fixed64)4, Fixed64.Zero));
        index2D.Sort();
        var results2D = new SwiftList<int>(2);
        index2D.AddOrUpdate(11, CreatePlanarBounds(Fixed64.One, Fixed64.Zero));
        index2D.Query(CreatePlanarBounds(Fixed64.One, Fixed64.Zero), results2D);
        results2D.Should().Equal(11);
        index2D.AddOrUpdate(11, CreatePlanarBounds((Fixed64)8, Fixed64.Zero));
        index2D.Query(CreatePlanarBounds((Fixed64)8, Fixed64.Zero), results2D);
        results2D.Should().Equal(11);
        index2D.AddOrUpdate(
            11,
            new DynamicCcdPlanarBounds((Fixed64)6, -Fixed64.Half, (Fixed64)10, Fixed64.Half));
        index2D.Query(CreatePlanarBounds((Fixed64)8, Fixed64.Zero), results2D);
        results2D.Should().Equal(11);
        index2D.AddOrUpdate(11, CreatePlanarBounds(Fixed64.One, Fixed64.Zero));
        index2D.Query(CreatePlanarBounds(Fixed64.One, Fixed64.Zero), results2D);

        index3D.Count.Should().Be(2);
        results3D.Should().Equal(7);
        index2D.Count.Should().Be(2);
        results2D.Should().Equal(11);
    }

    [Fact]
    public void Remove_ShouldRepairMovedEntryIndicesAndRejectUnavailableIdentity()
    {
        var index3D = new DynamicCcdCandidateIndex(3, supportsUpdates: true);
        index3D.Add(1, CreateBounds(Fixed64.Zero, Fixed64.Zero));
        index3D.Add(2, CreateBounds(Fixed64.One, Fixed64.Zero));
        index3D.Add(3, CreateBounds(Fixed64.Two, Fixed64.Zero));
        index3D.Remove(1).Should().BeTrue();
        index3D.Remove(3).Should().BeTrue();
        index3D.Remove(9).Should().BeFalse();
        new DynamicCcdCandidateIndex(1).Remove(1).Should().BeFalse();

        var index2D = new DynamicCcdCandidateIndex2D(3, supportsUpdates: true);
        index2D.Add(1, CreatePlanarBounds(Fixed64.Zero, Fixed64.Zero));
        index2D.Add(2, CreatePlanarBounds(Fixed64.One, Fixed64.Zero));
        index2D.Add(3, CreatePlanarBounds(Fixed64.Two, Fixed64.Zero));
        index2D.Remove(1).Should().BeTrue();
        index2D.Remove(3).Should().BeTrue();
        index2D.Remove(9).Should().BeFalse();
        new DynamicCcdCandidateIndex2D(1).Remove(1).Should().BeFalse();

        var results3D = new SwiftList<int>(1);
        index3D.Query(CreateBounds(Fixed64.One, Fixed64.Zero), results3D);
        var results2D = new SwiftList<int>(1);
        index2D.Query(CreatePlanarBounds(Fixed64.One, Fixed64.Zero), results2D);
        results3D.Should().Equal(2);
        results2D.Should().Equal(2);
    }

    [Fact]
    public void Query_ShouldAdmitCandidatesCoveredOnlyByExtremeProxyRadii()
    {
        Fixed64 radius3D = new Vector3d((Fixed64)20000, (Fixed64)40000, (Fixed64)40000).Magnitude;
        var index3D = new DynamicCcdCandidateIndex(1);
        index3D.Add(7, DynamicCcdCandidateIndex.CreateSweptSphereBounds(Vector3d.Zero, Vector3d.Zero, radius3D));
        index3D.Sort();
        var results3D = new SwiftList<int>(1);
        index3D.Query(CreateBounds((Fixed64)55000, Fixed64.Zero), results3D);

        Fixed64 radius2D = new Vector2d((Fixed64)60000, (Fixed64)80000).Magnitude;
        var index2D = new DynamicCcdCandidateIndex2D(1);
        index2D.Add(11, DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(Vector2d.Zero, Vector2d.Zero, radius2D));
        index2D.Sort();
        var results2D = new SwiftList<int>(1);
        index2D.Query(CreatePlanarBounds((Fixed64)75000, Fixed64.Zero), results2D);

        results3D.Should().Equal(7);
        results2D.Should().Equal(11);
    }

    [Fact]
    public void Query_ShouldAdmitFullDomainSpansFromEitherEndpoint()
    {
        var updatedFullDomainBounds3D = new FixedBoundVolume(
            new Vector3d(Fixed64.MinValue + Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
        var endpointQuery3D = new FixedBoundVolume(
            new Vector3d(Fixed64.MaxValue - Fixed64.One, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
        var updatedFullDomainBounds2D = new DynamicCcdPlanarBounds(
            Fixed64.MinValue + Fixed64.One,
            Fixed64.Zero,
            Fixed64.MaxValue,
            Fixed64.Zero);
        var endpointQuery2D = new DynamicCcdPlanarBounds(
            Fixed64.MaxValue - Fixed64.One,
            Fixed64.Zero,
            Fixed64.MaxValue,
            Fixed64.Zero);

        var index3D = new DynamicCcdCandidateIndex(2, supportsUpdates: true);
        index3D.Add(5, CreateBounds(Fixed64.Zero, Fixed64.Zero));
        index3D.Add(
            7,
            new FixedBoundVolume(
                new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
                new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero)));
        index3D.AddOrUpdate(7, updatedFullDomainBounds3D);
        var results3D = new SwiftList<int>(1);

        var index2D = new DynamicCcdCandidateIndex2D(2, supportsUpdates: true);
        index2D.Add(9, CreatePlanarBounds(Fixed64.Zero, Fixed64.Zero));
        index2D.Add(
            11,
            new DynamicCcdPlanarBounds(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.MaxValue,
                Fixed64.Zero));
        index2D.AddOrUpdate(11, updatedFullDomainBounds2D);
        var results2D = new SwiftList<int>(1);

        index3D.Query(endpointQuery3D, results3D);
        index2D.Query(endpointQuery2D, results2D);

        results3D.Should().Equal(7);
        results2D.Should().Equal(11);

        index3D.AddOrUpdate(7, CreateBounds(Fixed64.Zero, Fixed64.Zero));
        index2D.AddOrUpdate(11, CreatePlanarBounds(Fixed64.Zero, Fixed64.Zero));
        index3D.Query(endpointQuery3D, results3D);
        index2D.Query(endpointQuery2D, results2D);
        results3D.Should().BeEmpty();
        results2D.Should().BeEmpty();

        index3D.AddOrUpdate(7, updatedFullDomainBounds3D);
        index2D.AddOrUpdate(11, updatedFullDomainBounds2D);
        index3D.Query(endpointQuery3D, results3D);
        index2D.Query(endpointQuery2D, results2D);
        results3D.Should().Equal(7);
        results2D.Should().Equal(11);

        index3D.Remove(7).Should().BeTrue();
        index2D.Remove(11).Should().BeTrue();
        index3D.Query(CreateBounds(Fixed64.Zero, Fixed64.Zero), results3D);
        index2D.Query(CreatePlanarBounds(Fixed64.Zero, Fixed64.Zero), results2D);

        results3D.Should().Equal(5);
        results2D.Should().Equal(9);
    }

    [Fact]
    public void MotionSegments_WithUnrepresentableEndpointDelta_ShouldPreserveEndpointsAndRefuseSeparationBound()
    {
        Vector3d start3D = new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero);
        Vector3d end3D = new(Fixed64.MaxValue, Fixed64.One, Fixed64.Zero);
        var segment3D = new ContinuousCollisionMotionSegment3D(
            Fixed64.Zero,
            Fixed64.One,
            start3D,
            end3D,
            end3D - start3D,
            FixedQuaternion.Identity,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Fixed64.Zero,
            ContinuousCollisionRotationPath3D.IntegratedAngularVelocity);
        Vector2d start2D = new(Fixed64.MinValue, Fixed64.Zero);
        Vector2d end2D = new(Fixed64.MaxValue, Fixed64.One);
        var segment2D = new ContinuousCollisionMotionSegment2D(
            Fixed64.Zero,
            Fixed64.One,
            start2D,
            end2D,
            end2D - start2D,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero);

        segment3D.SamplePosition(Fixed64.One).Should().Be(end3D);
        segment2D.SamplePosition(Fixed64.One).Should().Be(end2D);
        segment3D.TryResolveMotionBound(
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();
        segment2D.TryResolveMotionBound(
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.One,
                out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void CreateSweptBounds_ShouldPreserveAnisotropicExtents()
    {
        FixedBoundVolume bounds = DynamicCcdCandidateIndex.CreateSweptBounds(
            new Vector3d((Fixed64)(-2), Fixed64.One, (Fixed64)3),
            new Vector3d((Fixed64)4, (Fixed64)(-2), Fixed64.Two),
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Two));

        bounds.Min.Should().Be(new Vector3d(
            Fixed64.FromFraction(-5, 2),
            (Fixed64)(-2),
            Fixed64.One));
        bounds.Max.Should().Be(new Vector3d(
            Fixed64.FromFraction(5, 2),
            Fixed64.Two,
            (Fixed64)7));
    }

    [Fact]
    public void CreateSweptCircleBounds_ShouldCreatePlanarBounds()
    {
        DynamicCcdPlanarBounds bounds = DynamicCcdCandidateIndex2D.CreateSweptCircleBounds(
            new Vector2d((Fixed64)(-2), Fixed64.One),
            new Vector2d((Fixed64)4, (Fixed64)(-2)),
            Fixed64.Half);

        bounds.MinX.Should().Be(Fixed64.FromFraction(-5, 2));
        bounds.MinZ.Should().Be(Fixed64.FromFraction(-3, 2));
        bounds.MaxX.Should().Be(Fixed64.FromFraction(5, 2));
        bounds.MaxZ.Should().Be(Fixed64.FromFraction(3, 2));
    }

    private static FixedBoundVolume CreateBounds(Fixed64 x, Fixed64 z)
    {
        Vector3d center = new(x, Fixed64.Zero, z);
        Vector3d extents = new(Fixed64.Half, Fixed64.Half, Fixed64.Half);
        return new FixedBoundVolume(center - extents, center + extents);
    }

    private static DynamicCcdPlanarBounds CreatePlanarBounds(Fixed64 x, Fixed64 z)
    {
        return new DynamicCcdPlanarBounds(
            x - Fixed64.Half,
            z - Fixed64.Half,
            x + Fixed64.Half,
            z + Fixed64.Half);
    }
}
