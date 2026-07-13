using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;
using System.Collections.Generic;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class CollisionDetectionBenchmarks
{
    private const int DenseMeshSubdivision = 3;

    private GravitasWorldContext _context;
    private CollisionPair[] _pairs;
    private CollisionPair[] _primitivePairs;
    private CollisionPair[] _conePrimitivePairs;
    private CollisionPair[] _cylinderCapPairs;
    private CollisionPair[] _cuboidFacePairs;
    private CollisionPair[] _cuboidSatPairs;
    private CollisionPair[] _cuboidCapsulePairs;
    private CollisionPair[] _meshCylinderPairs;
    private CollisionPair[] _meshConePairs;
    private CollisionPair[] _meshCapsulePairs;
    private CollisionPair[] _meshCapsuleFallbackPairs;
    private CollisionPair[] _meshCuboidPairs;
    private CollisionPair[] _meshMeshPairs;
    private CollisionPair[] _concaveMeshCylinderPairs;
    private CollisionPair[] _concaveMeshCuboidPairs;
    private CollisionPair[] _concaveMeshMeshPairs;
    private CollisionPair[] _denseConcaveMeshMeshPairs;
    private CollisionPair[] _contactHeavyConcaveMeshMeshPairs;
    private CollisionPair[] _closedDenseMeshMeshPairs;
    private CollisionPair[] _authoredCompoundProxyPairs;
    private CollisionPair[] _denseConcaveMeshAuthoredCompoundProxyPairs;
    private CollisionPair[] _compoundSpherePairs;

    [Params(64)]
    public int PairCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkPhysicsScene.CreateContext(
            BenchmarkPhysicsScene.GridExtentForGrid(PairCount * 2),
            clearAllPools: true);
        _pairs = new CollisionPair[PairCount];

        for (int i = 0; i < _pairs.Length; i++)
        {
            Vector3d origin = PositionForPair(i);
            _pairs[i] = CreateDetectionPair(i, origin);
        }

        _primitivePairs = CreatePairSet(CreatePrimitivePair);
        _conePrimitivePairs = CreatePairSet(CreateConePrimitivePair);
        _cylinderCapPairs = CreatePairSet(CreateCylinderCapPair);
        _cuboidFacePairs = CreatePairSet(CreateCuboidFacePair);
        _cuboidSatPairs = CreatePairSet(CreateCuboidSatPair);
        _cuboidCapsulePairs = CreatePairSet(CreateCuboidCapsulePair);
        _meshCylinderPairs = CreatePairSet(CreateMeshCylinderPair);
        _meshConePairs = CreatePairSet(CreateMeshConePair);
        _meshCapsulePairs = CreatePairSet(CreateMeshCapsulePair);
        _meshCapsuleFallbackPairs = CreatePairSet(CreateMeshCapsuleFallbackPair);
        _meshCuboidPairs = CreatePairSet(CreateMeshCuboidPair);
        _meshMeshPairs = CreatePairSet(CreateMeshMeshPair);
        _concaveMeshCylinderPairs = CreatePairSet(CreateConcaveMeshCylinderPair);
        _concaveMeshCuboidPairs = CreatePairSet(CreateConcaveMeshCuboidPair);
        _concaveMeshMeshPairs = CreatePairSet(CreateConcaveMeshMeshPair);
        _denseConcaveMeshMeshPairs = CreatePairSet(CreateDenseConcaveMeshMeshPair);
        _contactHeavyConcaveMeshMeshPairs = CreatePairSet(CreateContactHeavyConcaveMeshMeshPair);
        _closedDenseMeshMeshPairs = CreatePairSet(CreateClosedDenseMeshMeshPair);
        _authoredCompoundProxyPairs = CreatePairSet(CreateAuthoredCompoundProxyPair);
        _denseConcaveMeshAuthoredCompoundProxyPairs = CreatePairSet(CreateDenseConcaveMeshAuthoredCompoundProxyPair);
        _compoundSpherePairs = CreatePairSet(CreateCompoundSpherePair);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context.Dispose();
        _context = null;
        _pairs = null;
        _primitivePairs = null;
        _conePrimitivePairs = null;
        _cylinderCapPairs = null;
        _cuboidFacePairs = null;
        _cuboidSatPairs = null;
        _cuboidCapsulePairs = null;
        _meshCylinderPairs = null;
        _meshConePairs = null;
        _meshCapsulePairs = null;
        _meshCapsuleFallbackPairs = null;
        _meshCuboidPairs = null;
        _meshMeshPairs = null;
        _concaveMeshCylinderPairs = null;
        _concaveMeshCuboidPairs = null;
        _concaveMeshMeshPairs = null;
        _denseConcaveMeshMeshPairs = null;
        _contactHeavyConcaveMeshMeshPairs = null;
        _closedDenseMeshMeshPairs = null;
        _authoredCompoundProxyPairs = null;
        _denseConcaveMeshAuthoredCompoundProxyPairs = null;
        _compoundSpherePairs = null;
    }

    [Benchmark]
    public int CheckPreparedPrimitivePairs()
    {
        return CountCollisions(_pairs);
    }

    [Benchmark]
    public int CheckNonSatPrimitivePairs()
    {
        return CountCollisions(_primitivePairs);
    }

    [Benchmark]
    public int GeneratePrimitiveManifolds()
    {
        return CountManifoldContacts(_primitivePairs);
    }

    [Benchmark]
    public int CheckConePrimitivePairs()
    {
        return CountCollisions(_conePrimitivePairs);
    }

    [Benchmark]
    public int GenerateConePrimitiveManifolds()
    {
        return CountManifoldContacts(_conePrimitivePairs);
    }

    [Benchmark]
    public int GenerateCylinderCapManifolds()
    {
        return CountManifoldContacts(_cylinderCapPairs);
    }

    [Benchmark]
    public int GenerateCuboidFaceManifolds()
    {
        return CountManifoldContacts(_cuboidFacePairs);
    }

    [Benchmark]
    public int CheckCuboidCuboidSatPairs()
    {
        return CountCollisions(_cuboidSatPairs);
    }

    [Benchmark]
    public int CheckCuboidCapsulePairs()
    {
        return CountCollisions(_cuboidCapsulePairs);
    }

    [Benchmark]
    public int CheckMeshCylinderPairs()
    {
        return CountCollisions(_meshCylinderPairs);
    }

    [Benchmark]
    public int GenerateMeshCylinderManifolds()
    {
        return CountManifoldContacts(_meshCylinderPairs);
    }

    [Benchmark]
    public int CheckMeshConePairs()
    {
        return CountCollisions(_meshConePairs);
    }

    [Benchmark]
    public int GenerateMeshConeManifolds()
    {
        return CountManifoldContacts(_meshConePairs);
    }

    [Benchmark]
    public int CheckMeshCapsulePairs()
    {
        return CountCollisions(_meshCapsulePairs);
    }

    [Benchmark]
    public int CheckMeshCapsuleFallbackPairs()
    {
        return CountCollisions(_meshCapsuleFallbackPairs);
    }

    [Benchmark]
    public int CheckMeshCuboidPairs()
    {
        return CountCollisions(_meshCuboidPairs);
    }

    [Benchmark]
    public int GenerateMeshCuboidManifolds()
    {
        return CountManifoldContacts(_meshCuboidPairs);
    }

    [Benchmark]
    public int CheckMeshMeshPairs()
    {
        return CountCollisions(_meshMeshPairs);
    }

    [Benchmark]
    public int CheckConcaveMeshCylinderPairs()
    {
        return CountCollisions(_concaveMeshCylinderPairs);
    }

    [Benchmark]
    public int CheckConcaveMeshCuboidPairs()
    {
        return CountCollisions(_concaveMeshCuboidPairs);
    }

    [Benchmark]
    public int CheckConcaveMeshMeshPairs()
    {
        return CountCollisions(_concaveMeshMeshPairs);
    }

    [Benchmark]
    public int CheckDenseConcaveMeshMeshPairs()
    {
        return CountCollisions(_denseConcaveMeshMeshPairs);
    }

    [Benchmark]
    public int CheckContactHeavyConcaveMeshMeshPairs()
    {
        return CountCollisions(_contactHeavyConcaveMeshMeshPairs);
    }

    [Benchmark]
    public int CheckClosedDenseMeshMeshPairs()
    {
        return CountCollisions(_closedDenseMeshMeshPairs);
    }

    [Benchmark]
    public int CheckAuthoredCompoundProxyPairs()
    {
        return CountCollisions(_authoredCompoundProxyPairs);
    }

    [Benchmark]
    public int GenerateAuthoredCompoundProxyManifolds()
    {
        return CountManifoldContacts(_authoredCompoundProxyPairs);
    }

    [Benchmark]
    public int CheckDenseConcaveMeshAuthoredCompoundProxyPairs()
    {
        return CountCollisions(_denseConcaveMeshAuthoredCompoundProxyPairs);
    }

    [Benchmark]
    public int CheckCompoundSpherePairs()
    {
        return CountCollisions(_compoundSpherePairs);
    }

    [Benchmark]
    public int GenerateCompoundManifolds()
    {
        return CountManifoldContacts(_compoundSpherePairs);
    }

    private static int CountCollisions(CollisionPair[] pairs)
    {
        int collisionCount = 0;
        for (int i = 0; i < pairs.Length; i++)
        {
            if (CollisionDetection.DoCollisionCheck(pairs[i]))
                collisionCount++;
        }

        return collisionCount;
    }

    private static int CountManifoldContacts(CollisionPair[] pairs)
    {
        int contactCount = 0;
        for (int i = 0; i < pairs.Length; i++)
        {
            if (CollisionDetection.DoCollisionCheck(pairs[i]))
                contactCount += pairs[i].Manifold.Count;
        }

        return contactCount;
    }

    private CollisionPair[] CreatePairSet(Func<int, Vector3d, CollisionPair> pairFactory)
    {
        var pairs = new CollisionPair[PairCount];
        for (int i = 0; i < pairs.Length; i++)
            pairs[i] = pairFactory(i, PositionForPair(i));

        return pairs;
    }

    private CollisionPair CreateDetectionPair(int index, Vector3d origin)
    {
        return (index % 9) switch
        {
            0 => new CollisionPair(
                CreateSphere(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            1 => new CollisionPair(
                CreateCapsule(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            2 => new CollisionPair(
                CreateCuboid(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            3 => new CollisionPair(
                CreateCuboid(origin),
                CreateCuboid(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            4 => new CollisionPair(
                CreateCylinder(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            5 => new CollisionPair(
                CreateCylinder(origin),
                CreateCapsule(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            6 => new CollisionPair(
                CreateCylinder(origin),
                CreateCylinder(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            7 => new CollisionPair(
                CreateCuboid(origin),
                CreateCylinder(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            _ => new CollisionPair(
                CreateMeshFloor(origin),
                CreateCylinder(origin + new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero))),
        };
    }

    private CollisionPair CreatePrimitivePair(int index, Vector3d origin)
    {
        return (index % 7) switch
        {
            0 => new CollisionPair(
                CreateSphere(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            1 => new CollisionPair(
                CreateCapsule(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            2 => new CollisionPair(
                CreateCuboid(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            3 => new CollisionPair(
                CreateCylinder(origin),
                CreateSphere(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            4 => new CollisionPair(
                CreateCylinder(origin),
                CreateCapsule(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            5 => new CollisionPair(
                CreateCylinder(origin),
                CreateCylinder(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
            _ => new CollisionPair(
                CreateCuboid(origin),
                CreateCylinder(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero))),
        };
    }

    private CollisionPair CreateConePrimitivePair(int index, Vector3d origin)
    {
        return (index % 4) switch
        {
            0 => new CollisionPair(
                CreateCone(origin),
                CreateSphere(origin + new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero))),
            1 => new CollisionPair(
                CreateCone(origin),
                CreateCuboid(origin + new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero))),
            2 => new CollisionPair(
                CreateCone(origin),
                CreateCylinder(origin + new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero))),
            _ => new CollisionPair(
                CreateCone(origin),
                CreateCone(origin + new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero))),
        };
    }

    private CollisionPair CreateCuboidFacePair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateCuboid(origin),
            CreateCuboid(origin + new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)));
    }

    private CollisionPair CreateCylinderCapPair(int index, Vector3d origin)
    {
        Vector3d upper = origin + new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero);
        if ((index & 1) == 0)
        {
            return new CollisionPair(
                CreateCylinder(origin),
                CreateCylinder(upper));
        }

        return new CollisionPair(
            CreateCuboid(origin),
            CreateCylinder(upper));
    }

    private CollisionPair CreateCuboidSatPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateCuboid(origin, FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)35, Fixed64.Zero)),
            CreateCuboid(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero)));
    }

    private CollisionPair CreateCuboidCapsulePair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateCuboid(origin, FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)20, Fixed64.Zero)),
            CreateCapsule(
                origin + new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.FromFraction(1, 4)),
                new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)));
    }

    private CollisionPair CreateMeshCylinderPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateMeshFloor(origin),
            CreateCylinder(origin + new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero)));
    }

    private CollisionPair CreateMeshConePair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateMeshFloor(origin),
            CreateCone(origin + new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero)));
    }

    private CollisionPair CreateMeshCapsulePair(int index, Vector3d origin)
    {
        return new CollisionPair(
            BenchmarkPhysicsScene.CreateDynamicConvexCube(_context, origin),
            CreateCapsule(
                origin + new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.FromFraction(1, 4)),
                new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)));
    }

    private CollisionPair CreateMeshCapsuleFallbackPair(int index, Vector3d origin)
    {
        Vector3d segmentStart = origin + new Vector3d(
            Fixed64.FromFraction(5, 4),
            Fixed64.FromFraction(3, 2),
            Fixed64.Half);
        Vector3d segmentEnd = origin + new Vector3d(
            Fixed64.Half,
            Fixed64.FromFraction(-5, 4),
            Fixed64.FromFraction(-3, 4));
        Vector3d segment = segmentEnd - segmentStart;
        Fixed64 segmentLength = segment.Magnitude;
        Vector3d segmentDirection = segment / segmentLength;
        Vector3d rotationAxis = Vector3d.Cross(Vector3d.Up, segmentDirection);
        FixedQuaternion rotation = new FixedQuaternion(
            rotationAxis.X,
            rotationAxis.Y,
            rotationAxis.Z,
            Fixed64.One + Vector3d.Dot(Vector3d.Up, segmentDirection)).Normalized;
        LSCapsuleCollider capsule = CreateBody(
            new LSCapsuleCollider
            {
                Radius = Fixed64.FromFraction(1, 4),
                Size = new Vector3d(Fixed64.Half, segmentLength + Fixed64.Half, Fixed64.Half)
            },
            (segmentStart + segmentEnd) * Fixed64.Half,
            rotation,
            preventAngularForces: true).Collider;

        return new CollisionPair(
            BenchmarkPhysicsScene.CreateDynamicConvexCube(_context, origin),
            capsule);
    }

    private CollisionPair CreateMeshCuboidPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateMeshFloor(origin),
            CreateCuboid(origin + new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero)));
    }

    private CollisionPair CreateMeshMeshPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateMeshFloor(origin),
            CreateMeshFloor(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero)));
    }

    private CollisionPair CreateConcaveMeshCylinderPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateConcaveUChannel(origin),
            CreateCylinder(origin + new Vector3d(Fixed64.FromFraction(7, 4), Fixed64.One, (Fixed64)2)));
    }

    private CollisionPair CreateConcaveMeshCuboidPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateConcaveUChannel(origin),
            CreateCuboid(origin + new Vector3d(Fixed64.FromFraction(7, 4), Fixed64.One, (Fixed64)2)));
    }

    private CollisionPair CreateConcaveMeshMeshPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateConcaveUChannel(origin),
            CreateInsideCornerMesh(origin));
    }

    private CollisionPair CreateDenseConcaveMeshMeshPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateDenseConcaveUChannel(origin),
            CreateDenseInsideCornerMesh(origin));
    }

    private CollisionPair CreateContactHeavyConcaveMeshMeshPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateDenseConcaveUChannel(origin),
            CreateDenseConcaveUChannel(origin + new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero)));
    }

    private CollisionPair CreateClosedDenseMeshMeshPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateClosedDenseConcaveCube(origin),
            CreateClosedDenseConcaveCube(origin + new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero)));
    }

    private CollisionPair CreateAuthoredCompoundProxyPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateAuthoredUChannelProxy(origin),
            CreateAuthoredInsideCornerProxy(origin));
    }

    private CollisionPair CreateDenseConcaveMeshAuthoredCompoundProxyPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateDenseConcaveUChannel(origin),
            CreateAuthoredInsideCornerProxy(origin));
    }

    private CollisionPair CreateCompoundSpherePair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateCompound(origin),
            CreateSphere(origin + new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero)));
    }

    private LSSphereCollider CreateSphere(Vector3d position) =>
        CreateBody(new LSSphereCollider(), position).Collider;

    private LSCapsuleCollider CreateCapsule(Vector3d position) =>
        CreateCapsule(position, Vector3d.One);

    private LSCapsuleCollider CreateCapsule(Vector3d position, Vector3d size) =>
        CreateBody(new LSCapsuleCollider { Size = size }, position, preventAngularForces: true).Collider;

    private LSCuboidCollider CreateCuboid(Vector3d position, FixedQuaternion? rotation = null) =>
        CreateBody(new LSCuboidCollider(), position, rotation ?? FixedQuaternion.Identity).Collider;

    private LSCylinderCollider CreateCylinder(Vector3d position) =>
        CreateBody(new LSCylinderCollider(), position).Collider;

    private LSConeCollider CreateCone(Vector3d position) =>
        CreateBody(new LSConeCollider { Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One) }, position).Collider;

    private LSCompoundCollider CreateCompound(Vector3d position) =>
        CreateBody(
            new LSCompoundCollider(
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero)),
                CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero))),
            position,
            preventAngularForces: true).Collider;

    private LSCompoundCollider CreateAuthoredUChannelProxy(Vector3d position) =>
        CreateBody(
            new LSCompoundCollider(
                CreateCuboidPart(
                    new Vector3d((Fixed64)(-2), Fixed64.One, (Fixed64)2),
                    new Vector3d(Fixed64.FromFraction(1, 4), (Fixed64)2, (Fixed64)4)),
                CreateCuboidPart(
                    new Vector3d((Fixed64)2, Fixed64.One, (Fixed64)2),
                    new Vector3d(Fixed64.FromFraction(1, 4), (Fixed64)2, (Fixed64)4)),
                CreateCuboidPart(
                    new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero),
                    new Vector3d((Fixed64)4, (Fixed64)2, Fixed64.FromFraction(1, 4)))),
            position,
            preventAngularForces: true).Collider;

    private LSCompoundCollider CreateAuthoredInsideCornerProxy(Vector3d position) =>
        CreateBody(
            new LSCompoundCollider(
                CreateCuboidPart(
                    new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2),
                    new Vector3d((Fixed64)4, Fixed64.FromFraction(1, 4), (Fixed64)4)),
                CreateCuboidPart(
                    new Vector3d(Fixed64.Zero, (Fixed64)2, (Fixed64)2),
                    new Vector3d(Fixed64.FromFraction(1, 4), (Fixed64)4, (Fixed64)4)),
                CreateCuboidPart(
                    new Vector3d((Fixed64)2, (Fixed64)2, Fixed64.Zero),
                    new Vector3d((Fixed64)4, (Fixed64)4, Fixed64.FromFraction(1, 4)))),
            position,
            preventAngularForces: true).Collider;

    private static CompoundColliderPart CreateCuboidPart(Vector3d localOffset, Vector3d size) =>
        CompoundColliderPart.Cuboid(size, localOffset);

    private LSMeshCollider CreateMeshFloor(Vector3d position) =>
        CreateBody(
            new LSMeshCollider(
                new[]
                {
                    new Vector3d((Fixed64)(-1), Fixed64.Zero, (Fixed64)(-1)),
                    new Vector3d(Fixed64.One, Fixed64.Zero, (Fixed64)(-1)),
                    new Vector3d((Fixed64)(-1), Fixed64.Zero, Fixed64.One),
                    new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One)
                },
                new[] { 0, 2, 1, 1, 2, 3 }),
            position,
            preventAngularForces: true).Collider;

    private LSMeshCollider CreateConcaveUChannel(Vector3d position) =>
        CreateBody(
            new LSMeshCollider(
                CreateUChannelVertices(),
                new[]
                {
                    0, 1, 2, 2, 1, 3,
                    4, 5, 6, 6, 5, 7,
                    8, 9, 10, 10, 9, 11
                },
                MeshColliderMode.Concave),
            position,
            preventAngularForces: true).Collider;

    private LSMeshCollider CreateInsideCornerMesh(Vector3d position) =>
        CreateBody(
            new LSMeshCollider(
                new[]
                {
                    new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
                    new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
                    new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)4),
                    new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)4),
                    new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
                    new Vector3d(Fixed64.Zero, (Fixed64)4, Fixed64.Zero),
                    new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64)4),
                    new Vector3d(Fixed64.Zero, (Fixed64)4, (Fixed64)4),
                    new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero),
                    new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
                    new Vector3d(Fixed64.Zero, (Fixed64)4, Fixed64.Zero),
                    new Vector3d((Fixed64)4, (Fixed64)4, Fixed64.Zero)
                },
                new[] { 0, 2, 1, 1, 2, 3, 4, 5, 6, 6, 5, 7, 8, 9, 10, 10, 9, 11 },
                MeshColliderMode.Concave),
            position,
            preventAngularForces: true).Collider;

    private LSMeshCollider CreateDenseConcaveUChannel(Vector3d position)
    {
        CreateSubdividedUChannel(DenseMeshSubdivision, out Vector3d[] vertices, out int[] triangles);
        return CreateBody(
            new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave),
            position,
            preventAngularForces: true).Collider;
    }

    private LSMeshCollider CreateDenseInsideCornerMesh(Vector3d position)
    {
        CreateSubdividedInsideCorner(DenseMeshSubdivision, out Vector3d[] vertices, out int[] triangles);
        return CreateBody(
            new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave),
            position,
            preventAngularForces: true).Collider;
    }

    private LSMeshCollider CreateClosedDenseConcaveCube(Vector3d position)
    {
        CreateSubdividedClosedCube(DenseMeshSubdivision, out Vector3d[] vertices, out int[] triangles);
        return CreateBody(
            new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave),
            position,
            preventAngularForces: true).Collider;
    }

    private static Vector3d[] CreateUChannelVertices()
    {
        Fixed64 left = (Fixed64)(-2);
        Fixed64 right = (Fixed64)2;
        Fixed64 height = (Fixed64)2;
        Fixed64 depth = (Fixed64)4;

        return new[]
        {
            new Vector3d(left, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(left, height, Fixed64.Zero),
            new Vector3d(left, Fixed64.Zero, depth),
            new Vector3d(left, height, depth),
            new Vector3d(right, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(right, Fixed64.Zero, depth),
            new Vector3d(right, height, Fixed64.Zero),
            new Vector3d(right, height, depth),
            new Vector3d(left, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(right, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(left, height, Fixed64.Zero),
            new Vector3d(right, height, Fixed64.Zero)
        };
    }

    private static void CreateSubdividedUChannel(
        int subdivisions,
        out Vector3d[] vertices,
        out int[] triangles)
    {
        Fixed64 left = (Fixed64)(-2);
        Fixed64 right = (Fixed64)2;
        Fixed64 height = (Fixed64)2;
        Fixed64 depth = (Fixed64)4;
        var vertexList = new List<Vector3d>(3 * (subdivisions + 1) * (subdivisions + 1));
        var triangleList = new List<int>(18 * subdivisions * subdivisions);

        AddSubdividedQuad(vertexList, triangleList, new Vector3d(left, Fixed64.Zero, Fixed64.Zero), new Vector3d(Fixed64.Zero, height, Fixed64.Zero), new Vector3d(Fixed64.Zero, Fixed64.Zero, depth), subdivisions);
        AddSubdividedQuad(vertexList, triangleList, new Vector3d(right, Fixed64.Zero, Fixed64.Zero), new Vector3d(Fixed64.Zero, Fixed64.Zero, depth), new Vector3d(Fixed64.Zero, height, Fixed64.Zero), subdivisions);
        AddSubdividedQuad(vertexList, triangleList, new Vector3d(left, Fixed64.Zero, Fixed64.Zero), new Vector3d(right - left, Fixed64.Zero, Fixed64.Zero), new Vector3d(Fixed64.Zero, height, Fixed64.Zero), subdivisions);

        vertices = vertexList.ToArray();
        triangles = triangleList.ToArray();
    }

    private static void CreateSubdividedInsideCorner(
        int subdivisions,
        out Vector3d[] vertices,
        out int[] triangles)
    {
        Fixed64 four = (Fixed64)4;
        var vertexList = new List<Vector3d>(3 * (subdivisions + 1) * (subdivisions + 1));
        var triangleList = new List<int>(18 * subdivisions * subdivisions);

        AddSubdividedQuad(vertexList, triangleList, Vector3d.Zero, new Vector3d(Fixed64.Zero, Fixed64.Zero, four), new Vector3d(four, Fixed64.Zero, Fixed64.Zero), subdivisions);
        AddSubdividedQuad(vertexList, triangleList, Vector3d.Zero, new Vector3d(Fixed64.Zero, four, Fixed64.Zero), new Vector3d(Fixed64.Zero, Fixed64.Zero, four), subdivisions);
        AddSubdividedQuad(vertexList, triangleList, Vector3d.Zero, new Vector3d(four, Fixed64.Zero, Fixed64.Zero), new Vector3d(Fixed64.Zero, four, Fixed64.Zero), subdivisions);

        vertices = vertexList.ToArray();
        triangles = triangleList.ToArray();
    }

    private static void CreateSubdividedClosedCube(
        int subdivisions,
        out Vector3d[] vertices,
        out int[] triangles)
    {
        Fixed64 negative = -Fixed64.One;
        Fixed64 positive = Fixed64.One;
        Fixed64 size = (Fixed64)2;
        var vertexList = new List<Vector3d>(6 * (subdivisions + 1) * (subdivisions + 1));
        var triangleList = new List<int>(36 * subdivisions * subdivisions);

        AddSubdividedQuad(vertexList, triangleList, new Vector3d(negative, negative, negative), new Vector3d(Fixed64.Zero, size, Fixed64.Zero), new Vector3d(size, Fixed64.Zero, Fixed64.Zero), subdivisions);
        AddSubdividedQuad(vertexList, triangleList, new Vector3d(negative, negative, positive), new Vector3d(size, Fixed64.Zero, Fixed64.Zero), new Vector3d(Fixed64.Zero, size, Fixed64.Zero), subdivisions);
        AddSubdividedQuad(vertexList, triangleList, new Vector3d(negative, negative, negative), new Vector3d(Fixed64.Zero, Fixed64.Zero, size), new Vector3d(Fixed64.Zero, size, Fixed64.Zero), subdivisions);
        AddSubdividedQuad(vertexList, triangleList, new Vector3d(positive, negative, negative), new Vector3d(Fixed64.Zero, size, Fixed64.Zero), new Vector3d(Fixed64.Zero, Fixed64.Zero, size), subdivisions);
        AddSubdividedQuad(vertexList, triangleList, new Vector3d(negative, negative, negative), new Vector3d(size, Fixed64.Zero, Fixed64.Zero), new Vector3d(Fixed64.Zero, Fixed64.Zero, size), subdivisions);
        AddSubdividedQuad(vertexList, triangleList, new Vector3d(negative, positive, negative), new Vector3d(Fixed64.Zero, Fixed64.Zero, size), new Vector3d(size, Fixed64.Zero, Fixed64.Zero), subdivisions);

        vertices = vertexList.ToArray();
        triangles = triangleList.ToArray();
    }

    private static void AddSubdividedQuad(
        List<Vector3d> vertices,
        List<int> triangles,
        Vector3d origin,
        Vector3d edgeA,
        Vector3d edgeB,
        int subdivisions)
    {
        for (int a = 0; a < subdivisions; a++)
        {
            Fixed64 a0 = Fixed64.FromFraction(a, subdivisions);
            Fixed64 a1 = Fixed64.FromFraction(a + 1, subdivisions);

            for (int b = 0; b < subdivisions; b++)
            {
                Fixed64 b0 = Fixed64.FromFraction(b, subdivisions);
                Fixed64 b1 = Fixed64.FromFraction(b + 1, subdivisions);
                int p00 = AddVertex(vertices, origin + edgeA * a0 + edgeB * b0);
                int p10 = AddVertex(vertices, origin + edgeA * a1 + edgeB * b0);
                int p01 = AddVertex(vertices, origin + edgeA * a0 + edgeB * b1);
                int p11 = AddVertex(vertices, origin + edgeA * a1 + edgeB * b1);

                AddTriangle(triangles, p00, p10, p01);
                AddTriangle(triangles, p01, p10, p11);
            }
        }
    }

    private static int AddVertex(List<Vector3d> vertices, Vector3d vertex)
    {
        int index = vertices.Count;
        vertices.Add(vertex);
        return index;
    }

    private static void AddTriangle(List<int> triangles, int a, int b, int c)
    {
        triangles.Add(a);
        triangles.Add(b);
        triangles.Add(c);
    }

    private ScenarioBody<TCollider> CreateBody<TCollider>(
        TCollider collider,
        Vector3d position,
        FixedQuaternion? rotation = null,
        bool preventAngularForces = false)
        where TCollider : LSCollider
    {
        var agent = new BenchmarkMatterAgent(_context, position);
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = preventAngularForces ? BodyFreezeAxes3D.Rotation : BodyFreezeAxes3D.None
        };

        body.Initialize(position, rotation ?? FixedQuaternion.Identity);
        return new ScenarioBody<TCollider>(body, collider);
    }

    private static Vector3d PositionForPair(int index)
    {
        int x = index % 8;
        int z = index / 8;
        return new Vector3d(x * 3, 0, z * 3);
    }

    private readonly struct ScenarioBody<TCollider>
        where TCollider : LSCollider
    {
        public ScenarioBody(SolidBody body, TCollider collider)
        {
            Body = body;
            Collider = collider;
        }

        public SolidBody Body { get; }

        public TCollider Collider { get; }
    }
}
