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
    private CollisionPair[] _cuboidFacePairs;
    private CollisionPair[] _cuboidSatPairs;
    private CollisionPair[] _meshCylinderPairs;
    private CollisionPair[] _meshCuboidPairs;
    private CollisionPair[] _meshMeshPairs;
    private CollisionPair[] _concaveMeshCylinderPairs;
    private CollisionPair[] _concaveMeshCuboidPairs;
    private CollisionPair[] _concaveMeshMeshPairs;
    private CollisionPair[] _denseConcaveMeshMeshPairs;
    private CollisionPair[] _contactHeavyConcaveMeshMeshPairs;
    private CollisionPair[] _closedDenseMeshMeshPairs;
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
        _cuboidFacePairs = CreatePairSet(CreateCuboidFacePair);
        _cuboidSatPairs = CreatePairSet(CreateCuboidSatPair);
        _meshCylinderPairs = CreatePairSet(CreateMeshCylinderPair);
        _meshCuboidPairs = CreatePairSet(CreateMeshCuboidPair);
        _meshMeshPairs = CreatePairSet(CreateMeshMeshPair);
        _concaveMeshCylinderPairs = CreatePairSet(CreateConcaveMeshCylinderPair);
        _concaveMeshCuboidPairs = CreatePairSet(CreateConcaveMeshCuboidPair);
        _concaveMeshMeshPairs = CreatePairSet(CreateConcaveMeshMeshPair);
        _denseConcaveMeshMeshPairs = CreatePairSet(CreateDenseConcaveMeshMeshPair);
        _contactHeavyConcaveMeshMeshPairs = CreatePairSet(CreateContactHeavyConcaveMeshMeshPair);
        _closedDenseMeshMeshPairs = CreatePairSet(CreateClosedDenseMeshMeshPair);
        _compoundSpherePairs = CreatePairSet(CreateCompoundSpherePair);
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context.Dispose();
        _context = null;
        _pairs = null;
        _primitivePairs = null;
        _cuboidFacePairs = null;
        _cuboidSatPairs = null;
        _meshCylinderPairs = null;
        _meshCuboidPairs = null;
        _meshMeshPairs = null;
        _concaveMeshCylinderPairs = null;
        _concaveMeshCuboidPairs = null;
        _concaveMeshMeshPairs = null;
        _denseConcaveMeshMeshPairs = null;
        _contactHeavyConcaveMeshMeshPairs = null;
        _closedDenseMeshMeshPairs = null;
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
    public int CheckMeshCylinderPairs()
    {
        return CountCollisions(_meshCylinderPairs);
    }

    [Benchmark]
    public int CheckMeshCuboidPairs()
    {
        return CountCollisions(_meshCuboidPairs);
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

    private CollisionPair CreateCuboidFacePair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateCuboid(origin),
            CreateCuboid(origin + new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Zero)));
    }

    private CollisionPair CreateCuboidSatPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateCuboid(origin, FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)35, Fixed64.Zero)),
            CreateCuboid(origin + new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero)));
    }

    private CollisionPair CreateMeshCylinderPair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateMeshFloor(origin),
            CreateCylinder(origin + new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.Zero)));
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

    private CollisionPair CreateCompoundSpherePair(int index, Vector3d origin)
    {
        return new CollisionPair(
            CreateCompound(origin),
            CreateSphere(origin + new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Zero, Fixed64.Zero)));
    }

    private LSSphereCollider CreateSphere(Vector3d position) =>
        CreateBody(new LSSphereCollider(), position).Collider;

    private LSCapsuleCollider CreateCapsule(Vector3d position) =>
        CreateBody(new LSCapsuleCollider(), position, preventAngularForces: true).Collider;

    private LSCuboidCollider CreateCuboid(Vector3d position, FixedQuaternion? rotation = null) =>
        CreateBody(new LSCuboidCollider(), position, rotation ?? FixedQuaternion.Identity).Collider;

    private LSCylinderCollider CreateCylinder(Vector3d position) =>
        CreateBody(new LSCylinderCollider(), position).Collider;

    private LSCompoundCollider CreateCompound(Vector3d position) =>
        CreateBody(
            new LSCompoundCollider(
                new CompoundColliderPart(new LSSphereCollider { LocalOffset = new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero) }),
                new CompoundColliderPart(new LSSphereCollider { LocalOffset = new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero) })),
            position,
            preventAngularForces: true).Collider;

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
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One,
            PreventAngularForces = preventAngularForces
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
        public ScenarioBody(StiffBody body, TCollider collider)
        {
            Body = body;
            Collider = collider;
        }

        public StiffBody Body { get; }

        public TCollider Collider { get; }
    }
}
