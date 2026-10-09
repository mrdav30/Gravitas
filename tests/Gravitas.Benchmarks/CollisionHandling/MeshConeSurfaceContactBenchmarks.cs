using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;

namespace Gravitas.Benchmarks;

[MemoryDiagnoser]
public class MeshConeSurfaceContactBenchmarks
{
    private GravitasWorldContext _context;
    private CollisionPair _pair;

    [Params("QuadInterior", "QuadEdge", "SmallInterior", "SmallEdge", "SingleTriangle", "SubdividedInterior", "SubdividedTilted", "JoinedWalls", "DisconnectedSupports", "Ring", "ThinTab")]
    public string Geometry { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = GravitasWorldContext.CreateOwned();
        bool tilted = Geometry == "SubdividedTilted";
        int subdivision = Geometry.StartsWith("Subdivided", StringComparison.Ordinal) ? 8 : 1;
        Fixed64 extent = Geometry == "SmallInterior" ? Fixed64.FromFraction(1, 5)
            : Geometry == "SmallEdge" ? Fixed64.FromFraction(3, 20) : Fixed64.Two;
        Vector3d[] vertices;
        int[] triangles;
        if (Geometry == "JoinedWalls")
        {
            vertices = new Vector3d[] { new(0,-4,0), new(0,4,0), new(0,4,4), new(0,-4,4), new(4,-4,0), new(4,4,0) };
            triangles = new[] { 0,1,2, 0,2,3, 0,4,5, 0,5,1 };
        }
        else if (Geometry == "DisconnectedSupports")
        {
            Fixed64 near = Fixed64.FromFraction(1,16), far = Fixed64.FromFraction(3,16);
            vertices = new[] { new Vector3d(-far,Fixed64.Zero,-near), new Vector3d(-near,Fixed64.Zero,-near),
                new Vector3d(-near,Fixed64.Zero,near), new Vector3d(-far,Fixed64.Zero,near),
                new Vector3d(near,Fixed64.Zero,-near), new Vector3d(far,Fixed64.Zero,-near),
                new Vector3d(far,Fixed64.Zero,near), new Vector3d(near,Fixed64.Zero,near) };
            triangles = new[] { 0,1,2, 0,2,3, 4,5,6, 4,6,7 };
        }
        else if (Geometry == "Ring")
        {
            Fixed64 inner = Fixed64.FromFraction(1,16);
            vertices = new[] { new Vector3d(-1,0,-1), new Vector3d(1,0,-1), new Vector3d(1,0,1), new Vector3d(-1,0,1),
                new Vector3d(-inner,Fixed64.Zero,-inner), new Vector3d(inner,Fixed64.Zero,-inner),
                new Vector3d(inner,Fixed64.Zero,inner), new Vector3d(-inner,Fixed64.Zero,inner) };
            triangles = new[] { 0,1,5, 0,5,4, 1,2,6, 1,6,5, 2,3,7, 2,7,6, 3,0,4, 3,4,7 };
        }
        else if (Geometry == "ThinTab")
        {
            Fixed64 side = Fixed64.FromFraction(1,10), tab = Fixed64.FromFraction(1,32);
            vertices = new[] { new Vector3d(-side,Fixed64.Zero,-side), new Vector3d(side,Fixed64.Zero,-side),
                new Vector3d(side,Fixed64.Zero,-tab), new Vector3d(Fixed64.Half,Fixed64.Zero,-tab),
                new Vector3d(Fixed64.Half,Fixed64.Zero,tab), new Vector3d(side,Fixed64.Zero,tab),
                new Vector3d(side,Fixed64.Zero,side), new Vector3d(-side,Fixed64.Zero,side) };
            triangles = new[] { 0,1,2, 0,2,5, 0,5,6, 0,6,7, 2,3,4, 2,4,5 };
        }
        else
        {
            int width = subdivision + 1;
            vertices = new Vector3d[width * width];
            triangles = new int[subdivision * subdivision * 6];
            for (int z = 0; z < width; z++)
                for (int x = 0; x < width; x++)
                    vertices[z * width + x] = new Vector3d(
                        extent * Fixed64.FromFraction(2 * x, subdivision) - extent,
                        Fixed64.Zero,
                        extent * Fixed64.FromFraction(2 * z, subdivision) - extent);

            int offset = 0;
            for (int z = 0; z < subdivision; z++)
                for (int x = 0; x < subdivision; x++)
                {
                    int first = z * width + x;
                    triangles[offset++] = first;
                    triangles[offset++] = first + width;
                    triangles[offset++] = first + 1;
                    triangles[offset++] = first + 1;
                    triangles[offset++] = first + width;
                    triangles[offset++] = first + width + 1;
                }

            if (Geometry == "SingleTriangle")
            {
                Array.Resize(ref vertices, 3);
                triangles = new[] { 0, 2, 1 };
            }
        }
        var mesh = new LSMeshCollider(vertices, triangles,
            MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new BenchmarkMatterAgent(_context, Vector3d.Zero));
        var cone = new LSConeCollider();
        var coneAgent = new BenchmarkMatterAgent(_context,
            Geometry == "JoinedWalls"
                ? new Vector3d(Fixed64.FromFraction(3,10), Fixed64.Zero, Fixed64.FromFraction(3,10))
                : new Vector3d(tilted ? -Fixed64.FromFraction(28, 25)
                    : Geometry == "QuadEdge" ? Fixed64.Two : Fixed64.Zero,
                    -Fixed64.Quarter, Fixed64.Zero));
        if (tilted)
            coneAgent.Transform.LocalRotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
                Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        cone.InitializeWithNoBody(coneAgent);
        _pair = new CollisionPair(mesh, cone);
        if (!CollisionDetection.DoCollisionCheck(_pair))
            throw new InvalidOperationException("Surface contact benchmark must contain a real intersection.");
        if (Geometry == "JoinedWalls")
        {
            bool right = false, forward = false;
            foreach (ManifoldContact contact in _pair.Manifold)
            { right |= contact.Normal == Vector3d.Right; forward |= contact.Normal == Vector3d.Forward; }
            if (!right || !forward) throw new InvalidOperationException("A genuine wall constraint was lost.");
            return;
        }
        bool face = false;
        Fixed64 faceDepth = Fixed64.Zero;
        foreach (ManifoldContact contact in _pair.Manifold)
        {
            if (contact.PointA.Y != Fixed64.Zero || contact.Depth < Fixed64.Zero)
                throw new InvalidOperationException("A surface sample left its authored plane.");
            if (contact.Normal == Vector3d.Down && contact.Depth > Fixed64.Zero)
            {
                face = true;
                faceDepth = FixedMath.Max(faceDepth, contact.Depth);
            }
        }
        if (!face)
            throw new InvalidOperationException("The intersected sheet lost its face constraint.");
        if (!tilted && Geometry is not ("DisconnectedSupports" or "Ring") && FixedMath.Abs(faceDepth - Fixed64.Quarter) > Fixed64.FromRaw(4))
            throw new InvalidOperationException("The sheet's analytic face maximum changed.");
    }

    [Benchmark]
    public bool ConeSurface() => CollisionDetection.DoCollisionCheck(_pair);

    [GlobalCleanup]
    public void Cleanup() => _context.Dispose();
}
