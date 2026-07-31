//=======================================================================
// QueryProjectionScalingBenchmarks.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using BenchmarkDotNet.Attributes;
using FixedMathSharp;
using Gravitas.Queries;
using Gravitas.Support;
using GridForge.Configuration;
using GridForge.Grids.Storage;
using GridForge.Spatial;
using SwiftCollections;
using System;

namespace Gravitas.Benchmarks;

/// <summary>
/// Measures X/Z projection-query cost as the irrelevant world Y range grows.
/// </summary>
[MemoryDiagnoser]
public class QueryProjectionScalingBenchmarks
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    private GravitasWorldContext _context;
    private SwiftList<Physics3DHit> _hits;

    [Params(8, 128, 1024)]
    public int VerticalExtent { get; set; }

    [Params(GridStorageKind.Dense, GridStorageKind.Sparse)]
    public GridStorageKind StorageKind { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _context = BenchmarkEnvironment.PrepareOwnedContext(clearAllPools: true);
        int halfHeight = VerticalExtent / 2;
        var configuration = new GridConfiguration(
            new Vector3d((Fixed64)(-4), (Fixed64)(-halfHeight), (Fixed64)(-4)),
            new Vector3d((Fixed64)4, (Fixed64)(halfHeight - 1), (Fixed64)4),
            storageKind: StorageKind);

        bool added = StorageKind == GridStorageKind.Dense
            ? _context.World.TryAddGrid(configuration, out _)
            : _context.World.TryAddGrid(configuration, CreateSparseColliderVoxels(halfHeight), out _);
        if (!added)
            throw new InvalidOperationException("Unable to add projection-scaling benchmark grid.");

        BenchmarkPhysicsScene.CreateDynamicSphereLine(_context, count: 1);
        _hits = new SwiftList<Physics3DHit>(1);
        if (OverlapCircleAll() != 1)
            throw new InvalidOperationException("Projection-scaling benchmark must retain its expected hit.");
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        _context.Dispose();
        _context = null;
        _hits = null;
    }

    [Benchmark]
    public int OverlapCircleAll() =>
        _context.Query3D.OverlapCircleAll(
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero),
            (Fixed64)2,
            IncludeLayerZero,
            _hits);

    private static VoxelIndex[] CreateSparseColliderVoxels(int halfHeight)
    {
        var voxels = new VoxelIndex[27];
        int index = 0;
        for (int x = 3; x <= 5; x++)
        {
            for (int y = halfHeight - 1; y <= halfHeight + 1; y++)
            {
                for (int z = 3; z <= 5; z++)
                    voxels[index++] = new VoxelIndex(x, y, z);
            }
        }

        return voxels;
    }
}
