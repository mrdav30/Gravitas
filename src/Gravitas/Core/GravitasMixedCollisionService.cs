using FixedMathSharp;
using Gravitas.Colliders;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Owns mixed 2D/3D collision lifecycle and broad-phase state for one <see cref="GravitasWorldContext"/>.
/// </summary>
internal sealed class GravitasMixedCollisionService
{
    private const int DefaultPartitionPoolCapacity = 1024;

    private readonly GravitasWorldContext _context;
    private readonly SwiftBucket<PhysicsMixedPartition> _activePartitions = new(DefaultPartitionPoolCapacity);
    private readonly SwiftStack<PhysicsMixedPartition> _inactivePartitionPool = new(DefaultPartitionPoolCapacity);
    private readonly SwiftHashSet<int> _redundancyChecker = new();
    private readonly SwiftHashSet<ushort> _processedGrids = new();
    private readonly SwiftHashSet<ulong> _processedPairKeys = new();
    private readonly SwiftList<PhysicsMixedPartition> _retainedPartitions = new();
    private readonly SwiftList<PhysicsMixedPartition> _distributionPartitions = new();
    private readonly SwiftList<int> _distributionDynamic3DIds = new();
    private readonly SwiftList<int> _distributionAwakeDynamic3DIds = new();
    private readonly SwiftList<int> _distributionStatic3DIds = new();
    private readonly SwiftList<int> _distributionDynamic2DIds = new();
    private readonly SwiftList<int> _distributionAwakeDynamic2DIds = new();
    private readonly SwiftList<int> _distributionStatic2DIds = new();
    private readonly SwiftList<MixedColliderKey> _candidatePairs = new();

    private int _retainedPartitionRetirementCursor;

    internal GravitasMixedCollisionService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    internal GravitasWorldContext Context => _context;

    internal uint Version { get; private set; } = 1;

    internal int ActivePartitionCount => _activePartitions.Count;

    internal int InactivePartitionCount => _inactivePartitionPool.Count;

    internal int RetainedPartitionCount => _retainedPartitions.Count;

    internal int LastBroadPhaseCandidateCount { get; private set; }

    internal int SimulateCount { get; private set; }

    internal int LateSimulateCount { get; private set; }

    internal int VisualizeCount { get; private set; }

    internal int LateVisualizeCount { get; private set; }

    internal MixedColliderKey GetCandidate(int index) => _candidatePairs[index];

    internal void Simulate()
    {
        SimulateCount++;
        LastBroadPhaseCandidateCount = 0;
        _candidatePairs.FastClear();
        _processedPairKeys.Clear();

        Refresh3DColliderPartitions();
        Refresh2DColliderPartitions();

        Version++;
        _distributionPartitions.FastClear();
        foreach (PhysicsMixedPartition partition in _activePartitions)
            _distributionPartitions.Add(partition);

        SortPartitions(_distributionPartitions);
        for (int i = 0; i < _distributionPartitions.Count; i++)
        {
            _distributionPartitions[i].Distribute(
                _distributionDynamic3DIds,
                _distributionAwakeDynamic3DIds,
                _distributionStatic3DIds,
                _distributionDynamic2DIds,
                _distributionAwakeDynamic2DIds,
                _distributionStatic2DIds);
        }

        SortCandidatePairs(_candidatePairs);
        LastBroadPhaseCandidateCount = _candidatePairs.Count;
        RetireExpiredRetainedPartitions();
    }

    internal void LateSimulate()
    {
        LateSimulateCount++;
    }

    internal void Visualize()
    {
        VisualizeCount++;
    }

    internal void LateVisualize()
    {
        LateVisualizeCount++;
    }

    internal void Reset()
    {
        ClearRetainedPartitions();
        _activePartitions.Clear();
        _inactivePartitionPool.Clear();
        _redundancyChecker.Clear();
        _processedGrids.Clear();
        _processedPairKeys.Clear();
        _distributionPartitions.FastClear();
        _distributionDynamic3DIds.FastClear();
        _distributionAwakeDynamic3DIds.FastClear();
        _distributionStatic3DIds.FastClear();
        _distributionDynamic2DIds.FastClear();
        _distributionAwakeDynamic2DIds.FastClear();
        _distributionStatic2DIds.FastClear();
        _candidatePairs.FastClear();
        Version = 1;
        LastBroadPhaseCandidateCount = 0;
        SimulateCount = 0;
        LateSimulateCount = 0;
        VisualizeCount = 0;
        LateVisualizeCount = 0;
        _retainedPartitionRetirementCursor = 0;
    }

    internal void Deactivate() => Reset();

    internal bool Refresh3DColliderPartition(LSCollider collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "3D collider must belong to this mixed collision service context.");

        if (!collider.IsActive || collider.Shape == ColliderType.None)
        {
            if (collider.IsMixedPartitioned)
                ClearPartitioned3DCollider(collider, force: true);

            return false;
        }

        GetSnapped3DBounds(collider, out Vector3d snappedMin, out Vector3d snappedMax);
        if (collider.MatchesMixedPartitionGridBounds(snappedMin, snappedMax))
        {
            Refresh3DPartitionAwakeState(collider);
            return false;
        }

        if (collider.IsMixedPartitioned)
            ClearPartitioned3DCollider(collider, force: true);

        return Partition3DCollider(collider, snappedMin, snappedMax);
    }

    internal bool Refresh2DColliderPartition(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this mixed collision service context.");

        collider.Rebuild();
        if (!collider.IsActive || collider.Shape == ColliderType2D.None)
        {
            if (collider.IsMixedPartitioned)
                ClearPartitioned2DCollider(collider, force: true);

            return false;
        }

        GetSnapped2DMixedBounds(collider, out Vector3d snappedMin, out Vector3d snappedMax);
        if (collider.MatchesMixedPartitionGridBounds(snappedMin, snappedMax))
        {
            Refresh2DPartitionAwakeState(collider);
            return false;
        }

        if (collider.IsMixedPartitioned)
            ClearPartitioned2DCollider(collider, force: true);

        return Partition2DCollider(collider, snappedMin, snappedMax);
    }

    internal bool ClearPartitioned3DCollider(LSCollider collider, bool force = false)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        if (!collider.IsMixedPartitioned)
            return false;

        GetSnapped3DBounds(collider, out Vector3d snappedMin, out Vector3d snappedMax);
        if (!force && collider.MatchesMixedPartitionGridBounds(snappedMin, snappedMax))
            return false;

        SwiftList<WorldVoxelIndex>? coordinates = collider.MixedPartitionCoordinates;
        if (coordinates == null)
            return false;

        GridWorld world = _context.World;
        bool isStatic = IsStatic3DCollider(collider);
        try
        {
            for (int i = 0; i < coordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = coordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !_redundancyChecker.Add(voxel!.SpawnToken)
                    || !voxel.TryGetPartition(out PhysicsMixedPartition? partition))
                {
                    continue;
                }

                if (isStatic)
                    partition!.RemoveStatic3DObject(collider.Id);
                else
                    partition!.RemoveDynamic3DObject(collider.Id);
            }
        }
        finally
        {
            _redundancyChecker.Clear();
        }

        collider.MarkMixedUnpartitioned();
        collider.ClearMixedPartitionCoordinates();
        return true;
    }

    internal bool ClearPartitioned2DCollider(LSCollider2D collider, bool force = false)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        if (!collider.IsMixedPartitioned)
            return false;

        GetSnapped2DMixedBounds(collider, out Vector3d snappedMin, out Vector3d snappedMax);
        if (!force && collider.MatchesMixedPartitionGridBounds(snappedMin, snappedMax))
            return false;

        SwiftList<WorldVoxelIndex>? coordinates = collider.MixedPartitionCoordinates;
        if (coordinates == null)
            return false;

        GridWorld world = _context.World;
        bool isStatic = IsStatic2DCollider(collider);
        try
        {
            for (int i = 0; i < coordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = coordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !_redundancyChecker.Add(voxel!.SpawnToken)
                    || !voxel.TryGetPartition(out PhysicsMixedPartition? partition))
                {
                    continue;
                }

                if (isStatic)
                    partition!.RemoveStatic2DObject(collider.Id);
                else
                    partition!.RemoveDynamic2DObject(collider.Id);
            }
        }
        finally
        {
            _redundancyChecker.Clear();
        }

        collider.MarkMixedUnpartitioned();
        collider.ClearMixedPartitionCoordinates();
        return true;
    }

    internal void Refresh3DPartitionAwakeState(LSCollider collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        if (!collider.IsMixedPartitioned || collider.MixedPartitionCoordinates == null)
            return;

        StiffBody? body = collider.Body;
        if (body == null || body.Immovable)
            return;

        bool awake = body.IsAwakeForCollision;
        GridWorld world = _context.World;
        try
        {
            for (int i = 0; i < collider.MixedPartitionCoordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = collider.MixedPartitionCoordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !_redundancyChecker.Add(voxel!.SpawnToken)
                    || !voxel.TryGetPartition(out PhysicsMixedPartition? partition))
                {
                    continue;
                }

                partition!.SetDynamic3DObjectAwake(collider.Id, awake);
            }
        }
        finally
        {
            _redundancyChecker.Clear();
        }
    }

    internal void Refresh2DPartitionAwakeState(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        if (!collider.IsMixedPartitioned || collider.MixedPartitionCoordinates == null)
            return;

        StiffBody2D? body = collider.Body;
        if (body == null || body.Immovable)
            return;

        bool awake = body.IsAwakeForCollision;
        GridWorld world = _context.World;
        try
        {
            for (int i = 0; i < collider.MixedPartitionCoordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = collider.MixedPartitionCoordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !_redundancyChecker.Add(voxel!.SpawnToken)
                    || !voxel.TryGetPartition(out PhysicsMixedPartition? partition))
                {
                    continue;
                }

                partition!.SetDynamic2DObjectAwake(collider.Id, awake);
            }
        }
        finally
        {
            _redundancyChecker.Clear();
        }
    }

    internal void ProcessPartitionCandidate(int collider3DId, int collider2DId)
    {
        if (!_context.Physics.TryGetColliderById(collider3DId, out LSCollider? collider3D)
            || !_context.Physics2D.TryGetColliderById(collider2DId, out LSCollider2D? collider2D)
            || !RequireCollisionPair(collider3D!, collider2D!)
            || !MixedBoundsOverlap(collider3D!, collider2D!))
        {
            return;
        }

        ulong key = MixedColliderKey.CreateKey(collider3DId, collider2DId);
        if (!_processedPairKeys.Add(key))
            return;

        _candidatePairs.Add(new MixedColliderKey(collider3DId, collider2DId));
    }

    internal bool RequireCollisionPair(LSCollider collider3D, LSCollider2D collider2D)
    {
        return collider3D.IsActive && collider2D.IsActive
            && collider3D.Shape != ColliderType.None && collider2D.Shape != ColliderType2D.None
            && (collider3D.Body != null || collider2D.Body != null)
            && !ReferenceEquals(collider3D.AgentOrNull, collider2D.AgentOrNull)
            && !collider3D.ExcludesMixedCollisionWith(collider2D)
            && !_context.Physics.IsLayerCollisionDisabled(collider3D.Layer, collider2D.Layer);
    }

    internal int ActivatePartition(PhysicsMixedPartition partition)
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(partition.Owner, this),
            nameof(partition),
            "Mixed partition must belong to this collision service.");

        return _activePartitions.Add(partition);
    }

    internal void DeactivatePartition(int activationId)
    {
        if (activationId < 0)
            return;

        _activePartitions.TryRemoveAt(activationId);
    }

    internal PhysicsMixedPartition RentPartition()
    {
        PhysicsMixedPartition partition = _inactivePartitionPool.Count > 0
            ? _inactivePartitionPool.Pop()
            : new PhysicsMixedPartition();
        partition.SetOwner(this);
        return partition;
    }

    internal void ReleasePartition(PhysicsMixedPartition partition)
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(partition.Owner, this),
            nameof(partition),
            "Mixed partition must be released through its owning collision service.");

        UntrackRetainedPartition(partition);
        partition.ResetForPool();
        _inactivePartitionPool.Push(partition);
    }

    private void Refresh3DColliderPartitions()
    {
        int peak = _context.Physics.PeakColliderCount;
        for (int id = 1; id <= peak; id++)
            if (_context.Physics.TryGetColliderById(id, out LSCollider? collider))
                Refresh3DColliderPartition(collider!);
    }

    private void Refresh2DColliderPartitions()
    {
        int count = _context.Physics2D.ColliderCount;
        for (int i = 0; i < count; i++)
            if (_context.Physics2D.TryGetColliderByServiceIndex(i, out LSCollider2D? collider))
                Refresh2DColliderPartition(collider!);
    }

    private bool Partition3DCollider(LSCollider collider, Vector3d snappedMin, Vector3d snappedMax)
    {
        if (collider.IsMixedPartitioned || !collider.IsActive)
            return false;

        SwiftList<WorldVoxelIndex> coordinates = collider.GetOrCreateMixedPartitionCoordinates();
        coordinates.FastClear();

        try
        {
            ScanCovered3DVoxels(collider, snappedMin, snappedMax, coordinates);
            if (coordinates.Count == 0)
                return false;

            collider.MarkMixedPartitioned(snappedMin, snappedMax);
            return true;
        }
        finally
        {
            _redundancyChecker.Clear();
            _processedGrids.Clear();
        }
    }

    private bool Partition2DCollider(LSCollider2D collider, Vector3d snappedMin, Vector3d snappedMax)
    {
        if (collider.IsMixedPartitioned || !collider.IsActive)
            return false;

        SwiftList<WorldVoxelIndex> coordinates = collider.GetOrCreateMixedPartitionCoordinates();
        coordinates.FastClear();

        try
        {
            ScanCovered2DMixedVoxels(collider, snappedMin, snappedMax, coordinates);
            if (coordinates.Count == 0)
                return false;

            collider.MarkMixedPartitioned(snappedMin, snappedMax);
            return true;
        }
        finally
        {
            _redundancyChecker.Clear();
            _processedGrids.Clear();
        }
    }

    private void ScanCovered3DVoxels(
        LSCollider collider,
        Vector3d snappedMin,
        Vector3d snappedMax,
        SwiftList<WorldVoxelIndex> coordinates)
    {
        GridWorld world = _context.World;
        GetSpatialCellBounds(
            world,
            snappedMin,
            snappedMax,
            out int xMin,
            out int yMin,
            out int zMin,
            out int xMax,
            out int yMax,
            out int zMax);

        for (int z = zMin; z <= zMax; z++)
        {
            for (int y = yMin; y <= yMax; y++)
            {
                for (int x = xMin; x <= xMax; x++)
                {
                    int cellIndex = SwiftHashTools.CombineHashCodes(x, y, z);
                    if (!world.SpatialGridHash.TryGetValue(cellIndex, out SwiftHashSet<ushort> gridList))
                        continue;

                    foreach (ushort gridIndex in gridList)
                    {
                        if (!world.ActiveGrids.IsAllocated(gridIndex) || !_processedGrids.Add(gridIndex))
                            continue;

                        VoxelGrid currentGrid = world.ActiveGrids[gridIndex];
                        VisitGrid3DVoxels(currentGrid, collider, snappedMin, snappedMax, coordinates);
                    }
                }
            }
        }
    }

    private void ScanCovered2DMixedVoxels(
        LSCollider2D collider,
        Vector3d snappedMin,
        Vector3d snappedMax,
        SwiftList<WorldVoxelIndex> coordinates)
    {
        GridWorld world = _context.World;
        GetSpatialCellBounds(
            world,
            snappedMin,
            snappedMax,
            out int xMin,
            out int yMin,
            out int zMin,
            out int xMax,
            out int yMax,
            out int zMax);

        for (int z = zMin; z <= zMax; z++)
        {
            for (int y = yMin; y <= yMax; y++)
            {
                for (int x = xMin; x <= xMax; x++)
                {
                    int cellIndex = SwiftHashTools.CombineHashCodes(x, y, z);
                    if (!world.SpatialGridHash.TryGetValue(cellIndex, out SwiftHashSet<ushort> gridList))
                        continue;

                    foreach (ushort gridIndex in gridList)
                    {
                        if (!world.ActiveGrids.IsAllocated(gridIndex) || !_processedGrids.Add(gridIndex))
                            continue;

                        VoxelGrid currentGrid = world.ActiveGrids[gridIndex];
                        VisitGrid2DMixedVoxels(currentGrid, collider, snappedMin, snappedMax, coordinates);
                    }
                }
            }
        }
    }

    private void VisitGrid3DVoxels(
        VoxelGrid currentGrid,
        LSCollider collider,
        Vector3d snappedMin,
        Vector3d snappedMax,
        SwiftList<WorldVoxelIndex> coordinates)
    {
        Fixed64 voxelSize = _context.World.VoxelSize;
        for (Fixed64 x = snappedMin.x; x <= snappedMax.x; x += voxelSize)
        {
            for (Fixed64 y = snappedMin.y; y <= snappedMax.y; y += voxelSize)
            {
                for (Fixed64 z = snappedMin.z; z <= snappedMax.z; z += voxelSize)
                    TryPartition3DVoxel(currentGrid, collider, coordinates, new Vector3d(x, y, z), voxelSize);
            }
        }
    }

    private void VisitGrid2DMixedVoxels(
        VoxelGrid currentGrid,
        LSCollider2D collider,
        Vector3d snappedMin,
        Vector3d snappedMax,
        SwiftList<WorldVoxelIndex> coordinates)
    {
        Fixed64 voxelSize = _context.World.VoxelSize;
        for (Fixed64 x = snappedMin.x; x <= snappedMax.x; x += voxelSize)
        {
            for (Fixed64 y = snappedMin.y; y <= snappedMax.y; y += voxelSize)
            {
                for (Fixed64 z = snappedMin.z; z <= snappedMax.z; z += voxelSize)
                    TryPartition2DMixedVoxel(currentGrid, collider, coordinates, new Vector3d(x, y, z), voxelSize);
            }
        }
    }

    private void TryPartition3DVoxel(
        VoxelGrid currentGrid,
        LSCollider collider,
        SwiftList<WorldVoxelIndex> coordinates,
        Vector3d position,
        Fixed64 voxelSize)
    {
        if (!currentGrid.IsInBounds(position)
            || !currentGrid.TryGetVoxel(position, out Voxel? voxel)
            || !_redundancyChecker.Add(voxel!.SpawnToken)
            || !collider.IsPositionInBounds(voxelSize, voxel.WorldPosition))
        {
            return;
        }

        PhysicsMixedPartition partition = GetOrCreatePartition(voxel);
        coordinates.Add(voxel.WorldIndex);
        if (IsStatic3DCollider(collider))
            partition.AddStatic3DObject(collider.Id);
        else
            partition.AddDynamic3DObject(collider.Id);
    }

    private void TryPartition2DMixedVoxel(
        VoxelGrid currentGrid,
        LSCollider2D collider,
        SwiftList<WorldVoxelIndex> coordinates,
        Vector3d position,
        Fixed64 voxelSize)
    {
        if (!currentGrid.IsInBounds(position)
            || !currentGrid.TryGetVoxel(position, out Voxel? voxel)
            || !_redundancyChecker.Add(voxel!.SpawnToken)
            || !collider.IsPositionInMixedBounds(voxelSize, voxel.WorldPosition))
        {
            return;
        }

        PhysicsMixedPartition partition = GetOrCreatePartition(voxel);
        coordinates.Add(voxel.WorldIndex);
        if (IsStatic2DCollider(collider))
            partition.AddStatic2DObject(collider.Id);
        else
            partition.AddDynamic2DObject(collider.Id);
    }

    private PhysicsMixedPartition GetOrCreatePartition(Voxel voxel)
    {
        if (voxel.TryGetPartition(out PhysicsMixedPartition? partition))
            return partition!;

        partition = RentPartition();
        if (voxel.TryAddPartition(partition))
        {
            TrackRetainedPartition(partition);
            return partition;
        }

        ReleasePartition(partition);
        SwiftThrowHelper.ThrowIfTrue(
            true,
            nameof(GravitasMixedCollisionService),
            "Unable to attach mixed physics partition to voxel.");
        return partition;
    }

    private void GetSnapped3DBounds(LSCollider collider, out Vector3d snappedMin, out Vector3d snappedMax)
    {
        (snappedMin, snappedMax) =
            _context.World.SnapBoundsToVoxelSize(collider.BoundsMin, collider.BoundsMax, Fixed64.Half);
    }

    private void GetSnapped2DMixedBounds(LSCollider2D collider, out Vector3d snappedMin, out Vector3d snappedMax)
    {
        BoundingBox bounds = collider.MixedBounds3D;
        (snappedMin, snappedMax) =
            _context.World.SnapBoundsToVoxelSize(bounds.Min, bounds.Max, Fixed64.Half);
    }

    private static void GetSpatialCellBounds(
        GridWorld world,
        Vector3d min,
        Vector3d max,
        out int xMin,
        out int yMin,
        out int zMin,
        out int xMax,
        out int yMax,
        out int zMax)
    {
        SnapToSpatialGrid(world, min, out xMin, out yMin, out zMin);
        SnapToSpatialGrid(world, max, out xMax, out yMax, out zMax);

        if (xMin > xMax)
            (xMin, xMax) = (xMax, xMin);
        if (yMin > yMax)
            (yMin, yMax) = (yMax, yMin);
        if (zMin > zMax)
            (zMin, zMax) = (zMax, zMin);
    }

    private static void SnapToSpatialGrid(GridWorld world, Vector3d position, out int x, out int y, out int z)
    {
        x = (position.x.Abs() / world.SpatialGridCellSize).FloorToInt() * position.x.Sign();
        y = (position.y.Abs() / world.SpatialGridCellSize).FloorToInt() * position.y.Sign();
        z = (position.z.Abs() / world.SpatialGridCellSize).FloorToInt() * position.z.Sign();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsStatic3DCollider(LSCollider collider) => collider.Body == null || collider.Body.Immovable;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsStatic2DCollider(LSCollider2D collider) => collider.Body == null || collider.Body.Immovable;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool MixedBoundsOverlap(LSCollider collider3D, LSCollider2D collider2D)
    {
        BoundingBox bounds2D = collider2D.MixedBounds3D;
        return collider3D.BoundsMax.x >= bounds2D.Min.x
            && collider3D.BoundsMin.x <= bounds2D.Max.x
            && collider3D.BoundsMax.y >= bounds2D.Min.y
            && collider3D.BoundsMin.y <= bounds2D.Max.y
            && collider3D.BoundsMax.z >= bounds2D.Min.z
            && collider3D.BoundsMin.z <= bounds2D.Max.z;
    }

    private void ClearRetainedPartitions()
    {
        for (int i = 0; i < _retainedPartitions.Count; i++)
        {
            PhysicsMixedPartition partition = _retainedPartitions[i];
            if (partition.IsOwnedBy(this))
                partition.ResetRetainedMembership();
        }
    }

    private void TrackRetainedPartition(PhysicsMixedPartition partition)
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        SwiftThrowHelper.ThrowIfArgument(
            partition.RetainedIndex >= 0,
            nameof(partition),
            "PhysicsMixedPartition is already tracked as retained.");

        partition.SetRetainedIndex(_retainedPartitions.Count);
        _retainedPartitions.Add(partition);
    }

    private void UntrackRetainedPartition(PhysicsMixedPartition partition)
    {
        int index = FindRetainedPartitionIndex(partition);
        if (index < 0)
        {
            partition.ClearRetainedIndex();
            return;
        }

        int lastIndex = _retainedPartitions.Count - 1;
        if (index != lastIndex)
        {
            PhysicsMixedPartition movedPartition = _retainedPartitions[lastIndex];
            _retainedPartitions[index] = movedPartition;
            movedPartition.SetRetainedIndex(index);
        }

        _retainedPartitions.RemoveAt(lastIndex);
        partition.ClearRetainedIndex();

        if (_retainedPartitionRetirementCursor > index)
            _retainedPartitionRetirementCursor--;
        if (_retainedPartitionRetirementCursor >= _retainedPartitions.Count)
            _retainedPartitionRetirementCursor = 0;
    }

    private int FindRetainedPartitionIndex(PhysicsMixedPartition partition)
    {
        int index = partition.RetainedIndex;
        if ((uint)index < (uint)_retainedPartitions.Count && ReferenceEquals(_retainedPartitions[index], partition))
            return index;

        for (int i = 0; i < _retainedPartitions.Count; i++)
            if (ReferenceEquals(_retainedPartitions[i], partition))
                return i;

        return -1;
    }

    private void RetireExpiredRetainedPartitions()
    {
        int budget = _context.Settings.RetainedPartitionRetirementSweepBudget;
        if (budget <= 0 || _retainedPartitions.Count == 0)
            return;

        int inspected = 0;
        while (inspected < budget && _retainedPartitions.Count > 0)
        {
            if (_retainedPartitionRetirementCursor >= _retainedPartitions.Count)
                _retainedPartitionRetirementCursor = 0;

            PhysicsMixedPartition partition = _retainedPartitions[_retainedPartitionRetirementCursor];
            inspected++;

            if (!ShouldRetireRetainedPartition(partition))
            {
                _retainedPartitionRetirementCursor++;
                continue;
            }

            if (!RetireRetainedPartition(partition))
                _retainedPartitionRetirementCursor++;
        }
    }

    private bool ShouldRetireRetainedPartition(PhysicsMixedPartition partition)
    {
        if (!partition.IsOwnedBy(this) || !partition.IsEmpty || partition.IsAllocated || partition.EmptySinceFrame < 0)
            return false;

        int idleFrames = _context.FrameCount - partition.EmptySinceFrame;
        return idleFrames >= _context.Settings.RetainedPartitionTimeToKillFrames;
    }

    private bool RetireRetainedPartition(PhysicsMixedPartition partition)
    {
        if (!_context.World.TryGetVoxel(partition.WorldIndex, out Voxel? voxel))
        {
            ReleasePartition(partition);
            return true;
        }

        if (!voxel!.TryGetPartition(out PhysicsMixedPartition? attachedPartition)
            || !ReferenceEquals(attachedPartition, partition))
        {
            ReleasePartition(partition);
            return true;
        }

        return voxel.TryRemovePartition<PhysicsMixedPartition>();
    }

    private static void SortPartitions(SwiftList<PhysicsMixedPartition> partitions)
    {
        if (partitions.Count < 2)
            return;

        QuickSortPartitions(partitions, 0, partitions.Count - 1);
    }

    private static void QuickSortPartitions(SwiftList<PhysicsMixedPartition> partitions, int left, int right)
    {
        while (left < right)
        {
            if (right - left <= 16)
            {
                InsertionSortPartitions(partitions, left, right);
                return;
            }

            int i = left;
            int j = right;
            PhysicsMixedPartition pivot = partitions[left + ((right - left) / 2)];
            while (i <= j)
            {
                while (ComparePartitions(partitions[i], pivot) < 0)
                    i++;
                while (ComparePartitions(partitions[j], pivot) > 0)
                    j--;

                if (i > j)
                    continue;

                if (i != j)
                    (partitions[i], partitions[j]) = (partitions[j], partitions[i]);

                i++;
                j--;
            }

            if (j - left < right - i)
            {
                if (left < j)
                    QuickSortPartitions(partitions, left, j);

                left = i;
            }
            else
            {
                if (i < right)
                    QuickSortPartitions(partitions, i, right);

                right = j;
            }
        }
    }

    private static void InsertionSortPartitions(SwiftList<PhysicsMixedPartition> partitions, int left, int right)
    {
        for (int i = left + 1; i <= right; i++)
        {
            PhysicsMixedPartition value = partitions[i];
            int index = i - 1;
            while (index >= left && ComparePartitions(partitions[index], value) > 0)
            {
                partitions[index + 1] = partitions[index];
                index--;
            }

            partitions[index + 1] = value;
        }
    }

    private static int ComparePartitions(PhysicsMixedPartition left, PhysicsMixedPartition right)
    {
        WorldVoxelIndex leftIndex = left.WorldIndex;
        WorldVoxelIndex rightIndex = right.WorldIndex;

        int compare = leftIndex.GridIndex.CompareTo(rightIndex.GridIndex);
        if (compare != 0)
            return compare;

        compare = leftIndex.GridSpawnToken.CompareTo(rightIndex.GridSpawnToken);
        if (compare != 0)
            return compare;

        compare = leftIndex.VoxelIndex.x.CompareTo(rightIndex.VoxelIndex.x);
        if (compare != 0)
            return compare;

        compare = leftIndex.VoxelIndex.y.CompareTo(rightIndex.VoxelIndex.y);
        if (compare != 0)
            return compare;

        return leftIndex.VoxelIndex.z.CompareTo(rightIndex.VoxelIndex.z);
    }

    private static void SortCandidatePairs(SwiftList<MixedColliderKey> pairs)
    {
        if (pairs.Count < 2)
            return;

        QuickSortCandidatePairs(pairs, 0, pairs.Count - 1);
    }

    private static void QuickSortCandidatePairs(SwiftList<MixedColliderKey> pairs, int left, int right)
    {
        while (left < right)
        {
            if (right - left <= 16)
            {
                InsertionSortCandidatePairs(pairs, left, right);
                return;
            }

            int i = left;
            int j = right;
            MixedColliderKey pivot = pairs[left + ((right - left) / 2)];
            while (i <= j)
            {
                while (pairs[i].Key < pivot.Key)
                    i++;
                while (pairs[j].Key > pivot.Key)
                    j--;

                if (i > j)
                    continue;

                if (i != j)
                    (pairs[i], pairs[j]) = (pairs[j], pairs[i]);

                i++;
                j--;
            }

            if (j - left < right - i)
            {
                if (left < j)
                    QuickSortCandidatePairs(pairs, left, j);

                left = i;
            }
            else
            {
                if (i < right)
                    QuickSortCandidatePairs(pairs, i, right);

                right = j;
            }
        }
    }

    private static void InsertionSortCandidatePairs(SwiftList<MixedColliderKey> pairs, int left, int right)
    {
        for (int i = left + 1; i <= right; i++)
        {
            MixedColliderKey value = pairs[i];
            int index = i - 1;
            while (index >= left && pairs[index].Key > value.Key)
            {
                pairs[index + 1] = pairs[index];
                index--;
            }

            pairs[index + 1] = value;
        }
    }
}
