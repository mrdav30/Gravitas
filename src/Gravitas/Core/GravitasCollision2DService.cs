using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using SwiftCollections.Utility;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Owns GridForge-backed pure 2D collision partitioning state for one context.
/// </summary>
public sealed class GravitasCollision2DService
{
    private const int DefaultPartitionPoolCapacity = 1024;

    private readonly GravitasWorldContext _context;
    private readonly SwiftBucket<PhysicsPartition2D> _activePartitions = new(DefaultPartitionPoolCapacity);
    private readonly SwiftStack<PhysicsPartition2D> _inactivePartitionPool = new(DefaultPartitionPoolCapacity);
    private readonly SwiftHashSet<int> _redundancyChecker = new();
    private readonly SwiftHashSet<ushort> _processedGrids = new();
    private readonly SwiftList<PhysicsPartition2D> _retainedPartitions = new();
    private readonly SwiftList<PhysicsPartition2D> _distributionPartitions = new();
    private readonly SwiftList<int> _distributionDynamicIds = new();
    private readonly SwiftList<int> _distributionAwakeDynamicIds = new();
    private readonly SwiftList<int> _distributionStaticIds = new();
    private readonly SwiftList<PhysicsPartition2D> _queryPartitions = new();
    private readonly SwiftList<int> _queryColliderIds = new();
    private readonly SwiftSparseSet _deferredPartitionRefreshIds = new();

    private int _retainedPartitionRetirementCursor;
    private bool _isDistributing;

    public GravitasCollision2DService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    public GravitasWorldContext Context => _context;

    public uint Version { get; private set; } = 1;

    public int ActivePartitionCount => _activePartitions.Count;

    public int InactivePartitionCount => _inactivePartitionPool.Count;

    internal int RetainedPartitionCount => _retainedPartitions.Count;

    public void Reset()
    {
        ClearRetainedPartitions();
        _activePartitions.Clear();
        _redundancyChecker.Clear();
        _processedGrids.Clear();
        _distributionPartitions.FastClear();
        _distributionDynamicIds.FastClear();
        _distributionAwakeDynamicIds.FastClear();
        _distributionStaticIds.FastClear();
        _queryPartitions.FastClear();
        _queryColliderIds.FastClear();
        _deferredPartitionRefreshIds.Clear();
        _inactivePartitionPool.Clear();
        Version = 1;
        _retainedPartitionRetirementCursor = 0;
        _isDistributing = false;
    }

    public void Deactivate() => Reset();

    internal bool RefreshColliderPartition(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this collision service context.");

        if (!collider.IsActive)
        {
            if (collider.IsPartitioned)
                ClearPartitionedCollider(collider, force: true);

            return false;
        }

        GetSnappedPlanarBounds(collider, out Vector2d snappedMin, out Vector2d snappedMax);
        if (collider.MatchesPartitionGridBounds(snappedMin, snappedMax))
            return false;

        if (collider.IsPartitioned)
            ClearPartitionedCollider(collider, force: true);

        return PartitionCollider(collider, snappedMin, snappedMax);
    }

    internal bool RefreshColliderPartitionAfterShapeChange(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        if (!_isDistributing)
        {
            if (collider.Id >= 0)
                _deferredPartitionRefreshIds.Remove(collider.Id);

            return RefreshColliderPartition(collider);
        }

        if (collider.Id >= 0)
            _deferredPartitionRefreshIds.Add(collider.Id);

        return false;
    }

    internal bool PartitionCollider(LSCollider2D collider)
    {
        GetSnappedPlanarBounds(collider, out Vector2d snappedMin, out Vector2d snappedMax);
        return PartitionCollider(collider, snappedMin, snappedMax);
    }

    private bool PartitionCollider(LSCollider2D collider, Vector2d snappedMin, Vector2d snappedMax)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this collision service context.");

        if (collider.IsPartitioned || !collider.IsActive)
            return false;

        SwiftList<WorldVoxelIndex> partitionedCoordinates = collider.GetOrCreatePartitionCoordinates();
        partitionedCoordinates.FastClear();

        try
        {
            PartitionCoveredVoxels(collider, snappedMin, snappedMax, partitionedCoordinates);
            if (partitionedCoordinates.Count == 0)
                return false;

            collider.MarkPartitioned(snappedMin, snappedMax);
            return true;
        }
        finally
        {
            _redundancyChecker.Clear();
            _processedGrids.Clear();
        }
    }

    internal bool ClearPartitionedCollider(LSCollider2D collider, bool force = false)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this collision service context.");

        if (!collider.IsPartitioned)
            return false;

        GetSnappedPlanarBounds(collider, out Vector2d snappedMin, out Vector2d snappedMax);
        if (!force && collider.MatchesPartitionGridBounds(snappedMin, snappedMax))
            return false;

        SwiftList<WorldVoxelIndex>? coordinates = collider.PartitionCoordinates;
        if (coordinates == null)
            return false;

        GridWorld world = _context.World;
        bool isStatic = IsStaticCollider(collider);
        try
        {
            for (int i = 0; i < coordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = coordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !_redundancyChecker.Add(voxel!.SpawnToken)
                    || !voxel.TryGetPartition(out PhysicsPartition2D? partition))
                {
                    continue;
                }

                if (isStatic)
                    partition!.RemoveStaticObject(collider.Id);
                else
                    partition!.RemoveDynamicObject(collider.Id);
            }
        }
        finally
        {
            _redundancyChecker.Clear();
        }

        collider.MarkUnpartitioned();
        collider.ClearPartitionCoordinates();
        return true;
    }

    internal void RefreshPartitionAwakeState(LSCollider2D collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this collision service context.");

        if (!collider.IsPartitioned || collider.PartitionCoordinates == null)
            return;

        StiffBody2D? body = collider.Body;
        if (body == null || body.Immovable)
            return;

        bool awake = body.IsAwakeForCollision;
        GridWorld world = _context.World;

        try
        {
            for (int i = 0; i < collider.PartitionCoordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = collider.PartitionCoordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !_redundancyChecker.Add(voxel!.SpawnToken)
                    || !voxel.TryGetPartition(out PhysicsPartition2D? partition))
                {
                    continue;
                }

                partition!.SetDynamicObjectAwake(collider.Id, awake);
            }
        }
        finally
        {
            _redundancyChecker.Clear();
        }
    }

    internal void CheckAndDistributeCollisions()
    {
        RefreshDeferredColliderPartitions();
        Version++;

        _distributionPartitions.FastClear();
        foreach (PhysicsPartition2D partition in _activePartitions)
            _distributionPartitions.Add(partition);

        SortPartitions(_distributionPartitions);

        _isDistributing = true;
        try
        {
            for (int i = 0; i < _distributionPartitions.Count; i++)
            {
                _distributionPartitions[i].Distribute(
                    _distributionDynamicIds,
                    _distributionAwakeDynamicIds,
                    _distributionStaticIds);
            }
        }
        finally
        {
            _isDistributing = false;
        }

        RetireExpiredRetainedPartitions();
    }

    internal void CollectOverlapCircleCandidates(
        Vector2d center,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        uint queryVersion,
        SwiftList<LSCollider2D> candidates)
    {
        candidates.FastClear();

        Vector2d min = new(center.X - radius, center.Y - radius);
        Vector2d max = new(center.X + radius, center.Y + radius);

        CollectBoundsCandidates(min, max, layerMask, queryVersion, raycastQuery: false, candidates);
    }

    internal void CollectBoundsCandidates(
        Vector2d min,
        Vector2d max,
        PhysicsLayerMask layerMask,
        uint queryVersion,
        bool raycastQuery,
        SwiftList<LSCollider2D> candidates)
    {
        RefreshDeferredColliderPartitions();
        candidates.FastClear();

        CollectCoveredPartitions(min, max, _queryPartitions);
        SortPartitions(_queryPartitions);

        for (int i = 0; i < _queryPartitions.Count; i++)
        {
            PhysicsPartition2D partition = _queryPartitions[i];
            partition.CopyAllColliderIds(_queryColliderIds);
            for (int j = 0; j < _queryColliderIds.Count; j++)
            {
                int colliderId = _queryColliderIds[j];
                if (!_context.Physics2D.TryGetColliderById(colliderId, out LSCollider2D? collider)
                    || !collider!.IsActive
                    || IsDuplicateQueryCandidate(collider, queryVersion, raycastQuery)
                    || !layerMask.Includes(collider.Layer)
                    || collider.MaxX < min.X
                    || collider.MinX > max.X
                    || collider.MaxY < min.Y
                    || collider.MinY > max.Y)
                {
                    continue;
                }

                candidates.Add(collider);
            }
        }

        SortCollidersById(candidates);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsDuplicateQueryCandidate(LSCollider2D collider, uint queryVersion, bool raycastQuery)
    {
        if (raycastQuery)
        {
            if (collider.RaycastVersion == queryVersion)
                return true;

            collider.RaycastVersion = queryVersion;
            return false;
        }

        if (collider.CircleQueryVersion == queryVersion)
            return true;

        collider.CircleQueryVersion = queryVersion;
        return false;
    }

    private void RefreshDeferredColliderPartitions()
    {
        if (_deferredPartitionRefreshIds.Count == 0)
            return;

        for (int i = 0; i < _deferredPartitionRefreshIds.Count; i++)
        {
            int colliderId = _deferredPartitionRefreshIds.DenseKeys[i];
            if (_context.Physics2D.TryGetColliderById(colliderId, out LSCollider2D? collider) && collider!.IsActive)
                RefreshColliderPartition(collider);
        }

        _deferredPartitionRefreshIds.Clear();
    }

    private void CollectCoveredPartitions(
        Vector2d min,
        Vector2d max,
        SwiftList<PhysicsPartition2D> partitions)
    {
        partitions.FastClear();
        (Vector2d snappedMin, Vector2d snappedMax) = SnapPlanarBounds(min, max);

        try
        {
            ScanCoveredQueryPartitions(snappedMin, snappedMax, min, max, partitions);
        }
        finally
        {
            _redundancyChecker.Clear();
            _processedGrids.Clear();
        }
    }

    private void PartitionCoveredVoxels(
        LSCollider2D collider,
        Vector2d snappedMin,
        Vector2d snappedMax,
        SwiftList<WorldVoxelIndex> partitionedCoordinates)
    {
        ScanCoveredColliderVoxels(collider, snappedMin, snappedMax, partitionedCoordinates);
    }

    private void ScanCoveredQueryPartitions(
        Vector2d snappedMin,
        Vector2d snappedMax,
        Vector2d queryMin,
        Vector2d queryMax,
        SwiftList<PhysicsPartition2D> partitions)
    {
        GridWorld world = _context.World;
        Vector3d snappedMin3 = ToWorldStoragePosition(snappedMin);
        Vector3d snappedMax3 = ToWorldStoragePosition(snappedMax);

        GetSpatialCellBounds(
            world,
            snappedMin3,
            snappedMax3,
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
                        VisitGridPlanarVoxelsForQuery(currentGrid, snappedMin, snappedMax, queryMin, queryMax, partitions);
                    }
                }
            }
        }
    }

    private void ScanCoveredColliderVoxels(
        LSCollider2D collider,
        Vector2d snappedMin,
        Vector2d snappedMax,
        SwiftList<WorldVoxelIndex> partitionedCoordinates)
    {
        GridWorld world = _context.World;
        Vector3d snappedMin3 = ToWorldStoragePosition(snappedMin);
        Vector3d snappedMax3 = ToWorldStoragePosition(snappedMax);

        GetSpatialCellBounds(
            world,
            snappedMin3,
            snappedMax3,
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
                        VisitGridPlanarVoxelsForCollider(currentGrid, collider, snappedMin, snappedMax, partitionedCoordinates);
                    }
                }
            }
        }
    }

    private void VisitGridPlanarVoxelsForQuery(
        VoxelGrid currentGrid,
        Vector2d snappedMin,
        Vector2d snappedMax,
        Vector2d queryMin,
        Vector2d queryMax,
        SwiftList<PhysicsPartition2D> partitions)
    {
        Fixed64 voxelSize = _context.World.VoxelSize;
        for (Fixed64 x = snappedMin.X; x <= snappedMax.X; x += voxelSize)
        {
            for (Fixed64 z = snappedMin.Y; z <= snappedMax.Y; z += voxelSize)
            {
                Vector3d position = new(x, Fixed64.Zero, z);
                if (!currentGrid.IsInBounds(position)
                    || !currentGrid.TryGetVoxel(position, out Voxel? voxel)
                    || !_redundancyChecker.Add(voxel!.SpawnToken)
                    || !IsPlanarPositionInBounds(queryMin, queryMax, voxelSize, voxel.WorldPosition)
                    || !voxel.TryGetPartition(out PhysicsPartition2D? partition)
                    || partition!.IsEmpty)
                {
                    continue;
                }

                partitions.Add(partition);
            }
        }
    }

    private void VisitGridPlanarVoxelsForCollider(
        VoxelGrid currentGrid,
        LSCollider2D collider,
        Vector2d snappedMin,
        Vector2d snappedMax,
        SwiftList<WorldVoxelIndex> partitionedCoordinates)
    {
        Fixed64 voxelSize = _context.World.VoxelSize;
        for (Fixed64 x = snappedMin.X; x <= snappedMax.X; x += voxelSize)
        {
            for (Fixed64 z = snappedMin.Y; z <= snappedMax.Y; z += voxelSize)
                TryPartitionVoxel(currentGrid, collider, partitionedCoordinates, new Vector3d(x, Fixed64.Zero, z), voxelSize);
        }
    }

    private void TryPartitionVoxel(
        VoxelGrid currentGrid,
        LSCollider2D collider,
        SwiftList<WorldVoxelIndex> partitionedCoordinates,
        Vector3d position,
        Fixed64 voxelSize)
    {
        if (!currentGrid.IsInBounds(position)
            || !currentGrid.TryGetVoxel(position, out Voxel? voxel)
            || !_redundancyChecker.Add(voxel!.SpawnToken)
            || !collider.IsPositionInPlanarBounds(voxelSize, voxel.WorldPosition))
        {
            return;
        }

        if (!voxel.TryGetPartition(out PhysicsPartition2D? partition))
        {
            partition = RentPartition();
            if (!voxel.TryAddPartition(partition))
            {
                ReleasePartition(partition);
                return;
            }

            TrackRetainedPartition(partition);
        }

        partitionedCoordinates.Add(voxel.WorldIndex);
        if (IsStaticCollider(collider))
            partition!.AddStaticObject(collider.Id);
        else
            partition!.AddDynamicObject(collider.Id);
    }

    private void GetSnappedPlanarBounds(LSCollider2D collider, out Vector2d snappedMin, out Vector2d snappedMax)
    {
        (snappedMin, snappedMax) = SnapPlanarBounds(
            new Vector2d(collider.MinX, collider.MinY),
            new Vector2d(collider.MaxX, collider.MaxY));
    }

    private (Vector2d min, Vector2d max) SnapPlanarBounds(Vector2d min, Vector2d max)
    {
        Fixed64 padding = _context.World.VoxelSize * Fixed64.Half;
        Vector3d boundsMin = new(min.X - padding, Fixed64.Zero, min.Y - padding);
        Vector3d boundsMax = new(max.X + padding, Fixed64.Zero, max.Y + padding);
        (Vector3d snappedMin, Vector3d snappedMax) = _context.World.SnapBoundsToVoxelSize(boundsMin, boundsMax);
        return (new Vector2d(snappedMin.X, snappedMin.Z), new Vector2d(snappedMax.X, snappedMax.Z));
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
        x = (position.X.Abs() / world.SpatialGridCellSize).FloorToInt() * position.X.Sign();
        y = (position.Y.Abs() / world.SpatialGridCellSize).FloorToInt() * position.Y.Sign();
        z = (position.Z.Abs() / world.SpatialGridCellSize).FloorToInt() * position.Z.Sign();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ToWorldStoragePosition(Vector2d position) =>
        new(position.X, Fixed64.Zero, position.Y);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsPlanarPositionInBounds(Vector2d min, Vector2d max, Fixed64 voxelSize, Vector3d worldPosition)
    {
        Fixed64 padding = voxelSize * Fixed64.Half;
        return worldPosition.X >= min.X - padding
            && worldPosition.X <= max.X + padding
            && worldPosition.Z >= min.Y - padding
            && worldPosition.Z <= max.Y + padding;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsStaticCollider(LSCollider2D collider) => collider.Body == null || collider.Body.Immovable;

    private void ClearRetainedPartitions()
    {
        for (int i = 0; i < _retainedPartitions.Count; i++)
        {
            PhysicsPartition2D partition = _retainedPartitions[i];
            if (partition.IsOwnedBy(this))
                partition.ResetRetainedMembership();
        }
    }

    private void TrackRetainedPartition(PhysicsPartition2D partition)
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        SwiftThrowHelper.ThrowIfArgument(
            partition.RetainedIndex >= 0,
            nameof(partition),
            "PhysicsPartition2D is already tracked as retained.");

        partition.SetRetainedIndex(_retainedPartitions.Count);
        _retainedPartitions.Add(partition);
    }

    private void UntrackRetainedPartition(PhysicsPartition2D partition)
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
            PhysicsPartition2D movedPartition = _retainedPartitions[lastIndex];
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

    private int FindRetainedPartitionIndex(PhysicsPartition2D partition)
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

            PhysicsPartition2D partition = _retainedPartitions[_retainedPartitionRetirementCursor];
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

    private bool ShouldRetireRetainedPartition(PhysicsPartition2D partition)
    {
        if (!partition.IsOwnedBy(this) || !partition.IsEmpty || partition.IsAllocated || partition.EmptySinceFrame < 0)
            return false;

        int idleFrames = _context.FrameCount - partition.EmptySinceFrame;
        return idleFrames >= _context.Settings.RetainedPartitionTimeToKillFrames;
    }

    private bool RetireRetainedPartition(PhysicsPartition2D partition)
    {
        if (!_context.World.TryGetVoxel(partition.WorldIndex, out Voxel? voxel))
        {
            ReleasePartition(partition);
            return true;
        }

        if (!voxel!.TryGetPartition(out PhysicsPartition2D? attachedPartition)
            || !ReferenceEquals(attachedPartition, partition))
        {
            ReleasePartition(partition);
            return true;
        }

        return voxel.TryRemovePartition<PhysicsPartition2D>();
    }

    internal int ActivatePartition(PhysicsPartition2D partition)
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(partition.Owner, this),
            nameof(partition),
            "2D partition must belong to this collision service.");

        return _activePartitions.Add(partition);
    }

    internal void DeactivatePartition(int activationId)
    {
        _activePartitions.TryRemoveAt(activationId);
    }

    internal PhysicsPartition2D RentPartition()
    {
        PhysicsPartition2D partition = _inactivePartitionPool.Count > 0
            ? _inactivePartitionPool.Pop()
            : new PhysicsPartition2D();
        partition.SetOwner(this);
        return partition;
    }

    internal void ReleasePartition(PhysicsPartition2D partition)
    {
        SwiftThrowHelper.ThrowIfNull(partition, nameof(partition));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(partition.Owner, this),
            nameof(partition),
            "2D partition must be released through its owning collision service.");

        UntrackRetainedPartition(partition);
        partition.ResetForPool();
        _inactivePartitionPool.Push(partition);
    }

    private static void SortPartitions(SwiftList<PhysicsPartition2D> partitions)
    {
        if (partitions.Count < 2)
            return;

        QuickSortPartitions(partitions, 0, partitions.Count - 1);
    }

    private static void QuickSortPartitions(SwiftList<PhysicsPartition2D> partitions, int left, int right)
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
            PhysicsPartition2D pivot = partitions[left + ((right - left) / 2)];
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

    private static void InsertionSortPartitions(SwiftList<PhysicsPartition2D> partitions, int left, int right)
    {
        for (int i = left + 1; i <= right; i++)
        {
            PhysicsPartition2D value = partitions[i];
            int index = i - 1;
            while (index >= left && ComparePartitions(partitions[index], value) > 0)
            {
                partitions[index + 1] = partitions[index];
                index--;
            }

            partitions[index + 1] = value;
        }
    }

    private static void SortCollidersById(SwiftList<LSCollider2D> colliders)
    {
        if (colliders.Count < 2)
            return;

        QuickSortCollidersById(colliders, 0, colliders.Count - 1);
    }

    private static void QuickSortCollidersById(SwiftList<LSCollider2D> colliders, int left, int right)
    {
        while (left < right)
        {
            if (right - left <= 16)
            {
                InsertionSortCollidersById(colliders, left, right);
                return;
            }

            int i = left;
            int j = right;
            LSCollider2D pivot = colliders[left + ((right - left) / 2)];
            while (i <= j)
            {
                while (colliders[i].Id < pivot.Id)
                    i++;
                while (colliders[j].Id > pivot.Id)
                    j--;

                if (i > j)
                    continue;

                if (i != j)
                    (colliders[i], colliders[j]) = (colliders[j], colliders[i]);

                i++;
                j--;
            }

            if (j - left < right - i)
            {
                if (left < j)
                    QuickSortCollidersById(colliders, left, j);

                left = i;
            }
            else
            {
                if (i < right)
                    QuickSortCollidersById(colliders, i, right);

                right = j;
            }
        }
    }

    private static void InsertionSortCollidersById(SwiftList<LSCollider2D> colliders, int left, int right)
    {
        for (int i = left + 1; i <= right; i++)
        {
            LSCollider2D value = colliders[i];
            int index = i - 1;
            while (index >= left && colliders[index].Id > value.Id)
            {
                colliders[index + 1] = colliders[index];
                index--;
            }

            colliders[index + 1] = value;
        }
    }

    private static int ComparePartitions(PhysicsPartition2D left, PhysicsPartition2D right)
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

        compare = leftIndex.VoxelIndex.z.CompareTo(rightIndex.VoxelIndex.z);
        if (compare != 0)
            return compare;

        return leftIndex.VoxelIndex.y.CompareTo(rightIndex.VoxelIndex.y);
    }
}
