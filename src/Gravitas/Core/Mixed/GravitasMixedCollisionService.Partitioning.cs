//=======================================================================
// GravitasMixedCollisionService.Partitioning.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Colliders;
using Gravitas.Support;
using GridForge.Grids;
using GridForge.Spatial;
using GridForge.Utility;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

internal sealed partial class GravitasMixedCollisionService
{
    internal bool Refresh3DColliderPartition(LSCollider collider)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "3D collider must belong to this mixed collision service context.");

        if (!collider.IsActive || collider.Shape == ColliderType.None)
        {
            ClearPartitioned3DCollider(collider, force: true);
            collider.MarkMixedUnpartitioned();
            collider.ClearMixedPartitionCoordinates();

            return false;
        }

        Get3DCoverageBounds(collider, out Vector3d coverageMin, out Vector3d coverageMax);
        MixedPartitionMobilityKind kind = Get3DMobilityKind(collider);
        if (collider.MatchesMixedPartitionGridBounds(coverageMin, coverageMax, (int)kind))
        {
            Refresh3DPartitionAwakeState(collider);
            return false;
        }

        if (collider.IsMixedPartitioned)
            ClearPartitioned3DCollider(collider, force: true);

        return Partition3DCollider(collider, coverageMin, coverageMax);
    }

    internal bool Refresh2DColliderPartition(LSCollider2D collider) =>
        Refresh2DColliderPartition(collider, rebuildShape: true);

    private bool Refresh2DColliderPartition(LSCollider2D collider, bool rebuildShape)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(collider.Context, _context),
            nameof(collider),
            "2D collider must belong to this mixed collision service context.");

        if (rebuildShape)
            collider.Rebuild();

        if (!collider.IsActive || collider.Shape == ColliderType2D.None)
        {
            ClearPartitioned2DCollider(collider, force: true);
            collider.MarkMixedUnpartitioned();
            collider.ClearMixedPartitionCoordinates();

            return false;
        }

        Get2DMixedCoverageBounds(collider, out Vector3d coverageMin, out Vector3d coverageMax);
        MixedPartitionMobilityKind kind = Get2DMobilityKind(collider);
        if (collider.MatchesMixedPartitionGridBounds(coverageMin, coverageMax, (int)kind))
        {
            Refresh2DPartitionAwakeState(collider);
            return false;
        }

        if (collider.IsMixedPartitioned)
            ClearPartitioned2DCollider(collider, force: true);

        return Partition2DCollider(collider, coverageMin, coverageMax);
    }

    internal bool ClearPartitioned3DCollider(LSCollider collider, bool force = false)
    {
        SwiftThrowHelper.ThrowIfNull(collider, nameof(collider));
        if (!collider.IsMixedPartitioned)
            return false;

        Get3DCoverageBounds(collider, out Vector3d coverageMin, out Vector3d coverageMax);
        MixedPartitionMobilityKind currentKind = Get3DMobilityKind(collider);
        if (!force && collider.MatchesMixedPartitionGridBounds(coverageMin, coverageMax, (int)currentKind))
            return false;

        SwiftList<WorldVoxelIndex> coordinates = collider.MixedPartitionCoordinates!;
        GridWorld world = _context.World;
        MixedPartitionMobilityKind partitionKind = GetStoredMobilityKind(collider.MixedPartitionKind);
        try
        {
            for (int i = 0; i < coordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = coordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !GridTraversal.TryGetUniquePartition(voxel!, _redundancyChecker, out PhysicsMixedPartition? partition))
                {
                    continue;
                }

                Remove3DObject(partition!, collider.Id, partitionKind);
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

        Get2DMixedCoverageBounds(collider, out Vector3d coverageMin, out Vector3d coverageMax);
        MixedPartitionMobilityKind currentKind = Get2DMobilityKind(collider);
        if (!force && collider.MatchesMixedPartitionGridBounds(coverageMin, coverageMax, (int)currentKind))
            return false;

        SwiftList<WorldVoxelIndex> coordinates = collider.MixedPartitionCoordinates!;
        GridWorld world = _context.World;
        MixedPartitionMobilityKind partitionKind = GetStoredMobilityKind(collider.MixedPartitionKind);
        try
        {
            for (int i = 0; i < coordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = coordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !GridTraversal.TryGetUniquePartition(voxel!, _redundancyChecker, out PhysicsMixedPartition? partition))
                {
                    continue;
                }

                Remove2DObject(partition!, collider.Id, partitionKind);
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
        if (!collider.IsMixedPartitioned)
            return;

        SwiftList<WorldVoxelIndex> coordinates = collider.MixedPartitionCoordinates!;
        SolidBody? body = collider.Body;
        if (collider.IsStatic)
            return;

        bool awake = body!.IsAwakeForCollision;
        GridWorld world = _context.World;
        try
        {
            for (int i = 0; i < coordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = coordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !GridTraversal.TryGetUniquePartition(voxel!, _redundancyChecker, out PhysicsMixedPartition? partition))
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
        if (!collider.IsMixedPartitioned)
            return;

        SwiftList<WorldVoxelIndex> coordinates = collider.MixedPartitionCoordinates!;
        SolidBody2D? body = collider.Body;
        if (collider.IsStatic)
            return;

        bool awake = body!.IsAwakeForCollision;
        GridWorld world = _context.World;
        try
        {
            for (int i = 0; i < coordinates.Count; i++)
            {
                WorldVoxelIndex coordinate = coordinates[i];
                if (!world.TryGetVoxel(coordinate, out Voxel? voxel)
                    || !GridTraversal.TryGetUniquePartition(voxel!, _redundancyChecker, out PhysicsMixedPartition? partition))
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

    internal void Collect2DCandidatesInMixedBounds(
        Vector3d min,
        Vector3d max,
        PhysicsLayerMask layerMask,
        SwiftList<LSCollider2D> candidates,
        bool staticStyleOnly = false,
        bool cachePartitionRefresh = false)
    {
        SwiftThrowHelper.ThrowIfNull(candidates, nameof(candidates));
        Refresh2DColliderPartitionsForQuery(cachePartitionRefresh);
        candidates.FastClear();
        FixedBoundBox queryBounds = FixedBoundBox.FromMinMax(min, max);
        CollectCoveredMixedQueryPartitions(min, max, _queryPartitions);
        _queryPartitions.SortInPlace(PartitionOrderComparer);
        _queryColliderRedundancy.Clear();

        for (int i = 0; i < _queryPartitions.Count; i++)
        {
            PhysicsMixedPartition partition = _queryPartitions[i];
            if (staticStyleOnly)
                partition.CopyStaticStyle2DColliderIds(_queryColliderIds);
            else
                partition.Copy2DColliderIds(_queryColliderIds);

            for (int j = 0; j < _queryColliderIds.Count; j++)
            {
                int colliderId = _queryColliderIds[j];
                if (!_queryColliderRedundancy.Add(colliderId)
                    || !_context.Physics2D.TryGetColliderById(colliderId, out LSCollider2D? collider)
                    || !Is2DQueryCandidate(collider!, queryBounds, layerMask))
                {
                    continue;
                }

                candidates.Add(collider!);
            }
        }

        _queryColliderRedundancy.Clear();
        candidates.SortInPlace(Collider2DIdOrderComparer);
    }

    internal void Collect3DCandidatesInMixedBounds(
        Vector3d min,
        Vector3d max,
        PhysicsLayerMask layerMask,
        SwiftList<LSCollider> candidates,
        bool staticStyleOnly = false,
        bool cachePartitionRefresh = false)
    {
        SwiftThrowHelper.ThrowIfNull(candidates, nameof(candidates));
        Refresh3DColliderPartitionsForQuery(cachePartitionRefresh);
        candidates.FastClear();
        FixedBoundBox queryBounds = FixedBoundBox.FromMinMax(min, max);
        CollectCoveredMixedQueryPartitions(min, max, _queryPartitions);
        _queryPartitions.SortInPlace(PartitionOrderComparer);
        _queryColliderRedundancy.Clear();

        for (int i = 0; i < _queryPartitions.Count; i++)
        {
            PhysicsMixedPartition partition = _queryPartitions[i];
            if (staticStyleOnly)
                partition.CopyStaticStyle3DColliderIds(_queryColliderIds);
            else
                partition.Copy3DColliderIds(_queryColliderIds);

            for (int j = 0; j < _queryColliderIds.Count; j++)
            {
                int colliderId = _queryColliderIds[j];
                if (!_queryColliderRedundancy.Add(colliderId)
                    || !_context.Physics.TryGetColliderById(colliderId, out LSCollider? collider)
                    || !Is3DQueryCandidate(collider!, queryBounds, layerMask))
                {
                    continue;
                }

                candidates.Add(collider!);
            }
        }

        _queryColliderRedundancy.Clear();
        candidates.SortInPlace(Collider3DIdOrderComparer);
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
        if (_inactivePartitionPool.Count == 0)
            TryRetireEmptyRetainedPartitionForReuse();

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
        int count = _context.Physics.ColliderCount;
        for (int i = 0; i < count; i++)
            Refresh3DColliderPartition(_context.Physics.GetColliderByServiceIndex(i));
    }

    private void Refresh3DColliderPartitionsForQuery(bool cachePartitionRefresh)
    {
        if (!cachePartitionRefresh)
        {
            Refresh3DColliderPartitions();
            return;
        }

        int frame = _context.FrameCount;
        int lateToken = _context.LateSimulateToken;
        // CCD batches query stable opposite-dimension targets many times in one late-sim phase.
        // Cache only the partition refresh; every sweep still gathers and sorts its own hits.
        if (_cached3DQueryRefreshFrame == frame && _cached3DQueryRefreshLateToken == lateToken)
            return;

        Refresh3DColliderPartitions();
        _cached3DQueryRefreshFrame = frame;
        _cached3DQueryRefreshLateToken = lateToken;
    }

    private void Refresh2DColliderPartitions(bool rebuildShapes = true)
    {
        int count = _context.Physics2D.ColliderCount;
        for (int i = 0; i < count; i++)
            Refresh2DColliderPartition(_context.Physics2D.GetColliderByServiceIndex(i), rebuildShapes);
    }

    private void Refresh2DColliderPartitionsForQuery(bool cachePartitionRefresh)
    {
        if (!cachePartitionRefresh)
        {
            Refresh2DColliderPartitions();
            return;
        }

        int frame = _context.FrameCount;
        int lateToken = _context.LateSimulateToken;
        // CCD batches query stable opposite-dimension targets many times in one late-sim phase.
        // Cache only the partition refresh; every sweep still gathers and sorts its own hits.
        if (_cached2DQueryRefreshFrame == frame && _cached2DQueryRefreshLateToken == lateToken)
            return;

        Refresh2DColliderPartitions();
        _cached2DQueryRefreshFrame = frame;
        _cached2DQueryRefreshLateToken = lateToken;
    }

    private bool Partition3DCollider(LSCollider collider, Vector3d coverageMin, Vector3d coverageMax)
    {
        MixedPartitionMobilityKind kind = Get3DMobilityKind(collider);
        SwiftList<WorldVoxelIndex> coordinates = collider.GetOrCreateMixedPartitionCoordinates();
        coordinates.FastClear();

        try
        {
            ScanCovered3DVoxels(collider, coverageMin, coverageMax, coordinates, kind);
            if (coordinates.Count == 0)
                return false;

            collider.MarkMixedPartitioned(coverageMin, coverageMax, (int)kind);
            return true;
        }
        finally
        {
            _redundancyChecker.Clear();
        }
    }

    private bool Partition2DCollider(LSCollider2D collider, Vector3d coverageMin, Vector3d coverageMax)
    {
        MixedPartitionMobilityKind kind = Get2DMobilityKind(collider);
        SwiftList<WorldVoxelIndex> coordinates = collider.GetOrCreateMixedPartitionCoordinates();
        coordinates.FastClear();

        try
        {
            ScanCovered2DMixedVoxels(collider, coverageMin, coverageMax, coordinates, kind);
            if (coordinates.Count == 0)
                return false;

            collider.MarkMixedPartitioned(coverageMin, coverageMax, (int)kind);
            return true;
        }
        finally
        {
            _redundancyChecker.Clear();
        }
    }

    private void ScanCovered3DVoxels(
        LSCollider collider,
        Vector3d coverageMin,
        Vector3d coverageMax,
        SwiftList<WorldVoxelIndex> coordinates,
        MixedPartitionMobilityKind kind)
    {
        GridWorld world = _context.World;
        GridTracer.GetCoveredVoxelsInto(
            world,
            coverageMin,
            coverageMax,
            _coveredVoxels,
            _traceScratch,
            Fixed64.Half);

        var traversal = new GridTraversalState(world, GridTraversalPaddingMode.MaxCellEdge);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            TryPartition3DVoxel(collider, coordinates, _coveredVoxels[i], ref traversal, kind);
    }

    private void ScanCovered2DMixedVoxels(
        LSCollider2D collider,
        Vector3d coverageMin,
        Vector3d coverageMax,
        SwiftList<WorldVoxelIndex> coordinates,
        MixedPartitionMobilityKind kind)
    {
        GridWorld world = _context.World;
        GridTracer.GetCoveredVoxelsInto(
            world,
            coverageMin,
            coverageMax,
            _coveredVoxels,
            _traceScratch,
            Fixed64.Half);

        var traversal = new GridTraversalState(world, GridTraversalPaddingMode.MaxCellEdge);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            TryPartition2DMixedVoxel(collider, coordinates, _coveredVoxels[i], ref traversal, kind);
    }

    private void CollectCoveredMixedQueryPartitions(
        Vector3d min,
        Vector3d max,
        SwiftList<PhysicsMixedPartition> partitions)
    {
        partitions.FastClear();

        try
        {
            ScanCoveredMixedQueryPartitions(min, max, partitions);
        }
        finally
        {
            _redundancyChecker.Clear();
        }
    }

    private void ScanCoveredMixedQueryPartitions(
        Vector3d queryMin,
        Vector3d queryMax,
        SwiftList<PhysicsMixedPartition> partitions)
    {
        GridTracer.GetCoveredVoxelsInto(
            _context.World,
            queryMin,
            queryMax,
            _coveredVoxels,
            _traceScratch,
            Fixed64.Half);

        VisitCoveredVoxelsForMixedQuery(queryMin, queryMax, partitions);
    }

    private void VisitCoveredVoxelsForMixedQuery(
        Vector3d queryMin,
        Vector3d queryMax,
        SwiftList<PhysicsMixedPartition> partitions)
    {
        GridWorld world = _context.World;
        var traversal = new GridTraversalState(world, GridTraversalPaddingMode.MaxCellEdge);
        for (int i = 0; i < _coveredVoxels.Count; i++)
        {
            Voxel voxel = _coveredVoxels[i];

            if (!traversal.TryVisitUnique(voxel, _redundancyChecker, out Fixed64 cellEdge)
                || !GridTraversal.IsWorldPositionInPaddedBounds(queryMin, queryMax, cellEdge, voxel.WorldPosition)
                || !voxel.TryGetPartition(out PhysicsMixedPartition? partition)
                || partition!.IsEmpty)
            {
                continue;
            }

            partitions.Add(partition);
        }
    }

    private void TryPartition3DVoxel(
        LSCollider collider,
        SwiftList<WorldVoxelIndex> coordinates,
        Voxel voxel,
        ref GridTraversalState traversal,
        MixedPartitionMobilityKind kind)
    {
        if (!traversal.TryVisitUnique(voxel, _redundancyChecker, out Fixed64 cellEdge)
            || !collider.IsPositionInBounds(cellEdge, voxel.WorldPosition))
        {
            return;
        }

        PhysicsMixedPartition partition = GetOrCreatePartition(voxel);
        coordinates.Add(voxel.WorldIndex);
        Add3DObject(partition, collider.Id, kind);
    }

    private void TryPartition2DMixedVoxel(
        LSCollider2D collider,
        SwiftList<WorldVoxelIndex> coordinates,
        Voxel voxel,
        ref GridTraversalState traversal,
        MixedPartitionMobilityKind kind)
    {
        if (!traversal.TryVisitUnique(voxel, _redundancyChecker, out Fixed64 cellEdge)
            || !collider.IsPositionInMixedBounds(cellEdge, voxel.WorldPosition))
        {
            return;
        }

        PhysicsMixedPartition partition = GetOrCreatePartition(voxel);
        coordinates.Add(voxel.WorldIndex);
        Add2DObject(partition, collider.Id, kind);
    }

    private PhysicsMixedPartition GetOrCreatePartition(Voxel voxel)
    {
        if (voxel.TryGetPartition(out PhysicsMixedPartition? partition))
            return partition!;

        partition = RentPartition();
        if (!voxel.TryAddPartition(partition))
        {
            ReleasePartition(partition);
            SwiftThrowHelper.ThrowIfTrue(
                true,
                nameof(GravitasMixedCollisionService),
                "Unable to attach mixed physics partition to voxel.");
        }

        TrackRetainedPartition(partition);
        return partition;
    }

    private void Get3DCoverageBounds(LSCollider collider, out Vector3d coverageMin, out Vector3d coverageMax)
    {
        coverageMin = collider.BoundsMin;
        coverageMax = collider.BoundsMax;
    }

    private void Get2DMixedCoverageBounds(LSCollider2D collider, out Vector3d coverageMin, out Vector3d coverageMax)
    {
        FixedBoundBox bounds = collider.MixedBounds3D;
        coverageMin = bounds.Min;
        coverageMax = bounds.Max;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static MixedPartitionMobilityKind Get3DMobilityKind(LSCollider collider)
    {
        if (collider.IsStatic)
            return MixedPartitionMobilityKind.Static;

        SolidBody? body = collider.Body;
        return body!.IsKinematic ? MixedPartitionMobilityKind.Kinematic : MixedPartitionMobilityKind.Dynamic;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static MixedPartitionMobilityKind Get2DMobilityKind(LSCollider2D collider)
    {
        if (collider.IsStatic)
            return MixedPartitionMobilityKind.Static;

        SolidBody2D? body = collider.Body;
        return body!.IsKinematic ? MixedPartitionMobilityKind.Kinematic : MixedPartitionMobilityKind.Dynamic;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static MixedPartitionMobilityKind GetStoredMobilityKind(int partitionKind)
    {
        return partitionKind == (int)MixedPartitionMobilityKind.Kinematic
            ? MixedPartitionMobilityKind.Kinematic
            : partitionKind == (int)MixedPartitionMobilityKind.Static
                ? MixedPartitionMobilityKind.Static
                : MixedPartitionMobilityKind.Dynamic;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Add3DObject(PhysicsMixedPartition partition, int id, MixedPartitionMobilityKind kind)
    {
        if (kind == MixedPartitionMobilityKind.Static)
        {
            partition.AddStatic3DObject(id);
            return;
        }

        if (kind == MixedPartitionMobilityKind.Kinematic)
        {
            partition.AddKinematic3DObject(id);
            return;
        }

        partition.AddDynamic3DObject(id);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Add2DObject(PhysicsMixedPartition partition, int id, MixedPartitionMobilityKind kind)
    {
        if (kind == MixedPartitionMobilityKind.Static)
        {
            partition.AddStatic2DObject(id);
            return;
        }

        if (kind == MixedPartitionMobilityKind.Kinematic)
        {
            partition.AddKinematic2DObject(id);
            return;
        }

        partition.AddDynamic2DObject(id);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Remove3DObject(PhysicsMixedPartition partition, int id, MixedPartitionMobilityKind kind)
    {
        if (kind == MixedPartitionMobilityKind.Static)
        {
            partition.RemoveStatic3DObject(id);
            return;
        }

        if (kind == MixedPartitionMobilityKind.Kinematic)
        {
            partition.RemoveKinematic3DObject(id);
            return;
        }

        partition.RemoveDynamic3DObject(id);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Remove2DObject(PhysicsMixedPartition partition, int id, MixedPartitionMobilityKind kind)
    {
        if (kind == MixedPartitionMobilityKind.Static)
        {
            partition.RemoveStatic2DObject(id);
            return;
        }

        if (kind == MixedPartitionMobilityKind.Kinematic)
        {
            partition.RemoveKinematic2DObject(id);
            return;
        }

        partition.RemoveDynamic2DObject(id);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool MixedBoundsOverlap(LSCollider collider3D, LSCollider2D collider2D)
    {
        return collider3D.Bounds.Intersects(collider2D.MixedBounds3D);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool Is2DQueryCandidate(LSCollider2D collider, FixedBoundBox queryBounds, PhysicsLayerMask layerMask)
    {
        if (!collider.IsActive || !layerMask.Includes(collider.Layer))
            return false;

        return collider.MixedBounds3D.Intersects(queryBounds);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool Is3DQueryCandidate(LSCollider collider, FixedBoundBox queryBounds, PhysicsLayerMask layerMask)
    {
        return collider.IsActive
            && layerMask.Includes(collider.Layer)
            && collider.Bounds.Intersects(queryBounds);
    }

    private void DetachRetainedPartitions() => RetainedPartitionLifecycle.DetachAll(
            _retainedPartitions,
            _context.World,
            this,
            _releaseRetainedPartition,
            nameof(PhysicsMixedPartition),
            "Unable to detach retained mixed physics partition from its voxel during reset.");

    private void TrackRetainedPartition(PhysicsMixedPartition partition) => RetainedPartitionLifecycle.Track(
            _retainedPartitions,
            this,
            partition,
            nameof(PhysicsMixedPartition));

    private void UntrackRetainedPartition(PhysicsMixedPartition partition) => RetainedPartitionLifecycle.Untrack(
            _retainedPartitions,
            this,
            partition,
            ref _retainedPartitionRetirementCursor);

    private void RetireExpiredRetainedPartitions() => RetainedPartitionLifecycle.RetireExpired(
            _retainedPartitions,
            _context.World,
            this,
            _context.Settings.RetainedPartitionRetirementSweepBudget,
            _context.FrameCount,
            _context.Settings.RetainedPartitionTimeToKillFrames,
            _releaseRetainedPartition,
            ref _retainedPartitionRetirementCursor);

    private bool TryRetireEmptyRetainedPartitionForReuse() => RetainedPartitionLifecycle.TryRetireEmptyForReuse(
            _retainedPartitions,
            _inactivePartitionPool,
            _context.World,
            this,
            _releaseRetainedPartition,
            ref _retainedPartitionRetirementCursor);

}
