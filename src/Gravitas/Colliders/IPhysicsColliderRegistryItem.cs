//=======================================================================
// IPhysicsColliderRegistryItem.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

internal interface IPhysicsColliderRegistryItem
{
    int Id { get; }

    int ServiceIndex { get; }

    int ReplayOrder { get; }

    void SetRegistryState(int id, int serviceIndex, int replayOrder);

    void SetRegistryServiceIndex(int serviceIndex);

    void SetRegistryReplayOrdinal(int replayOrdinal);

    void ClearRegistryState();
}
