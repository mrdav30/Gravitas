//=======================================================================
// IIslandNodeState.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas;

/// <summary>
/// Exposes body identity, disjoint-set state, and wake behavior for a collision island node.
/// </summary>
internal interface IIslandNodeState
{
    int BodyKey { get; }

    int ParentIndex { get; set; }

    int RootKey { get; set; }

    bool IsAwakeForCollision { get; }

    void WakeFromCollision();
}
