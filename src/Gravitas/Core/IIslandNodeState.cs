//=======================================================================
// IIslandNodeState.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas;

internal interface IIslandNodeState
{
    int BodyKey { get; }

    int ParentIndex { get; set; }

    int RootKey { get; set; }

    bool IsAwakeForCollision { get; }

    void WakeFromCollision();
}
