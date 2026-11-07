//=======================================================================
// IslandNodeKeyComparer.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Collections.Generic;

namespace Gravitas;

internal sealed class IslandNodeKeyComparer<TNode> : IComparer<TNode>
    where TNode : struct, IIslandNodeState
{
    public int Compare(TNode left, TNode right) =>
        left.BodyKey.CompareTo(right.BodyKey);
}
