//=======================================================================
// MatrixRow.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Text.Json.Serialization;
using MemoryPack;

namespace Gravitas;

/// <summary>
/// Stores one serialized row of a physics collision matrix.
/// </summary>
[Serializable]
[MemoryPackable]
public partial struct MatrixRow
{
    /// <summary>
    /// Collision enablement values for this matrix row.
    /// </summary>
    [JsonInclude]
    public bool[] row;
}
