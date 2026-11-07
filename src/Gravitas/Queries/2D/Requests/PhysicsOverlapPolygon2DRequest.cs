//=======================================================================
// PhysicsOverlapPolygon2DRequest.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Support;

namespace Gravitas.Queries;

/// <summary>
/// Describes one pure 2D convex polygon overlap by referencing a flat vertex buffer.
/// </summary>
public readonly struct PhysicsOverlapPolygon2DRequest
{
    /// <summary>
    /// Creates a pure 2D convex-polygon overlap request with an include layer mask.
    /// </summary>
    public PhysicsOverlapPolygon2DRequest(int vertexStart, int vertexCount, PhysicsLayerMask layerMask)
    {
        VertexStart = vertexStart;
        VertexCount = vertexCount;
        LayerMask = layerMask;
    }

    /// <summary>
    /// Creates a pure 2D convex-polygon overlap request against all physics layers.
    /// </summary>
    public PhysicsOverlapPolygon2DRequest(int vertexStart, int vertexCount)
        : this(vertexStart, vertexCount, PhysicsLayerMask.All)
    {
    }

    /// <summary>
    /// Gets the first polygon vertex index in the batch's shared vertex buffer.
    /// </summary>
    public int VertexStart { get; }

    /// <summary>
    /// Gets the number of polygon vertices in the shared vertex buffer.
    /// </summary>
    public int VertexCount { get; }

    /// <summary>
    /// Gets the included physics layers.
    /// </summary>
    public PhysicsLayerMask LayerMask { get; }
}
