//=======================================================================
// GravitasQueryMixedService.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas.Queries;

/// <summary>
/// Owns explicit mixed 3D/2D query buffers for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed partial class GravitasQueryMixedService
{
    private readonly GravitasWorldContext _context;
    private readonly SwiftList<LSCollider2D> _candidates2D = new();
    private readonly SwiftList<LSCollider> _candidates3D = new();
    private readonly SwiftList<int> _meshTriangleCandidates = new();

    /// <summary>
    /// Creates a mixed query service owned by the specified world context.
    /// </summary>
    public GravitasQueryMixedService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    /// <summary>
    /// Gets the world context that owns this query service.
    /// </summary>
    public GravitasWorldContext Context => _context;

    internal int LastQueryCandidateCount { get; private set; }

    internal int LastMeshTriangleCandidateCount { get; private set; }

    /// <summary>
    /// Clears reusable mixed-query buffers and diagnostic counters.
    /// </summary>
    public void Reset()
    {
        _candidates2D.FastClear();
        _candidates3D.FastClear();
        _meshTriangleCandidates.FastClear();
        _batchMixedHits.FastClear();
        ResetLastQueryCounters();
        ResetBatchCounters(0);
    }

}
