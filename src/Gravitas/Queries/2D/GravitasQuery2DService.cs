//=======================================================================
// GravitasQuery2DService.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas.Queries;

/// <summary>
/// Owns pure 2D query buffers and query dispatch for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed partial class GravitasQuery2DService
{
    private readonly GravitasWorldContext _context;
    private readonly SwiftList<LSCollider2D> _queryCandidates = new();
    private uint _overlapQueryVersion;
    private uint _raycastVersion;

    /// <summary>
    /// Initializes a pure 2D query service for the supplied context.
    /// </summary>
    /// <param name="context">The owning world context.</param>
    public GravitasQuery2DService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    /// <summary>
    /// Gets the owning world context.
    /// </summary>
    public GravitasWorldContext Context => _context;

    internal int LastQueryCandidateCount { get; private set; }

    /// <summary>
    /// Resets context-local pure 2D query buffers.
    /// </summary>
    public void Reset()
    {
        _queryCandidates.FastClear();
        _batch2DHits.FastClear();
        LastQueryCandidateCount = 0;
        ResetBatchCounters(0);
        _overlapQueryVersion = 0;
        _raycastVersion = 0;
    }

}
