//=======================================================================
// StaticSurface.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;
using SwiftCollections.Query;
using System;

namespace Gravitas;

/// <summary>Internal staged continuity declaration; public exposure requires query, CCD and restore parity.</summary>
internal sealed class StaticSurface : IDisposable
{
    private readonly GravitasWorldContext _context;
    private LSMeshCollider[] _members = Array.Empty<LSMeshCollider>();
    private PhysicsMesh[] _meshes = Array.Empty<PhysicsMesh>();
    private readonly SwiftDictionary<(PhysicsMesh, int), MeshSurfaceBoundaryDomain> _boundaries = new();
    private readonly SwiftList<int> _candidates = new(), _triangles = new();
    private readonly SwiftList<(LSMeshCollider Collider, int Triangle)> _surfaces = new();
    private readonly SwiftHashSet<(PhysicsMesh, int)> _owners = new();
    private readonly SwiftList<(PhysicsMesh, int)> _cachedOwners = new();
    private Vector3d _center;
    private FixedQuaternion _rotation;
    private Fixed64 _coreLength, _radius;
    private bool _facesCurrent;
    private bool _dirty, _disposed;
    internal readonly MeshCapsuleFaceRegions Faces = new();

    internal StaticSurface(GravitasWorldContext context, ReadOnlySpan<LSMeshCollider> members)
    {
        _context = context;
        ReplaceMembers(members);
    }

    internal void ReplaceMembers(ReadOnlySpan<LSMeshCollider> members)
    {
        _context.ThrowIfDisposed();
        _context.ThrowIfFixedStepMutationNotAllowed();
        if (_disposed) throw new ObjectDisposedException(nameof(StaticSurface));
        var next = members.ToArray();
        var meshes = new PhysicsMesh[next.Length];
        var unique = new SwiftHashSet<LSMeshCollider>();
        for (int index = 0; index < next.Length; index++)
        {
            LSMeshCollider member = next[index];
            // Registry identity verifies context and rejects stale or recycled IDs.
            if (member == null || !unique.Add(member)
                || !_context.Physics.TryGetColliderById(member.Id, out var registered) || registered != member
                || !member.IsStatic || !member.IsActive || member.IsDeactivationInProgress
                || member.Mode != MeshColliderMode.Concave
                || member.StaticSurface != null && member.StaticSurface != this)
                throw new ArgumentException("Members must be distinct registered static concave meshes in this context.", nameof(members));
            meshes[index] = member.Mesh;
        }
        // Prepare before publishing associations so failed admission leaves the old declaration usable.
        var prepared = PrepareBoundaries(meshes);
        Invalidate();
        foreach (LSMeshCollider previous in _members) previous.StaticSurface = null;
        _members = next; _meshes = meshes;
        Faces.ReleaseJoined(); _surfaces.Clear(); _cachedOwners.Clear(); _owners.Clear();
        _boundaries.Clear();
        foreach (var entry in prepared) _boundaries.Add(entry.Key, entry.Value);
        foreach (LSMeshCollider member in _members) member.StaticSurface = this;
        Invalidate();
        _dirty = false;
    }

    private static SwiftDictionary<(PhysicsMesh, int), MeshSurfaceBoundaryDomain> PrepareBoundaries(PhysicsMesh[] meshes)
    {
        var result = new SwiftDictionary<(PhysicsMesh, int), MeshSurfaceBoundaryDomain>();
        for (int member = 0; member < meshes.Length; member++)
        {
            PhysicsMesh mesh = meshes[member];
            for (int triangle = 0; triangle < mesh.TriangleCount; triangle++)
            {
                int owner = mesh.GetManifoldSurfaceOwner(triangle);
                if (result.ContainsKey((mesh, owner))) continue;
                var domain = new MeshSurfaceBoundaryDomain();
                domain.Build(meshes, member, triangle);
                result.Add((mesh, owner), domain);
            }
        }
        return result;
    }

    internal void Invalidate(bool geometryChanged = false)
    {
        _dirty |= geometryChanged;
        _facesCurrent = false;
        foreach (LSMeshCollider member in _members) member.MarkContactDependencyChanged(notifySurface: false);
    }

    internal void Remove(LSMeshCollider member)
    {
        int index = Array.IndexOf(_members, member);
        if (index < 0) return;
        Invalidate(geometryChanged: true);
        member.StaticSurface = null;
        Faces.ReleaseJoined(); _surfaces.Clear(); _cachedOwners.Clear(); _owners.Clear(); _boundaries.Clear();
        Array.Copy(_members, index + 1, _members, index, _members.Length - index - 1);
        Array.Resize(ref _members, _members.Length - 1);
        _meshes = new PhysicsMesh[_members.Length];
        for (int i = 0; i < _members.Length; i++) _meshes[i] = _members[i].Mesh;
    }

    internal bool Gather(LSCollider mover, FixedBoundVolume bounds)
    {
        if (_dirty)
        {
            var prepared = PrepareBoundaries(_meshes);
            _boundaries.Clear();
            foreach (var entry in prepared) _boundaries.Add(entry.Key, entry.Value);
            _dirty = false;
        }
        _surfaces.Clear(); _owners.Clear();
        LSCollider participant = mover.CompoundOwner ?? mover;
        // Reuse the committed index; Y rejection keeps its planar candidates conservative in 3D.
        _context.Collisions.QueryPlanarColliderCandidates(new(bounds.Min.X, bounds.Min.Z, bounds.Max.X, bounds.Max.Z), _candidates);
        foreach (int id in _candidates.AsReadOnlySpan())
        {
            _context.Physics.TryGetColliderById(id, out var candidate);
            if (candidate is not LSMeshCollider mesh
                || mesh.StaticSurface != this || mesh.IsTrigger
                || !_context.Physics.RequireCollisionPair(mesh, participant)
                || mesh.BoundsMax.Y < bounds.Min.Y || mesh.BoundsMin.Y > bounds.Max.Y) continue;
            mesh.GetTrianglesInBounds(bounds, _triangles);
            foreach (int triangle in _triangles.AsReadOnlySpan())
                if (_owners.Add((mesh.Mesh, mesh.Mesh.GetManifoldSurfaceOwner(triangle)))) _surfaces.Add((mesh, triangle));
        }
        return _surfaces.Count > 1;
    }

    internal void BuildRounded(Vector3d center, FixedQuaternion rotation, Fixed64 coreLength, Fixed64 radius)
    {
        // Gather rechecks live eligibility. Reuse requires the complete owner set, not a hash of it.
        bool reuse = _facesCurrent && _center == center && _rotation == rotation && _coreLength == coreLength
            && _radius == radius && _cachedOwners.Count == _owners.Count;
        foreach (var owner in _cachedOwners.AsReadOnlySpan()) reuse &= _owners.Contains(owner);
        if (reuse) return;
        _facesCurrent = false;
        Faces.BuildJoined(_surfaces.AsReadOnlySpan(), center, rotation, coreLength, radius, _meshes, _boundaries);
        _cachedOwners.Clear();
        foreach (var surface in _surfaces.AsReadOnlySpan())
            _cachedOwners.Add((surface.Collider.Mesh, surface.Collider.Mesh.GetManifoldSurfaceOwner(surface.Triangle)));
        _center = center; _rotation = rotation; _coreLength = coreLength; _radius = radius;
        _facesCurrent = true;
    }

    public void Dispose()
    {
        if (_disposed) return;
        _context.ThrowIfFixedStepMutationNotAllowed();
        Release();
        _context.Physics.RemoveStaticSurface(this);
    }

    internal void Release()
    {
        Invalidate();
        foreach (LSMeshCollider member in _members) member.StaticSurface = null;
        _members = Array.Empty<LSMeshCollider>(); _meshes = Array.Empty<PhysicsMesh>();
        _boundaries.Clear(); _surfaces.Clear(); _cachedOwners.Clear(); _owners.Clear();
        Faces.ReleaseJoined(); _disposed = true;
    }
}
