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

/// <summary>Internal mesh continuity declaration with shared response, continuation and membership.</summary>
/// <content>Owns declared member lifetime and cached finite response families.</content>
internal sealed partial class StaticSurface : IDisposable, Chronicler.IRecordable
{
    private readonly GravitasWorldContext _context;
    private LSMeshCollider[] _members = Array.Empty<LSMeshCollider>();
    private PhysicsMesh[] _meshes = Array.Empty<PhysicsMesh>();
    private FixedBoundBox[] _memberBounds = Array.Empty<FixedBoundBox>();
    private readonly SwiftHashSet<int> _changedSuppliers = new();
    private readonly SwiftHashSet<PhysicsMesh> _dirtyMeshes = new();
    private readonly SwiftDictionary<(PhysicsMesh, int), MeshSurfaceBoundaryDomain> _boundaries = new();
    private readonly SwiftList<int> _candidates = new(), _triangles = new();
    private readonly SwiftList<(LSMeshCollider Collider, int Triangle)> _surfaces = new();
    private readonly SwiftHashSet<(PhysicsMesh, int)> _owners = new();
    private readonly SwiftList<(PhysicsMesh, int)> _cachedOwners = new();
    private Vector3d _center;
    private FixedQuaternion _rotation;
    private Fixed64 _coreLength, _radius;
    private enum ContactFamily { Rounded, Cone, Slab, Hull }
    private ContactFamily _cachedFamily;
    private bool _facesCurrent;
    private Vector2d _halfFootprint;
    private MeshSlabFaceRegions? _slabFaces;
    private MeshHullSurfaceContacts? _hullContacts;
    private PhysicsMesh? _movingMesh;
    private uint _movingVersion;
    private bool _dirty, _disposed;
    internal readonly MeshCapsuleFaceRegions Faces = new();
    internal readonly MeshConeFaceRegions ConeFaces = new();
    internal MeshSlabFaceRegions SlabFaces => _slabFaces ??= new();
    internal MeshHullSurfaceContacts HullContacts => _hullContacts ??= new();
    internal int ReplayMembership => _members.Length == 0 ? 0 : _members[0].ReplayOrdinal + 1;

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
        }
        bool unchanged = next.Length == _members.Length;
        foreach (LSMeshCollider member in _members) unchanged &= unique.Contains(member);
        if (unchanged) return;
        // Logical live registration order survives recycled registry-slot layouts.
        Array.Sort(next, static (first, second) => ((IPhysicsColliderRegistryItem)first).ReplayOrder
            .CompareTo(((IPhysicsColliderRegistryItem)second).ReplayOrder));
        for (int index = 0; index < next.Length; index++) meshes[index] = next[index].Mesh;
        // Prepare before publishing associations so failed admission leaves the old declaration usable.
        var prepared = PrepareBoundaries(meshes);
        var bounds = new FixedBoundBox[next.Length];
        for (int index = 0; index < next.Length; index++) bounds[index] = next[index].Bounds;
        LSMeshCollider[] previousMembers = _members;
        foreach (LSMeshCollider previous in previousMembers)
            if (!unique.Contains(previous)) Invalidate(changed: previous);
        foreach (LSMeshCollider previous in previousMembers) previous.StaticSurface = null;
        _members = next; _meshes = meshes; _memberBounds = bounds;
        _dirtyMeshes.Clear();
        _continuousBoundaries.Clear(); Faces.ReleaseJoined(); ConeFaces.ReleaseJoined(); _slabFaces?.ReleaseJoined(); _hullContacts?.Release(); _movingMesh = null; _surfaces.Clear(); _cachedOwners.Clear(); _owners.Clear();
        _boundaries.Clear();
        foreach (var entry in prepared) _boundaries.Add(entry.Key, entry.Value);
        foreach (LSMeshCollider member in _members) member.StaticSurface = this;
        foreach (LSMeshCollider member in _members)
            if (Array.IndexOf(previousMembers, member) < 0) Invalidate(changed: member);
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

    internal void Invalidate(bool geometryChanged = false, LSMeshCollider? changed = null, bool includeNeighbors = true)
    {
        _dirty |= geometryChanged;
        _facesCurrent = false;
        _mixedSlab = false;
        _changedSuppliers.Clear();
        FixedBoundBox before = default, after = default;
        if (changed != null)
        {
            int index = Array.IndexOf(_members, changed);
            before = _memberBounds[index]; after = changed.Bounds;
            _memberBounds[index] = after;
        }
        for (int index = 0; index < _members.Length; index++)
            if (changed == null || _members[index] == changed || includeNeighbors
                && (_memberBounds[index].Intersects(before) || _memberBounds[index].Intersects(after)))
            {
                _changedSuppliers.Add(_members[index].Id + 1);
                _members[index].MarkContactDependencyChanged(notifySurface: false);
                if (geometryChanged) _dirtyMeshes.Add(_meshes[index]);
            }
        // Borrowed rows can live on a carrier without an overlap pair for the
        // edited supplier. Scan existing references only when a member changes.
        foreach (LSMeshCollider member in _members)
            _context.Physics.ClearWarmStartCachesForCollider(member, _changedSuppliers);
        _context.MixedCollisions.InvalidateSurfaceContacts(this, null, _changedSuppliers);
        _changedSuppliers.Clear();
    }

    internal void Remove(LSMeshCollider member)
    {
        int index = Array.IndexOf(_members, member);
        if (index < 0) return;
        Invalidate(geometryChanged: true, changed: member);
        member.StaticSurface = null;
        _continuousBoundaries.Clear(); Faces.ReleaseJoined(); ConeFaces.ReleaseJoined(); _slabFaces?.ReleaseJoined(); _hullContacts?.Release(); _movingMesh = null; _surfaces.Clear(); _cachedOwners.Clear(); _owners.Clear(); _boundaries.Clear();
        Array.Copy(_members, index + 1, _members, index, _members.Length - index - 1);
        Array.Copy(_memberBounds, index + 1, _memberBounds, index, _memberBounds.Length - index - 1);
        Array.Resize(ref _members, _members.Length - 1);
        Array.Resize(ref _memberBounds, _members.Length);
        _meshes = new PhysicsMesh[_members.Length];
        for (int i = 0; i < _members.Length; i++) _meshes[i] = _members[i].Mesh;
        _dirtyMeshes.Clear();
        foreach (PhysicsMesh mesh in _meshes) _dirtyMeshes.Add(mesh);
    }

    internal int Gather(LSCollider mover, FixedBoundVolume bounds) => Gather(mover, null, bounds);

    private int Gather(LSCollider? mover, LSCollider2D? embedded, FixedBoundVolume bounds)
    {
        if (_dirty)
        {
            for (int member = 0; member < _meshes.Length; member++)
            {
                PhysicsMesh mesh = _meshes[member];
                if (!_dirtyMeshes.Contains(mesh)) continue;
                for (int triangle = 0; triangle < mesh.TriangleCount; triangle++)
                {
                    int owner = mesh.GetManifoldSurfaceOwner(triangle);
                    if (owner != triangle) continue;
                    if (!_boundaries.TryGetValue((mesh, owner), out MeshSurfaceBoundaryDomain domain))
                        _boundaries.Add((mesh, owner), domain = new());
                    domain.Build(_meshes, member, triangle);
                }
            }
            _dirtyMeshes.Clear();
            _dirty = false;
        }
        _surfaces.Clear(); _owners.Clear();
        LSCollider? participant = mover?.CompoundOwner ?? mover;
        LSCollider2D? planar = embedded?.CompoundOwner ?? embedded;
        // Reuse the committed index; Y rejection keeps its planar candidates conservative in 3D.
        _context.Collisions.QueryPlanarColliderCandidates(new(bounds.Min.X, bounds.Min.Z, bounds.Max.X, bounds.Max.Z), _candidates);
        foreach (int id in _candidates.AsReadOnlySpan())
        {
            _context.Physics.TryGetColliderById(id, out var candidate);
            if (candidate is not LSMeshCollider mesh
                || mesh.StaticSurface != this || mesh.IsTrigger
                || (participant != null ? !_context.Physics.RequireCollisionPair(mesh, participant)
                    : !_context.MixedCollisions.RequireCollisionPair(mesh, planar!))
                || mesh.BoundsMax.Y < bounds.Min.Y || mesh.BoundsMin.Y > bounds.Max.Y) continue;
            mesh.GetTrianglesInBounds(bounds, _triangles);
            foreach (int triangle in _triangles.AsReadOnlySpan())
                if (_owners.Add((mesh.Mesh, mesh.Mesh.GetManifoldSurfaceOwner(triangle)))) _surfaces.Add((mesh, triangle));
        }
        return _surfaces.Count;
    }

    internal int GatherProjectedSupport(LSCollider mover)
    {
        // A support projected onto an intersecting plane is within sqrt(2) R
        // of the shape center. 2 R conservatively includes its finite supplier,
        // even when that piece has no initial overlap or broad-phase pair.
        Fixed64 reach = mover.CanonicalCenteredProxyRadius * Fixed64.Two;
        FixedBoundBox bounds = FixedBoundBox.FromCenterAndScopeClippedToDomain(mover.Center, Vector3d.One * reach);
        return Gather(mover, new(bounds.Min, bounds.Max));
    }

    internal void BuildHull(LSMeshCollider mover)
    {
        bool reuse = _facesCurrent && _cachedFamily == ContactFamily.Hull && _movingMesh == mover.Mesh
            && _movingVersion == mover.RuntimeShapeVersion && _center == mover.Center && _rotation == mover.Rotation
            && _cachedOwners.Count == _owners.Count;
        foreach (var owner in _cachedOwners.AsReadOnlySpan()) reuse &= _owners.Contains(owner);
        if (reuse) return;
        _facesCurrent = false; _cachedFamily = ContactFamily.Hull;
        HullContacts.Build(_surfaces.AsReadOnlySpan(), mover, _meshes, _boundaries);
        _cachedOwners.Clear();
        foreach (var surface in _surfaces.AsReadOnlySpan())
            _cachedOwners.Add((surface.Collider.Mesh, surface.Collider.Mesh.GetManifoldSurfaceOwner(surface.Triangle)));
        _movingMesh = mover.Mesh; _movingVersion = mover.RuntimeShapeVersion;
        _center = mover.Center; _rotation = mover.Rotation; _facesCurrent = true;
    }

    internal void BuildRounded(Vector3d center, FixedQuaternion rotation, Fixed64 coreLength, Fixed64 radius)
    {
        // Gather rechecks live eligibility. Reuse requires the complete owner set, not a hash of it.
        bool reuse = _facesCurrent && _cachedFamily == ContactFamily.Rounded && _center == center && _rotation == rotation && _coreLength == coreLength
            && _radius == radius && _cachedOwners.Count == _owners.Count;
        foreach (var owner in _cachedOwners.AsReadOnlySpan()) reuse &= _owners.Contains(owner);
        if (reuse) return;
        _facesCurrent = false; _cachedFamily = ContactFamily.Rounded;
        Faces.BuildJoined(_surfaces.AsReadOnlySpan(), center, rotation, coreLength, radius, _meshes, _boundaries);
        _cachedOwners.Clear();
        foreach (var surface in _surfaces.AsReadOnlySpan())
            _cachedOwners.Add((surface.Collider.Mesh, surface.Collider.Mesh.GetManifoldSurfaceOwner(surface.Triangle)));
        _center = center; _rotation = rotation; _coreLength = coreLength; _radius = radius;
        _facesCurrent = true;
    }

    internal void BuildCone(LSConeCollider cone)
    {
        bool reuse = _facesCurrent && _cachedFamily == ContactFamily.Cone && _center == cone.Center && _rotation == cone.Rotation
            && _coreLength == cone.Height && _radius == cone.ScaledRadius && _cachedOwners.Count == _owners.Count;
        foreach (var owner in _cachedOwners.AsReadOnlySpan()) reuse &= _owners.Contains(owner);
        if (reuse) return;
        _facesCurrent = false; _cachedFamily = ContactFamily.Cone;
        ConeFaces.BuildJoined(_surfaces.AsReadOnlySpan(), cone, _meshes, _boundaries);
        _cachedOwners.Clear();
        foreach (var surface in _surfaces.AsReadOnlySpan())
            _cachedOwners.Add((surface.Collider.Mesh, surface.Collider.Mesh.GetManifoldSurfaceOwner(surface.Triangle)));
        _center = cone.Center; _rotation = cone.Rotation; _coreLength = cone.Height; _radius = cone.ScaledRadius;
        _facesCurrent = true;
    }

    internal void BuildSlab(LSCollider mover, Fixed64 radius, Fixed64 halfThickness, Vector2d halfFootprint)
    {
        bool reuse = _facesCurrent && _cachedFamily == ContactFamily.Slab && !_mixedSlab && _center == mover.Center && _rotation == mover.Rotation
            && _coreLength == halfThickness && _radius == radius && _halfFootprint == halfFootprint && _cachedOwners.Count == _owners.Count;
        foreach (var owner in _cachedOwners.AsReadOnlySpan()) reuse &= _owners.Contains(owner);
        if (reuse) return;
        _facesCurrent = false; _cachedFamily = ContactFamily.Slab;
        _mixedSlab = false;
        SlabFaces.BuildJoined(_surfaces.AsReadOnlySpan(), mover, radius, halfThickness, halfFootprint, _meshes, _boundaries);
        _cachedOwners.Clear();
        foreach (var surface in _surfaces.AsReadOnlySpan())
            _cachedOwners.Add((surface.Collider.Mesh, surface.Collider.Mesh.GetManifoldSurfaceOwner(surface.Triangle)));
        _center = mover.Center; _rotation = mover.Rotation; _coreLength = halfThickness; _radius = radius; _halfFootprint = halfFootprint;
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
        _memberBounds = Array.Empty<FixedBoundBox>(); _dirtyMeshes.Clear();
        _boundaries.Clear(); _surfaces.Clear(); _cachedOwners.Clear(); _owners.Clear();
        _continuousBoundaries.Clear(); Faces.ReleaseJoined(); ConeFaces.ReleaseJoined(); _slabFaces?.ReleaseJoined(); _hullContacts?.Release(); _movingMesh = null; _disposed = true;
    }
}
