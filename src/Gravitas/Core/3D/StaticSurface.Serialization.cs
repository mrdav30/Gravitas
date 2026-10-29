// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using Chronicler;
using Gravitas.Colliders;
using System;
using System.Globalization;

namespace Gravitas;

/// <content>Records declared membership using host-resolved links to existing collider lifetimes.</content>
public sealed partial class StaticSurface
{
    /// <inheritdoc/>
    public void RecordData(IChronicler chronicler)
    {
        ChronicleGuard.ThrowIfNull(chronicler, nameof(chronicler));
        _context.ThrowIfDisposed();
        if (_disposed) throw new ObjectDisposedException(nameof(StaticSurface));
        bool loading = chronicler.Mode == SerializationMode.Loading;
        if (loading) _context.ThrowIfFixedStepMutationNotAllowed();
        int schema = 1;
        RecordValues.Look(chronicler, ref schema, "SurfaceSchemaVersion", 0);
        if (schema != 1) throw new InvalidOperationException("Unsupported static-surface membership schema.");
        int count = _members.Length;
        RecordValues.Look(chronicler, ref count, "MemberCount", -1);
        if (count < 0 || count > _context.Physics.ColliderCount)
            throw new ArgumentOutOfRangeException(nameof(count), "Membership must fit the registered collider graph.");
        LSMeshCollider[] members = loading ? new LSMeshCollider[count] : _members;
        for (int index = 0; index < count; index++)
        {
            LSMeshCollider member = members[index];
            RecordLinks.Look(chronicler, ref member, "Member" + index.ToString(CultureInfo.InvariantCulture));
            if (loading) members[index] = member;
        }
        // Resolve the entire candidate graph before publishing any association.
        if (loading) ReplaceMembers(members);
    }
}
