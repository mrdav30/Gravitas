//=======================================================================
// LSCollider.Serialization.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using Gravitas.Materials;
using Gravitas.Support;

namespace Gravitas.Colliders;

public abstract partial class LSCollider
{
    public void RecordData(IChronicler chronicler)
    {
        RecordValues.Look(chronicler, ref _debug, "Debug", false);
        RecordValues.Look(chronicler, ref _drawShape, "DrawShape", false);
        RecordValues.Look(chronicler, ref _drawPartitions, "DrawPartitions", false);
        RecordValues.Look(chronicler, ref _drawBoundingBox, "DrawBoundingBox", false);
        RecordValues.Look(chronicler, ref _active, "Active", true);
        RecordValues.Look(chronicler, ref _layer, "Layer", new());
        RecordValues.Look(chronicler, ref _ignoredCollisionLayers, "IgnoredCollisionLayers", PhysicsLayerMask.None);
        RecordValues.Look(chronicler, ref _material, "Material", PhysicsMaterial.Default);
        RecordValues.Look(chronicler, ref _isTrigger, "IsTrigger", false);
        RecordValues.Look(chronicler, ref _offset, "Offset", Vector3d.Zero);
        RecordValues.Look(chronicler, ref _radius, "Radius", Fixed64.Half);
        RecordValues.Look(chronicler, ref _size, "Size", Vector3d.One);

        if (chronicler.Mode == SerializationMode.Loading)
        {
            ThrowIfLoadedTriggerHasBody(nameof(IsTrigger));
            ApplyLoadedState();
        }
        else
        {
            _runtimeShapeState.MarkDirty();
        }
    }

    private void ApplyLoadedState()
    {
        _runtimeShapeState.MarkDirty();
        if (_context == null)
            return;

        RebuildRuntimeShapeState();

        if (!_active)
        {
            if (IsPartitioned)
                _context.Collisions.ClearPartitionedObject(this, force: true);

            if (IsMixedPartitioned)
                _context.MixedCollisions.ClearPartitioned3DCollider(this, force: true);
            return;
        }

        if (IsPartitioned)
            UpdatePartition();
        else
            InitialPartition();
        if (_context.Settings.RuntimeMode.RunsMixedContacts())
            _context.MixedCollisions.Refresh3DColliderPartition(this);
    }
}
