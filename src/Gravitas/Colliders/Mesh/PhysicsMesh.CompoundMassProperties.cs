//=======================================================================
// PhysicsMesh.CompoundMassProperties.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

public partial class PhysicsMesh
{
    // Allocate retained moments only when inertia is requested. Each physical
    // policy keeps its geometry across alternating requests and pose changes.
    private CompoundMassCache? _compoundMassCache;

    private sealed class CompoundMassCache
    {
        internal readonly MeshInertiaPolicy Policy;
        internal MeshMassIntegrals Current, Prepared;
        internal bool CurrentValid, PreparedValid;
        internal CompoundMassCache? Alternate;
        internal MeshRuntimeInertia? Runtime;

        internal CompoundMassCache(MeshInertiaPolicy policy) => Policy = policy;
    }

    private CompoundMassCache GetMassCache(MeshInertiaPolicy policy)
    {
        if (_compoundMassCache == null)
            _compoundMassCache = new CompoundMassCache(policy);
        return _compoundMassCache.Policy == policy ? _compoundMassCache
            : _compoundMassCache.Alternate ??= new CompoundMassCache(policy);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private CompoundMassCache GetRuntimeMassCache(MeshInertiaPolicy policy)
    {
        if (policy != MeshInertiaPolicy.RequireClosedVolume && policy != MeshInertiaPolicy.SurfaceApproximation)
            throw new ArgumentOutOfRangeException(nameof(policy), policy, "Unsupported mesh inertia policy.");
        CompoundMassCache cache = GetMassCache(policy);
        if (cache.Runtime?.Ready != true) BuildRuntimeMassCache(cache);
        return cache;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private void BuildRuntimeMassCache(CompoundMassCache cache)
    {
        Vector3d center;
        if (cache.Policy == MeshInertiaPolicy.RequireClosedVolume)
        {
            if (!TryGetClosedVolumeMassProperties(out MeshMassProperties properties, out MeshVolumeValidationResult result))
                throw new InvalidOperationException($"Mesh inertia requires a validated closed volume. Validation result: {result}.");
            center = properties.CenterOfMass;
        }
        else center = SurfaceMassProperties.CenterOfMass;
        MeshMassIntegrals integrals = GetCompoundMassIntegrals(usePrepared: false, cache.Policy);
        (cache.Runtime ??= new MeshRuntimeInertia()).Build(integrals, center);
    }

    internal MeshMassIntegrals GetCompoundMassIntegrals(bool usePrepared, MeshInertiaPolicy policy)
    {
        CompoundMassCache cache = GetMassCache(policy);
        ref MeshMassIntegrals integrals = ref (usePrepared ? ref cache.Prepared : ref cache.Current);
        ref bool valid = ref (usePrepared ? ref cache.PreparedValid : ref cache.CurrentValid);
        if (!valid)
        {
            Vector3d[] vertices = usePrepared ? _preparedScaledLocalVertices : _scaledLocalVertices;
            // Public unit tensors are projections, not an input to mass-applied
            // moments. Evaluate once per effective geometry, never per mass/pose.
            if (policy == MeshInertiaPolicy.SurfaceApproximation)
            {
                // Surface projection was already validated during preparation;
                // the same immutable geometry supplies its unrounded integrals.
                _ = TriangleShellMassProperties.TryCreateUniformShell(
                    vertices, _triangles, out _, out _, out _, out integrals);
            }
            else
            {
                Vector3d reference = (usePrepared ? _preparedScaledLocalBounds : _scaledLocalBounds).Center;
                IntegrateClosedVolume(vertices, reference,
                    out Signed576 volume,
                    out Signed576 firstX, out Signed576 firstY, out Signed576 firstZ,
                    out Signed576 x2, out Signed576 y2, out Signed576 z2,
                    out Signed576 xy, out Signed576 xz, out Signed576 yz);
                integrals = MeshMassIntegrals.FromSolid(reference, volume,
                    firstX, firstY, firstZ, x2, y2, z2, xy, xz, yz);
            }
            valid = true;
        }
        return integrals;
    }
}
