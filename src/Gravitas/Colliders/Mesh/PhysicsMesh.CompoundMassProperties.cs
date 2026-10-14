//=======================================================================
// PhysicsMesh.CompoundMassProperties.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Colliders;

public partial class PhysicsMesh
{
    // Allocate the wide retained cache only when this mesh participates in
    // compound inertia. Standalone meshes do not allocate this wide cache.
    private CompoundMassCache? _compoundMassCache;

    private sealed class CompoundMassCache
    {
        internal readonly MeshInertiaPolicy Policy;
        internal MeshMassIntegrals Current, Prepared;
        internal bool CurrentValid, PreparedValid;

        internal CompoundMassCache(MeshInertiaPolicy policy) => Policy = policy;
    }

    internal MeshMassIntegrals GetCompoundMassIntegrals(bool usePrepared, MeshInertiaPolicy policy)
    {
        if (_compoundMassCache == null || _compoundMassCache.Policy != policy)
            _compoundMassCache = new CompoundMassCache(policy);
        CompoundMassCache cache = _compoundMassCache;
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
