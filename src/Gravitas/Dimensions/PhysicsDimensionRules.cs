using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas;

internal static class PhysicsDimensionRules
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsSupported(PhysicsDimension dimension) =>
        dimension == PhysicsDimension.TwoD || dimension == PhysicsDimension.ThreeD;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ThrowIfUnsupported(PhysicsDimension dimension, string paramName)
    {
        SwiftThrowHelper.ThrowIfArgument(
            !IsSupported(dimension),
            paramName,
            "Unsupported physics dimension.");
    }
}
