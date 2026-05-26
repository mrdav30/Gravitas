using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

internal struct ColliderQueryState
{
    public uint RaycastVersion { get; set; }

    public uint CircleQueryVersion { get; set; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Reset()
    {
        RaycastVersion = 0;
        CircleQueryVersion = 0;
    }
}
