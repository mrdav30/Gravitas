using FixedMathSharp;
using Gravitas.Support;

namespace Gravitas.Tests.Support;

internal sealed class TestMatterAgent : IMatterAgent
{
    public TestMatterAgent(
        GravitasWorldContext context,
        FixedTransform? transform = null,
        bool isParent = true,
        bool isInteracting = false)
    {
        Context = context;
        Transform = transform ?? new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        IsParent = isParent;
        IsInteracting = isInteracting;
    }

    public GravitasWorldContext Context { get; }

    public FixedTransform Transform { get; }

    public bool IsParent { get; }

    public bool IsInteracting { get; }
}
