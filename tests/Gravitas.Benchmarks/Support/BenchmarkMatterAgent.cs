using FixedMathSharp;

namespace Gravitas.Benchmarks;

internal sealed class BenchmarkMatterAgent : IMatterAgent
{
    public BenchmarkMatterAgent(
        GravitasWorldContext context,
        Vector3d position,
        bool isParent = true,
        bool isInteracting = false)
    {
        Context = context;
        Transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        IsParent = isParent;
        IsInteracting = isInteracting;
    }

    public GravitasWorldContext Context { get; }

    public FixedTransform Transform { get; }

    public bool IsParent { get; }

    public bool IsInteracting { get; }
}
