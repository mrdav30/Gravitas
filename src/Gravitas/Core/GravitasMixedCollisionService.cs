using SwiftCollections;

namespace Gravitas;

/// <summary>
/// Owns mixed 2D/3D collision lifecycle state for one <see cref="GravitasWorldContext"/>.
/// </summary>
internal sealed class GravitasMixedCollisionService
{
    internal GravitasMixedCollisionService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        Context = context;
    }

    internal GravitasWorldContext Context { get; }

    internal int SimulateCount { get; private set; }

    internal int LateSimulateCount { get; private set; }

    internal int VisualizeCount { get; private set; }

    internal int LateVisualizeCount { get; private set; }

    internal void Simulate()
    {
        SimulateCount++;
    }

    internal void LateSimulate()
    {
        LateSimulateCount++;
    }

    internal void Visualize()
    {
        VisualizeCount++;
    }

    internal void LateVisualize()
    {
        LateVisualizeCount++;
    }

    internal void Reset()
    {
        SimulateCount = 0;
        LateSimulateCount = 0;
        VisualizeCount = 0;
        LateVisualizeCount = 0;
    }
}
