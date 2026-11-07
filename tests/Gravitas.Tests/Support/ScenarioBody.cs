using Gravitas.Colliders;

namespace Gravitas.Tests.Support;

internal readonly struct ScenarioBody<TCollider>
    where TCollider : LSCollider
{
    public ScenarioBody(SolidBody body, TCollider collider)
    {
        Body = body;
        Collider = collider;
    }

    public SolidBody Body { get; }

    public TCollider Collider { get; }
}
