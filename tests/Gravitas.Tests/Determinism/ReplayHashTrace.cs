using Chronicler.Hashing;

namespace Gravitas.Tests.Determinism;

internal sealed class ReplayHashTrace
{
    public ReplayHashTrace(int frameCount)
    {
        Hashes = new ChronicleHash[frameCount];
    }

    public ChronicleHash[] Hashes { get; }

    public ChronicleHash this[int frame]
    {
        get => Hashes[frame];
        set => Hashes[frame] = value;
    }
}
