namespace Gravitas.Benchmarks;

internal static class QualityDuration
{
    internal const int Seconds = 8;
    internal const int PeriodSeconds = 2;
    internal const int Episodes = Seconds / PeriodSeconds;

    internal static bool IsPulse(int frame, int rate) => frame > 1 && (frame - 1) % (PeriodSeconds * rate) == 0;
}
