using Chronicler.Timing;
using System.Reflection;

namespace Gravitas.Tests.Support;

internal static class TimingTestUtility
{
    internal static void SetClock(GravitasWorldContext context, long frame, ChronicleTimestamp elapsed)
    {
        object owner = typeof(GravitasWorldContext).GetField("_clock", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(context)!;
        var clock = (ChronicleClock)owner.GetType().GetField("_timeline", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(owner)!;
        typeof(ChronicleClock).GetProperty(nameof(ChronicleClock.FrameCount))!.SetValue(clock, frame);
        typeof(ChronicleClock).GetProperty(nameof(ChronicleClock.ElapsedTime))!.SetValue(clock, elapsed);
    }

    internal static void SetLateToken(GravitasWorldContext context, long token) =>
        typeof(GravitasWorldContext).GetField("_lateSimulateToken", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(context, token);
}
