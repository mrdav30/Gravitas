//=======================================================================
// GravitasReplayHashService.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp.Chronicler;

namespace Gravitas;

internal static class GravitasReplayHashService
{
    public static ChronicleHash Compute(
        GravitasWorldContext context,
        GravitasReplayHashMode mode)
    {
        var writer = new ChronicleHashWriter();
        writer.WriteSection("gravitas.replay", 1);
        writer.WriteEnum(mode);
        writer.WriteEnum(context.Settings.RuntimeMode);
        writer.WriteInt32(context.FrameRate);
        writer.WriteFixed64(context.DeltaTime);
        writer.WriteFixed64(context.InvDeltaTime);
        writer.WriteInt32(context.FrameCount);
        writer.WriteFixed64(context.TotalTime);
        writer.WriteInt32(context.LateSimulateToken);

        context.Settings.ContributeReplayHash(ref writer);
        context.Environment.ContributeReplayHash(ref writer);
        context.Constraints3D.ContributeReplayHash(ref writer, mode);
        context.Constraints2D.ContributeReplayHash(ref writer, mode);

        PhysicsRuntimeMode runtimeMode = context.Settings.RuntimeMode;
        if (runtimeMode.Runs3D())
            context.Physics.ContributeReplayHash(ref writer, mode);
        if (runtimeMode.Runs2D())
            context.Physics2D.ContributeReplayHash(ref writer, mode);
        if (runtimeMode.RunsMixedContacts())
            context.MixedCollisions.ContributeReplayHash(ref writer, mode);

        return writer.ToHash();
    }
}
