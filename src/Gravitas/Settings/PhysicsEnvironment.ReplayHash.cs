//=======================================================================
// PhysicsEnvironment.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Chronicler;
using Chronicler;

namespace Gravitas;

public sealed partial class PhysicsEnvironment
{
    internal void ContributeReplayHash(ref ChronicleHashWriter writer)
    {
        writer.WriteSection("environment", 1);
        writer.WriteFixed64(Gravity);
        writer.WriteFixed64(AirDensity);
        writer.WriteFixed64(MinSpeed);
        writer.WriteFixed64(MaxSpeed);
        writer.WriteFixed64(MaxFallSpeed);
        writer.WriteFixed64(FrictionTransitionSpeed);
        writer.WriteFixed64(DecelerationMultiplier);
        writer.WriteFixed64(DampingFactor);
        writer.WriteInt32(CullDistanceMax);
        writer.WriteFixed64(CullFastDistanceMax);
        writer.WriteInt32(CullVelocityStep);
        writer.WriteInt32(CullVelocityMax);
        writer.WriteInt32(CullTimeStep);
        writer.WriteInt32(CullTimeMax);
    }
}
