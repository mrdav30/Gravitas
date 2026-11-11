//=======================================================================
// GravitasWorldContext.Sleep.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas;

/// <content>Coordinates automatic sleep after all participating response phases.</content>
public sealed partial class GravitasWorldContext
{
    private readonly SleepIslands _sleepIslands = new();

    internal void ReserveSleepIslands() =>
        _sleepIslands.Reserve(checked(Physics.BodySlotCount + Physics2D.BodySlotCount));

    internal void UpdateAutomaticSleep(bool ran3D, bool ran2D, bool mixed = false)
    {
        bool ready3D = ran3D && Physics.AdvanceSleepCounters();
        bool ready2D = ran2D && Physics2D.AdvanceSleepCounters();
        if (!ready3D && !ready2D)
            return;

        // Only rebuild connectivity on a candidate frame. Current retained
        // contacts include sleeping edges and reflect response-time removals.
        int planarOffset = ran3D || mixed ? Physics.BodySlotCount : 0;
        int planarSlots = ran2D || mixed ? Physics2D.BodySlotCount : 0;
        _sleepIslands.Begin(checked(planarOffset + planarSlots));
        if (ran3D) Physics.JoinSleepIslands(_sleepIslands);
        if (ran2D) Physics2D.JoinSleepIslands(_sleepIslands, planarOffset);
        if (mixed) MixedCollisions.JoinSleepIslands(_sleepIslands, planarOffset);

        if (ran3D || mixed) Physics.BlockUnreadySleepIslands(_sleepIslands, ran3D);
        if (ran2D || mixed) Physics2D.BlockUnreadySleepIslands(_sleepIslands, planarOffset, ran2D);
        if (ready3D) Physics.SleepReadyIslands(_sleepIslands);
        if (ready2D) Physics2D.SleepReadyIslands(_sleepIslands, planarOffset);
    }
}
