using FluentAssertions;
using Gravitas.Support;
using SwiftCollections;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using Xunit;
using static Gravitas.Tests.Support.Coroutines.CoroutineTestRoutines;

namespace Gravitas.Tests.Support.Coroutines;

public sealed class GravitasCoroutineServiceHardeningTests
{
    [Fact]
    public void Simulate_WhenCoroutineReentersService_ShouldNotAdvanceSnapshotTwice()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int steps = 0;
        context.Coroutines.StartCoroutine(Run());

        context.Coroutines.Simulate();

        steps.Should().Be(1);

        IEnumerator<ILockedYieldInstruction> Run()
        {
            steps++;
            context.Coroutines.Simulate();
            yield return context.Coroutines.WaitForNextSimulate();
        }
    }

    [Fact]
    public void Simulate_ShouldDeferChildInsertedIntoFreedUpcomingSlot()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int childSteps = 0;
        LSCoroutine sibling = null!;
        context.Coroutines.StartCoroutine(Parent());
        sibling = context.Coroutines.StartCoroutine(WaitForever(context));

        context.Coroutines.Simulate();

        childSteps.Should().Be(0);
        context.Coroutines.ActiveCoroutineCount.Should().Be(2);

        context.Coroutines.Simulate();

        childSteps.Should().Be(1);

        IEnumerator<ILockedYieldInstruction> Parent()
        {
            context.Coroutines.StopCoroutine(sibling);
            context.Coroutines.StartCoroutine(Child());
            yield return context.Coroutines.WaitForNextSimulate();
        }

        IEnumerator<ILockedYieldInstruction> Child()
        {
            childSteps++;
            yield return context.Coroutines.WaitForNextSimulate();
        }
    }

    [Fact]
    public void Simulate_ShouldReleaseSnapshotReferencesAfterTick()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Coroutines.StartCoroutine(WaitForever(context));

        context.Coroutines.Simulate();

        FieldInfo snapshotField = typeof(GravitasCoroutineService).GetField(
            "_simulationSnapshot",
            BindingFlags.Instance | BindingFlags.NonPublic)!;
        var snapshot = (SwiftList<LSCoroutine>)snapshotField.GetValue(context.Coroutines)!;
        snapshot.Count.Should().Be(0);
        snapshot.InnerArray[0].Should().BeNull();
    }

    [Fact]
    public void StopCoroutine_WhenAllHandlesAreRemoved_ShouldRestartBucketAtZero()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var handles = new List<LSCoroutine>();
        for (int i = 0; i < 16; i++)
            handles.Add(context.Coroutines.StartCoroutine(WaitForever(context)));

        for (int i = 0; i < handles.Count; i++)
            context.Coroutines.StopCoroutine(handles[i]);

        LSCoroutine restarted = context.Coroutines.StartCoroutine(WaitForever(context));
        restarted.Index.Should().Be(0);
    }

    [Fact]
    public void Reset_WhenDisposeReentersSimulate_ShouldNotAdvanceRemainingCoroutines()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int steps = 0;
        context.Coroutines.StartCoroutine(DisposableWait(context, context.Coroutines.Simulate));
        context.Coroutines.StartCoroutine(CountAndDispose(() => steps++, () => { }));
        context.Coroutines.Simulate();
        steps.Should().Be(1);

        context.Coroutines.Reset();

        steps.Should().Be(1);
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);
    }

    [Fact]
    public void Reset_WhenDisposeReentersReset_ShouldCompleteOuterCleanupOnce()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int disposed = 0;
        context.Coroutines.StartCoroutine(new CallbackEnumerator(() =>
        {
            disposed++;
            context.Coroutines.Reset();
        }));

        context.Coroutines.Reset();

        disposed.Should().Be(1);
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);
    }

    [Fact]
    public void Reset_AfterHighWaterMark_ShouldRestartBucketAtZero()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        for (int i = 0; i < 16; i++)
            context.Coroutines.StartCoroutine(WaitForever(context));

        context.Coroutines.Reset();
        LSCoroutine restarted = context.Coroutines.StartCoroutine(WaitForever(context));

        restarted.Index.Should().Be(0);
    }

    [Fact]
    public void ContextDispose_ShouldEndAndDisposeActiveCoroutines()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int disposed = 0;
        LSCoroutine coroutine = context.Coroutines.StartCoroutine(DisposableWait(context, () => disposed++));
        context.Simulate();

        context.Dispose();

        coroutine.Active.Should().BeFalse();
        disposed.Should().Be(1);
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);

        Action restart = () => context.Coroutines.StartCoroutine(WaitForever(context));
        restart.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void Initialize_AfterContextDispose_ShouldNotReactivateService()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Dispose();

        Action initialize = context.Coroutines.Initialize;
        Action start = () => context.Coroutines.StartCoroutine(WaitForever(context));

        initialize.Should().Throw<InvalidOperationException>();
        start.Should().Throw<InvalidOperationException>();
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);
    }

    [Fact]
    public void Deactivate_WhenDisposeReentersInitialize_ShouldRemainDeactivated()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Exception? initializeException = null;
        context.Coroutines.StartCoroutine(new CallbackEnumerator(() =>
        {
            try
            {
                context.Coroutines.Initialize();
            }
            catch (Exception exception)
            {
                initializeException = exception;
            }
        }));

        context.Coroutines.Deactivate();

        initializeException.Should().BeOfType<InvalidOperationException>();
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);
        Action start = () => context.Coroutines.StartCoroutine(WaitForever(context));
        start.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void Simulate_ShouldReadEnumeratorCurrentOnlyAfterSuccessfulMoveNext()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var enumerator = new StrictCurrentEnumerator(context);
        LSCoroutine coroutine = context.Coroutines.StartCoroutine(enumerator);

        context.Coroutines.Simulate();

        enumerator.MoveNextCount.Should().Be(1);
        enumerator.CurrentReadCount.Should().Be(1);
        coroutine.Active.Should().BeTrue();
    }

    [Fact]
    public void Simulate_WhenCoroutineStopsItselfDuringMoveNext_ShouldDisposeYieldedInstruction()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int instructionDisposed = 0;
        var instruction = new TrackingInstruction(context, () => instructionDisposed++);
        LSCoroutine coroutine = null!;
        coroutine = context.Coroutines.StartCoroutine(Run());

        context.Coroutines.Simulate();

        coroutine.Active.Should().BeFalse();
        instructionDisposed.Should().Be(1);
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);

        IEnumerator<ILockedYieldInstruction> Run()
        {
            context.Coroutines.StopCoroutine(coroutine);
            yield return instruction;
        }
    }

    [Fact]
    public void Simulate_WhenCoroutineStopsInsideMoveNext_ShouldDisposeAfterCallbackUnwinds()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var events = new List<string>();
        var instruction = new TrackingInstruction(context, () => events.Add("instruction-dispose"));
        LSCoroutine coroutine = null!;
        coroutine = context.Coroutines.StartCoroutine(Run());

        context.Coroutines.Simulate();

        events.Should().Equal(
            "before-stop",
            "after-stop",
            "instruction-dispose",
            "enumerator-dispose");

        IEnumerator<ILockedYieldInstruction> Run()
        {
            try
            {
                events.Add("before-stop");
                context.Coroutines.StopCoroutine(coroutine);
                events.Add("after-stop");
                yield return instruction;
            }
            finally
            {
                events.Add("enumerator-dispose");
            }
        }
    }

    [Fact]
    public void Simulate_WhenCoroutineStopsItselfThenYieldsNull_ShouldNotRetainInstruction()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCoroutine coroutine = null!;
        coroutine = context.Coroutines.StartCoroutine(Run());

        context.Coroutines.Simulate();

        coroutine.Active.Should().BeFalse();
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);

        IEnumerator<ILockedYieldInstruction> Run()
        {
            context.Coroutines.StopCoroutine(coroutine);
            yield return null!;
        }
    }

    [Fact]
    public void Simulate_WhenMoveNextThrows_ShouldRemoveAndDisposeFaultedCoroutine()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int disposed = 0;
        LSCoroutine coroutine = context.Coroutines.StartCoroutine(
            new ThrowingMoveNextEnumerator(() => disposed++));
        Action simulate = context.Coroutines.Simulate;

        simulate.Should().Throw<InvalidOperationException>().WithMessage("move-next failure");

        coroutine.Active.Should().BeFalse();
        disposed.Should().Be(1);
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);
        simulate.Should().NotThrow();
    }

    [Fact]
    public void Simulate_WhenMoveNextAndDisposeThrow_ShouldPreserveBothFailuresAndRemoveCoroutine()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCoroutine coroutine = context.Coroutines.StartCoroutine(
            new ThrowingMoveNextEnumerator(() => throw new InvalidOperationException("dispose failure")));
        Action simulate = context.Coroutines.Simulate;

        AggregateException exception = simulate.Should().Throw<AggregateException>().Which;

        exception.InnerExceptions.Should().HaveCount(2);
        exception.InnerExceptions[0].Message.Should().Be("move-next failure");
        exception.InnerExceptions[1].Message.Should().Be("dispose failure");
        coroutine.Active.Should().BeFalse();
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenSelfStopThenMoveNextAndDisposeThrow_ShouldPreserveBothFailures()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCoroutine coroutine = null!;
        coroutine = context.Coroutines.StartCoroutine(new ThrowingMoveNextEnumerator(
            () => throw new InvalidOperationException("dispose failure"),
            () => context.Coroutines.StopCoroutine(coroutine)));
        Action simulate = context.Coroutines.Simulate;

        AggregateException exception = simulate.Should().Throw<AggregateException>().Which;

        exception.InnerExceptions.Should().HaveCount(2);
        exception.InnerExceptions[0].Message.Should().Be("move-next failure");
        exception.InnerExceptions[1].Message.Should().Be("dispose failure");
        coroutine.Active.Should().BeFalse();
    }

    [Fact]
    public void Simulate_WhenSelfStopInstructionDisposeThrows_ShouldStillDisposeEnumerator()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int enumeratorDisposed = 0;
        var instruction = new TrackingInstruction(
            context,
            () => throw new InvalidOperationException("instruction dispose failure"));
        LSCoroutine coroutine = null!;
        coroutine = context.Coroutines.StartCoroutine(Run());
        Action simulate = context.Coroutines.Simulate;

        simulate.Should().Throw<InvalidOperationException>().WithMessage("instruction dispose failure");

        enumeratorDisposed.Should().Be(1);
        coroutine.Active.Should().BeFalse();

        IEnumerator<ILockedYieldInstruction> Run()
        {
            try
            {
                context.Coroutines.StopCoroutine(coroutine);
                yield return instruction;
            }
            finally
            {
                enumeratorDisposed++;
            }
        }
    }

    [Fact]
    public void Reset_WhenEnumeratorDisposeThrows_ShouldCleanRemainingCoroutinesAndBucket()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int disposed = 0;
        LSCoroutine first = context.Coroutines.StartCoroutine(new CallbackEnumerator(() =>
        {
            disposed++;
            throw new InvalidOperationException("dispose failure");
        }));
        LSCoroutine second = context.Coroutines.StartCoroutine(new CallbackEnumerator(() => disposed++));
        Action reset = context.Coroutines.Reset;

        reset.Should().Throw<InvalidOperationException>().WithMessage("dispose failure");

        first.Active.Should().BeFalse();
        second.Active.Should().BeFalse();
        disposed.Should().Be(2);
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);
    }

    [Fact]
    public void Reset_WhenInstructionAndEnumeratorDisposeThrow_ShouldPreserveBothFailures()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var instruction = new TrackingInstruction(
            context,
            () => throw new InvalidOperationException("instruction dispose failure"));
        context.Coroutines.StartCoroutine(Run());
        context.Coroutines.Simulate();
        Action reset = context.Coroutines.Reset;

        AggregateException exception = reset.Should().Throw<AggregateException>().Which;

        exception.InnerExceptions.Should().HaveCount(2);
        exception.InnerExceptions[0].Message.Should().Be("instruction dispose failure");
        exception.InnerExceptions[1].Message.Should().Be("enumerator dispose failure");
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);

        IEnumerator<ILockedYieldInstruction> Run()
        {
            try
            {
                yield return instruction;
            }
            finally
            {
                throw new InvalidOperationException("enumerator dispose failure");
            }
        }
    }

    [Fact]
    public void Reset_WhenEnumeratorIsCurrentInstruction_ShouldDisposeSharedObjectOnce()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var shared = new SelfYieldingInstructionEnumerator(context);
        context.Coroutines.StartCoroutine(shared);
        context.Coroutines.Simulate();

        context.Coroutines.Reset();

        shared.DisposeCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WhenSelfStoppingEnumeratorIsCurrentInstruction_ShouldDisposeSharedObjectOnce()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCoroutine coroutine = null!;
        var shared = new SelfYieldingInstructionEnumerator(
            context,
            () => context.Coroutines.StopCoroutine(coroutine));
        coroutine = context.Coroutines.StartCoroutine(shared);

        context.Coroutines.Simulate();

        shared.DisposeCount.Should().Be(1);
        coroutine.Active.Should().BeFalse();
    }

    [Fact]
    public void Simulate_WhenEnumeratorInstructionCompletes_ShouldDisposeSharedObjectOnce()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var shared = new SelfYieldingInstructionEnumerator(context, keepWaiting: false);
        LSCoroutine coroutine = context.Coroutines.StartCoroutine(shared);
        context.Coroutines.Simulate();

        context.Coroutines.Simulate();

        shared.DisposeCount.Should().Be(1);
        coroutine.Active.Should().BeFalse();
    }

    [Fact]
    public void Reset_WhenDisposeStartsCoroutine_ShouldRejectNestedStartWithoutLeakingHandle()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Exception? startException = null;
        LSCoroutine? nested = null;
        context.Coroutines.StartCoroutine(new CallbackEnumerator(() =>
        {
            try
            {
                nested = context.Coroutines.StartCoroutine(WaitForever(context));
            }
            catch (Exception exception)
            {
                startException = exception;
            }
        }));

        context.Coroutines.Reset();

        startException.Should().BeOfType<InvalidOperationException>();
        nested.Should().BeNull();
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenInstructionCompletes_ShouldDisposeInstructionExactlyOnce()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int disposed = 0;
        var instruction = new TrackingInstruction(context, () => disposed++);
        LSCoroutine coroutine = context.Coroutines.StartCoroutine(Run());

        context.Coroutines.Simulate();
        context.Coroutines.Simulate();

        instruction.KeepWaitingReadCount.Should().Be(1);
        disposed.Should().Be(1);
        coroutine.Active.Should().BeFalse();

        IEnumerator<ILockedYieldInstruction> Run()
        {
            yield return instruction;
        }
    }

    [Fact]
    public void Simulate_WhenKeepWaitingStopsCoroutine_ShouldNotAdvanceDisposedEnumerator()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int resumed = 0;
        int disposed = 0;
        LSCoroutine coroutine = null!;
        var instruction = new TrackingInstruction(
            context,
            () => disposed++,
            onRead: () => context.Coroutines.StopCoroutine(coroutine));
        coroutine = context.Coroutines.StartCoroutine(Run());
        context.Coroutines.Simulate();

        context.Coroutines.Simulate();

        resumed.Should().Be(0);
        disposed.Should().Be(1);
        coroutine.Active.Should().BeFalse();

        IEnumerator<ILockedYieldInstruction> Run()
        {
            yield return instruction;
            resumed++;
        }
    }

    [Fact]
    public void Simulate_WhenCompletedInstructionDisposeStopsCoroutine_ShouldNotAdvanceEnumerator()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int resumed = 0;
        int disposed = 0;
        LSCoroutine coroutine = null!;
        var instruction = new TrackingInstruction(context, () =>
        {
            disposed++;
            context.Coroutines.StopCoroutine(coroutine);
        });
        coroutine = context.Coroutines.StartCoroutine(Run());
        context.Coroutines.Simulate();

        context.Coroutines.Simulate();

        resumed.Should().Be(0);
        disposed.Should().Be(1);
        coroutine.Active.Should().BeFalse();

        IEnumerator<ILockedYieldInstruction> Run()
        {
            yield return instruction;
            resumed++;
        }
    }

    [Fact]
    public void Simulate_WhenInstructionThrows_ShouldDisposeAndRemoveCoroutine()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int disposed = 0;
        var instruction = new TrackingInstruction(context, () => disposed++, throwWhenRead: true);
        LSCoroutine coroutine = context.Coroutines.StartCoroutine(Run());
        context.Coroutines.Simulate();
        Action simulate = context.Coroutines.Simulate;

        simulate.Should().Throw<InvalidOperationException>().WithMessage("keep-waiting failure");

        instruction.KeepWaitingReadCount.Should().Be(1);
        disposed.Should().Be(1);
        coroutine.Active.Should().BeFalse();
        context.Coroutines.ActiveCoroutineCount.Should().Be(0);

        IEnumerator<ILockedYieldInstruction> Run()
        {
            yield return instruction;
        }
    }

    [Fact]
    public void Simulate_WithForeignContextInstruction_ShouldRejectAndDisposeCoroutine()
    {
        using GravitasWorldContext contextA = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext contextB = GravitasWorldContext.CreateOwned();
        LSCoroutine coroutine = contextA.Coroutines.StartCoroutine(Run());
        Action simulate = contextA.Coroutines.Simulate;

        simulate.Should().Throw<InvalidOperationException>();

        coroutine.Active.Should().BeFalse();
        contextA.Coroutines.ActiveCoroutineCount.Should().Be(0);

        IEnumerator<ILockedYieldInstruction> Run()
        {
            yield return contextB.Coroutines.WaitForNextSimulate();
        }
    }

    private sealed class StrictCurrentEnumerator : IEnumerator<ILockedYieldInstruction>
    {
        private readonly GravitasWorldContext _context;
        private bool _hasCurrent;

        public StrictCurrentEnumerator(GravitasWorldContext context)
        {
            _context = context;
        }

        public int MoveNextCount { get; private set; }

        public int CurrentReadCount { get; private set; }

        public ILockedYieldInstruction Current
        {
            get
            {
                CurrentReadCount++;
                if (!_hasCurrent)
                    throw new InvalidOperationException("Current was read before MoveNext succeeded.");

                return _context.Coroutines.WaitForNextSimulate();
            }
        }

        object IEnumerator.Current => Current;

        public bool MoveNext()
        {
            MoveNextCount++;
            if (MoveNextCount != 1)
                return false;

            _hasCurrent = true;
            return true;
        }

        public void Reset() => throw new NotSupportedException();

        public void Dispose()
        {
        }
    }

    private sealed class ThrowingMoveNextEnumerator : IEnumerator<ILockedYieldInstruction>
    {
        private readonly Action _onDispose;
        private readonly Action? _onMoveNext;

        public ThrowingMoveNextEnumerator(Action onDispose, Action? onMoveNext = null)
        {
            _onDispose = onDispose;
            _onMoveNext = onMoveNext;
        }

        public ILockedYieldInstruction Current => null!;

        object IEnumerator.Current => Current;

        public bool MoveNext()
        {
            _onMoveNext?.Invoke();
            throw new InvalidOperationException("move-next failure");
        }

        public void Reset() => throw new NotSupportedException();

        public void Dispose() => _onDispose();
    }

    private sealed class CallbackEnumerator : IEnumerator<ILockedYieldInstruction>
    {
        private readonly Action _onDispose;

        public CallbackEnumerator(Action onDispose)
        {
            _onDispose = onDispose;
        }

        public ILockedYieldInstruction Current => null!;

        object IEnumerator.Current => Current;

        public bool MoveNext() => true;

        public void Reset() => throw new NotSupportedException();

        public void Dispose() => _onDispose();
    }

    private sealed class TrackingInstruction : ILockedYieldInstruction
    {
        private readonly Action _onDispose;
        private readonly bool _throwWhenRead;
        private readonly Action? _onRead;

        public TrackingInstruction(
            GravitasWorldContext context,
            Action onDispose,
            bool throwWhenRead = false,
            Action? onRead = null)
        {
            Context = context;
            _onDispose = onDispose;
            _throwWhenRead = throwWhenRead;
            _onRead = onRead;
        }

        public GravitasWorldContext Context { get; }

        public int KeepWaitingReadCount { get; private set; }

        public bool KeepWaiting
        {
            get
            {
                KeepWaitingReadCount++;
                _onRead?.Invoke();
                if (_throwWhenRead)
                    throw new InvalidOperationException("keep-waiting failure");

                return false;
            }
        }

        public object Current => null!;

        public bool MoveNext() => KeepWaiting;

        public void Reset()
        {
        }

        public void Dispose() => _onDispose();
    }

    private sealed class SelfYieldingInstructionEnumerator : IEnumerator<ILockedYieldInstruction>, ILockedYieldInstruction
    {
        private readonly Action? _onMoveNext;
        private readonly bool _keepWaiting;
        private bool _moved;

        public SelfYieldingInstructionEnumerator(
            GravitasWorldContext context,
            Action? onMoveNext = null,
            bool keepWaiting = true)
        {
            Context = context;
            _onMoveNext = onMoveNext;
            _keepWaiting = keepWaiting;
        }

        public GravitasWorldContext Context { get; }

        public int DisposeCount { get; private set; }

        public bool KeepWaiting => _keepWaiting;

        public ILockedYieldInstruction Current => this;

        object IEnumerator.Current => Current;

        public bool MoveNext()
        {
            if (_moved)
                return false;

            _moved = true;
            _onMoveNext?.Invoke();
            return true;
        }

        public void Reset() => _moved = false;

        public void Dispose() => DisposeCount++;
    }
}
