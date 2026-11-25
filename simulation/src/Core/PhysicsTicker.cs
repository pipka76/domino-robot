using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace DominoSim;

/// <summary>
/// Minimal awaitable that resumes continuations once per physics tick (when Pump is called).
/// Lets mission logic be written as sequential async code that runs in lock-step with physics.
/// </summary>
public sealed class PhysicsTicker
{
    List<Action> _pending = new();
    List<Action> _running = new();

    public void Pump()
    {
        (_pending, _running) = (_running, _pending);
        foreach (var a in _running) a();
        _running.Clear();
    }

    public Awaitable Next() => new(this);

    public readonly struct Awaitable : INotifyCompletion
    {
        readonly PhysicsTicker _t;
        public Awaitable(PhysicsTicker t) { _t = t; }
        public Awaitable GetAwaiter() => this;
        public bool IsCompleted => false;
        public void OnCompleted(Action continuation) => _t._pending.Add(continuation);
        public void GetResult() { }
    }
}
