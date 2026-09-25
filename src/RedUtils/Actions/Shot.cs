using RedUtils.Math;
using System;

namespace RedUtils
{
    public abstract class Shot : IAction
    {
        public abstract bool Finished { get; protected internal set; }
        public abstract bool Interruptible { get; protected internal set; }
        public abstract BallSlice Slice { get; protected internal set; }
        public abstract Vec3 ShotTarget { get; protected internal set; }
        public abstract Vec3 TargetLocation { get; protected internal set; }
        public abstract Vec3 ShotDirection { get; protected internal set; }
        public abstract bool IsValid(Car car);

        /// <summary>Reject expired, missing, or diverged predictions before executing a stale shot.</summary>
        public bool IsPredictionValid(float threshold = 60)
        {
            if (Slice == null || !float.IsFinite(Slice.Time) || Slice.Time <= Game.Time ||
                !float.IsFinite(threshold) || threshold <= 0) return false;
            return Ball.Prediction.TrySample(Slice.Time, out Ball predicted) &&
                (Slice.Location - predicted.location).Length() < threshold;
        }
        internal bool ShotValid(float threshold = 60) => IsPredictionValid(threshold);
        public abstract void Run(RUBot bot);
    }
}
