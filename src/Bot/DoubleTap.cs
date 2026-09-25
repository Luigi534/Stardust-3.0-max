using System;
using RedUtils;
using RedUtils.Math;

namespace Bot
{
    /// <summary>
    /// Double tap: an aerial that sends the ball into the opponent's backboard above the goal, then a second airborne
    /// strike on the rebound into the net. Both touches are ordinary <see cref="AerialStrike"/> plans from the live
    /// prediction (the rebound comes from the ball prediction, which includes the wall bounce), so the sequence adapts
    /// to where the ball actually goes rather than following a script.
    /// </summary>
    public sealed class DoubleTap : IAction
    {
        public bool Finished { get; private set; }
        public bool Interruptible => first == null || (!first.Launched && second == null);
        public bool SecondTouch => second != null;

        private AerialStrike first, second;
        private readonly float started = Game.Time;
        private float firstDone = float.NaN;

        /// <summary>Backboard aim point: above the crossbar, near the middle of the goal.</summary>
        public static Vec3 BackboardPoint(int team, Vec3 ball)
        {
            float y = -Field.Side(team) * (Field.Length * 0.5f);
            return new Vec3(System.Math.Clamp(ball.x * 0.3f, -600f, 600f), y, Goal.Height + 450f);
        }

        /// <summary>A first touch into the backboard, if one is modelled on target within the next ~2 s.</summary>
        public static DoubleTap TryCreate(RUBot bot)
        {
            Car car = bot.Me;
            if (car == null || car.Boost < 40f)
                return null;
            float attack = -Field.Side(bot.Team);
            if (Ball.Location.y * attack < 1500f)
                return null;
            foreach (BallSlice slice in Ball.Prediction.Slices)
            {
                if (slice == null || slice.Time < Game.Time + 0.2f)
                    continue;
                if (slice.Time > Game.Time + 2.2f)
                    break;
                if (slice.Location.z < 500f || slice.Location.y * attack < 2000f)
                    continue;
                Vec3 target = BackboardPoint(bot.Team, slice.Location);
                AerialStrike strike = AerialStrike.TryCreate(car, slice, target, false, 450f, 0f);
                if (strike != null && strike.AimError < 0.35f && strike.PredictedOutgoing.Length() > 900f)
                    return new DoubleTap { first = strike };
            }
            return null;
        }

        public void Run(RUBot bot)
        {
            Car car = bot.Me;
            if (Game.Time - started > 6f || car.IsDemolished)
            {
                Finished = true;
                return;
            }
            if (second != null)
            {
                second.Run(bot);
                if (second.Finished)
                    Finished = true;
                return;
            }
            if (!first.Finished)
            {
                first.Run(bot);
                return;
            }
            if (float.IsNaN(firstDone))
                firstDone = Game.Time;
            if (car.IsGrounded && Game.Time - firstDone > 0.3f)
            {
                Finished = true;
                return;
            }
            // Rebound: the prediction already includes the backboard bounce; strike it into the goal.
            Vec3 goal = bot.TheirGoal.Location + new Vec3(0, 0, 250);
            foreach (BallSlice slice in Ball.Prediction.Slices)
            {
                if (slice == null || slice.Time < Game.Time + 0.15f)
                    continue;
                if (slice.Time > Game.Time + 2.2f)
                    break;
                foreach (float closing in new[] { AerialStrike.Closing, 350f })
                {
                    AerialStrike strike = AerialStrike.TryCreate(car, slice, goal, true, closing, closing > 500f ? AerialStrike.ThroughDistance : 0f);
                    if (strike != null && strike.AimError < UnderBallReset.GoalCone(slice.Location, bot.TheirGoal.Location))
                    {
                        second = strike;
                        second.Run(bot);
                        return;
                    }
                }
            }
            // Nothing yet: keep flying with the ball toward the goal mouth so a rebound strike becomes reachable.
            // Wait under the predicted rebound: first slice after the ball comes back off the back wall.
            float attack = -Field.Side(bot.Team);
            Vec3 hold = new Vec3(Ball.Location.x * 0.5f, attack * (Field.Length * 0.5f - 1200f), MathF.Max(400f, Ball.Location.z - 200f));
            Vec3 holdVelocity = Vec3.Zero;
            float horizon = 1.0f;
            foreach (BallSlice slice in Ball.Prediction.Slices)
            {
                if (slice == null || slice.Time < Game.Time + 0.3f) continue;
                if (slice.Time > Game.Time + 2.5f) break;
                if (slice.Velocity.y * attack < 0f && slice.Location.z > 300f)
                {
                    hold = slice.Location - new Vec3(0, attack * 350f, 250f);
                    holdVelocity = slice.Velocity;
                    horizon = MathF.Max(0.4f, slice.Time - Game.Time);
                    break;
                }
            }
            Vec3 demand = AerialContact.Guidance(car.Location, car.Velocity, hold, holdVelocity, horizon, Game.Gravity, 0.5f);
            Vec3 nose = ControlMath.Unit(demand, car.Forward);
            ControlMath.Aim(car, bot.Controller, nose, Vec3.Up);
            bot.Controller.Boost = !car.IsGrounded && car.Forward.Dot(nose) > 0.8f && demand.Length() > 300f && car.Boost > 0f;
            bot.Controller.Throttle = 1f;
            if (Game.Time - firstDone > 2.5f)
                Finished = true;
        }
    }
}
