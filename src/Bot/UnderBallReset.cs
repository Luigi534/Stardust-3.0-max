using System;
using RedUtils;
using RedUtils.Math;

namespace Bot
{
    /// <summary>
    /// Flip reset from under the ball. Principles (verified in RocketSim): the reset is granted when the wheels
    /// meet the ball with at least ~250 uu/s closing speed, up to ~30-45 deg of wheel tilt and little slip; and once
    /// airborne, car and ball fall together, so their relative motion is fixed by the jump impulses (about
    /// 876 uu/s upward for jump + full hold + double jump) plus boost. The play therefore:
    /// 1. shadows under the ball on the ground, matching its horizontal velocity;
    /// 2. launches when the ball's vertical speed makes the closing speed right, the contact will be high enough,
    ///    the flight is long enough for the half roll, and the predicted horizontal miss is small;
    /// 3. rolls wheels-up (the fastest rotation axis), then holds under the ball with boost tilted at most ~30 deg
    ///    from the wheel plane, and settles square onto the ball for the last moments;
    /// 4. after a confirmed reset, shoots with the restored flip (a goal-directed aerial strike) or chains another.
    /// Nothing here is time-scripted: every decision is recomputed from the live car/ball state.
    /// </summary>
    public sealed class UnderBallReset : IPossessionAction
    {
        public const float JumpImpulse = 292f * 3f;
        public const float WheelRadius = 93f + 18f;
        public const float TargetClosing = 700f;
        public const float MinClosing = 150f, MaxClosing = 1400f;
        public const float MinContactHeight = 450f;
        public const float MinFlight = 0.9f;
        public const float LaunchMiss = 30f, LaunchSlip = 200f;
        public const float MaxTilt = 0.52f; // ~30 deg
        public const float HoldGain = 4f, HoldDamping = 4.5f;
        public const float Settle = 0.3f;
        public const int MaxResets = 2;
        public const float BehindOffset = 40f;
        /// <summary>Contact firmness options for the post-reset shot, firmest first.</summary>
        private static readonly float[] ShotClosings = { AerialStrike.Closing, 350f, 0f };

        private enum Phase { Shadow, Jump, DoubleJump, Under, Follow, Shoot }

        public bool Finished { get; private set; }
        public bool Interruptible => phase == Phase.Shadow;
        public float ClaimTime => Game.Time + 0.3f;
        public int Resets { get; private set; }
        public bool Confirmed => Resets > 0;
        public bool Committed => phase != Phase.Shadow;
        public string Stage => phase.ToString();

        private Phase phase = Phase.Shadow;
        private float phaseStart = Game.Time;
        private readonly float started = Game.Time;
        private readonly ResetEvidence evidence = new();
        private AerialStrike strike;

        public static UnderBallReset TryCreate(RUBot bot) =>
            bot.Me != null && bot.Me.IsGrounded && Worthwhile(bot.Me) ? new UnderBallReset() : null;

        /// <summary>A ball high and slow enough in its arc that a launch window will open soon.</summary>
        public static bool Worthwhile(Car car)
        {
            if (car.Boost < 30f || Ball.Location.z < 250f)
                return false;
            float apexTime = MathF.Max(0f, Ball.Velocity.z / -Game.Gravity.z);
            float apex = Ball.Location.z + Ball.Velocity.z * apexTime + 0.5f * Game.Gravity.z * apexTime * apexTime;
            return apex > MinContactHeight + 350f && car.Location.FlatDist(Ball.Location) < 1600f;
        }

        /// <summary>Contact time and ball height at contact for a launch now with the given upward impulse.</summary>
        public static (float time, float height, float closing) LaunchPrediction(Vec3 car, Vec3 ball, Vec3 ballVelocity, float impulse)
        {
            float closing = impulse - ballVelocity.z;
            float gap = ball.z - car.z - WheelRadius - 60f;
            float time = 0.25f + MathF.Max(gap - 110f, 0f) / MathF.Max(closing, 1f);
            float g = Game.Gravity.z;
            float height = ball.z + ballVelocity.z * time + 0.5f * g * time * time;
            return (time, height, closing);
        }

        public void Run(RUBot bot)
        {
            Car car = bot.Me;
            if (car.IsDemolished || Game.Time - started > 7f)
            {
                Finished = true;
                return;
            }
            if (phase != Phase.Shadow && phase != Phase.Jump && car.IsGrounded && car.Location.z < 60f &&
                Game.Time - phaseStart > 0.3f)
            {
                Finished = true;
                return;
            }

            bool wheelsOnBall = (-car.Up).Dot(ControlMath.Unit(Ball.Location - car.Location, Vec3.Up)) > 0.6f;
            if (evidence.Observe(bot.Jump, bot.OwnTouchThisTick, wheelsOnBall, car.Location.z, Game.Time) &&
                phase == Phase.Under)
            {
                Resets++;
                evidence.Reset();
                Enter(Phase.Follow);
            }

            switch (phase)
            {
                case Phase.Shadow: Shadow(bot, car); return;
                case Phase.Jump: Jump(bot, car); return;
                case Phase.DoubleJump: DoubleJump(bot, car); return;
                case Phase.Under: Under(bot, car); return;
                case Phase.Follow: Follow(bot, car); return;
                case Phase.Shoot:
                    strike.Run(bot);
                    if (strike.Finished)
                        Finished = true;
                    return;
            }
        }

        private void Enter(Phase next)
        {
            phase = next;
            phaseStart = Game.Time;
        }

        private void Shadow(RUBot bot, Car car)
        {
            Vec3 ball = Ball.Location, ballVelocity = Ball.Velocity;
            Vec3 relative = (ball - car.Location).Flatten();
            Vec3 relativeVelocity = (ballVelocity - car.Velocity).Flatten();
            var (time, height, closing) = LaunchPrediction(car.Location, ball, ballVelocity, JumpImpulse);
            bool window = closing >= MinClosing && closing <= MaxClosing && height >= MinContactHeight && time >= MinFlight;
            if (window && car.IsGrounded && (relative + relativeVelocity * time).Length() < LaunchMiss &&
                relativeVelocity.Length() < LaunchSlip)
            {
                Enter(Phase.Jump);
                Jump(bot, car);
                return;
            }
            // The window has passed for good: the ball is falling below any useful contact height.
            if (ballVelocity.z < 0f && height < MinContactHeight && ball.z < MinContactHeight + 300f)
            {
                Finished = true;
                return;
            }

            // Drive to the ball's ground projection while matching its horizontal velocity.
            float distance = relative.Length();
            Vec3 lead = (ball + ballVelocity * MathF.Min(0.35f, distance / 1500f)).Flatten();
            Vec3 toLead = lead - car.Location.Flatten();
            Vec3 want = ballVelocity.Flatten() + toLead * 2.5f;
            float speed = want.Length();
            Vec3 heading = speed > 50f ? want / speed : car.Forward.Flatten();
            Vec3 local = car.Local(heading);
            float yawError = MathF.Atan2(local.y, local.x);
            float current = car.Velocity.Dot(car.Forward);
            var c = bot.Controller;
            c.Jump = false;
            c.Handbrake = false;
            if (MathF.Abs(yawError) > 2.2f)
            {
                c.Throttle = -1f;
                c.Steer = ControlRuntime.Axis(-3f * yawError);
                c.Boost = false;
            }
            else
            {
                float target = MathF.Min(speed, Car.MaxSpeed);
                c.Steer = ControlRuntime.Axis(3f * yawError);
                c.Throttle = ControlRuntime.Axis((target - current) / 250f);
                c.Boost = target - current > 350f && MathF.Abs(yawError) < 0.3f;
            }
        }

        private void Jump(RUBot bot, Car car)
        {
            var c = bot.Controller;
            float elapsed = Game.Time - phaseStart;
            c.Jump = elapsed < Car.JumpMaxDuration;
            c.Boost = false;
            c.Throttle = 0f;
            c.Pitch = c.Yaw = c.Roll = 0f;
            if (elapsed >= Car.JumpMaxDuration)
                Enter(Phase.DoubleJump);
        }

        /// <summary>Second jump with a neutral stick: any stick input would turn it into a dodge.</summary>
        private void DoubleJump(RUBot bot, Car car)
        {
            var c = bot.Controller;
            float elapsed = Game.Time - phaseStart;
            c.Pitch = c.Yaw = c.Roll = 0f;
            c.Boost = false;
            c.Jump = elapsed < 0.04f;
            if (elapsed > 0.05f)
                Enter(Phase.Under);
        }

        /// <summary>Wheels to the ball; tilted boost holds the car under it at the planned closing speed.</summary>
        private void Under(RUBot bot, Car car)
        {
            var c = bot.Controller;
            Vec3 r = car.Location - Ball.Location, w = car.Velocity - Ball.Velocity;
            Vec3 n = ControlMath.Unit(r, -Vec3.Up); // ball -> car
            // Hold slightly behind the ball (relative to the attacked goal): the reset touch then lifts the ball
            // up and goalward, which sets up the follow-up shot.
            Vec3 toGoal = ControlMath.FlatUnit(bot.TheirGoal.Location - Ball.Location, Vec3.Y);
            Vec3 rh = r.Flatten() + toGoal * BehindOffset, wh = w.Flatten();
            float closing = -w.Dot(n);
            float timeToGo = MathF.Max(0.08f, (r.Length() - WheelRadius) / MathF.Max(closing, 100f));
            Vec3 lateral = -(rh * HoldGain + wh * HoldDamping);
            Vec3 demand = lateral + n * ((closing - TargetClosing) * 3f);
            Vec3 plane = demand - n * demand.Dot(n);
            float planeLength = plane.Length();
            Vec3 nose;
            bool boost;
            if (planeLength < 1f)
            {
                nose = ControlMath.Unit(car.Forward - n * car.Forward.Dot(n), car.Forward);
                boost = false;
            }
            else
            {
                float angle = System.Math.Clamp(MathF.Atan2(demand.Dot(n), planeLength), -MaxTilt, MaxTilt);
                nose = plane / planeLength * MathF.Cos(angle) + n * MathF.Sin(angle);
                boost = demand.Length() > 250f;
            }
            Vec3 roof = ControlMath.Unit(n - nose * n.Dot(nose), n);
            if (timeToGo < Settle)
            {
                // Last moments: no thrust, wheels square onto the ball.
                nose = ControlMath.Unit(car.Forward - n * car.Forward.Dot(n), car.Forward);
                roof = n;
                boost = false;
            }
            else if (car.Up.Dot(n) < 0.3f && Game.Time - phaseStart < 0.5f)
            {
                // Half roll to wheels-up with the nose held: the rotation stays on the fast roll axis.
                nose = car.Forward;
                roof = ControlMath.Unit(n - car.Forward * n.Dot(car.Forward), n);
                boost = false;
            }
            ControlMath.AimStiff(car, c, nose, roof);
            c.Boost = boost && car.Forward.Dot(nose) > 0.85f && car.Boost > 0f;
            c.Throttle = car.Forward.Dot(demand) > 0f ? 1f : 0f;
            c.Jump = false;
            if (Game.Time - phaseStart > 3.5f)
                Finished = true;
        }

        /// <summary>Aim tolerance that still puts the ball inside the goal mouth from this distance.</summary>
        public static float GoalCone(Vec3 ball, Vec3 goal)
        {
            float distance = MathF.Max(300f, ball.FlatDist(goal));
            return MathF.Max(0.25f, MathF.Atan2(Goal.Width * 0.5f - 150f, distance)) + 0.1f;
        }

        /// <summary>
        /// After a reset: shoot with the restored flip once a goal-directed strike is modelled on target; while
        /// waiting, keep holding under the ball. A second reset is chained once the flip has been spent.
        /// </summary>
        private void Follow(RUBot bot, Car car)
        {
            Vec3 goal = bot.TheirGoal.Location + new Vec3(0, 0, 250);
            if (bot.Jump.CanDodge && Game.Time - phaseStart > 0.12f)
            {
                foreach (BallSlice slice in Ball.Prediction.Slices)
                {
                    if (slice == null || slice.Time < Game.Time + 0.15f)
                        continue;
                    if (slice.Time > Game.Time + 1.6f)
                        break;
                    foreach (float closing in ShotClosings)
                    {
                        AerialStrike candidate = AerialStrike.TryCreate(car, slice, goal, true, closing, closing > 500f ? AerialStrike.ThroughDistance : 0f);
                        if (candidate == null || candidate.AimError >= GoalCone(slice.Location, bot.TheirGoal.Location))
                            continue;
                        strike = candidate;
                        Enter(Phase.Shoot);
                        strike.Run(bot);
                        return;
                    }
                }
            }
            if (Game.Time - phaseStart > 2f || Ball.Location.z < 250f)
            {
                Finished = true;
                return;
            }
            Under(bot, car);
        }
    }
}
