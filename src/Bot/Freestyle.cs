using System;
using RedUtils;
using RedUtils.Math;

namespace Bot
{
    /// <summary>
    /// Flip reset play, modelled on recorded human resets rather than on any one recording:
    /// take off early and fly up with the ball, shadow a point below/behind it at matched velocity, then
    /// commit — a short boost pulse along the push builds a few hundred uu/s of closing speed, thrust is cut
    /// and the car rotates its wheel normal onto the ball while both fall together. The restored flip is
    /// then spent on a goal-directed dodge strike, or another reset is chained once it has been used.
    /// </summary>
    public sealed class FlipResetPlay : IPossessionAction
    {
        public const float Closing = 260f;
        public const float HoldDistance = 250f;
        public const int MaxResets = 3;

        private enum Phase { Launch, Shadow, Commit, Follow, Shoot }

        public bool Finished { get; private set; }
        public bool Interruptible => (phase == Phase.Launch && !launched) || (phase == Phase.Shadow && !Confirmed);
        /// <summary>Still only approaching the first reset: a direct scoring contact should take over.</summary>
        public bool Yieldable => phase == Phase.Shadow && !Confirmed;
        public float ClaimTime => Game.Time + 0.3f;
        public int Resets { get; private set; }
        public bool Confirmed => Resets > 0;
        public string Stage => phase.ToString();

        private Phase phase;
        private AerialStrike strike;
        private readonly BoostGate boost = new();
        private readonly ResetEvidence evidence = new();
        private readonly JumpSequence launchJump = new(Car.JumpMaxDuration);
        private readonly float started = Game.Time;
        private float phaseStart = Game.Time, convergedSince = float.NaN;
        private bool launched;

        private FlipResetPlay(Car car, JumpState jump)
        {
            phase = car.IsGrounded ? Phase.Launch : Phase.Shadow;
            launched = !car.IsGrounded;
            evidence.Observe(jump, false, false, car.Location.z, Game.Time);
        }

        /// <summary>Push that keeps the ball playable: mostly up, partly toward the attacking goal.</summary>
        public static Vec3 ResetPush(Vec3 ball, Vec3 attackGoal)
        {
            Vec3 toGoal = ControlMath.FlatUnit(attackGoal - ball, Vec3.X);
            return ControlMath.Unit(Vec3.Up * 0.8f + toGoal * 0.45f, Vec3.Up);
        }

        /// <summary>
        /// Whether a reset attempt is a good use of this possession: free ball, time to act, and fuel for
        /// the acquisition plus the follow-up shot. Never over our own half's final third.
        /// </summary>
        public static bool Worthwhile(TacticalFrame frame, Car car, Ball ball, Vec3 ownGoal, float threatTime)
        {
            if (frame == null || car == null || ball == null || frame.TeamRank != 0)
                return false;
            if (float.IsFinite(threatTime) || car.Boost < 50f)
                return false;
            if (frame.OpponentEta < 1.6f || frame.FreeTime < 0.8f)
                return false;
            float side = ownGoal.y < 0 ? -1f : 1f;
            return ball.location.y * side < 1500f && ball.location.z > 350f;
        }

        /// <summary>The ball must stay high enough, long enough, for a climb and a controlled contact.</summary>
        public static bool Reachable(Car car)
        {
            if (car == null || car.Boost < 45f || !Ball.Prediction.TrySample(Game.Time + 1.2f, out Ball later))
                return false;
            if (later.location.z < 800f || later.velocity.z < -900f)
                return false;
            float flat = car.Location.FlatDist(later.location);
            if (car.IsGrounded)
            {
                if (car.Up.z < 0.8f || flat > 1300f)
                    return false;
                return (later.location - car.Location).Flatten().Dot(car.Forward.Flatten()) > -200f;
            }
            return flat < 1100f && later.location.z - car.Location.z > -100f;
        }

        public static FlipResetPlay TryCreate(RUBot bot) =>
            Reachable(bot.Me) ? new FlipResetPlay(bot.Me, bot.Jump) : null;

        public void Run(RUBot bot)
        {
            Car car = bot.Me;
            if (car.IsDemolished || Game.Time - started > 6f ||
                (car.IsGrounded && launched && Game.Time - phaseStart > 0.3f))
            {
                Finished = true;
                return;
            }

            bool wheelsOnBall = (-car.Up).Dot(ControlMath.Unit(Ball.Location - car.Location, Vec3.Up)) > 0.6f;
            if (evidence.Observe(bot.Jump, bot.OwnTouchThisTick, wheelsOnBall, car.Location.z, Game.Time) &&
                (phase == Phase.Commit || phase == Phase.Shadow))
            {
                Resets++;
                evidence.Reset();
                Enter(Phase.Follow);
            }

            switch (phase)
            {
                case Phase.Launch: Launch(bot, car); return;
                case Phase.Shadow: ShadowPhase(bot, car); return;
                case Phase.Commit: Commit(bot, car); return;
                case Phase.Follow: Follow(bot, car); return;
                case Phase.Shoot:
                    strike.Run(bot);
                    if (strike.Finished)
                        Finished = true;
                    return;
            }
        }

        private static Vec3 Push(RUBot bot) => ResetPush(Ball.Location, bot.TheirGoal.Location);

        /// <summary>Jump, release, neutral double jump; nose pitched up toward the ball, boosting once aligned.</summary>
        private void Launch(RUBot bot, Car car)
        {
            launched = true;
            float elapsed = Game.Time - phaseStart;
            JumpCommand command = launchJump.Step(Game.Time, true);
            Vec3 toward = ControlMath.Unit(Ball.Location - Vec3.Up * HoldDistance - car.Location, Vec3.Up);
            ControlMath.Aim(car, bot.Controller, ControlMath.Unit(toward + Vec3.Up, Vec3.Up), -car.Forward);
            bot.Controller.Jump = command.Jump;
            if (command.Dodge)
                bot.Controller.Pitch = bot.Controller.Yaw = bot.Controller.Roll = 0f;
            bot.Controller.Boost = elapsed > 0.08f && car.Forward.z > 0.3f;
            bot.Controller.Throttle = 1f;
            if (elapsed > 0.45f)
                Enter(Phase.Shadow);
        }

        /// <summary>
        /// Earliest time in [<paramref name="minTime"/>, <paramref name="maxTime"/>] at which unpowered relative
        /// motion (gravity cancels when both objects coast) brings the ball to wheel-contact distance on the
        /// push side with a moderate closing speed. Returns the contact normal (car → ball) or null.
        /// </summary>
        public static Vec3? CoastContact(Vec3 relativePosition, Vec3 relativeVelocity, Vec3 push,
            float minTime = 0.25f, float maxTime = 0.9f, float radius = 112f)
        {
            float a = relativeVelocity.Dot(relativeVelocity);
            if (a < 1e-3f)
                return null;
            float b = 2f * relativePosition.Dot(relativeVelocity);
            float c = relativePosition.Dot(relativePosition) - radius * radius;
            float discriminant = b * b - 4f * a * c;
            if (discriminant < 0f)
                return null;
            float t = (-b - MathF.Sqrt(discriminant)) / (2f * a);
            if (t < minTime || t > maxTime)
                return null;
            Vec3 normal = (relativePosition + relativeVelocity * t) / radius;
            float closing = -relativeVelocity.Dot(normal);
            if (normal.Dot(push) < 0.6f || closing < 100f || closing > 550f)
                return null;
            return normal;
        }

        /// <summary>
        /// Steer toward a state that coasts into a wheel contact at <see cref="Closing"/> uu/s; commit (cut
        /// thrust, rotate) as soon as the coast prediction says the contact will happen.
        /// </summary>
        private void ShadowPhase(RUBot bot, Car car)
        {
            if (Ball.Location.z < 450f || car.Boost <= 0f || Game.Time - phaseStart > 2.4f)
            {
                Finished = true;
                return;
            }
            Vec3 push = Push(bot);
            if (CoastContact(Ball.Location - car.Location, Ball.Velocity - car.Velocity, push) != null)
            {
                Enter(Phase.Commit);
                Commit(bot, car);
                return;
            }
            Shadow(bot, car, push, 110f + Closing * 0.5f, Closing);
        }

        /// <summary>
        /// Coast: no thrust; rotate the wheel normal onto the ball with the nose toward the attack direction.
        /// Both objects fall together, so the planned closing speed holds. If the coast stops predicting a
        /// contact, resume steering.
        /// </summary>
        private void Commit(RUBot bot, Car car)
        {
            float elapsed = Game.Time - phaseStart;
            Vec3 relative = Ball.Location - car.Location;
            Vec3 toBall = ControlMath.Unit(relative, Vec3.Up);
            if (elapsed > 1.2f)
            {
                Finished = true;
                return;
            }
            if (elapsed > 0.15f && relative.Length() > 200f &&
                CoastContact(relative, Ball.Velocity - car.Velocity, Push(bot), 0f, 1f) == null)
            {
                Enter(Phase.Shadow);
                Shadow(bot, car, Push(bot), 110f + Closing * 0.5f, Closing);
                return;
            }
            Vec3 heading = ControlMath.FlatUnit(bot.TheirGoal.Location - Ball.Location, car.Forward);
            Vec3 nose = heading - toBall * heading.Dot(toBall);
            ControlMath.Aim(car, bot.Controller, ControlMath.Unit(nose, car.Forward), -toBall);
            bot.Controller.Boost = false;
            bot.Controller.Throttle = 0f;
            bot.Controller.Jump = false;
        }

        /// <summary>
        /// Fly toward a point <paramref name="offset"/> below/behind the ball along the push, arriving with
        /// <paramref name="closing"/> uu/s toward it. The horizon adapts so the demand stays within ~80% of
        /// available thrust; otherwise each burst swings the demand faster than the car can rotate onto it.
        /// </summary>
        internal static void Shadow(RUBot bot, Car car, Vec3 push, float offset = HoldDistance, float closing = 0f)
        {
            Vec3 demand = Vec3.Zero;
            foreach (float tau in ShadowHorizons)
            {
                Ball target = Ball.Prediction.TrySample(Game.Time + tau, out Ball sample)
                    ? sample : Ball.MainBall.Predict(tau);
                Vec3 hold = target.location - push * offset;
                demand = AerialContact.Guidance(car.Location, car.Velocity, hold,
                    target.velocity + push * closing, tau, Game.Gravity, 0.6f);
                if (demand.Length() < 850f)
                    break;
            }
            Vec3 nose = ControlMath.Unit(demand, car.Forward);
            ControlMath.Aim(car, bot.Controller, nose, -ControlMath.Unit(Ball.Location - car.Location, Vec3.Up));
            bot.Controller.Boost = car.Boost > 0f && demand.Length() > 200f && car.Forward.Dot(nose) > 0.75f;
            bot.Controller.Throttle = ControlRuntime.Axis(demand.Dot(car.Forward) / Car.AirThrottleAccel);
            bot.Controller.Jump = false;
        }

        private static readonly float[] ShadowHorizons = { 0.5f, 0.65f, 0.8f, 1f, 1.25f, 1.5f, 1.8f, 2.2f };

        /// <summary>
        /// After a reset: shoot with the restored flip as soon as a goal-directed airborne strike is feasible;
        /// chain another reset once the flip has been spent; otherwise shadow the ball and wait.
        /// </summary>
        private void Follow(RUBot bot, Car car)
        {
            Vec3 goal = bot.TheirGoal.Location + new Vec3(0, 0, 250);
            if (bot.Jump.CanDodge && Game.Time - phaseStart > 0.15f)
            {
                foreach (BallSlice slice in Ball.Prediction.Slices)
                {
                    if (slice == null || slice.Time < Game.Time + 0.12f)
                        continue;
                    if (slice.Time > Game.Time + 1.6f)
                        break;
                    AerialStrike candidate = AerialStrike.TryCreate(car, slice, goal, dodge: true);
                    if (candidate != null && candidate.AimError < 0.6f)
                    {
                        strike = candidate;
                        Enter(Phase.Shoot);
                        strike.Run(bot);
                        return;
                    }
                }
            }
            else if (!bot.Jump.CanDodge && Resets < MaxResets && Ball.Location.z > 600f)
            {
                Enter(Phase.Shadow);
                Shadow(bot, car, Push(bot));
                return;
            }

            if (Game.Time - phaseStart > 1.8f || Ball.Location.z < 300f)
            {
                Finished = true;
                return;
            }
            Shadow(bot, car, Push(bot));
        }

        private void Enter(Phase next)
        {
            phase = next;
            phaseStart = Game.Time;
            convergedSince = float.NaN;
        }
    }
    /// <summary>
    /// Bouncy air dribble: after our own airborne touch, keep the ball in the air with a chain of soft
    /// nose touches that each lift it and carry it goalward, following on boost, and convert with a firm
    /// strike once a goal-directed contact is modelled on target. Each touch is an ordinary
    /// <see cref="AerialStrike"/> planned from the live prediction; nothing about the chain is scripted.
    /// </summary>
    public sealed class AirDribble : IPossessionAction
    {
        public bool Finished { get; private set; }
        public bool Interruptible => strike == null || !strike.Launched;
        public float ClaimTime => strike?.Slice?.Time ?? Game.Time + 0.3f;
        public int Touches { get; private set; }
        public bool Shooting { get; private set; }

        public const float SoftClosing = 350f;
        public const int MaxTouches = 6;
        /// <summary>Upward ball speed a keep-up touch must leave: enough for the car to follow and re-touch.</summary>
        public const float MinLift = 350f;
        /// <summary>
        /// Keep-up touches come from underneath (contact normal this steep): the car then arrives nose-up and
        /// can keep boosting after the ball instead of rotating a quarter turn while it falls away.
        /// </summary>
        public const float MinUnderside = 0.5f;

        /// <summary>Longest shadow flight between touches before handing back.</summary>
        public const float MaxFollow = 1.2f;

        private AerialStrike strike;
        private readonly float started = Game.Time;
        private float followSince = Game.Time, nextPlan;

        /// <summary>
        /// Worth keeping the ball up: our car is airborne with fuel near a lofted ball that no opponent can
        /// contest soon, and the ball is not in our defensive third.
        /// </summary>
        public static bool CanStart(TacticalFrame frame, Car car, Vec3 attackGoal)
        {
            if (frame == null || car == null || car.IsGrounded || car.Boost < 30f)
                return false;
            if (Ball.Location.z < 350f || frame.OpponentEta < 1.1f)
                return false;
            float attackSide = attackGoal.y > 0 ? 1f : -1f;
            if (Ball.Location.y * attackSide < -2500f)
                return false;
            return car.Location.Dist(Ball.Location) < 1400f;
        }

        public static AirDribble TryCreate(RUBot bot)
        {
            var play = new AirDribble();
            return play.Plan(bot) ? play : null;
        }
        /// <summary>Where a keep-up touch should send the ball: up, and toward the goal we attack.</summary>
        public static Vec3 KeepUpTarget(Vec3 ball, Vec3 attackGoal)
        {
            Vec3 toGoal = ControlMath.FlatUnit(attackGoal - ball, Vec3.Up);
            return ball + toGoal * 1200f + Vec3.Up * 1200f;
        }

        /// <summary>Choose the next contact: a firm on-target finish if one exists, else a soft keep-up.</summary>
        private bool Plan(RUBot bot)
        {
            Car car = bot.Me;
            Vec3 goal = bot.TheirGoal.Location + new Vec3(0, 0, 250);
            bool inRange = Ball.Location.Dist(goal) < 4200f;
            AerialStrike keepUp = null;
            foreach (BallSlice slice in Ball.Prediction.Slices)
            {
                if (slice == null || slice.Time < Game.Time + 0.15f)
                    continue;
                if (slice.Time > Game.Time + 1.6f)
                    break;
                if (inRange && bot.Jump.CanDodge)
                {
                    AerialStrike shot = AerialStrike.TryCreate(car, slice, goal, dodge: true);
                    if (shot != null && shot.AimError < 0.5f)
                    {
                        strike = shot;
                        Shooting = true;
                        return true;
                    }
                }
                if (keepUp == null && slice.Location.z > 400f)
                {
                    AerialStrike touch = AerialStrike.TryCreate(car, slice,
                        KeepUpTarget(slice.Location, bot.TheirGoal.Location), false, SoftClosing, 0f);
                    if (touch != null && touch.AimError < 0.6f && touch.PredictedOutgoing.z > MinLift &&
                        touch.ShotDirection.z > MinUnderside)
                        keepUp = touch;
                }
            }
            strike = keepUp;
            Shooting = false;
            return strike != null;
        }

        public void Run(RUBot bot)
        {
            Car car = bot.Me;
            if (Game.Time - started > 7f || (car.IsGrounded && strike?.Launched != true))
            {
                Finished = true;
                return;
            }
            if (strike != null && !strike.Finished)
            {
                strike.Run(bot);
                return;
            }
            if (strike != null)
            {
                if (Shooting)
                {
                    Finished = true;
                    return;
                }
                Touches++;
                strike = null;
                followSince = Game.Time;
            }
            if (Touches >= MaxTouches || car.Boost < 10f || Ball.Location.z < 300f ||
                Game.Time - followSince > MaxFollow)
            {
                Finished = true;
                return;
            }
            // Between touches: replan a few times a second, and meanwhile shadow the ball from below-behind
            // along the keep-up direction so the next soft touch stays reachable.
            if (Game.Time >= nextPlan)
            {
                nextPlan = Game.Time + 0.05f;
                if (Plan(bot))
                {
                    strike.Run(bot);
                    return;
                }
            }
            Follow(bot, car);
        }

        private static readonly float[] FollowHorizons = { 0.6f, 0.8f, 1f, 1.3f, 1.6f };

        /// <summary>
        /// Fly to a point under-behind the ball's future position (along the keep-up direction), matching
        /// its velocity, with the roof kept as close to its current attitude as the nose allows.
        /// </summary>
        private static void Follow(RUBot bot, Car car)
        {
            Vec3 demand = Vec3.Zero;
            foreach (float tau in FollowHorizons)
            {
                Ball target = Ball.Prediction.TrySample(Game.Time + tau, out Ball sample)
                    ? sample : Ball.MainBall.Predict(tau);
                Vec3 push = ControlMath.Unit(KeepUpTarget(target.location, bot.TheirGoal.Location) - target.location, Vec3.Up);
                Vec3 hold = target.location - push * 240f;
                demand = AerialContact.Guidance(car.Location, car.Velocity, hold, target.velocity, tau, Game.Gravity, 0.5f);
                if (demand.Length() < 900f)
                    break;
            }
            Vec3 nose = ControlMath.Unit(demand, car.Forward);
            Vec3 roof = ControlMath.Unit(car.Up - nose * car.Up.Dot(nose), Vec3.Up);
            ControlMath.Aim(car, bot.Controller, nose, roof);
            bot.Controller.Boost = car.Boost > 0f && demand.Length() > 250f && car.Forward.Dot(nose) > 0.7f;
            bot.Controller.Throttle = ControlRuntime.Axis(demand.Dot(car.Forward) / Car.AirThrottleAccel);
            bot.Controller.Jump = false;
        }
    }
}
