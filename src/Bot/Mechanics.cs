using System;
using RedUtils;
using RedUtils.Math;

namespace Bot
{
    public sealed class GroundDribble : IPossessionAction
    {
        private readonly float started = Game.Time;
        private float stableSince = float.NaN;
        public bool Finished { get; private set; }
        public bool Interruptible => true;
        public float ClaimTime => Game.Time + 0.25f;
        public static bool CanStart(Car car, Ball ball, float freeTime)
        {
            if (car == null || ball == null || !car.IsGrounded || car.Up.z <= 0.9f)
                return false;

            Vec3 local = car.Local(ball.location - car.Location);
            float relativeSpeed = (ball.velocity - car.Velocity).Length();
            bool alreadyControlled = PossessionControl.HasControlledPossession(car, ball);
            return ball.location.z < 300f &&
                local.x > -105f && local.x < 590f && MathF.Abs(local.y) < 195f &&
                relativeSpeed < 1100f && (alreadyControlled || freeTime > 0.10f);
        }
        public void Run(RUBot bot)
        {
            Car car = bot.Me;
            Vec3 offset = Ball.Location - car.Location;
            if (!car.IsGrounded || offset.Length() > 850f || Game.Time - started > 8f || Ball.Location.z > 365f)
            { Finished = true; return; }

            Vec3 lane = PossessionControl.AttackingLane(
                car, Ball.MainBall, bot.LivingOpponents,
                bot.TheirGoal.Location, bot.OurGoal.Location);
            bool carried = PossessionControl.HasControlledPossession(car, Ball.MainBall);
            stableSince = carried ? (float.IsFinite(stableSince) ? stableSince : Game.Time) : float.NaN;

            float pressure = bot is Stardust stardust
                ? MathF.Min(stardust.Situation.OpponentEta, stardust.Situation.PressureTime)
                : 6f;
            float opponentDistance = float.PositiveInfinity;
            foreach (Car opponent in bot.LivingOpponents)
                opponentDistance = MathF.Min(opponentDistance, opponent.Location.Dist(Ball.Location));

            float requiredStable = pressure < 0.40f || opponentDistance < 450f ? 0.06f : 0.13f;
            if (carried && Game.Time - stableSince >= requiredStable &&
                PossessionControl.ShouldFlick(
                    car, Ball.MainBall, lane, pressure, opponentDistance, bot.OurGoal.Location))
            {
                bot.Action = new ControlledFlick(car, lane);
                return;
            }

            // Stay on the ball under pressure. The pressure response is the flick above, not abandoning
            // possession and driving back into a shadow lane.
            bot.Controller = PossessionControl.GroundCarry(car, Ball.MainBall, lane);
        }
    }

    public sealed class GroundCatch : IPossessionAction
    {
        private readonly float started = Game.Time;
        private Drive drive;
        public bool Finished { get; private set; }
        public bool Interruptible => drive?.Interruptible ?? true;
        public float ClaimTime { get; private set; }
        public static BallSlice FindCatch(Car car)
        {
            if (!car.IsGrounded || Ball.Prediction.Slices == null) return null;
            float next = Game.Time + 0.15f;
            foreach (BallSlice slice in Ball.Prediction.Slices)
            {
                if (slice == null || slice.Time < next) continue;
                float time = slice.Time - Game.Time;
                if (time > 1.5f) break;
                next = slice.Time + 0.04f;
                if (slice.Location.z < 105 || slice.Location.z > 175 || slice.Velocity.z > -80) continue;
                float eta = Drive.GetEta(car, slice.Location.Flatten());
                if (float.IsFinite(eta) && eta < time - 0.06f) return slice;
            }
            return null;
        }
        public void Run(RUBot bot)
        {
            if (Game.Time - started > 1.8f || !bot.Me.IsGrounded || GroundDribble.CanStart(bot.Me, Ball.MainBall, 1))
            { Finished = true; return; }
            BallSlice catchSlice = FindCatch(bot.Me);
            if (catchSlice == null) { Finished = true; return; }
            ClaimTime = catchSlice.Time;
            Vec3 lane = ControlMath.FlatUnit(bot.TheirGoal.Location - catchSlice.Location, bot.Me.Forward);
            Vec3 target = (catchSlice.Location - lane * 40).Flatten();
            float time = MathF.Max(0.1f, catchSlice.Time - Game.Time);
            float distance = bot.Me.Location.FlatDist(target);
            float speed = System.Math.Clamp(distance / time, 100, 1800);
            if (distance < 300) speed = MathF.Min(speed, catchSlice.Velocity.FlatLen() + 150);
            drive ??= new Drive(bot.Me, target, speed, allowDodges: false, wasteBoost: false);
            drive.Target = target;
            drive.TargetSpeed = MathF.Max(100, speed);
            drive.Run(bot);
            bot.Controller.Boost = false;
            bot.Controller.Handbrake = false;
        }
    }

    public sealed class ControlledFlick : IPossessionAction
    {
        private readonly JumpSequence jumps = new();
        private readonly Vec3 direction;
        private readonly float started = Game.Time;
        private float elapsed;
        public bool Finished { get; private set; }
        public bool Interruptible => elapsed > 0.65f;
        public float ClaimTime => started + 0.25f;
        public ControlledFlick(Car car, Vec3 direction) { this.direction = ControlMath.Unit(direction, car.Forward); }
        public void Run(RUBot bot)
        {
            elapsed = Game.Time - started;
            if (elapsed > 0.9f || (elapsed > 0.3f && bot.Me.IsGrounded)) { Finished = true; return; }
            JumpCommand command = jumps.Step(Game.Time, bot.Jump.CanDodge);
            ControlMath.Aim(bot.Me, bot.Controller, direction, Vec3.Up);
            bot.Controller.Jump = command.Jump;
            bot.Controller.Boost = false;
            bot.Controller.Throttle = 1;
            if (command.Dodge)
            {
                Vec3 local = ControlMath.FlatUnit(bot.Me.Local(direction), new Vec3(1, 0, 0));
                bot.Controller.Pitch = -local.x;
                bot.Controller.Yaw = local.y;
                bot.Controller.Roll = 0;
            }
            else if (!jumps.Fired) bot.Controller.Pitch = -0.15f;
        }
    }

    public sealed class AerialCarry : IPossessionAction
    {
        private readonly BoostGate boost = new();
        private readonly float started = Game.Time;
        public bool Finished { get; private set; }
        public bool Interruptible => true;
        public float ClaimTime => Game.Time + 0.3f;
        public static bool CanStart(Car car, Ball ball, float opponentEta)
        {
            if (car == null || ball == null || car.IsGrounded || car.Location.z <= 180f ||
                ball.location.z <= 300f || car.Boost <= 8f || opponentEta <= 0.18f)
                return false;

            Vec3 delta = ball.location - car.Location;
            float distance = delta.Length();
            if (delta.z <= 20f || delta.z >= 440f || distance >= 690f)
                return false;

            Vec3 relativeVelocity = car.Velocity - ball.velocity;
            float relativeSpeed = relativeVelocity.Length();
            if (relativeSpeed >= 1100f)
                return false;

            // A close, already-controlled air dribble may tolerate a small temporary separation.
            // A distant ball that is rapidly moving away is not a carry start; boosting after it
            // just burns the recovery budget.
            float closing = relativeVelocity.Dot(ControlMath.Unit(delta, Vec3.Up));
            float allowedSeparation = distance < 220f ? -420f : -220f;
            return closing >= allowedSeparation;
        }
        public void Run(RUBot bot)
        {
            Car car = bot.Me;
            Vec3 delta = Ball.Location - car.Location;
            float distance = delta.Length();
            float separationClosing = (car.Velocity - Ball.Velocity)
                .Dot(ControlMath.Unit(delta, Vec3.Up));
            if (car.IsGrounded || distance > 900f || Ball.Location.z < 180f ||
                Game.Time - started > 4f ||
                (car.Boost <= 0f && distance > 220f) ||
                (distance > 360f && separationClosing < -360f))
            {
                Finished = true;
                return;
            }
            if (bot is Stardust stardust && stardust.Options.FlipResets &&
                stardust.Situation.OpponentEta > 1.2f && FlipReset.CanStart(car, Ball.MainBall, bot.Jump))
            {
                FlipResetPlay play = FlipResetPlay.TryCreate(bot);
                if (play != null) { bot.Action = play; return; }
            }
            const float horizon = 0.12f;
            Ball prediction = Ball.Prediction.TrySample(Game.Time + horizon, out Ball sample) ? sample : Ball.MainBall.Predict(horizon);
            Vec3 lane = PossessionControl.AttackingLane(
                car, prediction, bot.LivingOpponents,
                bot.TheirGoal.Location, bot.OurGoal.Location);
            float pressure = bot is Stardust stardustPressure
                ? MathF.Min(stardustPressure.Situation.OpponentEta, stardustPressure.Situation.PressureTime)
                : float.PositiveInfinity;
            bool challenged = float.IsFinite(pressure) && pressure < 0.65f;
            float forwardPush = challenged ? 190f : 90f;
            float lift = pressure < 0.40f ? 35f : 80f;
            Vec3 contactNormal = ControlMath.Unit(
                lane * (challenged ? 0.62f : 0.48f) + Vec3.Up * 0.88f, Vec3.Up);
            Vec3 target = prediction.location - contactNormal * (Ball.Radius + 40);
            Vec3 targetVelocity = prediction.velocity + lane * forwardPush + Vec3.Up * lift;
            Vec3 acceleration = PossessionControl.FlightAtHorizon(car, target, targetVelocity, horizon);
            Vec3 nose = ControlMath.Unit(acceleration, car.Forward);
            ControlMath.Aim(car, bot.Controller, nose, Vec3.Up);
            float closing = (car.Velocity - Ball.Velocity).Dot(ControlMath.Unit(delta, Vec3.Up));
            bool gentle = delta.Length() < 185 && closing > 160;
            bot.Controller.Boost = boost.Step(Game.Time, acceleration.Dot(car.Forward), car.Forward.Dot(nose), car.Boost, gentle);
            bot.Controller.Throttle = gentle ? 0 : 1;
            bot.Controller.Jump = false;
        }
    }

    /// <summary>Experimental, evidence-gated acquisition. Enable with STARDUST_FLIP_RESETS=1.</summary>
    public sealed class FlipReset : IPossessionAction
    {
        private readonly ResetEvidence evidence = new();
        private readonly BoostGate boost = new();
        private readonly float started = Game.Time;
        private float confirmedAt = float.NaN, firedAt = float.NaN;
        public bool Finished { get; private set; }
        public bool Interruptible => !float.IsFinite(firedAt);
        public float ClaimTime => Game.Time + 0.25f;
        public FlipReset(JumpState initialState)
        {
            // Capture spent state at selection, even if contact happens before the next control tick.
            evidence.Observe(initialState, false, false, 0, Game.Time);
        }
        public static bool CanStart(Car car, Ball ball, JumpState jump)
        {
            Vec3 delta = ball.location - car.Location;
            return !car.IsGrounded && car.Location.z > 350 && ball.location.z > 550 && car.Boost > 20 &&
                (jump.DoubleJumped || jump.Dodged) && delta.z > 60 && delta.z < 280 && delta.Length() < 360 &&
                (car.Velocity - ball.velocity).Length() < 650;
        }
        public void Run(RUBot bot)
        {
            Car car = bot.Me;
            Vec3 delta = Ball.Location - car.Location;
            if (car.Location.z < 180 || delta.Length() > 650 || Game.Time - started > 2)
            { Finished = true; return; }
            Vec3 towardBall = ControlMath.Unit(delta, Vec3.Up);
            Vec3 lane = ControlMath.FlatUnit(bot.TheirGoal.Location - Ball.Location, car.Forward);
            bool wheelsAligned = (-car.Up).Dot(towardBall) > 0.85f;
            bool confirmed = evidence.Observe(bot.Jump, bot.OwnTouchThisTick, wheelsAligned, car.Location.z, Game.Time);
            bot.Controller.Jump = false;
            bot.Controller.Boost = false;
            if (confirmed)
            {
                if (!float.IsFinite(confirmedAt)) confirmedAt = Game.Time;
                ControlMath.Aim(car, bot.Controller, towardBall, Vec3.Up);
                if (!float.IsFinite(firedAt) && Game.Time - confirmedAt > 0.08f && bot.Jump.CanDodge &&
                    delta.Length() < 230 && car.Forward.Dot(towardBall) > 0.8f && car.AngularVelocity.Length() < 2.5f)
                    firedAt = Game.Time;
                if (float.IsFinite(firedAt))
                {
                    bool pulse = Game.Time - firedAt < 0.05f;
                    bot.Controller.Jump = pulse;
                    if (pulse)
                    {
                        Vec3 local = ControlMath.FlatUnit(car.Local(towardBall), new Vec3(1, 0, 0));
                        bot.Controller.Pitch = -local.x;
                        bot.Controller.Yaw = local.y;
                        bot.Controller.Roll = 0;
                    }
                    Finished = Game.Time - firedAt > 0.8f;
                }
                else if (Game.Time - confirmedAt > 0.55f) Finished = true;
                return;
            }
            if (Game.Time - started > 1.35f) { Finished = true; return; }
            const float horizon = 0.08f;
            Ball prediction = Ball.MainBall.Predict(horizon);
            Vec3 target = prediction.location - Vec3.Up * (Ball.Radius + 18) - lane * 20;
            Vec3 acceleration = PossessionControl.FlightAtHorizon(car, target, prediction.velocity, horizon);
            if (delta.Length() > 220)
            {
                Vec3 nose = ControlMath.Unit(acceleration, car.Forward);
                ControlMath.Aim(car, bot.Controller, nose, -Vec3.Up);
                bot.Controller.Boost = boost.Step(Game.Time, acceleration.Dot(car.Forward), car.Forward.Dot(nose), car.Boost, false);
            }
            else
            {
                // Coast into wheel contact rather than boosting the ball away.
                ControlMath.Aim(car, bot.Controller, lane, -Vec3.Up);
            }
        }
    }

    public sealed class Recover : IAction
    {
        private readonly float started = Game.Time;
        public bool Finished { get; private set; }
        public bool Interruptible => true;
        public void Run(RUBot bot)
        {
            Car car = bot.Me;
            if (car.IsGrounded || Game.Time - started > 1.5f) { Finished = true; return; }
            float time = car.PredictLandingTime();
            time = float.IsFinite(time) ? System.Math.Clamp(time, 0, 2) : 0.3f;
            Vec3 normal = Field.NearestSurface(car.PredictLocation(time)).Normal;
            Vec3 tangent = car.Velocity - normal * car.Velocity.Dot(normal);
            Vec3 fallback = bot.TheirGoal.Location - car.Location;
            fallback -= normal * fallback.Dot(normal);
            ControlMath.Aim(car, bot.Controller, ControlMath.Unit(tangent, ControlMath.Unit(fallback, car.Forward)), normal);
            bot.Controller.Throttle = 1;
            bot.Controller.Boost = false;
            bot.Controller.Jump = false;
            bot.Controller.Handbrake = time < 0.1f && tangent.Length() > 800;
        }
    }
}
