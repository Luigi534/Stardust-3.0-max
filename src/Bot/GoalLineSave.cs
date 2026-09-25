using System;
using RedUtils;
using RedUtils.Math;

namespace Bot
{
    /// <summary>
    /// Final-line save controller. It preserves precise goal-mouth driving, but unlike a pure
    /// DefensiveDrive it can leave the ground when the predicted goal crossing is elevated.
    /// </summary>
    public sealed class GoalLineSave : IAction
    {
        public bool Finished { get; private set; }
        public bool Interruptible =>
            !jumping || (float.IsFinite(jumpStarted) && Game.Time - jumpStarted > 0.68f);

        public Vec3 Crossing { get; set; }
        public float CrossingTime { get; set; }
        public Vec3 GuardTarget { get; private set; }
        public bool Jumping => jumping;
        public bool UsesDoubleJump => doubleJump;

        private readonly DefensiveDrive drive;
        private readonly JumpSequence jumps = new(0.16f);
        private bool jumping;
        private bool doubleJump;
        private float jumpStarted = float.NaN;

        public GoalLineSave(Car car, Vec3 crossing, float crossingTime)
        {
            Crossing = crossing;
            CrossingTime = crossingTime;
            GuardTarget = crossing;
            drive = new DefensiveDrive(
                car, crossing, Car.MaxSpeed, 0f,
                holdPosition: true, allowDodges: false);
        }

        public void Run(RUBot bot)
        {
            if (bot == null || !ControlMath.Finite(Crossing) ||
                !float.IsFinite(CrossingTime))
            {
                Finished = true;
                return;
            }

            Car car = bot.Me;
            float timeRemaining = CrossingTime - Game.Time;
            if (!float.IsFinite(timeRemaining) || timeRemaining < -0.18f)
            {
                Finished = true;
                return;
            }

            GuardTarget = Defense.EmergencyTarget(Crossing, bot.OurGoal.Location, Ball.Location);
            // If the straight route to the guard point would clip the ball from the field side, go around
            // it on the side the car is already on.
            bool nearLowBall = car.IsGrounded && car.Location.FlatDist(Ball.Location) <= 900f && Ball.Location.z <= 250f;
            if (nearLowBall && !Defense.SafeContactTarget(car, Ball.Location, bot.OurGoal.Location, out _))
            {
                Vec3 away = ControlMath.FlatUnit(Ball.Location - Defense.NetAnchor(bot.OurGoal.Location), Vec3.Y);
                Vec3 lateral = new Vec3(-away.y, away.x, 0);
                if (lateral.Dot(car.Location - Ball.Location) < 0f)
                    lateral = -lateral;
                GuardTarget = new Vec3(Ball.Location.x, Ball.Location.y, 17f) + lateral * 280f - away * 120f;
            }

            if (!jumping)
            {
                drive.Target = GuardTarget;
                drive.CruiseSpeed = Car.MaxSpeed;
                drive.TerminalSpeed = 0f;
                drive.HoldPosition = true;
                drive.AllowDodges = false;
                drive.Run(bot);

                // A low crossing is best covered by staying on the wheels. For an elevated crossing,
                // start the jump only once lateral positioning is close enough and the vertical
                // flight time matches the remaining shot time.
                if (Crossing.z <= 155f || !car.IsGrounded)
                    return;

                float blockHeight = System.Math.Clamp(Crossing.z - 120f, 35f, 430f);
                doubleJump = blockHeight > 235f;
                float jumpTime = Utils.TimeToJump(Vec3.Up, blockHeight, doubleJump);
                if (!float.IsFinite(jumpTime) || jumpTime <= 0f)
                    jumpTime = doubleJump ? 0.55f : 0.32f;

                float lateralError = MathF.Abs(car.Location.x - GuardTarget.x);
                bool positioned = lateralError <= 650f ||
                    car.Location.FlatDist(GuardTarget) <= 760f;

                if (positioned && timeRemaining <= jumpTime + 0.12f)
                {
                    jumping = true;
                    jumpStarted = Game.Time;
                }
                else
                    return;
            }

            Vec3 towardCrossing = ControlMath.Unit(
                Crossing - car.Location, car.Forward);
            ControlMath.Aim(car, bot.Controller, towardCrossing, Vec3.Up);
            JumpCommand command = jumps.Step(
                Game.Time, doubleJump && !car.HasDoubleJumped);

            bot.Controller.Jump = command.Jump;
            bot.Controller.Throttle = 1f;
            bot.Controller.Boost = false;
            bot.Controller.Handbrake = false;

            // The second press should be a neutral double jump. Directional input here would turn
            // a vertical block into an accidental dodge across the mouth.
            if (command.Dodge)
            {
                bot.Controller.Pitch = 0f;
                bot.Controller.Yaw = 0f;
                bot.Controller.Roll = 0f;
            }

            if (float.IsFinite(jumpStarted) && Game.Time - jumpStarted > 0.30f &&
                car.IsGrounded)
                Finished = true;
        }
    }
}
