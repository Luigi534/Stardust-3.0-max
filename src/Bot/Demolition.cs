using System;
using RedUtils;
using RedUtils.Math;

namespace Bot
{
    /// <summary>A planned car-to-car hit: target, intercept point/time, and whether it should be a demolition.</summary>
    public sealed class HitPlan
    {
        public Car Target;
        public Vec3 Point;
        public float Time;
        public bool Demolition;
        public string Reason;
    }

    /// <summary>
    /// Demolitions and bumps as tactical tools. The planner leads each grounded opponent along its current
    /// velocity, finds the earliest time our drive model reaches that point, and estimates our arrival speed:
    /// supersonic arrival demolishes, otherwise the hit is a bump. Hits are only chosen when they help — to
    /// stop a ball carrier we cannot beat to the ball, to clear the last defender while our team attacks, or
    /// opportunistically while already fast on a non-critical task.
    /// </summary>
    public static class Demolitions
    {
        public const float DemoSpeed = 2200f;
        public const float BumpSpeed = 1300f;

        /// <summary>Straight-line arrival speed after <paramref name="distance"/> at full throttle + boost.</summary>
        public static float ArrivalSpeed(float distance, float speed, float fuel)
        {
            speed = System.Math.Clamp(speed, 0f, Car.MaxSpeed);
            float covered = 0f;
            const float dt = 0.05f;
            for (int i = 0; i < 80 && covered < distance; i++)
            {
                float boostFraction = speed < Car.MaxSpeed ? MathF.Min(1f, fuel / (Car.BoostConsumption * dt)) : 0f;
                float next = MathF.Min(Car.MaxSpeed, speed + (DrivePhysics.ThrottleAcceleration(speed) + Car.BoostAccel * boostFraction) * dt);
                covered += (speed + next) * 0.5f * dt;
                fuel = MathF.Max(0f, fuel - boostFraction * Car.BoostConsumption * dt);
                speed = next;
            }
            return speed;
        }

        /// <summary>Earliest reachable hit on <paramref name="target"/> within <paramref name="horizon"/> seconds.</summary>
        public static HitPlan Plan(Car me, Car target, float horizon = 1.6f)
        {
            if (me == null || target == null || target.IsDemolished || !me.IsGrounded || me.Up.z < 0.7f)
                return null;
            // Grounded targets only: an airborne car's path is not a straight line on a surface.
            if (!target.IsGrounded || target.Location.z > 120f)
                return null;

            for (float t = 0.1f; t <= horizon; t += 0.05f)
            {
                Vec3 point = target.Location + target.Velocity.Flatten() * t;
                if (!Field.InField(point, 150f))
                    break;
                point = new Vec3(point.x, point.y, 17f);
                float eta = Drive.GetEta(me, point);
                if (!float.IsFinite(eta) || eta > t)
                    continue;
                // Must arrive roughly straight: a steep turn at the end costs the speed.
                Vec3 approach = ControlMath.FlatUnit(point - me.Location, me.Forward);
                if (me.Forward.Flatten().Dot(approach) < 0.5f && me.Location.FlatDist(point) < 1200f)
                    return null;
                float speed = ArrivalSpeed(me.Location.FlatDist(point), MathF.Max(0f, me.Velocity.Dot(approach)), me.Boost);
                if (speed < BumpSpeed)
                    return null;
                return new HitPlan { Target = target, Point = point, Time = Game.Time + t, Demolition = speed >= DemoSpeed };
            }
            return null;
        }

        /// <summary>
        /// A useful hit for this situation, or null.
        /// </summary>
        public static HitPlan Choose(Stardust bot, TacticalFrame frame, bool emergency)
        {
            Car me = bot.Me;
            if (frame == null || me == null || !me.IsGrounded)
                return null;

            float side = Field.Side(bot.Team);
            HitPlan best = null;
            foreach (Car opponent in bot.LivingOpponents)
            {
                HitPlan plan = Plan(me, opponent);
                if (plan == null)
                    continue;

                float opponentToBall = opponent.Location.Dist(Ball.Location);
                bool carrier = opponentToBall < 260f && (opponent.Velocity - Ball.Velocity).Length() < 700f &&
                    Ball.Location.z < 320f;
                bool attackingUs = (Ball.Location - opponent.Location).Dot(new Vec3(0, side, 0)) > -100f &&
                    Ball.Velocity.y * side > -200f;
                // Disrupt a ball carrier we cannot beat to the ball: the hit arrives before they can shoot.
                if (carrier && attackingUs && frame.FreeTime < 0f && plan.Time - Game.Time < 1.1f)
                {
                    plan.Reason = "carrier";
                    return plan;
                }

                if (emergency)
                    continue;

                // Clear the last defender while a teammate (or we) threaten their goal.
                bool lastDefender = true;
                foreach (Car other in bot.LivingOpponents)
                    if (other != opponent && other.Location.y * -side > opponent.Location.y * -side)
                        lastDefender = false;
                bool weAttack = Ball.Location.y * side < -1500f && frame.TeamCount > 1 && frame.TeamRank > 0;
                if (lastDefender && weAttack && plan.Demolition && opponent.Location.y * side < -3500f)
                {
                    plan.Reason = "goalie";
                    best = plan;
                    continue;
                }

                // Opportunistic demolition while already supersonic on a support/rotation task.
                if (plan.Demolition && me.IsSupersonic && plan.Time - Game.Time < 0.6f && frame.TeamRank > 0 &&
                    !frame.LastBack && best == null)
                {
                    plan.Reason = "opportunistic";
                    best = plan;
                }
            }
            return best;
        }
    }

    /// <summary>Drive through a car-hit plan at full speed, re-leading the target every tick.</summary>
    public sealed class Demolish : IAction
    {
        public HitPlan Plan { get; private set; }
        public bool Finished { get; private set; }
        public bool Interruptible => true;
        private readonly Drive drive;
        private readonly float started = Game.Time;
        private readonly uint demosAtStart;

        public Demolish(Car me, HitPlan plan)
        {
            Plan = plan;
            drive = new Drive(me, plan.Point, Car.MaxSpeed, allowDodges: false, wasteBoost: true);
            demosAtStart = me.Demolitions;
        }

        public void Run(RUBot bot)
        {
            Car me = bot.Me;
            Car target = Plan.Target;
            if (target == null || target.IsDemolished || me.Demolitions > demosAtStart || !me.IsGrounded ||
                Game.Time - started > 2.2f || Game.Time > Plan.Time + 0.5f)
            {
                Finished = true;
                return;
            }
            HitPlan fresh = Demolitions.Plan(me, target, MathF.Max(0.3f, Plan.Time - Game.Time + 0.4f));
            if (fresh == null && me.Location.Dist(target.Location) > 400f)
            {
                Finished = true;
                return;
            }
            if (fresh != null)
            {
                fresh.Reason = Plan.Reason;
                Plan = fresh;
            }
            // Aim at the target itself once close; the lead point otherwise.
            drive.Target = me.Location.Dist(target.Location) < 500f ? new Vec3(target.Location.x, target.Location.y, 17f) : Plan.Point;
            drive.TargetSpeed = Car.MaxSpeed;
            drive.WasteBoost = true;
            drive.AllowDodges = false;
            drive.Run(bot);
            bot.Controller.Boost = me.Boost > 0f;
        }
    }
}
