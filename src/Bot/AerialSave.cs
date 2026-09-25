using System;
using RedUtils;
using RedUtils.Math;

namespace Bot
{
    /// <summary>
    /// Airborne block for shots the ground save cannot reach. Instead of a precise nose strike (which a fast
    /// incoming ball simply deflects past), the car puts its broad roof face into the ball's line of travel,
    /// so the ball's own momentum is reflected back out. Launches from the floor or from a wall.
    /// </summary>
    public sealed class AerialSave : IAction
    {
        public bool Finished { get; private set; }
        public bool Interruptible => contact.Interruptible;
        public AerialPlan Plan { get; }
        private readonly AerialContact contact;

        private AerialSave(Car car, AerialPlan plan)
        {
            Plan = plan;
            contact = new AerialContact(car, plan)
            {
                // Keep the nose pointing upfield-ish so the recovery after the block faces play.
                Heading = ControlMath.FlatUnit(-plan.BallVelocity, car.Forward),
                DriftLimit = 240f,
            };
        }

        /// <summary>
        /// Push that reflects the ball away from the goal: opposite the ball's travel, tilted up and away
        /// from the goal centre so a partial block still clears wide/high rather than into the net.
        /// </summary>
        public static Vec3 BlockPush(BallSlice slice, Vec3 ownGoal)
        {
            Vec3 back = ControlMath.Unit(-slice.Velocity, ControlMath.Unit(slice.Location - ownGoal, Vec3.Up));
            Vec3 away = ControlMath.Unit((slice.Location - ownGoal).Flatten(), back);
            return ControlMath.Unit(back * 0.6f + away * 0.4f + Vec3.Up * 0.35f, back);
        }

        /// <summary>Earliest feasible airborne block before <paramref name="deadline"/> seconds.</summary>
        public static AerialSave TryCreate(Car car, Vec3 ownGoal, float deadline)
        {
            if (car == null || car.Boost < 6f)
                return null;
            AerialPlan plan = AerialPlanner.Find(car, 0.15f, MathF.Min(deadline, 3f), ContactFace.Roof, 250f, false,
                slice =>
                {
                    if (slice.Location.z < 180f || !Defense.IsGoalSide(car.Location, slice.Location, ownGoal, -150f))
                        return (false, Vec3.Zero);
                    // Only threatening balls: moving toward our goal.
                    Vec3 toGoal = ControlMath.Unit(ownGoal - slice.Location, Vec3.Up);
                    if (slice.Velocity.Dot(toGoal) < 300f)
                        return (false, Vec3.Zero);
                    return (true, BlockPush(slice, ownGoal));
                }, 70, allowApproach: true);
            if (plan == null)
                return null;
            plan.ArriveEarly = true;
            // A ground launch toward a low contact is the ground save's job.
            if (car.IsGrounded && (plan.ContactPoint - car.Location).Dot(car.Up) < 200f)
                return null;
            return new AerialSave(car, plan);
        }

        public void Run(RUBot bot)
        {
            contact.Run(bot);
            if (contact.Finished)
                Finished = true;
        }
    }
}
