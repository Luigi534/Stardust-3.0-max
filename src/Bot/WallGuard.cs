using System;
using RedUtils;
using RedUtils.Math;

namespace Bot
{
    /// <summary>
    /// "Spiderman" defence: instead of parking on the goal line, the last defender hangs on its own back wall
    /// beside or above the goal, tracking the ball's lateral position, and jumps off the wall into the ball
    /// when an intercept that sends it wide/upfield becomes reachable. The back wall position covers the
    /// high part of the goal mouth, keeps a jump available, and turns a save into an upfield touch. The
    /// geometry (which side, how high, when to release) is derived from the live ball and threat, not from
    /// any recording.
    /// </summary>
    public sealed class WallGuard : IAction
    {
        public bool Finished { get; private set; }
        public bool Interruptible => release == null || !release.Airborne;
        public bool OnWall { get; private set; }
        public bool Released => release != null;

        private readonly int team;
        private readonly Drive drive;
        private AerialContact release;
        private readonly float started = Game.Time;

        public WallGuard(Car car, int team)
        {
            this.team = team;
            drive = new Drive(car, HangPoint(team), 1400f, allowDodges: false, wasteBoost: false);
        }

        public static float BackWallY(int team) => Field.Side(team) * (Field.Length * 0.5f - 17f);

        /// <summary>Where to hang: the ball's side of the goal, beyond the post or above the bar.</summary>
        public static Vec3 HangPoint(int team)
        {
            Vec3 reference = Ball.Prediction.TrySample(Game.Time + 0.6f, out Ball later) ? later.location : Ball.Location;
            float side = MathF.Sign(reference.x == 0 ? 1f : reference.x);
            float x = side * System.Math.Clamp(MathF.Abs(reference.x) * 0.7f, 1050f, 2500f);
            float z = System.Math.Clamp(reference.z * 0.55f + 260f, 300f, 820f);
            return new Vec3(x, BackWallY(team), z);
        }

        /// <summary>
        /// Worth hanging: we are the last defender close to our goal and the threat is an elevated ball near a
        /// side or back wall (lobs, wall plays, air dribbles) — where a floor position covers the top of the
        /// goal poorly. A ball on the floor, or a grounded carrier, is left to the ordinary shadow/save.
        /// </summary>
        public static bool Worthwhile(TacticalFrame frame, Car car, Vec3 ownGoal, float threatTime,
            System.Collections.Generic.IEnumerable<Car> opponents)
        {
            if (frame == null || car == null || !frame.LastBack || car.IsDemolished || car.Boost < 12f)
                return false;
            float side = ownGoal.y < 0 ? -1f : 1f;
            Vec3 ball = Ball.Location;
            float ballDepth = ball.y * side;
            if (ballDepth < 2300f || ballDepth > 5000f)
                return false;
            if (ball.z < 320f || !Ball.Prediction.TrySample(Game.Time + 0.8f, out Ball later) || later.location.z < 260f)
                return false;
            bool nearWall = MathF.Abs(ball.x) > Field.Width * 0.5f - 1300f || ballDepth > Field.Length * 0.5f - 1300f;
            if (!nearWall)
                return false;
            foreach (Car opponent in opponents)
                if (opponent != null && opponent.IsGrounded && opponent.Location.Dist(ball) < 350f && ball.z < 400f)
                    return false;
            return car.Location.Dist(ownGoal) < 1900f;
        }

        public void Run(RUBot bot)
        {
            Car car = bot.Me;
            if (Game.Time - started > 6f)
            {
                Finished = true;
                return;
            }

            if (release != null)
            {
                release.Run(bot);
                if (release.Finished)
                    Finished = true;
                return;
            }

            float wallY = BackWallY(team);
            OnWall = car.IsGrounded && MathF.Abs(car.Location.y - wallY) < 60f && car.Location.z > 120f;

            // Release: a reachable contact that sends the ball away from our goal.
            if (car.IsGrounded)
            {
                Vec3 ownGoal = bot.OurGoal.Location;
                AerialPlan plan = AerialPlanner.Find(car, 0.15f, 1.8f, ContactFace.Nose, 700f, false, slice =>
                {
                    if (slice.Location.z < 160f)
                        return (false, Vec3.Zero);
                    Vec3 away = ControlMath.FlatUnit(slice.Location - ownGoal, Vec3.Up);
                    Vec3 upfield = new Vec3(0, -Field.Side(team), 0);
                    Vec3 push = ControlMath.Unit(away * 0.6f + upfield * 0.6f + Vec3.Up * 0.35f, upfield);
                    return (true, push);
                }, 55);
                bool threatening = plan != null &&
                    (Ball.Velocity.y * Field.Side(team) > 150f || Ball.Location.y * Field.Side(team) > 4200f);
                if (plan != null && !plan.Deferred && (OnWall || threatening))
                {
                    release = new AerialContact(car, plan) { DodgeAtContact = true };
                    release.Run(bot);
                    return;
                }
            }

            // Leave the wall for a ground ball rolling at the goal: the floor save covers it better.
            float side = Field.Side(team);
            if (Ball.Location.z < 200f && Ball.Location.y * side > 3800f && MathF.Abs(Ball.Location.x) < 1300f &&
                Ball.Velocity.y * side > 200f)
            {
                Finished = true;
                return;
            }

            Vec3 hang = HangPoint(team);
            drive.Target = hang;
            float distance = car.Location.Dist(hang);
            drive.TargetSpeed = OnWall ? MathF.Min(900f, 150f + distance * 2f) : MathF.Min(Car.MaxSpeed, 400f + distance * 1.5f);
            drive.AllowDodges = false;
            drive.WasteBoost = !OnWall;
            drive.Run(bot);
            if (OnWall)
            {
                // Hold height against wall slide: gentle throttle up the wall when below the hang point.
                bot.Controller.Throttle = System.Math.Clamp((hang.z - car.Location.z) / 150f + 0.25f, -1f, 1f);
                bot.Controller.Boost = false;
                bot.Controller.Handbrake = false;
            }
        }
    }
}
