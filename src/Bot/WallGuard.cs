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
            // Hang above the likely shot height (reference saves release at 450-960 uu and fall into the contact).
            float z = System.Math.Clamp(reference.z * 0.55f + 450f, 550f, 850f);
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

    /// <summary>
    /// Spiderman save: from a wall (the back wall beside the goal, or a side wall), leap off into the path of a shot
    /// that is about to reach our goal and strike it nose-first upfield/away before it crosses the line. Human
    /// Spiderman saves in the reference replays are made exactly like this: never from the wall itself, but by
    /// jumping off it into the ball 500-1100 uu in front of the goal line. The plan is an analytic reachability check
    /// over the ball prediction (no simulation); the flight is an ordinary <see cref="AerialContact"/>.
    /// </summary>
    public sealed class WallRelease : IAction
    {
        public bool Finished { get; private set; }
        public bool Interruptible => false;
        public bool Launched => phase != Phase.Windup;

        private enum Phase { Windup, Jump, Fly, Dodge }
        private Phase phase;
        private float phaseStart = Game.Time;
        private readonly float started = Game.Time;
        private readonly Vec3 ownGoal;
        private readonly Drive windup;
        private Vec3 dodgeDirection;

        /// <summary>Upward impulse (along the wall normal) of a held jump + neutral double jump, minus losses.</summary>
        public const float LaunchImpulse = 420f;
        /// <summary>Planned thrust margin: a block only needs the body in the ball's path, but keep headroom.</summary>
        public const float MaxDemand = 950f;
        /// <summary>Largest angle between the nose and the required thrust at which boosting starts immediately.</summary>
        public const float MaxBoostAngle = 0.6f;
        /// <summary>Launch when the straight dash passes this close to the ball's path.</summary>
        public const float LaunchMiss = 120f;
        /// <summary>Only launch for contacts this soon: an early jump crosses the goal mouth before a slow shot.</summary>
        public const float LaunchHorizon = 2.2f;
        /// <summary>Block no further out than this from the goal line (reference saves: 500-1100 uu out).</summary>
        public const float BlockReach = 1400f;

        private WallRelease(Car car, Vec3 ownGoal, bool launchNow)
        {
            this.ownGoal = ownGoal;
            windup = new Drive(car, WindupTarget(car), Car.MaxSpeed, allowDodges: false, wasteBoost: true);
            phase = launchNow ? Phase.Jump : Phase.Windup;
        }

        /// <summary>On a wall (not the floor or ceiling), with wheels down on it.</summary>
        public static bool OnWall(Car car) =>
            car != null && car.IsGrounded && MathF.Abs(car.Up.z) < 0.5f && car.Location.z > 80f;

        /// <summary>On our back wall, beside or above the goal.</summary>
        public static bool OnBackWall(Car car, Vec3 ownGoal) =>
            OnWall(car) && car.Location.y * MathF.Sign(ownGoal.y) > Field.Length * 0.5f - 120f;

        /// <summary>Wind-up target: along the back wall toward the goal centre, stopping short of the post.</summary>
        public static Vec3 WindupTarget(Car car)
        {
            float sx = MathF.Sign(car.Location.x == 0 ? 1f : car.Location.x);
            // Climb while winding up if the shot will cross the goal mouth higher than we hang: the jump off the wall
            // is flat, so the car should start above the contact and fall into it.
            float shotHeight = 0f, crossingX = float.NaN;
            foreach (BallSlice slice in Ball.Prediction.Slices ?? Array.Empty<BallSlice>())
            {
                if (slice == null || slice.Time < Game.Time) continue;
                if (slice.Time > Game.Time + 2.2f) break;
                if (MathF.Abs(slice.Location.y) > Field.Length * 0.5f - 900f) { shotHeight = slice.Location.z; crossingX = slice.Location.x; break; }
            }
            float z = System.Math.Clamp(MathF.Max(car.Location.z, shotHeight + 150f), 300f, 1100f);
            // Run only as far as needed: toward where the shot crosses, stopping short on our side, never past the post.
            float x = sx * (Goal.Width * 0.5f + 60f);
            if (float.IsFinite(crossingX))
            {
                float stop = crossingX + sx * 300f;
                x = sx > 0 ? MathF.Max(x, stop) : MathF.Min(x, stop);
            }
            return new Vec3(x, car.Location.y, z);
        }

        /// <summary>
        /// Closed-form block reachability: the smallest constant thrust that puts the car on the ball's predicted path in
        /// front of our goal, from the given state. Returns the demand and the chosen ball point (earliest reachable).
        /// </summary>
        public static (float demand, BallSlice slice) BlockDemand(Vec3 position, Vec3 velocity, Vec3 ownGoal, float delay,
            float limit, Vec3 nose, float maxAngle = MaxBoostAngle, float horizon = 2.2f)
        {
            float side = ownGoal.y < 0 ? -1f : 1f;
            BallSlice best = null;
            float bestCost = float.PositiveInfinity, bestDemand = float.PositiveInfinity;
            BallSlice[] slices = Ball.Prediction.Slices;
            if (slices == null)
                return (bestDemand, null);
            float cosLimit = MathF.Cos(maxAngle);
            float next = Game.Time + delay + 0.1f;
            foreach (BallSlice slice in slices)
            {
                if (slice == null || slice.Time < next)
                    continue;
                float t = slice.Time - Game.Time;
                if (t > horizon)
                    break;
                next = slice.Time + 1f / 30f;
                float depth = slice.Location.y * side;
                if (depth > Field.Length * 0.5f + 40f)
                    break;                               // the ball is already in the net by then
                if (depth < Field.Length * 0.5f - BlockReach || slice.Location.z < 60f)
                    continue;
                Vec3 need = (BlockPoint(slice.Location, ownGoal) - position - velocity * t) * (2f / (t * t)) - Game.Gravity;
                float demand = need.Length();
                // Falling on the way is fine (as in the reference saves); what costs time is turning before boosting.
                float align = demand > 1f ? need.Dot(nose) / demand : 1f;
                if (demand < limit && (demand < 200f || align > cosLimit))
                    return (demand, slice);              // earliest reachable point with boost available right away
                float cost = demand + 900f * MathF.Max(0f, cosLimit - align);
                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestDemand = demand;
                    best = slice;
                }
            }
            return (float.PositiveInfinity, best);
        }

        /// <summary>
        /// Human launch rule: jump off the wall when a flat dash (launch impulse plus full boost along the current nose,
        /// under gravity, closed form) passes through the ball's predicted path before it reaches our goal line.
        /// Returns the smallest predicted miss distance and the time it happens.
        /// </summary>
        public static (float miss, float time) DashMiss(Car car, Vec3 ownGoal, float delay = 0.15f)
        {
            var (p0, v0) = AfterLaunch(car);
            Vec3 a = car.Forward * AerialPlanner.ThrustAccel + Game.Gravity;
            float side = ownGoal.y < 0 ? -1f : 1f;
            float bestMiss = float.PositiveInfinity, bestTime = float.NaN;
            BallSlice[] slices = Ball.Prediction.Slices;
            if (slices == null)
                return (bestMiss, bestTime);
            float next = Game.Time + delay + 0.1f;
            foreach (BallSlice slice in slices)
            {
                if (slice == null || slice.Time < next)
                    continue;
                float t = slice.Time - Game.Time - delay;
                if (t > 1.8f)
                    break;
                next = slice.Time + 1f / 60f;
                if (slice.Location.y * side > Field.Length * 0.5f + 40f)
                    break;
                Vec3 carAt = p0 + v0 * t + a * (0.5f * t * t);
                float miss = carAt.Dist(BlockPoint(slice.Location, ownGoal));
                if (miss < bestMiss)
                {
                    bestMiss = miss;
                    bestTime = t;
                }
            }
            return (bestMiss, bestTime);
        }

        /// <summary>Launch now if a block is reachable with boost available at once, or the straight dash meets the ball.</summary>
        public static bool LaunchReady(Car car, Vec3 ownGoal)
        {
            var (p, v) = AfterLaunch(car);
            return BlockDemand(p, v, ownGoal, 0.15f, MaxDemand, car.Forward, MaxBoostAngle, LaunchHorizon).demand < MaxDemand ||
                DashMiss(car, ownGoal).miss < LaunchMiss;
        }

        /// <summary>Where the car's body should be: just goal-side of the ball, so the contact pushes it upfield.</summary>
        public static Vec3 BlockPoint(Vec3 ball, Vec3 ownGoal)
        {
            float side = ownGoal.y < 0 ? -1f : 1f;
            return ball + new Vec3(0, side * GoalSideOffset, 0);
        }

        /// <summary>How far goal-side of the ball the car aims (ball radius + part of the car).</summary>
        public const float GoalSideOffset = 110f;

        /// <summary>Launch state after jump + double jump off the current surface (wall normal = car.Up).</summary>
        private static (Vec3 position, Vec3 velocity) AfterLaunch(Car car) =>
            (car.Location + car.Velocity * 0.15f + car.Up * 40f, car.Velocity + car.Up * LaunchImpulse);

        public static WallRelease TryCreate(Car car, Vec3 ownGoal, float deadline)
        {
            if (!OnWall(car) || car.Boost < 5f)
                return null;
            if (LaunchReady(car, ownGoal))
                return new WallRelease(car, ownGoal, true);
            if (OnBackWall(car, ownGoal) && MathF.Abs(car.Location.x) > Goal.Width * 0.5f + 150f && car.Boost > 10f)
                return new WallRelease(car, ownGoal, false);
            return null;
        }

        private void Enter(Phase next)
        {
            phase = next;
            phaseStart = Game.Time;
        }

        public void Run(RUBot bot)
        {
            Car car = bot.Me;
            var c = bot.Controller;
            float elapsed = Game.Time - phaseStart;
            if (Game.Time - started > 3f || car.IsDemolished)
            {
                Finished = true;
                return;
            }
            switch (phase)
            {
                case Phase.Windup:
                {
                    bool reachable = LaunchReady(car, ownGoal);
                    bool atPost = MathF.Abs(car.Location.x) < Goal.Width * 0.5f + 180f;
                    if (reachable || atPost)
                    {
                        Enter(Phase.Jump);
                        Run(bot);
                        return;
                    }
                    if (!OnWall(car))
                    {
                        Finished = true;
                        return;
                    }
                    windup.Target = WindupTarget(car);
                    windup.TargetSpeed = Car.MaxSpeed;
                    windup.WasteBoost = true;
                    windup.Run(bot);
                    return;
                }
                case Phase.Jump:
                    // Single held jump off the wall (the reference saves never double jump), boosting along the nose.
                    c.Jump = elapsed < 0.15f;
                    c.Pitch = c.Yaw = c.Roll = 0f;
                    c.Throttle = 1f;
                    c.Boost = car.Boost > 0f;
                    if (elapsed >= 0.15f)
                        Enter(Phase.Fly);
                    return;
                case Phase.Fly:
                {
                    var (demand, slice) = BlockDemand(car.Location, car.Velocity, ownGoal, 0f, MaxDemand, car.Forward, 1.2f);
                    if (slice == null || car.IsGrounded)
                    {
                        Finished = true;
                        return;
                    }
                    float t = MathF.Max(0.05f, slice.Time - Game.Time);
                    Vec3 need = (BlockPoint(slice.Location, ownGoal) - car.Location - car.Velocity * t) * (2f / (t * t)) - Game.Gravity;
                    Vec3 toBall = Ball.Location - car.Location;
                    // Close to the ball: dodge into it to block with the whole car and knock it away.
                    if (toBall.Length() < 230f && bot.Jump.CanDodge)
                    {
                        // Dodge upfield and away from the goal centre: the flip knocks the ball out of the mouth.
                        float side = ownGoal.y < 0 ? -1f : 1f;
                        Vec3 clear = new Vec3(MathF.Sign(Ball.Location.x == 0 ? 1f : Ball.Location.x) * 0.5f, -side, 0);
                        dodgeDirection = ControlMath.Unit(clear, car.Forward.Flatten());
                        Enter(Phase.Dodge);
                        Run(bot);
                        return;
                    }
                    Vec3 nose = ControlMath.Unit(need, car.Forward);
                    ControlMath.AimStiff(car, c, nose, car.Up);
                    c.Boost = car.Boost > 0f && car.Forward.Dot(nose) > 0.7f && need.Length() > 250f;
                    c.Throttle = 1f;
                    return;
                }
                case Phase.Dodge:
                {
                    Vec3 local = car.Local(dodgeDirection);
                    c.Jump = elapsed < 0.03f;
                    c.Pitch = -local.x;
                    c.Yaw = local.y;
                    c.Roll = 0f;
                    if (elapsed > 0.6f || car.IsGrounded)
                        Finished = true;
                    return;
                }
            }
        }
    }
}
