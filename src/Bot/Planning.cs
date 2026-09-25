using System;
using System.Collections.Generic;
using RedUtils;
using RedUtils.Math;

namespace Bot
{
    public sealed class TacticalFrame
    {
        public float MyEta, OpponentEta = 6f, TeammateEta = 6f;
        public float PressureTime = float.PositiveInfinity;
        public int FirstMan, TeamRank, TeamCount = 1;
        public bool LastBack, HasCover;
        public float FreeTime => OpponentEta - MyEta;
        public bool UnderPressure => float.IsFinite(PressureTime);
    }

    public static class Tactics
    {
        public static float GoalThreat(BallSlice[] slices, Vec3 goal, float now, float horizon = 2.5f) =>
            Defense.GoalThreat(slices, goal, now, horizon, out _);

        /// <summary>Fixed ETA buckets give all observers the same transitive race ownership order.</summary>
        public static bool WinsTie(float eta, int index, float otherEta, int otherIndex, float margin = 0.08f)
        {
            float bucketSize = float.IsFinite(margin) ? MathF.Max(margin, 0.001f) : 0.08f;
            long bucket = float.IsFinite(eta)
                ? (long)MathF.Floor(MathF.Max(0f, eta) / bucketSize)
                : long.MaxValue;
            long otherBucket = float.IsFinite(otherEta)
                ? (long)MathF.Floor(MathF.Max(0f, otherEta) / bucketSize)
                : long.MaxValue;
            return bucket != otherBucket ? bucket < otherBucket : index < otherIndex;
        }

        public static bool KickoffBefore(Car a, Car b, Vec3 ball, int team)
        {
            float da = a.Location.Dist(ball), db = b.Location.Dist(ball);
            if (MathF.Abs(da - db) > 20f)
                return da < db;

            float leftA = -Field.Side(team) * a.Location.x;
            float leftB = -Field.Side(team) * b.Location.x;
            if (MathF.Abs(leftA - leftB) > 1f)
                return leftA > leftB;
            return a.Index < b.Index;
        }

        /// <summary>
        /// Earliest plausible low-ball ground intercept. This is a race estimate, not a proof that a
        /// touch is strategically safe; the latter is handled separately by Defense.CanChallenge.
        /// </summary>
        public static float GroundEta(Car car)
        {
            if (car == null || car.IsDemolished || !ControlMath.Finite(car.Location) ||
                !ControlMath.Finite(car.Velocity))
                return 6f;

            if (car.Location.Dist(Ball.Location) < 180f &&
                (car.Velocity - Ball.Velocity).Length() < 600f)
                return 0.05f;

            float next = Game.Time + 0.1f;
            BallSlice[] slices = Ball.Prediction.Slices;
            if (slices != null)
            {
                foreach (BallSlice slice in slices)
                {
                    if (slice == null || !float.IsFinite(slice.Time) ||
                        !ControlMath.Finite(slice.Location) || slice.Time < next)
                        continue;

                    float t = slice.Time - Game.Time;
                    if (t > 3.5f)
                        break;

                    next = slice.Time + 0.15f;
                    if (slice.Location.z > 300f)
                        continue;

                    float eta = Drive.GetEta(car, slice.Location);
                    if (float.IsFinite(eta) && eta <= t)
                        return t;
                }
            }

            float fallback = Drive.GetEta(car, Ball.Location);
            return float.IsFinite(fallback)
                ? System.Math.Clamp(fallback, 0.05f, 6f)
                : 6f;
        }

        public static TacticalFrame Evaluate(RUBot bot)
        {
            var result = new TacticalFrame
            {
                MyEta = GroundEta(bot.Me),
                FirstMan = bot.Index,
                LastBack = true
            };

            float best = result.MyEta;
            int side = Field.Side(bot.Team);
            int myDepth = (int)MathF.Floor(bot.Me.Location.y * side / 100f);

            foreach (Car car in Cars.AllLivingCars)
            {
                if (car == null || car.Index == bot.Index)
                    continue;

                float eta = GroundEta(car);
                if (car.Team != bot.Team)
                {
                    result.OpponentEta = MathF.Min(result.OpponentEta, eta);
                    continue;
                }

                result.TeamCount++;
                result.TeammateEta = MathF.Min(result.TeammateEta, eta);
                result.HasCover |= Defense.CoversGoal(car, Ball.Location, bot.OurGoal.Location);

                // Stable depth buckets prevent cars at effectively identical depth from both deciding
                // that the other car is last back.
                int otherDepth = (int)MathF.Floor(car.Location.y * side / 100f);
                if (otherDepth > myDepth || (otherDepth == myDepth && car.Index < bot.Index))
                    result.LastBack = false;

                if (WinsTie(eta, car.Index, result.MyEta, bot.Index))
                    result.TeamRank++;

                if (WinsTie(eta, car.Index, best, result.FirstMan))
                {
                    best = eta;
                    result.FirstMan = car.Index;
                }
            }

            return result;
        }

        /// <summary>Compatibility overload retained for existing callers and tests.</summary>
        public static Vec3 ShadowTarget(Vec3 ball, Vec3 ownGoal, bool lastBack) =>
            Defense.ShadowTarget(ball, ownGoal,
                lastBack ? DefensiveRole.Anchor : DefensiveRole.Support);

        /// <summary>
        /// Short-horizon pre-contact threat estimate. RLBot's ball prediction cannot predict the
        /// next player touch, so controlled/coasting dribblers and committed approaches are modeled
        /// independently. The resulting time adjusts urgency; it never becomes the position target.
        /// </summary>
        public static float OpponentPressure(IEnumerable<Car> opponents, Ball ball, Vec3 ownGoal,
            float maxContactTime = 1.35f)
        {
            if (opponents == null || ball == null || !ControlMath.Finite(ball.location) ||
                !ControlMath.Finite(ball.velocity) || !ControlMath.Finite(ownGoal))
                return float.PositiveInfinity;

            float earliest = float.PositiveInfinity;
            Vec3 attackDirection = ControlMath.FlatUnit(ownGoal - ball.location,
                new Vec3(0, ownGoal.y < 0 ? -1 : 1, 0));

            foreach (Car opponent in opponents)
            {
                if (opponent == null || opponent.IsDemolished || !ControlMath.Finite(opponent.Location) ||
                    !ControlMath.Finite(opponent.Velocity) || !ControlMath.Finite(opponent.Forward))
                    continue;

                Vec3 toBall = ControlMath.FlatUnit(ball.location - opponent.Location, opponent.Forward);
                float distance = opponent.Location.Dist(ball.location);
                float attackAlignment = toBall.Dot(attackDirection);
                float facing = opponent.Forward.FlatNorm().Dot(toBall);
                float closing = (opponent.Velocity - ball.velocity).Dot(toBall);

                // A dribbler can threaten without throttle input. Close physical control and reasonable
                // orientation are enough to force the defender to respect a near-future touch.
                bool closeControl = distance < 360f &&
                    attackAlignment > -0.18f && facing > 0.05f && closing > -320f;
                if (closeControl)
                {
                    earliest = MathF.Min(earliest, 0.08f + MathF.Max(0f, distance - 180f) / 2300f);
                    continue;
                }

                if (attackAlignment < 0.30f)
                    continue;

                var input = opponent.LastInput ?? new RLBot.Flat.ControllerStateT();
                bool forwardIntent = input.Boost || input.Throttle > 0.18f || closing > 420f;
                if (!forwardIntent || (facing < 0.35f && closing < 520f))
                    continue;

                float eta = Drive.GetEta(opponent, ball.location);
                if (!opponent.IsGrounded && closing > 100f)
                    eta = MathF.Min(eta, distance / closing);

                if (float.IsFinite(eta) && eta >= 0f && eta <= maxContactTime)
                    earliest = MathF.Min(earliest, eta);
            }

            return earliest;
        }

        /// <summary>
        /// Short predicted contact waypoint used when a pressured first man has no valid scripted
        /// shot. Approach from the attacking side of the ball rather than dropping back to shadow.
        /// </summary>
        public static Vec3 PressureChallengeTarget(Car car, BallPrediction prediction, Ball ball,
            Vec3 attackGoal, float now, float eta)
        {
            if (car == null || ball == null || !ControlMath.Finite(ball.location) ||
                !ControlMath.Finite(attackGoal))
                return ball?.location ?? Vec3.Zero;

            float horizon = float.IsFinite(eta)
                ? System.Math.Clamp(eta * 0.45f, 0.08f, 0.32f)
                : 0.16f;
            Ball contact = prediction.TrySample(now + horizon, out Ball sample)
                ? sample
                : ball.Predict(horizon);

            Vec3 lane = ControlMath.FlatUnit(attackGoal - contact.location, car.Forward);
            Vec3 target = contact.location - lane * 70f;
            return Field.LimitToNearestSurface(target);
        }

        /// <summary>
        /// Geometric open-net check for a direct ball-to-goal lane. It intentionally ignores opponents
        /// behind the ball and expands the blocking corridor toward the goal mouth.
        /// </summary>
        public static bool GoalLaneOpen(IEnumerable<Car> opponents, Vec3 ball, Vec3 goal)
        {
            if (!ControlMath.Finite(ball) || !ControlMath.Finite(goal))
                return false;

            Vec3 axis = ControlMath.FlatUnit(goal - ball, new Vec3(0, goal.y < ball.y ? -1f : 1f, 0));
            float length = ball.FlatDist(goal);
            if (length < 1f)
                return true;

            if (opponents == null)
                return true;

            foreach (Car opponent in opponents)
            {
                if (opponent == null || opponent.IsDemolished ||
                    !ControlMath.Finite(opponent.Location) || opponent.Location.z > 520f)
                    continue;

                Vec3 rel = (opponent.Location - ball).Flatten();
                float along = rel.Dot(axis);
                if (along < 80f || along > length + 250f)
                    continue;

                float fraction = System.Math.Clamp(along / length, 0f, 1f);
                float halfWidth = 430f + fraction * (Goal.Width * 0.5f - 120f);
                Vec3 lateral = rel - axis * along;
                if (lateral.Length() <= halfWidth)
                    return false;
            }

            return true;
        }

        /// <summary>
        /// A high-value direct finish outranks keeping possession. Possession is still preferred for
        /// low-value/slow contacts, but an imminent goal-directed hit in the attacking half should
        /// not be converted into another catch or dribble setup.
        /// </summary>
        public static bool PreferImmediateShot(RUBot bot, Shot shot, TacticalFrame frame)
        {
            if (bot == null || shot == null || shot.Slice == null ||
                !float.IsFinite(shot.Slice.Time) || !ControlMath.Finite(shot.Slice.Location) ||
                !ControlMath.Finite(shot.ShotDirection))
                return false;

            float contactTime = shot.Slice.Time - Game.Time;
            if (!float.IsFinite(contactTime) || contactTime <= 0f || contactTime > 1.35f)
                return false;

            Vec3 attackGoal = bot.TheirGoal.Location;
            Vec3 goalAxis = ControlMath.FlatUnit(
                attackGoal - shot.Slice.Location, bot.Me.Forward);
            Vec3 shotDirection = ControlMath.FlatUnit(shot.ShotDirection, goalAxis);
            float goalward = shotDirection.Dot(goalAxis);
            if (goalward < 0.42f)
                return false;

            float side = Field.Side(bot.Team);
            bool offensiveHalf = shot.Slice.Location.y * side < -250f;
            float goalDistance = shot.Slice.Location.FlatDist(attackGoal);
            bool openLane = GoalLaneOpen(bot.LivingOpponents, shot.Slice.Location, attackGoal);

            bool immediateBoom = offensiveHalf && contactTime <= 0.72f && goalDistance < 5000f;
            bool closeFinish = goalDistance < 3200f && contactTime <= 1.00f;
            bool openNet = openLane && offensiveHalf && goalDistance < 5200f && contactTime <= 1.25f;
            bool pressuredRelease = frame?.UnderPressure == true &&
                offensiveHalf && contactTime <= 0.70f;

            return immediateBoom || closeFinish || openNet || pressuredRelease;
        }

        /// <summary>Compatibility wrapper: stationary defensive parking has zero terminal speed.</summary>
        public static float GuardSpeed(Car car, Vec3 target, float cruiseSpeed) =>
            Defense.DriveSpeed(car, target, cruiseSpeed, 0f);

        /// <summary>
        /// A genuinely deep-net car exits through the central mouth before receiving a field-side
        /// waypoint. A correctly placed shallow guard is not repeatedly ejected from the net.
        /// </summary>
        public static Vec3 GoalReturnTarget(Car car, Vec3 desiredGuard, Vec3 ownGoal)
        {
            if (car == null || !ControlMath.Finite(desiredGuard) || !ControlMath.Finite(ownGoal))
                return desiredGuard;

            float side = ownGoal.y < 0 ? -1 : 1;
            float line = MathF.Abs(ownGoal.y);
            bool behindLine = car.Location.y * side > line + 40f;
            bool insideMouth = MathF.Abs(car.Location.x - ownGoal.x) < Goal.Width * 0.5f + 220f;
            bool shallowGuard = desiredGuard.y * side >= line - 100f &&
                car.Location.y * side <= line + 260f;

            if (!behindLine || !insideMouth || shallowGuard)
                return desiredGuard;

            float safeHalfWidth = Goal.Width * 0.5f - 160f;
            float x = System.Math.Clamp(desiredGuard.x,
                ownGoal.x - safeHalfWidth, ownGoal.x + safeHalfWidth);
            return new Vec3(x, side * (line - 300f), 17);
        }

        /// <summary>
        /// An aerial is worth taking only if its modelled outcome helps: a clear must not send the ball
        /// toward our own goal, and an attacking touch must roughly head for the target.
        /// </summary>
        public static bool UsefulAerial(RUBot bot, AerialStrike aerial, bool emergency)
        {
            Vec3 outgoing = ControlMath.Unit(aerial.PredictedOutgoing, aerial.ShotDirection);
            if (emergency)
            {
                Vec3 towardOwnGoal = ControlMath.Unit(bot.OurGoal.Location - aerial.Slice.Location, Vec3.Up);
                return outgoing.Dot(towardOwnGoal) < 0.3f;
            }
            return aerial.AimError < 0.9f;
        }

        /// <summary>
        /// Bounded shot search with an optional hard contact deadline. Emergency defense must not
        /// select a nominally valid contact that occurs after the ball has already crossed the line.
        /// </summary>
        public static Shot SelectShot(RUBot bot, bool emergency, float opponentEta,
            Func<float, bool> claimed, float maxContactTime = 3f)
        {
            BallSlice[] slices = Ball.Prediction.Slices;
            if (slices == null || slices.Length == 0 ||
                !float.IsFinite(maxContactTime) || maxContactTime <= 0f)
                return null;

            Target target = new Target(emergency ? bot.OurGoal : bot.TheirGoal, emergency);
            float next = Game.Time + 0.08f;
            float bestScore = float.NegativeInfinity;
            int evaluated = 0;
            Shot best = null;

            foreach (BallSlice slice in slices)
            {
                if (slice == null || !float.IsFinite(slice.Time) ||
                    !ControlMath.Finite(slice.Location) || !ControlMath.Finite(slice.Velocity) ||
                    slice.Time < next)
                    continue;

                float t = slice.Time - Game.Time;
                if (t > MathF.Min(3f, maxContactTime) || evaluated >= 48)
                    break;

                next = slice.Time + 0.06f;
                evaluated++;

                if (emergency &&
                    !Defense.IsGoalSide(bot.Me.Location, slice.Location, bot.OurGoal.Location, -100f))
                    continue;
                if (!target.Fits(slice.Location) || (!emergency && claimed(slice.Time)))
                    continue;

                Ball after = slice.ToBall();
                Vec3 approach = (slice.Location - bot.Me.Location) / t;
                after.velocity = approach.Cap(0f, Car.MaxSpeed) + slice.Velocity * 0.25f;
                Vec3 destination = target.Clamp(after);
                if (!ControlMath.Finite(destination))
                    continue;

                bool strikes = bot is not Stardust stardust || stardust.Options.AerialStrikes;
                // Ground mechanics cannot be executed from the air; airborne cars only consider aerials.
                if (strikes && !bot.Me.IsGrounded)
                {
                    AerialStrike aerial = AerialStrike.TryCreate(bot.Me, slice, destination);
                    if (aerial == null || !UsefulAerial(bot, aerial, emergency))
                        continue;
                    float airScore = -t - aerial.AimError * 0.6f -
                        MathF.Max(0f, t - opponentEta) * (emergency ? 0f : 2f);
                    if (airScore > bestScore)
                    {
                        best = aerial;
                        bestScore = airScore;
                    }
                    break;
                }

                Shot candidate = new GroundShot(bot.Me, slice, destination);
                float cost = 0f;
                if (!candidate.IsValid(bot.Me))
                {
                    candidate = new JumpShot(bot.Me, slice, destination);
                    cost = 0.15f;
                }
                if (!candidate.IsValid(bot.Me))
                {
                    candidate = new DoubleJumpShot(bot.Me, slice, destination);
                    cost = 0.4f;
                }
                // From the floor the legacy aerial (turn, drive, then jump when its boost model allows)
                // measured at least as well as AerialStrike in paired scenarios, so it stays the ground choice.
                if (!candidate.IsValid(bot.Me))
                {
                    candidate = new AerialShot(bot.Me, slice, destination);
                    cost = 0.8f;
                }
                if (candidate == null || !candidate.IsValid(bot.Me))
                    continue;

                float score = -t - cost -
                    MathF.Max(0f, t - opponentEta) * (emergency ? 0f : 2f);
                if (score > bestScore)
                {
                    best = candidate;
                    bestScore = score;
                }

                if (best != null && t > best.Slice.Time - Game.Time + 0.3f)
                    break;
            }

            return best;
        }
    }
}
