using System;
using RedUtils;
using RedUtils.Math;

namespace Bot
{
    /// <summary>Which part of the car meets the ball.</summary>
    public enum ContactFace { Nose, Wheels, Roof }

    /// <summary>
    /// A planned airborne contact: meet the predicted ball at <see cref="Time"/> with <see cref="Face"/>,
    /// pushing it along <see cref="Push"/> with roughly <see cref="Closing"/> uu/s of relative approach.
    /// </summary>
    public sealed class AerialPlan
    {
        public float Time;
        public Vec3 BallLocation, BallVelocity, Push;
        public ContactFace Face;
        public float Closing;
        public bool DoubleJump;
        public float BoostCost;
        /// <summary>0 = reach the point only; 1 = also arrive with <see cref="ContactVelocity"/>.</summary>
        public float VelocityWeight;
        /// <summary>Reachable only after driving closer: approach on the floor, then launch when feasible.</summary>
        public bool Deferred;
        /// <summary>
        /// Guidance aims this far beyond the contact along the push, at the time the car would get there
        /// moving at <see cref="ThroughSpeed"/>. The car then carries speed through the ball (a firm touch)
        /// without the stiffness of a hard terminal-velocity constraint.
        /// </summary>
        public float ThroughDistance;
        public float ThroughSpeed = 1600f;
        /// <summary>Approach style: get under the contact as early as possible and wait (blocks), instead of
        /// holding the arrival-matched speed that carries momentum into the touch (shots).</summary>
        public bool ArriveEarly;

        public Vec3 ContactPoint => AerialPlanner.ContactPoint(BallLocation, Push, Face);
        public Vec3 ContactVelocity => BallVelocity + Push * Closing;
    }

    /// <summary>
    /// Analytic reachability for airborne contacts. The model is deliberately simple: launch impulses from
    /// the current surface, a rotation delay proportional to the square root of the required turn, then
    /// constant boost acceleration along one direction, then an unpowered coast when the contact face is
    /// not the nose (the car must rotate its wheels/roof onto the ball without thrusting through it).
    /// </summary>
    public static class AerialPlanner
    {
        public const float ThrustAccel = Car.BoostAccel + Car.AirThrottleAccel;

        /// <summary>Distance from car origin to ball centre at contact for each face (Octane-like hitbox).</summary>
        public static float FaceDistance(ContactFace face) => face switch
        {
            ContactFace.Nose => Ball.Radius + 52f,
            ContactFace.Roof => Ball.Radius + 40f,
            _ => Ball.Radius + 14f,
        };

        /// <summary>Seconds of unpowered flight reserved before contact to rotate the face onto the ball.</summary>
        public static float CoastTime(ContactFace face) => face switch
        {
            ContactFace.Nose => 0f,
            // Half a pitch rotation takes ~0.7 s from rest; the car pre-rolls during thrust, so less is needed.
            ContactFace.Wheels => 0.55f,
            _ => 0.42f,
        };

        public static Vec3 ContactPoint(Vec3 ball, Vec3 push, ContactFace face) =>
            ball - push * FaceDistance(face);

        /// <summary>World axis of a face (points from car centre toward the ball at contact).</summary>
        public static Vec3 FaceAxis(Car car, ContactFace face) => face switch
        {
            ContactFace.Nose => car.Forward,
            ContactFace.Roof => car.Up,
            _ => -car.Up,
        };

        /// <summary>Ballistic car state after <paramref name="time"/>, including a ground/wall launch.</summary>
        public static void Ballistic(Car car, float time, bool launch, bool doubleJump, out Vec3 position, out Vec3 velocity)
        {
            if (launch && car.IsGrounded)
            {
                position = doubleJump ? car.LocationAfterDoubleJump(time, 0) : car.LocationAfterJump(time, 0);
                velocity = doubleJump ? car.VelocityAfterDoubleJump(time, 0) : car.VelocityAfterJump(time, 0);
            }
            else
            {
                position = car.PredictLocation(time);
                velocity = car.PredictVelocity(time);
            }
        }

        /// <summary>Latest sensible contact time for a launch from the current surface.</summary>
        public static float LaunchHorizon(Car car, Vec3 target)
        {
            float height = MathF.Max(0f, (target - car.Location).Dot(car.Up));
            return RiseTime(height) + 0.5f;
        }

        /// <summary>Seconds needed to climb <paramref name="height"/> with a double jump and full boost.</summary>
        public static float RiseTime(float height)
        {
            // z(t) ≈ 2·JumpVel·t + ½(thrust − gravity)t², after ~0.15 s of jump/nose-up.
            const float v0 = 2 * Car.JumpVel, a = ThrustAccel - 650f;
            float h = MathF.Max(0f, height - 60f);
            return 0.15f + (-v0 + MathF.Sqrt(v0 * v0 + 2 * a * h)) / a;
        }

        /// <summary>Estimated turn time to point the nose along <paramref name="direction"/>.</summary>
        public static float TurnTime(Car car, Vec3 direction)
        {
            float angle = MathF.Acos(System.Math.Clamp(car.Forward.Dot(ControlMath.Unit(direction, car.Forward)), -1f, 1f));
            return 0.33f * MathF.Sqrt(MathF.Max(angle, 0f));
        }

        /// <summary>
        /// Boost needed to reach <paramref name="target"/> with velocity near <paramref name="targetVelocity"/>
        /// after <paramref name="tau"/> seconds, or -1 when infeasible.
        /// </summary>
        public static float Cost(Car car, Vec3 target, Vec3 targetVelocity, float tau, ContactFace face,
            bool matchVelocity, bool doubleJump, float margin = 0.88f)
        {
            if (!float.IsFinite(tau) || tau <= 0.05f || !ControlMath.Finite(target))
                return -1;

            // From a surface, launching long before contact wastes the jump and invites landing early;
            // the caller keeps driving and replans instead.
            if (car.IsGrounded && tau > LaunchHorizon(car, target))
                return -1;

            float coast = MathF.Min(CoastTime(face), tau * 0.6f);
            float powered = tau - coast;
            Vec3 g = Game.Gravity;
            // Where must the car be when thrust stops so that it coasts into the contact point? A
            // velocity-matched contact coasts at the contact velocity; otherwise at the average flight velocity.
            Vec3 coastVelocity = matchVelocity ? targetVelocity : (target - car.Location) / tau;
            Vec3 coastStart = target - coastVelocity * coast + g * (0.5f * coast * coast);
            Ballistic(car, powered, true, doubleJump, out Vec3 endPosition, out Vec3 endVelocity);
            Vec3 error = coastStart - endPosition;
            float distance = error.Length();
            // From a surface the nose pitches toward the target during the jump itself, so the turn
            // overlaps the launch instead of adding to it.
            float turn = TurnTime(car, error);
            float available = powered - (car.IsGrounded ? MathF.Max(turn, 0.2f) : turn);
            if (available <= 0.02f)
                return distance < 40f ? 0f : -1f;

            float accel = (car.Boost > 1f ? ThrustAccel : Car.AirThrottleAccel) * margin;
            float maxDistance = 0.5f * accel * available * available;
            if (distance > maxDistance)
                return -1;

            // Burn duration for a bang-coast profile covering the error distance.
            float burn = available - MathF.Sqrt(MathF.Max(0f, available * available - 2f * distance / accel));
            float fuel = burn * Car.BoostConsumption;
            if (fuel > MathF.Max(0f, car.Boost - 2f))
                return -1;

            Vec3 finalVelocity = endVelocity + ControlMath.Unit(error, Vec3.Up) * (accel / margin) * burn +
                g * coast;
            if (finalVelocity.Length() > Car.MaxSpeed + 50f)
                return -1;
            if (matchVelocity && (finalVelocity - targetVelocity).Length() > 650f)
                return -1;
            return fuel;
        }

        /// <summary>
        /// Drive-then-launch reachability from the floor: reach the point under the contact early enough to
        /// climb to it. The exact launch moment is decided later by <see cref="Cost"/> each tick.
        /// </summary>
        public static bool ApproachFeasible(Car car, Vec3 contact, float tau)
        {
            if (car == null || !car.IsGrounded || car.Up.z < 0.8f)
                return false;
            float rise = RiseTime(contact.z);
            if (tau < rise + 0.1f || car.Boost < 8f + rise * 26f)
                return false;
            float eta = Drive.GetEta(car, new Vec3(contact.x, contact.y, 17f));
            return float.IsFinite(eta) && eta <= tau - rise * 0.9f;
        }

        /// <summary>Contact geometry is only sensible inside the arena and away from surfaces.</summary>
        public static bool Clear(Vec3 contact, Vec3 ball)
        {
            if (!ControlMath.Finite(contact) || contact.z < 40f || contact.z > Field.Height - 40f)
                return false;
            if (MathF.Abs(contact.x) > Field.Width * 0.5f - 170f)
                return false;
            if (MathF.Abs(contact.y) > Field.Length * 0.5f - 170f && MathF.Abs(contact.x) > Goal.Width * 0.5f - 100f)
                return false;
            return ball.z > 120f;
        }

        /// <summary>
        /// Earliest feasible contact in the prediction. <paramref name="choosePush"/> maps a slice to a push
        /// direction (or returns false to reject the slice).
        /// </summary>
        public static AerialPlan Find(Car car, float minTime, float maxTime, ContactFace face, float closing,
            bool matchVelocity, Func<BallSlice, (bool ok, Vec3 push)> choosePush, int budget = 60,
            bool allowApproach = false)
        {
            BallSlice[] slices = Ball.Prediction.Slices;
            if (car == null || slices == null || slices.Length == 0)
                return null;

            float next = Game.Time + MathF.Max(0.1f, minTime);
            int evaluated = 0;
            foreach (BallSlice slice in slices)
            {
                if (slice == null || slice.Time < next)
                    continue;
                float tau = slice.Time - Game.Time;
                if (tau > maxTime || evaluated >= budget)
                    break;
                next = slice.Time + (tau < 1.5f ? 1f / 30f : 1f / 15f);
                evaluated++;
                if (MathF.Abs(slice.Location.y) > Field.Length * 0.5f + 60f)
                    break;

                (bool ok, Vec3 push) = choosePush(slice);
                if (!ok || !ControlMath.Finite(push))
                    continue;
                push = ControlMath.Unit(push, Vec3.Up);
                Vec3 contact = ContactPoint(slice.Location, push, face);
                if (!Clear(contact, slice.Location))
                    continue;

                Vec3 contactVelocity = slice.Velocity + push * closing;
                float single = Cost(car, contact, contactVelocity, tau, face, matchVelocity, false);
                float dbl = car.IsGrounded && !car.HasDoubleJumped
                    ? Cost(car, contact, contactVelocity, tau, face, matchVelocity, true) : -1;
                bool deferred = false;
                if (single < 0 && dbl < 0)
                {
                    if (!allowApproach || !ApproachFeasible(car, contact, tau))
                        continue;
                    deferred = true;
                }

                bool useDouble = dbl >= 0 && (single < 0 || dbl < single - 1f);
                return new AerialPlan
                {
                    Time = slice.Time,
                    BallLocation = slice.Location,
                    BallVelocity = slice.Velocity,
                    Push = push,
                    Face = face,
                    Closing = closing,
                    DoubleJump = useDouble,
                    BoostCost = useDouble ? dbl : single,
                    VelocityWeight = matchVelocity ? 1f : 0f,
                    Deferred = deferred,
                };
            }
            return null;
        }

        /// <summary>
        /// Ball velocity after a nose contact with normal <paramref name="normal"/> (car centre → ball centre).
        /// Fitted to simulated contacts: a restitution term along the normal plus the game's extra hit impulse,
        /// whose direction has its vertical component scaled by 0.35 and is reduced along the car's nose.
        /// </summary>
        public static Vec3 Outgoing(Vec3 ballVelocity, Vec3 carVelocity, Vec3 normal, Vec3 carForward)
        {
            Vec3 relative = carVelocity - ballVelocity;
            Vec3 extra = new Vec3(normal.x, normal.y, normal.z * 0.35f);
            extra -= carForward * (0.35f * extra.Dot(carForward));
            extra = ControlMath.Unit(extra, normal);
            return ballVelocity + normal * (0.77f * MathF.Max(0f, relative.Dot(normal))) +
                extra * (0.55f * relative.Length());
        }

        /// <summary>
        /// Contact normal that sends the ball toward <paramref name="aim"/>, found by inverting
        /// <see cref="Outgoing"/> with a few fixed-point corrections. The aim point is raised to cancel
        /// gravity drop over the flight time of the shot.
        /// </summary>
        public static Vec3 PushToward(Vec3 ballLocation, Vec3 ballVelocity, Vec3 carVelocity, Vec3 aim)
        {
            Vec3 toAim = aim - ballLocation;
            float flight = toAim.Length() / 2200f;
            Vec3 desired = ControlMath.Unit(toAim + new Vec3(0, 0, 325f * flight * flight), Vec3.Up);
            Vec3 normal = ControlMath.Unit(desired * 1600f - ballVelocity, desired);
            for (int i = 0; i < 6; i++)
            {
                Vec3 outgoing = ControlMath.Unit(Outgoing(ballVelocity, carVelocity, normal, normal), desired);
                normal = ControlMath.Unit(normal + (desired - outgoing) * 1.2f, desired);
            }
            return normal;
        }

        /// <summary>Rotate <paramref name="direction"/> toward <paramref name="axis"/> until within <paramref name="maxAngle"/>.</summary>
        public static Vec3 LimitAngle(Vec3 direction, Vec3 axis, float maxAngle)
        {
            direction = ControlMath.Unit(direction, axis);
            axis = ControlMath.Unit(axis, direction);
            float cos = System.Math.Clamp(direction.Dot(axis), -1f, 1f);
            float angle = MathF.Acos(cos);
            if (angle <= maxAngle)
                return direction;
            Vec3 perpendicular = direction - axis * cos;
            if (perpendicular.Length() < 1e-4f)
                perpendicular = MathF.Abs(axis.z) < 0.9f ? Vec3.Up - axis * axis.z : new Vec3(1, 0, 0);
            perpendicular = ControlMath.Unit(perpendicular, Vec3.Up);
            return ControlMath.Unit(axis * MathF.Cos(maxAngle) + perpendicular * MathF.Sin(maxAngle), axis);
        }

        /// <summary>Rough car velocity at contact for a flight of <paramref name="tau"/> seconds.</summary>
        public static Vec3 ArrivalVelocity(Car car, Vec3 contact, float tau)
        {
            Vec3 average = (contact - car.Location) / MathF.Max(tau, 0.1f);
            Vec3 estimate = average * 1.15f;
            float speed = estimate.Length();
            return speed > Car.MaxSpeed ? estimate * (Car.MaxSpeed / speed) : estimate;
        }
    }

    /// <summary>
    /// Executes an <see cref="AerialPlan"/>: surface launch (single, double or wall jump), minimum-effort
    /// optimal guidance with gravity compensation toward the coast-start state, then a face-on coast into
    /// contact. The target is re-sampled from the live prediction every tick, so ordinary prediction drift
    /// is corrected rather than executed open-loop.
    /// </summary>
    public sealed class AerialContact : IAction
    {
        public AerialPlan Plan { get; }
        public bool Finished { get; private set; }
        public bool Interruptible => !launching && (approaching || !Airborne);
        public bool Airborne { get; private set; }
        /// <summary>Optional dodge into the ball at contact (nose contacts only).</summary>
        public bool DodgeAtContact { get; set; }
        /// <summary>World direction used to orient the car's secondary axis (nose for wheel/roof contacts).</summary>
        public Vec3 Heading { get; set; }
        /// <summary>Maximum tolerated drift between the planned and live predicted ball.</summary>
        public float DriftLimit { get; set; } = 180f;
        public float TimeRemaining => Plan.Time - Game.Time;

        private readonly ImpulseBoostGate boost = new();
        private readonly bool launchFromSurface;
        private readonly Vec3 launchUp;
        private float launchStart = float.NaN;
        private bool launching, released, secondJump, dodged;
        private float dodgeAt = float.NaN;

        public AerialContact(Car car, AerialPlan plan)
        {
            Plan = plan;
            launchFromSurface = car.IsGrounded;
            launchUp = car.Up;
            approaching = plan.Deferred && car.IsGrounded;
            launching = launchFromSurface && !approaching;
            Airborne = !car.IsGrounded;
            Heading = ControlMath.FlatUnit(plan.Push, car.Forward);
        }

        /// <summary>True while still driving toward the launch point.</summary>
        public bool Approaching => approaching;
        private bool approaching;
        private Drive approach;

        /// <summary>
        /// Floor approach: hold v = distance / τ toward the point under the contact (a launch `rise` seconds
        /// before contact then arrives on time), and launch as soon as the analytic model says it works.
        /// </summary>
        private void Approach(RUBot bot, Car car, Vec3 contact, Vec3 contactVelocity, float tau)
        {
            float single = AerialPlanner.Cost(car, contact, contactVelocity, tau, Plan.Face, Plan.VelocityWeight > 0f, false);
            float dbl = !car.HasDoubleJumped
                ? AerialPlanner.Cost(car, contact, contactVelocity, tau, Plan.Face, Plan.VelocityWeight > 0f, true) : -1f;
            if (single >= 0 || dbl >= 0)
            {
                Plan.DoubleJump = dbl >= 0 && (single < 0 || dbl < single - 1f);
                Plan.BoostCost = Plan.DoubleJump ? dbl : single;
                approaching = false;
                launching = true;
                Launch(bot, car, contact);
                return;
            }
            float rise = AerialPlanner.RiseTime(contact.z);
            if (!car.IsGrounded || tau < rise - 0.05f)
            {
                Finished = true;
                return;
            }
            Vec3 ground = new Vec3(contact.x, contact.y, 17f);
            float distance = car.Location.FlatDist(ground);
            float speed = Plan.ArriveEarly
                ? MathF.Min(Car.MaxSpeed, MathF.Sqrt(2f * 2800f * distance))
                : System.Math.Clamp(distance / MathF.Max(0.1f, tau) * 1.03f, 300f, Car.MaxSpeed);
            approach ??= new Drive(car, ground, speed, allowDodges: false, wasteBoost: false);
            approach.Target = ground;
            approach.TargetSpeed = speed;
            approach.Run(bot);
        }

        public void Run(RUBot bot)
        {
            Car car = bot.Me;
            float tau = Plan.Time - Game.Time;
            if (!float.IsFinite(tau) || tau < -0.25f)
            {
                Finished = true;
                return;
            }

            Vec3 ballAtContact = Plan.BallLocation;
            Vec3 ballVelocity = Plan.BallVelocity;
            if (tau > 0.02f && Ball.Prediction.TrySample(Plan.Time, out Ball live))
            {
                if ((live.location - Plan.BallLocation).Length() > DriftLimit && !dodged)
                {
                    Finished = true;
                    return;
                }
                ballAtContact = live.location;
                ballVelocity = live.velocity;
            }

            Vec3 contact = AerialPlanner.ContactPoint(ballAtContact, Plan.Push, Plan.Face);
            Vec3 contactVelocity = ballVelocity + Plan.Push * Plan.Closing;

            if (approaching)
            {
                Approach(bot, car, contact, contactVelocity, tau);
                return;
            }

            if (launching)
            {
                Launch(bot, car, contact);
                return;
            }

            if (car.IsGrounded && Game.Time - launchStart > 0.3f)
            {
                Finished = true;
                return;
            }
            Airborne = !car.IsGrounded;

            // After contact, a short follow-through keeps the requested orientation, then hand back.
            if (tau <= 0f)
            {
                if (Plan.Face == ContactFace.Nose && DodgeAtContact && !dodged && bot.Jump.CanDodge)
                    Dodge(bot, car, ballAtContact);
                else if (float.IsFinite(dodgeAt))
                    ContinueDodge(bot, car);
                else
                    FaceOn(bot, car, ballAtContact);
                if (tau < -0.12f)
                    Finished = true;
                return;
            }

            if (float.IsFinite(dodgeAt))
            {
                ContinueDodge(bot, car);
                return;
            }

            float coast = MathF.Min(AerialPlanner.CoastTime(Plan.Face), tau);
            float powered = tau - coast;
            Vec3 g = Game.Gravity;
            Vec3 demand;
            if (powered > 0.03f)
            {
                Vec3 arrival = Plan.VelocityWeight > 0f ? contactVelocity : (contact - car.Location) / tau;
                Vec3 coastStart = contact - arrival * coast + g * (0.5f * coast * coast);
                Vec3 coastVelocity = arrival - g * coast;
                demand = Guidance(car.Location, car.Velocity, coastStart, coastVelocity, powered, g,
                    Plan.VelocityWeight);
            }
            else
            {
                // Coast: only a small position trim is possible (air throttle / aligned boost).
                demand = AerialPhysics.RequiredAcceleration(car.Location, car.Velocity, contact, MathF.Max(tau, 0.05f), g);
            }
            if (Plan.ThroughDistance > 0f && Plan.Face == ContactFace.Nose && powered > 0.03f)
            {
                float extra = Plan.ThroughDistance / MathF.Max(400f, Plan.ThroughSpeed);
                Vec3 through = contact + Plan.Push * Plan.ThroughDistance - g * (0.5f * extra * extra);
                demand = Guidance(car.Location, car.Velocity, through, contactVelocity, powered + extra, g, 0f);
            }

            float demandLength = demand.Length();
            Vec3 direction = ControlMath.Unit(demand, car.Forward);
            bool facing = powered <= 0.03f || (Plan.Face != ContactFace.Nose && powered < 0.12f) ||
                (Plan.Face == ContactFace.Nose && tau < 0.22f);
            if (facing || demandLength < 120f)
                FaceOn(bot, car, ballAtContact);
            else
            {
                // Roll is free about the thrust axis: pre-roll the contact face toward the ball so the
                // coast only has to finish the rotation.
                Vec3 toBall = ControlMath.Unit(ballAtContact - car.Location, Vec3.Up);
                ControlMath.Aim(car, bot.Controller, direction, Plan.Face == ContactFace.Wheels ? -toBall : toBall);
            }

            float forwardDemand = MathF.Max(0f, demand.Dot(car.Forward));
            bool gentle = facing && Plan.Face != ContactFace.Nose;
            bot.Controller.Boost = boost.Step(Game.Time, forwardDemand, car.Forward.Dot(direction), car.Boost, gentle);
            bot.Controller.Throttle = ControlRuntime.Axis(demand.Dot(car.Forward) / Car.AirThrottleAccel);
            bot.Controller.Jump = false;

            if (Plan.Face == ContactFace.Nose && DodgeAtContact && !dodged && tau < 0.09f && bot.Jump.CanDodge &&
                car.Location.Dist(ballAtContact) < AerialPlanner.FaceDistance(ContactFace.Nose) + 90f)
                Dodge(bot, car, ballAtContact);
        }

        /// <summary>
        /// Minimum-effort guidance to a terminal state (or position only). u = [6(P'-x) - 2τ(2v + V')]/τ²
        /// in the gravity-free frame, where P' and V' remove gravity's contribution over the horizon.
        /// </summary>
        public static Vec3 Guidance(Vec3 x, Vec3 v, Vec3 target, Vec3 targetVelocity, float tau, Vec3 g, bool matchVelocity) =>
            Guidance(x, v, target, targetVelocity, tau, g, matchVelocity ? 1f : 0f);

        /// <summary>Blend of position-only (weight 0) and terminal-velocity (weight 1) guidance.</summary>
        public static Vec3 Guidance(Vec3 x, Vec3 v, Vec3 target, Vec3 targetVelocity, float tau, Vec3 g, float velocityWeight)
        {
            tau = MathF.Max(tau, 0.06f);
            Vec3 p = target - g * (0.5f * tau * tau);
            Vec3 positionOnly = (p - x - v * tau) * (2f / (tau * tau));
            if (velocityWeight <= 0f)
                return positionOnly;
            Vec3 vf = targetVelocity - g * tau;
            Vec3 terminal = ((p - x) * 6f - (v * 2f + vf) * (2f * tau)) / (tau * tau);
            return positionOnly + (terminal - positionOnly) * System.Math.Clamp(velocityWeight, 0f, 1f);
        }

        private void Launch(RUBot bot, Car car, Vec3 contact)
        {
            if (!float.IsFinite(launchStart))
                launchStart = Game.Time;
            float elapsed = Game.Time - launchStart;
            Vec3 toward = ControlMath.Unit(contact - car.Location, launchUp);
            // During the jump, pitch the nose toward the contact (fast aerial) and boost once aligned.
            ControlMath.Aim(car, bot.Controller, toward, ControlMath.Unit(launchUp, Vec3.Up));
            bot.Controller.Boost = elapsed > 0.05f && car.Forward.Dot(toward) > 0.6f && car.Boost > 0 &&
                Plan.BoostCost > 0.5f;
            bot.Controller.Throttle = 1f;
            const float hold = Car.JumpMaxDuration;
            if (elapsed < hold)
            {
                bot.Controller.Jump = true;
                return;
            }
            if (!Plan.DoubleJump)
            {
                bot.Controller.Jump = false;
                launching = false;
                return;
            }
            if (!released)
            {
                bot.Controller.Jump = false;
                released = true;
                return;
            }
            if (!secondJump)
            {
                // Neutral stick for the second press so it is a double jump, not a dodge.
                bot.Controller.Jump = true;
                bot.Controller.Pitch = bot.Controller.Yaw = bot.Controller.Roll = 0f;
                secondJump = true;
                return;
            }
            bot.Controller.Jump = false;
            launching = false;
        }

        /// <summary>Orient the contact face onto the ball, nose along <see cref="Heading"/>.</summary>
        private void FaceOn(RUBot bot, Car car, Vec3 ball)
        {
            Vec3 toBall = ControlMath.Unit(ball - car.Location, Plan.Push);
            switch (Plan.Face)
            {
                case ContactFace.Nose:
                    ControlMath.Aim(car, bot.Controller, toBall, ControlMath.Unit(car.Up, Vec3.Up));
                    break;
                case ContactFace.Roof:
                    ControlMath.Aim(car, bot.Controller, OrthogonalHeading(toBall), toBall);
                    break;
                default:
                    ControlMath.Aim(car, bot.Controller, OrthogonalHeading(toBall), -toBall);
                    break;
            }
        }

        private Vec3 OrthogonalHeading(Vec3 axis)
        {
            Vec3 h = Heading - axis * Heading.Dot(axis);
            if (h.Length() < 0.2f)
            {
                h = Vec3.Up - axis * axis.z;
                if (h.Length() < 0.2f) h = new Vec3(1, 0, 0);
            }
            return ControlMath.Unit(h, Vec3.Up);
        }

        private void Dodge(RUBot bot, Car car, Vec3 ball)
        {
            dodged = true;
            dodgeAt = Game.Time;
            Vec3 local = car.Local(ControlMath.Unit(ball - car.Location, car.Forward));
            float planar = MathF.Sqrt(local.x * local.x + local.y * local.y);
            dodgePitch = planar > 0.05f ? -local.x / planar : -1f;
            dodgeYaw = planar > 0.05f ? local.y / planar : 0f;
            ContinueDodge(bot, car);
        }

        private float dodgePitch = -1f, dodgeYaw;

        private void ContinueDodge(RUBot bot, Car car)
        {
            bool pulse = Game.Time - dodgeAt < 0.05f;
            bot.Controller.Jump = pulse;
            bot.Controller.Pitch = dodgePitch;
            bot.Controller.Yaw = dodgeYaw;
            bot.Controller.Roll = 0f;
            bot.Controller.Boost = false;
            if (Game.Time - dodgeAt > 0.35f)
                Finished = true;
        }
    }
}
