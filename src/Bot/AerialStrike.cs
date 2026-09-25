using System;
using RedUtils;
using RedUtils.Math;

namespace Bot
{
    /// <summary>
    /// Aerial shot/clear built on <see cref="AerialContact"/>. From a surface it first drives toward the
    /// point under the contact at an arrival-matched speed, then launches as soon as the analytic launch
    /// model says the contact is reachable; from the air it flies immediately.
    /// </summary>
    public sealed class AerialStrike : Shot
    {
        public override bool Finished { get; protected set; }
        public override bool Interruptible { get; protected set; } = true;
        public override BallSlice Slice { get; protected set; }
        public override Vec3 ShotTarget { get; protected set; }
        public override Vec3 TargetLocation { get; protected set; }
        public override Vec3 ShotDirection { get; protected set; }
        public AerialPlan Plan { get; private set; }
        public bool Launched => contact != null;
        /// <summary>Modelled ball velocity after contact, from the fitted impulse model.</summary>
        public Vec3 PredictedOutgoing { get; private set; }
        /// <summary>Angle (radians) between the modelled outgoing ball direction and the target.</summary>
        public float AimError { get; private set; }

        private AerialContact contact;
        private AerialPlan pending;
        private Drive approach;
        private readonly bool dodge;

        private AerialStrike(BallSlice slice, Vec3 target, Vec3 push, bool dodge)
        {
            Slice = slice;
            ShotTarget = target;
            ShotDirection = push;
            TargetLocation = AerialPlanner.ContactPoint(slice.Location, push, ContactFace.Nose);
            this.dodge = dodge;
        }

        public const float Closing = 700f;
        /// <summary>Carry speed through the ball: firm clears and shots instead of floaty taps.</summary>
        public const float ThroughDistance = 350f;

        /// <summary>Plan for one specific slice; null if unreachable.</summary>
        public static AerialStrike TryCreate(Car car, BallSlice slice, Vec3 target, bool dodge = true)
        {
            if (car == null || slice == null || slice.Location.z < 220f)
                return null;
            float tau = slice.Time - Game.Time;
            if (tau <= 0.12f || tau > 4f)
                return null;

            // The normal depends on the arrival velocity, which depends on the contact point: two passes.
            Vec3 push = ControlMath.Unit(target - slice.Location, Vec3.Up);
            for (int pass = 0; pass < 2; pass++)
            {
                Vec3 arrival = AerialPlanner.ArrivalVelocity(car,
                    AerialPlanner.ContactPoint(slice.Location, push, ContactFace.Nose), tau);
                push = AerialPlanner.PushToward(slice.Location, slice.Velocity, arrival, target);
                // The car can only strike the side of the ball it is travelling toward.
                Vec3 approach = ControlMath.Unit(arrival - slice.Velocity, push);
                push = AerialPlanner.LimitAngle(push, approach, 0.7f);
            }
            Vec3 approachDirection = ControlMath.Unit(slice.Location - car.Location, push);
            if (approachDirection.Dot(push) < -0.3f)
                return null;
            Vec3 contactPoint = AerialPlanner.ContactPoint(slice.Location, push, ContactFace.Nose);
            if (!AerialPlanner.Clear(contactPoint, slice.Location))
                return null;

            // Low contacts from a surface are jump-shot territory; the aerial is for genuinely high balls.
            if (car.IsGrounded && (contactPoint - car.Location).Dot(car.Up) < 380f)
                return null;
            Vec3 outgoing = AerialPlanner.Outgoing(slice.Velocity,
                AerialPlanner.ArrivalVelocity(car, contactPoint, tau), push, push);
            var strike = new AerialStrike(slice, target, push, dodge)
            {
                Interruptible = car.IsGrounded,
                PredictedOutgoing = outgoing,
                AimError = MathF.Acos(System.Math.Clamp(ControlMath.Unit(outgoing, push)
                    .Dot(ControlMath.Unit(target - slice.Location, push)), -1f, 1f)),
            };
            strike.pending = strike.LaunchPlan(car);
            if (strike.pending != null)
                return strike;
            if (!car.IsGrounded || car.Up.z < 0.8f)
                return null;

            // Drive-then-launch feasibility (floor only): reach the launch point, then climb.
            float rise = AerialPlanner.RiseTime(contactPoint.z);
            if (tau < rise + 0.1f || car.Boost < 8f + rise * 26f)
                return null;
            Vec3 ground = new Vec3(contactPoint.x, contactPoint.y, 17f);
            float eta = Drive.GetEta(car, ground);
            if (!float.IsFinite(eta) || eta > tau - rise * 0.9f)
                return null;
            return strike;
        }

        /// <summary>A launch-now plan (surface) or fly-now plan (air), or null.</summary>
        private AerialPlan LaunchPlan(Car car)
        {
            float tau = Slice.Time - Game.Time;
            Vec3 point = TargetLocation;
            Vec3 velocity = Slice.Velocity + ShotDirection * Closing;
            float single = AerialPlanner.Cost(car, point, velocity, tau, ContactFace.Nose, false, false);
            float dbl = car.IsGrounded && !car.HasDoubleJumped
                ? AerialPlanner.Cost(car, point, velocity, tau, ContactFace.Nose, false, true) : -1f;
            if (single < 0 && dbl < 0)
                return null;
            bool useDouble = dbl >= 0 && (single < 0 || dbl < single - 1f);
            return new AerialPlan
            {
                ThroughDistance = ThroughDistance,
                Time = Slice.Time,
                BallLocation = Slice.Location,
                BallVelocity = Slice.Velocity,
                Push = ShotDirection,
                Face = ContactFace.Nose,
                Closing = Closing,
                DoubleJump = useDouble,
                BoostCost = useDouble ? dbl : single,
            };
        }

        public override bool IsValid(Car car) => !Finished && Slice != null && Slice.Time > Game.Time;

        public override void Run(RUBot bot)
        {
            Car car = bot.Me;
            if (contact == null)
            {
                float tau = Slice.Time - Game.Time;
                if (tau <= 0.1f || !IsPredictionValid(120f))
                {
                    Finished = true;
                    return;
                }
                // A plan that was feasible when selected is executed; closed-loop guidance absorbs the
                // one-tick drift. Re-evaluating would flip borderline plans on and off.
                AerialPlan plan = pending ?? LaunchPlan(car);
                pending = null;
                if (plan != null)
                {
                    Plan = plan;
                    contact = new AerialContact(car, plan) { DodgeAtContact = dodge };
                }
                else
                {
                    if (!car.IsGrounded)
                    {
                        Finished = true;
                        return;
                    }
                    Vec3 ground = Field.LimitToNearestSurface(TargetLocation);
                    float distance = car.Location.FlatDist(ground);
                    // Holding v = distance / τ means a launch `rise` seconds before contact arrives on
                    // time while the car keeps its ground speed through the climb. The launch check
                    // above fires as soon as the flight becomes feasible.
                    float rise = AerialPlanner.RiseTime(TargetLocation.z);
                    float speed = System.Math.Clamp(distance / MathF.Max(0.1f, tau) * 1.03f, 300f, Car.MaxSpeed);
                    approach ??= new Drive(car, ground, speed, allowDodges: false, wasteBoost: false);
                    approach.Target = ground;
                    approach.TargetSpeed = speed;
                    approach.Run(bot);
                    Interruptible = true;
                    if (tau < rise - 0.05f)
                        Finished = true;
                    return;
                }
            }

            contact.Run(bot);
            Interruptible = contact.Interruptible && !contact.Airborne;
            if (contact.Finished)
                Finished = true;
        }
    }
}
