using System;
using RedUtils;
using RedUtils.Math;

namespace Bot
{
    /// <summary>Independent feature switches support in-game ablation against the baseline.</summary>
    public sealed class StardustOptions
    {
        public bool GroundControl { get; init; } = Environment.GetEnvironmentVariable("STARDUST_GROUND_CONTROL") != "0";
        public bool AerialCarry { get; init; } = Environment.GetEnvironmentVariable("STARDUST_AERIAL_CARRY") != "0";
        public bool FlipResets { get; init; } = Environment.GetEnvironmentVariable("STARDUST_FLIP_RESETS") != "0";
        public bool AerialStrikes { get; init; } = Environment.GetEnvironmentVariable("STARDUST_AERIAL_STRIKE") != "0";
        public bool AerialBlocks { get; init; } = Environment.GetEnvironmentVariable("STARDUST_AERIAL_BLOCK") == "1";
        public bool Demolitions { get; init; } = Environment.GetEnvironmentVariable("STARDUST_DEMOS") != "0";
        public bool WallGuard { get; init; } = Environment.GetEnvironmentVariable("STARDUST_SPIDERMAN") != "0";
        public bool Trace { get; init; } = Environment.GetEnvironmentVariable("STARDUST_TRACE") == "1";
        public string TelemetrySetting { get; init; } = Environment.GetEnvironmentVariable("STARDUST_TELEMETRY");
        public bool TelemetryConsole { get; init; } = Environment.GetEnvironmentVariable("STARDUST_TELEMETRY_CONSOLE") == "1";
        public string TelemetryFile { get; init; } = Environment.GetEnvironmentVariable("STARDUST_TELEMETRY_FILE");
        public int TelemetryHz { get; init; } = ReadTelemetryHz();

        public bool TelemetryDisabled => TelemetrySetting == "0";
        public bool TelemetryExplicit => TelemetrySetting != null;

        private static int ReadTelemetryHz()
        {
            string raw = Environment.GetEnvironmentVariable("STARDUST_TELEMETRY_HZ");
            return int.TryParse(raw, out int hz) ? System.Math.Clamp(hz, 1, 30) : 10;
        }
    }

    /// <summary>Threat-first planning with explicit goal-side recovery and moving shadow defense.</summary>
    public class Stardust : RUBot
    {
        public StardustOptions Options { get; } = new();
        public TacticalFrame Situation { get; private set; } = new();
        public string Decision { get; private set; } = "startup";
        public bool Shooting { get; set; }

        private float nextPlan = float.NegativeInfinity;
        private float challengeCommitUntil = float.NegativeInfinity;
        private bool defending, countering, pressured;
        private Shot defensiveShot;
        private readonly StardustTelemetry telemetry;

        public bool RawCanChallenge { get; private set; }
        public bool ChallengeCommitted { get; private set; }
        public float EmergencyThreatTime { get; private set; } = float.PositiveInfinity;
        public float CounterThreatTime { get; private set; } = float.PositiveInfinity;

        public Stardust(string defaultAgentId = null) : base(defaultAgentId)
        {
            // The packaged RLBot process enables file telemetry by default. Test/probe instances pass
            // an explicit agent id and remain quiet unless STARDUST_TELEMETRY was explicitly provided.
            bool productionEntry = defaultAgentId == null;
            bool telemetryEnabled = !Options.TelemetryDisabled &&
                (productionEntry || Options.TelemetryExplicit);
            telemetry = new StardustTelemetry(
                telemetryEnabled, Options.TelemetryHz, Options.TelemetryConsole, Options.TelemetryFile);
        }

        public override void Run()
        {
            if (ClockReset)
            {
                nextPlan = float.NegativeInfinity;
                challengeCommitUntil = float.NegativeInfinity;
                defending = false;
                countering = false;
                pressured = false;
                defensiveShot = null;
                RawCanChallenge = false;
                ChallengeCommitted = false;
                EmergencyThreatTime = float.PositiveInfinity;
                CounterThreatTime = float.PositiveInfinity;
                telemetry.Reset();
            }

            Shooting = Action is Shot;

            if (IsKickoff)
            {
                if (Action != null)
                    return;

                int rank = 0;
                foreach (Car car in LivingTeammates)
                    if (Tactics.KickoffBefore(car, Me, Ball.Location, Team))
                        rank++;

                if (rank == 0)
                {
                    Action = new Kickoff();
                    SetDecision("kickoff / taker");
                }
                else
                {
                    Vec3 target = rank == 1
                        ? new Vec3(0, Field.Side(Team) * 1200, 17)
                        : new Vec3(-MathF.Sign(Me.Location.x) * 750, Field.Side(Team) * 3500, 17);
                    DriveTo(target, rank == 1 ? 1300f : 1600f, false);
                    SetDecision(rank == 1 ? "kickoff / cheat" : "kickoff / cover");
                }
                return;
            }

            float threat = Defense.GoalThreat(Ball.Prediction.Slices, OurGoal.Location,
                Game.Time, 2.5f, out Vec3 crossing);
            float counterThreat = threat;
            if (!float.IsFinite(counterThreat))
                counterThreat = Defense.GoalThreat(Ball.Prediction.Slices, OurGoal.Location,
                    Game.Time, 4.5f, out _);

            EmergencyThreatTime = threat;
            CounterThreatTime = counterThreat;

            float pressureTime = Tactics.OpponentPressure(LivingOpponents,
                Ball.MainBall, OurGoal.Location);
            bool emergency = float.IsFinite(threat);
            bool counterDanger = !emergency && float.IsFinite(counterThreat);
            bool underPressure = float.IsFinite(pressureTime);
            bool threatEdge = emergency != defending || counterDanger != countering;
            bool pressureEdge = underPressure != pressured;

            if (threatEdge || pressureEdge)
                nextPlan = float.NegativeInfinity;

            // Do not acknowledge a tactical edge until a physically committed flip/dodge can be
            // interrupted. Otherwise the event is consumed while the old action keeps running.
            if (Action != null && !Action.Interruptible)
                return;

            defending = emergency;
            countering = counterDanger;
            pressured = underPressure;

            if (Action is Shot oldShot && !oldShot.IsPredictionValid())
                Action = null;
            if (threatEdge)
            {
                Action = null;
                defensiveShot = null;
            }
            if (Action == null)
                nextPlan = MathF.Min(nextPlan, Game.Time);
            if (Game.Time < nextPlan)
                return;

            Situation = Tactics.Evaluate(this);
            Situation.PressureTime = pressureTime;
            nextPlan = Game.Time + (underPressure || emergency || counterDanger ? 0.05f : 0.12f);

            RawCanChallenge = Defense.CanChallenge(
                Situation, Me, Ball.Location, OurGoal.Location);
            bool challengeSafe = Defense.CanContinueChallenge(
                Situation, Me, Ball.Location, OurGoal.Location);
            if (RawCanChallenge)
                challengeCommitUntil = Game.Time + (underPressure ? 0.36f : 0.27f);
            else if (Game.Time >= challengeCommitUntil || !challengeSafe)
                challengeCommitUntil = float.NegativeInfinity;
            ChallengeCommitted = RawCanChallenge ||
                (Game.Time < challengeCommitUntil && challengeSafe);

            // Spiderman defence owns the car while it hangs on the back wall or is releasing into a save.
            if (Action is WallGuard spider && !spider.Finished && (spider.OnWall || spider.Released))
            {
                SetDecision(spider.Released ? "defend / spiderman release" : "defend / spiderman");
                return;
            }

            if (emergency || counterDanger)
            {
                float dangerTime = emergency ? threat : counterThreat;
                float deadline = MathF.Max(0.05f, dangerTime - 0.025f);
                bool clearSide = Defense.IsGoalSide(
                    Me.Location, Ball.Location, OurGoal.Location, -100f);

                // Before a hard emergency, a clean shot at the opponent net is the strongest clear:
                // it removes the threat and can score. Only do this from safe goal-side ownership.
                if (counterDanger && clearSide && ChallengeCommitted)
                {
                    Shot counterShot = Tactics.SelectShot(
                        this, false, Situation.OpponentEta, HasClaim,
                        MathF.Min(deadline, 1.35f));
                    if (counterShot != null)
                    {
                        float counterContact = counterShot.Slice.Time - Game.Time;
                        bool directCounter = counterContact <= 0.82f &&
                            Tactics.GoalLaneOpen(
                                LivingOpponents, counterShot.Slice.Location, TheirGoal.Location);
                        if (directCounter || Tactics.PreferImmediateShot(this, counterShot, Situation))
                        {
                            Action = counterShot;
                            defensiveShot = null;
                            SetDecision("attack / counter-shot clear");
                            return;
                        }
                    }
                }

                // An airborne block already under way owns the car until it resolves.
                if (Action is AerialSave activeSave && !activeSave.Finished)
                {
                    SetDecision("defend / aerial block");
                    return;
                }

                // A formal clear is only legal from approximately goal-side geometry. The uploaded
                // match contained a probable own-goal acceleration from a wrong-side recovery dodge.
                if (clearSide)
                {
                    if (!(Action is Shot current) || !ReferenceEquals(Action, defensiveShot) ||
                        current.Slice.Time - Game.Time > deadline)
                    {
                        defensiveShot = Tactics.SelectShot(this, true, Situation.OpponentEta,
                            _ => false, deadline);
                        Action = defensiveShot;
                    }
                }
                else if (Action is Shot)
                {
                    Action = null;
                    defensiveShot = null;
                }

                // A clear that meets the ball late (or no clear at all) loses to an airborne block that
                // gets the car's body into the ball's path earlier.
                if (Options.AerialBlocks && emergency && (Ball.Location.z > 150f || crossing.z > 200f))
                {
                    AerialSave airBlock = AerialSave.TryCreate(Me, OurGoal.Location, deadline);
                    // Jump/double-jump clears use optimistic legacy reachability checks against airborne
                    // balls; the block's reachability is modelled. Only a clearly earlier ground shot or
                    // committed aerial strike keeps priority.
                    bool reliableClear = Action is GroundShot || Action is AerialStrike;
                    float clearTime = Action is Shot clear && clear.Slice != null
                        ? clear.Slice.Time : float.PositiveInfinity;
                    if (airBlock != null && (!reliableClear || airBlock.Plan.Time < clearTime - 0.12f))
                    {
                        Action = airBlock;
                        defensiveShot = null;
                        SetDecision("defend / aerial block");
                        return;
                    }
                }

                if (Action != null && !(Action is GoalLineSave))
                {
                    SetDecision(emergency
                        ? "defend / emergency clear"
                        : "defend / counter clear");
                    return;
                }

                if (Defense.TryDefensiveIntercept(
                    Me, Ball.Prediction, OurGoal.Location, Game.Time, deadline,
                    out Vec3 block))
                {
                    DriveTo(block, Car.MaxSpeed, true, false);
                    SetDecision(emergency
                        ? "defend / emergency intercept"
                        : "defend / counter intercept");
                    return;
                }

                if (!emergency)
                {
                    Vec3 counterReference = Defense.ReferenceBall(
                        Ball.Prediction, Ball.Location, OurGoal.Location, Game.Time);
                    bool counterGoalSide = Defense.IsGoalSide(
                        Me.Location, Ball.Location, OurGoal.Location, 20f);
                    Vec3 route = counterGoalSide
                        ? Defense.ShadowTarget(
                            counterReference, OurGoal.Location, DefensiveRole.Shadow,
                            MathF.Min(pressureTime, 0.35f))
                        : Defense.RecoveryTarget(Me.Location, counterReference, OurGoal.Location);
                    Vec3 counterSupport = Tactics.GoalReturnTarget(
                        Me, route, OurGoal.Location);
                    bool fastCounterRecovery = !counterGoalSide &&
                        Defense.CanFastRecover(
                            Me, Ball.Location, counterSupport, OurGoal.Location);
                    GuardTo(counterSupport, 2250f, 650f, false,
                        allowDodges: fastCounterRecovery);
                    SetDecision(counterGoalSide
                        ? "defend / counter shadow"
                        : "defend / counter recover");
                    return;
                }

                float crossingTime = Game.Time + MathF.Max(0f, threat);
                if (Action is GoalLineSave save && !save.Finished)
                {
                    save.Crossing = crossing;
                    save.CrossingTime = crossingTime;
                }
                else
                    Action = new GoalLineSave(Me, crossing, crossingTime);

                SetDecision("defend / goal-line save");
                return;
            }

            bool controlledPossession =
                PossessionControl.HasControlledPossession(Me, Ball.MainBall) ||
                PossessionControl.HasAirControl(Me, Ball.MainBall);
            bool canChallenge = ChallengeCommitted;
            float attackDeadline = Defense.AttackDeadline(Situation, controlledPossession);

            bool canOwnAttack = canChallenge || controlledPossession;
            Shot priorityAttack = canOwnAttack
                ? Tactics.SelectShot(
                    this, false, Situation.OpponentEta, HasClaim, attackDeadline)
                : null;
            bool finishNow = Tactics.PreferImmediateShot(
                this, priorityAttack, Situation);

            if (Action is Demolish hit && !hit.Finished)
            {
                SetDecision("mechanic / " + (hit.Plan.Demolition ? "demo " : "bump ") + hit.Plan.Reason);
                return;
            }

            if (Action is FlipResetPlay resetPlay && !resetPlay.Finished)
            {
                SetDecision(resetPlay.Confirmed ? "mechanic / flip reset shot" : "mechanic / flip reset");
                return;
            }

            if (Action is IPossessionAction possession)
            {
                if (finishNow)
                {
                    Action = priorityAttack;
                    SetDecision("attack / finish now");
                    return;
                }

                bool retain = Action is GroundCatch
                    ? Situation.TeamRank == 0 && (canChallenge || Situation.FreeTime >= -0.12f)
                    : PossessionControl.ShouldRetainPossession(
                        Situation, Me, Ball.MainBall, OurGoal.Location);

                if (retain && !possession.Finished)
                    return;
                Action = null;
            }

            if (Action is Shot shot)
            {
                if (canChallenge && shot.IsPredictionValid() &&
                    shot.Slice.Time - Game.Time <= attackDeadline &&
                    !HasTeammateEarlierShot(shot.Slice.Time))
                    return;
                Action = null;
            }

            if (Action is GetBoost refill)
            {
                if (!Defense.CanRefill(Situation, Me, Ball.Location, OurGoal.Location, underPressure))
                    Action = null;
                else if (!refill.Finished)
                {
                    SetDecision(refill.ChosenBoost?.IsLarge == true
                        ? "support / full-pad refill"
                        : "support / pad refill");
                    return;
                }
                else
                    Action = null;
            }

            if (!(Action is Drive) && !(Action is DefensiveDrive) && !(Action is Demolish) && !(Action is WallGuard))
                Action = null;

            if (!Me.IsGrounded)
            {
                if (finishNow)
                {
                    Action = priorityAttack;
                    SetDecision("attack / airborne finish");
                    return;
                }

                if (TryStartFlipReset())
                    return;

                bool canPossessAir = Options.AerialCarry &&
                    PossessionControl.CanAcquireAir(Situation, Me, Ball.MainBall, OurGoal.Location);
                if (canPossessAir &&
                    AerialCarry.CanStart(Me, Ball.MainBall, Situation.OpponentEta))
                {
                    Action = new AerialCarry();
                    SetDecision(underPressure
                        ? "mechanic / pressured aerial carry"
                        : "mechanic / aerial carry");
                    return;
                }

                Shot aerial = priorityAttack;
                Action = aerial ?? (IAction)new Recover();
                SetDecision(aerial == null
                    ? "recover / landing surface"
                    : "attack / airborne intercept");
                return;
            }

            bool canPossessGround = Options.GroundControl &&
                PossessionControl.CanAcquireGround(
                    Situation, Me, Ball.MainBall, OurGoal.Location);
            bool canDribble = canPossessGround &&
                GroundDribble.CanStart(Me, Ball.MainBall, Situation.FreeTime);

            // A direct scoring contact outranks continuing a dribble. Otherwise preserve controlled
            // possession and use its pressure-triggered outplays.
            if (finishNow)
            {
                Action = priorityAttack;
                SetDecision("attack / finish now");
                return;
            }

            if (TryStartFlipReset())
                return;

            if (controlledPossession && canDribble)
            {
                Action = new GroundDribble();
                SetDecision(underPressure
                    ? "mechanic / pressured ground carry"
                    : "mechanic / ground carry");
                return;
            }

            Shot attack = priorityAttack;
            BallSlice catchSlice = canPossessGround && Ball.Location.z > 175f
                ? GroundCatch.FindCatch(Me)
                : null;

            // Prefer a real scoring/clearing contact when it is imminent or contested. With time and a
            // descending ball, keep the softer catch available instead of forcing every touch.
            if (attack != null &&
                (underPressure || catchSlice == null || attack.Slice.Time - Game.Time <= 0.72f))
            {
                Action = attack;
                SetDecision(underPressure
                    ? "attack / pressured intercept"
                    : "attack / economical intercept");
                return;
            }

            if (canDribble)
            {
                Action = new GroundDribble();
                SetDecision(underPressure
                    ? "mechanic / pressured ground carry"
                    : "mechanic / ground carry");
                return;
            }

            if (catchSlice != null)
            {
                Action = new GroundCatch();
                SetDecision(underPressure
                    ? "mechanic / contested cushion catch"
                    : "mechanic / cushion catch");
                return;
            }

            if (attack != null)
            {
                Action = attack;
                SetDecision("attack / economical intercept");
                return;
            }

            if (Options.Demolitions && Demolitions.Choose(this, Situation, false) is HitPlan hitPlan)
            {
                Action = new Demolish(Me, hitPlan);
                SetDecision("mechanic / " + (hitPlan.Demolition ? "demo " : "bump ") + hitPlan.Reason);
                return;
            }

            if (canChallenge)
            {
                if (underPressure || Situation.FreeTime < 0.35f)
                {
                    Vec3 contact = Tactics.PressureChallengeTarget(
                        Me, Ball.Prediction, Ball.MainBall, TheirGoal.Location,
                        Game.Time, Situation.MyEta);
                    DriveTo(contact, Car.MaxSpeed, false);
                    SetDecision("attack / pressure challenge");
                    return;
                }

                Vec3 lane = PossessionControl.AttackingLane(
                    Me, Ball.MainBall, LivingOpponents,
                    TheirGoal.Location, OurGoal.Location);
                DriveTo(Field.LimitToNearestSurface(Ball.Location - lane * 300f),
                    1750f, false);
                SetDecision("possess / approach behind ball");
                return;
            }

            DefensiveRole role = Situation.TeamCount == 1
                ? DefensiveRole.Shadow
                : Defense.ShouldAnchor(Situation)
                    ? DefensiveRole.Anchor
                    : DefensiveRole.Support;

            Vec3 reference = Defense.ReferenceBall(Ball.Prediction, Ball.Location,
                OurGoal.Location, Game.Time);
            bool goalSide = Defense.IsGoalSide(Me.Location, Ball.Location, OurGoal.Location, 20f);
            bool recoveringGoalSide = !goalSide;

            Vec3 rawSupport = recoveringGoalSide
                ? Defense.RecoveryTarget(Me.Location, reference, OurGoal.Location)
                : Defense.ShadowTarget(reference, OurGoal.Location, role, pressureTime);
            Vec3 support = Tactics.GoalReturnTarget(Me, rawSupport, OurGoal.Location);
            bool exitingGoal = support.FlatDist(rawSupport) > 1f;

            if (!recoveringGoalSide && !exitingGoal &&
                Defense.CanRefill(Situation, Me, Ball.Location, OurGoal.Location, underPressure) &&
                TryBoostDetour(support))
                return;

            if (Options.WallGuard && !recoveringGoalSide && !exitingGoal &&
                global::Bot.WallGuard.Worthwhile(Situation, Me, OurGoal.Location, EmergencyThreatTime, LivingOpponents))
            {
                if (!(Action is WallGuard guard) || guard.Finished)
                    Action = new WallGuard(Me, Team);
                SetDecision("defend / spiderman");
                return;
            }

            float cruise;
            float terminal;
            bool hold;
            if (recoveringGoalSide || exitingGoal)
            {
                cruise = 2200f;
                terminal = 500f;
                hold = false;
            }
            else if (role == DefensiveRole.Shadow)
            {
                cruise = underPressure ? 2050f : 1850f;
                terminal = Defense.ShadowTerminalSpeed(Ball.MainBall, OurGoal.Location, underPressure);
                hold = false;
            }
            else if (role == DefensiveRole.Support)
            {
                cruise = 2050f;
                terminal = underPressure ? 700f : 500f;
                hold = false;
            }
            else
            {
                cruise = underPressure ? 1900f : 1700f;
                terminal = 0f;
                hold = true;
            }

            bool fastRecovery = recoveringGoalSide && !exitingGoal &&
                Defense.CanFastRecover(
                    Me, Ball.Location, support, OurGoal.Location);
            GuardTo(support, cruise, terminal, hold,
                allowDodges: fastRecovery);

            if (exitingGoal)
                SetDecision("defend / exit net");
            else if (recoveringGoalSide)
                SetDecision("defend / recover behind ball");
            else if (role == DefensiveRole.Shadow)
                SetDecision(underPressure ? "defend / solo shadow" : "defend / moving shadow");
            else if (role == DefensiveRole.Anchor)
                SetDecision(underPressure ? "defend / ball-goal anchor" : "support / goal-cover anchor");
            else
                SetDecision(underPressure ? "defend / second-man support" : "support / wide lane");
        }

        private bool HasClaim(float sliceTime) => HasTeammateEarlierShot(sliceTime);

        private bool TryStartFlipReset()
        {
            if (!Options.FlipResets || !FlipResetPlay.Worthwhile(
                    Situation, Me, Ball.MainBall, OurGoal.Location, EmergencyThreatTime))
                return false;
            FlipResetPlay play = FlipResetPlay.TryCreate(this);
            if (play == null || HasTeammateEarlierShot(play.ClaimTime))
                return false;
            Action = play;
            SetDecision("mechanic / flip reset");
            return true;
        }

        private bool TryBoostDetour(Vec3 destination)
        {
            if (Ball.Location.y * Field.Side(Team) > 2500f)
                return false;

            Boost pad = RoutePlanner.SelectBoost(Me, Field.Boosts, Ball.Location,
                destination, Team, Situation.OpponentEta);
            if (pad == null)
                return false;

            Action = new GetBoost(Me, pad.Index, interruptible: true);
            SetDecision(pad.IsLarge
                ? "support / full-pad refill"
                : "support / small-pad route");
            return true;
        }

        private void GuardTo(Vec3 destination, float cruiseSpeed, float terminalSpeed,
            bool holdPosition, bool allowDodges = false)
        {
            if (!ControlMath.Finite(destination))
                destination = OurGoal.Location;

            if (Action is DefensiveDrive guard)
            {
                guard.Target = destination;
                guard.CruiseSpeed = cruiseSpeed;
                guard.TerminalSpeed = terminalSpeed;
                guard.HoldPosition = holdPosition;
                guard.AllowDodges = allowDodges;
            }
            else
            {
                Action = new DefensiveDrive(Me, destination, cruiseSpeed,
                    terminalSpeed, holdPosition, allowDodges);
            }
        }

        private void DriveTo(Vec3 destination, float speed, bool allowDodges, bool allowHandbrake = true)
        {
            if (!ControlMath.Finite(destination))
                destination = OurGoal.Location;

            if (Action is Drive drive)
            {
                drive.Target = destination;
                drive.TargetSpeed = speed;
                drive.AllowDodges = allowDodges;
                drive.AllowHandbrake = allowHandbrake;
                drive.WasteBoost = false;
            }
            else
            {
                Action = new Drive(Me, destination, speed, allowDodges, wasteBoost: false)
                {
                    AllowHandbrake = allowHandbrake
                };
            }
        }

        private void SetDecision(string decision)
        {
            if (Decision == decision)
                return;

            string previous = Decision;
            Decision = decision;
            telemetry.Decision(this, previous);
            if (Options.Trace)
            {
                Console.WriteLine(FormattableString.Invariant(
                    $"stardust t={Game.Time:F3} car={Index} decision={Decision} rank={Situation.TeamRank}/{Situation.TeamCount} eta={Situation.MyEta:F2} opponent={Situation.OpponentEta:F2} pressure={Situation.PressureTime:F2} last_back={Situation.LastBack} cover={Situation.HasCover} goal_side={Defense.IsGoalSide(Me.Location, Ball.Location, OurGoal.Location)}"));
            }
        }

        protected override void OnOutputReady() => telemetry.Sample(this);

        // Retained for compatibility with the original Shadow action.
        public bool IsBack() => CanDefend(Me, OurGoal.Location) || Situation.FirstMan == Index;

        public static bool CanBlock(Car car, Vec3 location) =>
            ControlMath.Unit(location - car.Location, Vec3.Up)
                .Dot(ControlMath.Unit(car.Location - Ball.Location, Vec3.Up)) > 0.7f;

        public static bool CanDefend(Car car, Vec3 location)
        {
            if (CanBlock(car, location))
                return true;

            float eta = Drive.GetEta(car, location);
            float speed = MathF.Max(
                Ball.Velocity.Dot(ControlMath.Unit(location - Ball.Location, Vec3.Up)),
                1500f);
            return float.IsFinite(eta) && eta < Ball.Location.Dist(location) / speed;
        }
    }
}
