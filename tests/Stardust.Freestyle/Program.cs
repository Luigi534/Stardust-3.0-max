using System.Reflection;
using Bot;
using RedUtils;
using RedUtils.Math;
using RLBot.Flat;
using BallPrediction = RedUtils.BallPrediction;

// Deterministic checks of the freestyle/defence geometry and control primitives. These establish
// software properties (convergence, sign conventions, symmetry, safety rules), not match strength.
int passed = 0, failed = 0;
void Test(string name, Action action)
{
    try { action(); passed++; Console.WriteLine($"PASS {name}"); }
    catch (Exception e) { failed++; Console.WriteLine($"FAIL {name}: {e.Message}"); }
}
void Check(bool value, string message) { if (!value) throw new Exception(message); }
void Set(Type type, string property, object? instance, object value) =>
    type.GetProperty(property, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static | BindingFlags.Instance)!
        .SetValue(instance, value);

Set(typeof(Game), nameof(Game.Gravity), null, new Vec3(0, 0, -650));
Set(typeof(Game), nameof(Game.Time), null, 10f);
Vec3 g = new(0, 0, -650);

// Integrate a point mass under gravity with a guidance law, capping acceleration like real thrust.
(Vec3 position, Vec3 velocity) Fly(Vec3 x, Vec3 v, Func<Vec3, Vec3, float, Vec3> control, float seconds, float cap = 1100f)
{
    const float dt = 1f / 120f;
    for (float t = 0; t < seconds - 1e-4f; t += dt)
    {
        // Like the controllers, stop correcting in the final instants where 1/τ² gains are meaningless.
        Vec3 u = seconds - t > 0.06f ? control(x, v, seconds - t) : Vec3.Zero;
        if (u.Length() > cap) u = u.Normalize() * cap;
        v += (u + g) * dt;
        x += v * dt;
    }
    return (x, v);
}

Test("guidance: position-only law reaches a target with gravity", () =>
{
    var rng = new Random(11);
    for (int i = 0; i < 200; i++)
    {
        Vec3 x0 = new(rng.Next(-1000, 1000), rng.Next(-1000, 1000), rng.Next(100, 800));
        Vec3 v0 = new(rng.Next(-500, 500), rng.Next(-500, 500), rng.Next(-300, 500));
        Vec3 target = x0 + new Vec3(rng.Next(-600, 600), rng.Next(-600, 600), rng.Next(-200, 400));
        // Only feasible cases (what the planner would accept): initial demand within 80% of thrust.
        if (AerialContact.Guidance(x0, v0, target, Vec3.Zero, 1.5f, g, 0f).Length() > 0.8f * 1600f)
            continue;
        var (x, _) = Fly(x0, v0, (x, v, tau) => AerialContact.Guidance(x, v, target, Vec3.Zero, tau, g, 0f), 1.5f, 1600f);
        Check(x.Dist(target) < 30f, $"miss {x.Dist(target):F1} uu for case {i}");
    }
});

Test("guidance: terminal-velocity law matches the requested arrival velocity", () =>
{
    Vec3 target = new(300, 900, 1000), vf = new(0, 400, 200);
    var (x, v) = Fly(new Vec3(0, 0, 500), new Vec3(0, 800, 300),
        (x, v, tau) => AerialContact.Guidance(x, v, target, vf, tau, g, 1f), 1.4f, 5000f);
    Check(x.Dist(target) < 30f, $"position miss {x.Dist(target):F1}");
    Check((v - vf).Length() < 90f, $"velocity miss {(v - vf).Length():F1}");
});

Test("coast contact: ball above and closing gives an upward wheel normal", () =>
{
    Vec3 push = Vec3.Up;
    Vec3? n = FlipResetPlay.CoastContact(new Vec3(0, 0, 250), new Vec3(0, 0, -300), push);
    Check(n != null && n.Value.z > 0.95f, "expected an upward contact normal");
});

Test("coast contact: separating, sideways or violent approaches are rejected", () =>
{
    Vec3 push = Vec3.Up;
    Check(FlipResetPlay.CoastContact(new Vec3(0, 0, 250), new Vec3(0, 0, 300), push) == null, "separating accepted");
    Check(FlipResetPlay.CoastContact(new Vec3(300, 0, 20), new Vec3(-600, 0, 0), push) == null, "side contact accepted");
    Check(FlipResetPlay.CoastContact(new Vec3(0, 0, 400), new Vec3(0, 0, -1400), push) == null, "violent closing accepted");
});

Test("goal-side contact: defender behind the ball is safe and aims from the goal side", () =>
{
    Vec3 goal = new(0, -5120, 0);
    var car = new Car { Location = new Vec3(0, -4700, 17), Forward = Vec3.Y, Up = Vec3.Up };
    Check(Defense.SafeContactTarget(car, new Vec3(0, -3500, 93), goal, out Vec3 target), "goal-side defender rejected");
    Check(target.y < -3500f, $"contact point {target} is not on the goal side of the ball");
});

Test("goal-side contact: routes that clip the ball from the field side are rejected", () =>
{
    Vec3 goal = new(0, -5120, 0);
    var car = new Car { Location = new Vec3(0, -2500, 17), Forward = -Vec3.Y, Up = Vec3.Up };
    Check(!Defense.SafeContactTarget(car, new Vec3(0, -3500, 93), goal, out _), "own-goal geometry accepted");
    car.Location = new Vec3(1400, -2500, 17);
    Check(!Defense.SafeContactTarget(car, new Vec3(0, -3500, 93), goal, out _), "upfield route passing 94 uu from the ball accepted");
    car.Location = new Vec3(1500, -4200, 17);
    Check(Defense.SafeContactTarget(car, new Vec3(0, -3500, 93), goal, out _), "wide goal-side defender rejected");
});

Test("goal line: a ball at the line puts the guard between ball and net, inside the posts", () =>
{
    foreach (int team in new[] { 0, 1 })
    {
        float side = team == 0 ? -1 : 1;
        Vec3 goal = new(0, side * 5120, 0);
        Vec3 ball = new(-650, side * 5100, 110);
        Vec3 target = Defense.EmergencyTarget(new Vec3(-650, side * 5120, 110), goal, ball);
        Check(target.y * side > ball.y * side, $"team {team}: guard {target} is field-side of ball {ball}");
        Check(MathF.Abs(target.x) < Goal.Width * 0.5f, $"team {team}: guard {target} outside the posts");
    }
});

Test("goal line: far from the line the guard keeps the crossing-point target", () =>
{
    Vec3 goal = new(0, -5120, 0);
    Vec3 crossing = new(300, -5120, 200);
    Vec3 legacy = Defense.EmergencyTarget(crossing, goal);
    Vec3 target = Defense.EmergencyTarget(crossing, goal, new Vec3(300, -2500, 200));
    Check(target.Dist(legacy) < 1f, "far ball changed the goal-line target");
});

Test("goal line: blue and orange targets are mirror images", () =>
{
    var rng = new Random(5);
    for (int i = 0; i < 500; i++)
    {
        Vec3 ball = new(rng.Next(-1500, 1500), -rng.Next(3800, 5200), rng.Next(93, 300));
        Vec3 crossing = new(rng.Next(-900, 900), -5120, rng.Next(93, 600));
        Vec3 blue = Defense.EmergencyTarget(crossing, new Vec3(0, -5120, 0), ball);
        Vec3 orange = Defense.EmergencyTarget(new Vec3(-crossing.x, -crossing.y, crossing.z),
            new Vec3(0, 5120, 0), new Vec3(-ball.x, -ball.y, ball.z));
        Check(MathF.Abs(blue.x + orange.x) < 0.5f && MathF.Abs(blue.y + orange.y) < 0.5f, $"asymmetry at case {i}");
    }
});

Test("impulse model: inverted push sends the ball within 8 degrees of the aim", () =>
{
    var rng = new Random(3);
    for (int i = 0; i < 300; i++)
    {
        Vec3 ball = new(rng.Next(-2000, 2000), rng.Next(-1000, 3000), rng.Next(300, 1200));
        Vec3 ballVelocity = new(rng.Next(-600, 600), rng.Next(-600, 600), rng.Next(-400, 400));
        Vec3 aim = new(rng.Next(-600, 600), 5120, 300);
        Vec3 carVelocity = ControlMath.Unit(aim - ball, Vec3.Up) * 1600f;
        Vec3 push = AerialPlanner.PushToward(ball, ballVelocity, carVelocity, aim);
        Vec3 outgoing = ControlMath.Unit(AerialPlanner.Outgoing(ballVelocity, carVelocity, push, push), Vec3.Up);
        float flight = (aim - ball).Length() / 2200f;
        Vec3 desired = ControlMath.Unit(aim - ball + new Vec3(0, 0, 325f * flight * flight), Vec3.Up);
        float angle = MathF.Acos(System.Math.Clamp(outgoing.Dot(desired), -1f, 1f));
        Check(angle < 0.14f, $"case {i}: {angle * 57.3f:F1} degrees off");
    }
});

Test("geometry: angle limit clamps toward the axis and preserves small angles", () =>
{
    Vec3 axis = Vec3.X;
    Vec3 limited = AerialPlanner.LimitAngle(Vec3.Y, axis, 0.5f);
    Check(MathF.Abs(MathF.Acos(limited.Dot(axis)) - 0.5f) < 1e-3f, "large angle not clamped to the limit");
    Vec3 small = ControlMath.Unit(new Vec3(1, 0.2f, 0), Vec3.X);
    Check(AerialPlanner.LimitAngle(small, axis, 0.5f).Dist(small) < 1e-4f, "small angle altered");
});

Test("aerial: climb time grows with height and launch horizon covers it", () =>
{
    float previous = 0f;
    for (float h = 100f; h <= 1900f; h += 100f)
    {
        float rise = AerialPlanner.RiseTime(h);
        Check(float.IsFinite(rise) && rise >= previous, $"non-monotonic rise time at {h}");
        previous = rise;
    }
    var car = new Car { Location = new Vec3(0, 0, 17), Up = Vec3.Up, Forward = Vec3.X, IsGrounded = true, Boost = 100 };
    Check(AerialPlanner.LaunchHorizon(car, new Vec3(0, 0, 900)) > AerialPlanner.RiseTime(883f), "horizon shorter than climb");
});

Test("reset evidence: a reset during the first-jump window is confirmed", () =>
{
    JumpState Jump(bool jumped, AirState state = AirState.InAir) =>
        new(new PlayerInfoT { HasJumped = jumped, AirState = state, DodgeTimeout = jumped ? 0.8f : -1f });
    var evidence = new ResetEvidence();
    Check(!evidence.Observe(Jump(true), false, false, 900, 1f), "confirmed without contact");
    Check(evidence.Observe(Jump(false), true, true, 900, 1.05f), "single-jump reset not confirmed");
    evidence.Reset();
    Check(!evidence.Observe(Jump(false), true, true, 900, 1.2f), "reset evidence persisted after Reset()");
});

Test("demolition: arrival speed is monotonic and airborne targets are not planned", () =>
{
    float slow = Demolitions.ArrivalSpeed(500f, 1000f, 50f), fast = Demolitions.ArrivalSpeed(2500f, 1000f, 50f);
    Check(fast >= slow && fast <= Car.MaxSpeed + 1f, $"arrival speeds {slow}/{fast}");
    var me = new Car { Location = new Vec3(0, 0, 17), Velocity = new Vec3(0, 2000, 0), Forward = Vec3.Y, Up = Vec3.Up, IsGrounded = true, Boost = 50 };
    var flying = new Car { Location = new Vec3(0, 1500, 400), Velocity = new Vec3(0, -500, 0), IsGrounded = false };
    Check(Demolitions.Plan(me, flying) == null, "planned a ground hit on an airborne car");
});

Test("spiderman: hang point is on our back wall, beyond the post or above the bar", () =>
{
    foreach (int team in new[] { 0, 1 })
    {
        foreach (float bx in new[] { -3000f, -800f, 0f, 900f, 3200f })
        {
            var slices = new BallSlice[] { new(10.3f, new Vec3(bx, Field.Side(team) * 3500, 600), Vec3.Zero),
                                           new(11.5f, new Vec3(bx, Field.Side(team) * 3600, 500), Vec3.Zero) };
            Set(typeof(Ball), nameof(Ball.Location), null, new Vec3(bx, Field.Side(team) * 3500, 600));
            Set(typeof(Ball), nameof(Ball.Prediction), null, new BallPrediction { Slices = slices });
            Vec3 hang = WallGuard.HangPoint(team);
            Check(MathF.Abs(MathF.Abs(hang.y) - (Field.Length * 0.5f - 17f)) < 1f && hang.y * Field.Side(team) > 0, $"hang {hang} not on own back wall");
            Check(MathF.Abs(hang.x) > Goal.Width * 0.5f || hang.z > Goal.Height, $"hang {hang} inside the goal mouth");
        }
    }
});

Test("air dribble: keep-up aim lifts the ball and carries it toward the attacked goal", () =>
{
    foreach (float gy in new[] { 5120f, -5120f })
    {
        foreach (Vec3 ball in new[] { new Vec3(0, 0, 600), new Vec3(-2500, 1500, 900), new Vec3(3000, -2000, 400) })
        {
            Vec3 goal = new Vec3(0, gy, 300);
            Vec3 aim = AirDribble.KeepUpTarget(ball, goal) - ball;
            Check(aim.z > 500f, $"keep-up aim {aim} does not lift the ball");
            Check(aim.Flatten().Dot((goal - ball).Flatten()) > 0f, $"keep-up aim {aim} carries the ball away from goal");
        }
    }
});

Console.WriteLine($"FREESTYLE RESULT: {passed} passed, {failed} failed.");
Environment.ExitCode = failed == 0 ? 0 : 1;
