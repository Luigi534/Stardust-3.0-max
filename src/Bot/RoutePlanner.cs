using System;
using System.Collections.Generic;
using RedUtils;
using RedUtils.Math;

namespace Bot
{
    public static class RoutePlanner
    {
        public static float Detour(Vec3 start, Vec3 pad, Vec3 destination) =>
            MathF.Max(0, start.FlatDist(pad) + pad.FlatDist(destination) - start.FlatDist(destination));

        /// <summary>
        /// Select a covered refill. Small pads remain strict on-route pickups; a critically low support
        /// car may make a bounded full-pad excursion only when the opponent window covers the pickup.
        /// </summary>
        public static Boost SelectBoost(Car car, IEnumerable<Boost> pads, Vec3 ball, Vec3 destination,
            int team, float opponentEta, Func<Car, Vec3, float> travelTime = null)
        {
            if (pads == null || car.IsDemolished || car.Boost >= 35 || !float.IsFinite(opponentEta) || opponentEta < 1)
                return null;

            travelTime ??= (c, target) => Drive.GetEta(c, target);
            bool critical = car.Boost < 20;
            Boost best = null;
            float bestCost = float.PositiveInfinity;

            foreach (Boost pad in pads)
            {
                if (pad == null || !ControlMath.Finite(pad.Location)) continue;
                // Defensive/support refills stay goal-side of the ball.
                if (pad.Location.y * Field.Side(team) < ball.y * Field.Side(team)) continue;

                float eta = travelTime(car, pad.Location);
                if (!float.IsFinite(eta) || eta < 0) continue;
                // A pad which respawns before arrival is a valid pickup; do not require it active now.
                if (!pad.IsActive && pad.TimeUntilActive > eta + 0.12f) continue;

                float detour = Detour(car.Location, pad.Location, destination);
                bool deliberateFull = pad.IsLarge && critical && detour <= 2600 &&
                    opponentEta >= eta + 1.0f;

                if (!deliberateFull)
                {
                    if (detour > 450 || eta + 0.5f > opponentEta) continue;
                }

                float usefulBoost = MathF.Min(100 - car.Boost, pad.IsLarge ? 100 : 12);
                float value = (pad.IsLarge ? 9f : 5f) * usefulBoost;
                float cost = detour + 120 * eta - value;
                if (cost < bestCost)
                {
                    best = pad;
                    bestCost = cost;
                }
            }
            return best;
        }
    
        /// <summary>
        /// Pad-aware routing: an active pad that lies on the way to <paramref name="destination"/> (ahead of the car,
        /// close to the straight line, goal-side of the ball) and can be driven through with a negligible detour.
        /// Collecting small pads while rotating is where most boost comes from; returns null if none qualifies.
        /// </summary>
        public static Vec3? PadWaypoint(Car car, IEnumerable<Boost> pads, Vec3 destination, Vec3 ball, int team)
        {
            if (pads == null || car == null || !car.IsGrounded || car.Boost >= 90f)
                return null;
            Vec3 start = car.Location.Flatten(), end = destination.Flatten();
            Vec3 path = end - start;
            float length = path.Length();
            if (length < 800f)
                return null;
            Vec3 dir = path / length;
            float side = Field.Side(team);
            Vec3? best = null;
            float bestDetour = float.PositiveInfinity;
            foreach (Boost pad in pads)
            {
                if (pad == null || !pad.IsActive)
                    continue;
                Vec3 p = pad.Location.Flatten();
                float along = (p - start).Dot(dir);
                if (along < length * 0.1f || along > length * 0.85f)
                    continue;
                float lateral = (p - start - dir * along).Length();
                float limit = pad.IsLarge && car.Boost < 50f ? 350f : 180f;
                if (lateral > limit)
                    continue;
                // Never trade defensive position for boost: the pad must be goal-side of the ball.
                if (pad.Location.y * side < ball.y * side - 200f)
                    continue;
                float detour = p.Dist(start) + p.Dist(end) - length;
                if (detour < bestDetour)
                {
                    bestDetour = detour;
                    best = new Vec3(pad.Location.x, pad.Location.y, destination.z);
                }
            }
            return best;
        }
}
}
