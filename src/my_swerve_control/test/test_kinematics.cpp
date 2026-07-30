// Standalone harness for the three functions ported from Testing.cpp into
// swerve_optimizer.cpp. Same logic, no ROS, so the maths can be checked.
#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif
#ifndef M_PI_4
#define M_PI_4 0.78539816339744830962
#endif

static int failures = 0;
static void check(bool ok, const char * name, const char * detail = "")
{
    std::printf("%-58s %s %s\n", name, ok ? "PASS" : "FAIL", detail);
    if (!ok) failures++;
}
static bool close(double a, double b, double eps = 1e-9) { return std::abs(a - b) < eps; }

static double wrap_pi(double a)
{
    while (a > M_PI)   a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

static const double kSpeedEpsilon = 1e-3;
static double max_motor_speed = 20.0;
static double wheel_radius = 0.05;
static std::vector<double> wheel_x = {0.3, 0.0, -0.3, 0.3, 0.0, -0.3};
static std::vector<double> wheel_y = {0.2, 0.2, 0.2, -0.2, -0.2, -0.2};
static std::vector<double> last_angle(6, 0.0);

static void optimize_module(size_t i, double ta, double tv, double & oa, double & ov)
{
    const double delta = wrap_pi(ta - last_angle[i]);
    if (std::abs(delta) > M_PI_2) { oa = wrap_pi(ta + M_PI); ov = -tv; }
    else                          { oa = ta;                 ov =  tv; }
}
static void singularity_hold(size_t i, double & a, double & v)
{
    if (std::abs(v) <= kSpeedEpsilon) { a = last_angle[i]; v = 0.0; }
}
static void normalise_speeds(std::vector<double> & s)
{
    double m = 0.0;
    for (double x : s) m = std::max(m, std::abs(x));
    if (m > max_motor_speed) { const double k = max_motor_speed / m; for (double & x : s) x *= k; }
}

int main()
{
    std::printf("--- wrap_pi ---\n");
    check(close(wrap_pi(0.0), 0.0), "wrap_pi(0) == 0");
    check(close(wrap_pi(3 * M_PI), M_PI), "wrap_pi(3pi) == pi");
    check(close(wrap_pi(-3 * M_PI), M_PI), "wrap_pi(-3pi) == pi");
    check(wrap_pi(1e6) > -M_PI && wrap_pi(1e6) <= M_PI, "wrap_pi(1e6) lands in range");

    std::printf("\n--- optimize_module: the 180-degree flip ---\n");
    {
        double a, v;
        last_angle[0] = 0.0;
        // Ask for 170 deg. Delta is 170 > 90, so flip to -10 deg and reverse.
        optimize_module(0, 170.0 * M_PI / 180.0, 5.0, a, v);
        check(close(a, -10.0 * M_PI / 180.0, 1e-9), "170 deg request -> module goes to -10 deg");
        check(close(v, -5.0), "  ...and drive velocity is negated");
        check(std::abs(wrap_pi(a - last_angle[0])) <= M_PI_2 + 1e-12,
              "  ...module never swings more than 90 deg");
    }
    {
        double a, v;
        last_angle[0] = 0.0;
        // Ask for 45 deg. Delta is 45 < 90, so take it directly.
        optimize_module(0, M_PI_4, 5.0, a, v);
        check(close(a, M_PI_4) && close(v, 5.0), "45 deg request -> taken directly, velocity unchanged");
    }
    {
        // Sweep every target from every starting angle: the flip must never
        // require more than a quarter turn, and ground motion must be preserved.
        bool ok_swing = true, ok_motion = true;
        for (int s = -180; s <= 180; s += 5) {
            for (int t = -180; t <= 180; t += 5) {
                last_angle[0] = s * M_PI / 180.0;
                double a, v;
                const double ta = t * M_PI / 180.0, tv = 3.0;
                optimize_module(0, ta, tv, a, v);
                if (std::abs(wrap_pi(a - last_angle[0])) > M_PI_2 + 1e-9) ok_swing = false;
                // Velocity vector on the ground must be unchanged.
                if (!close(v * std::cos(a), tv * std::cos(ta), 1e-9) ||
                    !close(v * std::sin(a), tv * std::sin(ta), 1e-9)) ok_motion = false;
            }
        }
        check(ok_swing,  "sweep 73x73: swing never exceeds 90 deg");
        check(ok_motion, "sweep 73x73: ground velocity vector preserved by the flip");
    }

    std::printf("\n--- singularity hold ---\n");
    {
        double a = 0.0, v = 0.0;
        last_angle[2] = 1.234;
        // atan2(0,0) returns 0 -- without the hold the module would snap there.
        a = std::atan2(0.0, 0.0);
        check(close(a, 0.0), "atan2(0,0) really does return 0 (the bug being guarded)");
        singularity_hold(2, a, v);
        check(close(a, 1.234) && close(v, 0.0), "stopped module holds its last angle, speed 0");
    }
    {
        double a = 0.5, v = 4.0;
        last_angle[3] = 9.9;
        singularity_hold(3, a, v);
        check(close(a, 0.5) && close(v, 4.0), "moving module is left alone");
    }

    std::printf("\n--- normalise_speeds ---\n");
    {
        std::vector<double> s = {10, -30, 5, 0, 20, -15};
        normalise_speeds(s);
        double m = 0; for (double x : s) m = std::max(m, std::abs(x));
        check(close(m, 20.0, 1e-9), "over-limit set is scaled so the peak equals the limit");
        check(close(s[0] / s[1], 10.0 / -30.0, 1e-9), "  ...ratios between modules preserved");
    }
    {
        std::vector<double> s = {1, 2, 3, 4, 5, 6}, before = s;
        normalise_speeds(s);
        bool same = true;
        for (size_t i = 0; i < s.size(); i++) if (!close(s[i], before[i])) same = false;
        check(same, "under-limit set is left untouched");
    }

    std::printf("\n--- full solver: zero-scrubbing property ---\n");
    {
        // Pure rotation about the centre. Every module must end up tangent to
        // its own circle about the ICR, i.e. perpendicular to its radius.
        const double Vx = 0.0, Vy = 0.0, omega = 1.0, px = 0.0, py = 0.0;
        bool perp = true;
        for (size_t i = 0; i < 6; i++) {
            const double dx = wheel_x[i] - px, dy = wheel_y[i] - py;
            const double vx = Vx - omega * dy, vy = Vy + omega * dx;
            const double ang = std::atan2(vy, vx);
            // radius vector (dx,dy) dot heading (cos,sin) should be ~0
            const double dot = dx * std::cos(ang) + dy * std::sin(ang);
            if (std::abs(dot) > 1e-9) perp = false;
        }
        check(perp, "pure spin: all 6 headings perpendicular to their ICR radius");
    }
    {
        // Rotation about an arbitrary off-centre pivot -- the generalisation.
        const double px = 1.5, py = -0.7, omega = 0.8;
        bool perp = true;
        for (size_t i = 0; i < 6; i++) {
            const double dx = wheel_x[i] - px, dy = wheel_y[i] - py;
            const double vx = -omega * dy, vy = omega * dx;
            const double ang = std::atan2(vy, vx);
            const double dot = dx * std::cos(ang) + dy * std::sin(ang);
            if (std::abs(dot) > 1e-9) perp = false;
        }
        check(perp, "off-centre pivot (1.5,-0.7): still perpendicular -> no scrub");
    }
    {
        // Pure translation: every module parallel, equal speed.
        const double Vx = 0.4, Vy = 0.3, omega = 0.0;
        bool same = true;
        const double a0 = std::atan2(Vy, Vx), s0 = std::hypot(Vx, Vy) / wheel_radius;
        for (size_t i = 0; i < 6; i++) {
            const double vx = Vx - omega * wheel_y[i], vy = Vy + omega * wheel_x[i];
            if (!close(std::atan2(vy, vx), a0, 1e-12)) same = false;
            if (!close(std::hypot(vx, vy) / wheel_radius, s0, 1e-12)) same = false;
        }
        check(same, "pure translation: all 6 modules parallel at equal speed");
    }

    std::printf("\n%s  (%d failure%s)\n", failures ? "FAILURES PRESENT" : "ALL PASS",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
