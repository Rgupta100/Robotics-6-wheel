// Scrub benchmark: how much sideways dragging does the swerve solver avoid?
//
// Scrub is the component of a wheel's ground velocity perpendicular to the
// direction it is pointing. A wheel that is not pointing where it is going has
// to drag sideways to get there -- that is wasted energy and tyre wear.
//
// For an IDEAL swerve, scrub is exactly zero by construction: every module is
// steered onto its own velocity vector. Reporting "100% reduction" would be
// true and useless. Real modules cannot snap instantly, so this benchmark
// rate-limits the steering (--rate, rad/s) and measures the scrub that
// survives during transients. That is the number worth quoting.
//
// Baseline is a fixed-heading (skid-steer) rover: all six wheels locked
// forward, which is what the platform does without a steering solver.
//
// Build standalone:
//   g++ -std=c++17 -O2 benchmark/scrub_benchmark.cpp -o scrub_benchmark
//   ./scrub_benchmark
//
// Options: --rate <rad/s>  --dt <s>  --csv <file>

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

static double wrap_pi(double a)
{
    while (a > M_PI)   a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

static const double WX[6] = {0.3, 0.0, -0.3, 0.3, 0.0, -0.3};
static const double WY[6] = {0.2, 0.2, 0.2, -0.2, -0.2, -0.2};

struct Twist { double vx, vy, omega; };

// A representative manoeuvre: straight, then a figure-eight, then a strafe,
// then a spin in place. The spin and the strafe are where a fixed-heading
// platform suffers most, and they are exactly what a rover does when
// repositioning.
static Twist path(double t)
{
    if (t < 4.0)  return { 0.6,  0.0,  0.0 };                      // straight
    if (t < 16.0) return { 0.6,  0.0,  0.8 * std::sin((t - 4.0) * M_PI / 3.0) };  // figure-eight
    if (t < 20.0) return { 0.0,  0.5,  0.0 };                      // strafe
    if (t < 24.0) return { 0.0,  0.0,  1.2 };                      // spin in place
    return { 0.0, 0.0, 0.0 };
}

struct Result { double scrub, roll; };

// mode 0 = swerve with rate-limited steering, mode 1 = fixed heading
static Result run(int mode, double max_rate, double dt, const char * csv_path)
{
    std::vector<double> angle(6, 0.0);
    double scrub_total = 0.0, roll_total = 0.0;
    std::FILE * csv = nullptr;
    if (csv_path) {
        csv = std::fopen(csv_path, "w");
        if (csv) std::fprintf(csv, "t,wheel,scrub_rate,heading,desired\n");
    }

    for (double t = 0.0; t < 24.0; t += dt) {
        const Twist c = path(t);

        for (int i = 0; i < 6; i++) {
            // True ground velocity of this module, from the chassis twist.
            const double vx = c.vx - c.omega * WY[i];
            const double vy = c.vy + c.omega * WX[i];
            const double speed = std::hypot(vx, vy);

            double desired = angle[i];
            if (mode == 0) {
                if (speed > 1e-6) {
                    desired = std::atan2(vy, vx);
                    // Shortest-path flip, same rule as the node.
                    if (std::abs(wrap_pi(desired - angle[i])) > M_PI_2)
                        desired = wrap_pi(desired + M_PI);
                }
                // Finite steering rate: the module chases, it does not snap.
                const double delta = wrap_pi(desired - angle[i]);
                const double step = std::max(-max_rate * dt,
                                             std::min(max_rate * dt, delta));
                angle[i] = wrap_pi(angle[i] + step);
            } else {
                angle[i] = 0.0;   // locked forward
            }

            // Scrub is the perpendicular component; roll is along the heading.
            const double nx = -std::sin(angle[i]), ny = std::cos(angle[i]);
            const double hx =  std::cos(angle[i]), hy = std::sin(angle[i]);
            const double scrub_rate = std::abs(vx * nx + vy * ny);
            const double roll_rate  = std::abs(vx * hx + vy * hy);

            scrub_total += scrub_rate * dt;
            roll_total  += roll_rate * dt;

            if (csv) std::fprintf(csv, "%.3f,%d,%.6f,%.6f,%.6f\n",
                                  t, i, scrub_rate, angle[i], desired);
        }
    }
    if (csv) std::fclose(csv);
    return { scrub_total, roll_total };
}

int main(int argc, char ** argv)
{
    double max_rate = 6.0;    // rad/s, ~1 rev/s -- a normal steering module
    double dt = 0.001;
    const char * csv = nullptr;

    for (int i = 1; i < argc - 1; i++) {
        if (!std::strcmp(argv[i], "--rate")) max_rate = std::atof(argv[i + 1]);
        if (!std::strcmp(argv[i], "--dt"))   dt       = std::atof(argv[i + 1]);
        if (!std::strcmp(argv[i], "--csv"))  csv      = argv[i + 1];
    }

    const Result sw = run(0, max_rate, dt, csv);
    const Result fx = run(1, max_rate, dt, nullptr);

    const double reduction = 100.0 * (fx.scrub - sw.scrub) / fx.scrub;

    std::printf("Scrub benchmark -- 24 s path (straight, figure-eight, strafe, spin)\n");
    std::printf("  steering rate limit : %.2f rad/s\n", max_rate);
    std::printf("  timestep            : %.4f s\n\n", dt);
    std::printf("  %-28s %12s %12s\n", "", "scrub (m)", "roll (m)");
    std::printf("  %-28s %12.4f %12.4f\n", "fixed heading (baseline)", fx.scrub, fx.roll);
    std::printf("  %-28s %12.4f %12.4f\n", "swerve (rate-limited)", sw.scrub, sw.roll);
    std::printf("\n  scrub reduction     : %.1f%%\n", reduction);
    std::printf("  residual swerve scrub is transient only -- it occurs while\n"
                "  modules are still slewing toward their commanded heading.\n");

    if (csv) std::printf("\n  per-wheel trace written to %s\n", csv);
    return 0;
}
