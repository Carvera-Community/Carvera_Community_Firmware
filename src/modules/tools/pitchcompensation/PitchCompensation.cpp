#include "PitchCompensation.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "Config.h"
#include "checksumm.h"
#include "modules/robot/Robot.h"
#include "ConfigValue.h"
#include "StreamOutput.h"
#include "libs/StreamOutputPool.h"

#include <string>
#include <cmath>
#include <functional>
#include <algorithm>
#include <cstring>

#define pitch_compensation_checksum CHECKSUM("pitch_compensation")
#define enable_checksum             CHECKSUM("enable")
#define points_checksum             CHECKSUM("points")

using namespace std;

PitchCompensation::PitchCompensation() {
    this->enable = false;
}

void PitchCompensation::on_module_loaded() {
    this->config_load();

    if(this->enable) {
        // Register compensation transform
        using std::placeholders::_1;
        using std::placeholders::_2;
        using std::placeholders::_3;
        THEROBOT->pitchCompensationTransform = std::bind(&PitchCompensation::do_compensation, this, _1, _2, _3);
    }
}

void PitchCompensation::config_load() {
    this->enable = THEKERNEL->config->value(pitch_compensation_checksum, enable_checksum)->by_default(false)->as_bool();

    if(!this->enable) return;

    // Load axis configurations
    // pitch_compensation.points.x_axis = ...
    string axes = "XYZ";
    for(char c : axes) {
        uint16_t axis_chk = 0;
        if(c == 'X') axis_chk = CHECKSUM("x_axis");
        else if(c == 'Y') axis_chk = CHECKSUM("y_axis");
        else if(c == 'Z') axis_chk = CHECKSUM("z_axis");

        string points_str = THEKERNEL->config->value(pitch_compensation_checksum, points_checksum, axis_chk)->by_default("")->as_string();

        if(!points_str.empty()) {
            AxisCompensation axis_comp;

            // Parse the points string into a series of position:multiplier pairs
            const char* str = points_str.c_str();
            const char* start = str;
            const char* end = str + points_str.length();

            while(start < end) {
                // Find next comma or end of string
                const char* comma = start;
                while(comma < end && *comma != ',') comma++;

                // Find colon within this segment
                const char* colon = start;
                while(colon < comma && *colon != ':') colon++;

                if(colon < comma) {
                    float pos = strtof(start, NULL);
                    float mul = strtof(colon + 1, NULL);
                    axis_comp.points.push_back({pos, mul, 0.0});
                }

                // Move to next segment
                start = comma + 1;
            }

            if(!axis_comp.points.empty()) {
                // Sort points by position and precompute integrals
                std::sort(axis_comp.points.begin(), axis_comp.points.end());
                precompute_integrals(axis_comp);
                axis_comp.points.shrink_to_fit();
                axis_compensations[c] = axis_comp;
            }
        }
    }
}

void PitchCompensation::precompute_integrals(AxisCompensation& axis_comp) {
    if(axis_comp.points.empty()) return;

    // First pass: compute integrals starting from 0 at the first point
    CompensationPoint& first = axis_comp.points[0];
    double integral = 0.0;
    first.integral = integral;

    float prev_pos = first.pos;
    float prev_mul = first.multiplier;

    for(size_t i = 1; i < axis_comp.points.size(); ++i) {
        CompensationPoint& cp = axis_comp.points[i];
        float dx = cp.pos - prev_pos;

        // Trapezoidal rule
        integral += dx * (prev_mul + cp.multiplier) / 2.0;
        cp.integral = integral;

        prev_pos = cp.pos;
        prev_mul = cp.multiplier;
    }

    // Normalize so that C(0) = 0
    // This ensures homing works correctly - when the machine homes to physical
    // position 0, the compensated position should also be 0
    double c_at_zero = integrate(axis_comp, 0.0f);
    for(auto& cp : axis_comp.points) {
        cp.integral -= c_at_zero;
    }
}

double PitchCompensation::integrate(const AxisCompensation& axis_comp, float pos) {
    if(axis_comp.points.empty()) return pos;

    // Get the first point with position > pos
    CompensationPoint search_val = {pos, 0, 0};
    auto it = std::upper_bound(axis_comp.points.begin(), axis_comp.points.end(), search_val);

    // Before start (using first multiplier constant)
    if(it == axis_comp.points.begin()) {
        const CompensationPoint& p0 = axis_comp.points.front();
        // Extrapolate backwards using the first multiplier constant
        // C(x) = C(p0) - (p0 - x) * m0
        return p0.integral - (p0.pos - pos) * p0.multiplier;
    }

    // After end (using last multiplier constant)
    if(it == axis_comp.points.end()) {
        const CompensationPoint& p_last = axis_comp.points.back();
        // Extrapolate forwards using the last multiplier constant
        // C(x) = C(last) + (x - last_pos) * last_mul
        return p_last.integral + (pos - p_last.pos) * p_last.multiplier;
    }

    // Between two points p1 and p2 where:
    // p2 => end of the segment (it)
    // p1 => start of the segment (it - 1, safe since it != begin)
    const CompensationPoint& p2 = *it;
    const CompensationPoint& p1 = *(--it);

    float dx = pos - p1.pos;
    float range = p2.pos - p1.pos;

    // Interpolated multiplier at pos
    // m(x) = m1 + (m2 - m1) * (x - p1) / range
    // Integral = C1 + m1 * dx + 0.5 * slope * dx*dx
    float slope = (p2.multiplier - p1.multiplier) / range;
    return p1.integral + p1.multiplier * dx + 0.5 * slope * dx * dx;
}

float PitchCompensation::inverse_integrate(const AxisCompensation& axis_comp, double val) {
    if(axis_comp.points.empty()) return (float)val;

    // Find the first point with integral >= val
    auto it = std::lower_bound(axis_comp.points.begin(), axis_comp.points.end(), val,
        [](const CompensationPoint& cp, double v) {
            return cp.integral < v;
        });

    // Before start (using first multiplier constant)
    if(it == axis_comp.points.begin()) {
        const CompensationPoint& p0 = axis_comp.points.front();
        // val = C(p0) - (p0 - x) * m0
        // val - C(p0) = -p0*m0 + x*m0
        // x = (val - C(p0))/m0 + p0
        return (val - p0.integral) / p0.multiplier + p0.pos;
    }

    // After end (using last multiplier constant)
    if(it == axis_comp.points.end()) {
        const CompensationPoint& p_last = axis_comp.points.back();
        // val = C(last) + (x - last_pos) * last_mul
        // val - C(last) = (x - last_pos) * last_mul
        // x = (val - C(last))/last_mul + last_pos
        return (val - p_last.integral) / p_last.multiplier + p_last.pos;
    }

    // Between two points p1 and p2 where:
    // p2 => end of the segment (it)
    // p1 => start of the segment (it - 1, safe since it != begin)
    const CompensationPoint& p2 = *it;
    const CompensationPoint& p1 = *(--it);

    float range = p2.pos - p1.pos;
    float slope = (p2.multiplier - p1.multiplier) / range;

    // val = c1 + m1 * dx + 0.5 * slope * dx*dx
    // 0.5*slope*dx^2 + m1*dx + (c1 - val) = 0
    double A = 0.5 * slope;
    double B = p1.multiplier;
    double C = p1.integral - val;

    if(abs(A) < 1e-9) {
        // Linear: m1 * dx + C = 0 -> dx = -C / m1
        return p1.pos + (-C / B);
    } else {
        // Quadratic
        double delta = B*B - 4*A*C;
        if(delta < 0) delta = 0;
        double dx = (-B + sqrt(delta)) / (2*A);
        return p1.pos + dx;
    }
}

void PitchCompensation::do_compensation(float* target, bool inverse, bool debug) {
    if(!enable) return;

    // X axis
    if(axis_compensations.count('X')) {
        if(!inverse) {
            target[0] = integrate(axis_compensations['X'], target[0]);
        } else {
            target[0] = inverse_integrate(axis_compensations['X'], target[0]);
        }
    }

    // Y axis
    if(axis_compensations.count('Y')) {
        if(!inverse) {
            target[1] = integrate(axis_compensations['Y'], target[1]);
        } else {
            target[1] = inverse_integrate(axis_compensations['Y'], target[1]);
        }
    }

    // Z axis
    if(axis_compensations.count('Z')) {
        if(!inverse) {
            target[2] = integrate(axis_compensations['Z'], target[2]);
        } else {
            target[2] = inverse_integrate(axis_compensations['Z'], target[2]);
        }
    }

    if(debug) {
        THEKERNEL->streams->printf("//DEBUG: PitchComp NEW TARGET: %f, %f, %f\n", target[0], target[1], target[2]);
    }
}
