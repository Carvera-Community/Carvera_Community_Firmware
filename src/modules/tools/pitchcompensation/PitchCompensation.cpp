#include "PitchCompensation.h"
#include "libs/Kernel.h"
#include "libs/utils.h"
#include "Config.h"
#include "checksumm.h"
#include "modules/robot/Robot.h"
#include "modules/robot/Conveyor.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"
#include "Gcode.h"

#include <string>
#include <cmath>
#include <functional>
#include <algorithm>
#include <cstring>

#define PITCH_COMPENSATION_FILE "/sd/pitch_compensation.dat"
#define EPSILON 1e-6f

#define pitch_compensation_checksum CHECKSUM("pitch_compensation")
#define enable_checksum             CHECKSUM("enable")

using namespace std;

PitchCompensation::PitchCompensation() {
    this->enabled = false;
}

void PitchCompensation::on_module_loaded() {
    this->config_load();
    this->register_for_event(ON_GCODE_RECEIVED);
}

void PitchCompensation::config_load() {
    this->enabled = THEKERNEL->config->value(pitch_compensation_checksum, enable_checksum)->by_default(false)->as_bool();
    if (this->enabled) {
        load_points_from_file();
        update_compensation_transform();
    }
}

void PitchCompensation::on_gcode_received(void* argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if(gcode->has_m && gcode->m == 381) {
        const char axes[] = "XYZ";

        // M381: Disable pitch compensation
        // M381.1: Display current pitch compensation data
        // M381.2: Save current pitch compensation data
        // M381.3: Load pitch compensation data and enable compensation
        // M381.4: Delete compensation data for all axes and save
        // M381.5: Add point (ex: M381.5 X10 C1.00034)
        // M381.6: Remove point (ex: M381.6 X10)
        // M381.7: Remove all points for the given axes (ex: M381.7 X Y)
        if(gcode->subcode == 1) {
            // Display current pitch compensation data
            print_compensation_data(gcode->stream);
        } else if(gcode->subcode == 2) {
            // Save pitch compensation data
            save_points_to_file();
        } else if(gcode->subcode == 3) {
            // Load pitch compensation data and enable compensation
            THEKERNEL->conveyor->wait_for_idle();
            load_points_from_file();
            this->enabled = true;
            update_compensation_transform();
        } else if(gcode->subcode == 4) {
            // Delete pitch compensation data for all axes and save
            THEKERNEL->conveyor->wait_for_idle();
            for(char axis : axes) {
                clear_points(axis);
            }
            save_points_to_file();
        } else if(gcode->subcode == 5) {
            if (!gcode->has_letter('C')) {
                gcode->stream->printf("Pitch compensation: missing compensation value\n");
                return;
            }

            // Add point
            THEKERNEL->conveyor->wait_for_idle();
            for(char axis : axes) {
                if(gcode->has_letter(axis)) {
                    add_point(axis, gcode->get_value(axis), gcode->get_value('C'));
                }
            }
        } else if(gcode->subcode == 6) {
            // Remove point
            THEKERNEL->conveyor->wait_for_idle();
            for (char axis : axes) {
                if(gcode->has_letter(axis)) {
                    remove_point(axis, gcode->get_value(axis));
                }
            }
        } else if(gcode->subcode == 7) {
            // Remove all points
            THEKERNEL->conveyor->wait_for_idle();
            for(char axis : axes) {
                if (gcode->has_letter(axis)) {
                    clear_points(axis);
                }
            }
        } else {
            // Disable pitch compensation
            THEKERNEL->conveyor->wait_for_idle();
            this->enabled = false;
            update_compensation_transform();
            gcode->stream->printf("Pitch compensation disabled\n");
        }
    }
}

void PitchCompensation::print_compensation_data(StreamOutput *stream) {
    const char axes[] = "XYZ";
    for(char axis : axes) {
        if (!axis_compensations.count(axis)) continue;
        stream->printf("Pitch compensation data for %c:\n", axis);
        for(const CompensationPoint& cp : axis_compensations[axis].points) {
            stream->printf("  %f: %f\n", cp.pos, cp.multiplier);
        }
    }
}

void PitchCompensation::add_point(char axis, float pos, float multiplier) {
    if (multiplier < 0.5f || multiplier > 1.5f) {
        THEKERNEL->streams->printf("Pitch compensation: Multiplier %f out of range [0.5, 1.5]\n", multiplier);
        return;
    }

    AxisCompensation& axis_comp = axis_compensations[axis];

    // Remove the point if it already exists
    auto it = std::remove_if(axis_comp.points.begin(), axis_comp.points.end(),
        [pos](const CompensationPoint& cp) {
            return abs(cp.pos - pos) < EPSILON;
        });

    bool existed = (it != axis_comp.points.end());
    axis_comp.points.erase(it, axis_comp.points.end());

    // Add the new point
    axis_comp.points.push_back({pos, multiplier, 0.0});
    update_compensation_transform();

    if (existed) {
        THEKERNEL->streams->printf("Pitch compensation: updated point %c%f (compensation: %f)\n", axis, pos, multiplier);
    } else {
        THEKERNEL->streams->printf("Pitch compensation: added point %c%f (compensation: %f)\n", axis, pos, multiplier);
    }
}

void PitchCompensation::remove_point(char axis, float pos) {
    AxisCompensation& axis_comp = axis_compensations[axis];
    auto it = std::remove_if(axis_comp.points.begin(), axis_comp.points.end(),
        [pos](const CompensationPoint& cp) {
            return abs(cp.pos - pos) < EPSILON;
        });

    if (it != axis_comp.points.end()) {
        axis_comp.points.erase(it, axis_comp.points.end());
        update_compensation_transform();
        THEKERNEL->streams->printf("Pitch compensation: removed point %c%f\n", axis, pos);
    } else {
        THEKERNEL->streams->printf("Pitch compensation: point %c%f not found\n", axis, pos);
    }
}

void PitchCompensation::clear_points(char axis) {
    AxisCompensation& axis_comp = axis_compensations[axis];
    axis_comp.points.clear();
    update_compensation_transform();
    THEKERNEL->streams->printf("Pitch compensation: cleared points for %c\n", axis);
}

void PitchCompensation::save_points_to_file() {
    FILE *file = fopen(PITCH_COMPENSATION_FILE, "w");
    if(!file) {
        THEKERNEL->streams->printf("error: Failed to open pitch compensation file %s\n", PITCH_COMPENSATION_FILE);
        return;
    }

    const char axes[] = "XYZ";
    for(char axis : axes) {
        if (!axis_compensations.count(axis)) continue;
        for(const CompensationPoint& cp : axis_compensations[axis].points) {
            // Format: <axis> <position> <multiplier>
            // Example: X 10.5 1.00034
            int retval = fprintf(file, "%c %f %f\n", axis, cp.pos, cp.multiplier);
            if(retval == EOF) {
                THEKERNEL->streams->printf("Pitch compensation: Failed to write point %c%f %f to file (errno: %d)\n", axis, cp.pos, cp.multiplier, errno);
                fclose(file);
                return;
            }
        }
    }

    THEKERNEL->streams->printf("Pitch compensation data saved to %s\n", PITCH_COMPENSATION_FILE);
    fclose(file);
}

void PitchCompensation::load_points_from_file() {
    FILE *file = fopen(PITCH_COMPENSATION_FILE, "r");
    if(!file) {
        THEKERNEL->streams->printf("error: Failed to open pitch compensation file %s\n", PITCH_COMPENSATION_FILE);
        return;
    }

    // Clear existing points before loading
    const char axes[] = "XYZ";
    for(char axis : axes) {
        if(axis_compensations.count(axis)) {
            axis_compensations[axis].points.clear();
        }
    }

    int line_num = 0;
    int points_loaded = 0;
    char current_axis;
    float pos, multiplier;

    int read_result;
    while((read_result = fscanf(file, " %c %f %f", &current_axis, &pos, &multiplier)) != EOF) {
        line_num++;

        if(read_result != 3) {
            THEKERNEL->streams->printf("Pitch comp.: Failed to read line %d (needed 3 values, got %d), skipped\n", line_num, read_result);
            continue;
        }

        if(current_axis != 'X' && current_axis != 'Y' && current_axis != 'Z') {
            THEKERNEL->streams->printf("Pitch comp.: Invalid axis '%c' on line %d, skipped\n", current_axis, line_num);
            continue;
        }

        if(!isfinite(pos) || !isfinite(multiplier)) {
            THEKERNEL->streams->printf("Pitch comp.: Invalid values (pos=%f, mult=%f) on line %d, skipped\n", pos, multiplier, line_num);
            continue;
        }

        if(multiplier < 0.5f || multiplier > 1.5f) {
            THEKERNEL->streams->printf("Pitch comp.: Multiplier %f out of range [0.5, 1.5] on line %d, skipped\n", multiplier, line_num);
            continue;
        }

        // Add point to the axis compensation data
        AxisCompensation& axis_comp = axis_compensations[current_axis];
        axis_comp.points.push_back({pos, multiplier, 0.0});
        points_loaded++;
    }

    // Apply the compensation data
    update_compensation_transform();

    if(points_loaded > 0) {
        THEKERNEL->streams->printf("Pitch compensation: loaded %d points from %s\n", points_loaded, PITCH_COMPENSATION_FILE);
    } else {
        THEKERNEL->streams->printf("Pitch compensation: No valid point loaded from %s\n", PITCH_COMPENSATION_FILE);
    }

    fclose(file);
}

void PitchCompensation::update_compensation_transform() {
    if(this->enabled) {
        if (!THEROBOT->pitchCompensationTransform) {
            // Bind the compensation function
            using std::placeholders::_1;
            using std::placeholders::_2;
            using std::placeholders::_3;
            THEROBOT->pitchCompensationTransform = std::bind(&PitchCompensation::do_compensation, this, _1, _2, _3);
        }

        // Precompute the integrals for each axis
        const char axes[] = "XYZ";
        for(char axis : axes) {
            if (!axis_compensations.count(axis)) continue;

            std::sort(axis_compensations[axis].points.begin(), axis_compensations[axis].points.end());
            precompute_integrals(axis_compensations[axis]);
            axis_compensations[axis].points.shrink_to_fit();
        }
    } else {
        // Unbind the compensation function
        THEROBOT->pitchCompensationTransform = nullptr;
    }

    // Update current position to reflect the new compensation
    // Only applies if arm_solution is set to make sure that initialization is complete
    if(THEROBOT->arm_solution) {
        THEROBOT->reset_position_from_current_actuator_position();
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
    if(!enabled) return;

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
