#ifndef PITCH_COMPENSATION_H
#define PITCH_COMPENSATION_H

#include "Module.h"
#include "StreamOutputPool.h"
#include <vector>
#include <map>

class PitchCompensation : public Module {
public:
    PitchCompensation();
    virtual ~PitchCompensation() {};

    void on_module_loaded();
    void on_gcode_received(void* argument);

    struct CompensationPoint {
        float pos; // Position of the point
        float multiplier; // Multiplier at this point
        double integral; // Cumulative integral up to this point

        bool operator<(const CompensationPoint& other) const {
            return pos < other.pos;
        }
    };

    struct AxisCompensation {
        std::vector<CompensationPoint> points;
    };

private:
    void config_load();
    void do_compensation(float* target, bool inverse, bool debug);
    void print_compensation_data(StreamOutput *stream);
    void add_point(char axis, float pos, float multiplier);
    void remove_point(char axis, float pos);
    void clear_points(char axis);
    void save_points_to_file();
    void load_points_from_file();
    void update_compensation_transform();
    void precompute_integrals(AxisCompensation& axis_comp);
    double integrate(const AxisCompensation& axis_comp, float pos);
    float inverse_integrate(const AxisCompensation& axis_comp, double val);

    std::map<char, AxisCompensation> axis_compensations;
    bool enabled;
};

#endif
