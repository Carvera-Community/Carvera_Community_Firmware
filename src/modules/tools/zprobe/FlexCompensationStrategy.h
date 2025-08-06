#pragma once

#include "LevelingStrategy.h"

#include <string.h>
#include <tuple>

#define flex_compensation_strategy_checksum CHECKSUM("flex-compensation")

class StreamOutput;
class Gcode;

class FlexCompensationStrategy : public LevelingStrategy
{
public:
    FlexCompensationStrategy(ZProbe *zprobe);
    ~FlexCompensationStrategy();
    bool handleGcode(Gcode* gcode);
    bool handleConfig();

private:
    // Core functionality
    bool doMeasurement(Gcode *gc);
    void setAdjustFunction(bool on);
    void doCompensation(float *target, bool inverse, bool debug);
    void reset_compensation();
    
    // Helper functions
    void print_compensation_data(StreamOutput *stream);
    void save_compensation_data(StreamOutput *stream);
    bool load_compensation_data(StreamOutput *stream);
    
    // Configuration parameters
    float tolerance;
    std::string before_probe, after_probe;
    
    // Compensation data storage
    float *compensation_data;
    size_t data_size;
    bool compensation_active;
    
    // Measurement parameters
    float x_start;
    float x_size;
    uint8_t grid_x_size;
    uint8_t current_grid_x_size;
    
    // Configuration flags
    struct {
        bool save:1;
        bool human_readable:1;
    };
}; 