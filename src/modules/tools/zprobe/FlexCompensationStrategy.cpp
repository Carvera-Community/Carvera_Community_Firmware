/*
 * Custom Compensation Strategy
 * 
 * This strategy provides custom bed compensation using:
 * - G33: Perform measurement and enable compensation
 * - M380: Disable compensation
 * 
 * Configuration
 * -------------
 * The strategy must be enabled in the config as well as zprobe.
 * 
 * leveling-strategy.custom-compensation.enable         true
 * 
 * Grid size can be configured:
 * leveling-strategy.custom-compensation.grid_x_size    7
 * leveling-strategy.custom-compensation.grid_y_size    7
 * 
 * Bed dimensions:
 * leveling-strategy.custom-compensation.x_size         100
 * leveling-strategy.custom-compensation.y_size         90
 * 
 * Probe offsets:
 * leveling-strategy.custom-compensation.probe_offsets  0,0,0
 * 
 * Initial height for probing:
 * leveling-strategy.custom-compensation.initial_height 10
 * 
 * Before/after probe gcode:
 * leveling-strategy.custom-compensation.before_probe_gcode M280
 * leveling-strategy.custom-compensation.after_probe_gcode M281
 * 
 * Usage
 * -----
 * G33: Perform measurement and enable compensation
 * M380: Disable compensation
 * M380.1: Display current compensation data
 * M380.2: Save compensation data
 * M380.3: Load compensation data
 */

#include "FlexCompensationStrategy.h"

#include "Kernel.h"
#include "Config.h"
#include "Robot.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "Conveyor.h"
#include "ZProbe.h"
#include "nuts_bolts.h"
#include "utils.h"
#include "platform_memory.h"

#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <fastmath.h>
#include <functional>

#define grid_x_size_checksum         CHECKSUM("grid_x_size")
#define grid_y_size_checksum         CHECKSUM("grid_y_size")
#define tolerance_checksum           CHECKSUM("tolerance")
#define save_checksum                CHECKSUM("save")
#define probe_offsets_checksum       CHECKSUM("probe_offsets")
#define initial_height_checksum      CHECKSUM("initial_height")
#define x_size_checksum              CHECKSUM("x_size")
#define y_size_checksum              CHECKSUM("y_size")
#define do_home_checksum             CHECKSUM("do_home")
#define human_readable_checksum      CHECKSUM("human_readable")
#define before_probe_gcode_checksum  CHECKSUM("before_probe_gcode")
#define after_probe_gcode_checksum   CHECKSUM("after_probe_gcode")

#define COMPENSATION_FILE "/sd/flex_compensation.dat"

#define POS 1
#define NEG -1

FlexCompensationStrategy::FlexCompensationStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe)
{
    compensation_data = nullptr;
    compensation_active = false;
    data_size = 0;
}

FlexCompensationStrategy::~FlexCompensationStrategy()
{
    if(compensation_data != nullptr) AHB0.dealloc(compensation_data);
}

bool FlexCompensationStrategy::handleConfig()
{
    // Load configuration parameters
    this->grid_x_size = THEKERNEL->config->value(leveling_strategy_checksum, flex_compensation_strategy_checksum, grid_x_size_checksum)->by_default(30)->as_number();
    
    tolerance = THEKERNEL->config->value(leveling_strategy_checksum, flex_compensation_strategy_checksum, tolerance_checksum)->by_default(0.03F)->as_number();
    save = THEKERNEL->config->value(leveling_strategy_checksum, flex_compensation_strategy_checksum, save_checksum)->by_default(false)->as_bool();
    human_readable = THEKERNEL->config->value(leveling_strategy_checksum, flex_compensation_strategy_checksum, human_readable_checksum)->by_default(false)->as_bool();

    this->x_start = 0.0F;
    this->x_size = THEKERNEL->config->value(leveling_strategy_checksum, flex_compensation_strategy_checksum, x_size_checksum)->by_default(0.0F)->as_number();
    
    if (this->x_size == 0.0F) {
        THEKERNEL->streams->printf("Error: Invalid config, x_size must be defined\n");
        return false;
    }

    // Before/after probe gcode
    this->before_probe = THEKERNEL->config->value(leveling_strategy_checksum, flex_compensation_strategy_checksum, before_probe_gcode_checksum)->by_default("")->as_string();
    this->after_probe = THEKERNEL->config->value(leveling_strategy_checksum, flex_compensation_strategy_checksum, after_probe_gcode_checksum)->by_default("")->as_string();

    // Replace _ with space for gcode commands
    std::replace(before_probe.begin(), before_probe.end(), '_', ' ');
    std::replace(after_probe.begin(), after_probe.end(), '_', ' ');

    // Allocate memory for compensation data
    data_size = grid_x_size * sizeof(float);
    compensation_data = (float *)AHB0.alloc(data_size);

    if(compensation_data == nullptr) {
        THEKERNEL->streams->printf("Error: Not enough memory for compensation data\n");
        return false;
    }

    reset_compensation();

    return true;
}

bool FlexCompensationStrategy::handleGcode(Gcode *gcode)
{
    if(gcode->has_g) {
        if(gcode->g == 33) { // G33: Perform measurement and enable compensation
            THEKERNEL->streams->printf("G33: Perform measurement and enable compensation\n");
            // Wait for empty queue
            THEKERNEL->conveyor->wait_for_idle();

            // Execute before probe gcode
            if(!before_probe.empty()) {
                Gcode gc(before_probe, &(StreamOutput::NullStream));
                THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
            }

            THEROBOT->disable_segmentation = true;
            if(!doMeasurement(gcode)) {
                gcode->stream->printf("Measurement failed to complete, check the initial probe height and/or initial_height settings\n");
            } else {
                gcode->stream->printf("Measurement completed and compensation enabled.\n");
            }
            THEROBOT->disable_segmentation = false;

            // Execute after probe gcode
            if(!after_probe.empty()) {
                Gcode gc(after_probe, &(StreamOutput::NullStream));
                THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
            }

            return true;
        }
    } else if(gcode->has_m) {
        if(gcode->m == 380) { // M380: Disable compensation, M380.1: Display data, M380.2: Save, M380.3: Load
            if(gcode->subcode == 1) {
                // Display current compensation data
                print_compensation_data(gcode->stream);
            } else if(gcode->subcode == 2) {
                // Save compensation data
                save_compensation_data(gcode->stream);
            } else if(gcode->subcode == 3) {
                // Load compensation data
                if (load_compensation_data(gcode->stream)) {
                    setAdjustFunction(true);
                }
            } else {
                // Disable compensation
                setAdjustFunction(false);
                reset_compensation();
                gcode->stream->printf("Compensation disabled\n");
            }
            return true;
        }
    }

    return false;
}

void FlexCompensationStrategy::setAdjustFunction(bool on)
{
    if(on) {
        // Set the compensationTransform in robot
        using std::placeholders::_1;
        using std::placeholders::_2;
        using std::placeholders::_3;
        THEROBOT->compensationTransform = std::bind(&FlexCompensationStrategy::doCompensation, this, _1, _2, _3);
        compensation_active = true;
    } else {
        // Clear it
        THEROBOT->compensationTransform = nullptr;
        compensation_active = false;
    }
}

bool FlexCompensationStrategy::doMeasurement(Gcode *gc)
{
    gc->stream->printf("Flex Compensation Measurement...\n");

    // Parse G33 parameters
    float y_coordinate = 0.0F;
    float x_distance = 0.0F;
    int num_points = 0;

    if(gc->has_letter('Y')) {
        y_coordinate = gc->get_value('Y');
    } else {
        gc->stream->printf("ERROR: Y parameter required for G33\n");
        return false;
    }

    if(gc->has_letter('X')) {
        x_distance = gc->get_value('X');
    } else {
        gc->stream->printf("ERROR: X parameter required for G33\n");
        return false;
    }

    if(gc->has_letter('I')) {
        num_points = gc->get_value('I');
    } else {
        gc->stream->printf("ERROR: I parameter required for G33\n");
        return false;
    }

    if(x_distance <= 0 || num_points <= 0) {
        gc->stream->printf("ERROR: X and I parameters must be positive\n");
        return false;
    }

    // Get current machine position
    float current_x = THEROBOT->get_axis_position(X_AXIS);
    float current_y = THEROBOT->get_axis_position(Y_AXIS);
    float current_z = THEROBOT->get_axis_position(Z_AXIS);

    gc->stream->printf("Starting measurement at current position: X%1.3f Y%1.3f Z%1.3f\n", current_x, current_y, current_z);
    gc->stream->printf("Parameters: Y coordinate=%1.3f, X distance=%1.3f, Points=%d\n", y_coordinate, x_distance, num_points);

    // Allocate array for storing delta values
    float *delta_array = (float *)AHB0.alloc(num_points * sizeof(float));
    if(delta_array == nullptr) {
        gc->stream->printf("ERROR: Not enough memory for delta array\n");
        return false;
    }

    // Initialize array
    for(int i = 0; i < num_points; i++) {
        delta_array[i] = NAN;
    }

    // First measurement as reference
    float reference_y = 0.0F;

    // Calculate X step size
    float x_step = x_distance / (num_points - 1);
    // Probe at each point along X-axis
    for(int i = 0; i < num_points; i++) {
        float probe_x = current_x + (i * x_step);

        // Set up probe parameters for Y-axis probing
        probe_parameters& params = zprobe->get_probe_parameters();
        params.y_axis_distance = y_coordinate;
        params.feed_rate = (gc->has_letter('F')) ? gc->get_value('F') : 600;
        params.rapid_rate = (gc->has_letter('R')) ? gc->get_value('R') : 800;
        
        gc->stream->printf("Probing point %d: X%1.3f\n", i, probe_x);
        zprobe->coordinated_move(probe_x, NAN, NAN, params.rapid_rate / 60);
        
        
        
        // Use ZProbe's internal fast_slow_probe_sequence for Y-axis
        zprobe->fast_slow_probe_sequence_public(Y_AXIS, POS); // Probe in positive Y direction
        
        // Get the result from ZProbe's output coordinates
        xy_output_coordinates& coords = zprobe->get_output_coordinates();
        float measured_y = coords.y_positive_y_out;
        
        // Alternative: Use THEROBOT's get_last_probe_position
        // auto last_probe_pos = THEROBOT->get_last_probe_position();
        // float measured_y = std::get<Y_AXIS>(last_probe_pos);
        
        if(isnan(measured_y)) {
            gc->stream->printf("ERROR: Failed to probe at point %d\n", i);
            AHB0.dealloc(delta_array);
            return false;
        }
        
        if (i == 0){
            reference_y = measured_y;
        }
        gc->stream->printf("Reference Y value: %1.3f mm\n", reference_y);
        // Calculate delta from reference
        float delta = measured_y - reference_y;
        delta_array[i] = delta;
        
        gc->stream->printf("Point %d: measured=%1.3f, delta=%1.3f\n", i, measured_y, delta);
    }

    // Store the delta array for later use
    // TODO: You can implement the rest of the logic here
    gc->stream->printf("Measurement completed. Delta array stored.\n");
    
    // Clean up
    AHB0.dealloc(delta_array);

    return true;
}

void FlexCompensationStrategy::doCompensation(float *target, bool inverse, bool debug)
{
    // TODO: Implement your custom compensation algorithm here
    // This is a skeleton - you need to implement the actual compensation logic
    
    // Example: Simple linear interpolation (similar to CartGridStrategy)
    // You can replace this with your own algorithm
    
    return;
}

void FlexCompensationStrategy::print_compensation_data(StreamOutput *stream)
{
    return;
}

void FlexCompensationStrategy::save_compensation_data(StreamOutput *stream)
{
    /*
    if(isnan(compensation_data[0])) {
        stream->printf("error: No compensation data to save\n");
        return;
    }

    FILE *fp = fopen(COMPENSATION_FILE, "w");
    if(fp == NULL) {
        stream->printf("error: Failed to open compensation file %s\n", COMPENSATION_FILE);
        return;
    }

    if(fwrite(&grid_x_size, sizeof(uint8_t), 1, fp) != 1) {
        stream->printf("error: Failed to write grid x size\n");
        fclose(fp);
        return;
    }

    if(fwrite(&grid_y_size, sizeof(uint8_t), 1, fp) != 1) {
        stream->printf("error: Failed to write grid y size\n");
        fclose(fp);
        return;
    }

    if(fwrite(&x_size, sizeof(float), 1, fp) != 1) {
        stream->printf("error: Failed to write x_size\n");
        fclose(fp);
        return;
    }

    if(fwrite(&y_size, sizeof(float), 1, fp) != 1) {
        stream->printf("error: Failed to write y_size\n");
        fclose(fp);
        return;
    }

    for (int y = 0; y < grid_y_size; y++) {
        for (int x = 0; x < grid_x_size; x++) {
            if(fwrite(&compensation_data[x + (grid_x_size * y)], sizeof(float), 1, fp) != 1) {
                stream->printf("error: Failed to write compensation data\n");
                fclose(fp);
                return;
            }
        }
    }
    stream->printf("Compensation data saved to %s\n", COMPENSATION_FILE);
    fclose(fp);
    */
}

bool FlexCompensationStrategy::load_compensation_data(StreamOutput *stream)
{
    return true;
    /*
    FILE *fp = fopen(COMPENSATION_FILE, "r");
    if(fp == NULL) {
        stream->printf("error: Failed to open compensation file %s\n", COMPENSATION_FILE);
        return false;
    }

    uint8_t load_grid_x_size, load_grid_y_size;
    float x, y;

    if(fread(&load_grid_x_size, sizeof(uint8_t), 1, fp) != 1) {
        stream->printf("error: Failed to read grid x size\n");
        fclose(fp);
        return false;
    }

    if(load_grid_x_size != grid_x_size) {
        stream->printf("error: grid size x is different read %d - config %d\n", load_grid_x_size, grid_x_size);
        fclose(fp);
        return false;
    }

    if(fread(&load_grid_y_size, sizeof(uint8_t), 1, fp) != 1) {
        stream->printf("error: Failed to read grid y size\n");
        fclose(fp);
        return false;
    }

    if(load_grid_y_size != grid_y_size) {
        stream->printf("error: grid size y is different read %d - config %d\n", load_grid_y_size, grid_y_size);
        fclose(fp);
        return false;
    }

    if(fread(&x, sizeof(float), 1, fp) != 1) {
        stream->printf("error: Failed to read x_size\n");
        fclose(fp);
        return false;
    }

    if(fread(&y, sizeof(float), 1, fp) != 1) {
        stream->printf("error: Failed to read y_size\n");
        fclose(fp);
        return false;
    }

    if(x != x_size || y != y_size) {
        stream->printf("error: bed dimensions changed read (%f, %f) - config (%f,%f)\n", x, y, x_size, y_size);
        fclose(fp);
        return false;
    }

    for (int y = 0; y < grid_y_size; y++) {
        for (int x = 0; x < grid_x_size; x++) {
            if(fread(&compensation_data[x + (grid_x_size * y)], sizeof(float), 1, fp) != 1) {
                stream->printf("error: Failed to read compensation data\n");
                fclose(fp);
                return false;
            }
        }
    }
    stream->printf("Compensation data loaded, bed: (%f, %f), size: %d x %d\n", x_size, y_size, load_grid_x_size, load_grid_y_size);
    fclose(fp);
    return true;
    */
}

void FlexCompensationStrategy::reset_compensation()
{
    for (int x = 0; x < grid_x_size; x++) {
        compensation_data[x] = NAN;
    }
    compensation_active = false;
} 