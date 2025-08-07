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
                __disable_irq();
                save_compensation_data(gcode->stream);
                __enable_irq();
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
        THEKERNEL->streams->printf("Compensation enabled\n");
    } else {
        // Clear it
        THEROBOT->compensationTransform = nullptr;
        compensation_active = false;
        THEKERNEL->streams->printf("Compensation disabled\n");
    }
}

bool FlexCompensationStrategy::doMeasurement(Gcode *gc)
{
    gc->stream->printf("Flex Compensation Measurement...\n");

    // Parse G33 parameters
    float y_coordinate = 0.0F;
    float x_distance = 0.0F;
    int num_points = 0;
    float max_delta = 0.0F;

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
        current_grid_x_size = num_points;
    } else {
        gc->stream->printf("ERROR: I parameter required for G33\n");
        return false;
    }

    if(x_distance <= 0 || num_points <= 0) {
        gc->stream->printf("ERROR: X and I parameters must be positive\n");
        return false;
    }

    // Validate that num_points matches grid_x_size
    if(num_points > grid_x_size) {
        gc->stream->printf("ERROR: I parameter (%d) must not be greater than grid_x_size (%d)\n", num_points, grid_x_size);
        return false;
    }

    // Get current machine position
    float current_x = THEROBOT->get_axis_position(X_AXIS);
    float current_y = THEROBOT->get_axis_position(Y_AXIS);
    float current_z = THEROBOT->get_axis_position(Z_AXIS);

    this->x_start = current_x;
    this->x_size = x_distance;

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

    zprobe->init_parameters_and_out_coords();
    // Set up probe parameters for Y-axis probing
    probe_parameters& params = zprobe->get_probe_parameters();
    params.y_axis_distance = y_coordinate;
    params.feed_rate = (gc->has_letter('F')) ? gc->get_value('F') : 600;
    params.rapid_rate = (gc->has_letter('R')) ? gc->get_value('R') : 800;

    // Probe at each point along X-axis
    for(int i = 0; i < num_points; i++) {
        float probe_x = current_x + (i * x_step);
        
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
        if (fabs(delta) > fabs(max_delta)) {
            max_delta = delta;
        }
        
        gc->stream->printf("Point %d: measured=%1.3f, delta=%1.3f\n", i, measured_y, delta);
    }

    for (int i = 0; i < grid_x_size; i++) {
        if (i < current_grid_x_size) {
            compensation_data[i] = delta_array[i];
        } else {
            compensation_data[i] = 0.0;
        }
        gc->stream->printf("Stored compensation_data[%d] = %1.6f\n", i, compensation_data[i]);
    }

    AHB0.dealloc(delta_array);

    // Store the delta array for later use
    // TODO: You can implement the rest of the logic here
    gc->stream->printf("Measurement completed. Delta array stored.\n");

    setAdjustFunction(true);

    THEROBOT->set_max_delta(max_delta);

    return true;
}

void FlexCompensationStrategy::doCompensation(float *target, bool inverse, bool debug)
{
    float triangle_y = 90.0;            // Y distance between the plane through both rods to the center of the spindle
    float machine_offset_z = 51.0;      // Z distance between the centerplane between the rods and the end of the spindle (AIR specific)
    float sensor_machine_z = -115.36;   // Z machine coordinate if the tool length would be 0
    float refmz = THEKERNEL->eeprom_data->REFMZ;                  
    float TLO = THEKERNEL->eeprom_data->TLO;
    float interpolated_delta = 0.0;

    float triangle_z = fabs(target[Z_AXIS]) + machine_offset_z + TLO + refmz - sensor_machine_z;
    THEKERNEL->streams->printf("x_size: %1.3f\n", x_size);

    // Check if target is within compensation range
    if (target[X_AXIS] < x_start || target[X_AXIS] > x_start + x_size) {
        return;
    }

    // Calculate grid spacing
    float grid_spacing = x_size / (current_grid_x_size - 1);
    
    // Find which grid segment the target falls into
    float relative_x = target[X_AXIS] - x_start;
    int grid_index = (int)(relative_x / grid_spacing);
    
    // Clamp grid_index to valid range
    if (grid_index >= current_grid_x_size - 1) {
        grid_index = current_grid_x_size - 2;  // Use last segment
    }
    if (grid_index < 0) {
        grid_index = 0;  // Use first segment
    }
    
    // Calculate interpolation factor (0.0 to 1.0)
    float grid_x_low = x_start + grid_index * grid_spacing;
    float grid_x_high = x_start + (grid_index + 1) * grid_spacing;
    float t = (target[X_AXIS] - grid_x_low) / (grid_x_high - grid_x_low);
    
    // Clamp t to [0, 1] range
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    
    // Linear interpolation between two grid points
    float delta_low = compensation_data[grid_index];
    float delta_high = compensation_data[grid_index + 1];
    interpolated_delta = delta_low + t * (delta_high - delta_low);
    
    if (inverse) {
        // Inverse operation: subtract the compensation instead of adding it
        target[Y_AXIS] -= interpolated_delta;
        // Inverse Z compensation: add back the compensation instead of subtracting
        target[Z_AXIS] += triangle_y / triangle_z * interpolated_delta + 0.5 * interpolated_delta;
    } else {
        // Normal operation: add the compensation
        target[Y_AXIS] += interpolated_delta;
        // rotational component + translation component
        target[Z_AXIS] -= triangle_y / triangle_z * interpolated_delta + 0.5 * interpolated_delta;
    }
    
    return;
}

void FlexCompensationStrategy::print_compensation_data(StreamOutput *stream)
{
    for (int i = 0; i < current_grid_x_size; i++) {
        stream->printf("%1.3f ", (i * (x_size / (current_grid_x_size - 1)) + x_start));
    }
    stream->printf("\n");
    for (int i = 0; i < current_grid_x_size; i++) {
        stream->printf("%1.3f ", compensation_data[i]);
    }
    stream->printf("\n");
    return;
}

void FlexCompensationStrategy::save_compensation_data(StreamOutput *stream)
{
    // Check if we have valid compensation data to save
    if(compensation_data == nullptr || current_grid_x_size == 0) {
        stream->printf("error: No compensation data to save\n");
        return;
    }

    if(current_grid_x_size > grid_x_size) {
        stream->printf("error: Invalid size\n");
        return;
    }

    bool has_valid_data = false;
    for(int i = 0; i < current_grid_x_size; i++) {
        if(!isnan(compensation_data[i])) {
            has_valid_data = true;
            break;
        }
    }
    
    if(!has_valid_data) {
        stream->printf("error: No valid compensation data to save\n");
        return;
    }

    FILE *fp = fopen(COMPENSATION_FILE, "w");
    if(fp == NULL) {
        stream->printf("error: Failed to open compensation file %s\n", COMPENSATION_FILE);
        return;
    }

    // Write x_start (float)
    if(fwrite(&x_start, sizeof(float), 1, fp) != 1) {
        stream->printf("error: Failed to write x_start\n");
        fclose(fp);
        return;
    }

    // Write current_grid_x_size (uint8_t)
    if(fwrite(&current_grid_x_size, sizeof(uint8_t), 1, fp) != 1) {
        stream->printf("error: Failed to write current_grid_x_size\n");
        fclose(fp);
        return;
    }

    // Write x_size (float)
    if(fwrite(&x_size, sizeof(float), 1, fp) != 1) {
        stream->printf("error: Failed to write x_size\n");
        fclose(fp);
        return;
    }

    // Write compensation data for the actual grid size used
    for(int i = 0; i < current_grid_x_size; i++) {
        if(fwrite(&compensation_data[i], sizeof(float), 1, fp) != 1) {
            stream->printf("error: Failed to write compensation data at index %d\n", i);
            fclose(fp);
            return;
        }
    }

    stream->printf("Compensation data saved to %s\n", COMPENSATION_FILE);
    stream->printf("Saved: x_start=%.3f, grid_size=%d, x_size=%.3f\n", 
                   x_start, current_grid_x_size, x_size);
    fclose(fp);
}

bool FlexCompensationStrategy::load_compensation_data(StreamOutput *stream)
{
    FILE *fp = fopen(COMPENSATION_FILE, "r");
    if(fp == NULL) {
        stream->printf("error: Failed to open compensation file %s\n", COMPENSATION_FILE);
        return false;
    }

    float load_x_start;
    uint8_t load_current_grid_x_size;
    float load_x_size;

    // Read x_start (float)
    if(fread(&load_x_start, sizeof(float), 1, fp) != 1) {
        stream->printf("error: Failed to read x_start\n");
        fclose(fp);
        return false;
    }

    // Read current_grid_x_size (uint8_t)
    if(fread(&load_current_grid_x_size, sizeof(uint8_t), 1, fp) != 1) {
        stream->printf("error: Failed to read current_grid_x_size\n");
        fclose(fp);
        return false;
    }

    // Validate grid size
    if(load_current_grid_x_size > grid_x_size) {
        stream->printf("error: Loaded grid size %d exceeds maximum configured size %d\n", 
                      load_current_grid_x_size, grid_x_size);
        fclose(fp);
        return false;
    }

    // Read x_size (float)
    if(fread(&load_x_size, sizeof(float), 1, fp) != 1) {
        stream->printf("error: Failed to read x_size\n");
        fclose(fp);
        return false;
    }


    // Reset compensation data to NAN
    reset_compensation();

    // Load compensation data for the actual grid size used
    for(int i = 0; i < load_current_grid_x_size; i++) {
        if(fread(&compensation_data[i], sizeof(float), 1, fp) != 1) {
            stream->printf("error: Failed to read compensation data at index %d\n", i);
            fclose(fp);
            return false;
        }
    }

    // Set the loaded values
    x_start = load_x_start;
    current_grid_x_size = load_current_grid_x_size;
    x_size = load_x_size;

    stream->printf("Compensation data loaded from %s\n", COMPENSATION_FILE);
    stream->printf("Loaded: x_start=%.3f, grid_size=%d, x_size=%.3f\n", 
                   x_start, current_grid_x_size, x_size);
    fclose(fp);
    return true;
}

void FlexCompensationStrategy::reset_compensation()
{
    for (int x = 0; x < grid_x_size; x++) {
        compensation_data[x] = NAN;
    }
    compensation_active = false;
} 