/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ZProbe.h"

#include "Kernel.h"
#include "BaseSolution.h"
#include "Config.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "Conveyor.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "SlowTicker.h"
#include "Planner.h"
#include "SerialMessage.h"
#include "PublicDataRequest.h"
#include "EndstopsPublicAccess.h"
#include "ZProbePublicAccess.h"
#include "PublicData.h"
#include "LevelingStrategy.h"
#include "StepTicker.h"
#include "utils.h"
#include "us_ticker_api.h"
#include "ATCHandlerPublicAccess.h"
// strategies we know about
#include "DeltaCalibrationStrategy.h"
#include "ThreePointStrategy.h"
#include "DeltaGridStrategy.h"
#include "CartGridStrategy.h"

#include <vector>

#define enable_checksum          CHECKSUM("enable")
#define probe_pin_checksum       CHECKSUM("probe_pin")
#define calibrate_pin_checksum   CHECKSUM("calibrate_pin")
#define debounce_ms_checksum     CHECKSUM("debounce_ms")
#define slow_feedrate_checksum   CHECKSUM("slow_feedrate")
#define fast_feedrate_checksum   CHECKSUM("fast_feedrate")
#define return_feedrate_checksum CHECKSUM("return_feedrate")
#define probe_height_checksum    CHECKSUM("probe_height")
#define probe_tip_diameter_checksum CHECKSUM("probe_tip_diameter")
#define probe_calibration_safety_margin_checksum CHECKSUM("calibration_safety_margin")
#define toolZeroIs3Axis_checksum  CHECKSUM("tool_zero_is_3axis")
#define gamma_max_checksum       CHECKSUM("gamma_max")
#define max_z_checksum           CHECKSUM("max_z")
#define reverse_z_direction_checksum CHECKSUM("reverse_z")
#define dwell_before_probing_checksum CHECKSUM("dwell_before_probing")

// from endstop section
#define delta_homing_checksum    CHECKSUM("delta_homing")
#define rdelta_homing_checksum    CHECKSUM("rdelta_homing")

#define detector_switch_checksum    CHECKSUM("toolsensor")
#define switch_checksum 			CHECKSUM("switch")
#define state_checksum              CHECKSUM("state")
#define ignore_on_halt_checksum     CHECKSUM("ignore_on_halt")

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define XYZ 10

#define POS 1
#define NEG -1

#define STEPPER THEROBOT->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())
#define Z_STEPS_PER_MM STEPS_PER_MM(Z_AXIS)

constexpr double pi = 3.141592653589793;

void ZProbe::on_module_loaded()
{
    // if the module is disabled -> do nothing
    if(!THEKERNEL->config->value( zprobe_checksum, enable_checksum )->by_default(true)->as_bool()) {
        // as this module is not needed free up the resource
        delete this;
        return;
    }

    // load settings
    this->config_load();
    // register event-handlers
    register_for_event(ON_GCODE_RECEIVED);
    register_for_event(ON_GET_PUBLIC_DATA);
    register_for_event(ON_SET_PUBLIC_DATA);
    register_for_event(ON_MAIN_LOOP);

    this->probing_cycle = NONE;

    // we read the probe in this timer
    probing = false;
    calibrating = false;
    tlo_calibrating = false;
    THEKERNEL->slow_ticker->attach(1000, this, &ZProbe::read_probe);
    THEKERNEL->slow_ticker->attach(1000, this, &ZProbe::read_calibrate);
	if(CARVERA_AIR == THEKERNEL->factory_set->MachineModel)	//Manual Tool change 
	{
    	THEKERNEL->slow_ticker->attach(100, this, &ZProbe::probe_doubleHit);
    }
    this->probe_trigger_time = 0;
}

void ZProbe::config_load()
{
    this->pin.from_string( THEKERNEL->config->value(zprobe_checksum, probe_pin_checksum)->by_default("2.6v" )->as_string())->as_input();
    this->calibrate_pin.from_string( THEKERNEL->config->value(zprobe_checksum, calibrate_pin_checksum)->by_default("0.5^" )->as_string())->as_input();
    this->debounce_ms    = THEKERNEL->config->value(zprobe_checksum, debounce_ms_checksum)->by_default(0  )->as_number();
    this->probe_calibration_safety_margin = THEKERNEL->config->value(zprobe_checksum, probe_calibration_safety_margin_checksum)->by_default(0.1F)->as_number();
    this->halt_pending = false;
    this->probe_triggered = false;

    // get strategies to load
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list( &modules, leveling_strategy_checksum);
    for( auto cs : modules ){
        if( THEKERNEL->config->value(leveling_strategy_checksum, cs, enable_checksum )->as_bool() ){
            bool found= false;
            LevelingStrategy *ls= nullptr;

            // check with each known strategy and load it if it matches
            switch(cs) {
                case delta_calibration_strategy_checksum:
                    ls= new DeltaCalibrationStrategy(this);
                    found= true;
                    break;

                case three_point_leveling_strategy_checksum:
                    // NOTE this strategy is mutually exclusive with the delta calibration strategy
                    ls= new ThreePointStrategy(this);
                    found= true;
                    break;

                case delta_grid_leveling_strategy_checksum:
                    ls= new DeltaGridStrategy(this);
                    found= true;
                    break;

                case cart_grid_leveling_strategy_checksum:
                    ls= new CartGridStrategy(this);
                    found= true;
                    break;
            }
            if(found) {
                if(ls->handleConfig()) {
                    this->strategies.push_back(ls);
                }else{
                    delete ls;
                }
            }
        }
    }

    // need to know if we need to use delta kinematics for homing
    this->is_delta = THEKERNEL->config->value(delta_homing_checksum)->by_default(false)->as_bool();
    this->is_rdelta = THEKERNEL->config->value(rdelta_homing_checksum)->by_default(false)->as_bool();

    // default for backwards compatibility add DeltaCalibrationStrategy if a delta
    // may be deprecated
    if(this->strategies.empty()) {
        if(this->is_delta) {
            this->strategies.push_back(new DeltaCalibrationStrategy(this));
            this->strategies.back()->handleConfig();
        }
    }

    this->probe_height  = THEKERNEL->config->value(zprobe_checksum, probe_height_checksum)->by_default(5)->as_number();
    this->slow_feedrate = THEKERNEL->config->value(zprobe_checksum, slow_feedrate_checksum)->by_default(5)->as_number(); // feedrate in mm/sec
    this->fast_feedrate = THEKERNEL->config->value(zprobe_checksum, fast_feedrate_checksum)->by_default(100)->as_number(); // feedrate in mm/sec
    this->return_feedrate = THEKERNEL->config->value(zprobe_checksum, return_feedrate_checksum)->by_default(5)->as_number(); // feedrate in mm/sec
    this->reverse_z     = THEKERNEL->config->value(zprobe_checksum, reverse_z_direction_checksum)->by_default(false)->as_bool(); // Z probe moves in reverse direction
    this->max_z         = THEKERNEL->config->value(zprobe_checksum, max_z_checksum)->by_default(NAN)->as_number(); // maximum zprobe distance
    THEKERNEL->probe_tip_diameter = THEKERNEL->config->value(zprobe_checksum, probe_tip_diameter_checksum)->by_default(2)->as_number(); // probe tip diameter
    this->tool_0_3axis  = THEKERNEL->config->value(zprobe_checksum, toolZeroIs3Axis_checksum)->by_default(false)->as_bool();
    if(isnan(this->max_z)){
        this->max_z = THEKERNEL->config->value(gamma_max_checksum)->by_default(200)->as_number(); // maximum zprobe distance
    }
    this->dwell_before_probing = THEKERNEL->config->value(zprobe_checksum, dwell_before_probing_checksum)->by_default(0)->as_number(); // dwell time in seconds before probing

}

void ZProbe::on_main_loop(void *argument)
{
    // Handle deferred halt event from crash detection
    if (halt_pending) {
        halt_pending = false;
        THEKERNEL->call_event(ON_HALT, nullptr);
    }

    if (check_probe_tool() == 2){
        is_3dprobe_active = true;
        if (CARVERA_AIR == THEKERNEL->factory_set->MachineModel) {
            bool ignore_on_halt = true;
            PublicData::set_value( switch_checksum, detector_switch_checksum, ignore_on_halt_checksum, &ignore_on_halt );
            bool on = true;
            PublicData::set_value( switch_checksum, detector_switch_checksum, state_checksum, &on );
        }
    }else{
        if(is_3dprobe_active) {
            bool off = false;
            PublicData::set_value( switch_checksum, detector_switch_checksum, state_checksum, &off );
        }
        is_3dprobe_active = false;    
        if (CARVERA_AIR == THEKERNEL->factory_set->MachineModel) {
            bool ignore_on_halt = false;
            PublicData::set_value( switch_checksum, detector_switch_checksum, ignore_on_halt_checksum, &ignore_on_halt );
        }
    }
    
    
    switch(probing_cycle)
    {
        case CALIBRATE_PROBE_BORE:
            calibrate_probe_bore();
            probing_cycle = NONE;
            break;
        case CALIBRATE_PROBE_BOSS:
            calibrate_probe_boss();
            probing_cycle = NONE;
            break;
        case PROBE_BORE:
            probe_bore();
            probing_cycle = NONE;
            break;
        case PROBE_BOSS:
            probe_boss();
            probing_cycle = NONE;
            break;
        case PROBE_INSIDE_CORNER:
            probe_insideCorner();
            probing_cycle = NONE;
            break;
        case PROBE_OUTSIDE_CORNER:
            probe_outsideCorner();
            probing_cycle = NONE;
            break;
        case PROBE_AXIS_ANGLE:
            probe_axisangle();
            probing_cycle = NONE;
            break;
        case PROBE_A_AXIS:
            probe_axisangle(true, false);
            probing_cycle = NONE;
            break;
        case PROBE_A_AXIS_WITH_OFFSET:
            probe_axisangle(true, true);
            probing_cycle = NONE;
            break;
        case PROBE_SINGLE_AXIS_DOUBLE_TAP:
            single_axis_probe_double_tap();
            probing_cycle = NONE;
            break;
        case PROBE_SQUARE:
            probe_square();
            probing_cycle = NONE;
            break;
        default:
            break;
    }
}

uint32_t ZProbe::read_probe(uint32_t dummy)
{

    if (probe_triggered && !(this->pin.get() != invert_probe)) {
        probe_triggered = false;
    }

    if(STEPPER[X_AXIS]->is_moving() || STEPPER[Y_AXIS]->is_moving() || STEPPER[Z_AXIS]->is_moving()) {
        if (this->pin.get() != invert_probe && is_3dprobe_active && !probe_triggered) {
            probe_triggered = true;
            // Set halt state immediately for fast response, defer event processing to main loop
            // Disable crash detection during TLO calibration to prevent false positives
            if (!probing && !calibrating && !tlo_calibrating) {
                THEKERNEL->set_halt_reason(CRASH_DETECTED);
                THEKERNEL->set_halted(true);
                // Set a flag to process the halt event in the main loop
                halt_pending = true;
                THEKERNEL->streams->printf("error:3D Probe crash detected\r\n");
                THEKERNEL->streams->printf("Manually move the probe to a safe position\r\n");
            } 
        }
    }

    if (!probing) return 0;

    // we check all axis as it maybe a G38.2 X10 for instance, not just a probe in Z
    if(STEPPER[X_AXIS]->is_moving() || STEPPER[Y_AXIS]->is_moving() || STEPPER[Z_AXIS]->is_moving()) {
        // if it is moving then we check the probe, and debounce it
        if (this->pin.get() != invert_probe) {
            if (debounce < debounce_ms) {
                debounce ++;
                return 0;
            }
            
            if (!probe_detected) {
                probe_detected = true;
                probe_pin_position = STEPPER[Z_AXIS]->get_current_position();
            // if we are calibrating, the stop to the actuators comes from the read_calibrate method
            } else if (!calibrating) {
                // we signal the motors to stop, which will preempt any moves on that axis
                // we do all motors as it may be a delta
                for (auto &a : THEROBOT->actuators) a->stop_moving();
                debounce = 0;
            }
        } else {
            // The endstop was not hit yet
            debounce = 0;
        }
    }

    return 0;
}

uint32_t ZProbe::read_calibrate(uint32_t dummy)
{
    if (!calibrating) return 0;

    // just check z Axis move
    if (STEPPER[Z_AXIS]->is_moving()) {
        // if it is moving then we check the probe, and debounce it
        if (this->calibrate_pin.get()) {
            if (cali_debounce < debounce_ms) {
                cali_debounce++;
                return 0;       
            }
            
            if (!calibrate_detected) {
                // Record that the calibration pin is on, and at what position
                // we detected this.
                calibrate_detected = true;
                calibrate_pin_position = STEPPER[Z_AXIS]->get_current_position();
            }

            if (!probing || probe_detected) {
                // if we are not probing, e.g. doing a regular TLO calibration,
                // or we are probing and the probe was detected we signal the
                // motors to stop, which will preempt any moves on that axis we
                // do all motors as it may be a delta
                for (auto &a : THEROBOT->actuators) a->stop_moving();
                cali_debounce = 0;
            } else {
                // We have a probe tool; we must make sure we don't move too far.
                // Store the current Z position for later reporting if necessary.
                calibrate_current_z = STEPPER[Z_AXIS]->get_current_position();
                distance_moved = fabs(calibrate_current_z - calibrate_pin_position);
                // If we've exceeded the calibration distance, set PROBE_FAIL.
                // The error will be reported in calibrate_Z.
                if (distance_moved > probe_calibration_safety_margin) {
                    safety_margin_exceeded = true;
                    for (auto &a : THEROBOT->actuators) a->stop_moving();                    
                }
            }
        } else {
            // The endstop was not hit yet
            cali_debounce = 0;
        } 
    }
    return 0;
}

uint32_t ZProbe::probe_doubleHit(uint32_t dummy)
{

    if (is_3dprobe_active) return 0;

	if (this->pin.get()) 
	{
		if(!bfirstHitDetected)
		{
			bfirstHitDetected = true;
			probe_hit_time = us_ticker_read();
		}
		else if( bNoHited && (us_ticker_read() - probe_hit_time < 500000) )
		{
			if(bDoubleHited == false)
			{
				THEKERNEL->set_probeLaser(true);
				bool b = true;
			    PublicData::set_value( switch_checksum, detector_switch_checksum, state_checksum, &b );		
			    bDoubleHited = true;
			}
			else
			{
				THEKERNEL->set_probeLaser(false);
				bool b = false;
			    PublicData::set_value( switch_checksum, detector_switch_checksum, state_checksum, &b );		
			    bDoubleHited = false;
			}
		}
		bNoHited = false;
	}
	else
	{
		if(bfirstHitDetected)
			bNoHited = true;
		if(us_ticker_read() - probe_hit_time > 500000)
		{
			bfirstHitDetected = false;
		}
	}


    return 0;
}

bool ZProbe::check_last_probe_ok(){
    float px, py, pz;
    uint8_t ps;
    std::tie(px, py, pz, ps) = THEROBOT->get_last_probe_position();
    if (ps == 1) {
        return true;
    }
    return false;
}

// single probe in Z with custom feedrate
// returns boolean value indicating if probe was triggered
bool ZProbe::run_probe(float& mm, float feedrate, float max_dist, bool reverse)
{
    if(dwell_before_probing > .0001F) safe_delay_ms(dwell_before_probing*1000);

    if(this->pin.get()) {
    	THEKERNEL->streams->printf("Error: Probe already triggered so aborts\r\n");
        // probe already triggered so abort
        return false;
    }
    float maxz= max_dist < 0 ? this->max_z*2 : max_dist;

    probing = true;
    calibrating = false;
    probe_detected = false;
    debounce = 0;
    cali_debounce = 0;

    reset_probe_tracking();

    // save current actuator position so we can report how far we moved
    float z_start_pos= THEROBOT->actuators[Z_AXIS]->get_current_position();

    // move Z down
    bool dir= (!reverse_z != reverse); // xor
    float delta[3]= {0,0,0};
    delta[Z_AXIS]= dir ? -maxz : maxz;
    THEKERNEL->set_zprobing(true);
    THEROBOT->delta_move(delta, feedrate, 3);
    THEKERNEL->set_zprobing(false);

    // wait until finished
    THECONVEYOR->wait_for_idle();
    if(THEKERNEL->is_halted()) return false;

    // now see how far we moved, get delta in z we moved
    // NOTE this works for deltas as well as all three actuators move the same amount in Z
    mm = z_start_pos - THEROBOT->actuators[2]->get_current_position();

    // set the last probe position to the actuator units moved during this home
    THEROBOT->set_last_probe_position(std::make_tuple(0, 0, mm, probe_detected ? 1:0));

    probing= false;

    if(probe_detected) {
        // if the probe stopped the move we need to correct the last_milestone as it did not reach where it thought
        THEROBOT->reset_position_from_current_actuator_position();
    }

    return probe_detected;
}

// do probe then return to start position
bool ZProbe::run_probe_return(float& mm, float feedrate, float max_dist, bool reverse)
{
    float save_z_pos= THEROBOT->get_axis_position(Z_AXIS);

    bool ok= run_probe(mm, feedrate, max_dist, reverse);

    // move probe back to where it was
    float fr;
    if(this->return_feedrate != 0) { // use return_feedrate if set
        fr = this->return_feedrate;
    } else {
        fr = this->slow_feedrate*2; // nominally twice slow feedrate
        if(fr > this->fast_feedrate) fr = this->fast_feedrate; // unless that is greater than fast feedrate
    }

    // absolute move back to saved starting position
    coordinated_move(NAN, NAN, save_z_pos, fr, false);

    return ok;
}

bool ZProbe::doProbeAt(float &mm, float x, float y)
{
    // move to xy
    coordinated_move(x, y, NAN, getFastFeedrate() * 4);
    return run_probe_return(mm, slow_feedrate);
}

void ZProbe::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if( gcode->has_g && gcode->g >= 29 && gcode->g <= 33) {

        invert_probe = false;
        // make sure the probe is defined and not already triggered before moving motors
        if(!this->pin.connected()) {
            gcode->stream->printf("ZProbe pin not configured.\n");
            return;
        }

        // first wait for all moves to finish
        THEKERNEL->conveyor->wait_for_idle();

        if(this->pin.get()) {
            gcode->stream->printf("ZProbe triggered before move, aborting command.\n");
            return;
        }

        if( gcode->g == 30 ) { // simple Z probe

            bool set_z= (gcode->has_letter('Z') && !is_rdelta);
            bool probe_result;
            bool reverse= (gcode->has_letter('R') && gcode->get_value('R') != 0); // specify to probe in reverse direction
            float rate= gcode->has_letter('F') ? gcode->get_value('F') / 60 : this->slow_feedrate;
            float mm;

            // if not setting Z then return probe to where it started, otherwise leave it where it is
            probe_result = (set_z ? run_probe(mm, rate, -1, reverse) : run_probe_return(mm, rate, -1, reverse));

            if(probe_result) {
                // the result is in actuator coordinates moved
                gcode->stream->printf("Z:%1.4f\n", THEKERNEL->robot->from_millimeters(mm));

                if(set_z) {
                    // set current Z to the specified value, shortcut for G92 Znnn
                    char buf[32];
                    int n = snprintf(buf, sizeof(buf), "G92 Z%f", gcode->get_value('Z'));
                    string g(buf, n);
                    Gcode gc(g, &(StreamOutput::NullStream));
                    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
                }

            } else {
                gcode->stream->printf("ZProbe not triggered\n");
            }

        } else {
            if(!gcode->has_letter('P')) {
                // find the first strategy to handle the gcode
                for(auto s : strategies){
                    if(s->handleGcode(gcode)) {
                        return;
                    }
                }
                gcode->stream->printf("No strategy found to handle G%d\n", gcode->g);

            }else{
                // P paramater selects which strategy to send the code to
                // they are loaded in the order they are defined in config, 0 being the first, 1 being the second and so on.
                uint16_t i= gcode->get_value('P');
                if(i < strategies.size()) {
                    if(!strategies[i]->handleGcode(gcode)){
                        gcode->stream->printf("strategy #%d did not handle G%d\n", i, gcode->g);
                    }
                    return;

                }else{
                    gcode->stream->printf("strategy #%d is not loaded\n", i);
                }
            }
        }

    } else if(gcode->has_g && gcode->g == 38 ) { // G38.2 Straight Probe with error, G38.3 straight probe without error
        // linuxcnc/grbl style probe http://www.linuxcnc.org/docs/2.5/html/gcode/gcode.html#sec:G38-probe
        if(gcode->subcode < 2 || gcode->subcode > 6) {
            gcode->stream->printf("Error :Only G38.2 to G38.5 are supported\n");
            return;
        }

        // make sure the probe is defined and not already triggered before moving motors
        if(!this->pin.connected()) {
            gcode->stream->printf("Error :ZProbe not connected.\n");
            return;
        }

        if (gcode->subcode == 4 || gcode->subcode == 5) {
            invert_probe = true;
        } else {
            invert_probe = false;
        }

        if (gcode->subcode == 6) {
            calibrate_Z(gcode);
        } else {
            probe_XYZ(gcode);
        }

        invert_probe = false;

        return;

    } else if(gcode->has_m) {
        // M code processing here
        int c;
        switch (gcode->m) {
            case 119:
                c = this->pin.get();
                gcode->stream->printf(" Probe: %d", c);
                gcode->add_nl = true;
                break;

            case 460:
                if (gcode->subcode == 3) { //calibrate using anchor . Moved to atchandler for cleanliness
                    parse_parameters(gcode);
                } else if (gcode->subcode == 2){//calibrate using boss
                    if (!gcode->has_letter('X') && !gcode->has_letter('Y')){ //error if there is a problem
                        gcode->stream->printf("ALARM: Probe fail: No Gague Length\n");
                        THEKERNEL->call_event(ON_HALT, nullptr);
                        THEKERNEL->set_halt_reason(PROBE_FAIL);
                        return;
                    }
                
                    if (gcode->has_letter('X') && gcode->has_letter('Y')){
                        gcode->stream->printf("ALARM: Probe fail: Multiple Axes Given When 1 Expected\n");
                        THEKERNEL->call_event(ON_HALT, nullptr);
                        THEKERNEL->set_halt_reason(PROBE_FAIL);
                        return;
                    }
                    if (parse_parameters(gcode)){
                        probing_cycle = CALIBRATE_PROBE_BOSS;
                    }
                }else {
                    if (!gcode->has_letter('X') && !gcode->has_letter('Y') ) { //error if there is a problem
                        gcode->stream->printf("ALARM: Probe fail: No Radius Given\n");
                        THEKERNEL->call_event(ON_HALT, nullptr);
                        THEKERNEL->set_halt_reason(PROBE_FAIL);
                        return;
                    }
                    if (parse_parameters(gcode)){
                        probing_cycle = CALIBRATE_PROBE_BORE;
                    }
                }
                
                break;
            case 461:
                if (!gcode->has_letter('X') && !gcode->has_letter('Y')){ //error if there is a problem
                    gcode->stream->printf("ALARM: Probe fail: No Axis Set\n");
                    THEKERNEL->call_event(ON_HALT, nullptr);
                    THEKERNEL->set_halt_reason(PROBE_FAIL);
                    return;
                }
                if (parse_parameters(gcode)){
                    probing_cycle = PROBE_BORE;
                }
                break;
            case 462:
                if (!gcode->has_letter('X') && !gcode->has_letter('Y')){ //error if there is a problem
                    gcode->stream->printf("ALARM: Probe fail: No Axis Set\n");
                    THEKERNEL->call_event(ON_HALT, nullptr);
                    THEKERNEL->set_halt_reason(PROBE_FAIL);
                    return;
                }
                if (parse_parameters(gcode)){
                    probing_cycle = PROBE_BOSS;
                }
                break;
            case 463:
                if (!gcode->has_letter('X') || !gcode->has_letter('Y')){
                    gcode->stream->printf("ALARM: Probe fail: Both X and Y axis need to be set for Corner Probing\n");
                    THEKERNEL->call_event(ON_HALT, nullptr);
                    THEKERNEL->set_halt_reason(PROBE_FAIL);
                    return;
                }
                if (parse_parameters(gcode)){
                    probing_cycle = PROBE_INSIDE_CORNER;
                }
                break;
            case 464:
                if (!gcode->has_letter('X') || !gcode->has_letter('Y')){
                    gcode->stream->printf("ALARM: Probe fail: Both X and Y axis need to be set for Corner Probing\n");
                    THEKERNEL->call_event(ON_HALT, nullptr);
                    THEKERNEL->set_halt_reason(PROBE_FAIL);
                    return;
                }
                if (parse_parameters(gcode)){
                    probing_cycle = PROBE_OUTSIDE_CORNER;
                }
                break;
            case 465:
                parse_parameters(gcode, true);
                if (gcode->subcode == 1){
                    if (!gcode->has_letter('Y') || !gcode->has_letter('H')){
                        gcode->stream->printf("ALARM: Probe fail: No distance or height set\n");
                        THEKERNEL->call_event(ON_HALT, nullptr);
                        THEKERNEL->set_halt_reason(PROBE_FAIL);
                        return;
                    }
                    probing_cycle = PROBE_A_AXIS;
                }else if (gcode->subcode == 2){
                    if (!gcode->has_letter('X') || !gcode->has_letter('Y') || !gcode->has_letter('R')){
                        gcode->stream->printf("ALARM: Probe fail: No offset, distance or height set\n");
                        THEKERNEL->call_event(ON_HALT, nullptr);
                        THEKERNEL->set_halt_reason(PROBE_FAIL);
                        return;
                    }
                    probing_cycle = PROBE_A_AXIS_WITH_OFFSET;
                }else{
                    if (!gcode->has_letter('X') && !gcode->has_letter('Y')){
                            gcode->stream->printf("ALARM: Probe fail: No axis set\n");
                        THEKERNEL->call_event(ON_HALT, nullptr);
                        THEKERNEL->set_halt_reason(PROBE_FAIL);
                        return;
                    }
                    if (gcode->has_letter('X') && gcode->has_letter('Y')){
                        gcode->stream->printf("ALARM: Probe fail: Axis probing only supports 1 axis input\n");
                        THEKERNEL->call_event(ON_HALT, nullptr);
                        THEKERNEL->set_halt_reason(PROBE_FAIL);
                        return;
                    }
                    probing_cycle = PROBE_AXIS_ANGLE;
                }
                break;
            case 466:
                if (gcode->subcode == 1){
                    if (!gcode->has_letter('X') || !gcode->has_letter('Y') || !gcode->has_letter('H')){
                        gcode->stream->printf("ALARM: Probe fail: Both X and Y axis and height need to be set for Square Probing\n");
                        THEKERNEL->call_event(ON_HALT, nullptr);
                        THEKERNEL->set_halt_reason(PROBE_FAIL);
                        return;
                    }
                    if (parse_parameters(gcode)){
                        probing_cycle = PROBE_SQUARE;
                    }
                }else{
                    if (!gcode->has_letter('X') && !gcode->has_letter('Y') && !gcode->has_letter('Z')){
                        gcode->stream->printf("ALARM: Probe fail: No Axis Set\n");
                        THEKERNEL->call_event(ON_HALT, nullptr);
                        THEKERNEL->set_halt_reason(PROBE_FAIL);
                        return;
                    }
                    if (parse_parameters(gcode, (gcode->has_letter('Z') && !gcode->has_letter('X') && !gcode->has_letter('Y')))){
                        probing_cycle = PROBE_SINGLE_AXIS_DOUBLE_TAP;
                    }
                }
                break;                
            case 670:
                if (gcode->has_letter('S')) this->slow_feedrate = gcode->get_value('S');
                if (gcode->has_letter('K')) this->fast_feedrate = gcode->get_value('K');
                if (gcode->has_letter('R')) this->return_feedrate = gcode->get_value('R');
                if (gcode->has_letter('Z')) this->max_z = gcode->get_value('Z');
                if (gcode->has_letter('H')) this->probe_height = gcode->get_value('H');
                if (gcode->has_letter('I')) { // NOTE this is temporary and toggles the invertion status of the pin
                    invert_override= (gcode->get_value('I') != 0);
                    pin.set_inverting(pin.is_inverting() != invert_override); // XOR so inverted pin is not inverted and vice versa
                    gcode->stream->printf("// Invert override set: %d\n", pin.is_inverting());
                }
                if (gcode->has_letter('D')) this->dwell_before_probing = gcode->get_value('D');
                break;

            case 500: // save settings
            case 503: // print settings
                gcode->stream->printf(";Probe feedrates Slow/fast(K)/Return (mm/sec) max_z (mm) height (mm) dwell (s):\nM670 S%1.2f K%1.2f R%1.2f Z%1.2f H%1.2f D%1.2f\n",
                    this->slow_feedrate, this->fast_feedrate, this->return_feedrate, this->max_z, this->probe_height, this->dwell_before_probing);

                // fall through is intended so leveling strategies can handle m-codes too

            default:
                for(auto s : strategies){
                    if(s->handleGcode(gcode)) {
                        return;
                    }
                }
        }
    }
}

void ZProbe::reset_probe_tracking() {
    safety_margin_exceeded = false;
    calibrate_pin_position = 0.0F;
    probe_pin_position = 0.0F;
    calibrate_current_z = 0.0F;
}

// special way to probe in the X or Y or Z direction using planned moves, should work with any kinematics
bool ZProbe::probe_XYZ(Gcode *gcode)
{
    float x= 0, y= 0, z= 0;
    if(gcode->has_letter('X')) {
        x= gcode->get_value('X');
    }

    if(gcode->has_letter('Y')) {
        y= gcode->get_value('Y');
    }

    if(gcode->has_letter('Z')) {
        z= gcode->get_value('Z');
    }

    // Apply wcs rotation for G38
    rotateXY(x, y, &x, &y, THEROBOT->r[THEROBOT->get_current_wcs()]);

    if(x == 0 && y == 0 && z == 0) {
        gcode->stream->printf("error:at least one of X Y or Z must be specified, and be > or < 0\n");
        return false;
    }

    // get probe feedrate in mm/min and convert to mm/sec if specified
    float rate = (gcode->has_letter('F')) ? gcode->get_value('F')/60 : this->slow_feedrate;

    // first wait for all moves to finish
    THEKERNEL->conveyor->wait_for_idle();

    if(this->pin.get() != invert_probe) {
        gcode->stream->printf("Error:ZProbe triggered before move, aborting command.\n");
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return false;
    }
    
    probing = true;
    probe_detected = false;
    calibrating = false;
    debounce = 0;
    cali_debounce = 0;

    reset_probe_tracking();    

    // do a delta move which will stop as soon as the probe is triggered, or the distance is reached
    float delta[3]= {x, y, z};
    THEKERNEL->set_zprobing(true);
    if(!THEROBOT->delta_move(delta, rate, 3)) {
    	gcode->stream->printf("ERROR: Move too small,  %1.3f, %1.3f, %1.3f\n", x, y, z);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        THEKERNEL->call_event(ON_HALT, nullptr);
        probing = false;
        THEKERNEL->set_zprobing(false);
        return false;
    }
    THEKERNEL->set_zprobing(false);

    THEKERNEL->conveyor->wait_for_idle();

    // disable probe checking
    probing = false;

    // if the probe stopped the move we need to correct the last_milestone as it did not reach where it thought
    // this also sets last_milestone to the machine coordinates it stopped at
    THEROBOT->reset_position_from_current_actuator_position();
    float pos[3];
    THEROBOT->get_axis_position(pos, 3);

    if(THEKERNEL->is_flex_compensation_active()) {
        if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(pos, true, false); // get inverse compensation transform
    }

    uint8_t probeok = probe_detected ? 1 : 0;

    // print results using the GRBL format
    gcode->stream->printf("[PRB:%1.3f,%1.3f,%1.3f:%d]\n", THEKERNEL->robot->from_millimeters(pos[X_AXIS]), THEKERNEL->robot->from_millimeters(pos[Y_AXIS]), THEKERNEL->robot->from_millimeters(pos[Z_AXIS]), probeok);
    THEROBOT->set_last_probe_position(std::make_tuple(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], probeok));

    if(probeok == 0 && (gcode->subcode == 2 || gcode->subcode == 4)) {
        // issue error if probe was not triggered and subcode is 2 or 4
        gcode->stream->printf("ALARM: Probe fail\n");
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return false; //probe was not activated but failed due to subcodes
    }
    if(probeok == 0){
        return false; // probe was not activated but did not fail
    }
    return true; //probe was activated
}

uint8_t ZProbe::check_probe_tool() {
    struct tool_status tool;
    bool ok = PublicData::get_value( atc_handler_checksum, get_tool_status_checksum, &tool );
    if (!ok) {
        return 0;
    }
    
    // 3d probe tool
    if ((tool.active_tool == 0 && this->tool_0_3axis) || tool.active_tool >= 999990){
        return 2;
    // probe tool in general
    }else if (tool.active_tool == 0 || tool.active_tool >= 999990){
        return 1;
    }
    return 0;
}

void ZProbe::set_tlo_calibrating(bool state) {
    tlo_calibrating = state;
}

void ZProbe::on_set_public_data(void* argument) {
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);
    
    if(!pdr->starts_with(zprobe_checksum)) return;
    
    if(pdr->second_element_is(set_tlo_calibrating_checksum)) {
        bool *state = static_cast<bool*>(pdr->get_data_ptr());
        this->set_tlo_calibrating(*state);
        pdr->set_taken();
    }
}

// just probe / calibrate Z using calibrate pin
void ZProbe::calibrate_Z(Gcode *gcode)
{
    float z= 0;
    if(gcode->has_letter('Z')) {
        z= gcode->get_value('Z');
    }

    if(z == 0) {
        gcode->stream->printf("error: Z must be specified, and be > or < 0\n");
        return;
    }

    // get probe feedrate in mm/min and convert to mm/sec if specified
    float rate = (gcode->has_letter('F')) ? gcode->get_value('F') / 60 : this->slow_feedrate;

    // first wait for all moves to finish
    THEKERNEL->conveyor->wait_for_idle();

    if (this->calibrate_pin.get()) {
        gcode->stream->printf("error: ZCalibrate triggered before move, aborting command.\n");
        return;
    }

    probing = false;
    calibrating = true;
    probe_detected = false;
    calibrate_detected = false;
    debounce = 0;
    cali_debounce = 0;

    reset_probe_tracking();

    // If calibration is happening with a probe tool, enable tracking of probe position in the read_probe ISR.
    if (check_probe_tool() > 0) {
        probing = true;
    }

    // do a delta move which will stop as soon as the probe is triggered, or the distance is reached
    float delta[3]= {0, 0, z};
    THEKERNEL->set_zprobing(true);
    if(!THEROBOT->delta_move(delta, rate, 3)) {
        gcode->stream->printf("ERROR: Move too small,  %1.3f\n", z);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        THEKERNEL->call_event(ON_HALT, nullptr);
        calibrating = false;
        THEKERNEL->set_zprobing(false);
        return;
    }
    THEKERNEL->set_zprobing(false);

    THEKERNEL->conveyor->wait_for_idle();

    // disable calibrate and probe tracking
    calibrating = false;
    probing = false;

    // if the probe stopped the move we need to correct the last_milestone as it did not reach where it thought
    // this also sets last_milestone to the machine coordinates it stopped at
    THEROBOT->reset_position_from_current_actuator_position();
    float pos[3];
    THEROBOT->get_axis_position(pos, 3);

    if(THEKERNEL->is_flex_compensation_active()) {
        if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(pos, true, false); // get inverse compensation transform
    }
    
    if (safety_margin_exceeded) {
        safety_margin_exceeded = false;
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        THEKERNEL->call_event(ON_HALT, nullptr);
        gcode->stream->printf("ALARM: Probe failed to trigger within safety margin (%.2fmm)\n", 
                             this->probe_calibration_safety_margin);
        gcode->stream->printf("Distance moved: %.3f\n", distance_moved);
        gcode->stream->printf("Probe pin triggered: %d, position: %.3f\n", probe_detected, probe_pin_position);
        gcode->stream->printf("Calibrate pin triggered: %d, position: %.3f\n", calibrate_detected, calibrate_pin_position);
        gcode->stream->printf("Current position: %.3f\n", THEKERNEL->robot->from_millimeters(pos[Z_AXIS]));
        gcode->stream->printf("Error detected at position: %.3f\n", calibrate_current_z);
        gcode->stream->printf("Safety Margin Value: %.3f\n",  probe_calibration_safety_margin);
        gcode->stream->printf("debounce: %d, cali_debounce: %d, debounce_ms: %d\n", debounce, cali_debounce, debounce_ms);
        return;
    }

    if (probe_detected && calibrate_detected) {
        float offset = probe_pin_position - calibrate_pin_position;
        gcode->stream->printf("Probe trigger offset: %.3fmm (probe Z:%.3f, cal Z:%.3f)\n",
                             offset,
                             probe_pin_position,
                             calibrate_pin_position);
    }
    
    uint8_t calibrateok = calibrate_detected ? 1 : 0;

    // print results using the GRBL format
    gcode->stream->printf("[PRB:%1.3f,%1.3f,%1.3f:%d]\n", 
        THEKERNEL->robot->from_millimeters(pos[X_AXIS]), 
        THEKERNEL->robot->from_millimeters(pos[Y_AXIS]), 
        THEKERNEL->robot->from_millimeters(pos[Z_AXIS]), 
        calibrateok);
    THEROBOT->set_last_probe_position(std::make_tuple(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], calibrateok));

    if (calibrateok == 0) {
        // issue error if probe was not triggered and subcode is 2 or 4
        gcode->stream->printf("ALARM: Calibrate fail!\n");
        THEKERNEL->set_halt_reason(CALIBRATE_FAIL);
        THEKERNEL->call_event(ON_HALT, nullptr);
    }

    if (probe_detected) {
    	this->probe_trigger_time = us_ticker_read();
    }

}

// issue a coordinated move directly to robot, and return when done
// Only move the coordinates that are passed in as not nan
// NOTE must use G53 to force move in machine coordinates and ignore any WCS offsets
void ZProbe::coordinated_move(float x, float y, float z, float feedrate, bool relative)
{
    #define CMDLEN 128
    char *cmd= new char[CMDLEN]; // use heap here to reduce stack usage

    if(relative) strcpy(cmd, "G91 G0 ");
    else strcpy(cmd, "G53 G0 "); // G53 forces movement in machine coordinate system

    if(!isnan(x)) {
        size_t n= strlen(cmd);
        snprintf(&cmd[n], CMDLEN-n, " X%1.3f", THEROBOT->from_millimeters(x));
    }
    if(!isnan(y)) {
        size_t n= strlen(cmd);
        snprintf(&cmd[n], CMDLEN-n, " Y%1.3f", THEROBOT->from_millimeters(y));
    }
    if(!isnan(z)) {
        size_t n= strlen(cmd);
        snprintf(&cmd[n], CMDLEN-n, " Z%1.3f", THEROBOT->from_millimeters(z));
    }

    {
        size_t n= strlen(cmd);
        // use specified feedrate (mm/sec)
        snprintf(&cmd[n], CMDLEN-n, " F%1.1f", feedrate * 60); // feed rate is converted to mm/min
    }

    //THEKERNEL->streams->printf("DEBUG: move: %s: %u\n", cmd, strlen(cmd));

    // send as a command line as may have multiple G codes in it
    THEROBOT->push_state();
    struct SerialMessage message;
    message.message = cmd;
    delete [] cmd;

    message.stream = &(StreamOutput::NullStream);
    message.line = 0;
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    THEKERNEL->conveyor->wait_for_idle();
    THEROBOT->pop_state();

}

// issue home command
void ZProbe::home()
{
    Gcode gc(THEKERNEL->is_grbl_mode() ? "G28.2" : "G28", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
}

void ZProbe::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(zprobe_checksum)) return;
    if (pdr->second_element_is(get_zprobe_pin_states_checksum)) {
        char *data = static_cast<char *>(pdr->get_data_ptr());
        // cover endstop
        data[0] = (char)this->pin.get();
        data[1] = (char)this->calibrate_pin.get();
        pdr->set_taken();
    } else if (pdr->second_element_is(get_zprobe_time_checksum)) {
    	uint32_t *probe_time = static_cast<uint32_t *>(pdr->get_data_ptr());
    	*probe_time = this->probe_trigger_time;
        pdr->set_taken();
    }
}

void ZProbe::rotate(int axis, float axis_distance, float* x, float* y, float rotation_angle){
    if (axis == 0){
        rotateXY(axis_distance, NAN, x, y, rotation_angle);
    }else if (axis == 1){
        rotateXY(NAN, axis_distance, x, y, rotation_angle);
    }
}

void ZProbe::rotateXY(float x_in, float y_in, float* x_out, float* y_out, float rotation_angle){
    if (!isnan(x_in) && !isnan(y_in)){
        *x_out = x_in * cos(rotation_angle * (pi/180.0)) - y_in * sin(rotation_angle * (pi/180.0));
        *y_out = x_in * sin(rotation_angle * (pi/180.0)) + y_in * cos(rotation_angle * (pi/180.0));
    }else if(!isnan(x_in)){
        *x_out = x_in * cos(rotation_angle * (pi/180.0));
        *y_out = x_in * sin(rotation_angle * (pi/180.0));
    }else if(!isnan(y_in)){
        *x_out = - y_in * sin(rotation_angle * (pi/180.0));
        *y_out = y_in * cos(rotation_angle * (pi/180.0));
    }
}

float ZProbe::get_xyz_move_length(float x, float y, float z){
    return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

bool ZProbe::fast_slow_probe_sequence(int axis, int direction){
    float moveBuffer[3];
    float mpos[3];
    float old_mpos[3];
    float retract_direction = 0;
    float x = 0;
    float y = 0;
    float z = 0;
    float retractx = 0;
    float retracty = 0;
    float retractz = 0;
    float axis_distance = 0;

    // make sure the direction is correct for the function call
    if (axis == 0){
        axis_distance = direction * param.x_axis_distance;
    }else if(axis == 1){
        axis_distance = direction * param.y_axis_distance;
    }else if(axis == 2){
        axis_distance = param.z_axis_distance < 0 ? param.z_axis_distance : -param.z_axis_distance;
    }else if(axis == 10){
        axis_distance = get_xyz_move_length(param.x_axis_distance, param.y_axis_distance, param.z_axis_distance);
    }
    
    
    // retract direction always needs to be opposite of the probing direction
    if (axis != 10){
        if (axis_distance >= 0){
            retract_direction = -1.0;
        }else{
            retract_direction = 1.0;
        }
    }else{
        retract_direction = -1.0;
    }

    // in case of a rotation split the x_axis_distance in their corresponding x and y component
    if (axis == 0 || axis == 1){
        // rotate either x or y
        rotate(axis, axis_distance, &x, &y, param.rotation_angle);
        // rotate the retraction distance to the correct values in the mcs;
        rotate(axis, (retract_direction * param.retract_distance), &retractx, &retracty, param.rotation_angle_mcs);
        z = retractz = 0;
    }else if(axis == 2){
        retractx = x = retracty = y = 0;
        z = axis_distance;
        retractz = retract_direction * param.retract_distance;
    }else if (axis == 10){
        rotateXY(param.x_axis_distance, param.y_axis_distance, &x, &y, param.rotation_angle);
        z = param.z_axis_distance;
        retractx = retract_direction * (param.retract_distance/axis_distance) * x;
        retracty = retract_direction * (param.retract_distance/axis_distance) * y;
        retractz = retract_direction * (param.retract_distance/axis_distance) * z;

        // rotate the retraction again, because the delta move is in mcs and not wcs
        rotateXY(retractx, retracty, &retractx, &retracty, THEROBOT->r[THEROBOT->get_current_wcs()]);
    }

    // do positive probe
    memset(&this->buff, 0 , sizeof(this->buff));
    std::sprintf(this->buff, "G38.%i X%.3f Y%.3f Z%.3f F%.3f", 2 + param.probe_g38_subcode, THEROBOT->from_millimeters(x), THEROBOT->from_millimeters(y), THEROBOT->from_millimeters(z), param.feed_rate);
    this->gcodeBuffer = new Gcode(this->buff, &StreamOutput::NullStream);
    probe_XYZ(this->gcodeBuffer);
    delete gcodeBuffer;
    //move off the surface
    moveBuffer[0] = retractx;
    moveBuffer[1] = retracty;
    moveBuffer[2] = retractz;
    THEROBOT->delta_move(moveBuffer, param.feed_rate, 3);
    //slow probe
    memset(&this->buff, 0 , sizeof(this->buff));
    std::sprintf(this->buff, "G38.%i X%.3f Y%.3f Z%.3f", 2 + param.probe_g38_subcode,THEROBOT->from_millimeters(x), THEROBOT->from_millimeters(y), THEROBOT->from_millimeters(z));
    this->gcodeBuffer = new Gcode(this->buff, &StreamOutput::NullStream);
    probe_XYZ(this->gcodeBuffer);
    delete gcodeBuffer;
    // always wait for idle before getting the machine pos
    THECONVEYOR->wait_for_idle();
    //store position
    THEROBOT->get_current_machine_position(mpos);
    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
    if(THEKERNEL->is_flex_compensation_active()) {
        if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, false); // get inverse compensation transform
    }

    // if probing x positive then the output goes to the positive out and vice versa
    if (axis == 0){
        if (direction > 0){
            out_coords.x_positive_x_out= mpos[0];
            out_coords.x_positive_y_out = mpos[1];
        }else{
            out_coords.x_negative_x_out= mpos[0];
            out_coords.x_negative_y_out = mpos[1];
        }
    }else if (axis == 1){
        if (direction > 0){
            out_coords.y_positive_x_out= mpos[0];
            out_coords.y_positive_y_out = mpos[1];
        }else{
            out_coords.y_negative_x_out= mpos[0];
            out_coords.y_negative_y_out = mpos[1];
        }
    }else if (axis == 2){
        out_coords.z_negative_z_out = mpos[2];
    }else if (axis == 10){
        out_coords.x_positive_x_out = mpos[0];
        out_coords.y_positive_y_out = mpos[1];
        out_coords.z_negative_z_out = mpos[2];
    }
   
    moveBuffer[0] = retractx;
    moveBuffer[1] = retracty;
    moveBuffer[2] = retractz;
    THEROBOT->delta_move(moveBuffer, param.feed_rate, 3);
    // always wait for idle before getting the machine pos
    THECONVEYOR->wait_for_idle();
    return probe_detected;
}

int ZProbe::xy_probe_move_alarm_when_hit(int direction, int probe_g38_subcode, float x, float y, float feed_rate){
    // do positive x probe
    //probe no hit alarm x_positive - G38.3, alarm if true
    memset(&this->buff, 0 , sizeof(this->buff));
    std::sprintf(this->buff, "G38.%i X%.3f Y%.3f F%.3f", 3+probe_g38_subcode,THEROBOT->from_millimeters(direction * x), THEROBOT->from_millimeters(direction * y), feed_rate);
    this->gcodeBuffer = new Gcode(this->buff, &StreamOutput::NullStream);
    if (probe_XYZ(this->gcodeBuffer)){
        THEKERNEL->streams->printf("ALARM: Probe hit wall when moving to outer position\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        delete gcodeBuffer;
        return 1;
    }
    delete gcodeBuffer;
    return 0;
}

void ZProbe::z_probe_move_with_retract(int probe_g38_subcode, float z, float clearance_height, float feed_rate){
    float moveBuffer[3];

    // do z probe with slow speed
    THEKERNEL->streams->printf("Probing Z with a distance of %.3f\n", z);
    memset(&this->buff, 0 , sizeof(this->buff));
    std::sprintf(this->buff, "G38.%i Z%.3f F%.3f", 3 + probe_g38_subcode, THEROBOT->from_millimeters(z), feed_rate);
    this->gcodeBuffer = new Gcode(this->buff, &StreamOutput::NullStream);
    if (probe_XYZ(this->gcodeBuffer)){ //probe down, if bottom surface reached, retract slightly
        THEKERNEL->streams->printf("Probed surface hit");
        moveBuffer[0] = 0;
        moveBuffer[1] = 0;
        moveBuffer[2] = THEROBOT->from_millimeters(clearance_height);
        THEROBOT->delta_move(moveBuffer, feed_rate, 3);
        THECONVEYOR->wait_for_idle();
    }
    delete gcodeBuffer;
}

bool ZProbe::parse_parameters(Gcode *gcode, bool override_probe_check){
    init_parameters_and_out_coords();

    if (!((override_probe_check && THEKERNEL->eeprom_data->TOOL == 0) || (this->tool_0_3axis && THEKERNEL->eeprom_data->TOOL == 0) || THEKERNEL->eeprom_data->TOOL >= 999990)){
        THEKERNEL->streams->printf("ALARM: Attempted to 3 axis probe with an improper tool number. Tool number needs to be >= 999990\n or you need to set tool 0 as a 3 axis probe with: \n config-set sd zprobe.tool_zero_is_3axis true \n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return false;
    }else if(THEROBOT->get_tool_not_calibrated() && gcode->has_letter('S') && (gcode->has_letter('H') || gcode->has_letter('Z'))){
        if(gcode->get_value('S') == 2){
            THEKERNEL->streams->printf("ALARM: Probe not calibrated. Please calibrate probe before probing.\n");
            THEKERNEL->call_event(ON_HALT, nullptr);
            THEKERNEL->set_halt_reason(PROBE_FAIL);
            return false;
        }
    }

    if (gcode->has_letter('D')) { //probe tip diameter
        param.tool_dia = gcode->get_value('D');
    }
    if (gcode->has_letter('E')) { //depth to probe sides from
        param.side_depth = gcode->get_value('E');
    }
    if (gcode->has_letter('H')) { //probe height above bore/disance to move down before probing
        param.probe_height = gcode->get_value('H');
    }
    if (gcode->has_letter('C')){
        param.clearance_height = gcode->get_value('C');
    }
    if (gcode->has_letter('X')) { //radius x
        param.x_axis_distance = gcode->get_value('X');
    }
    if (gcode->has_letter('Y')) { //radius y
        param.y_axis_distance = gcode->get_value('Y');
    }
    if (gcode->has_letter('Z')) { //radius y
        param.z_axis_distance = gcode->get_value('Z');
    }
    if (gcode->has_letter('Q')) { //add a rotation angle to the currently active angle (wcs)
        param.rotation_angle = gcode->get_value('Q');
        param.rotation_angle_mcs = param.rotation_angle_mcs + param.rotation_angle;
    }
    if (gcode->has_letter('F')) { //feed rate
        param.feed_rate = gcode->get_value('F');
    }
    if (gcode->has_letter('K')) { //rapid feed rate
        param.rapid_rate = gcode->get_value('K');
    }
    if (gcode->has_letter('L')) { //repeat touch off
        param.repeat = gcode->get_value('L');
    }
    if (gcode->has_letter('R')) { //retract distance
        param.retract_distance = gcode->get_value('R');
    }
    if (gcode->has_letter('S')) { //save probed position
        param.save_position = gcode->get_value('S');
    }
    if (gcode->has_letter('V')) { //visualize path distance
        param.visualize_path_distance = (gcode->get_value('V'));
    }
    if (gcode->has_letter('U')) { 
        param.rotation_offset_per_probe = gcode->get_value('U');
    }
    if (gcode->has_letter('J')){
        param.extra_probe_distance = gcode->get_value('J');
    }
    if (gcode->has_letter('I')){ //invert for NC probe
        if (gcode->get_value('I') > 0)
        {
            param.probe_g38_subcode = 2;
            invert_probe = true;
        }
    }

    return true;
}

void ZProbe::init_parameters_and_out_coords(){
    memset(&out_coords, 0, sizeof(out_coords));
    memset(&param, 0, sizeof(param));
    param.tool_dia = THEKERNEL->probe_tip_diameter;    //D
    param.save_position = 0;
    param.probe_height = 0;                            //H
    param.feed_rate = 300;                             //F
    param.rapid_rate = 800;                            //K
    param.x_axis_distance = 0;                         //X
    param.y_axis_distance = 0;                         //Y
    param.z_axis_distance = 0;                         //Z
    param.rotation_angle = 0;                          //Q
    param.rotation_angle_mcs = THEROBOT->r[THEROBOT->get_current_wcs()];
    param.repeat = 1;                                  //L
    param.retract_distance = 1.5;                      //R
    param.clearance_height = 2;                        //C
    param.side_depth = 2;                              //E
    param.probe_g38_subcode = 0;                       //I                         
    param.extra_probe_distance = 4;                    //J
}

void ZProbe::probe_bore(bool calibration) //M461
{
    THECONVEYOR->wait_for_idle();
    THEKERNEL->streams->printf("Probing Bore/Rectangular Pocket\n");

    float mpos[3];
    float old_mpos[3];

    if (calibration){
        param.tool_dia = 0;
    }

    if (param.repeat < 1){
        THEKERNEL->streams->printf("ALARM: Probe fail: repeat value cannot be less than 1\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }

	//slow zprobe without alarm to probe_height. Skip if probe height is 0
	if (param.probe_height != 0){
        z_probe_move_with_retract(param.probe_g38_subcode, -param.probe_height, param.clearance_height, param.feed_rate);
    }
    // always wait for idle before getting the machine pos
    THECONVEYOR->wait_for_idle();
    //save center position to use later
    THEROBOT->get_current_machine_position(mpos);
    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
    if(THEKERNEL->is_flex_compensation_active()) {
        if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, false); // get inverse compensation transform
    }
    out_coords.origin_x = mpos[0];
    out_coords.origin_y = mpos[1];

	//setup repeat
	for(int i=0; i< param.repeat; i++) {
        if (param.x_axis_distance != 0) {
            // probe in positive x direction
            fast_slow_probe_sequence(X_AXIS, POS);

            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", out_coords.x_positive_x_out, out_coords.x_positive_y_out);
            //move back to the center position
            coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate);

            // probe in negative x direction
            fast_slow_probe_sequence(X_AXIS, NEG);

            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", out_coords.x_negative_x_out, out_coords.x_negative_y_out);
            //calculate center of bore (will only be centered in x)
            out_coords.origin_x = (out_coords.x_positive_x_out + out_coords.x_negative_x_out)/2;
            out_coords.origin_y = (out_coords.x_positive_y_out + out_coords.x_negative_y_out)/2;
            //goto current center position
            coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate);
            THECONVEYOR->wait_for_idle();
            THEKERNEL->probe_outputs[0] = sqrt(   (out_coords.x_positive_x_out - out_coords.x_negative_x_out) 
                                                * (out_coords.x_positive_x_out - out_coords.x_negative_x_out) 
                                                + (out_coords.x_positive_y_out - out_coords.x_negative_y_out) 
                                                * (out_coords.x_positive_y_out - out_coords.x_negative_y_out)
                                            ) + param.tool_dia;

            THEKERNEL->streams->printf("Distance Point 2 X surfaces (Diameter) is: %.3f and center is stored at variable #151\n" , THEKERNEL->probe_outputs[0] );
        }

        if (param.y_axis_distance != 0) {
            // probe in positive < direction
            fast_slow_probe_sequence(Y_AXIS, POS);

            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", out_coords.y_positive_x_out, out_coordsy_positive_y_out);
            //goto current center position
            coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate );
            THECONVEYOR->wait_for_idle();
            
            // probe in negative y direction
            fast_slow_probe_sequence(Y_AXIS, NEG);
            
            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", out_coords.x_negative_x_out, out_coords.x_negative_y_out);
            //calculate center of bore (will only be centered in x)
            out_coords.origin_x = (out_coords.y_positive_x_out + out_coords.y_negative_x_out)/2;
            out_coords.origin_y = (out_coords.y_positive_y_out + out_coords.y_negative_y_out)/2;
            //goto current center position
            coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate);
            THECONVEYOR->wait_for_idle();
            THEKERNEL->probe_outputs[1] = sqrt(   (out_coords.y_positive_x_out - out_coords.y_negative_x_out) 
                                                * (out_coords.y_positive_x_out - out_coords.y_negative_x_out) 
                                                + (out_coords.y_positive_y_out - out_coords.y_negative_y_out) 
                                                * (out_coords.y_positive_y_out - out_coords.y_negative_y_out) 
                                            ) + param.tool_dia;
            THEKERNEL->streams->printf("Distance between 2 Y surfaces (Diameter) is: %.3f and is stored at variable #152\n" , THEKERNEL->probe_outputs[1] );
        }

		
	}
	THEKERNEL->streams->printf("Center of bore or rectangular pocket found. Ready to Zero X and Y\n");
    THEKERNEL->probe_outputs[3] = out_coords.origin_x;
    THEKERNEL->probe_outputs[4] = out_coords.origin_y;
    THEKERNEL->streams->printf("Center Point is: %.3f , %.3f and is stored in MCS as #154,#155\n" , THEKERNEL->probe_outputs[3],THEKERNEL->probe_outputs[4] );

    if (param.save_position > 0 && check_last_probe_ok()){
        if (param.x_axis_distance != 0 && param.y_axis_distance != 0){
            THEROBOT->set_current_wcs_by_mpos( THEKERNEL->probe_outputs[3], THEKERNEL->probe_outputs[4], NAN);
        }else if (param.x_axis_distance != 0){
            THEROBOT->set_current_wcs_by_mpos( THEKERNEL->probe_outputs[3], NAN, NAN);
        }else if (param.y_axis_distance != 0){
            THEROBOT->set_current_wcs_by_mpos( NAN, THEKERNEL->probe_outputs[4], NAN);
        } 
    }
}

void ZProbe::probe_boss(bool calibration) //M462
{
    THECONVEYOR->wait_for_idle();
    THEKERNEL->streams->printf("Probing Boss or Rectangular Block\n");
    
    float mpos[3];
    float old_mpos[3];
    bool probe_x_axis = false;
    bool probe_y_axis = false;

    if (param.x_axis_distance != 0){
        probe_x_axis = true;
    }
    if (param.y_axis_distance != 0){
        probe_y_axis = true;
    }

    // Only modify axis distances for axes that are actually being probed
    if (probe_x_axis) {
        param.x_axis_distance = param.x_axis_distance/2 + param.extra_probe_distance;
    }
    if (probe_y_axis) {
        param.y_axis_distance = param.y_axis_distance/2 + param.extra_probe_distance;
    }

    if (calibration){
        param.tool_dia = 0;
    }

    // When called from calibrate_probe_boss(), don't use repeat loop here
    // The repeat logic is handled by the calling function
    int repeat_count = calibration ? 1 : param.repeat;
    
    if (repeat_count < 1){
        THEKERNEL->streams->printf("ALARM: Probe fail: repeat value cannot be less than 1\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }
	//slow zprobe without alarm to probe_height. Skip if probe height is 0
	if (param.probe_height != 0){
        param.z_axis_distance = param.probe_height;
        fast_slow_probe_sequence(Z_AXIS, POS);
        if (param.save_position == 2 && check_last_probe_ok()){
            THEROBOT->set_current_wcs_by_mpos( NAN, NAN, out_coords.z_negative_z_out);
        }
        //z_probe_move_with_retract(param.probe_g38_subcode, -param.probe_height, param.clearance_height, param.feed_rate);
    }

    rotate(X_AXIS, param.x_axis_distance, &param.x_rotated_x, &param.x_rotated_y, param.rotation_angle);
    rotate(Y_AXIS, param.y_axis_distance, &param.y_rotated_x, &param.y_rotated_y, param.rotation_angle);
    // always wait for idle before getting the machine pos
    THECONVEYOR->wait_for_idle();
    //save center position to use later
    THEROBOT->get_current_machine_position(mpos);
    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
    if(THEKERNEL->is_flex_compensation_active()) {
        if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, false); // get inverse compensation transform
    }
    out_coords.origin_x = mpos[0];
    this->out_coords.origin_y = mpos[1];
    this->param.clearance_world_pos = mpos[2]; //test old_mpos[2];
	//setup repeat
	for(int i=0; i< repeat_count; i++) {
        //goto clearance height
        coordinated_move(NAN, NAN, param.clearance_world_pos, param.rapid_rate);
        THECONVEYOR->wait_for_idle();
        if (probe_x_axis) {
            // return if probe touches wall during outside move
            if (xy_probe_move_alarm_when_hit(POS, param.probe_g38_subcode, param.x_rotated_x, param.x_rotated_y, param.rapid_rate) == 1){
                return;
            }
            //probe z no hit no alarm -side_depth, retract slightly if probe point hit
            if (param.probe_height != 0){
                z_probe_move_with_retract(param.probe_g38_subcode, -(param.side_depth + param.clearance_height + param.tool_dia/2.0), 1.0, param.rapid_rate);
            }else{
                z_probe_move_with_retract(param.probe_g38_subcode, -(param.side_depth + param.tool_dia/2.0), 1.0, param.rapid_rate);
            }

            // probe in negative x direction
            fast_slow_probe_sequence(X_AXIS, NEG);

            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", out_coords.x_positive_x_out, out_coords.x_positive_y_out);
            //goto clearance_world_pos in z
            coordinated_move(NAN, NAN, param.clearance_world_pos, param.rapid_rate);
            THECONVEYOR->wait_for_idle();
            //return to center position
            coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate );
            
            // return if probe touches wall during outside move
            if (xy_probe_move_alarm_when_hit(NEG, param.probe_g38_subcode, param.x_rotated_x, param.x_rotated_y, param.rapid_rate) == 1){
                return;
            }

            //probe z no hit no alarm -side_depth, retract slightly if probe point hit
            if (param.probe_height != 0){
                z_probe_move_with_retract(param.probe_g38_subcode, -(param.side_depth + param.clearance_height + param.tool_dia/2.0), 1.0, param.rapid_rate);
            }else{
                z_probe_move_with_retract(param.probe_g38_subcode, -(param.side_depth + param.tool_dia/2.0), 1.0, param.rapid_rate);
            }

            // probe in positive x direction
            fast_slow_probe_sequence(X_AXIS, POS);

            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", out_coords.x_positive_x_out, out_coords.x_positive_y_out);
            //calculate center of bore (will only be centered in x)
            out_coords.origin_x = (out_coords.x_positive_x_out + out_coords.x_negative_x_out)/2;
            out_coords.origin_y = (out_coords.x_positive_y_out + out_coords.x_negative_y_out)/2;

            //goto clearance_world_pos in z
            coordinated_move(NAN, NAN, param.clearance_world_pos, param.rapid_rate);
            THECONVEYOR->wait_for_idle();
            //goto current center position
            coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate );
            THECONVEYOR->wait_for_idle();
            THEKERNEL->probe_outputs[0] = sqrt(   (out_coords.x_positive_x_out - out_coords.x_negative_x_out) 
                                                * (out_coords.x_positive_x_out - out_coords.x_negative_x_out) 
                                                + (out_coords.x_positive_y_out - out_coords.x_negative_y_out) 
                                                * (out_coords.x_positive_y_out - out_coords.x_negative_y_out) 
                                            ) - param.tool_dia;
            THEKERNEL->streams->printf("Distance Betweeen 2 X surfaces (Diameter) is: %.3f and is stored at variable #151\n" , THEKERNEL->probe_outputs[0] );
        }

        if (probe_y_axis) {
            // return if probe touches wall during outside move
            if (xy_probe_move_alarm_when_hit(POS, param.probe_g38_subcode, param.y_rotated_x, param.y_rotated_y, param.rapid_rate) == 1){
                return;
            }

            //probe z no hit no alarm -side_depth, retract slightly if probe point hit
            if (param.probe_height != 0){
                z_probe_move_with_retract(param.probe_g38_subcode, -(param.side_depth + param.clearance_height + param.tool_dia/2.0), 1.0, param.rapid_rate);
            }else{
                z_probe_move_with_retract(param.probe_g38_subcode, -(param.side_depth + param.tool_dia/2.0), 1.0, param.rapid_rate);
            }
            
            // probe in negative y direction
            fast_slow_probe_sequence(Y_AXIS, NEG);

            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", out_coords.y_positive_x_out, out_coords.y_positive_y_out);
            //goto clearance_world_pos in z
            coordinated_move(NAN, NAN, param.clearance_world_pos, param.rapid_rate);
            THECONVEYOR->wait_for_idle();
            //goto current center position
            coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate );

            // return if probe touches wall during outside move
            if (xy_probe_move_alarm_when_hit(NEG, param.probe_g38_subcode, param.y_rotated_x, param.y_rotated_y, param.rapid_rate) == 1){
                return;
            }

            //probe z no hit no alarm -side_depth, retract slightly if probe point hit
            if (param.probe_height != 0){
                z_probe_move_with_retract(param.probe_g38_subcode, -(param.side_depth + param.clearance_height + param.tool_dia/2.0), 1.0, param.rapid_rate);
            }else{
                z_probe_move_with_retract(param.probe_g38_subcode, -(param.side_depth + param.tool_dia/2.0), 1.0, param.rapid_rate);
            }

            // probe in positive y direction
            fast_slow_probe_sequence(Y_AXIS, POS);
            
            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", out_coords.y_positive_x_out, out_coords.y_positive_y_out);
            //calculate center of bore (will only be centered in x)
            out_coords.origin_x = (out_coords.y_positive_x_out + out_coords.y_negative_x_out)/2;
            out_coords.origin_y = (out_coords.y_positive_y_out + out_coords.y_negative_y_out)/2;

            //goto clearance_world_pos in z
            coordinated_move(NAN, NAN, param.clearance_world_pos, param.rapid_rate);
            THECONVEYOR->wait_for_idle();
            //goto current center position
            coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate );
            THECONVEYOR->wait_for_idle();
            THEKERNEL->probe_outputs[1] = sqrt(   (out_coords.y_positive_x_out - out_coords.y_negative_x_out) 
                                                * (out_coords.y_positive_x_out - out_coords.y_negative_x_out) 
                                                + (out_coords.y_positive_y_out - out_coords.y_negative_y_out) 
                                                * (out_coords.y_positive_y_out - out_coords.y_negative_y_out) 
                                            ) - param.tool_dia;
            THEKERNEL->streams->printf("Distance Betweeen 2 Y surfaces (Diameter) is: %.3f and is stored at variable #152\n" , THEKERNEL->probe_outputs[1] );
        }

		
	}
	THEKERNEL->streams->printf("Center of Boss or Rectangular Block found. Ready to Zero X and Y\n");
    THEKERNEL->probe_outputs[3] = out_coords.origin_x;
    THEKERNEL->probe_outputs[4] = out_coords.origin_y;
    THEKERNEL->streams->printf("Center Point is: %.3f , %.3f and is stored in MCS as #154,#155\n" , THEKERNEL->probe_outputs[3],THEKERNEL->probe_outputs[4] );

    if (param.save_position > 0 && check_last_probe_ok()){
        if (probe_x_axis && probe_y_axis){
            THEROBOT->set_current_wcs_by_mpos( THEKERNEL->probe_outputs[3], THEKERNEL->probe_outputs[4], NAN);
        }else if (probe_x_axis){
            THEROBOT->set_current_wcs_by_mpos( THEKERNEL->probe_outputs[3], NAN, NAN);
        }else if (probe_y_axis){
            THEROBOT->set_current_wcs_by_mpos( NAN, THEKERNEL->probe_outputs[4], NAN);
        }
    }
}

void ZProbe::probe_insideCorner() //M463
{
    float mpos[3];

    if (param.repeat < 1){
        THEKERNEL->streams->printf("ALARM: Probe fail: repeat value cannot be less than 1\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }
    
	//slow zprobe without alarm to probe_height. Skip if probe height is 0
	if (param.probe_height != 0){
		z_probe_move_with_retract(param.probe_g38_subcode, -param.probe_height, param.clearance_height, param.feed_rate);
    }
    // always wait for idle before getting the machine pos
    THECONVEYOR->wait_for_idle();
    //save center position to use later
    THEROBOT->get_current_machine_position(mpos);
    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
    if(THEKERNEL->is_flex_compensation_active()) {
        if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, false); // get inverse compensation transform
    }
    out_coords.origin_x = mpos[0];
    out_coords.origin_y = mpos[1];

    rotate(X_AXIS, param.x_axis_distance, &param.x_rotated_x, &param.x_rotated_y, param.rotation_angle);
    rotate(Y_AXIS, param.y_axis_distance, &param.y_rotated_x, &param.y_rotated_y, param.rotation_angle);
    rotate(X_AXIS, (param.tool_dia/2.0), &param.half_tool_dia_rotated_x_x, &param.half_tool_dia_rotated_x_y, param.rotation_angle_mcs);
    rotate(Y_AXIS, (param.tool_dia/2.0), &param.half_tool_dia_rotated_y_x, &param.half_tool_dia_rotated_y_y, param.rotation_angle_mcs);

	//setup repeat
	for(int i=0; i< param.repeat; i++) {
        
        fast_slow_probe_sequence(X_AXIS, POS);

        out_coords.x_positive_x_out = out_coords.x_positive_x_out + (param.x_axis_distance>= 0 ? 1.0f : -1.0f) *  param.half_tool_dia_rotated_x_x;
        out_coords.x_positive_y_out = out_coords.x_positive_y_out + (param.x_axis_distance>= 0 ? 1.0f : -1.0f) *  param.half_tool_dia_rotated_x_y;

        //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", x_positive_x_out, x_positive_y_out);

        //goto current center position
        coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate );
        THECONVEYOR->wait_for_idle();

        fast_slow_probe_sequence(Y_AXIS, POS);

        out_coords.y_positive_y_out = out_coords.y_positive_y_out + (param.y_axis_distance>= 0 ? 1.0f : -1.0f) * param.half_tool_dia_rotated_y_y;
        out_coords.y_positive_x_out = out_coords.y_positive_x_out + (param.y_axis_distance>= 0 ? 1.0f : -1.0f) * param.half_tool_dia_rotated_y_x;

        //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", y_positive_x_out, y_positive_y_out);
        //goto current center position
        coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate );
        THECONVEYOR->wait_for_idle();
        
        // calculate center position
        if (param.rotation_angle_mcs == 0)
        {
            THEKERNEL->probe_outputs[3] = out_coords.x_positive_x_out;
            THEKERNEL->probe_outputs[4] = out_coords.y_positive_y_out;
        }
        else
        {
            // 
            float lines_m1_value = tan(param.rotation_angle_mcs * pi / 180);
            float lines_m2_value = tan((param.rotation_angle_mcs + 90.0) * pi / 180);
            float lines_c1_value = out_coords.y_positive_y_out - out_coords.y_positive_x_out * lines_m1_value;
            float lines_c2_value = out_coords.x_positive_y_out - out_coords.x_positive_x_out * lines_m2_value;
            THEKERNEL->probe_outputs[3] = (lines_c2_value - lines_c1_value) / (lines_m1_value-lines_m2_value); // x_out
            THEKERNEL->probe_outputs[4] = (lines_m1_value * lines_c2_value - lines_m2_value * lines_c1_value) 
                                          / (lines_m1_value - lines_m2_value);   // y_out
        }

	}
    THEKERNEL->streams->printf("Corner found. X coordinate stored in #154 as MCS %.3f , Y coordinate in #155 as MCS %.3f \n", THEKERNEL->probe_outputs[3], THEKERNEL->probe_outputs[4] );
    
    if (param.save_position > 0){
        THEROBOT->set_current_wcs_by_mpos( THEKERNEL->probe_outputs[3], THEKERNEL->probe_outputs[4], NAN);
    }
}

void ZProbe::probe_outsideCorner() //M464
{
    THECONVEYOR->wait_for_idle();
    THEKERNEL->streams->printf("Probing Outside Corner\n");

    float mpos[3];

    if (param.repeat < 1){
        THEKERNEL->streams->printf("ALARM: Probe fail: repeat value cannot be less than 1\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }
	//slow zprobe without alarm to probe_height. Skip if probe height is 0
	if (param.probe_height != 0){
        param.z_axis_distance = param.probe_height;
        fast_slow_probe_sequence(Z_AXIS, POS);
        if (param.save_position == 2 && check_last_probe_ok()){
            THEROBOT->set_current_wcs_by_mpos( NAN, NAN, out_coords.z_negative_z_out);
        }
        //z_probe_move_with_retract(param.probe_g38_subcode, -param.probe_height, param.clearance_height, param.feed_rate);
    }

    rotate(X_AXIS, param.x_axis_distance, &param.x_rotated_x, &param.x_rotated_y, param.rotation_angle);
    rotate(Y_AXIS, param.y_axis_distance, &param.y_rotated_x, &param.y_rotated_y, param.rotation_angle);
    rotate(X_AXIS, (param.tool_dia/2.0), &param.half_tool_dia_rotated_x_x, &param.half_tool_dia_rotated_x_y, param.rotation_angle_mcs);
    rotate(Y_AXIS, (param.tool_dia/2.0), &param.half_tool_dia_rotated_y_x, &param.half_tool_dia_rotated_y_y, param.rotation_angle_mcs);
    // always wait for idle before getting the machine pos
    THECONVEYOR->wait_for_idle();
    //save center position to use later
    THEROBOT->get_current_machine_position(mpos);
    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
    if(THEKERNEL->is_flex_compensation_active()) {
        if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, false); // get inverse compensation transform
    }
    out_coords.origin_x = mpos[0];
    out_coords.origin_y = mpos[1];
    param.clearance_world_pos = mpos[2];

	//setup repeat
	for(int i=0; i< param.repeat; i++) {

        coordinated_move(NAN, NAN, param.clearance_world_pos, param.rapid_rate);

        // return if probe touches wall during outside move
        if (xy_probe_move_alarm_when_hit(NEG, param.probe_g38_subcode, param.x_rotated_x, param.x_rotated_y, param.rapid_rate) == 1){
            return;
        }
        
        //probe z no hit no alarm -side_depth, retract slightly if probe point hit
        if (param.probe_height != 0){
            z_probe_move_with_retract(param.probe_g38_subcode, -(param.side_depth + param.clearance_height + param.tool_dia/2.0), 1.0, param.rapid_rate);
        }else{
            z_probe_move_with_retract(param.probe_g38_subcode, -(param.side_depth + param.tool_dia/2.0), 1.0, param.rapid_rate);
        }
        
        // probe in positive x direction
        fast_slow_probe_sequence(X_AXIS, POS);

        out_coords.x_positive_x_out = out_coords.x_positive_x_out + (param.x_axis_distance>= 0 ? 1.0f : -1.0f) *  param.half_tool_dia_rotated_x_x;
        out_coords.x_positive_y_out = out_coords.x_positive_y_out + (param.x_axis_distance>= 0 ? 1.0f : -1.0f) *  param.half_tool_dia_rotated_x_y;
       
        //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", out_coords.x_positive_x_out, out_coords.x_positive_y_out);

        //goto clearance_world_pos in z
        coordinated_move(NAN, NAN, param.clearance_world_pos, param.rapid_rate);
        THECONVEYOR->wait_for_idle();
        //goto current center position
        coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate );
        
        // return if probe touches wall during outside move
        if (xy_probe_move_alarm_when_hit(NEG, param.probe_g38_subcode, param.y_rotated_x, param.y_rotated_y, param.rapid_rate) == 1){
            return;
        }

        //probe z no hit no alarm -side_depth, retract slightly if probe point hit
        if (param.probe_height != 0){
            z_probe_move_with_retract(param.probe_g38_subcode, -(param.side_depth + param.clearance_height + param.tool_dia/2.0), 1.0, param.rapid_rate);
        }else{
            z_probe_move_with_retract(param.probe_g38_subcode, -(param.side_depth + param.tool_dia/2.0), 1.0, param.rapid_rate);
        }

        // probe in positive y direction
        fast_slow_probe_sequence(Y_AXIS, POS);

        out_coords.y_positive_y_out = out_coords.y_positive_y_out + (param.y_axis_distance>= 0 ? 1.0f : -1.0f) * param.half_tool_dia_rotated_y_y;
        out_coords.y_positive_x_out = out_coords.y_positive_x_out + (param.y_axis_distance>= 0 ? 1.0f : -1.0f) * param.half_tool_dia_rotated_y_x;

        //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", out_coords.y_positive_x_out, out_coords.y_positive_y_out);

        //goto clearance_world_pos in z
        coordinated_move(NAN, NAN, param.clearance_world_pos, param.rapid_rate);
        THECONVEYOR->wait_for_idle();

        // calculate center position
        if (param.rotation_angle_mcs == 0)
        {
            THEKERNEL->probe_outputs[3] = out_coords.x_positive_x_out;
            THEKERNEL->probe_outputs[4] = out_coords.y_positive_y_out;
        }
        else
        {
            // 
            float lines_m1_value = tan(param.rotation_angle_mcs * pi / 180);
            float lines_m2_value = tan((param.rotation_angle_mcs + 90.0) * pi / 180);
            float lines_c1_value = out_coords.y_positive_y_out - out_coords.y_positive_x_out * lines_m1_value;
            float lines_c2_value = out_coords.x_positive_y_out - out_coords.x_positive_x_out * lines_m2_value;
            THEKERNEL->probe_outputs[3] = (lines_c2_value - lines_c1_value) / (lines_m1_value-lines_m2_value); // x_out
            THEKERNEL->probe_outputs[4] = (lines_m1_value * lines_c2_value - lines_m2_value * lines_c1_value) 
                                          / (lines_m1_value - lines_m2_value);   // y_out
        }

        //goto center position
        coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate);
        THECONVEYOR->wait_for_idle();

		
	}
    THEKERNEL->streams->printf("Corner found. X coordinate stored in #154 as MCS %.3f , Y coordinate in #155 as MCS %.3f  \n", THEKERNEL->probe_outputs[3], THEKERNEL->probe_outputs[4] );
    
    if (param.save_position > 0 && check_last_probe_ok()){
        THEROBOT->set_current_wcs_by_mpos(THEKERNEL->probe_outputs[3], THEKERNEL->probe_outputs[4], NAN);
    }
    if (param.save_position == 2){
        coordinated_move(NAN, NAN, out_coords.z_negative_z_out + 2, param.rapid_rate);
        THECONVEYOR->wait_for_idle();
        coordinated_move(THEKERNEL->probe_outputs[3], THEKERNEL->probe_outputs[4], out_coords.z_negative_z_out + 2, param.rapid_rate);
        THECONVEYOR->wait_for_idle();
    }else{
        coordinated_move(NAN, NAN, param.clearance_world_pos, param.rapid_rate);
        THECONVEYOR->wait_for_idle();
        coordinated_move(THEKERNEL->probe_outputs[3], THEKERNEL->probe_outputs[4], param.clearance_world_pos, param.rapid_rate);
        THECONVEYOR->wait_for_idle();
    }
}

void ZProbe::probe_axisangle(bool probe_a_axis, bool probe_with_offset) //M465
{
    THECONVEYOR->wait_for_idle();
    THEKERNEL->streams->printf("Probing 2 points to find an angle\n");

    float mpos[3];
    float a_axis_pos;
    bool probe_x = false;

    if (probe_with_offset){
        probe_x = PublicData::get_value(atc_handler_checksum, get_machine_offsets_checksum, &machine_offset );
        // Validate the values before using them
        if (isnan(machine_offset.anchor1_x) || isnan(machine_offset.anchor1_y) || 
            isnan(machine_offset.rotation_offset_x) || isnan(machine_offset.rotation_offset_y)) {
            THEKERNEL->streams->printf("ALARM: Invalid machine offset values\n");
            THEKERNEL->call_event(ON_HALT, nullptr);
            THEKERNEL->set_halt_reason(PROBE_FAIL);
            return;
        }
        coordinated_move(NAN, NAN, machine_offset.clearance_z, param.rapid_rate / 60, false);
        THECONVEYOR->wait_for_idle();

        // Calculate target coordinates with validation
        float target_x = machine_offset.anchor1_x + machine_offset.rotation_offset_x + param.x_axis_distance;
        float target_y = machine_offset.anchor1_y + machine_offset.rotation_offset_y;

        if (isnan(target_x) || isnan(target_y)) {
            THEKERNEL->streams->printf("ALARM: Invalid target coordinates\n");
            THEKERNEL->call_event(ON_HALT, nullptr);
            THEKERNEL->set_halt_reason(PROBE_FAIL);
            return;
        }
        coordinated_move(target_x, target_y, NAN, param.rapid_rate / 60, false);
        THECONVEYOR->wait_for_idle();
        param.probe_height = 300;
    }

    if (!probe_a_axis){
        if (param.x_axis_distance != 0) {
            probe_x = true;
            if (param.visualize_path_distance != 0){
                param.visualize_path_distance = fabs(param.visualize_path_distance) * (param.x_axis_distance / fabs(param.x_axis_distance));
            }
            param.y_axis_distance = param.side_depth;
        }else{
            if (param.visualize_path_distance != 0){
                    param.visualize_path_distance = fabs(param.visualize_path_distance) * (param.y_axis_distance / fabs(param.y_axis_distance));
            }
            param.x_axis_distance = param.side_depth;
        }

        if (param.repeat < 1){
            THEKERNEL->streams->printf("ALARM: Probe fail: repeat value cannot be less than 1\n");
            THEKERNEL->call_event(ON_HALT, nullptr);
            THEKERNEL->set_halt_reason(PROBE_FAIL);
            return;
        }
    }else{
        param.rotation_angle = 0;
        param.y_axis_distance = param.y_axis_distance / 2.0;
        param.z_axis_distance = param.probe_height;
    }
	//slow zprobe without alarm to probe_height. Skip if probe height is 0
	if (param.probe_height != 0 && !probe_a_axis){
		// do z probe with slow speed
        z_probe_move_with_retract(param.probe_g38_subcode, -param.probe_height, param.clearance_height, param.feed_rate);
    }

    rotate(X_AXIS, param.x_axis_distance, &param.x_rotated_x, &param.x_rotated_y, param.rotation_angle);
    rotate(Y_AXIS, param.y_axis_distance, &param.y_rotated_x, &param.y_rotated_y, param.rotation_angle);
    // always wait for idle before getting the machine pos
    THECONVEYOR->wait_for_idle();
    //save center position to use later
    THEROBOT->get_current_machine_position(mpos);    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
    a_axis_pos = THEROBOT->actuators[3]->get_current_position();
    if(THEKERNEL->is_flex_compensation_active()) {
        if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, false); // get inverse compensation transform
    }
    out_coords.origin_x = mpos[0];
    out_coords.origin_y = mpos[1];
    param.clearance_world_pos = mpos[2];

    // Vector to collect all angle measurements for averaging
    vector<float> angle_measurements;

	//setup repeat
	for(int i=0; i< param.repeat; i++) {
        
        //goto clearance height
        //coordinated_move(NAN, NAN, clearance_world_pos, rapid_rate);

        if (probe_a_axis){
            // goto start point for repeated probing
            coordinated_move(NAN, NAN, param.clearance_world_pos, param.rapid_rate );
            coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate );

            // probe along x axis to second position, alarm if hit
            if (xy_probe_move_alarm_when_hit(POS, param.probe_g38_subcode, 0.0, param.y_rotated_y, param.rapid_rate) == 1) {
                THEKERNEL->streams->printf("ALARM: Probe fail: hit wall during first move\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                return;
            }
            
            fast_slow_probe_sequence(Z_AXIS, NEG);
            if (check_last_probe_ok()){
                out_coords.y_positive_y_out = out_coords.z_negative_z_out;
            }else{
                THEKERNEL->streams->printf("ALARM: Probe fail: first point not found\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                return;
            }
            
            if (xy_probe_move_alarm_when_hit(NEG, param.probe_g38_subcode, 0.0, 2 * param.y_rotated_y, param.rapid_rate) == 1) {
                THEKERNEL->streams->printf("ALARM: Probe fail: hit wall during second move\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                return;
            }
            fast_slow_probe_sequence(Z_AXIS, NEG);
            if (check_last_probe_ok()){
                out_coords.y_negative_y_out = out_coords.z_negative_z_out;
            }
            else{
                THEKERNEL->streams->printf("ALARM: Probe fail: second point not found\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                return;
            }

            float current_angle = atan (  (out_coords.y_positive_y_out - out_coords.y_negative_y_out) 
                                                / (2 * param.y_axis_distance)) * 180 /pi;
            angle_measurements.push_back(current_angle);
            THEKERNEL->streams->printf("Measurement %d: Angle from A Axis is: %.3f degrees or %.3f radians\n" , i+1, current_angle, current_angle * pi / 180 );

        }else if (probe_x) {
            
            fast_slow_probe_sequence(Y_AXIS, POS);
            THECONVEYOR->wait_for_idle();
            
            if (!check_last_probe_ok()) {
                THEKERNEL->streams->printf("ALARM: Probe fail: first point not found\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                return;
            }

            // save the first point in the x_positive output
            out_coords.x_positive_x_out = out_coords.y_positive_x_out;
            out_coords.x_positive_y_out = out_coords.y_positive_y_out;

            //return to center position
            coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate );
            //probe along x axis to second position, alarm if hit
            if (xy_probe_move_alarm_when_hit(POS, param.probe_g38_subcode, param.x_rotated_x, param.x_rotated_y, param.rapid_rate) == 1) {
                THEKERNEL->streams->printf("ALARM: Probe fail: hit wall during move to second position\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                return;
            }

            //probe along y axis to find second point
            fast_slow_probe_sequence(Y_AXIS, POS);
            THECONVEYOR->wait_for_idle();
            
            if (!check_last_probe_ok()) {
                THEKERNEL->streams->printf("ALARM: Probe fail: second point not found\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                return;
            }

            //return to x axis probe position 2
            coordinated_move(out_coords.origin_x + param.x_rotated_x, out_coords.origin_y + param.x_rotated_y, NAN, param.rapid_rate );
            THECONVEYOR->wait_for_idle();
            //return to center position
            coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate );
            THECONVEYOR->wait_for_idle();
            
            //calculate angle
            //inverse tan ( (Point 2 y - Point 1 Y) / (Point 2 x - point 1 X) )
            float current_angle = atan (  (out_coords.y_positive_y_out - out_coords.x_positive_y_out) 
                                                / (out_coords.y_positive_x_out - out_coords.x_positive_x_out)) * 180 /pi;
            angle_measurements.push_back(current_angle);
            THEKERNEL->streams->printf("Measurement %d: Angle from X Axis is: %.3f degrees or %.3f radians\n" , i+1, current_angle, current_angle * pi / 180 );
        }else{
            fast_slow_probe_sequence(X_AXIS, POS);
            THECONVEYOR->wait_for_idle();
            
            if (!check_last_probe_ok()) {
                THEKERNEL->streams->printf("ALARM: Probe fail: first point not found\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                return;
            }

            // save the first point in the x_positive output
            out_coords.y_positive_x_out = out_coords.x_positive_x_out;
            out_coords.y_positive_y_out = out_coords.x_positive_y_out;

            //return to center position
            coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate );
            //probe along x axis to second position, alarm if hit
            if (xy_probe_move_alarm_when_hit(POS, param.probe_g38_subcode, param.y_rotated_x, param.y_rotated_y, param.rapid_rate) == 1) {
                THEKERNEL->streams->printf("ALARM: Probe fail: hit wall during move to second position\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                return;
            }

            //probe along y axis to find second point
            fast_slow_probe_sequence(X_AXIS, POS);
            THECONVEYOR->wait_for_idle();
            
            if (!check_last_probe_ok()) {
                THEKERNEL->streams->printf("ALARM: Probe fail: second point not found\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                return;
            }

            //return to y axis probe position 2
            coordinated_move(out_coords.origin_x + param.y_rotated_x, out_coords.origin_y + param.y_rotated_y, NAN, param.rapid_rate );
            THECONVEYOR->wait_for_idle();
            //return to center position
            coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate );
            THECONVEYOR->wait_for_idle();
            
            //calculate angle
            //inverse tan ( (Point 2 y - Point 1 Y) / (Point 2 x - point 1 X) )
            float current_angle = atan (  (out_coords.y_positive_x_out - out_coords.x_positive_x_out) 
                                                / (out_coords.x_positive_y_out - out_coords.y_positive_y_out)) * 180 /pi;
            angle_measurements.push_back(current_angle);
            THEKERNEL->streams->printf("Measurement %d: Angle from Y Axis is: %.3f degrees or %.3f radians\n" , i+1, current_angle, current_angle * pi / 180 );
        }

		
	}

    // Calculate average angle from all measurements
    if (!angle_measurements.empty()) {
        float sum = 0.0;
        for (const auto& angle : angle_measurements) {
            sum += angle;
        }
        float average_angle = sum / angle_measurements.size();
        THEKERNEL->probe_outputs[2] = average_angle;
        
        // Display the average result
        if (probe_a_axis) {
            THEKERNEL->streams->printf("Average angle from A Axis is: %.3f degrees or %.3f radians and is stored in radians at variable #153\n", 
                                      average_angle, average_angle * pi / 180);
        } else if (probe_x) {
            THEKERNEL->streams->printf("Average angle from X Axis is: %.3f degrees or %.3f radians and is stored in radians at variable #153\n", 
                                      average_angle, average_angle * pi / 180);
        } else {
            THEKERNEL->streams->printf("Average angle from Y Axis is: %.3f degrees or %.3f radians and is stored in radians at variable #153\n", 
                                      average_angle, average_angle * pi / 180);
        }
    }

    if (param.visualize_path_distance != 0) {
        if (probe_a_axis){
            THECONVEYOR->wait_for_idle();
            coordinated_move(NAN, NAN, param.clearance_world_pos, param.rapid_rate );
            coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate );
            float delta[5];
            memset(delta, 0, sizeof(delta));
            delta[A_AXIS] = - THEKERNEL->probe_outputs[2];
            THEROBOT->delta_move(delta, param.rapid_rate, A_AXIS + 1);
            THECONVEYOR->wait_for_idle();
        }else{
        //probe to second position
        if (probe_x){
            xy_probe_move_alarm_when_hit(POS, 
                                        param.probe_g38_subcode, 
                                        THEROBOT->from_millimeters(param.visualize_path_distance * cos(THEKERNEL->probe_outputs[2] * pi/180)), 
                                        THEROBOT->from_millimeters(param.visualize_path_distance * sin(THEKERNEL->probe_outputs[2] * pi/180 )), 
                                        param.rapid_rate);
        }else{
            xy_probe_move_alarm_when_hit(POS, 
                                    param.probe_g38_subcode, 
                                    THEROBOT->from_millimeters(param.visualize_path_distance * cos((THEKERNEL->probe_outputs[2] + 90) * pi/180)), 
                                    THEROBOT->from_millimeters(param.visualize_path_distance * sin((THEKERNEL->probe_outputs[2] + 90) * pi/180 )), 
                                    param.rapid_rate);
        }
        //return to center position
        THECONVEYOR->wait_for_idle();
        coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate );
        }   
    }
    if (param.save_position == 1){
        if (probe_a_axis){
            THEROBOT->set_current_wcs_by_mpos(NAN,NAN,NAN,a_axis_pos - THEKERNEL->probe_outputs[2],NAN,NAN);
        }else{
        THEROBOT->set_current_wcs_by_mpos(NAN,NAN,NAN,NAN,NAN,THEKERNEL->probe_outputs[2]);
        }
    }
}

void ZProbe::probe_square(){
    THEKERNEL->streams->printf("Probing Square\n");

    float mpos[5];

    if (param.repeat < 1){
        THEKERNEL->streams->printf("ALARM: Probe fail: repeat value cannot be less than 1\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }

    rotate(X_AXIS, param.x_axis_distance, &param.x_rotated_x, &param.x_rotated_y, param.rotation_angle);
    rotate(Y_AXIS, param.y_axis_distance, &param.y_rotated_x, &param.y_rotated_y, param.rotation_angle);
    // always wait for idle before getting the machine pos
    THECONVEYOR->wait_for_idle();
    //save center position to use later
    THEROBOT->get_current_machine_position(mpos);
    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
    if(THEKERNEL->is_flex_compensation_active()) {
        if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, false); // get inverse compensation transform
    }

    // used for starting position
    out_coords.origin_x = mpos[0];
    out_coords.origin_y = mpos[1];
    out_coords.origin_z = mpos[2];

    float x_pos[4] = {0, 0, 0, 0};
    float y_pos[4] = {0, 0, 0, 0};
    float z_pos[4] = {out_coords.origin_z, out_coords.origin_z, out_coords.origin_z, out_coords.origin_z};

    param.z_axis_distance = -(fabs(param.probe_height));

    x_pos[0] = out_coords.origin_x;
    y_pos[0] = out_coords.origin_y;
    x_pos[1] = out_coords.origin_x + param.x_rotated_x;
    y_pos[1] = out_coords.origin_y + param.x_rotated_y;
    x_pos[2] = out_coords.origin_x + param.x_rotated_x + param.y_rotated_x;
    y_pos[2] = out_coords.origin_y + param.x_rotated_y + param.y_rotated_y;
    x_pos[3] = out_coords.origin_x + param.y_rotated_x;
    y_pos[3] = out_coords.origin_y + param.y_rotated_y;

    float min_z = 0;
    float max_z = 0;
    int min_z_index = 0;
    int max_z_index = 0;

    for (int i = 0; i < param.repeat; i++){
        // go to starting position
        coordinated_move(out_coords.origin_x, out_coords.origin_y, out_coords.origin_z, param.rapid_rate);
        for (int j = 0; j < 4; j++){
            fast_slow_probe_sequence(Z_AXIS, NEG);
            z_pos[j] = out_coords.z_negative_z_out;
            if (j == 0){
                min_z = max_z = z_pos[j];
                min_z_index = max_z_index = 1;
            }
            if (z_pos[j] < min_z){
                min_z = z_pos[j];
                min_z_index = j + 1;
            }
            if (z_pos[j] > max_z){
                max_z = z_pos[j];
                max_z_index = j + 1;
            }
            // move to next point
            coordinated_move(NAN, NAN, out_coords.origin_z, param.rapid_rate);
            if (j < 3){
                coordinated_move(x_pos[j+1], y_pos[j+1], NAN, param.rapid_rate);
            }else{
                coordinated_move(out_coords.origin_x, out_coords.origin_y, NAN, param.rapid_rate);
            }
        }
        THECONVEYOR->wait_for_idle();
        THEKERNEL->streams->printf("--- Machine Coordinates ---\n");
        for (int j = 0; j < 4; j++){
            THEKERNEL->streams->printf("Point %d: X: %.3f, Y: %.3f, Z: %.3f\n", j + 1, x_pos[j], y_pos[j], z_pos[j]);
        }
        THEKERNEL->streams->printf("Min Z: %.3f, Max Z: %.3f\n", min_z, max_z);
        THEKERNEL->streams->printf("--- Work Coordinates ---\n");
        for (int j = 0; j < 4; j++){
            mpos[0] = x_pos[j];
            mpos[1] = y_pos[j];
            mpos[2] = z_pos[j];
            mpos[3] = 0;
            mpos[4] = 0;
            Robot::wcs_t pos = THEROBOT->mcs2wcs(mpos);
            THEKERNEL->streams->printf("Point %d: X: %.3f, Y: %.3f, Z: %.3f\n", j + 1, std::get<X_AXIS>(pos), std::get<Y_AXIS>(pos), std::get<Z_AXIS>(pos));
            if (j == min_z_index - 1){
                min_z = std::get<Z_AXIS>(pos);
            }
            if (j == max_z_index - 1){
                max_z = std::get<Z_AXIS>(pos);
            }
        }
        THEKERNEL->streams->printf("Min Z: %.3f, Max Z: %.3f\n", min_z, max_z);
        THEKERNEL->streams->printf("Min Z Point#: %d, Max Z Point#: %d\n", min_z_index, max_z_index);
    }
}

void ZProbe::calibrate_probe_bore() //M460.1
{
    THECONVEYOR->wait_for_idle();
    THEKERNEL->streams->printf("Calibrating Probe With Bore\n");

    float knownDiameter = 0;

    if (param.x_axis_distance != 0){
        knownDiameter = param.x_axis_distance;
    }else{
        // if only Y is given set X to the same
        param.x_axis_distance = param.y_axis_distance;
    }
    if (param.y_axis_distance != 0){
        knownDiameter = param.y_axis_distance;
    }else{
        // if only X is given set Y to the same
        param.y_axis_distance = param.x_axis_distance;
    }

    if (param.repeat < 1){
        param.repeat = 1;
    }

    vector<float> probe_position_stack;
    THEKERNEL->probe_outputs[0] = 0;
    THEKERNEL->probe_outputs[1] = 0;

    for(int i=0; i< param.repeat; i++) {
        probe_bore(true);
        //delete gcodeBuffer;
        THECONVEYOR->wait_for_idle();
        // only Y because the first probe in X can be off center and therefore completely wrong
        probe_position_stack.push_back(THEKERNEL->probe_outputs[1]);
        param.rotation_angle += param.rotation_offset_per_probe;
    }
    
    float sum = 0.0;
    for (const auto& pos : probe_position_stack) {
        sum += pos;
    }
    float ave = sum / (param.repeat);
    
    THEKERNEL->streams->printf("Average bore diameter: %.3f\n", ave);
    
    // Don't write the tip diameter directly without the user knowing.
    //THEKERNEL->probe_tip_diameter = knownDiameter - ave;
    THEKERNEL->streams->printf("New Probe Tip Diameter is: %.3f\n", (knownDiameter - ave));
    if (param.save_position > 0){
        //TODO actually save the positon
    }else{
        THEKERNEL->streams->printf("This value is temporary \n and will neeed to be saved to the config file with \n config-set sd zprobe.probe_tip_diameter %.3f \n", (knownDiameter - ave));
    }
}

void ZProbe::calibrate_probe_boss() //M460.2
{
    THECONVEYOR->wait_for_idle();
    THEKERNEL->streams->printf("Calibrating Probe With Boss\n");

    float knownDiameter = 0;
    // Save original axis distances before they get modified by probe_boss()
    float original_x_axis_distance = param.x_axis_distance;
    float original_y_axis_distance = param.y_axis_distance;
    
    if (param.x_axis_distance != 0){
        knownDiameter = param.x_axis_distance;
    }
    if (param.y_axis_distance != 0){
        knownDiameter = param.y_axis_distance;
    }

    if (param.repeat < 1){
        param.repeat = 1;
    }

    vector<float> probe_position_stack;
    
    THEKERNEL->probe_outputs[0] = 0;
    THEKERNEL->probe_outputs[1] = 0;

    for(int i=0; i< param.repeat; i++) {
            // Restore original axis distances before each probe_boss() call
            // since probe_boss() modifies them by dividing by 2
            param.x_axis_distance = original_x_axis_distance;
            param.y_axis_distance = original_y_axis_distance;
            
            probe_boss(true);
            THECONVEYOR->wait_for_idle();
            
            if (original_x_axis_distance != 0){
                probe_position_stack.push_back(THEKERNEL->probe_outputs[0]);
            }else{
                probe_position_stack.push_back(THEKERNEL->probe_outputs[1]);
            }
    }
    
    float sum = 0.0;
    for (const auto& pos : probe_position_stack) {
        sum += pos;
    }
    float ave = sum / (param.repeat);
    
    THEKERNEL->streams->printf("Average boss distance: %.3f\n", ave);
    
    //THEKERNEL->probe_tip_diameter = ave - knownDiameter;
    THEKERNEL->streams->printf("New Probe Tip Diameter is: %.3f\n", (ave - knownDiameter));

    if (param.save_position > 0){
        //TODO actually save the positon
    }else{
        THEKERNEL->streams->printf("This value is temporary \n and will neeed to be saved to the config file with \n config-set sd zprobe.probe_tip_diameter %.3f \n", (ave - knownDiameter));
    }
}

void ZProbe::single_axis_probe_double_tap(){
    
    THECONVEYOR->wait_for_idle();
    THEKERNEL->streams->printf("Probing Single Axis\n");

    if (param.repeat < 1){
        THEKERNEL->streams->printf("ALARM: Probe fail: repeat value cannot be less than 1\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }

    // rotate the x y coordinates around z
    rotateXY(param.x_axis_distance, param.y_axis_distance, &param.x_rotated_x, &param.y_rotated_y, param.rotation_angle);
    // get the move length
    float move_distance = get_xyz_move_length(param.x_axis_distance, param.y_axis_distance, param.z_axis_distance);
    
    float tip_x = param.tool_dia/(2 * move_distance) * param.x_rotated_x;
    float tip_y = param.tool_dia/(2 * move_distance) * param.y_rotated_y;
    float tip_z = param.tool_dia/(2 * move_distance) * param.z_axis_distance;

    rotateXY(tip_x, tip_y, &tip_x, &tip_y, THEROBOT->r[THEROBOT->get_current_wcs()]);

    vector<float> probe_position_stack_x;
    vector<float> probe_position_stack_y;
    vector<float> probe_position_stack_z;

    for(int i=0; i < param.repeat; i++) {

        THECONVEYOR->wait_for_idle();
        // POS doesn't do anything here since it's a XYZ Probe move
        fast_slow_probe_sequence(XYZ, POS);

        THECONVEYOR->wait_for_idle();
        
        if (param.x_rotated_x != 0) {probe_position_stack_x.push_back(out_coords.x_positive_x_out);}
        if (param.y_rotated_y != 0) {probe_position_stack_y.push_back(out_coords.y_positive_y_out);}
        if (param.z_axis_distance != 0) {probe_position_stack_z.push_back(out_coords.z_negative_z_out);}
    }
    
    float sum = 0.0;
    if (!probe_position_stack_x.empty()){
        for (const auto& pos : probe_position_stack_x) {
            sum += pos;
        }
    }
    float ave_x = sum / (param.repeat) + tip_x;
    sum = 0.0;
    if (!probe_position_stack_y.empty()){
        for (const auto& pos : probe_position_stack_y) {
            sum += pos;
        }
    }
    float ave_y = sum / (param.repeat) + tip_y;
    sum = 0.0;
    if (!probe_position_stack_z.empty()){
        for (const auto& pos : probe_position_stack_z) {
            sum += pos;
        }
    }
    float ave_z = sum / (param.repeat) + (param.tool_dia/2 + tip_z);
    float mpos[5];
    mpos[0] = ave_x;
    mpos[1] = ave_y;
    mpos[2] = ave_z;
    mpos[3] = 0;
    mpos[4] = 0;
    Robot::wcs_t pos = THEROBOT->mcs2wcs(mpos);

    if ((param.x_axis_distance != 0 && param.y_axis_distance != 0) || param.rotation_angle != 0) {
        THEKERNEL->streams->printf("--- Machine Coordinates ---\n");
        THEKERNEL->streams->printf("Final Position: X:%.3f , Y:%.3f\n", ave_x , ave_y);
        THEKERNEL->streams->printf("--- Work Coordinates ---\n");
        THEKERNEL->streams->printf("Final Position: X:%.3f , Y:%.3f, Z:%.3f\n", std::get<X_AXIS>(pos), std::get<Y_AXIS>(pos), std::get<Z_AXIS>(pos));
        THEKERNEL->probe_outputs[3] = ave_x;
        THEKERNEL->probe_outputs[4] = ave_y;
        if (param.save_position > 0 && check_last_probe_ok()){
            THEROBOT->set_current_wcs_by_mpos( THEKERNEL->probe_outputs[3], THEKERNEL->probe_outputs[4], NAN);
        }
    } else if(param.x_axis_distance != 0){
        THEKERNEL->streams->printf("--- Machine Coordinates ---\n");
        THEKERNEL->streams->printf("Final Position X: %.3f\n", ave_x);
        THEKERNEL->streams->printf("--- Work Coordinates ---\n");
        THEKERNEL->streams->printf("Final Position X: %.3f\n", std::get<X_AXIS>(pos));
        THEKERNEL->probe_outputs[3] = ave_x;
        if (param.save_position > 0 && check_last_probe_ok()){
            THEROBOT->set_current_wcs_by_mpos( THEKERNEL->probe_outputs[3], NAN, NAN);
        }
    } else if(param.y_axis_distance != 0){
        THEKERNEL->streams->printf("--- Machine Coordinates ---\n");
        THEKERNEL->streams->printf("Final Position Y: %.3f\n", ave_y);
        THEKERNEL->streams->printf("--- Work Coordinates ---\n");
        THEKERNEL->streams->printf("Final Position Y: %.3f\n", std::get<Y_AXIS>(pos));
        THEKERNEL->probe_outputs[4] = ave_y;
        if (param.save_position > 0 && check_last_probe_ok()){
            THEROBOT->set_current_wcs_by_mpos( NAN, THEKERNEL->probe_outputs[4], NAN);
        }
    }
    if (param.z_axis_distance != 0){
        THEKERNEL->streams->printf("--- Machine Coordinates ---\n");
        THEKERNEL->streams->printf("Final Positon Z: %.3f\n", ave_z);
        THEKERNEL->streams->printf("--- Work Coordinates ---\n");
        THEKERNEL->streams->printf("Final Positon Z: %.3f\n", std::get<Z_AXIS>(pos));
        THEKERNEL->probe_outputs[5] = ave_z;
        if (param.save_position == 2 && check_last_probe_ok()){
            THEROBOT->set_current_wcs_by_mpos( NAN, NAN, THEKERNEL->probe_outputs[5]);
        }
    }
}

bool ZProbe::fast_slow_probe_sequence_public(int axis, int direction)
{
    return(fast_slow_probe_sequence(axis, direction));
}