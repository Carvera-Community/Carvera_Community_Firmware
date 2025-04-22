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

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

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

    // we read the probe in this timer
    probing = false;
    THEKERNEL->slow_ticker->attach(1000, this, &ZProbe::read_probe);
    THEKERNEL->slow_ticker->attach(1000, this, &ZProbe::read_calibrate);
	if(!(THEKERNEL->factory_set->FuncSetting & (1<<2)))	//Manual Tool change 
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
    if(isnan(this->max_z)){
        this->max_z = THEKERNEL->config->value(gamma_max_checksum)->by_default(200)->as_number(); // maximum zprobe distance
    }
    this->dwell_before_probing = THEKERNEL->config->value(zprobe_checksum, dwell_before_probing_checksum)->by_default(0)->as_number(); // dwell time in seconds before probing

}

uint32_t ZProbe::read_probe(uint32_t dummy)
{
    if (!probing || probe_detected) return 0;

    // we check all axis as it maybe a G38.2 X10 for instance, not just a probe in Z
    if(STEPPER[X_AXIS]->is_moving() || STEPPER[Y_AXIS]->is_moving() || STEPPER[Z_AXIS]->is_moving()) {
        // if it is moving then we check the probe, and debounce it
        if (this->pin.get() != invert_probe) {
            if (debounce < debounce_ms) {
                debounce ++;
            } else {
                // we signal the motors to stop, which will preempt any moves on that axis
                // we do all motors as it may be a delta
                for (auto &a : THEROBOT->actuators) a->stop_moving();
                probe_detected = true;
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
    if (!calibrating || calibrate_detected) return 0;

    // just check z Axis move
    if (STEPPER[X_AXIS]->is_moving() || STEPPER[Y_AXIS]->is_moving() || STEPPER[Z_AXIS]->is_moving()) {
    	if (this->pin.get()) {
    		probe_detected = true;
    	}
        // if it is moving then we check the toolsetter, and debounce it
        if (this->calibrate_pin.get()) {
            if (cali_debounce < debounce_ms) {
                cali_debounce++;
            } else {
                // we signal the motors to stop, which will preempt any moves on that axis
                // we do all motors as it may be a delta
                for (auto &a : THEROBOT->actuators) a->stop_moving();
                calibrate_detected = true;
                cali_debounce = 0;
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
    probe_detected = false;
    debounce = 0;
    cali_debounce = 0;

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

    if( gcode->has_g && gcode->g >= 29 && gcode->g <= 32) {

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
                calibrate_probe_bore(gcode);
                break;
            case 461:
                probe_bore(gcode);
                break;
            case 462:
                probe_boss(gcode);
                break;
            case 463:
                probe_insideCorner(gcode);
                break;
            case 464:
                probe_outsideCorner(gcode);
                break;
            case 465:
                probe_axisangle(gcode);
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

    // enable the probe checking in the timer
    probing = true;
    probe_detected = false;
    debounce = 0;
    cali_debounce = 0;

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

    uint8_t probeok= this->probe_detected ? 1 : 0;

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

// just probe / calibrate Z using calibrate pin
// This function is updated to leverage probe_XYZ code, accomodating 3d tool setter.
bool ZProbe::calibrate_Z(Gcode *gcode)
{
    float x = 0, y = 0, z = 0;
    if (gcode->has_letter('X')) {
        x = gcode->get_value('X');
    }

    if (gcode->has_letter('Y')) {
        y = gcode->get_value('Y');
    }

    if (gcode->has_letter('Z')) {
        z = gcode->get_value('Z');
    }

    if (x == 0 && y == 0 && z == 0) {
        gcode->stream->printf("error: at least one of X, Y, or Z must be specified, and be > or < 0\n");
        return false;
    }

    // get probe feedrate in mm/min and convert to mm/sec if specified
    float rate = (gcode->has_letter('F')) ? gcode->get_value('F') / 60 : this->slow_feedrate;

    // first wait for all moves to finish
    THEKERNEL->conveyor->wait_for_idle();

    if (this->calibrate_pin.get()) {
        gcode->stream->printf("Error: Tool calibration triggered before move, aborting command.\n");
        THEKERNEL->set_halt_reason(CALIBRATE_FAIL);
        THEKERNEL->call_event(ON_HALT, nullptr);
        return false;
    }

    // enable the probe checking in the timer
    calibrating = true;
    probe_detected = false;
    calibrate_detected = false;
    debounce = 0;
    cali_debounce = 0;

    // do a delta move which will stop as soon as the probe is triggered, or the distance is reached
    float delta[3] = {x, y, z};
    THEKERNEL->set_zprobing(true);
    if (!THEROBOT->delta_move(delta, rate, 3)) {
        gcode->stream->printf("ERROR: Move too small, X: %1.3f, Y: %1.3f, Z: %1.3f\n", x, y, z);
        THEKERNEL->set_halt_reason(CALIBRATE_FAIL);
        THEKERNEL->call_event(ON_HALT, nullptr);
        calibrating = false;
        THEKERNEL->set_zprobing(false);
        return false;
    }
    THEKERNEL->set_zprobing(false);

    // Wait until the move is finished
    THEKERNEL->conveyor->wait_for_idle();

    // disable probe checking
    calibrating = false;

    // if the probe stopped the move we need to correct the last_milestone as it did not reach where it thought
    // this also sets last_milestone to the machine coordinates it stopped at
    THEROBOT->reset_position_from_current_actuator_position();
    float pos[3];
    THEROBOT->get_axis_position(pos, 3);

    uint8_t calibrateok = this->calibrate_detected ? 1 : 0;

    // print results using the GRBL format
    gcode->stream->printf("[PRB:%1.3f,%1.3f,%1.3f:%d]\n", THEKERNEL->robot->from_millimeters(pos[X_AXIS]), THEKERNEL->robot->from_millimeters(pos[Y_AXIS]), THEKERNEL->robot->from_millimeters(pos[Z_AXIS]), calibrateok);
    THEROBOT->set_last_probe_position(std::make_tuple(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], calibrateok));

    if (calibrateok == 0 && gcode->subcode == 6) {
        // Issue error if probe was not triggered and subcode is 6
        gcode->stream->printf("ALARM: Calibrate fail!\n");
        THEKERNEL->set_halt_reason(CALIBRATE_FAIL);
        THEKERNEL->call_event(ON_HALT, nullptr);
        return false;
    }

    if (probe_detected) {
        this->probe_trigger_time = us_ticker_read();
    }

    return calibrateok == 1;
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

void ZProbe::probe_bore(Gcode *gcode) //G38.10
{
    THECONVEYOR->wait_for_idle();
    THEKERNEL->streams->printf("Probing Bore/Rectangular Pocket\n");
    float tool_dia = THEKERNEL->probe_tip_diameter;
    float probe_height = 0;
    float feed_rate = 300;
    float rapid_rate = 800;
    float x_axis_distance = 20;
    float y_axis_distance = 20;
    float roation_angle = 0;
    int repeat = 1;
    float retract_distance = 1.5;
    float clearance_height = 2;
    invert_probe = false;

    Gcode *gcodeBuffer; // = static_cast<Gcode *>(argument);
    float moveBuffer[3];
    char buff[100];

    if (!gcode->has_letter('X') && !gcode->has_letter('Y')){ //error if there is a problem
        gcode->stream->printf("ALARM: Probe fail: No Axis Set\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }

    if (gcode->has_letter('D')) { //probe tip diameter
        tool_dia = gcode->get_value('D');
    }
    if (gcode->has_letter('H')) { //probe height above bore/disance to move down before probing
        probe_height = gcode->get_value('H');
    }
    if (gcode->has_letter('C')){
        clearance_height = gcode->get_value('C');
    }
    if (gcode->has_letter('X')) { //radius x
        x_axis_distance = gcode->get_value('X');
    }
    if (gcode->has_letter('Y')) { //radius y
        y_axis_distance = gcode->get_value('Y');
    }
    if (gcode->has_letter('Q')) { //roation of pocket
        roation_angle = gcode->get_value('Q');
    }
    if (gcode->has_letter('F')) { //feed rate
        feed_rate = gcode->get_value('F');
    }
    if (gcode->has_letter('K')) { //probe height above bore/disance to move down before probing
        rapid_rate = gcode->get_value('K');
    }
    if (gcode->has_letter('L')) { //repeat touch off
        repeat = gcode->get_value('L');
    }
    if (gcode->has_letter('R')) { //retract distance
        retract_distance = gcode->get_value('R');
    }
    bool save_position = false;
    if (gcode->has_letter('S')) { //retract distance
        save_position = (gcode->get_value('S')!= 0);
    }

    if (repeat < 1){
        gcode->stream->printf("ALARM: Probe fail: repeat value cannot be less than 1\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }

	//slow zprobe without alarm to probe_height. Skip if probe height is 0
	if (probe_height != 0){
		// do z probe with slow speed
        THEKERNEL->streams->printf("Probing Z with a distance of %.3f\n", probe_height);

        std::sprintf(buff, "G38.3 Z%.3f F%.3f", THEROBOT->from_millimeters(-probe_height), feed_rate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        if (probe_XYZ(gcodeBuffer)){ //probe down, if bottom surface reached, retract slightly
            THEKERNEL->streams->printf("Probed surface hit");
            moveBuffer[0] = 0;
            moveBuffer[1] = 0;
            moveBuffer[2] = THEROBOT->from_millimeters(clearance_height);
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            THECONVEYOR->wait_for_idle();
        }
        delete gcodeBuffer;
    }

    float sin_angle = sin(roation_angle*pi/180);
	float cos_angle = cos(roation_angle*pi/180);

	//negative values are doubled as they start from probe from the positive location
	float x_positive_x = x_axis_distance * cos_angle; //with 0 rotation this is x positive
	float x_positive_y = x_axis_distance * sin_angle;
	float x_negative_x = -x_positive_x*2; //with 0 rotation this is x negative
	float x_negative_y = -x_positive_y*2;
	float y_positive_x = y_axis_distance * sin_angle; //with 0 rotation this is y positive
	float y_positive_y = y_axis_distance * -cos_angle;
	float y_negative_x = -y_positive_x*2; //with 0 rotation this is y negative
	float y_negative_y = -y_positive_y*2;

    
	//output points
	float x_positive_x_out = 0;
	float x_positive_y_out = 0;
	float x_negative_x_out = 0;
	float x_negative_y_out = 0;
	float y_positive_x_out = 0;
	float y_positive_y_out = 0;
	float y_negative_x_out = 0;
	float y_negative_y_out = 0;

	float center_x;
	float center_y;
    float mpos[3];
    float old_mpos[3];

    float slowZprobeRate = 50;

    //save center position to use later
    THEROBOT->get_current_machine_position(mpos);
    memcpy(old_mpos, mpos, sizeof(mpos));
    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
    if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
    center_x = old_mpos[0];
    center_y = old_mpos[1];


	//setup repeat
	for(int i=0; i< repeat; i++) {

        if (gcode->has_letter('X')) {
            // do positive x probe
            float retractx = THEROBOT->from_millimeters((x_axis_distance>= 0 ? 1.0f : -1.0f) * -retract_distance * cos_angle);
            float retracty = THEROBOT->from_millimeters((x_axis_distance>= 0 ? 1.0f : -1.0f) * -retract_distance * sin_angle);
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_positive_x), THEROBOT->from_millimeters(x_positive_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //move off the surface
            moveBuffer[0] = retractx;
            moveBuffer[1] = retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            //slow probe
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_positive_x), THEROBOT->from_millimeters(x_positive_y), slowZprobeRate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //store position
            THEROBOT->get_current_machine_position(mpos);
            memcpy(old_mpos, mpos, sizeof(mpos));
            // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
            if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
            x_positive_x_out = old_mpos[0];
            x_positive_y_out = old_mpos[1];
            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", x_positive_x_out, x_positive_y_out);
            //move back to the center position
            coordinated_move(center_x, center_y, NAN, rapid_rate );
            
            // do negative x probe
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(2*x_negative_x), THEROBOT->from_millimeters(2*x_negative_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //move off the surface
            moveBuffer[0] = -retractx;
            moveBuffer[1] = -retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            //slow probe
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_negative_x), THEROBOT->from_millimeters(x_negative_y), slowZprobeRate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //store position
            THEROBOT->get_current_machine_position(mpos);
            memcpy(old_mpos, mpos, sizeof(mpos));
            // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
            if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
            x_negative_x_out = old_mpos[0];
            x_negative_y_out = old_mpos[1];
            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", x_negative_x_out, x_negative_y_out);
            //calculate center of bore (will only be centered in x)
            center_x = (x_positive_x_out + x_negative_x_out)/2;
            center_y = (x_positive_y_out + x_negative_y_out)/2;
            //goto current center position
            coordinated_move(center_x, center_y, NAN, rapid_rate );
            THECONVEYOR->wait_for_idle();
            THEKERNEL->probe_outputs[0] = sqrt((x_positive_x_out - x_negative_x_out) * (x_positive_x_out - x_negative_x_out) + (x_positive_y_out - x_negative_y_out) * (x_positive_y_out - x_negative_y_out) ) + tool_dia;
            THEKERNEL->streams->printf("Distance Point 2 X surfaces (Diameter) is: %.3f and center is stored at variable #151\n" , THEKERNEL->probe_outputs[0] );
            
        }

        if (gcode->has_letter('Y')) {
            float retractx = THEROBOT->from_millimeters((y_axis_distance>= 0 ? 1.0f : -1.0f) * -retract_distance * sin_angle);
            float retracty = THEROBOT->from_millimeters((y_axis_distance>= 0 ? 1.0f : -1.0f) * -retract_distance * -cos_angle);
            // do positive y probe
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_positive_x), THEROBOT->from_millimeters(y_positive_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //move off the surface
            moveBuffer[0] = retractx;
            moveBuffer[1] = retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            //slow probe
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_positive_x), THEROBOT->from_millimeters(y_positive_y), slowZprobeRate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //store position
            THEROBOT->get_current_machine_position(mpos);
            memcpy(old_mpos, mpos, sizeof(mpos));
            // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
            if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
            y_positive_x_out = old_mpos[0];
            y_positive_y_out = old_mpos[1];
            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", y_positive_x_out, y_positive_y_out);
            //goto current center position
            coordinated_move(center_x, center_y, NAN, rapid_rate );
            THECONVEYOR->wait_for_idle();
            // do negative x probe
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_negative_x), THEROBOT->from_millimeters(y_negative_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //move off the surface
            moveBuffer[0] = -retractx;
            moveBuffer[1] = -retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            //slow probe
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_negative_x), THEROBOT->from_millimeters(y_negative_y), slowZprobeRate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //store position
            THEROBOT->get_current_machine_position(mpos);
            memcpy(old_mpos, mpos, sizeof(mpos));
            // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
            if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
            y_negative_x_out = old_mpos[0];
            y_negative_y_out = old_mpos[1];
            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", x_negative_x_out, x_negative_y_out);
            //calculate center of bore (will only be centered in x)
            center_x = (y_positive_x_out + y_negative_x_out)/2;
            center_y = (y_positive_y_out + y_negative_y_out)/2;
            //goto current center position
            coordinated_move(center_x, center_y, NAN, rapid_rate );
            THECONVEYOR->wait_for_idle();
            THEKERNEL->probe_outputs[1] = sqrt((y_positive_x_out - y_negative_x_out) * (y_positive_x_out - y_negative_x_out) + (y_positive_y_out - y_negative_y_out) * (y_positive_y_out - y_negative_y_out) ) + tool_dia;
            THEKERNEL->streams->printf("Distance Betweeen 2 Y surfaces (Diameter) is: %.3f and is stored at variable #152\n" , THEKERNEL->probe_outputs[1] );
        }

		
	}
	THEKERNEL->streams->printf("Center of bore or rectangular pocket found. Ready to Zero X and Y\n");
    THEKERNEL->probe_outputs[3] = center_x;
    THEKERNEL->probe_outputs[4] = center_y;
    THEKERNEL->streams->printf("Center Point is: %.3f , %.3f and is stored in MCS as #154,#155\n" , THEKERNEL->probe_outputs[3],THEKERNEL->probe_outputs[4] );

    if (save_position){ 
        THEROBOT->set_current_wcs_by_mpos( THEKERNEL->probe_outputs[3], THEKERNEL->probe_outputs[4], NAN);
    }
    
}

void ZProbe::probe_boss(Gcode *gcode) //M461
{
    THECONVEYOR->wait_for_idle();
    THEKERNEL->streams->printf("Probing Boss or Rectangular Block\n");
    float tool_dia = THEKERNEL->probe_tip_diameter;
    float probe_height = 0;
    float feed_rate = 300;
    float rapid_rate = 800;
    float x_axis_distance = 20;
    float y_axis_distance = 20;
    float roation_angle = 0;
    int repeat = 1;
    float retract_distance = 1.5;
    float clearance_height = 2;
    float side_depth = 2;
    float clearance_world_pos;
    invert_probe = false;

    Gcode *gcodeBuffer; // = static_cast<Gcode *>(argument);
    float moveBuffer[3];
    char buff[100];

    if (!gcode->has_letter('X') && !gcode->has_letter('Y')){ //error if there is a problem
        gcode->stream->printf("ALARM: Probe fail: No Axis Set\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }

    if (gcode->has_letter('D')) { //probe tip diameter
        tool_dia = gcode->get_value('D');
    }
    if (gcode->has_letter('E')) { //depth to probe sides from
        side_depth = gcode->get_value('E');
    }
    if (gcode->has_letter('H')) { //probe height above bore/disance to move down before probing
        probe_height = gcode->get_value('H');
    }
    if (gcode->has_letter('C')){
        clearance_height = gcode->get_value('C');
    }
    if (gcode->has_letter('X')) { //radius x
        x_axis_distance = gcode->get_value('X');
    }
    if (gcode->has_letter('Y')) { //radius y
        y_axis_distance = gcode->get_value('Y');
    }
    if (gcode->has_letter('Q')) { //roation of pocket
        roation_angle = gcode->get_value('Q');
    }
    if (gcode->has_letter('F')) { //feed rate
        feed_rate = gcode->get_value('F');
    }
    if (gcode->has_letter('K')) { //probe height above bore/disance to move down before probing
        rapid_rate = gcode->get_value('K');
    }
    if (gcode->has_letter('L')) { //repeat touch off
        repeat = gcode->get_value('L');
    }
    if (gcode->has_letter('R')) { //retract distance
        retract_distance = gcode->get_value('R');
    }
    bool save_position = false;
    if (gcode->has_letter('S')) { //retract distance
        save_position = (gcode->get_value('S')!= 0);
    }

    if (repeat < 1){
        gcode->stream->printf("ALARM: Probe fail: repeat value cannot be less than 1\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }
	//slow zprobe without alarm to probe_height. Skip if probe height is 0
	if (probe_height != 0){
		// do z probe with slow speed
        THEKERNEL->streams->printf("Probing Z with a distance of %.3f\n", probe_height);

        std::sprintf(buff, "G38.3 Z%.3f F%.3f", THEROBOT->from_millimeters(-probe_height), feed_rate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        if (probe_XYZ(gcodeBuffer)){ //probe down, if bottom surface reached, retract slightly
            THEKERNEL->streams->printf("Probed surface hit");
            moveBuffer[0] = 0;
            moveBuffer[1] = 0;
            moveBuffer[2] = THEROBOT->from_millimeters(clearance_height);
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            THECONVEYOR->wait_for_idle();
        }
        delete gcodeBuffer;
    }

    float sin_angle = sin(roation_angle*pi/180);
	float cos_angle = cos(roation_angle*pi/180);

	//negative values are doubled as they start from probe from the positive location
	float x_positive_x = x_axis_distance * cos_angle; //with 0 rotation this is x positive
	float x_positive_y = x_axis_distance * sin_angle;
	float x_negative_x = -x_positive_x; //with 0 rotation this is x negative
	float x_negative_y = -x_positive_y;
	float y_positive_x = y_axis_distance * sin_angle; //with 0 rotation this is y positive
	float y_positive_y = y_axis_distance * -cos_angle;
	float y_negative_x = -y_positive_x; //with 0 rotation this is y negative
	float y_negative_y = -y_positive_y;

    
	//output points
	float x_positive_x_out = 0;
	float x_positive_y_out = 0;
	float x_negative_x_out = 0;
	float x_negative_y_out = 0;
	float y_positive_x_out = 0;
	float y_positive_y_out = 0;
	float y_negative_x_out = 0;
	float y_negative_y_out = 0;

	float center_x;
	float center_y;
    float mpos[3];
    float old_mpos[3];

    float slowZprobeRate = 50;

    //save center position to use later
    THEROBOT->get_current_machine_position(mpos);
    memcpy(old_mpos, mpos, sizeof(mpos));
    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
    if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
    center_x = mpos[0];
    center_y = mpos[1];
    clearance_world_pos = mpos[2]; //test old_mpos[2];


	//setup repeat
	for(int i=0; i< repeat; i++) {
        
        //goto clearance height
        coordinated_move(NAN, NAN, clearance_world_pos, rapid_rate);
        THECONVEYOR->wait_for_idle();
        if (gcode->has_letter('X')) {
            float retractx = THEROBOT->from_millimeters((x_axis_distance>= 0 ? 1.0f : -1.0f) * retract_distance * cos_angle);
            float retracty = THEROBOT->from_millimeters((x_axis_distance>= 0 ? 1.0f : -1.0f) *retract_distance * sin_angle);
            // do positive x probe
            //probe no hit alarm x_positive - G38.3, alarm if true
            std::sprintf(buff, "G38.3 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_positive_x), THEROBOT->from_millimeters(x_positive_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            if (probe_XYZ(gcodeBuffer)){
                gcode->stream->printf("ALARM: Probe hit wall when moving to outer position\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                delete gcodeBuffer;
                return;
            }
            delete gcodeBuffer;
            //probe z no hit no alarm -side_depth, retract slightly if probe point hit - G38.3, retract if true
            std::sprintf(buff, "G38.3 Z%.3f F%.3f", THEROBOT->from_millimeters(-(side_depth + clearance_height)), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            if (probe_XYZ(gcodeBuffer)){
                moveBuffer[0] = 0;
                moveBuffer[1] = 0;
                moveBuffer[2] = THEROBOT->from_millimeters(1);
                THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            }
            delete gcodeBuffer;
            // probe alarm x_negative - G38.2
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_negative_x), THEROBOT->from_millimeters(x_negative_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //retract x_positive
            moveBuffer[0] = retractx;
            moveBuffer[1] = retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            //slow probe
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_negative_x), THEROBOT->from_millimeters(x_negative_y), slowZprobeRate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //store position
            THEROBOT->get_current_machine_position(mpos);
            memcpy(old_mpos, mpos, sizeof(mpos));
            // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
            if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
            x_positive_x_out = old_mpos[0];
            x_positive_y_out = old_mpos[1];
            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", x_positive_x_out, x_positive_y_out);
            //retract x_positive
            moveBuffer[0] = retractx;
            moveBuffer[1] = retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            //goto clearance_world_pos in z
            coordinated_move(NAN, NAN, clearance_world_pos, rapid_rate);
            THECONVEYOR->wait_for_idle();
            //return to center position
            coordinated_move(center_x, center_y, NAN, rapid_rate );

            //probe no hit alarm x_negative
            std::sprintf(buff, "G38.3 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_negative_x), THEROBOT->from_millimeters(x_negative_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            if (probe_XYZ(gcodeBuffer)){
                gcode->stream->printf("ALARM: Probe hit wall when moving to outer position\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                delete gcodeBuffer;
                return;
            }
            delete gcodeBuffer;
            //probe z no hit no alarm -side_depth, retract slightly if probe point hit
            std::sprintf(buff, "G38.3 Z%.3f F%.3f", THEROBOT->from_millimeters(-(side_depth + clearance_height)), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            if (probe_XYZ(gcodeBuffer)){
                moveBuffer[0] = 0;
                moveBuffer[1] = 0;
                moveBuffer[2] = THEROBOT->from_millimeters(1);
                THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            }
            delete gcodeBuffer;
            // probe alarm x_positive
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_positive_x), THEROBOT->from_millimeters(x_positive_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //retract x_negative
            moveBuffer[0] = -retractx;
            moveBuffer[1] = -retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            //slow probe
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_positive_x), THEROBOT->from_millimeters(x_positive_y), slowZprobeRate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //store position
            THEROBOT->get_current_machine_position(mpos);
            memcpy(old_mpos, mpos, sizeof(mpos));
            // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
            if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
            x_negative_x_out = old_mpos[0];
            x_negative_y_out = old_mpos[1];
            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", x_positive_x_out, x_positive_y_out);
            //calculate center of bore (will only be centered in x)
            center_x = (x_positive_x_out + x_negative_x_out)/2;
            center_y = (x_positive_y_out + x_negative_y_out)/2;
            //retract x_negative
            moveBuffer[0] = -retractx;
            moveBuffer[1] = -retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            //goto clearance_world_pos in z
            coordinated_move(NAN, NAN, clearance_world_pos, rapid_rate);
            THECONVEYOR->wait_for_idle();
            //goto current center position
            coordinated_move(center_x, center_y, NAN, rapid_rate );
            THECONVEYOR->wait_for_idle();
            THEKERNEL->probe_outputs[0] = sqrt((x_positive_x_out - x_negative_x_out) * (x_positive_x_out - x_negative_x_out) + (x_positive_y_out - x_negative_y_out) * (x_positive_y_out - x_negative_y_out) ) - tool_dia;
            THEKERNEL->streams->printf("Distance Betweeen 2 X surfaces (Diameter) is: %.3f and is stored at variable #151\n" , THEKERNEL->probe_outputs[0] );
        }

        if (gcode->has_letter('Y')) {
            float retractx = THEROBOT->from_millimeters((y_axis_distance>= 0 ? 1.0f : -1.0f) * retract_distance * sin_angle);
            float retracty = THEROBOT->from_millimeters((y_axis_distance>= 0 ? 1.0f : -1.0f) *retract_distance * -cos_angle);
            // do positive y probe
            //probe no hit alarm y_positive - G38.3, alarm if true
            std::sprintf(buff, "G38.3 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_positive_x), THEROBOT->from_millimeters(y_positive_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            if (probe_XYZ(gcodeBuffer)){
                gcode->stream->printf("ALARM: Probe hit wall when moving to outer position\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                delete gcodeBuffer;
                return;
            }
            delete gcodeBuffer;
            //probe z no hit no alarm -side_depth, retract slightly if probe point hit - G38.3, retract if true
            std::sprintf(buff, "G38.3 Z%.3f F%.3f", THEROBOT->from_millimeters(-(side_depth + clearance_height)), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            if (probe_XYZ(gcodeBuffer)){
                moveBuffer[0] = 0;
                moveBuffer[1] = 0;
                moveBuffer[2] = THEROBOT->from_millimeters(1);
                THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            }
            delete gcodeBuffer;
            // probe alarm y_negative - G38.2
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_negative_x), THEROBOT->from_millimeters(y_negative_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //retract y_positive
            moveBuffer[0] = retractx;
            moveBuffer[1] = retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            //slow probe
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_negative_x), THEROBOT->from_millimeters(y_negative_y), slowZprobeRate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //store position
            THEROBOT->get_current_machine_position(mpos);
            memcpy(old_mpos, mpos, sizeof(mpos));
            // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
            if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
            y_positive_x_out = old_mpos[0];
            y_positive_y_out = old_mpos[1];
            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", y_positive_x_out, y_positive_y_out);
            //retract y_positive
            moveBuffer[0] = retractx;
            moveBuffer[1] = retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            //goto clearance_world_pos in z
            coordinated_move(NAN, NAN, clearance_world_pos, rapid_rate);
            THECONVEYOR->wait_for_idle();
            //goto current center position
            coordinated_move(center_x, center_y, NAN, rapid_rate );

            //probe no hit alarm y_negative
            std::sprintf(buff, "G38.3 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_negative_x), THEROBOT->from_millimeters(y_negative_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            if (probe_XYZ(gcodeBuffer)){
                gcode->stream->printf("ALARM: Probe hit wall when moving to outer position\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                delete gcodeBuffer;
                return;
            }
            delete gcodeBuffer;
            //probe z no hit no alarm -side_depth, retract slightly if probe point hit
            std::sprintf(buff, "G38.3 Z%.3f F%.3f", THEROBOT->from_millimeters(-(side_depth + clearance_height)), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            if (probe_XYZ(gcodeBuffer)){
                moveBuffer[0] = 0;
                moveBuffer[1] = 0;
                moveBuffer[2] = THEROBOT->from_millimeters(1);
                THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            }
            delete gcodeBuffer;
            // probe alarm y_positive
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_positive_x), THEROBOT->from_millimeters(y_positive_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //retract y_negative
            moveBuffer[0] = -retractx;
            moveBuffer[1] = -retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            //slow probe
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_positive_x), THEROBOT->from_millimeters(y_positive_y), slowZprobeRate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //store position
            THEROBOT->get_current_machine_position(mpos);
            memcpy(old_mpos, mpos, sizeof(mpos));
            // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
            if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
            y_negative_x_out = old_mpos[0];
            y_negative_y_out = old_mpos[1];
            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", y_positive_x_out, y_positive_y_out);
            //calculate center of bore (will only be centered in x)
            center_x = (y_positive_x_out + y_negative_x_out)/2;
            center_y = (y_positive_y_out + y_negative_y_out)/2;
            //retract y_negative
            moveBuffer[0] = -retractx;
            moveBuffer[1] = -retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            //goto clearance_world_pos in z
            coordinated_move(NAN, NAN, clearance_world_pos, rapid_rate);
            THECONVEYOR->wait_for_idle();
            //goto current center position
            coordinated_move(center_x, center_y, NAN, rapid_rate );
            THECONVEYOR->wait_for_idle();
            THEKERNEL->probe_outputs[1] = sqrt((y_positive_x_out - y_negative_x_out) * (y_positive_x_out - y_negative_x_out) + (y_positive_y_out - y_negative_y_out) * (y_positive_y_out - y_negative_y_out) ) - tool_dia;
            THEKERNEL->streams->printf("Distance Betweeen 2 Y surfaces (Diameter) is: %.3f and is stored at variable #152\n" , THEKERNEL->probe_outputs[1] );
        }

		
	}
	THEKERNEL->streams->printf("Center of Boss or Rectangular Block found. Ready to Zero X and Y\n");
    THEKERNEL->probe_outputs[3] = center_x;
    THEKERNEL->probe_outputs[4] = center_y;
    THEKERNEL->streams->printf("Center Point is: %.3f , %.3f and is stored in MCS as #154,#155\n" , THEKERNEL->probe_outputs[3],THEKERNEL->probe_outputs[4] );

    if (save_position){
        THEROBOT->set_current_wcs_by_mpos( THEKERNEL->probe_outputs[3], THEKERNEL->probe_outputs[4], NAN);
    }
}

void ZProbe::probe_insideCorner(Gcode *gcode) //M462
{
    THECONVEYOR->wait_for_idle();
    THEKERNEL->streams->printf("Probing Inside Corner\n");
    float tool_dia = THEKERNEL->probe_tip_diameter;
    float probe_height = 0;
    float feed_rate = 300;
    float rapid_rate = 800;
    float x_axis_distance = 20;
    float y_axis_distance = 20;
    float roation_angle = 0;
    int repeat = 1;
    invert_probe = false;

    float retract_distance = 1.5;
    float clearance_height = 2;

    Gcode *gcodeBuffer;
    float moveBuffer[3];
    char buff[100];

    if (!gcode->has_letter('X') || !gcode->has_letter('Y')){
        gcode->stream->printf("ALARM: Probe fail: Both X and Y axis need to be set for Corner Probing\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }

    if (gcode->has_letter('D')) { //probe tip diameter
        tool_dia = gcode->get_value('D');
    }
    if (gcode->has_letter('H')) { //probe height above bore/disance to move down before probing
        probe_height = gcode->get_value('H');
    }
    if (gcode->has_letter('C')){
        clearance_height = gcode->get_value('C');
    }
    if (gcode->has_letter('X')) { //radius x
        x_axis_distance = gcode->get_value('X');
    }
    if (gcode->has_letter('Y')) { //radius y
        y_axis_distance = - gcode->get_value('Y');
    }
    if (gcode->has_letter('Q')) { //roation of pocket disabled for now due to a bug with the final output
        //roation_angle = gcode->get_value('Q'); 
    }
    if (gcode->has_letter('F')) { //feed rate
        feed_rate = gcode->get_value('F');
    }
    if (gcode->has_letter('K')) { //probe height above bore/disance to move down before probing
        rapid_rate = gcode->get_value('K');
    }
    if (gcode->has_letter('L')) { //repeat touch off
        repeat = gcode->get_value('L');
    }
    if (gcode->has_letter('R')) { //retract distance
        retract_distance = gcode->get_value('R');
    }
    bool save_position = false;
    if (gcode->has_letter('S')) { //retract distance
        save_position = (gcode->get_value('S')!= 0);
    }

    if (repeat < 1){
        gcode->stream->printf("ALARM: Probe fail: repeat value cannot be less than 1\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }
    
	//slow zprobe without alarm to probe_height. Skip if probe height is 0
	if (probe_height != 0){
		// do z probe with slow speed
        THEKERNEL->streams->printf("Probing Z with a distance of %.3f\n", probe_height);

        std::sprintf(buff, "G38.3 Z%.3f F%.3f", THEROBOT->from_millimeters(-probe_height), feed_rate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        if (probe_XYZ(gcodeBuffer)){ //probe down, if bottom surface reached, retract slightly
            THEKERNEL->streams->printf("Probed surface hit");
            moveBuffer[0] = 0;
            moveBuffer[1] = 0;
            moveBuffer[2] = THEROBOT->from_millimeters(clearance_height);
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            THECONVEYOR->wait_for_idle();
        }
        delete gcodeBuffer;
    }

    float sin_angle = sin(roation_angle*pi/180);
	float cos_angle = cos(roation_angle*pi/180);

	//negative values are doubled as they start from probe from the positive location
	float x_positive_x = x_axis_distance * cos_angle; //with 0 rotation this is x positive
	float x_positive_y = x_axis_distance * sin_angle;
	float y_positive_x = y_axis_distance * sin_angle; //with 0 rotation this is y positive
	float y_positive_y = y_axis_distance * -cos_angle;

    
	//output points
	float x_positive_x_out = 0;
	float y_positive_y_out = 0;
    float x_positive_y_out = 0;
	float y_positive_x_out = 0;

	float center_x;
	float center_y;
    float mpos[3];
    float old_mpos[3];

    float slowZprobeRate = 50;

    //save center position to use later
    THEROBOT->get_current_machine_position(mpos);
    memcpy(old_mpos, mpos, sizeof(mpos));
    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
    if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
    center_x = old_mpos[0];
    center_y = old_mpos[1];


	//setup repeat
	for(int i=0; i< repeat; i++) {
        float retractx = THEROBOT->from_millimeters((x_axis_distance>= 0 ? 1.0f : -1.0f) * -retract_distance * cos_angle);
        float retracty = THEROBOT->from_millimeters((x_axis_distance>= 0 ? 1.0f : -1.0f) * -retract_distance * sin_angle);
        // do positive x probe
        std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_positive_x), THEROBOT->from_millimeters(x_positive_y), feed_rate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        probe_XYZ(gcodeBuffer);
        delete gcodeBuffer;
        //move off the surface
        moveBuffer[0] = retractx;
        moveBuffer[1] = retracty;
        moveBuffer[2] = 0;
        THEROBOT->delta_move(moveBuffer, feed_rate, 3);
        //slow probe
        std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_positive_x), THEROBOT->from_millimeters(x_positive_y), slowZprobeRate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        probe_XYZ(gcodeBuffer);
        delete gcodeBuffer;
        //store position
        THEROBOT->get_current_machine_position(mpos);
        memcpy(old_mpos, mpos, sizeof(mpos));
        // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
        if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
        x_positive_x_out = old_mpos[0] - tool_dia  * cos_angle/ 2;
        x_positive_y_out = old_mpos[1] - tool_dia * sin_angle /2;
        //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", x_positive_x_out, x_positive_y_out);

        //goto current center position
        coordinated_move(center_x, center_y, NAN, rapid_rate );
        THECONVEYOR->wait_for_idle();

        retractx = THEROBOT->from_millimeters((y_axis_distance>= 0 ? 1.0f : -1.0f) * -retract_distance * sin_angle);
        retracty = THEROBOT->from_millimeters((y_axis_distance>= 0 ? 1.0f : -1.0f) * -retract_distance * -cos_angle);

        // do positive y probe
        std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_positive_x), THEROBOT->from_millimeters(y_positive_y), feed_rate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        probe_XYZ(gcodeBuffer);
        delete gcodeBuffer;
        //move off the surface
        moveBuffer[0] = retractx;
        moveBuffer[1] = retracty;
        moveBuffer[2] = 0;
        THEROBOT->delta_move(moveBuffer, feed_rate, 3);
        //slow probe
        std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_positive_x), THEROBOT->from_millimeters(y_positive_y), slowZprobeRate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        probe_XYZ(gcodeBuffer);
        delete gcodeBuffer;
        //store position
        THEROBOT->get_current_machine_position(mpos);
        memcpy(old_mpos, mpos, sizeof(mpos));
        // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
        if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
        y_positive_y_out = old_mpos[1] - ((y_axis_distance>= 0 ? 1.0f : -1.0f) * -retract_distance * -cos_angle)/2;
        y_positive_x_out = old_mpos[0] - ((y_axis_distance>= 0 ? 1.0f : -1.0f) * -retract_distance * -sin_angle)/2;
        //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", y_positive_x_out, y_positive_y_out);
        //goto current center position
        coordinated_move(center_x, center_y, NAN, rapid_rate );
        THECONVEYOR->wait_for_idle();
        
        // calculate center position
        if (roation_angle == 0)
        {
            THEKERNEL->probe_outputs[3] = x_positive_x_out;
            THEKERNEL->probe_outputs[4] = y_positive_y_out;
        }
        else //this has a bug in it and is disabled for now.
        {
            float lines_m_value = tan(roation_angle * pi / 180); //bug
            float lines_b_value = x_positive_y_out + x_positive_x_out * lines_m_value;
            float lines_c_value = y_positive_y_out - y_positive_x_out * lines_m_value;
            THEKERNEL->probe_outputs[3] = (lines_c_value - lines_b_value) / (2 * -lines_m_value); // x_out
            THEKERNEL->probe_outputs[4] = (lines_c_value - lines_b_value) / 2 + lines_b_value;   // y_out
            THEKERNEL->streams->printf("x_positive_y: %.3f\n", x_positive_y_out);
            THEKERNEL->streams->printf("x_positive_x: %.3f\n", x_positive_x_out);
            THEKERNEL->streams->printf("y_positive_y: %.3f\n", y_positive_y_out);
            THEKERNEL->streams->printf("y_positive_x: %.3f\n", y_positive_x_out);

            THEKERNEL->streams->printf("lines m: %.3f\n", lines_m_value);
            THEKERNEL->streams->printf("lines b: %.3f\n", lines_b_value);
            THEKERNEL->streams->printf("lines c: %.3f\n", lines_c_value);
            THEKERNEL->streams->printf("#154: %.3f\n", THEKERNEL->probe_outputs[3]);
            THEKERNEL->streams->printf("#155: %.3f\n", THEKERNEL->probe_outputs[4]);
        }

	}
    THEKERNEL->streams->printf("Corner found. X coordinate stored in #154 as MCS %.3f , Y coordinate in #155 as MCS %.3f \n", THEKERNEL->probe_outputs[3], THEKERNEL->probe_outputs[4] );
    
    if (save_position){
        THEROBOT->set_current_wcs_by_mpos( THEKERNEL->probe_outputs[3], THEKERNEL->probe_outputs[4], NAN);
    }
}
void ZProbe::probe_outsideCorner(Gcode *gcode) //M463
{
    THECONVEYOR->wait_for_idle();
    THEKERNEL->streams->printf("Probing Outside Corner\n");
    float tool_dia = THEKERNEL->probe_tip_diameter;
    float probe_height = 0;
    float feed_rate = 300;
    float rapid_rate = 800;
    float x_axis_distance = 20;
    float y_axis_distance = 20;
    float roation_angle = 0;
    int repeat = 1;
    float retract_distance = 1.5;
    float clearance_height = 2;
    float side_depth = 2;
    float clearance_world_pos;
    invert_probe = false;

    Gcode *gcodeBuffer; // = static_cast<Gcode *>(argument);
    float moveBuffer[3];
    char buff[100];

    if (!gcode->has_letter('X') && !gcode->has_letter('Y')){ //error if there is a problem
        gcode->stream->printf("ALARM: Probe fail: No Axis Set\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }

    if (gcode->has_letter('D')) { //probe tip diameter
        tool_dia = gcode->get_value('D');
    }
    if (gcode->has_letter('E')) { //depth to probe sides from
        side_depth = gcode->get_value('E');
    }
    if (gcode->has_letter('H')) { //probe height above bore/disance to move down before probing
        probe_height = gcode->get_value('H');
    }
    if (gcode->has_letter('C')){
        clearance_height = gcode->get_value('C');
    }
    if (gcode->has_letter('X')) { //radius x
        x_axis_distance = - gcode->get_value('X');
    }
    if (gcode->has_letter('Y')) { //radius y
        y_axis_distance = gcode->get_value('Y');
    }
    if (gcode->has_letter('Q')) { //roation of pocket disabled for now due to a bug with the final output
        //roation_angle = gcode->get_value('Q');
    }
    if (gcode->has_letter('F')) { //feed rate
        feed_rate = gcode->get_value('F');
    }
    if (gcode->has_letter('K')) { //probe height above bore/disance to move down before probing
        rapid_rate = gcode->get_value('K');
    }
    if (gcode->has_letter('L')) { //repeat touch off
        repeat = gcode->get_value('L');
    }
    if (gcode->has_letter('R')) { //retract distance
        retract_distance = gcode->get_value('R');
    }
    bool save_position = false;
    if (gcode->has_letter('S')) { //retract distance
        save_position = (gcode->get_value('S')!= 0);
    }
    
    if (repeat < 1){
        gcode->stream->printf("ALARM: Probe fail: repeat value cannot be less than 1\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }
	//slow zprobe without alarm to probe_height. Skip if probe height is 0
	if (probe_height != 0){
		// do z probe with slow speed
        THEKERNEL->streams->printf("Probing Z with a distance of %.3f\n", probe_height);

        std::sprintf(buff, "G38.3 Z%.3f F%.3f", THEROBOT->from_millimeters(-probe_height), feed_rate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        if (probe_XYZ(gcodeBuffer)){ //probe down, if bottom surface reached, retract slightly
            THEKERNEL->streams->printf("Probed surface hit");
            moveBuffer[0] = 0;
            moveBuffer[1] = 0;
            moveBuffer[2] = THEROBOT->from_millimeters(clearance_height);
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            THECONVEYOR->wait_for_idle();
        }
        delete gcodeBuffer;
    }

    float sin_angle = sin(roation_angle*pi/180);
	float cos_angle = cos(roation_angle*pi/180);

	//negative values are doubled as they start from probe from the positive location
	float x_positive_x = x_axis_distance * cos_angle; //with 0 rotation this is x positive
	float x_positive_y = x_axis_distance * sin_angle;
	float x_negative_x = -x_positive_x*2; //with 0 rotation this is x negative
	float x_negative_y = -x_positive_y*2;
	float y_positive_x = y_axis_distance * sin_angle; //with 0 rotation this is y positive
	float y_positive_y = y_axis_distance * -cos_angle;
	float y_negative_x = -y_positive_x*2; //with 0 rotation this is y negative
	float y_negative_y = -y_positive_y*2;

    
	//output points
	float x_positive_x_out = 0;
	float y_positive_y_out = 0;
    float x_positive_y_out = 0;
	float y_positive_x_out = 0;

	float center_x;
	float center_y;
    float mpos[3];
    float old_mpos[3];

    float slowZprobeRate = 50;

    //save center position to use later
    THEROBOT->get_current_machine_position(mpos);
    memcpy(old_mpos, mpos, sizeof(mpos));
    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
    if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
    center_x = old_mpos[0];
    center_y = old_mpos[1];
    clearance_world_pos = old_mpos[2];
    


	//setup repeat
	for(int i=0; i< repeat; i++) {
        float retractx = THEROBOT->from_millimeters((x_axis_distance>= 0 ? 1.0f : -1.0f) * retract_distance * cos_angle);
        float retracty = THEROBOT->from_millimeters((x_axis_distance>= 0 ? 1.0f : -1.0f) * retract_distance * sin_angle);
        
        coordinated_move(NAN, NAN, clearance_world_pos, rapid_rate);
        // do positive x probe
        //probe no hit alarm x_positive - G38.3, alarm if true
        std::sprintf(buff, "G38.3 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_positive_x), THEROBOT->from_millimeters(x_positive_y), feed_rate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        if (probe_XYZ(gcodeBuffer)){
            gcode->stream->printf("ALARM: Probe hit wall when moving to outer position\n");
            THEKERNEL->call_event(ON_HALT, nullptr);
            THEKERNEL->set_halt_reason(PROBE_FAIL);
            delete gcodeBuffer;
            return;
        }
        delete gcodeBuffer;
        //probe z no hit no alarm -side_depth, retract slightly if probe point hit - G38.3, retract if true
        std::sprintf(buff, "G38.3 Z%.3f F%.3f", THEROBOT->from_millimeters(-(side_depth + clearance_height)), feed_rate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        if (probe_XYZ(gcodeBuffer)){
            moveBuffer[0] = 0;
            moveBuffer[1] = 0;
            moveBuffer[2] = THEROBOT->from_millimeters(1);
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
        }
        delete gcodeBuffer;
        // probe alarm x_negative - G38.2
        std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_negative_x), THEROBOT->from_millimeters(x_negative_y), feed_rate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        probe_XYZ(gcodeBuffer);
        delete gcodeBuffer;
        //retract x_positive
        moveBuffer[0] = retractx;
        moveBuffer[1] = retracty;
        moveBuffer[2] = 0;
        THEROBOT->delta_move(moveBuffer, feed_rate, 3);
        //slow probe
        std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_negative_x), THEROBOT->from_millimeters(x_negative_y), slowZprobeRate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        probe_XYZ(gcodeBuffer);
        delete gcodeBuffer;
        //store position
        THEROBOT->get_current_machine_position(mpos);
        memcpy(old_mpos, mpos, sizeof(mpos));
        // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
        if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform

        x_positive_x_out = old_mpos[0] - (x_axis_distance>= 0 ? 1.0f : -1.0f) *  tool_dia *  cos_angle/ 2;
        x_positive_y_out = old_mpos[1] - (x_axis_distance>= 0 ? 1.0f : -1.0f) *  tool_dia *  sin_angle/ 2;
        //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", x_positive_x_out, x_positive_y_out);
        //retract x_positive
        moveBuffer[0] = retractx;
        moveBuffer[1] = retracty;
        moveBuffer[2] = 0;
        THEROBOT->delta_move(moveBuffer, feed_rate, 3);
        //goto clearance_world_pos in z
        coordinated_move(NAN, NAN, clearance_world_pos, rapid_rate);
        THECONVEYOR->wait_for_idle();
        //goto current center position
        coordinated_move(center_x, center_y, NAN, rapid_rate );
        
        // do positive y probe
        //probe no hit alarm y_positive - G38.3, alarm if true
        std::sprintf(buff, "G38.3 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_positive_x), THEROBOT->from_millimeters(y_positive_y), feed_rate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        if (probe_XYZ(gcodeBuffer)){
            gcode->stream->printf("ALARM: Probe hit wall when moving to outer position\n");
            THEKERNEL->call_event(ON_HALT, nullptr);
            THEKERNEL->set_halt_reason(PROBE_FAIL);
            delete gcodeBuffer;
            return;
        }
        delete gcodeBuffer;
        //probe z no hit no alarm -side_depth, retract slightly if probe point hit - G38.3, retract if true
        std::sprintf(buff, "G38.3 Z%.3f F%.3f", THEROBOT->from_millimeters(-(side_depth + clearance_height)), feed_rate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        if (probe_XYZ(gcodeBuffer)){
            moveBuffer[0] = 0;
            moveBuffer[1] = 0;
            moveBuffer[2] = THEROBOT->from_millimeters(1);
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
        }
        delete gcodeBuffer;
        // probe alarm y_negative - G38.2
        std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_negative_x), THEROBOT->from_millimeters(y_negative_y), feed_rate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        probe_XYZ(gcodeBuffer);
        delete gcodeBuffer;
        //retract y_positive
        retractx = THEROBOT->from_millimeters((y_axis_distance>= 0 ? 1.0f : -1.0f) * retract_distance * sin_angle);
        retracty = THEROBOT->from_millimeters((y_axis_distance>= 0 ? 1.0f : -1.0f) * retract_distance * -cos_angle);

        moveBuffer[0] = retractx;
        moveBuffer[1] = retracty;
        moveBuffer[2] = 0;
        THEROBOT->delta_move(moveBuffer, feed_rate, 3);
        //slow probe
        std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_negative_x), THEROBOT->from_millimeters(y_negative_y), slowZprobeRate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        probe_XYZ(gcodeBuffer);
        delete gcodeBuffer;
        //store position
        THEROBOT->get_current_machine_position(mpos);
        memcpy(old_mpos, mpos, sizeof(mpos));
        // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
        if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
        y_positive_y_out = old_mpos[1] - (y_axis_distance>= 0 ? 1.0f : -1.0f) * tool_dia * sin_angle / 2;
        y_positive_x_out = old_mpos[0] - (y_axis_distance>= 0 ? 1.0f : -1.0f) * tool_dia * cos_angle / 2;
        //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", y_positive_x_out, y_positive_y_out);
        //retract y_positive
        moveBuffer[0] = retractx;
        moveBuffer[1] = retracty;
        moveBuffer[2] = 0;
        THEROBOT->delta_move(moveBuffer, feed_rate, 3);
        //goto clearance_world_pos in z
        coordinated_move(NAN, NAN, clearance_world_pos, rapid_rate);
        THECONVEYOR->wait_for_idle();

            THEKERNEL->streams->printf("entering in calculation routine");
        // calculate center position
        if (roation_angle == 0)
        {
            THEKERNEL->probe_outputs[3] = x_positive_x_out;
            THEKERNEL->probe_outputs[4] = y_positive_y_out;
        }
        else
        {
            //this has a bug in it and is disabled for now.
            float lines_m_value = -tan(roation_angle * pi / 180);
            float lines_b_value = x_positive_y_out - x_positive_x_out * lines_m_value;
            float lines_c_value = y_positive_y_out + y_positive_x_out * lines_m_value;
            THEKERNEL->probe_outputs[3] = (lines_c_value - lines_b_value) / (2 * lines_m_value); // x_out
            THEKERNEL->probe_outputs[4] = (lines_c_value - lines_b_value) / 2 + lines_b_value;   // y_out
        }

        //goto center position
        coordinated_move(center_x, center_y, NAN, rapid_rate );
        THECONVEYOR->wait_for_idle();

		
	}
    THEKERNEL->streams->printf("Corner found. X coordinate stored in #154 as MCS %.3f , Y coordinate in #155 as MCS %.3f  \n", THEKERNEL->probe_outputs[3], THEKERNEL->probe_outputs[4] );
    
    if (save_position){

        THEROBOT->set_current_wcs_by_mpos( THEKERNEL->probe_outputs[3], THEKERNEL->probe_outputs[4], NAN);
    }
}

void ZProbe::probe_axisangle(Gcode *gcode) //M464
{
    THECONVEYOR->wait_for_idle();
    THEKERNEL->streams->printf("Probing 2 points to find an angle\n");
    float probe_height = 0;
    float feed_rate = 300;
    float rapid_rate = 800;
    float x_axis_distance = 20;
    float y_axis_distance = 20;
    float roation_angle = 0;
    int repeat = 1;
    float visualize_path_distance = -1;
    float retract_distance = 1.5;
    float clearance_height = 2;
    float side_depth = -30;
    invert_probe = false;

    Gcode *gcodeBuffer;
    float moveBuffer[3];
    char buff[100];

    if (!gcode->has_letter('X') && !gcode->has_letter('Y')){
        gcode->stream->printf("ALARM: Probe fail: No Axis Set\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }
    if (gcode->has_letter('X') && gcode->has_letter('Y')){
        gcode->stream->printf("ALARM: Probe fail: Axis Probing Only Supports 1 Axis Input\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }


    if (gcode->has_letter('E')) { //depth to probe sides from
        side_depth = - gcode->get_value('E');
    }
    if (gcode->has_letter('H')) { //probe height above bore/disance to move down before probing
        probe_height = gcode->get_value('H');
    }
    if (gcode->has_letter('C')){
        clearance_height = gcode->get_value('C');
    }
    if (gcode->has_letter('X')) { //radius x
        x_axis_distance = gcode->get_value('X');
        y_axis_distance = side_depth;
    }
    if (gcode->has_letter('Y')) { //radius y
        y_axis_distance = -gcode->get_value('Y');
        x_axis_distance = -side_depth;
    }
    if (gcode->has_letter('Q')) { //roation of pocket
        roation_angle = gcode->get_value('Q');
    }
    if (gcode->has_letter('F')) { //feed rate
        feed_rate = gcode->get_value('F');
    }
    if (gcode->has_letter('K')) { //probe height above bore/disance to move down before probing
        rapid_rate = gcode->get_value('K');
    }
    if (gcode->has_letter('L')) { //repeat touch off
        repeat = gcode->get_value('L');
    }
    if (gcode->has_letter('R')) { //retract distance
        retract_distance = gcode->get_value('R');
    }
    if (gcode->has_letter('V')) { //visualize path
        visualize_path_distance = gcode->get_value('V');
    }
    if (repeat < 1){
        gcode->stream->printf("ALARM: Probe fail: repeat value cannot be less than 1\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }
    
	//slow zprobe without alarm to probe_height. Skip if probe height is 0
	if (probe_height != 0){
		// do z probe with slow speed
        THEKERNEL->streams->printf("Probing Z with a distance of %.3f\n", probe_height);

        std::sprintf(buff, "G38.3 Z%.3f F%.3f", THEROBOT->from_millimeters(-probe_height), feed_rate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        if (probe_XYZ(gcodeBuffer)){ //probe down, if bottom surface reached, retract slightly
            THEKERNEL->streams->printf("Probed surface hit");
            moveBuffer[0] = 0;
            moveBuffer[1] = 0;
            moveBuffer[2] = THEROBOT->from_millimeters(clearance_height);
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            THECONVEYOR->wait_for_idle();
        }
        delete gcodeBuffer;
    }

    float sin_angle = sin(roation_angle*pi/180);
	float cos_angle = cos(roation_angle*pi/180);

	//negative values are doubled as they start from probe from the positive location
	float x_positive_x = x_axis_distance * cos_angle; //with 0 rotation this is x positive
	float x_positive_y = x_axis_distance * sin_angle;
	float y_positive_x = y_axis_distance * sin_angle; //with 0 rotation this is y positive
	float y_positive_y = y_axis_distance * -cos_angle;

    
	//output points
	float x_positive_x_out = 0;
	float x_positive_y_out = 0;
	float y_positive_x_out = 0;
	float y_positive_y_out = 0;

	float center_x;
	float center_y;
    float mpos[3];
    float old_mpos[3];

    float slowZprobeRate = 50;

    //save center position to use later
    THEROBOT->get_current_machine_position(mpos);
    memcpy(old_mpos, mpos, sizeof(mpos));
    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
    if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
    center_x = old_mpos[0];
    center_y = old_mpos[1];


	//setup repeat
	for(int i=0; i< repeat; i++) {
        
        //goto clearance height
        //coordinated_move(NAN, NAN, clearance_world_pos, rapid_rate);

        if (gcode->has_letter('X')) {
            float retractx = THEROBOT->from_millimeters((y_axis_distance>= 0 ? 1.0f : -1.0f) * - retract_distance * sin_angle);
            float retracty = THEROBOT->from_millimeters((y_axis_distance>= 0 ? 1.0f : -1.0f) * - retract_distance * -cos_angle);
            //probe along y axis to find first point
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_positive_x), THEROBOT->from_millimeters(y_positive_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //retract
            moveBuffer[0] = retractx;
            moveBuffer[1] = retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            THECONVEYOR->wait_for_idle();
            //slow probe
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_positive_x), THEROBOT->from_millimeters(y_positive_y), slowZprobeRate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //store position
            THEROBOT->get_current_machine_position(mpos);
            memcpy(old_mpos, mpos, sizeof(mpos));
            // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
            if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
            x_positive_x_out = old_mpos[0];
            x_positive_y_out = old_mpos[1];
            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", x_positive_x_out, x_positive_y_out);
            //retract
            moveBuffer[0] = retractx;
            moveBuffer[1] = retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            THECONVEYOR->wait_for_idle();

            //return to center position
            coordinated_move(center_x, center_y, NAN, rapid_rate );
            //probe along x axis to second position, alarm if hit
            std::sprintf(buff, "G38.3 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_positive_x), THEROBOT->from_millimeters(x_positive_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            if (probe_XYZ(gcodeBuffer)){
                gcode->stream->printf("ALARM: Probe hit wall when moving to second position\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                delete gcodeBuffer;
                return;
            }
            delete gcodeBuffer;

            //probe along y axis to find second point
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_positive_x), THEROBOT->from_millimeters(y_positive_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //retract
            moveBuffer[0] = retractx;
            moveBuffer[1] = retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            //slow probe
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_positive_x), THEROBOT->from_millimeters(y_positive_y), slowZprobeRate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //store position
            THEROBOT->get_current_machine_position(mpos);
            memcpy(old_mpos, mpos, sizeof(mpos));
            // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
            if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
            y_positive_x_out = old_mpos[0];
            y_positive_y_out = old_mpos[1];
            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", y_positive_x_out, y_positive_y_out);
            //retract
            moveBuffer[0] = retractx;
            moveBuffer[1] = retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            THECONVEYOR->wait_for_idle();
            //return to x axis probe position 2
            coordinated_move(center_x + x_positive_x, center_y + x_positive_y, NAN, rapid_rate );
            THECONVEYOR->wait_for_idle();
            //return to center position
            coordinated_move(center_x, center_y, NAN, rapid_rate );
            THECONVEYOR->wait_for_idle();
            
            //calculate angle
            //inverse tan ( (Point 2 y - Point 1 Y) / (Point 2 x - point 1 X) )
            THEKERNEL->probe_outputs[2] = atan ( ( y_positive_y_out - x_positive_y_out ) / (y_positive_x_out - x_positive_x_out)) * 180 /pi;
            THEKERNEL->streams->printf("Angle from X Axis is: %.3f degrees or %.3f radians and is stored in radians at variable #153\n" , THEKERNEL->probe_outputs[2] , THEKERNEL->probe_outputs[2] * pi / 180 );
    
        }

        if (gcode->has_letter('Y')) {
            float retractx = THEROBOT->from_millimeters((x_axis_distance>= 0 ? 1.0f : -1.0f) * - retract_distance * cos_angle);
            float retracty = THEROBOT->from_millimeters((x_axis_distance>= 0 ? 1.0f : -1.0f) * - retract_distance * sin_angle);
            //probe along x axis to find first point
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_positive_x), THEROBOT->from_millimeters(x_positive_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //retract
            moveBuffer[0] = retractx;
            moveBuffer[1] = retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            THECONVEYOR->wait_for_idle();
            //slow probe
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_positive_x), THEROBOT->from_millimeters(x_positive_y), slowZprobeRate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //store position
            THEROBOT->get_current_machine_position(mpos);
            memcpy(old_mpos, mpos, sizeof(mpos));
            // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
            if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
            x_positive_x_out = old_mpos[0];
            x_positive_y_out = old_mpos[1];
            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", x_positive_x_out, x_positive_y_out);
            //retract
            moveBuffer[0] = retractx;
            moveBuffer[1] = retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            THECONVEYOR->wait_for_idle();

            //return to center position
            coordinated_move(center_x, center_y, NAN, rapid_rate );
            //probe along y axis to second position, alarm if hit
            std::sprintf(buff, "G38.3 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(y_positive_x), THEROBOT->from_millimeters(y_positive_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            if (probe_XYZ(gcodeBuffer)){
                gcode->stream->printf("ALARM: Probe hit wall when moving to second position\n");
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->set_halt_reason(PROBE_FAIL);
                delete gcodeBuffer;
                return;
            }
            delete gcodeBuffer;

            //probe along x axis to find second point
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_positive_x), THEROBOT->from_millimeters(x_positive_y), feed_rate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //retract
            moveBuffer[0] = retractx;
            moveBuffer[1] = retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            //slow probe
            std::sprintf(buff, "G38.2 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(x_positive_x), THEROBOT->from_millimeters(x_positive_y), slowZprobeRate);
            gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
            probe_XYZ(gcodeBuffer);
            delete gcodeBuffer;
            //store position
            THEROBOT->get_current_machine_position(mpos);
            memcpy(old_mpos, mpos, sizeof(mpos));
            // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
            if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, true); // get inverse compensation transform
            y_positive_x_out = old_mpos[0];
            y_positive_y_out = old_mpos[1];
            //THEKERNEL->streams->printf("X: %.3f Y: %.3f\n", y_positive_x_out, y_positive_y_out);
            //retract
            moveBuffer[0] = retractx;
            moveBuffer[1] = retracty;
            moveBuffer[2] = 0;
            THEROBOT->delta_move(moveBuffer, feed_rate, 3);
            THECONVEYOR->wait_for_idle();
            //return to y axis probe position 2
            coordinated_move(center_x + y_positive_x, center_y + y_positive_y, NAN, rapid_rate );
            THECONVEYOR->wait_for_idle();
            //return to center position
            coordinated_move(center_x, center_y, NAN, rapid_rate );
            THECONVEYOR->wait_for_idle();
            
            //calculate angle
            //inverse tan ( (Point 2 y - Point 1 Y) / (Point 2 x - point 1 X) )
            THEKERNEL->probe_outputs[2] = atan ( ( y_positive_y_out - x_positive_y_out ) / (y_positive_x_out - x_positive_x_out)) * 180 /pi;
            THEKERNEL->streams->printf("Angle from X Axis is: %.3f degrees or %.3f radians and is stored in radians at variable #153\n" , THEKERNEL->probe_outputs[2] , THEKERNEL->probe_outputs[2] * pi / 180 );
        }

		
	}

    if (visualize_path_distance != 0) {
        //distance between two points:
        if (visualize_path_distance < 0){
            visualize_path_distance = sqrt((x_positive_x_out - y_positive_x_out)*(x_positive_x_out - y_positive_x_out) + (x_positive_y_out - y_positive_y_out)*(x_positive_y_out - y_positive_y_out));
        }
        //probe to second position
        std::sprintf(buff, "G38.3 X%.3f Y%.3f F%.3f", THEROBOT->from_millimeters(visualize_path_distance * cos(THEKERNEL->probe_outputs[2] * pi/180)), THEROBOT->from_millimeters(visualize_path_distance * sin(THEKERNEL->probe_outputs[2] * pi/180 )), feed_rate);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        probe_XYZ(gcodeBuffer);
        delete gcodeBuffer;
        //return to center position
        THECONVEYOR->wait_for_idle();
        coordinated_move(center_x, center_y, NAN, rapid_rate );
    }
}

void ZProbe::calibrate_probe_bore(Gcode *gcode) //M460
{
    THECONVEYOR->wait_for_idle();
    THEKERNEL->streams->printf("Probing Bore/Rectangular Pocket\n");
    float feed_rate = 300;
    float rapid_rate = 800;
    float knownDiameter = 25;
    float roation_angle = 0;
    float rotation_offset_per_probe = 0;
    int repeat = 1;
    float retract_distance = 1.5;

    Gcode *gcodeBuffer; // = static_cast<Gcode *>(argument);
    char buff[100];

    if (!gcode->has_letter('X') && !gcode->has_letter('Y') ) { //error if there is a problem
        gcode->stream->printf("ALARM: Probe fail: No Radius Given\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }

    if (gcode->has_letter('Q')) { //roation of pocket
        roation_angle = gcode->get_value('Q');
    }
    if (gcode->has_letter('F')) { //feed rate
        feed_rate = gcode->get_value('F');
    }
    if (gcode->has_letter('K')) { //probe height above bore/disance to move down before probing
        rapid_rate = gcode->get_value('K');
    }
    if (gcode->has_letter('L')) { //repeat touch off
        repeat = gcode->get_value('L');
    }
    if (gcode->has_letter('R')) { //retract distance
        retract_distance = gcode->get_value('R');
    }
        if (gcode->has_letter('X')) { //radius x
        knownDiameter = gcode->get_value('X');
    }
    if (gcode->has_letter('Y')) { //radius y
        knownDiameter = gcode->get_value('Y');
    }
    if (gcode->has_letter('I')) { //radius y
        rotation_offset_per_probe = gcode->get_value('I');
    }

    if (repeat < 1){
        gcode->stream->printf("ALARM: Probe fail: repeat value cannot be less than 1\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(PROBE_FAIL);
        return;
    }

    vector<float> probe_position_stack;
    

    for(int i=0; i< repeat; i++) {
        std::sprintf(buff, "G38.10 X%.3f Y%.3f F%.3f D0 Q%3f K%.3f R%3f", THEROBOT->from_millimeters(knownDiameter + 2), THEROBOT->from_millimeters(knownDiameter + 2), feed_rate,roation_angle,rapid_rate,retract_distance);
        gcodeBuffer = new Gcode(buff, &StreamOutput::NullStream);
        probe_bore(gcodeBuffer);
        delete gcodeBuffer;
        THECONVEYOR->wait_for_idle();
        probe_position_stack.push_back(THEKERNEL->probe_outputs[0]);
        probe_position_stack.push_back(THEKERNEL->probe_outputs[1]);
        roation_angle += rotation_offset_per_probe;
    }
    
    float sum = 0.0;
    for (const auto& pos : probe_position_stack) {
        sum += pos;
    }
    float ave = sum / (repeat * 2);
    
    THEKERNEL->streams->printf("Average bore diameter: %.3f\n", ave);
    
    THEKERNEL->probe_tip_diameter = knownDiameter - ave;
    THEKERNEL->streams->printf("New Probe Tip Diameter is: %.3f\n", THEKERNEL->probe_tip_diameter);
    THEKERNEL->streams->printf("This value is temporary \n and will neeed to be saved to the config file with \n config-set sd zprobe.probe_tip_diameter # \n");

}