/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "libs/FirmwareFileSystem.h"
#include "libs/Module.h"
#include "libs/Config.h"
#include "libs/CRC16.h"
#include "libs/nuts_bolts.h"
#include "libs/SlowTicker.h"
#include "libs/Adc.h"
#include "libs/compiler.h"
#include "libs/StreamOutputPool.h"
#include "libs/SerialMessage.h"
#include <mri.h>
#include "checksumm.h"
#include "ConfigValue.h"

#include "libs/StepTicker.h"
#include "libs/PublicData.h"
#include "us_ticker_api.h"
#include "modules/communication/SerialConsole.h"
#include "modules/communication/GcodeDispatch.h"
#include "modules/robot/Planner.h"
#include "modules/robot/Robot.h"
#include "modules/robot/Conveyor.h"
#include "StepperMotor.h"
#include "BaseSolution.h"
#include "EndstopsPublicAccess.h"
#include "Configurator.h"
#include "SimpleShell.h"
#include "TemperatureControlPublicAccess.h"
#include "LaserPublicAccess.h"
#include "ATCHandlerPublicAccess.h"
#include "PlayerPublicAccess.h"
#include "SpindlePublicAccess.h"
#include "SwitchPublicAccess.h"
#include "ZProbePublicAccess.h"
#include "MainButtonPublicAccess.h"
#include "modules/tools/accessories/BedCleaning.h"
#include "modules/tools/accessories/SpindleAccessories.h"
#include "mbed.h"
#include "utils.h"
#include "WifiPublicAccess.h"
#if defined(REMOTE_SETTINGS_TRANSFER)
#include "modules/communication/RemoteTransfer.h"
#endif

#include <algorithm>

#ifndef NO_TOOLS_LASER
#include "Laser.h"
#endif

#include <array>
#include <string>

#define laser_checksum CHECKSUM("laser")
#define baud_rate_setting_checksum CHECKSUM("baud_rate")
#define uart_checksum              CHECKSUM("uart")

#define base_stepping_frequency_checksum            CHECKSUM("base_stepping_frequency")
#define microseconds_per_step_pulse_checksum        CHECKSUM("microseconds_per_step_pulse")
#define disable_leds_checksum                       CHECKSUM("leds_disable")
#define grbl_mode_checksum                          CHECKSUM("grbl_mode")
#define feed_hold_enable_checksum                   CHECKSUM("enable_feed_hold")
#define ok_per_line_checksum                        CHECKSUM("ok_per_line")
#define disable_serial_console_checksum             CHECKSUM("disable_serial_console")
#define halt_on_error_debug_checksum                CHECKSUM("halt_on_error_debug")
#define protocol_checksum                           CHECKSUM("protocol")
Kernel* Kernel::instance;

static float local_vars_storage[20];
static float local_params_storage[30];

#define	EEP_MAX_PAGE_SIZE	32
#define EEPROM_DATA_STARTPAGE	1
#define EEPROM_FACTORYSET_PAGE	16
#if defined(MACHINE_FAMILY_Z1)
constexpr int boot_serial_baud = 230400;
constexpr FACTORY_SET default_factory_settings{Z1, 0, 0, 0};
#else
constexpr int boot_serial_baud = 115200;
constexpr FACTORY_SET default_factory_settings{CARVERA, 0x04, 0, 0};
#endif

namespace {
struct StockEepromData {
    float TLO;
    float G54[3];
    float REFMZ;
    float TOOLMZ;
    float reserve;
    int TOOL;
    float G54AB[2];
};

static_assert(sizeof(StockEepromData) == 40, "Unexpected stock EEPROM record size");
} // namespace

// The kernel is the central point in Smoothie : it stores modules, and handles event calls
Kernel::Kernel()
{
    halted = false;
    feed_hold = false;
    enable_feed_hold = false;
    bad_mcu= true;
    stop_request = false;
    uploading = false;
    laser_mode = false;
    optional_stop_mode = false;
    line_by_line_exec_mode = false;
    sleeping = false;
    waiting = false;
    tool_waiting = false;
    suspending = false;
    halt_reason = MANUAL;
    atc_state = 0;
    zprobing = false;
    probeLaserOn = false;
    probe_addr = 0;
    checkled = false;
    spindleon = false;
    for (bool &axis_on : axis_is_on) axis_on = false;
    debug_flags = {};
    cachewait = false;
    disable_serial_console = false;
    keep_alive_request = false;
    flex_compensation_load_error = false;
    config_load_error = false;
    dispatching_console_line = false;
    steppers_powered = false;
    steppers_powered_at_us = 0;
    bed_cleaning = nullptr;
    spindle_accessories = nullptr;

    local_vars   = local_vars_storage;
    local_params = local_params_storage;

    // Initialize user defined variables and subroutine call parameters.
    for(int i = 0; i < 20; ++i) local_vars[i]   = -1.0e6f;
    for(int i = 0; i < 30; ++i) local_params[i] = 0.0f;

    instance = this; // setup the Singleton instance of the kernel

    // init I2C
    this->i2c = new mbed::I2C(P0_27, P0_28);
    this->i2c->frequency(200000);

    // Bring up serial output first so factory and config errors are visible.
    this->streams = new StreamOutputPool();
    this->serial = new SerialConsole(P2_8, P2_9, boot_serial_baud);
    this->streams->append_stream(this->serial);

    this->factory_set = new FACTORY_SET();
    // read Factory setting data from eeprom
    this->read_Factory_data();
#if defined(REMOTE_SETTINGS_TRANSFER)
    const FACTORY_SET previous = *this->factory_set;
    FACTORY_SET received = previous;
    const remote::Result factory_result = remote::receive_factory_settings(*this->serial, previous, received);
    const bool factory_changed = received.MachineModel != previous.MachineModel ||
                                 received.FuncSetting != previous.FuncSetting ||
                                 received.reserve1 != previous.reserve1 || received.reserve2 != previous.reserve2;
    if (factory_result == remote::Result::success) {
        bool stored = true;
        if (factory_changed) {
            *this->factory_set = received;
            stored = write_Factory_data();
            if (!stored) *this->factory_set = previous;
        }
        remote::finish_factory_settings(*this->serial, stored);
        if (stored && factory_changed) system_reset(false);
    } else if (factory_result != remote::Result::cancelled) {
        this->streams->printf(
            "ERROR: factory settings transfer failed (%u); using stored settings\n",
            static_cast<unsigned>(factory_result));
    }
#else
    // read Factory settings data from sd
    this->read_Factroy_SD();
#endif


    // Config next, but does not load cache yet
    this->config = new Config();

    // Pre-load the config cache
    this->config->config_cache_load();

    this->current_path   = "/";

    NVIC_SetPriorityGrouping(0);
    //some boards don't have leds.. TOO BAD!
    this->use_leds = !this->config->value( disable_leds_checksum )->as_bool(false);

#ifdef CNC
    this->grbl_mode = this->config->value( grbl_mode_checksum )->as_bool(true);
#else
    this->grbl_mode = this->config->value( grbl_mode_checksum )->as_bool(false);
#endif

    this->enable_feed_hold = this->config->value( feed_hold_enable_checksum )->as_bool(this->grbl_mode);

    // we expect ok per line now not per G code, setting this to false will return to the old (incorrect) way of ok per G code
    this->ok_per_line = this->config->value( ok_per_line_checksum )->as_bool(true);

    // Option to disable serial console. Useful primarily if MRI is enabled and
    // you want to keep the serial port dedicated for such traffic. Or you want
    // to save some memory?
    this->disable_serial_console = this->config->value( disable_serial_console_checksum )->as_bool(false);
    
    // Check if we should break into the debugger on halt
    this->halt_on_error_debug = this->config->value( halt_on_error_debug_checksum )->as_bool(false);

    this->protocol_from_name(this->config->value( protocol_checksum )->as_string("smoothie"), communication_protocol);

    if (this->disable_serial_console) {
        this->streams->remove_stream(this->serial);
        delete this->serial;
        this->serial = nullptr;
    } else {
        // add_module() runs SerialConsole::on_module_loaded(). Carvera applies
        // uart.baud_rate there; the Z1 serial link remains fixed.
        this->add_module( this->serial );
    }

    // HAL stuff
    add_module( this->slow_ticker = new SlowTicker());

    this->step_ticker = new StepTicker();
    this->adc = new Adc();

    // TODO : These should go into platform-specific files
    // LPC17xx-specific
    NVIC_SetPriorityGrouping(0);
    NVIC_SetPriority(TIMER0_IRQn, 2);
    NVIC_SetPriority(TIMER1_IRQn, 1);
    NVIC_SetPriority(TIMER2_IRQn, 4);
    NVIC_SetPriority(TIMER3_IRQn, 4);
    NVIC_SetPriority(PendSV_IRQn, 3);

    // Set other priorities lower than the timers
    NVIC_SetPriority(ADC_IRQn, 5);
    NVIC_SetPriority(USB_IRQn, 5);

    // If MRI is enabled
    if( MRI_ENABLE ) {
        if( NVIC_GetPriority(UART0_IRQn) > 0 ) { NVIC_SetPriority(UART0_IRQn, 5); }
        if( NVIC_GetPriority(UART1_IRQn) > 0 ) { NVIC_SetPriority(UART1_IRQn, 5); }
        if( NVIC_GetPriority(UART2_IRQn) > 0 ) { NVIC_SetPriority(UART2_IRQn, 5); }
        if( NVIC_GetPriority(UART3_IRQn) > 0 ) { NVIC_SetPriority(UART3_IRQn, 5); }
    } else {
        NVIC_SetPriority(UART0_IRQn, 5);
        NVIC_SetPriority(UART1_IRQn, 5);
        NVIC_SetPriority(UART2_IRQn, 5);
        NVIC_SetPriority(UART3_IRQn, 5);
    }

    // Configure the step ticker
    this->base_stepping_frequency = this->config->value(base_stepping_frequency_checksum)->as_number(100000);
    float microseconds_per_step_pulse = this->config->value(microseconds_per_step_pulse_checksum)->as_number(1);

    // Configure the step ticker
    this->step_ticker->set_frequency( this->base_stepping_frequency );
    this->step_ticker->set_unstep_time( microseconds_per_step_pulse );

    this->eeprom_data = new EEPROM_data();
    // read eeprom data
    this->read_eeprom_data();
    // check eeprom data
    this->check_eeprom_data();

    // Core modules
    this->add_module( this->simpleshell    = new SimpleShell()   );
    this->add_module( this->conveyor       = new Conveyor()      );
    this->add_module( this->gcode_dispatch = new GcodeDispatch() );
    this->add_module( this->robot          = new Robot()         );

    this->planner = new Planner();
    this->configurator = new Configurator();
}

void Kernel::on_steppers_powered(bool powered, uint32_t now_us)
{
    steppers_powered = powered;
    if (powered)
        steppers_powered_at_us = now_us;
}

bool Kernel::stepper_alarms_enabled(uint32_t now_us, uint32_t settle_us) const
{
    return steppers_powered && (settle_us == 0 || now_us - steppers_powered_at_us >= settle_us);
}

void Kernel::protocol_from_name(const std::string& name, ProtocolMode& protocol)
{
    if (name == "smoothie") {
        protocol = PROTOCOL_SMOOTHIE;
        return;
    }

    if (name == "makera") {
        protocol = PROTOCOL_MAKERA;
        return;
    }
    protocol = PROTOCOL_MAKERA;
    return;
}

// get current state
uint8_t Kernel::get_state()
{
    bool homing;
    bool ok = PublicData::get_value(endstops_checksum, get_homing_status_checksum, 0, &homing);
    if(!ok) homing = false;
    if (sleeping) {
    	return SLEEP;
    } else if (suspending) {
    	return SUSPEND;
    } else if (waiting) {
    	return WAIT;
    } else if (tool_waiting) {
    	return TOOL;
    } else if(halted) {
    	return ALARM;
    } else if (homing) {
    	return HOME;
    } else if (feed_hold) {
    	return HOLD;
    } else if (this->conveyor->is_idle() && (this->spindleon == false)) {
    	return IDLE;
    } else {
    	return RUN;
    }
}

// return a GRBL-like query string for serial ?
std::string Kernel::get_query_string()
{

    std::string str;
    bool running = false;
    bool ok = false;

    uint8_t state = this->get_state();

    str.append("<");
    if (state == SLEEP) {
    	str.append("Sleep");
    } else if (state == SUSPEND) {
    	str.append("Pause");
    } else if (state == WAIT) {
        str.append("Wait");
    } else if (state == TOOL) {
		str.append("Tool");
    } else if (state == ALARM) {
        str.append("Alarm");
    } else if (state == HOME) {
        running = true;
        str.append("Home");
    } else if (state == HOLD) {
        str.append("Hold");
    } else if (state == IDLE) {
        str.append("Idle");
    } else if (state == RUN) {
        running = true;
        str.append("Run");
    }

    size_t n;
    char buf[128];
    if(running) {
        float mpos[5];
        robot->get_current_machine_position(mpos);
        // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
        if(robot->compensationTransform) robot->compensationTransform(mpos, true, false); // get inverse compensation transform

        // machine position
        n = snprintf(buf, sizeof(buf), "%1.4f,%1.4f,%1.4f", robot->from_millimeters(mpos[0]), robot->from_millimeters(mpos[1]), robot->from_millimeters(mpos[2]));
        if(n > sizeof(buf)) n= sizeof(buf);

        str.append("|MPos:").append(buf, n);

#if MAX_ROBOT_ACTUATORS > 3
        // deal with the ABC axis (E will be A)
        for (int i = A_AXIS; i < robot->get_number_registered_motors(); ++i) {
            // current actuator position
            n = snprintf(buf, sizeof(buf), ",%1.4f", robot->actuators[i]->get_current_position());
            if(n > sizeof(buf)) n= sizeof(buf);
            str.append(buf, n);
        }
#endif

        // work space position
        const int motor_count = robot->get_number_registered_motors();
        mpos[A_AXIS] = motor_count > A_AXIS ? robot->actuators[A_AXIS]->get_current_position() : 0.0F;
        mpos[B_AXIS] = motor_count > B_AXIS ? robot->actuators[B_AXIS]->get_current_position() : 0.0F;
        
        Robot::wcs_t pos = robot->mcs2wcs(mpos);
        n = snprintf(buf, sizeof(buf), "%1.4f,%1.4f,%1.4f", robot->from_millimeters(std::get<X_AXIS>(pos)), robot->from_millimeters(std::get<Y_AXIS>(pos)), robot->from_millimeters(std::get<Z_AXIS>(pos)));
        if(n > sizeof(buf)) n= sizeof(buf);
        str.append("|WPos:").append(buf, n);
        
        //n = snprintf(buf, sizeof(buf), ",%1.4f,%1.4f", robot->from_millimeters(std::get<A_AXIS>(pos)), robot->from_millimeters(std::get<B_AXIS>(pos)));
        n = snprintf(buf, sizeof(buf), ",%1.4f,%1.4f", std::get<A_AXIS>(pos), std::get<B_AXIS>(pos));
        if(n > sizeof(buf)) n= sizeof(buf);
        str.append(buf, n);

    } else {
        // return the last milestone if idle
        // machine position
        Robot::wcs_t mpos = robot->get_axis_position();
        size_t n = snprintf(buf, sizeof(buf), "%1.4f,%1.4f,%1.4f", robot->from_millimeters(std::get<X_AXIS>(mpos)), robot->from_millimeters(std::get<Y_AXIS>(mpos)), robot->from_millimeters(std::get<Z_AXIS>(mpos)));
        if(n > sizeof(buf)) n= sizeof(buf);

        str.append("|MPos:").append(buf, n);
        
        //n = snprintf(buf, sizeof(buf), ",%1.4f,%1.4f", robot->from_millimeters(std::get<A_AXIS>(mpos)), robot->from_millimeters(std::get<B_AXIS>(mpos)));
        n = snprintf(buf, sizeof(buf), ",%1.4f,%1.4f", std::get<A_AXIS>(mpos), std::get<B_AXIS>(mpos));
        if(n > sizeof(buf)) n= sizeof(buf);
        str.append(buf, n);
/*
#if MAX_ROBOT_ACTUATORS > 3
        // deal with the ABC axis (E will be A)
        for (int i = A_AXIS; i < robot->get_number_registered_motors(); ++i) {
            // current actuator position
            n = snprintf(buf, sizeof(buf), ",%1.4f", robot->actuators[i]->get_current_position());
            if(n > sizeof(buf)) n= sizeof(buf);
            str.append(buf, n);
        }
#endif
*/
        // work space position
        Robot::wcs_t pos = robot->mcs2wcs(mpos);
        n = snprintf(buf, sizeof(buf), "%1.4f,%1.4f,%1.4f", robot->from_millimeters(std::get<X_AXIS>(pos)), robot->from_millimeters(std::get<Y_AXIS>(pos)), robot->from_millimeters(std::get<Z_AXIS>(pos)));
        if(n > sizeof(buf)) n= sizeof(buf);
        str.append("|WPos:").append(buf, n);
        
        //n = snprintf(buf, sizeof(buf), ",%1.4f,%1.4f", robot->from_millimeters(std::get<A_AXIS>(pos)), robot->from_millimeters(std::get<B_AXIS>(pos)));
        n = snprintf(buf, sizeof(buf), ",%1.4f,%1.4f", std::get<A_AXIS>(pos), std::get<B_AXIS>(pos));
        if(n > sizeof(buf)) n= sizeof(buf);
        str.append(buf, n);
    }

    n = snprintf(buf, sizeof(buf), "|R:%1.4f", robot->r[robot->get_current_wcs()]);
    if(n > sizeof(buf)) n= sizeof(buf);
    str.append(buf, n);
    n = snprintf(buf, sizeof(buf), "|G:%d", robot->get_current_wcs());
    if(n > sizeof(buf)) n= sizeof(buf);
    str.append(buf, n);
    
    // current feedrate and requested fr and override
    float fr= running ? robot->from_millimeters(conveyor->get_current_feedrate()*60.0F) : 0;
    float frr= robot->from_millimeters(robot->get_feed_rate());
    float fro= 6000.0F / robot->get_seconds_per_minute();
    n = snprintf(buf, sizeof(buf), "|F:%1.1f,%1.1f,%1.1f", fr, frr, fro);
    if(n > sizeof(buf)) n= sizeof(buf);
    str.append(buf, n);

    // current spindle rpm and request rpm and override
    struct spindle_status ss;
    ok = PublicData::get_value(pwm_spindle_control_checksum, get_spindle_status_checksum, &ss);
    if (ok) {
        n = snprintf(buf, sizeof(buf), "|S:%1.1f,%1.1f,%1.1f,%d", ss.current_rpm, ss.target_rpm, ss.factor,
                     int(this->spindle_accessories->chip_clear_enabled()));
        if(n > sizeof(buf)) n= sizeof(buf);
        str.append(buf, n);
    }

    // get spindle temperature
    struct pad_temperature temp;
    ok = PublicData::get_value( temperature_control_checksum, current_temperature_checksum, spindle_temperature_checksum, &temp );
	if (ok) {
        n= snprintf(buf, sizeof(buf), ",%1.1f", temp.current_temperature);
        if(n > sizeof(buf)) n= sizeof(buf);
        str.append(buf, n);
	}	
	
    // get power temperature
    ok = PublicData::get_value( temperature_control_checksum, current_temperature_checksum, power_temperature_checksum, &temp );
	if (!ok) temp.current_temperature = 0;
    n= snprintf(buf, sizeof(buf), ",%1.1f", temp.current_temperature);
    if(n > sizeof(buf)) n= sizeof(buf);
    str.append(buf, n);
#if defined(MACHINE_FAMILY_Z1)
    laser_status accessory_laser{};
    PublicData::get_value(laser_checksum, get_laser_status_checksum, &accessory_laser);
    n = snprintf(buf, sizeof(buf), ",%d,%d,%d,%d",
        int(this->spindle_accessories->auto_blowing_enabled()),
        int(this->bed_cleaning->automatic_cleaning_enabled()), 0,
        int(accessory_laser.static_removal));
#else
	n = snprintf(buf, sizeof(buf), ",%d,%d,%d", 0, 0, int(this->spindle_accessories->extendout_enabled()));
#endif
    if(n > sizeof(buf)) n= sizeof(buf);
    str.append(buf, n);

    // current tool number and tool offset
    struct tool_status tool;
    ok = PublicData::get_value( atc_handler_checksum, get_tool_status_checksum, &tool );
    if (ok) {

	    n= snprintf(buf, sizeof(buf), "|T:%d,%1.3f,%d,%d", tool.active_tool, tool.tool_offset, tool.target_tool, tool.target_collet_type);
        if(n > sizeof(buf)) n= sizeof(buf);
        str.append(buf, n);
    }

    // wireless probe current voltage
    float wp_voltage;
    ok = PublicData::get_value( atc_handler_checksum, get_wp_voltage_checksum, &wp_voltage );
    if (ok) {
        n= snprintf(buf, sizeof(buf), "|W:%1.2f", wp_voltage);
        if(n > sizeof(buf)) n= sizeof(buf);
        str.append(buf, n);
    }

    // current Laser power and override
    struct laser_status ls;
	if(PublicData::get_value(laser_checksum, get_laser_status_checksum, &ls)) {
		n = snprintf(buf, sizeof(buf), "|L:%d, %d, %d, %1.1f,%1.1f", int(ls.mode), int(ls.state), int(ls.testing), ls.power, ls.scale);
		if(n > sizeof(buf)) n= sizeof(buf);
		str.append(buf, n);
	}

    // current running file info
	void *returned_data;
	ok = PublicData::get_value( player_checksum, get_progress_checksum, &returned_data );
	if (ok) {
		struct pad_progress p =  *static_cast<struct pad_progress *>(returned_data);
		n= snprintf(buf, sizeof(buf), "|P:%lu,%d,%lu,%d,%lu", p.played_lines, p.percent_complete, p.elapsed_secs, p.is_playing ? 1 : 0, p.parsed_lines);
		if(n > sizeof(buf)) n= sizeof(buf);
		str.append(buf, n);
	}

    // if not grbl mode get temperatures
    if(!is_grbl_mode()) {
        struct pad_temperature temp;
        // scan all temperature controls
        std::vector<struct pad_temperature> controllers;
        bool ok = PublicData::get_value(temperature_control_checksum, poll_controls_checksum, &controllers);
        if (ok) {
            char buf[32];
            for (auto &c : controllers) {
                size_t n= snprintf(buf, sizeof(buf), "|%s:%1.1f,%1.1f", c.designator.c_str(), c.current_temperature, c.target_temperature);
                if(n > sizeof(buf)) n= sizeof(buf);
                str.append(buf, n);
            }
        }
    }
	
	if(THEKERNEL->factory_set->FuncSetting & (1<<2))	//ATC 
	{
	    // if doing atc
	    if (atc_state != ATC_NONE) {
	        n = snprintf(buf, sizeof(buf), "|A:%d", atc_state);
	        if(n > sizeof(buf)) n = sizeof(buf);
	        str.append(buf, n);
	    }
	}

    // if auto leveling is active
    if (robot->compensationTransform != nullptr) {
        n = snprintf(buf, sizeof(buf), "|O:%1.3f", robot->get_max_delta());
        if(n > sizeof(buf)) n = sizeof(buf);
        str.append(buf, n);
    }

    // if halted
    if (halted) {
        n = snprintf(buf, sizeof(buf), "|H:%d", halt_reason);
        if(n > sizeof(buf)) n = sizeof(buf);
        str.append(buf, n);
    }
    
    // machine state
    n = snprintf(buf, sizeof(buf), "|C:%u,%d,%d,%d",
                 static_cast<unsigned>(THEKERNEL->factory_set->MachineModel),
                 THEKERNEL->factory_set->FuncSetting, THEROBOT->inch_mode, THEROBOT->absolute_mode);
    if(n > sizeof(buf)) n = sizeof(buf);
    str.append(buf, n);

    str.append(">\n");
    return str;
}


// return a Diagnose string
std::string Kernel::get_diagnose_string()
{
	std::string str;
    size_t n;
    char buf[128];
    bool ok = false;

    str.append("{");

    // get spindle state
    struct spindle_status ss;
    ok = PublicData::get_value(pwm_spindle_control_checksum, get_spindle_status_checksum, &ss);
    if (ok) {
        n = snprintf(buf, sizeof(buf), "S:%d,%d", (int)ss.state, (int)ss.target_rpm);
        if(n > sizeof(buf)) n= sizeof(buf);
        str.append(buf, n);
    }

    // get laser state
    struct laser_status ls;
    ok = PublicData::get_value(laser_checksum, get_laser_status_checksum, &ls);
    if (ok) {
        n = snprintf(buf, sizeof(buf), "|L:%d,%d", (int)ls.state, (int)ls.power);
        if(n > sizeof(buf)) n= sizeof(buf);
        str.append(buf, n);
    }

    // get switchs state
    struct pad_switch pad;
    if(CARVERA == THEKERNEL->factory_set->MachineModel)	//ATC 
    {
    	ok = PublicData::get_value(switch_checksum, get_checksum("vacuum"), 0, &pad);
    }
    else
    {
    	ok = PublicData::get_value(switch_checksum, get_checksum("powerfan"), 0, &pad);
    }
    	
    if (ok) {
        n = snprintf(buf, sizeof(buf), "|V:%d,%d", (int)pad.state, (int)pad.value);
        if(n > sizeof(buf)) n = sizeof(buf);
        str.append(buf, n);
    }
    ok = PublicData::get_value(switch_checksum, get_checksum("spindlefan"), 0, &pad);
    if (ok) {
        n = snprintf(buf, sizeof(buf), "|F:%d,%d", (int)pad.state, (int)pad.value);
        if(n > sizeof(buf)) n = sizeof(buf);
        str.append(buf, n);
    }
    ok = PublicData::get_value(switch_checksum, get_checksum("light"), 0, &pad);
    if (ok) {
        n = snprintf(buf, sizeof(buf), "|G:%d", (int)pad.state);
        if(n > sizeof(buf)) n = sizeof(buf);
        str.append(buf, n);
    }
    bool ok2 = false;
	bool ok3 = false;
	struct pad_switch pad2,pad3;
    ok = PublicData::get_value(switch_checksum, get_checksum("beep"), 0, &pad);
    ok2 = PublicData::get_value(switch_checksum, get_checksum("extendin"), 0, &pad2);
   	ok3 = PublicData::get_value(switch_checksum, get_checksum("extendout"), 0, &pad3);
    if(!ok) pad.state = false;
    if(!ok2) pad2.state = false;
    if(!ok3) { pad3.state = false; pad3.value = 0; }
    n = snprintf(buf, sizeof(buf), ",%d,%d,%d,%d", (int)pad.state, (int)pad2.state, (int)pad3.state, (int)pad3.value);
    if(n > sizeof(buf)) n = sizeof(buf);
    str.append(buf, n);
    ok = PublicData::get_value(switch_checksum, get_checksum("toolsensor"), 0, &pad);
    if (ok) {
        n = snprintf(buf, sizeof(buf), "|T:%d", (int)pad.state);
        if(n > sizeof(buf)) n = sizeof(buf);
        str.append(buf, n);
    }
    ok = PublicData::get_value(switch_checksum, get_checksum("air"), 0, &pad);
    if (ok) {
        n = snprintf(buf, sizeof(buf), "|R:%d", (int)pad.state);
        if(n > sizeof(buf)) n = sizeof(buf);
        str.append(buf, n);
    }
    ok = PublicData::get_value(switch_checksum, get_checksum("probecharger"), 0, &pad);
    if (ok) {
        n = snprintf(buf, sizeof(buf), "|C:%d", (int)pad.state);
        if(n > sizeof(buf)) n = sizeof(buf);
        str.append(buf, n);
    }

    // get states
    char data[11];
    ok = PublicData::get_value(endstops_checksum, get_endstop_states_checksum, 0, data);
    if (ok) {
        n = snprintf(buf, sizeof(buf), "|E:%d,%d,%d,%d,%d,%d", data[0], data[1], data[2], data[3], data[4], data[5]);
        if(n > sizeof(buf)) n = sizeof(buf);
        str.append(buf, n);
    }
    if(THEKERNEL->factory_set->FuncSetting & ((1<<0)|(1<<1)) )
    {
    	ok = PublicData::get_value(endstops_checksum, get_endstopAB_states_checksum, 0, data);
	    if (ok) {
	        n = snprintf(buf, sizeof(buf), ",%d,%d", data[0],data[1]);
	        if(n > sizeof(buf)) n = sizeof(buf);
	        str.append(buf, n);
	    }
    }

    // get probe and calibrate states
    ok = PublicData::get_value(zprobe_checksum, get_zprobe_pin_states_checksum, 0, &data[6]);
    if (ok) {
        n = snprintf(buf, sizeof(buf), "|P:%d,%d", data[6], data[7]);
        if(n > sizeof(buf)) n = sizeof(buf);
        str.append(buf, n);
    }
	
	if(THEKERNEL->factory_set->FuncSetting & (1<<2))	//ATC 
	{
	    // get atc endstop and tool senser states
	    ok = PublicData::get_value(atc_handler_checksum, get_atc_pin_status_checksum, 0, &data[8]);
	    if (ok) {
	        n = snprintf(buf, sizeof(buf), "|A:%d,%d", data[8], data[9]);
	        if(n > sizeof(buf)) n = sizeof(buf);
	        str.append(buf, n);
	    }
	}

    // get e-stop states
    ok = PublicData::get_value(main_button_checksum, get_e_stop_state_checksum, 0, &data[10]);
    if (ok) {
        n = snprintf(buf, sizeof(buf), "|I:%d", data[10]);
        if(n > sizeof(buf)) n = sizeof(buf);
        str.append(buf, n);
    }
    // get wifi rssi
    signed char rssidata;
    ok = PublicData::get_value(wlan_checksum, get_rssi_checksum, 0, &rssidata);
    if (ok) {
        n = snprintf(buf, sizeof(buf), "|RSSI:%d", rssidata);
        if(n > sizeof(buf)) n = sizeof(buf);
        str.append(buf, n);
    }

    str.append("}\n");
    return str;
}

// Add a module to Kernel. We don't actually hold a list of modules we just call its on_module_loaded
void Kernel::add_module(Module* module)
{
    module->on_module_loaded();
}

// Adds a hook for a given module and event
void Kernel::register_for_event(_EVENT_ENUM id_event, Module *mod)
{
    this->hooks[id_event].push_back(mod);
}

bool Kernel::dispatch_console_line(SerialMessage& message)
{
    if (dispatching_console_line) return false;

    dispatching_console_line = true;
    call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    dispatching_console_line = false;
    return true;
}

// Call a specific event with an argument
void Kernel::call_event(_EVENT_ENUM id_event, void * argument)
{
    bool was_idle = true;
    if(id_event == ON_HALT) {
        this->halted = (argument == nullptr);
        if(!this->halted && this->feed_hold) this->feed_hold= false; // also clear feed hold
        was_idle = conveyor->is_idle(); // see if we were doing anything like printing
        void *returned_data;
        bool ok = PublicData::get_value( player_checksum, get_progress_checksum, &returned_data );
        if (ok) {
            struct pad_progress p =  *static_cast<struct pad_progress *>(returned_data);
            this->streams->printf("Halt Happened at Line: %lu,percent: %d, time: %lu \n", p.played_lines, p.percent_complete, p.elapsed_secs);
        }
    }

    // send to all registered modules
    if (id_event == ON_IDLE && debug_flags.cpu_load) {
        uint32_t t0 = us_ticker_read();
        for (auto m : hooks[id_event]) {
            (m->*kernel_callback_functions[id_event])(argument);
        }
        slow_ticker->idle_us_accum += us_ticker_read() - t0;
    } else {
        for (auto m : hooks[id_event]) {
            (m->*kernel_callback_functions[id_event])(argument);
        }
    }

    if(id_event == ON_HALT) {
        // If we just entered a halt state AND the debug flag is enabled, break into the debugger.
        // This happens after ON_HALT handlers have run, presumably stopping motion planners etc.
#if MRI_ENABLE
        if (this->halted && this->halt_on_error_debug) {
             __debugbreak(); // Enter debugger
        }
#endif

        if(!this->halted || !was_idle) {
            // if we were running and this is a HALT
            // or if we are clearing the halt with $X or M999
            // fix up the current positions in case they got out of sync due to backed up commands
            this->robot->reset_position_from_current_actuator_position();
        }
    }
}

// These are used by tests to test for various things. basically mocks
bool Kernel::kernel_has_event(_EVENT_ENUM id_event, Module *mod)
{
    for (auto m : hooks[id_event]) {
        if(m == mod) return true;
    }
    return false;
}

void Kernel::unregister_for_event(_EVENT_ENUM id_event, Module *mod)
{
    for (auto i = hooks[id_event].begin(); i != hooks[id_event].end(); ++i) {
        if(*i == mod) {
            hooks[id_event].erase(i);
            return;
        }
    }
}

void Kernel::read_eeprom_data()
{
	size_t size = sizeof(EEPROM_data);
	char i2c_buffer[size];

    short address = EEPROM_DATA_STARTPAGE*EEP_MAX_PAGE_SIZE;
    i2c_buffer[0] = (unsigned char)(address >> 8);
    i2c_buffer[1] = (unsigned char)((unsigned char)address & 0xff);

    this->i2c->start();
    this->i2c->write(0xA0);
    this->i2c->write(i2c_buffer[0]);
    this->i2c->write(i2c_buffer[1]);
    this->i2c->start();
    this->i2c->write(0xA1);

    for (size_t i = 0; i < size; i ++) {
    	i2c_buffer[i] = this->i2c->read(1);
    }

	this->i2c->stop();
	this->i2c->stop();

    wait(0.05);

    const bool stock_layout = std::all_of(i2c_buffer + sizeof(StockEepromData), i2c_buffer + size,
                                          [](char byte) { return static_cast<unsigned char>(byte) == 0xff; });
    if (!stock_layout) {
        memcpy(this->eeprom_data, i2c_buffer, size);
        return;
    }

    StockEepromData stored;
    memcpy(&stored, i2c_buffer, sizeof(stored));
    memset(this->eeprom_data, 0xff, sizeof(*this->eeprom_data));
    this->eeprom_data->TLO = stored.TLO;
    this->eeprom_data->REFMZ = stored.REFMZ;
    this->eeprom_data->TOOLMZ = stored.TOOLMZ;
    this->eeprom_data->reserve = stored.reserve;
    this->eeprom_data->TOOL = stored.TOOL;
    this->eeprom_data->WCScoord[0][0] = stored.G54[0];
    this->eeprom_data->WCScoord[0][1] = stored.G54[1];
    this->eeprom_data->WCScoord[0][2] = stored.G54[2];
    this->eeprom_data->WCScoord[0][3] = stored.G54AB[0];
}

void Kernel::write_eeprom_data()
{
	size_t size = sizeof(EEPROM_data);
	char Data_buffer[size];
	unsigned int writenum = 0;
	unsigned int result = 0;
	unsigned int pagenum = 0;
	unsigned int bytenum =0;
	unsigned char * writeptr = 0;
	unsigned int u8Pagebegin=EEPROM_DATA_STARTPAGE;

	memcpy(Data_buffer, this->eeprom_data, size);

	writeptr = (unsigned char *)Data_buffer;
	while(writenum < size)
	{
		bytenum = (size-pagenum*EEP_MAX_PAGE_SIZE) >= EEP_MAX_PAGE_SIZE ? EEP_MAX_PAGE_SIZE : size-pagenum*EEP_MAX_PAGE_SIZE;
		result = iic_page_write(u8Pagebegin+pagenum, bytenum, (unsigned char *)writeptr);
		wait(0.1);
		if(result == 0)
		{
			pagenum ++;
			writenum += bytenum;
			writeptr += bytenum;
		}
		else
		{
			break;
		}
	}
	if (result != 0) {
		this->streams->printf("ERROR: EEPROM data write error:%d\n",pagenum);
	} else {
//		this->streams->printf("EEPROM data write finished.\n");
	}
}

void Kernel::erase_eeprom_data()
{
	size_t size = sizeof(EEPROM_data);
	char Data_buffer[size];
	unsigned int writenum = 0;
	unsigned int result = 0;
	unsigned int pagenum = 0;
	unsigned int bytenum =0;
	unsigned char * writeptr = 0;
	unsigned int u8Pagebegin=EEPROM_DATA_STARTPAGE;

	memset(Data_buffer, 0, sizeof(Data_buffer));


	writeptr = (unsigned char *)Data_buffer;
	while(writenum < size)
	{
		bytenum = (size-pagenum*EEP_MAX_PAGE_SIZE) >= EEP_MAX_PAGE_SIZE ? EEP_MAX_PAGE_SIZE : size-pagenum*EEP_MAX_PAGE_SIZE;
		result = iic_page_write(u8Pagebegin+pagenum, bytenum, (unsigned char *)writeptr);
		wait(0.05);
		if(result == 0)
		{
			pagenum ++;
			writenum += bytenum;
			writeptr += bytenum;
		}
		else
		{
			break;
		}
	}
	if (result != 0) {
		this->streams->printf("ERROR: EEPROM data erase error.\n");
	} else {
		this->streams->printf("EEPROM data erase finished.\n");
	}
}

void Kernel::check_eeprom_data()
{
	bool needrewtite = false;
	if(isnan(this->eeprom_data->TLO))
	{
		this->eeprom_data->TLO = 0;
		needrewtite = true;
	}
	if(isnan(this->eeprom_data->REFMZ))
	{
		this->eeprom_data->REFMZ = 0;
		needrewtite = true;
	}
	if(isnan(this->eeprom_data->TOOLMZ))
	{
		this->eeprom_data->TOOLMZ = 0;
		needrewtite = true;
	}
	if(isnan(this->eeprom_data->reserve))
	{
		this->eeprom_data->reserve = 0;
		needrewtite = true;
	}
	if(isnan(this->eeprom_data->TOOL))
	{
		this->eeprom_data->TOOL = 0;
		needrewtite = true;
	}

    if(this->eeprom_data->current_wcs > 5 || this->eeprom_data->current_wcs < 0)
    {
        this->eeprom_data->current_wcs = 0;
        needrewtite = true;
    }
	
	for (int wcs_index = 0; wcs_index < 6; wcs_index++){
        if (isnan(this->eeprom_data->WCSrotation[wcs_index])){
            this->eeprom_data->WCSrotation[wcs_index] = 0;
            needrewtite = true;
        }
		for (int axis = 0; axis < 4; axis++) {
			if (isnan(this->eeprom_data->WCScoord[wcs_index][axis])){
				this->eeprom_data->WCScoord[wcs_index][axis] = 0;
				needrewtite = true;
			}
		}
	}
    if(!((this->eeprom_data->tool_not_calibrated & ~1) == 0))
	{
		this->eeprom_data->tool_not_calibrated = true;
		needrewtite = true;
	}
	if(needrewtite)
		this->write_eeprom_data();
}

void Kernel::dump_eeprom(StreamOutput *stream)
{
    constexpr uint16_t eeprom_size = 4096;
    constexpr uint16_t row_size = 16;
    unsigned char data[row_size];
    uint16_t crc = 0;

    for (uint16_t address = 0; address < eeprom_size; address += row_size) {
        this->i2c->is_timed_out();
        this->i2c->start();
        bool ok = this->i2c->write(0xA0) && this->i2c->write(address >> 8) && this->i2c->write(address & 0xff);
        if (ok) {
            this->i2c->start();
            ok = this->i2c->write(0xA1);
        }
        for (uint16_t i = 0; ok && i < row_size; ++i)
            data[i] = this->i2c->read(i + 1 < row_size ? mbed::I2C::ACK : mbed::I2C::NoACK);
        this->i2c->stop();
        ok = ok && !this->i2c->is_timed_out();
        if (!ok) {
            stream->printf("ERROR: EEPROM read failed at %04X\r\n", address);
            return;
        }

        crc = crc16::ccitt_update(crc, data, row_size);
        stream->printf("%04X:", address);
        for (unsigned char byte : data)
            stream->printf("%02X", byte);
        stream->printf("\r\n");
    }
    stream->printf("CRC16-CCITT:%04X\r\n", crc);
}

void Kernel::read_Factory_data()
{
	unsigned int size = sizeof(FACTORY_SET)+4;	//0x5A 0xA5 DATA CRC(2byte)
	char i2c_buffer[size];

    short address = EEPROM_FACTORYSET_PAGE*EEP_MAX_PAGE_SIZE;
    i2c_buffer[0] = (unsigned char)(address >> 8);
    i2c_buffer[1] = (unsigned char)((unsigned char)address & 0xff);

    this->i2c->start();
    this->i2c->write(0xA0);
    this->i2c->write(i2c_buffer[0]);
    this->i2c->write(i2c_buffer[1]);
    this->i2c->start();
    this->i2c->write(0xA1);

    for (size_t i = 0; i < size; i ++) {
    	i2c_buffer[i] = this->i2c->read(1);
    }

	this->i2c->stop();
	this->i2c->stop();

    wait(0.05);
	
	if( Check_Factory_Data((unsigned char*)i2c_buffer, sizeof(FACTORY_SET)+2 ) )
	{
    	memcpy(this->factory_set, &i2c_buffer[2], sizeof(FACTORY_SET));
    }
    else
    {
        *this->factory_set = default_factory_settings;
    }
    
    if(this->factory_set->MachineModel == Machine::carvera)
    {
    	this->factory_set->FuncSetting |= 0x04;
    }
}

bool Kernel::write_Factory_data()
{
	unsigned int size = sizeof(FACTORY_SET);
	unsigned int datalen = size + 4;
	char Data_buffer[datalen];
	unsigned int writenum = 0;
	unsigned int result = 0;
	unsigned int pagenum = 0;
	unsigned int bytenum =0;
	unsigned char * writeptr = 0;
	unsigned int u8Pagebegin=EEPROM_FACTORYSET_PAGE;
	
	Data_buffer[0] = 0x5A;
	Data_buffer[1] = 0xA5;
	memcpy(&Data_buffer[2], this->factory_set, sizeof(FACTORY_SET));

	unsigned short crc = crc16::ccitt((unsigned char*)Data_buffer, size+2);
	Data_buffer[size+2] = crc & 0xff;
	Data_buffer[size+3] = (crc>>8) & 0xff;

	writeptr = (unsigned char *)Data_buffer;
	while(writenum < datalen)
	{
		bytenum = (datalen-pagenum*EEP_MAX_PAGE_SIZE) >= EEP_MAX_PAGE_SIZE ? EEP_MAX_PAGE_SIZE : datalen-pagenum*EEP_MAX_PAGE_SIZE;
		result = iic_page_write(u8Pagebegin+pagenum, bytenum, (unsigned char *)writeptr);
		wait(0.1);
		if(result == 0)
		{
			pagenum ++;
			writenum += bytenum;
			writeptr += bytenum;
		}
		else
		{
			break;
		}
	}
	if (result != 0) {
		this->streams->printf("ERROR: FACTORY setting data write error:%d\n",pagenum);
		return false;
	}
	return true;
}

void Kernel::erase_Factory_data()
{
	unsigned int size = sizeof(FACTORY_SET)+4;	//5A A5 DATA CRC
	char Data_buffer[size];
	unsigned int writenum = 0;
	unsigned int result = 0;
	unsigned int pagenum = 0;
	unsigned int bytenum =0;
	unsigned char * writeptr = 0;
	unsigned int u8Pagebegin=EEPROM_FACTORYSET_PAGE;

	memset(Data_buffer, 0, sizeof(Data_buffer));


	writeptr = (unsigned char *)Data_buffer;
	while(writenum < size)
	{
		bytenum = (size-pagenum*EEP_MAX_PAGE_SIZE) >= EEP_MAX_PAGE_SIZE ? EEP_MAX_PAGE_SIZE : size-pagenum*EEP_MAX_PAGE_SIZE;
		result = iic_page_write(u8Pagebegin+pagenum, bytenum, (unsigned char *)writeptr);
		wait(0.05);
		if(result == 0)
		{
			pagenum ++;
			writenum += bytenum;
			writeptr += bytenum;
		}
		else
		{
			break;
		}
	}
	if (result != 0) {
		this->streams->printf("ERROR: FACTORY setting data erase error.\n");
	}
}

void Kernel::read_Factroy_SD()
{
	string file_name = "/sd/factory.ini";
	FILE *lp = fwfs::fopen(file_name.c_str(), "r");
	bool bneedwrite = false;
    FactorySettings settings(*factory_set);
    int ln= 1;
    if(lp) {
        // For each line
    	while(!fwfs::feof(lp)) {
        	string line;
        	if(Factroy_readLine(line, ln++, lp)) 
        	{ 
				const FactorySettings::LineResult result = settings.apply_line(line);
				if (result == FactorySettings::LineResult::missing_pair)
					streams->printf("ERROR: factory file line %s is invalid, no key value pair found\r\n", line.c_str());
				else if (result == FactorySettings::LineResult::missing_value)
					streams->printf("ERROR: factory file line %s has no value\r\n", line.c_str());
				bneedwrite = result == FactorySettings::LineResult::applied || bneedwrite;
        	}
        	else
        	{
        		break;	
        	}
    		
    	}
    	if(bneedwrite)
    	{
    		write_Factory_data();	
    	}
    	
    	fwfs::fclose(lp);
    	fwfs::remove("/sd/factory.ini");
    	system_reset(false);
    }
}
bool Kernel::Factroy_readLine(string& line, int lineno, FILE *fp)
{
    char buf[132];
    char *l= fwfs::fgets(buf, sizeof(buf)-1, fp);
    if(l != NULL) {
        if(buf[strlen(l)-1] != '\n') {
            // truncate long lines
            if(lineno != 0) {
                // report if it is not truncating a comment
                if(strchr(buf, '#') == NULL)
                    THEKERNEL->streams->printf("Truncated long line %d in: %s\n", lineno, "Factory file");
            }
            // read until the next \n or eof
            int c;
            while((c=fwfs::fgetc(fp)) != '\n' && c != EOF) /* discard */;
        }
        line.assign(buf);
        return true;
    }

    return false;
}

bool Kernel::Check_Factory_Data(unsigned char *data, unsigned int len)
{
	if((data[0] == 0x5A) && (data[1] == 0xA5))
	{
		unsigned short crc = crc16::ccitt(data, len);
		if( ((crc&0xff) == data[len]) && (((crc>>8)&0xff) == data[len+1]) )
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

int Kernel::iic_page_write(unsigned char u8PageNum, unsigned char u8len, unsigned char *pu8Array)
{
	unsigned int  	u16ByteAdd;
	unsigned char   u8HighAdd;
	unsigned char   u8LowAdd;

	u16ByteAdd = (unsigned int)u8PageNum;
	u16ByteAdd = (u16ByteAdd<<5);
	u8LowAdd = (unsigned char)u16ByteAdd;
	u8HighAdd = (unsigned char)(u16ByteAdd>>8);

	if (u8len == 0)
	{
		return 1;
	}


	this->i2c->is_timed_out();
	this->i2c->start();
	bool ok = this->i2c->write(0xA0) && this->i2c->write(u8HighAdd) && this->i2c->write(u8LowAdd);
	for(unsigned char i = 0; ok && i < u8len; ++i)
		ok = this->i2c->write(pu8Array[i]);
	this->i2c->stop();
	return ok && !this->i2c->is_timed_out() ? 0 : 1;
}


void Kernel::set_tool_waiting(bool f) { 
	this->tool_waiting = f; 
	if (!this->tool_waiting) {
		struct tool_status tool;
		PublicData::get_value( atc_handler_checksum, get_tool_status_checksum, &tool );
		if (tool.target_collet_type != 0){
			uint8_t collet_type = 0;
			PublicData::set_value( atc_handler_checksum, set_target_collet_type_checksum, &collet_type );
		}
	}
}
