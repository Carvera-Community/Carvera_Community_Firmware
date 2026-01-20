/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#include "Gcode.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"
#include "utils.h"
#include <stdlib.h>
#include <algorithm>


//required for system variables
#include <cctype>
#include "libs/Kernel.h"
#include "Robot.h"
#include "PublicData.h"
#include "SpindlePublicAccess.h"
#include "StepperMotor.h"

// This is a gcode object. It represents a GCode string/command, and caches some important values about that command for the sake of performance.
// It gets passed around in events, and attached to the queue ( that'll change )
Gcode::Gcode(const string &command, StreamOutput *stream, bool strip, unsigned int line)
{
    this->command= strdup(command.c_str());
    this->m= 0;
    this->g= 0;
    this->subcode= 0;
    this->add_nl= false;
    this->is_error= false;
    this->stream= stream;
    prepare_cached_values(strip);
    this->stripped= strip;
    this->line = line;
}

Gcode::~Gcode()
{
    if(command != nullptr) {
        // TODO we can reference count this so we share copies, may save more ram than the extra count we need to store
        free(command);
    }
}

Gcode::Gcode(const Gcode &to_copy)
{
    this->command               = strdup(to_copy.command); // TODO we can reference count this so we share copies, may save more ram than the extra count we need to store
    this->has_m                 = to_copy.has_m;
    this->has_g                 = to_copy.has_g;
    this->m                     = to_copy.m;
    this->g                     = to_copy.g;
    this->subcode               = to_copy.subcode;
    this->add_nl                = to_copy.add_nl;
    this->is_error              = to_copy.is_error;
    this->stream                = to_copy.stream;
    this->txt_after_ok.assign( to_copy.txt_after_ok );
}

Gcode &Gcode::operator= (const Gcode &to_copy)
{
    if( this != &to_copy ) {
        this->command               = strdup(to_copy.command); // TODO we can reference count this so we share copies, may save more ram than the extra count we need to store
        this->has_m                 = to_copy.has_m;
        this->has_g                 = to_copy.has_g;
        this->m                     = to_copy.m;
        this->g                     = to_copy.g;
        this->subcode               = to_copy.subcode;
        this->add_nl                = to_copy.add_nl;
        this->is_error              = to_copy.is_error;
        this->stream                = to_copy.stream;
        this->txt_after_ok.assign( to_copy.txt_after_ok );
    }
    return *this;
}


// Whether or not a Gcode has a letter
bool Gcode::has_letter( char letter ) const
{
    for (size_t i = 0; i < strlen(this->command); ++i) {
        if( command[i] == letter ) {
            return true;
        }
    }
    return false;
}

//2024
/*
int Gcode::index_of_letter( char letter, int start ) const
{
    for (size_t i = start; i < strlen(this->command); ++i) {
        if( command[i] == letter ) {
            return i;
        }
    }
    return -1;
}
*/

float Gcode::set_variable_value() const {
    // Expecting a number after the `#` from 1-20, like #12
    const char* expr = this->get_command();
    if (*expr == '#') {
        char* endptr;
        float value = 0;
        int var_num = strtol(expr + 1, &endptr, 10); 

        while (isspace(*endptr)) endptr++; // Skip whitespace

        // Check if the next character is '=' indicating an assignment
        if (*endptr == '=') {
            endptr++; // Move past '='
            while (isspace(*endptr)) endptr++; // Skip whitespace

            value = evaluate_expression(endptr, &endptr);
            // Check if the expression evaluated to NAN
            if ((value != value)) {
                this->stream->printf("Error in expression evaluation, cannot set variable %d\n", var_num);
                return NAN; // Stop execution and do not set the variable
            }
        } else {
            // If it's not an assignment, get the variable value
            char* temp_expr = const_cast<char*>(expr);
            value = this->get_variable_value(expr, &temp_expr);

            // Check if the retrieved value is valid
            if (value == value) {
                this->stream->printf("Variable %d = %.4f \n", var_num, value);
            } else {
                // If the variable is not set, return early
                return NAN;
            }
            return NAN; // End the function since we only want to get the value, not set it
        }

        // Proceed to set the variable if it's valid
        if (var_num >= 101 && var_num <= 120) {
            THEKERNEL->local_vars[var_num - 101] = value; // Set local variable
            this->stream->printf("Variable %d set %.4f \n", var_num, value);
            return value;
        }else if(var_num == 150)
        {  
            if (value > 0 && value < 10){
                THEKERNEL->probe_tip_diameter = value;
                this->stream->printf("Probe tip diameter set %.4f \n", value);
                this->stream->printf("This value is temporary \n it will neeed to be saved to the config file with \n");
                this->stream->printf("config-set sd zprobe.probe_tip_diameter # \n");
                return value;
            }
            else{
                this->stream->printf("Probe tip input out of range, aborting \n", var_num);
                return NAN;
            }
        } else if (var_num >= 501 && var_num <= 520) {
            THEKERNEL->eeprom_data->perm_vars[var_num - 501] = value; // Set permanent variable
            THEKERNEL->write_eeprom_data(); // Save to EEPROM
            this->stream->printf("Variable %d set %.4f \n", var_num, value);
            return value;
        } else {
            // If the variable number is out of the expected range, print an error
            this->stream->printf("Variable not found \n");
            return NAN; // Variable not found
        }
    }

    // If the input doesn't start with '#', print an error message
    this->stream->printf("Variable not found \n");
    return 0; // Default return value
}


//get the value of a particular variable stored in EEPROM
float Gcode::get_variable_value(const char* expr, char** endptr) const{
    // Expecting a number after the `#` from 1-20, like #12
    if (*expr == '#') {
        int var_num = strtol(expr + 1, endptr, 10);         
        if (var_num >= 101 && var_num <= 120) {
            if (THEKERNEL->local_vars[var_num -101] > -100000)
            {
                return THEKERNEL->local_vars[var_num -101];
            }
            THEKERNEL->set_halt_reason(MANUAL);
            THEKERNEL->call_event(ON_HALT, nullptr);
            THEKERNEL->streams->printf("Variable %d not set \n", var_num);
            return NAN;
        
        } else if(var_num == 150)
        {
            return THEKERNEL->probe_tip_diameter;
        } else if(var_num >= 151 && var_num <= 156)
        {
            if (THEKERNEL->probe_outputs[var_num - 151] > -100000)
            {
                return THEKERNEL->probe_outputs[var_num - 151];
            }
            THEKERNEL->set_halt_reason(MANUAL);
            THEKERNEL->call_event(ON_HALT, nullptr);
            THEKERNEL->streams->printf("Variable %d not set \n", var_num);
            return NAN;

        } else if(var_num >= 501 && var_num <= 520)
        {
            if (THEKERNEL->eeprom_data->perm_vars[var_num - 501] > -100000)
            {
                return THEKERNEL->eeprom_data->perm_vars[var_num - 501]; // return permanent variables
            }
            
            THEKERNEL->set_halt_reason(MANUAL);
            THEKERNEL->call_event(ON_HALT, nullptr);
            THEKERNEL->streams->printf("Variable %d not set \n", var_num);
            return NAN;
        }else //system variables
        {
            float mpos[3];
            bool ok;
            Robot::wcs_t pos;
            switch (var_num){
                case 2000: //stored tool length offset
                    return THEKERNEL->eeprom_data->TLO;
                    break;
                case 3026: //tool in spindle
                    return THEKERNEL->eeprom_data->TOOL;
                    break;
                case 3027: //current spindle RPM
                    struct spindle_status ss;
                    ok = PublicData::get_value(pwm_spindle_control_checksum, get_spindle_status_checksum, &ss);
                    if (ok) {
                        return ss.current_rpm;
                        break;
                    }
                    return 0;
                    break;
                case 3033: //Op Stop Enabled
                    return THEKERNEL->get_optional_stop_mode();
                    break;
                case 5021: //current machine X position
                    THEROBOT->get_current_machine_position(mpos);
                    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
                    if(THEROBOT->pitchCompensationTransform) THEROBOT->pitchCompensationTransform(mpos, true, false); // get inverse pitch compensation transform
                    if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, false); // get inverse compensation transform
                    return mpos[X_AXIS];
                    break;
                case 5022: //current machine Y position
                    THEROBOT->get_current_machine_position(mpos);
                    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
                    if(THEROBOT->pitchCompensationTransform) THEROBOT->pitchCompensationTransform(mpos, true, false); // get inverse pitch compensation transform
                    if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, false); // get inverse compensation transform
                    return mpos[Y_AXIS];
                    break;
                case 5023: //current machine Z position
                    THEROBOT->get_current_machine_position(mpos);
                    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
                    if(THEROBOT->pitchCompensationTransform) THEROBOT->pitchCompensationTransform(mpos, true, false); // get inverse pitch compensation transform
                    if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, false); // get inverse compensation transform
                    return mpos[Z_AXIS];
                    break;

                #if MAX_ROBOT_ACTUATORS > 3
                case 5024: //current machine A position
                    return THEROBOT->actuators[A_AXIS]->get_current_position();
                    break;
                #endif
                case 5041: //current WCS X position
                     THEROBOT->get_current_machine_position(mpos);
                    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
                    if(THEROBOT->pitchCompensationTransform) THEROBOT->pitchCompensationTransform(mpos, true, false); // get inverse pitch compensation transform
                    if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, false); // get inverse compensation transform
                    pos= THEROBOT->mcs2wcs(mpos);
                    return THEROBOT->from_millimeters(std::get<X_AXIS>(pos));
                    return 0;
                    break;
                case 5042: //current WCS Y position
                     THEROBOT->get_current_machine_position(mpos);
                    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
                    if(THEROBOT->pitchCompensationTransform) THEROBOT->pitchCompensationTransform(mpos, true, false); // get inverse pitch compensation transform
                    if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, false); // get inverse compensation transform
                    pos= THEROBOT->mcs2wcs(mpos);
                    return THEROBOT->from_millimeters(std::get<Y_AXIS>(pos));
                    return 0;
                    break;
                case 5043: //current WCS A position
                     THEROBOT->get_current_machine_position(mpos);
                    // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
                    if(THEROBOT->pitchCompensationTransform) THEROBOT->pitchCompensationTransform(mpos, true, false); // get inverse pitch compensation transform
                    if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true, false); // get inverse compensation transform
                    pos= THEROBOT->mcs2wcs(mpos);
                    return THEROBOT->from_millimeters(std::get<Z_AXIS>(pos));
                    return 0;
                    break;
                #if MAX_ROBOT_ACTUATORS > 3
                case 5044: //current machine A position
                    return THEROBOT->actuators[A_AXIS]->get_current_position();
                    break;
                #endif

                default:
                    THEKERNEL->set_halt_reason(MANUAL);
                    THEKERNEL->call_event(ON_HALT, nullptr);
                    THEKERNEL->streams->printf("Variable %d not found \n", var_num);
                    return NAN;
                    break;
            }
        }
    }
    return 0;
}

float Gcode::parse_expression(const char*& expr) const {
    if (*expr == ']') {
        THEKERNEL->set_halt_reason(MANUAL);
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->streams->printf("Mismatched closing bracket ']' without opening '['\n");
        return NAN;
    }

    float result = parse_term(expr);

    while (isspace(*expr)) expr++; // Skip leading whitespace

    // Handle addition and subtraction
    while (*expr == '+' || *expr == '-') {
        char op = *expr;
        expr++; // Skip operator
        float next_term = parse_term(expr);
        if (op == '+') {
            result += next_term;
        } else {
            result -= next_term;
        }
        while (isspace(*expr)) expr++; // Skip whitespace after term
    }

    const float equal_tolerance = 1e-6;
    // Handle EQ,NE,GE,LE,GT,LT operator with lower precedence
    if (strncmp(expr, "eq", 2) == 0) {
        expr += 2; // Skip "EQ"
        while (isspace(*expr)) expr++;
        float comparison_value = parse_expression(expr);
        // Perform a fuzzy equality check with tolerance of 1e-6
        result = (fabs(result - comparison_value) < equal_tolerance) ? 1.0f : 0.0f;
    } else if(strncmp(expr, "ne", 2) == 0) {
        expr += 2; // Skip "NE"
        while (isspace(*expr)) expr++; 
        float comparison_value = parse_expression(expr);
        result = (fabs(result - comparison_value) < equal_tolerance) ? 0.0f : 1.0f;
    } else if(strncmp(expr, "gt", 2) == 0) {
        expr += 2; // Skip "GT"
        while (isspace(*expr)) expr++;
        float comparison_value = parse_expression(expr);
        result = (result > comparison_value) ? 1.0f : 0.0f;
    } else if(strncmp(expr, "ge", 2) == 0) {
        expr += 2; // Skip "GE"
        while (isspace(*expr)) expr++;
        float comparison_value = parse_expression(expr);        
        result = (result >= comparison_value) ? 1.0f : 0.0f;
    } else if(strncmp(expr, "lt", 2) == 0) {
        expr += 2; // Skip "LT"
        while (isspace(*expr)) expr++;
        float comparison_value = parse_expression(expr);
        result = (result < comparison_value) ? 1.0f : 0.0f;
    } else if(strncmp(expr, "le", 2) == 0) {
        expr += 2; // Skip "LE"
        while (isspace(*expr)) expr++;
        float comparison_value = parse_expression(expr);
        result = (result <= comparison_value) ? 1.0f : 0.0f;
    }

    // Handle boolean operators with the lowest precedence
    if (strncmp(expr, "and", 3) == 0) {
        expr += 3; // Skip "AND"
        while (isspace(*expr)) expr++; // Skip whitespace after "AND"
        float and_value = parse_expression(expr);
        // Evaluate AND: both values must be non-zero to return 1
        result = (result != 0 && and_value != 0) ? 1.0f : 0.0f;
    } else if (strncmp(expr, "or", 2) == 0) {
        expr += 2; // Skip "OR"
        while (isspace(*expr)) expr++; // Skip whitespace after "OR"
        float or_value = parse_expression(expr);
        result = (result != 0 || or_value != 0) ? 1.0f : 0.0f;
    } else if (strncmp(expr, "xor", 3) == 0) {
        expr += 3; // Skip "XOR"
        while (isspace(*expr)) expr++; // Skip whitespace after "XOR"
        float xor_value = parse_expression(expr);
        // Evaluate XOR: exactly one value must be non-zero to return 1
        result = ((result != 0) != (xor_value != 0)) ? 1.0f : 0.0f;
    } else if (strncmp(expr, "nor", 3) == 0) {
        expr += 3; // Skip "NOR"
        while (isspace(*expr)) expr++; // Skip whitespace after "NOR"
        float nor_value = parse_expression(expr);
        // Evaluate NOR: both values must be zero to return 1
        result = (result == 0 && nor_value == 0) ? 1.0f : 0.0f;
    }

    return result;
}

float Gcode::parse_term(const char*& expr) const {
    float result = parse_factor(expr);

    while (isspace(*expr)) expr++; // Skip leading whitespace

    while (*expr == '*' || *expr == '/' || strncmp(expr, "mod", 3) == 0) {
        if (*expr == '*' || *expr == '/') {
            char op = *expr;
            expr++; // Skip operator
            float next_factor = parse_factor(expr);
            if (op == '*') {
                result *= next_factor;
            } else {
                if (next_factor != 0) {
                    result /= next_factor;
                } else {
                    THEKERNEL->set_halt_reason(MANUAL);
                    THEKERNEL->call_event(ON_HALT, nullptr);
                    THEKERNEL->streams->printf("Division by zero\n");
                    return NAN;
                }
            }
        } else if (strncmp(expr, "mod", 3) == 0) { // Check for "MOD"
            expr += 3; // Skip "MOD"
            while (isspace(*expr)) expr++; // Skip whitespace after MOD
            float next_factor = parse_factor(expr);
            if (next_factor != 0) {
                result = fmod(result, next_factor);
            } else {
                THEKERNEL->set_halt_reason(MANUAL);
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->streams->printf("Modulo by zero\n");
                return NAN;
            }
        }

        while (isspace(*expr)) expr++; // Skip whitespace between terms
    }

    return result;
}

float Gcode::parse_factor(const char*& expr) const {
    while (isspace(*expr)) expr++; // Skip whitespace

    float result = NAN;

    // Handle functions with arguments in brackets
    if (strncmp(expr, "sin", 3) == 0 || strncmp(expr, "cos", 3) == 0 || strncmp(expr, "tan", 3) == 0 ||
        strncmp(expr, "asin", 4) == 0 || strncmp(expr, "acos", 4) == 0 || strncmp(expr, "atan", 4) == 0 ||
        strncmp(expr, "sqrt", 4) == 0 || strncmp(expr, "abs", 3) == 0 || strncmp(expr, "round", 5) == 0 ||
        strncmp(expr, "fix", 3) == 0 || strncmp(expr, "fup", 3) == 0 || strncmp(expr, "ln", 2) == 0 ||
        strncmp(expr, "exp", 3) == 0) 
    {
        const char* func_name = expr;
        // Adjust expr pointer based on function name length
        if (strncmp(func_name, "asin", 4) == 0 || strncmp(func_name, "acos", 4) == 0 || 
            strncmp(func_name, "atan", 4) == 0 || strncmp(func_name, "sqrt", 4) == 0) {
            expr += 4;
        } else if (strncmp(func_name, "round", 5) == 0) {
            expr += 5;
        } else if (strncmp(func_name, "ln", 2) == 0) {
            expr += 2;
        } else {
            expr += 3; // For 3-character functions
        }

        if (*expr == '[') {
            expr++; // Skip '['
            float arg = parse_expression(expr); // Parse the expression inside the brackets
            if (*expr == ']') {
                expr++; // Skip ']'
                const float DEG_TO_RAD = 3.141592653589793238463 / 180.0;
                if (strncmp(func_name, "sin", 3) == 0) {
                    result = sin(arg * DEG_TO_RAD);
                } else if (strncmp(func_name, "cos", 3) == 0) {
                    result = cos(arg * DEG_TO_RAD);
                } else if (strncmp(func_name, "asin", 3) == 0) {
                    result = asin(arg)/DEG_TO_RAD;
                } else if (strncmp(func_name, "acos", 3) == 0) {
                    result = acos(arg)/ DEG_TO_RAD;
                } else if (strncmp(func_name, "tan", 3) == 0) {
                    if (fmod(arg - 90, 180) == 0) { // Handle undefined tangent values
                        result = NAN;
                    } else {
                        result = tan(arg * DEG_TO_RAD);
                    }
                } else if (strncmp(func_name, "atan", 3) == 0) {
                    result = atan(arg)/DEG_TO_RAD;
                } else if (strncmp(func_name, "sqrt", 4) == 0) {
                    result = sqrt(arg);
                } else if (strncmp(func_name, "abs", 3) == 0) {
                    result = fabs(arg);
                } else if (strncmp(func_name, "round", 5) == 0) {
                    result = round(arg);
                } else if (strncmp(func_name, "fix", 3) == 0) {
                    result = floor(arg);
                } else if (strncmp(func_name, "fup", 3) == 0) {
                    result = ceil(arg);
                } else if (strncmp(func_name, "ln", 2) == 0) {
                    result = log(arg);
                } else if (strncmp(func_name, "exp", 3) == 0) {
                    result = exp(arg);
                }
            } else {
                THEKERNEL->set_halt_reason(MANUAL);
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->streams->printf("Mismatched brackets in function argument\n");
                return NAN;
            }
        } else {
            THEKERNEL->set_halt_reason(MANUAL);
            THEKERNEL->call_event(ON_HALT, nullptr);
            THEKERNEL->streams->printf("Expected '[' after function name\n");
            return NAN;
        }
    } else if (*expr == '[') {
        expr++; // Skip '['
        result = parse_expression(expr); // Parse the expression inside the brackets
        if (*expr == ']') {
            expr++; // Skip ']'
        } else {
            THEKERNEL->set_halt_reason(MANUAL);
            THEKERNEL->call_event(ON_HALT, nullptr);
            THEKERNEL->streams->printf("Mismatched brackets in expression\n");
            return NAN;
        }
    } else if (*expr == '#') {
        result = this->get_variable_value(expr, const_cast<char**>(&expr));
    } else {
        const char* original_expr = expr;
        while (*expr != '\0' && *expr != 'E' && *expr != 'e') {
            expr++;
        }

        char temp_char = *expr;
        *const_cast<char*>(expr) = '\0';

        // Parse the number without the 'E' or 'e'
        char* end;
        result = strtof(original_expr, &end);

        *const_cast<char*>(expr) = temp_char;

        if (end == original_expr) {
            THEKERNEL->set_halt_reason(MANUAL);
            THEKERNEL->call_event(ON_HALT, nullptr);
            THEKERNEL->streams->printf("Invalid number in expression, %c\n", *expr);
            return NAN;
        }
        expr = end;
    }

    // Handle exponentiation (e.g., 2^3)
    while (*expr == '^') {
        expr++;
        float exponent = parse_factor(expr);
        result = pow(result, exponent);
    }

    return result;
}

float Gcode::evaluate_expression(const char* expr, char** endptr) const {
    while (isspace(*expr)) expr++; // Skip leading whitespace

    // Check for unexpected closing bracket at the beginning
    if (*expr == ']') {
        THEKERNEL->set_halt_reason(MANUAL);
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->streams->printf("Mismatched closing bracket ']' without opening '['\n");
        return NAN;
    }

    float result = parse_expression(expr);

    // Ensure any remaining unmatched brackets are caught
    if (*expr == ']') {
        THEKERNEL->set_halt_reason(MANUAL);
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->streams->printf("Mismatched closing bracket at end of expression\n");
        return NAN;
    }

    if (endptr) {
        *endptr = const_cast<char*>(expr); // Set endptr to the current position
    }
    return result;
}

// Retrieve the value for a given letter
float Gcode::get_value( char letter, char **ptr ) const
{
    const char *cs = command;
    char *cn = NULL;
    for (; *cs; cs++) {
        if( letter == *cs ) {
            cs++;
            float result = this->evaluate_expression(cs, &cn);
            if(ptr != nullptr) *ptr = cn;
            
            // If a valid expression was found, return the result
            if (cn > cs)
                return result;
        }
    }
    // If no valid number or expression is found, return 0
    if (ptr != nullptr) *ptr = nullptr;
    return 0;
}

// 2024
/*
// Retrieve the value for a given letter
float Gcode::get_value_at_index( int index ) const
{
    const char *cs = command + index + 1;
    char *cn = NULL;
	float r = strtof(cs, &cn);
	if(cn > cs)
		return r;

    return 0;
}*/

int Gcode::get_int( char letter, char **ptr ) const
{
    const char *cs = command;
    char *cn = NULL;
    for (; *cs; cs++) {
        if( letter == *cs ) {
            cs++;
            int r = strtol(cs, &cn, 10);
            if(ptr != nullptr) *ptr= cn;
            if (cn > cs)
                return r;
        }
    }
    if(ptr != nullptr) *ptr= nullptr;
    return 0;
}

uint32_t Gcode::get_uint( char letter, char **ptr ) const
{
    const char *cs = command;
    char *cn = NULL;
    for (; *cs; cs++) {
        if( letter == *cs ) {
            cs++;
            int r = strtoul(cs, &cn, 10);
            if(ptr != nullptr) *ptr= cn;
            if (cn > cs)
                return r;
        }
    }
    if(ptr != nullptr) *ptr= nullptr;
    return 0;
}

int Gcode::get_num_args() const
{
    int count = 0;
    for(size_t i = stripped?0:1; i < strlen(command); i++) {
        if( this->command[i] >= 'A' && this->command[i] <= 'Z' ) {
            if(this->command[i] == 'T') continue;
            count++;
        }
    }
    return count;
}

std::map<char,float> Gcode::get_args() const
{
    std::map<char,float> m;
    for(size_t i = stripped?0:1; i < strlen(command); i++) {
        char c= this->command[i];
        if( c >= 'A' && c <= 'Z' ) {
            if(c == 'T') continue;
            m[c]= get_value(c);
        }
    }
    return m;
}

std::map<char,int> Gcode::get_args_int() const
{
    std::map<char,int> m;
    for(size_t i = stripped?0:1; i < strlen(command); i++) {
        char c= this->command[i];
        if( c >= 'A' && c <= 'Z' ) {
            if(c == 'T') continue;
            m[c]= get_int(c);
        }
    }
    return m;
}

// Cache some of this command's properties, so we don't have to parse the string every time we want to look at them
void Gcode::prepare_cached_values(bool strip)
{
    char *p= nullptr;

    if( this->has_letter('G') ) {
        this->has_g = true;
        this->g = this->get_int('G', &p);

    } else {
        this->has_g = false;
    }

    if( this->has_letter('M') ) {
        this->has_m = true;
        this->m = this->get_int('M', &p);

    } else {
        this->has_m = false;
    }


    if(has_g || has_m) {
        // look for subcode and extract it
        if(p != nullptr && *p == '.') {
            this->subcode = strtoul(p+1, &p, 10);

        }else{
            this->subcode= 0;
        }
    }

    if(!strip || this->has_letter('T')) return;

    // remove the Gxxx or Mxxx from string
    if (p != nullptr) {
        char *n= strdup(p); // create new string starting at end of the numeric value
        free(command);
        command= n;
    }
}

// strip off X Y Z I J K parameters if G0/1/2/3
void Gcode::strip_parameters()
{
    if(has_g && g < 4){
        // strip the command of the XYZIJK parameters
        string newcmd;
        char *cn= command;
        // find the start of each parameter
        char *pch= strpbrk(cn, "XYZIJK");
        while (pch != nullptr) {
            if(pch > cn) {
                // copy non parameters to new string
                newcmd.append(cn, pch-cn);
            }
            // find the end of the parameter and its value
            char *eos;
            strtof(pch+1, &eos);
            cn= eos; // point to end of last parameter
            pch= strpbrk(cn, "XYZIJK"); // find next parameter
        }
        // append anything left on the line
        newcmd.append(cn);

        // strip whitespace to save even more, this causes problems so don't do it
        //newcmd.erase(std::remove_if(newcmd.begin(), newcmd.end(), ::isspace), newcmd.end());

        // release the old one
        free(command);
        // copy the new shortened one
        command= strdup(newcmd.c_str());
    }
}
