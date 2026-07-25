#ifndef PIN_H
#define PIN_H

#include <stdlib.h>
#include <stdio.h>
#include <string>

// The device header. Must be the mbed one, not src/libs/LPC17xx/sLPC17xx.h:
// both declare the same peripheral types under the same include guard
// (__LPC17xx_H__), so whichever a translation unit reaches first silently wins
// and the other is preprocessed away. They do not agree - sLPC17xx.h puts the
// USB OTG I2C registers 4 bytes too high and sizes LPC_UART0_TypeDef at 456
// bytes instead of 92 - so which one a file happened to see decided its
// register layout. See the commit that unified this.
#include "LPC17xx.h"
#include "PinNames.h"

namespace mbed {
    class PwmOut;
    class InterruptIn;
}

class Pin {
    public:
        Pin();

        Pin* from_string(std::string value);

        inline bool connected(){
            return this->valid;
        }

        inline bool equals(const Pin& other) const {
            return (this->pin == other.pin) && (this->port == other.port);
        }

        inline Pin* as_output(){
            if (this->valid)
                this->port->FIODIR |= 1<<this->pin;
            return this;
        }

        inline Pin* as_input(){
            if (this->valid)
                this->port->FIODIR &= ~(1<<this->pin);
            return this;
        }

        Pin* as_open_drain(void);

        Pin* as_repeater(void);

        Pin* pull_up(void);

        Pin* pull_down(void);

        Pin* pull_none(void);

        inline bool get() const{
            if (!this->valid) return false;
            return this->inverting ^ (( this->port->FIOPIN >> this->pin ) & 1);
        }

        inline void set(bool value)
        {
            if (!this->valid) return;
            if ( this->inverting ^ value )
                this->port->FIOSET = 1 << this->pin;
            else
                this->port->FIOCLR = 1 << this->pin;
        }

        mbed::PwmOut *hardware_pwm();

        mbed::InterruptIn *interrupt_pin();

        bool is_inverting() const { return inverting; }
        void set_inverting(bool f) { inverting= f; }

        // these should be private, and use getters
        LPC_GPIO_TypeDef* port;

        unsigned char pin;
        char port_number;

    private:
        struct {
            bool inverting:1;
            bool valid:1;
        };
};




#endif
