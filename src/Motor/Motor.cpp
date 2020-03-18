/* mbed simple H-bridge motor controller
 * Copyright (c) 2007-2010, sford
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Motor.h"

Motor::Motor(PinName pwm, PinName fwd, PinName rev):
        _pwm(pwm), _fwd(fwd), _rev(rev) {

    // Set initial condition of PWM
    _pwm.period(0.002);
    _pwm = 0;

    // Initial condition of output enables
    _fwd = 0;
    _rev = 0;
}

void Motor::speed(float speed) {
    _fwd = (speed > (float)0.0);
    _rev = (speed < (float)0.0);
    _pwm = fabs(speed);
}

void Motor::period(float period){

    _pwm.period(period);

}

void Motor::brake(int highLow){

    if(highLow == BRAKE_HIGH){
        _pwm = 1;
        _fwd = 1;
        _rev = 1;
    }
    else if(highLow == BRAKE_LOW){
        _fwd = 0;
        _rev = 0;
    }

}

void Motor::forcebrake(){
    _pwm = 1;
    _fwd = 1;
    _rev = 1;
}
