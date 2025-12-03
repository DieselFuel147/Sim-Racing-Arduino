/*
 *  Project     Sim Racing Library for Arduino
 *  @author     David Madison
 *  @link       github.com/dmadison/Sim-Racing-Arduino
 *  @license    LGPLv3 - Copyright (c) 2024 David Madison
 *
 *  This file is part of the Sim Racing Library for Arduino.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 *  ---- New version ----
 *	 - Combined shifter and pedal inputs
 *   - Deep sleep
 *   - Long-press sleep button (1s)
 *   - Wake via interrupt
 *   - USB detach/attach
 *		@author Diesel Mitchell
 */

 /**
 * @details Emulates the Logitech G27 shifter and pedals as a joystick over USB.
 */

// This example requires the Arduino Joystick Library
// Download Here: https://github.com/MHeironimus/ArduinoJoystickLibrary

#include <SimRacing.h>
#include <Joystick.h>
#include <avr/sleep.h>
#include <avr/power.h>

// ---------------------- PIN DEFINITIONS ----------------------

// Sleep system pins
const int Pin_SleepLED   = 3;   // Shifter LED pin (also used for sleep status)
const int Pin_SleepButton = 2;  // Interrupt-capable pin for sleep/wake button

// Shifter pins
// Power (VCC): Shifter DE-9 pin 9
// Ground (GND): Shifter DE-9 pin 6
const int Pin_ShifterX      = A8;  // Shifter DE-9 pin 4
const int Pin_ShifterY      = A9;  // Shifter DE-9 pin 8

const int Pin_ShifterLatch  = 5;   // Shifter DE-9 pin 3
const int Pin_ShifterClock  = 6;   // Shifter DE-9 pin 1
const int Pin_ShifterData   = 7;   // Shifter DE-9 pin 2

// Pedal pins
// Power (VCC): Pedals DE-9 pin 1
// Ground (GND): Pedals DE-9 pin 9
const int Pin_Gas    = A2; // Pedals DE-9 pin 2
const int Pin_Brake  = A1; // Pedals DE-9 pin 3
const int Pin_Clutch = A0; // Pedals DE-9 pin 4

// This pin is optional! You do not need to connect it in order to read data from the shifter. 
// Connecting it and changing the pin number below will light the power LED.
// Note: Pin_SleepLED is already defined at the top for sleep status so should not be used here.
 const int Pin_ShifterLED = SimRacing::UnusedPin;  // DE-9 pin 5

// This pin requies a pull-down resistor! If you have made the proper connections, change the pin number to the one you're using. 
// Setting it will zero data when the shifter is disconnected.
const int Pin_ShifterDetect = SimRacing::UnusedPin;  // DE-9 pin 7

// ---------------------- SLEEP SYSTEM ----------------------

volatile bool wakeFlag = false;

void wakeISR() {
  wakeFlag = true;
}

void usbDetach() {
  UDCON = 1;   // detach USB
  USBCON = 0;  // disable USB controller
}

void usbAttach() {
  USBCON = 1;
  UDCON = 0;
  USBDevice.attach();
}

// Main deep sleep logic
void goToSleep() {
  wakeFlag = false;

	// Turn shifter LED off before sleep
	digitalWrite(Pin_SleepLED, HIGH);
	pinMode(Pin_SleepLED, OUTPUT);	

  // Turn off TX/RX LEDs before sleep
  pinMode(LED_BUILTIN_TX, INPUT);  // TX LED (Pin 30/PD5)
  pinMode(LED_BUILTIN_RX, INPUT);  // RX LED (Pin 17/PB0)

  // USB disconnect
  usbDetach();

  // Wait for button release BEFORE disabling timers
  while (digitalRead(Pin_SleepButton) == LOW);
  delay(50);  // Debounce while delay() still works

  // Disable power to peripherals
	power_all_disable();
  ADCSRA = 0;

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  attachInterrupt(digitalPinToInterrupt(Pin_SleepButton), wakeISR, LOW);
  sei();  // Enable interrupts

  sleep_cpu();  // Sleep - will wake on interrupt

  // ----------- WAKING UP HERE ------------------------

  // Disable sleep
  sleep_disable();

	// Detach interrupt
  detachInterrupt(digitalPinToInterrupt(Pin_SleepButton));
	
  // Re-enable power systems
	power_all_enable();

	// Reconnect USB
  usbAttach();

  // Re-enable TX/RX LEDs
  pinMode(LED_BUILTIN_TX, OUTPUT);
  pinMode(LED_BUILTIN_RX, OUTPUT);

	// Shifter LED on while awake
	digitalWrite(Pin_SleepLED, LOW);

  // Wait for button release
  while (digitalRead(Pin_SleepButton) == LOW);
  delay(100);  // Longer debounce after wake
}

// ---------------------- DEVICE INITIALIZATION ----------------------

SimRacing::LogitechShifterG27 shifter(
	Pin_ShifterX, Pin_ShifterY,
	Pin_ShifterLatch, Pin_ShifterClock, Pin_ShifterData,
	Pin_ShifterLED, Pin_ShifterDetect
);

SimRacing::LogitechPedals pedals(Pin_Gas, Pin_Brake, Pin_Clutch);

// ---------------------- JOYSTICK CONFIGURATION ----------------------

// Set this option to 'true' to send the shifter's X/Y position as a joystick. This is not needed for most games.
const bool SendAnalogAxis = false;

// Gear button mapping
const int Gears[] = { 1, 2, 3, 4, 5, 6, -1 };
const int NumGears = sizeof(Gears) / sizeof(Gears[0]);


// Shifter button mapping
using ShifterButton = SimRacing::LogitechShifterG27::Button;
const ShifterButton Buttons[] = {
	ShifterButton::BUTTON_SOUTH,
	ShifterButton::BUTTON_EAST,
	ShifterButton::BUTTON_WEST,
	ShifterButton::BUTTON_NORTH,
	ShifterButton::BUTTON_1,
	ShifterButton::BUTTON_2,
	ShifterButton::BUTTON_3,
	ShifterButton::BUTTON_4,
};
const int NumButtons = sizeof(Buttons) / sizeof(Buttons[0]);

// ADC configuration
const int ADC_Max = 1023;  // 10-bit on AVR
const bool AlwaysSend = false;  // override the position checks, *always* send data constantly

// Joystick initialization
Joystick_ Joystick(
	JOYSTICK_DEFAULT_REPORT_ID,      // default report (no additional pages)
	JOYSTICK_TYPE_JOYSTICK,          // so that this shows up in Windows joystick manager
	NumGears + NumButtons,           // number of buttons (7 gears: reverse and 1-6, 8 buttons)
	1,                               // number of hat switches (1, the directional pad)
	SendAnalogAxis, SendAnalogAxis,  // include X and Y axes for analog output, if set above
	pedals.hasPedal(SimRacing::Clutch),  // include Z axis for the clutch pedal
	pedals.hasPedal(SimRacing::Brake),   // include Rx axis for the brake pedal
	pedals.hasPedal(SimRacing::Gas),     // include Ry axis for the gas pedal
	false, false, false, false, false, false);  // no other axes

// Forward-declared function for non-Arduino environments
void updateJoystick();  

// ---------------------- SETUP ----------------------

void setup() {
  pinMode(Pin_SleepButton, INPUT_PULLUP);
  pinMode(Pin_SleepLED, OUTPUT);

	// Shifter LED on at startup
	digitalWrite(Pin_SleepLED, LOW);

	shifter.begin();
  pedals.begin();

	// If you have one, your calibration line should go here
	
	Joystick.begin(false);  // 'false' to disable auto-send
	Joystick.setXAxisRange(0, ADC_Max);
	Joystick.setYAxisRange(ADC_Max, 0);  // invert axis so 'up' is up
	Joystick.setZAxisRange(0, ADC_Max);
	Joystick.setRxAxisRange(0, ADC_Max);
	Joystick.setRyAxisRange(0, ADC_Max);

	updateJoystick();  // send initial state
}

// ---------------------- MAIN LOOP ----------------------

void loop() {
  // Sleep button long press detection (1 second)
  if (digitalRead(Pin_SleepButton) == LOW) {
    unsigned long t0 = millis();
    while (digitalRead(Pin_SleepButton) == LOW) {
      if (millis() - t0 > 1000) {
        goToSleep();
        break;
      }
    }
  }

	// Update shifter and send data if changed
	bool dataChanged = shifter.update();
	if (dataChanged || SendAnalogAxis == true) {
		updateJoystick();
	}

	// Update pedals and send data if changed
	pedals.update();
	if (pedals.positionChanged() || AlwaysSend) {
		updateJoystick();
	}
}

// ---------------------- JOYSTICK UPDATE ----------------------

void updateJoystick() {
	// Keep track of which button we're updating in the joystick output
	int currentButton = 0;

	// Set the buttons corresponding to the gears
	for (int i = 0; i < NumGears; i++) {
		if (shifter.getGear() == Gears[i]) {
			Joystick.pressButton(currentButton);
		}
		else {
			Joystick.releaseButton(currentButton);
		}

		currentButton++;
	}

	// Set the analog axes (if the option is set)
	if (SendAnalogAxis == true) {
		int x = shifter.getPosition(SimRacing::X, 0, ADC_Max);
		int y = shifter.getPosition(SimRacing::Y, 0, ADC_Max);
		Joystick.setXAxis(x);
		Joystick.setYAxis(y);
	}

	// Set the shifter buttons
	for (int i = 0; i < NumButtons; i++) {
		bool state = shifter.getButton(Buttons[i]);
		Joystick.setButton(currentButton, state);

		currentButton++;
	}

	// Set the hatswitch (directional pad)
	int angle = shifter.getDpadAngle();
	Joystick.setHatSwitch(0, angle);

	// Set pedal axes
	if (pedals.hasPedal(SimRacing::Gas)) {
		int gasPedal = pedals.getPosition(SimRacing::Gas, 0, ADC_Max);
		Joystick.setRyAxis(gasPedal);
	}

	if (pedals.hasPedal(SimRacing::Brake)) {
		int brakePedal = pedals.getPosition(SimRacing::Brake, 0, ADC_Max);
		Joystick.setRxAxis(brakePedal);
	}

	if (pedals.hasPedal(SimRacing::Clutch)) {
		int clutchPedal = pedals.getPosition(SimRacing::Clutch, 0, ADC_Max);
		Joystick.setZAxis(clutchPedal);
	}

	// Send the updated data via USB
	Joystick.sendState();
}
