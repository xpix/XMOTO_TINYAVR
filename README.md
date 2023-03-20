# Documentation for Motor Controller with PID Control to use a DC motor as stepper/servo

This code is designed for a motor controller and has merged the PID code of Josh Kopel with the code of the Makerbot servo-controller board. This code can be used on some boards by changing some values.

## Hardware Requirements

- motor controller as H-Bridge
- Hall Sensor PCB
- Ramps 1.4 stepper driver

## Software Requirements

- PID_v2.h library
- EEPROM.h library

## Code Explanation

### Motor Driver Type

The type of motor driver is defined using the `PWMCONTROL` variable. If `PWMCONTROL` is set to `1`, it means that the motor driver has only a PWM interface. In this case, the minimum and maximum values for PWM to turn the motor are defined using `PWMMIN` and `PWMMAX`.

### Pin Definitions

The pins used in the code are defined as follows:

- `encoder0PinA` and `encoder0PinB`: Pins for the Hall Sensor PCB
- `MotorIN1` and `MotorIN2`: Pins for the motor driver (if `PWMCONTROL` is set to `1`)
- `MotorDIR` and `MotorPWM`: Pins for the motor driver (if `PWMCONTROL` is set to `0`)
- `STEP_PIN` and `DIR_PIN`: Pins for the Ramps 1.4 stepper driver
- `EnableLED`: Pin for the LED that signals if the MCU is running

### PID Values

The default PID values are defined using the `MyObject` struct. The `kp`, `ki`, and `kd` values are set to `7.00`, `0.04`, and `0.20` respectively. These values can be changed by the user.

### Setup Function

In the `setup()` function, the following tasks are performed:

- Serial communication is started at a baud rate of 9600.
- The saved PID values are retrieved from the EEPROM. If no values are saved, the default values are used.
- The pins are set to their respective modes.
- Interrupts are attached to the `encoder0PinA` and `STEP_PIN` pins.
- The PID is set to automatic mode with a sample time of 2 and an output limit of -255 to 255.

### Loop Function

In the `loop()` function, the following tasks are performed:

- The set
