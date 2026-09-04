# Embedded BLDC Motor Control in C

These are my technical notes and C references from studying BLDC motor control and working with small embedded motor-control experiments.

The main focus is the complete control path:

```text
command -> control algorithm -> PWM -> power stage -> motor
                                      ^              |
                                      |              v
                                protection <- sensors/feedback
```

I use the ATmega328P examples for hands-on work with timers, PWM, sensor input, control logic and hardware/software troubleshooting. The sections on three-phase commutation, sensorless control, Field-Oriented Control and CAN are notes from studying how larger motor controllers are structured.

---

## 1. How I break down a motor-control system

A motor controller is more than a function that writes a PWM value. I separate the firmware into a few responsibilities so each part can be tested on its own.

```text
+--------------------------------------------------+
| Application                                      |
| start / stop / direction / target speed          |
+--------------------------------------------------+
| Control                                          |
| ramp / PID / output limits                       |
+--------------------------------------------------+
| Motor                                            |
| PWM / commutation / speed measurement            |
+--------------------------------------------------+
| Protection                                       |
| temperature / current / voltage / stall / sensor |
+--------------------------------------------------+
| Hardware interface                               |
| timers / ADC / GPIO / interrupts                 |
+--------------------------------------------------+
| Hardware                                         |
| MCU / power stage / motor / sensors              |
+--------------------------------------------------+
```

This separation is useful during debugging. If the motor does not behave as expected, I can check the path in order: requested command, PWM output, power stage, motor response and sensor feedback.

---

## 2. BLDC motor basics

A brushless DC motor uses electronic commutation instead of mechanical brushes.

The important parts are:

- rotor with permanent magnets
- stator windings
- electronic switching stage
- rotor-position information or estimation
- control logic

For a raw three-phase BLDC motor, the controller has to switch the motor phases in the correct sequence. Common methods include:

- six-step / trapezoidal commutation
- Hall-sensor commutation
- sensorless back-EMF detection
- sinusoidal control
- Field-Oriented Control (FOC)

For smaller experiments with a 12 V BLDC fan that already contains its own commutation electronics, I can control the delivered power externally using PWM and a switching stage.

A raw three-phase motor is different. In that case the MCU normally controls a three-phase inverter through a gate-driver stage.

---

## 3. Power stage

A microcontroller output pin is a control signal, not a motor power source.

The ATmega328P works with low-voltage GPIO and limited pin current, while a motor can require much more current and a separate supply voltage. The MCU therefore controls a switching device or driver rather than powering the motor directly.

A simple low-side switching arrangement looks like this:

```text
 +12 V
   |
   |
 BLDC motor / fan
   |
   +------ Drain
           |
        N-MOSFET
           |
   +------ Source
   |
  GND

 MCU PWM pin ------> Gate
 MCU GND ----------- GND
 12 V supply GND --- GND
```

When choosing a MOSFET I check more than the current rating. Important values include:

- drain-source voltage rating
- drain current rating
- `RDS(on)`
- gate charge
- power dissipation
- gate-drive requirements

The gate-threshold voltage is not the same as the voltage required for low `RDS(on)`. The datasheet has to be checked at the gate voltage available from the controller.

For larger BLDC systems, the switching stage normally uses multiple MOSFETs and a dedicated gate driver.

---

## 4. PWM control

PWM controls the average power by switching the output quickly between ON and OFF states.

```text
duty cycle = ON time / PWM period
```

Examples:

```text
0%    output always OFF
25%   ON for one quarter of the period
50%   ON for half of the period
75%   ON for three quarters of the period
100%  output continuously ON
```

One important point is that PWM duty cycle is not the same thing as motor speed.

A 60% PWM command means I am requesting 60% duty cycle. The actual RPM still depends on the supply, motor, load, driver, friction, temperature and the way the motor is controlled.

This is the difference between commanding an actuator and measuring its real response.

---

## 5. Open-loop and closed-loop control

### Open loop

In open-loop control the firmware writes an output without using measured speed to correct it.

```text
target -> PWM -> motor
```

The code is simple and useful for basic testing, but the speed can change when the mechanical load changes.

Example:

```c
motor_set_duty(60);
```

That means 60% duty cycle. It does not mean 60% of maximum RPM.

### Closed loop

Closed-loop control adds feedback.

```text
 target RPM
     |
     v
  +-------+
  | error | <-----------------------------+
  +-------+                               |
     |                                    |
     v                                    |
  +-------+      +-------+      +-------+ |
  |  PID  | ---> |  PWM  | ---> | motor | |
  +-------+      +-------+      +-------+ |
                                      |    |
                                      v    |
                                  speed ---+
                                  sensor
```

The basic speed error is:

```text
error = target_speed - measured_speed
```

The controller changes the output so the measured speed moves toward the target speed.

---

## 6. PID speed control

PID uses three terms:

- **P**: reacts to the current error
- **I**: reacts to accumulated error
- **D**: reacts to how quickly the error changes

The continuous form is:

```text
u(t) = Kp*e(t) + Ki*integral(e) + Kd*de/dt
```

For embedded software I care about the implementation details as much as the equation. The control loop should run at a known interval and the output should always be limited to the range the actuator can accept.

A small C implementation:

```c
#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float previous_error;

    float output_min;
    float output_max;
} pid_controller_t;

static float clamp_float(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

float pid_update(pid_controller_t *pid,
                 float setpoint,
                 float measurement,
                 float dt)
{
    float error = setpoint - measurement;
    float proportional = pid->kp * error;

    pid->integral += error * dt;

    float derivative =
        (error - pid->previous_error) / dt;

    float output =
        proportional +
        pid->ki * pid->integral +
        pid->kd * derivative;

    output = clamp_float(
        output,
        pid->output_min,
        pid->output_max
    );

    pid->previous_error = error;

    return output;
}
```

The important practical points are:

- fixed control-loop timing
- output saturation
- integral windup handling
- stable sensor measurements
- predictable reset behavior
- disabling or limiting the controller during faults

PID tuning depends on the real motor, load, inertia and feedback signal. I treat tuning as part of hardware testing rather than only a software calculation.

---

## 7. Sensor feedback

The controller needs reliable measurements before it can make reliable decisions.

### Hall sensors

Hall sensors can provide rotor-sector information and are commonly used for:

- commutation timing
- rotor position
- speed calculation
- stall detection

### Tachometer signal

Some BLDC fans provide a tachometer output. RPM can be calculated from the pulse frequency:

```text
RPM = (pulse_frequency * 60) / pulses_per_revolution
```

### Encoder

An encoder can provide higher-resolution position and speed feedback.

### Current measurement

Current information is useful for detecting:

- excessive load
- overcurrent
- stall conditions
- short circuits
- torque changes

### Voltage measurement

Voltage feedback can be used for:

- undervoltage detection
- overvoltage detection
- battery monitoring
- supply diagnostics

### Temperature measurement

Temperature feedback can protect the motor, switching devices and other power electronics.

### Environmental input

A sensor can also affect the requested motor behavior. In one of my small embedded experiments I use a light-sensitive input as a control condition and read it through the MCU input path before deciding the motor output state.

---

## 8. Fault handling

Fault handling should be part of the normal control design, not separate from it.

Typical motor-control faults include:

| Fault | Detection idea | Controller reaction |
|---|---|---|
| Overtemperature | temperature measurement | limit output or stop |
| Overcurrent | current measurement | disable output |
| Undervoltage | supply measurement | limit or stop |
| Overvoltage | supply measurement | stop / protect bus |
| Stall | commanded movement with no valid speed | stop or enter fault state |
| Overspeed | RPM above allowed limit | reduce output or stop |
| Sensor fault | invalid or impossible reading | safe state |
| Communication timeout | command missing too long | safe stop |
| Watchdog timeout | firmware stops executing correctly | reset MCU |

I represent faults as flags so more than one condition can be active at the same time.

```c
#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    FAULT_NONE         = 0,
    FAULT_OVERCURRENT  = 1u << 0,
    FAULT_OVERTEMP     = 1u << 1,
    FAULT_UNDERVOLTAGE = 1u << 2,
    FAULT_OVERVOLTAGE  = 1u << 3,
    FAULT_STALL        = 1u << 4,
    FAULT_SENSOR       = 1u << 5,
    FAULT_OVERSPEED    = 1u << 6
} motor_fault_t;

static uint16_t fault_flags = 0;

void fault_set(uint16_t fault)
{
    fault_flags |= fault;
}

void fault_clear(uint16_t fault)
{
    fault_flags &= (uint16_t)(~fault);
}

bool fault_is_active(uint16_t fault)
{
    return (fault_flags & fault) != 0;
}

bool fault_any_active(void)
{
    return fault_flags != 0;
}
```

Normal motor control should not be allowed to overwrite a safety decision.

```c
void motor_control_update(void)
{
    if (fault_any_active())
    {
        motor_set_duty(0);
        motor_state = MOTOR_STATE_FAULT;
        return;
    }

    /* normal control path */
}
```

For a real fault I also want to know:

- what condition triggered it
- how long the condition was present
- whether it is recoverable
- whether power must be removed immediately
- what has to be true before the system can run again

---

## 9. Temperature and derating

Temperature does not only matter at the shutdown point. A controller can reduce the allowed output before the hardware reaches an unsafe temperature.

The control idea is:

```text
normal temperature     -> full allowed output
temperature increasing -> reduce allowed output
too hot                 -> motor off + fault
```

I treat the thermal limit as another output constraint. The final PWM request should never exceed the lowest active safety limit.

```text
requested PWM
     |
     v
+------------------+
| control output   |
+------------------+
     |
     v
+------------------+
| thermal limit    |
+------------------+
     |
     v
+------------------+
| electrical limit|
+------------------+
     |
     v
 final PWM
```

The actual temperature limits have to come from the components, the mechanical design and measured thermal behavior of the system.

---

## 10. State machine

I prefer explicit states for motor behavior because they make the firmware easier to trace and debug.

```c
typedef enum
{
    MOTOR_STATE_INIT,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_STARTING,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_DERATING,
    MOTOR_STATE_STOPPING,
    MOTOR_STATE_FAULT
} motor_state_t;
```

A typical flow is:

```text
INIT
  |
  v
IDLE ---- start ----> STARTING ---- valid speed ----> RUNNING
 ^                       |                               |
 |                       | fault                         | thermal limit
 |                       v                               v
 +---- stop/reset ----- FAULT <--------------------- DERATING
                                                        |
                                                        | safe again
                                                        v
                                                     RUNNING
```

The state decides whether:

- PWM is allowed
- PID is allowed to run
- the motor is still starting
- derating is active
- a fault can be reset
- the motor must stop

---

## 11. Timing, interrupts and control-loop execution

Motor-control code depends on predictable timing.

I separate fast and slow work instead of running everything at the same rate.

Examples of timing-sensitive work:

- PWM generation
- speed pulse measurement
- commutation
- control-loop update
- ADC sampling

Examples of slower work:

- temperature supervision
- diagnostics
- display updates
- non-critical communication

Interrupt handlers should stay short. I avoid putting display work, long calculations or blocking delays inside time-critical ISRs.

A control loop should know its sample time because the integral and derivative terms depend on it.

```text
same PID gains + different dt = different controller behavior
```

This is one reason deterministic scheduling matters in embedded control.

---

## 12. ATmega328P PWM example

For my AVR practice I use the ATmega328P at 16 MHz and configure Timer1 directly instead of using Arduino helper functions.

The example below generates PWM on `OC1A`, which is `PB1` on the ATmega328P and Arduino Uno digital pin 9.

```c
#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdint.h>

#define PWM_TOP 799u

static void pwm_init(void)
{
    /* PB1 / OC1A output */
    DDRB |= (1u << PB1);

    /*
     * Timer1 Fast PWM, TOP = ICR1
     * Non-inverting output on OC1A
     * Prescaler = 1
     */
    TCCR1A =
        (1u << COM1A1) |
        (1u << WGM11);

    TCCR1B =
        (1u << WGM13) |
        (1u << WGM12) |
        (1u << CS10);

    ICR1 = PWM_TOP;
    OCR1A = 0;
}

void motor_set_duty(uint8_t duty_percent)
{
    if (duty_percent > 100u)
    {
        duty_percent = 100u;
    }

    OCR1A =
        ((uint32_t)duty_percent * PWM_TOP) / 100u;
}

int main(void)
{
    pwm_init();

    motor_set_duty(50u);

    while (1)
    {
        /* sensor, fault and motor-control tasks */
    }
}
```

With a 16 MHz clock, no prescaler and `TOP = 799`:

```text
PWM frequency = 16,000,000 / (799 + 1)
              = 20,000 Hz
```

This example is useful because it makes the timer configuration visible. I can inspect the register settings, calculate the PWM frequency and verify the actual waveform with a logic analyzer or oscilloscope.

---

## 13. Testing and troubleshooting

My normal way of debugging embedded motor hardware is to measure each stage instead of guessing where the problem is.

A practical order is:

1. check the power supply
2. check ground connections
3. check the MCU output pin
4. verify PWM frequency and duty cycle
5. verify the switching stage
6. start with a low motor command
7. observe current and motor response
8. verify sensor feedback
9. increase load gradually
10. watch component temperature
11. test fault behavior where it is safe to do so
12. compare target values with measured values

Tools I use or expect to use for this work include:

- multimeter
- oscilloscope
- logic analyzer
- bench power supply
- current measurement
- tachometer or speed feedback
- temperature measurement
- serial logging
- debugger

The measurements I care about most are:

```text
voltage
current
PWM frequency
PWM duty cycle
RPM
temperature
control-loop timing
```

A useful controller test is not only "does the motor spin?". I also want to know how it behaves when the load changes, when the target changes and when a sensor or safety condition becomes invalid.

---

## 14. Performance measurements

For closed-loop speed control I look at more than the final RPM.

### Rise time

How long it takes to reach the target after a command change.

### Overshoot

How far the measured speed goes above the target.

```text
overshoot % =
(max_speed - target_speed) / target_speed * 100
```

### Steady-state error

```text
steady_state_error = target_rpm - measured_rpm
```

### Settling time

How long it takes before the speed remains inside an acceptable range around the target.

### Load response

How quickly the controller recovers when the mechanical load changes.

### Thermal behavior

I also consider temperature rise, current and power loss because a controller that reaches the requested speed but overheats is not a good controller.

Input electrical power can be measured as:

```text
input_power = supply_voltage * supply_current
```

---

## 15. Notes on three-phase BLDC control

The ATmega PWM example above is useful for learning timer control and motor power control, but a raw three-phase BLDC motor requires commutation of the three motor phases.

### Six-step commutation

In six-step control, two phases are energized while the third phase is left floating. The active phase pair changes as the rotor moves through the electrical sectors.

### Hall-sensor commutation

Hall sensors identify the rotor sector. The controller uses the Hall pattern to select the correct switching state and can also use the transitions to calculate speed.

### Sensorless control

Sensorless control estimates rotor position from the motor electrical behavior. A common method is back-EMF zero-crossing detection.

Start-up is more difficult because back-EMF is weak or unavailable when the rotor is stopped.

### Current control

Motor current is closely related to torque. A more advanced controller can use an inner current loop and an outer speed loop.

```text
 target speed
     |
     v
 speed controller
     |
     v
 target current
     |
     v
 current controller
     |
     v
 PWM / inverter
     |
     v
 motor
```

### Field-Oriented Control

FOC controls the motor using a rotating reference frame instead of directly controlling the phase currents as three separate sinusoidal signals.

Concepts I have studied around FOC include:

- Clarke transform
- Park transform
- d-axis current
- q-axis current
- inverse Park transform
- Space Vector PWM
- rotor electrical angle
- current sensing

The reason FOC is important is that it can provide smooth torque, efficient operation and accurate torque control.

---

## 16. Communication and supervision

Larger embedded products often need the motor controller to exchange commands and diagnostics with other controllers.

CAN is commonly used for this type of communication.

A motor controller may transmit values such as:

```text
motor RPM
motor current
bus voltage
motor temperature
power-stage temperature
fault status
controller state
```

and receive commands such as:

```text
start / stop
target speed
target torque
direction
operating mode
```

The communication layer should not be able to bypass the local protection logic. A command is still subject to state, fault and output limits inside the motor controller.

A watchdog is another independent supervision mechanism. If the firmware stops running correctly and no longer services the watchdog, the MCU can be reset into a known start-up state.

---

## 17. Engineering notes I keep in mind

### Software and hardware have to be debugged together

A control algorithm cannot fix incorrect wiring, bad grounding, an unsuitable switching device or a missing sensor signal.

### Measure first

I prefer checking real signals before changing code. A scope trace, voltage reading, RPM measurement or current reading usually gives more direction than guessing.

### Command and feedback are different

A PWM percentage is a command. RPM is a measured result.

### Fault logic belongs in the main design

Protection should have control over the output and should not depend on the normal control loop behaving correctly.

### Timing changes control behavior

PID, speed measurement and commutation depend on known timing. The software structure has to respect that.

### Sensors can fail

A valid-looking variable in software does not always mean the physical measurement is valid. Range checks, timing checks and missing-signal detection are important.

### Temperature changes the system

Electrical resistance, losses and available output can change as the motor and power stage heat up.

### Start-up is its own operating condition

A stopped motor does not behave like a running motor, especially when rotor position is estimated from electrical signals.

### Good embedded debugging crosses domains

When a motor-control problem appears, I look at the code, electrical signals and mechanical behavior together.

---

## Safety

Motor-control work can involve high current, hot components and rotating parts.

During development I keep the following rules in mind:

- use a current-limited power supply when possible
- verify polarity before applying power
- verify the required common ground connections
- keep hands and loose objects away from rotating parts
- use components with suitable voltage and current ratings
- monitor component temperature
- use protection appropriate for the power level
- never power a motor directly from a microcontroller GPIO pin

---

## Author

**Meshel Alsadou**

Embedded C/C++, motor control, sensors, PID regulation, testing and hardware/software integration.
