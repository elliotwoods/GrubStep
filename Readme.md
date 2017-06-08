# GrubStep

An interrupt-driven stepper motor driver for Arduino.
(I stole that line from [here](https://github.com/bjpirt/HotStepper)).

The particular advantage of this library is that it handles accelerations (i.e. doesn't just move at constant velocity).
I find this is helps to avoid skipping steps when starting and stopping movements.

# License

[MIT License](https://opensource.org/licenses/MIT)
