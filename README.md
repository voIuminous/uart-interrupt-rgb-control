# Interrupt-Driven UART RGB Control System

This repository contains a bare-metal ARM Thumb assembly program developed for the NXP KL05Z32 (Cortex-M0+) microcontroller.

The project implements an interrupt-driven UART system with transmit and receive queues, a periodic timer for time tracking, and direct GPIO control of an RGB LED. UART input is processed asynchronously using interrupts, while timing is handled using the Periodic Interrupt Timer (PIT).

The application logic is structured as a simple finite state flow that reacts to UART input and timing events.

## Hardware
- NXP KL05Z32 (ARM Cortex-M0+)
- On-board RGB LED
- UART connection via USB-to-serial interface

## Notes
This project was developed as part of CMPE-250 (Assembly & Embedded Programming) and focuses on low-level interrupt handling, timing, and peripheral control.
