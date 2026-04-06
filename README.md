# CC2511 — CNC Milling Machine Project 2026 · TSV07

> **Raspberry Pi Pico firmware for the Vevor 3018 CNC Mill**  
> Bridging terminal control with precision stepper-driven hardware.

---

## Overview

This CNC mill control system is a **Raspberry Pi Pico-based embedded firmware** that bridges a terminal user interface with the physical hardware of a Vevor 3018 CNC mill.

The system operates in two modes:

| Mode | Description |
|------|-------------|
| **Manual Mode** | Direct keyboard jogging of each axis in real time |
| **Command Mode** | Execute G-code instructions for automated toolpaths |

---

## Hardware

### Axes — DRV8825 Stepper Drivers

Three **DRV8825** stepper motor drivers independently control each axis:

| Axis | Direction |
|------|-----------|
| **X** | Left / Right |
| **Y** | Forward / Back |
| **Z** | Up / Down |

### Spindle

The spindle motor speed is controlled via a **PWM-driven signal**, allowing variable RPM through software.

---

## Platform

| Component | Details |
|-----------|---------|
| **Microcontroller** | Raspberry Pi Pico |
| **Mill Hardware** | Vevor 3018 CNC |
| **Motor Drivers** | DRV8825 × 3 |
| **Interface** | Terminal (UART / USB Serial) |

---

## Project Info

| Field | Value |
|-------|-------|
| **Course** | CC2511 |
| **Year** | 2026 |
| **Group** | TSV07 |
