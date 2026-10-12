# Patient Assistant Robot

An interactive, tabletop assistant robot designed to support automated medication dispensing, vitals telemonitoring, and remote care coordination for elderly and chronically ill patients.

---

## Features

- **Automated Pill Dispensing**: Vertically stacked, modular rotary carousels deliver scheduled doses down a common chute into a collection drawer. Includes an actuated trapdoor to prevent forgotten doses from accumulating.
- **Vitals Monitoring**:
  - **Temperature**: Non-contact infrared thermal sensor detects emerging fevers against baseline data.
  - **Pulse Rate**: Optical heart-rate sensor with onboard digital filtering flags cardiovascular irregularities.
- **Teleconsultation & Voice**: Wide-angle camera, microphone, and speaker enable direct caregiver/doctor video and voice check-ins.
- **Companion Mobile App**: Remote scheduling, real-time alerts for missed doses or abnormal vitals, and longitudinal health dashboards.

---

## System Architecture

| Subsystem | Key Components |
| --- | --- |
| **Dispensing** | 4× ESP32 microcontrollers, stepper motors & drivers |
| **Vision & Comms** | Wide-angle camera module, microphone & speaker |
| **Vitals Acquisition** | Infrared temperature sensor, optical pulse sensor |
| **Chassis & Mechanics** | Modular interlocking 3D-printed carousels, trapdoor delivery tray |

---

## Team Systole (Team 06)

- E/22/008 – Thisum Abeywickrama
- E/22/010 – Dineth Adikari
- E/22/102 – Achintha Gamage
- E/22/130 – Sanindu Hansara