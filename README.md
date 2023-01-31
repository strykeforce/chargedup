
# 2023 FIRST CHARGED UP

[![CI](https://github.com/strykeforce/chargedup/actions/workflows/main.yml/badge.svg)](https://github.com/strykeforce/chargedup/actions/workflows/main.yml)

## Controls

![driver](docs/driver-controls.png)

![operator](docs/operator-controls.png)

## CAN Bus

| Subsystem  | Type     | Talon                 | ID | PDP | Motor  | Breaker |
| ---------- | -------- | --------------------- | -- | --- | ------ | ------- |
| Drive      | SRX      | azimuth               | 0  |  8  | 9015   |         |
| Drive      | SRX      | azimuth               | 1  |  12 | 9015   |         |
| Drive      | SRX      | azimuth               | 2  |  9  | 9015   |         |
| Drive      | SRX      | azimuth               | 3  |  11 | 9015   |         |
| Drive      | FX       | drive                 | 10 |  6  | falcon |         |
| Drive      | FX       | drive                 | 11 | 15  | falcon |         |
| Drive      | FX       | drive                 | 12 |  7  | falcon |         |
| Drive      | FX       | drive                 | 13 | 14  | falcon |         |
| Intake     | FX       | intake                | 20 | 16  | falcon |         |
| Intake     | SRX      | extend                | 21 | 13  | bag x2 |         |
| Shoulder   | SRX      | shoulder              | 30 | 10  | ??     |         |
| Elevator   | FX       | elevator              | 31 | 5   | falcon |         |
| Elbow      | FX       | elbow                 | 33 |  4  | falcon |         |
| Elbow      | Canifier | canifier              | 15 |  23 | -      |         |
| Hand       | SRX      | hand                  | 40 |     | ??     |         |

* elbow uses remote encoder attached to canifier
* forward limit switch on extend talon = beam break for intake


## Roborio
| Subsystem | Interface | Device | 
| --------- | --------- | ------ |
| Drive     | SPI/MXP   | NAVX   |


## DIO
| Subsystem | name       | ID |
| --------- | ---------- | -- |
| Auto      | autoSwitch | 0  |
| Auto      | autoSwitch | 1  |
| Auto      | autoSwitch | 2  |
| Auto      | autoSwitch | 3  |
| Auto      | autoSwitch | 4  |
| Auto      | autoSwitch | 5  |
| Robot     | BNC        | 6  |
