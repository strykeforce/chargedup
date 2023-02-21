
# 2023 FIRST CHARGED UP

[![CI](https://github.com/strykeforce/chargedup/actions/workflows/main.yml/badge.svg)](https://github.com/strykeforce/chargedup/actions/workflows/main.yml)

## Controls

![driver](docs/driver-controls.png)

![operator](docs/operator-controls.png)

## CAN Bus

| Subsystem  | Type     | Talon                 | ID | Comp PDP | Proto PDP | Motor  | Breaker |
| ---------- | -------- | --------------------- | -- | -------- | --------- | ------ | ------- |
| Drive      | SRX      | azimuth               | 0  | 9        |  8        | 9015   |         |
| Drive      | SRX      | azimuth               | 1  | 11       |  12       | 9015   |         |
| Drive      | SRX      | azimuth               | 2  | 8        |  9        | 9015   |         |
| Drive      | SRX      | azimuth               | 3  | 10       |  11       | 9015   |         |
| Drive      | FX       | drive                 | 10 | 0        |  6        | falcon |         |
| Drive      | FX       | drive                 | 11 | 19       |  15       | falcon |         |
| Drive      | FX       | drive                 | 12 | 1        |  7        | falcon |         |
| Drive      | FX       | drive                 | 13 | 18       |  14       | falcon |         |
| Intake     | FX       | intake                | 20 | 12       |  16       | falcon |         |
| Intake     | SRX      | extend                | 21 |          |  13       | bag    |         |
| Shoulder   | SRX      | left main             | 30 | 3        |  2        | 550    |         |
| Shoulder   | SRX      | right follow          | 34 | 16       | 19        | 550    |         |
| Elevator   | FX       | left main             | 31 | 2        | 5         | falcon |         |
| Elevator   | FX       | right follow          | 32 | 17       | 17        | falcon |         |
| Elbow      | FX       | elbow                 | 33 |          |  4        | falcon |         |
| Elbow      | Canifier | canifier              | 15 |          |  23       | -      |         |
| Hand       | SRX      | hand                  | 40 |          |  3        | 550    |         |

* elbow uses remote encoder attached to canifier
* forward limit switch on extend talon = beam break for intake
* analog input on hand talon = proximity sensor on hand


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


## PWM
| Subsystem | name  | ID |
| --------- | ----- | -- | 
| RGB       | red   | 0  |
| RGB       | green | 1  |
| RGB       | blue  | 2  | 
