Overview
********
This sample will draw a small plus in the last touched coordinates, that way you can check
if the touch screen works for a board, examine its parameters such as inverted/swapped axes.

Building and Running
********************
```shell
cd <driver_directory>
west build -p always -b <BOARD>
west flash
```
Calibration
***********
- Enter Calibration Mode: Press the SW0 button.
- Touch Calibration Points: Follow the on-screen prompts to touch each of the five calibration points.
- Calibration Completion: Once all points are collected, calibration coefficients are computed.

Samples Output
**************

```shell
*** Booting Zephyr OS ***
Calibration mode activated.
Calibration point 1: X=[20], Y=[20]
Calibration point 2: X=[108], Y=[20]
Calibration point 3: X=[108], Y=[140]
Calibration point 4: X=[20], Y=[140]
Calibration point 5: X=[64], Y=[80]
Calibration complete.
Touch X, Y: (50, 75)
Touch X, Y: (100, 125)
```
