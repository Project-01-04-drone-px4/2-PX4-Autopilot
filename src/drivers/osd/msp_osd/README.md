# MSP DisplayPort OSD layout

Flash firmware containing these parameters once. Afterwards, edit the OSD
parameters in QGroundControl to move elements without rebuilding or rebooting.
The driver checks parameter updates once per second and redraws the screen at
10 Hz, clearing the previous positions on each frame.

Coordinates refer to the first character (including the icon). X is the column
and increases to the right; Y is the row and increases downward. The top-left
corner is (0, 0). X accepts 0–59 and Y accepts 0–21. Use coordinates that fit your
actual display resolution and leave room for the full text: the driver clamps
the anchor to these limits but does not detect the canvas size or wrap text.

| Element | Column parameter | Row parameter | Default (X, Y) |
| --- | --- | --- | --- |
| Status / flight mode / warning message | OSD_MSG_X | OSD_MSG_Y | (20, 2) |
| RC link quality | OSD_RSSI_X | OSD_RSSI_Y | (2, 2) |
| Average cell voltage | OSD_CELL_X | OSD_CELL_Y | (2, 4) |
| Battery current | OSD_CURR_X | OSD_CURR_Y | (2, 5) |
| Consumed capacity | OSD_MAH_X | OSD_MAH_Y | (2, 6) |
| GPS latitude | OSD_LAT_X | OSD_LAT_Y | (41, 10) |
| GPS longitude | OSD_LON_X | OSD_LON_Y | (41, 9) |
| GPS satellite count | OSD_SAT_X | OSD_SAT_Y | (41, 8) |
| GPS ground speed | OSD_SPD_X | OSD_SPD_Y | (2, 8) |
| Distance to home | OSD_HOME_X | OSD_HOME_Y | (2, 10) |
| Pitch angle | OSD_PITCH_X | OSD_PITCH_Y | (41, 13) |
| Roll angle | OSD_ROLL_X | OSD_ROLL_Y | (41, 14) |
| Altitude | OSD_ALT_X | OSD_ALT_Y | (2, 9) |
| Crosshairs | OSD_CH_X | OSD_CH_Y | (26, 10) |

For example, move the altitude to column 40, row 16 in the MAVLink console:

```sh
param set OSD_ALT_X 40
param set OSD_ALT_Y 16
```

Existing OSD_SYMBOLS visibility settings remain in effect. Selecting a position
does not enable a disabled element. Status and warning text shares one message
location; it retains the existing behavior of displaying whenever OSD_SYMBOLS
is nonzero. Reserved/unused OSD_SYMBOLS bits do not gain new rendering support.

Crosshairs retain the existing offsets for compatibility:

- Final X = OSD_CH_X + OSD_CH_POS_HOR.
- Final Y = OSD_CH_Y - OSD_CH_POS_VER.

Set both offsets to zero to use the new coordinates directly. The default
coordinates preserve the previous layout, including any saved crosshair offsets.
The obsolete, uncalled MSP_OSD_CONFIG layout has been removed; these parameters
control the DisplayPort rendering actually sent by the driver.

## Hardware verification

With the vehicle disarmed, confirm each enabled element starts at its previous
position. Change each X/Y pair and check that the element moves within about
one second without leaving text at the old location. Verify the altitude text
is complete, check crosshair offsets and screen edges, then reboot and confirm
the saved positions are restored. Match the coordinate range to the goggles'
actual canvas size.
