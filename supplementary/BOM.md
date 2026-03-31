# Bill of Materials

> The following is an example of the hardware components necessary to build a quadcopter supporting **ALL** of `aerial-autonomy-stack`'s capabilities, including perception and multi-robot communication/swarming 

## Example AAS Quadcopter

| #   | Part                                  | Description                                            | Cost (USD) | Link         |
| --: | ------------------------------------- | ------------------------------------------------------ | ---------: | ------------ |
| 1   | Holybro X650 Almost-ready-to-fly Kit  | Quadcopter frame, motors, ESCs, propellers             | 699        | [URL][kit]
| 2   | Holybro H-RTK ZED-F9P Ultralight      | GNSS module (GPS, GLONASS, Galileo, BeiDou)            | 279        | [URL][gps]
| 3   | Holybro Fixed Carbon Fiber GPS mount  | GNSS module support                                    | 12         | [URL][mount]
| 4   | Holybro Microhard Telemetry Radio*    | Point-to-multipoint telemetry (1 ground + 1 per drone) | 449        | [URL][telem]
| 5   | RadioMaster Boxer RC CC2500           | Radio controller                                       | 100        | [URL][rc]
| 6   | RadioMaster R81 V2 Receiver           | Receiver for the radio controller                      | 18         | [URL][rec]
| 7   | Matek Power Module PM12S-4A           | 5V and 12V supply for Doodle and Jetson                | 20         | [URL][matek]
| 8   | Tattu G-Tech 6S 8000mAh 25C 22.2V     | Lipo battery pack with XT60                            | 162        | [URL][batt]
| 9   | Pixhawk Jetson Baseboard Bundle       | NVIDIA Orin NX 16GB + SSD + Pixhawk 6X + CSI ArduCam   | 1450       | [URL][jetson]
| 10  | ASIX AX88772A USB2.0 Ethernet Adapter | 1 for `AIR_SUBNET`, 1 for `SIM_SUBNET` (for HITL only) | 18         | [URL][eth]
| 11  | Doodle Labs RM-2450-11N3              | 2.4GHz Nano radio module (1 ground + 1 per drone)      | 1700       | [URL][doot1]
| 12  | Doodle Labs EK-2450-11N3              | Nano carrier/evaluation kit (1 ground + 1 per drone)   | 290        | [URL][doot2]
| 13  | DJI Livox Mid-360**                   | LiDAR sensor                                           | 749        | [URL][liv]
| 14  | Livox three-wire aviation connector** | Power and ethernet connector for the LiDAR             | 89         | [URL][liv2]

> *For a single drone, one can alternatively use the point-to-point [SiK telemetry radio][telem2] (89)
>
> **LiDAR sensor, optional for a camera-only solution

```mermaid
%%{init: {'theme': 'base', 'themeVariables': { 'fontFamily': 'monospace'}}}%%
flowchart TB
    Batt[6S Battery]
    PDB[Holybro X650 Power Distribution Board]
    Matek[Matek Power Module]

    subgraph Jetson_Baseboard ["Jetson Baseboard"]
        direction TB
        6X[6X Autopilot]
        Orin[NVIDIA Jetson Orin]
        Orin <-->|"TELEM2 / ETH"| 6X
    end
    

    subgraph Doodle_Lab_Radio ["Doodle Lab Radio"]
        direction TB
        EvalKit[Evaluation Kit]
        RadioMod[Radio Module]
        EvalKit <-->|" "| RadioMod
    end
    
    Telem[Telemetry Radio]

    Doodle_Lab_Radio ~~~ Telem

    GPS[GPS Module]
    Lidar[Livox 360]
    Camera[CSI ArduCam]

    RC_Rx[R81 Receiver]
    RC[RadioMaster Boxer RC]
    RC_Rx <-->|"bind"| RC

    Batt -- "24V" --> PDB
    PDB -- "24V" --> Matek
    PDB -- "24V" --> Telem
    Matek -- "12V" --> Jetson_Baseboard

    Jetson_Baseboard <-->|"TELEM1"| Telem
    GPS -- "GPS1" --> Jetson_Baseboard
    Jetson_Baseboard <-->|"RC IN"| RC_Rx

    Jetson_Baseboard ~~~ Lidar
    Jetson_Baseboard ~~~ Camera
    Lidar ~~~ GPS
    
    Camera -- "CSI" --> Jetson_Baseboard
    Lidar -- "ETH" --> Jetson_Baseboard
    Matek -- "5V" --> Doodle_Lab_Radio
    Jetson_Baseboard <-->|"USB 2.0 ETH Adapter"| Doodle_Lab_Radio

    Matek -- "12V" --> Lidar

    %%%%%%%%%%
    linkStyle 3 stroke:teal,stroke-width:3px,stroke-dasharray: 8 4;
    linkStyle 4,5,6,7,16,18 stroke:red,stroke-width:4px;    
    linkStyle 8,9,10,14,15,17 stroke:blue,stroke-width:3px;
```

[kit]:https://holybro.com/collections/x650-kits/products/x650-kits?variant=43994378240189
[gps]:https://holybro.com/collections/standard-h-rtk-series/products/h-rtk-f9p-ultralight?variant=45785783009469
[mount]:https://holybro.com/collections/gps-accessories/products/fixed-carbon-fiber-gps-mount?variant=42749655449789
[telem]:https://holybro.com/collections/telemetry-radios/products/microhard-radio?variant=42522025590973
[telem2]:https://holybro.com/collections/telemetry-radios/products/sik-telemetry-radio-1w?variant=45094904856765
[rc]:https://radiomasterrc.com/collections/boxer-radio/products/boxer-radio-controller-m2?variant=46486352298176
[rec]:https://holybro.com/collections/rc-radio-transmitter-receiver/products/radiomaster-r81-receiver
[matek]:https://www.mateksys.com/?portfolio=pm12s-4a
[batt]:https://genstattu.com/tattu-8000mah-22-2v-25c-6s1p-lipo-battery-pack-with-xt60-plug.html
[jetson]:https://holybro.com/collections/flight-controllers/products/pixhawk-jetson-baseboard?variant=44636223439037
[eth]:https://www.amazon.ca/TRENDnet-TU2-ET100-USB-Mbps-Adapter/dp/B00007IFED/
[liv]:https://store.dji.com/ca/product/livox-mid-360?vid=130851
[liv2]:https://store.dji.com/ca/product/livox-three-wire-aviation-connector?vid=117441
[doot1]:https://www.mouser.ca/ProductDetail/Doodle-Labs/RM-2450-11N3?qs=ulEaXIWI0c91eCn7VRB%2FpA%3D%3D
[doot2]:https://www.mouser.ca/ProductDetail/Doodle-Labs/EK-2450-11N3?qs=ulEaXIWI0c%2FLOqPeL4gNgg%3D%3D

## Holybro X650 Parameters

Autopilot parameters specific to the Holybro X650 kit

### PX4 Configuration

```sh
TODO
```

### ArduPilot Configuration

```sh
# GPS module
GPS1_TYPE           1               # Auto, GPS1_TYPE or GPS2_TYPE depending on the JST connector used on the Jetson Baseboard

# DShot ESCs (Tekko32 F4 45A)
SERVOx_FUNCTION     0               # Disabled, for SERVO1 to 4, these are channels 1 to 4 on IO PWM
MOT_PWM_TYPE        6               # DShot600, for the Tekko32 F4 45A ESCs, using the first 4 channels on FMU PWM, i.e. SERVO9 to 12
SERVO9_FUNCTION     33              # Motor 1, channel 1 on FMU PWM
SERVO10_FUNCTION    34              # Motor 2, channel 2 on FMU PWM
SERVO11_FUNCTION    35              # Motor 3, channel 3 on FMU PWM
SERVO12_FUNCTION    36              # Motor 4, channel 4 on FMU PWM
SERVOx_MIN          1000            # For SERVO9 to 12
SERVOx_MAX          2000            # For SERVO9 to 12
SERVOx_TRIM         1000            # For SERVO9 to 12
# If needed to configure the ESCs, remove the propellers and set BRD_SAFETY_DEFLT to 0 (Disabled) or make sure BRD_SAFETY_MASK ignores channels 9 to 12
# In QGC -> Vehicle Configuration -> Motors, use the sliders to verify motor numbering and spin direction; reverse with SERVO_BLH_RVMASK, if necessary

# Motor thrust curve exponent (T-Motor MN4014 KV330s with Gemfan 1555 propellers)
MOT_THST_EXPO       0.71            # For 15in props, 0.0 is linear, 1.0 is second order curve
# (optional) lower MOT_SPIN_ARM and MOT_SPIN_MIN from the 0.1 and 0.15 defaults to 0.05, 0.1

# Limit RPY acceleration (in centidegrees per square second)
ATC_ACCEL_P_MAX     52000           # Between slow and very slow
ATC_ACCEL_R_MAX     52000           # Between slow and very slow
ATC_ACCEL_Y_MAX     18000           # Slow

# 6S battery (Tattu G-Tech 6S 8000mAh 25C 22.2V)
MOT_BAT_VOLT_MAX    25.2            # 6 cells x 4.2V
MOT_BAT_VOLT_MIN    19.8            # 6 cells x 3.3V
BATT_CAPACITY       8000            # 8000mAh
BATT_MONITOR        21              # INA2XX
# Check MOT_BAT_CURR_TC is set to the default of 5.0

# Gyro filter
ATC_RAT_PIT_FLTD    10              # Pitch axis rate controller derivative frequency in Hz, default is 20
ATC_RAT_RLL_FLTD    10              # Roll axis rate controller derivative frequency in Hz, default is 20
# Check INS_GYRO_FILTER, the gyro filter cutoff frequency is 20

# (optional) filter vibration with BDShot telemetry (requires target Pixhawk6X-bdshot)
SERVO_BLH_BDMASK    3840            # Bitmask to activate bidirectional DShot ESC telemetry on channels 9 to 12
SERVO_BLH_POLES     24              # Default is 14, T-Motor MN4014 KV330s are 18N24P
INS_HNTCH_ENABLE    1               # Enable harmonic notch filter (reboot to set the other parameters)
INS_HNTCH_MODE      3               # Use ESC telemetry
INS_HNTCH_REF       1.0             # Setting for ESC telemetry
INS_HNTCH_FREQ      40              # Base frequency lower than the default 80 for the X650
INS_HNTCH_BW        20              # Half of INS_HNTCH_FREQ
INS_HNTCH_OPTS      2               # Bitmask, multi-source is the second bit

# In QGC -> Vehicle Configuration -> Radio, calibrate the Radiomaster Boxer RC
# In QGC -> Vehicle Configuration -> Flight Modes, set one switch for Stabilized/AltHold/Loiter, one for RTL
# In QGC -> Vehicle Configuration -> Safety, set battery and general failsafes, RTL settings
# In QGC -> Vehicle Configuration -> Sensors, calibrate accelerometer, horizon level, and compass (outdoors)
```
