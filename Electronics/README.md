# b1 Mark I - Self Balancing Robot

## Electronics

### Microcontroller
Technical specifications:
- Model: [Arduino Uno] R3

#### Diagram
                                 .-----.
    .----[PWR]-------------------| USB |--.
    |                            '-----'  |
    |         GND/RST2  [ ][ ]            |
    |       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] |   C5
    |          5V/MISO2 [ ][ ]  A4/SDA[ ] |   C4
    |                             AREF[ ] |
    |                              GND[ ] |
    | [ ]NC                     SCK/13[ ] |   B5
    | [ ]v.ref                 MISO/12[ ] |   .
    | [ ]RST                   MOSI/11[ ]~|   .
    | [ ]3V3    +---+               10[ ]~|   .
    | [ ]5v     | A |                9[ ]~|   .
    | [ ]GND   -| R |-               8[ ] |   B0
    | [ ]GND   -| D |-                    |
    | [ ]Vin   -| U |-               7[ ] |   D7
    |          -| I |-               6[ ]~|   .
    | [ ]A0    -| N |-               5[ ]~|   .
    | [ ]A1    -| O |-               4[ ] |   .
    | [ ]A2     +---+           INT1/3[ ]~|   .
    | [ ]A3                     INT0/2[ ] |   .
    | [ ]A4/SDA  RST SCK MISO     TX>1[ ] |   .
    | [ ]A5/SCL  [ ] [ ] [ ]      RX<0[ ] |   D0
    |            [ ] [ ] [ ]              |
    '--.                         .--------'
        \_______________________/

Diagram by http://busyducks.com/ascii-art-arduinos

#### Pin designation
 id | pin |   type    | direction |               description
----|-----|-----------|-----------|--------------------------------------------
  0 |   0 |           | input     | Serial communication
  1 |   1 |           | output    | Serial communication
  2 |   0 |           |           |
  3 |   3 |           |           |
  4 |   4 |           |           |
  5 |   5 |           |           |
  6 |   6 |           |           |
  7 |   7 |           |           |
  8 |   8 |           |           |
  9 |   9 |           |           |
 10 |  10 |           |           |
 11 |  11 |           |           |
 12 |  12 |           |           |
 13 |  13 |           |           |
 14 |   0 |           |           |
 15 |   1 |           |           |
 16 |   2 |           |           |
 17 |   3 |           |           |
 18 |   4 |           |           |
 19 |   5 |           |           |

### Digital Motion Processor
![MPU-6050](Pictures/mpu6050.jpg)

[MPU-6050](MPU-6050_DataSheet_V3_4.pdf) datasheet

Technical specifications:
- Model: MPU-6050
- Gyroscope: Yes
- Accelerometer: Yes
- Digital Motion Processor: Yes
- Output Type: I²C, SPI
- Operating Temperature: -40°C ~ 85°C (TA)

[Main page]

---
[Arduino Uno]: https://www.arduino.cc/en/Main/arduinoBoardUno/#techspecs
[Main page]: ../README.md
