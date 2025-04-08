# DFRobot_TFmini Library for ESP32

Provides an Arduino library for TFmini Distance Sensor
By babynghe2003

## Table of Contents

- [Methods](#methods)
- [History](#history)
- [Credits](#credits)
  <snippet>
  <content>

## Methods

```C++

/*
 * @breif Set Software Serial
 */
void      begin();

/*
 * @breif Measuring distance
 *
 * @return
 *     ture   Success
 *     false  Failed
 */
bool      measure2(void);

/*
 * @breif Get distance data
 *
 * @return  The value of distance
 */
uint16_t  getDistance(void);

/*
 * @breif Get signal strength data
 *
 * @return  The value of signal strength
 */
uint16_t  getStrength(void);

/*
 * @breif Get Temprature data
 *
 * @return the value of Temprature
 */
uint16_t getTemprature(void);

```

## History

- data 2023-06-14
- version V1.0

## Credits

- author [BabyNghe2003<himinhpho@gmail.com>]

