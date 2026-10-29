# MPU-6000 and MPU-6050 Register Map and Descriptions

**Document Number:** RM-MPU-6000A-00
**Revision:** 4.0
**Release Date:** 03/09/2012

**InvenSense Inc.**
1197 Borregas Ave, Sunnyvale, CA 94089 U.S.A.
Tel: +1 (408) 988-7339 Fax: +1 (408) 988-8104
Website: www.invensense.com

CONFIDENTIAL & PROPRIETARY

---

## 1. Revision History

| Revision | Date | Revision Description |
|----------|------|----------------------|
| 1.0 | 11/29/2010 | Initial Release |
| 1.1 | 04/20/2011 | Updated register map and descriptions to reflect enhanced register functionality. |
| 2.0 | 05/19/2011 | Updates for Rev C silicon: Edits for readability (section 2.1); Edits for changes in functionality (section 3, 4.4, 4.6, 4.7, 4.8, 4.21, 4.22, 4.23, 4.37) |
| 3.0 | 10/07/2011 | Updates for Rev D silicon: Updated accelerometer sensitivity specifications (sections 4.6, 4.8, 4.10, 4.23) |
| 3.1 | 10/24/2011 | Edits for clarity |
| 3.2 | 11/14/2011 | Updated reset value for register 107 (section 3); Updated register 27 with gyro self-test bits (section 4.4); Provided gyro self-test instructions and register bits (section 4.4); Provided accel self-test instructions (section 4.5) |
| 4.0 | 3/9/2012 | Updated register map to include Self-Test registers (section 3); Added description of Self-Test registers (section 4.1); Revised temperature register section (section 4.19); Corrections in registers 107 and 108 (section 4.30) |

---

## 2. Purpose and Scope

This document provides preliminary information regarding the register map and descriptions for the Motion Processing Units™ MPU-6000™ and MPU-6050™, collectively called the MPU-60X0™ or MPU™.

The MPU devices provide the world's first integrated 6-axis motion processor solution that eliminates the package-level gyroscope and accelerometer cross-axis misalignment associated with discrete solutions. The devices combine a 3-axis gyroscope and a 3-axis accelerometer on the same silicon die together with an onboard Digital Motion Processor™ (DMP™) capable of processing complex 9-axis sensor fusion algorithms using the field-proven and proprietary MotionFusion™ engine.

The MPU-6000 and MPU-6050's integrated 9-axis MotionFusion algorithms access external magnetometers or other sensors through an auxiliary master I2C bus, allowing the devices to gather a full set of sensor data without intervention from the system processor. The devices are offered in the same 4x4x0.9 mm QFN footprint and pinout as the current MPU-3000™ family of integrated 3-axis gyroscopes, providing a simple upgrade path and facilitating placement on already space constrained circuit boards.

For precision tracking of both fast and slow motions, the MPU-60X0 features a user-programmable gyroscope full-scale range of ±250, ±500, ±1000, and ±2000°/sec (dps). The parts also have a user-programmable accelerometer full-scale range of ±2g, ±4g, ±8g, and ±16g.

The MPU-6000 family is comprised of two parts, the MPU-6000 and MPU-6050. These parts are identical to each other with two exceptions. The MPU-6050 supports I2C communications at up to 400kHz and has a VLOGIC pin that defines its interface voltage levels; the MPU-6000 supports SPI at up to 20MHz in addition to I2C, and has a single supply pin, VDD, which is both the device's logic reference supply and the analog supply for the part.

For more detailed information for the MPU-60X0 devices, please refer to the "MPU-6000 and MPU-6050 Product Specification".

---

## 3. Register Map

The register map for the MPU-60X0 is listed below.

| Addr (Hex) | Addr (Dec.) | Register Name | Serial I/F | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|------------|-------------|---------------|------------|------|------|------|------|------|------|------|------|
| 0D | 13 | SELF_TEST_X | R/W | XA_TEST[4-2] | | | | | XG_TEST[4-0] | | |
| 0E | 14 | SELF_TEST_Y | R/W | YA_TEST[4-2] | | | | | YG_TEST[4-0] | | |
| 0F | 15 | SELF_TEST_Z | R/W | ZA_TEST[4-2] | | | | | ZG_TEST[4-0] | | |
| 10 | 16 | SELF_TEST_A | R/W | RESERVED | | XA_TEST[1-0] | | YA_TEST[1-0] | | ZA_TEST[1-0] | |
| 19 | 25 | SMPLRT_DIV | R/W | SMPLRT_DIV[7:0] | | | | | | | |
| 1A | 26 | CONFIG | R/W | - | - | EXT_SYNC_SET[2:0] | | | DLPF_CFG[2:0] | | |
| 1B | 27 | GYRO_CONFIG | R/W | - | - | - | FS_SEL [1:0] | - | - | - | |
| 1C | 28 | ACCEL_CONFIG | R/W | XA_ST | YA_ST | ZA_ST | AFS_SEL[1:0] | | | | |
| 1F | 31 | MOT_THR | R/W | MOT_THR[7:0] | | | | | | | |
| 23 | 35 | FIFO_EN | R/W | TEMP_FIFO_EN | XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN | ACCEL_FIFO_EN | SLV2_FIFO_EN | SLV1_FIFO_EN | SLV0_FIFO_EN |
| 24 | 36 | I2C_MST_CTRL | R/W | MULT_MST_EN | WAIT_FOR_ES | SLV_3_FIFO_EN | I2C_MST_P_NSR | I2C_MST_CLK[3:0] | | | |
| 25 | 37 | I2C_SLV0_ADDR | R/W | I2C_SLV0_RW | I2C_SLV0_ADDR[6:0] | | | | | | |
| 26 | 38 | I2C_SLV0_REG | R/W | I2C_SLV0_REG[7:0] | | | | | | | |
| 27 | 39 | I2C_SLV0_CTRL | R/W | I2C_SLV0_EN | I2C_SLV0_BYTE_SW | I2C_SLV0_REG_DIS | I2C_SLV0_GRP | I2C_SLV0_LEN[3:0] | | | |
| 28 | 40 | I2C_SLV1_ADDR | R/W | I2C_SLV1_RW | I2C_SLV1_ADDR[6:0] | | | | | | |
| 29 | 41 | I2C_SLV1_REG | R/W | I2C_SLV1_REG[7:0] | | | | | | | |
| 2A | 42 | I2C_SLV1_CTRL | R/W | I2C_SLV1_EN | I2C_SLV1_BYTE_SW | I2C_SLV1_REG_DIS | I2C_SLV1_GRP | I2C_SLV1_LEN[3:0] | | | |
| 2B | 43 | I2C_SLV2_ADDR | R/W | I2C_SLV2_RW | I2C_SLV2_ADDR[6:0] | | | | | | |
| 2C | 44 | I2C_SLV2_REG | R/W | I2C_SLV2_REG[7:0] | | | | | | | |
| 2D | 45 | I2C_SLV2_CTRL | R/W | I2C_SLV2_EN | I2C_SLV2_BYTE_SW | I2C_SLV2_REG_DIS | I2C_SLV2_GRP | I2C_SLV2_LEN[3:0] | | | |
| 2E | 46 | I2C_SLV3_ADDR | R/W | I2C_SLV3_RW | I2C_SLV3_ADDR[6:0] | | | | | | |
| 2F | 47 | I2C_SLV3_REG | R/W | I2C_SLV3_REG[7:0] | | | | | | | |
| 30 | 48 | I2C_SLV3_CTRL | R/W | I2C_SLV3_EN | I2C_SLV3_BYTE_SW | I2C_SLV3_REG_DIS | I2C_SLV3_GRP | I2C_SLV3_LEN[3:0] | | | |
| 31 | 49 | I2C_SLV4_ADDR | R/W | I2C_SLV4_RW | I2C_SLV4_ADDR[6:0] | | | | | | |
| 32 | 50 | I2C_SLV4_REG | R/W | I2C_SLV4_REG[7:0] | | | | | | | |
| 33 | 51 | I2C_SLV4_DO | R/W | I2C_SLV4_DO[7:0] | | | | | | | |
| 34 | 52 | I2C_SLV4_CTRL | R/W | I2C_SLV4_EN | I2C_SLV4_INT_EN | I2C_SLV4_REG_DIS | I2C_MST_DLY[4:0] | | | | |
| 35 | 53 | I2C_SLV4_DI | R | I2C_SLV4_DI[7:0] | | | | | | | |
| 36 | 54 | I2C_MST_STATUS | R | PASS_THROUGH | I2C_SLV4_DONE | I2C_LOST_ARB | I2C_SLV4_NACK | I2C_SLV3_NACK | I2C_SLV2_NACK | I2C_SLV1_NACK | I2C_SLV0_NACK |
| 37 | 55 | INT_PIN_CFG | R/W | INT_LEVEL | INT_OPEN | LATCH_INT_EN | INT_RD_CLEAR | FSYNC_INT_LEVEL | FSYNC_INT_EN | I2C_BYPASS_EN | - |
| 38 | 56 | INT_ENABLE | R/W | - | MOT_EN | - | FIFO_OFLOW_EN | I2C_MST_INT_EN | - | - | DATA_RDY_EN |
| 3A | 58 | INT_STATUS | R | - | MOT_INT | - | FIFO_OFLOW_INT | I2C_MST_INT | - | - | DATA_RDY_INT |
| 3B | 59 | ACCEL_XOUT_H | R | ACCEL_XOUT[15:8] | | | | | | | |
| 3C | 60 | ACCEL_XOUT_L | R | ACCEL_XOUT[7:0] | | | | | | | |
| 3D | 61 | ACCEL_YOUT_H | R | ACCEL_YOUT[15:8] | | | | | | | |
| 3E | 62 | ACCEL_YOUT_L | R | ACCEL_YOUT[7:0] | | | | | | | |
| 3F | 63 | ACCEL_ZOUT_H | R | ACCEL_ZOUT[15:8] | | | | | | | |
| 40 | 64 | ACCEL_ZOUT_L | R | ACCEL_ZOUT[7:0] | | | | | | | |
| 41 | 65 | TEMP_OUT_H | R | TEMP_OUT[15:8] | | | | | | | |
| 42 | 66 | TEMP_OUT_L | R | TEMP_OUT[7:0] | | | | | | | |
| 43 | 67 | GYRO_XOUT_H | R | GYRO_XOUT[15:8] | | | | | | | |
| 44 | 68 | GYRO_XOUT_L | R | GYRO_XOUT[7:0] | | | | | | | |
| 45 | 69 | GYRO_YOUT_H | R | GYRO_YOUT[15:8] | | | | | | | |
| 46 | 70 | GYRO_YOUT_L | R | GYRO_YOUT[7:0] | | | | | | | |
| 47 | 71 | GYRO_ZOUT_H | R | GYRO_ZOUT[15:8] | | | | | | | |
| 48 | 72 | GYRO_ZOUT_L | R | GYRO_ZOUT[7:0] | | | | | | | |
| 49 | 73 | EXT_SENS_DATA_00 | R | EXT_SENS_DATA_00[7:0] | | | | | | | |
| 4A | 74 | EXT_SENS_DATA_01 | R | EXT_SENS_DATA_01[7:0] | | | | | | | |
| 4B | 75 | EXT_SENS_DATA_02 | R | EXT_SENS_DATA_02[7:0] | | | | | | | |
| 4C | 76 | EXT_SENS_DATA_03 | R | EXT_SENS_DATA_03[7:0] | | | | | | | |
| 4D | 77 | EXT_SENS_DATA_04 | R | EXT_SENS_DATA_04[7:0] | | | | | | | |
| 4E | 78 | EXT_SENS_DATA_05 | R | EXT_SENS_DATA_05[7:0] | | | | | | | |
| 4F | 79 | EXT_SENS_DATA_06 | R | EXT_SENS_DATA_06[7:0] | | | | | | | |
| 50 | 80 | EXT_SENS_DATA_07 | R | EXT_SENS_DATA_07[7:0] | | | | | | | |
| 51 | 81 | EXT_SENS_DATA_08 | R | EXT_SENS_DATA_08[7:0] | | | | | | | |
| 52 | 82 | EXT_SENS_DATA_09 | R | EXT_SENS_DATA_09[7:0] | | | | | | | |
| 53 | 83 | EXT_SENS_DATA_10 | R | EXT_SENS_DATA_10[7:0] | | | | | | | |
| 54 | 84 | EXT_SENS_DATA_11 | R | EXT_SENS_DATA_11[7:0] | | | | | | | |
| 55 | 85 | EXT_SENS_DATA_12 | R | EXT_SENS_DATA_12[7:0] | | | | | | | |
| 56 | 86 | EXT_SENS_DATA_13 | R | EXT_SENS_DATA_13[7:0] | | | | | | | |
| 57 | 87 | EXT_SENS_DATA_14 | R | EXT_SENS_DATA_14[7:0] | | | | | | | |
| 58 | 88 | EXT_SENS_DATA_15 | R | EXT_SENS_DATA_15[7:0] | | | | | | | |
| 59 | 89 | EXT_SENS_DATA_16 | R | EXT_SENS_DATA_16[7:0] | | | | | | | |
| 5A | 90 | EXT_SENS_DATA_17 | R | EXT_SENS_DATA_17[7:0] | | | | | | | |
| 5B | 91 | EXT_SENS_DATA_18 | R | EXT_SENS_DATA_18[7:0] | | | | | | | |
| 5C | 92 | EXT_SENS_DATA_19 | R | EXT_SENS_DATA_19[7:0] | | | | | | | |
| 5D | 93 | EXT_SENS_DATA_20 | R | EXT_SENS_DATA_20[7:0] | | | | | | | |
| 5E | 94 | EXT_SENS_DATA_21 | R | EXT_SENS_DATA_21[7:0] | | | | | | | |
| 5F | 95 | EXT_SENS_DATA_22 | R | EXT_SENS_DATA_22[7:0] | | | | | | | |
| 60 | 96 | EXT_SENS_DATA_23 | R | EXT_SENS_DATA_23[7:0] | | | | | | | |
| 63 | 99 | I2C_SLV0_DO | R/W | I2C_SLV0_DO[7:0] | | | | | | | |
| 64 | 100 | I2C_SLV1_DO | R/W | I2C_SLV1_DO[7:0] | | | | | | | |
| 65 | 101 | I2C_SLV2_DO | R/W | I2C_SLV2_DO[7:0] | | | | | | | |
| 66 | 102 | I2C_SLV3_DO | R/W | I2C_SLV3_DO[7:0] | | | | | | | |
| 67 | 103 | I2C_MST_DELAY_CTRL | R/W | DELAY_ES_SHADOW | - | - | I2C_SLV4_DLY_EN | I2C_SLV3_DLY_EN | I2C_SLV2_DLY_EN | I2C_SLV1_DLY_EN | I2C_SLV0_DLY_EN |
| 68 | 104 | SIGNAL_PATH_RESET | R/W | - | - | - | - | - | GYRO_RESET | ACCEL_RESET | TEMP_RESET |
| 69 | 105 | MOT_DETECT_CTRL | R/W | - | - | ACCEL_ON_DELAY[1:0] | | - | - | | |
| 6A | 106 | USER_CTRL | R/W | - | FIFO_EN | I2C_MST_EN | I2C_IF_DIS | - | FIFO_RESET | I2C_MST_RESET | SIG_COND_RESET |
| 6B | 107 | PWR_MGMT_1 | R/W | DEVICE_RESET | SLEEP | CYCLE | - | TEMP_DIS | CLKSEL[2:0] | | |
| 6C | 108 | PWR_MGMT_2 | R/W | LP_WAKE_CTRL[1:0] | | STBY_XA | STBY_YA | STBY_ZA | STBY_XG | STBY_YG | STBY_ZG |
| 72 | 114 | FIFO_COUNTH | R/W | FIFO_COUNT[15:8] | | | | | | | |
| 73 | 115 | FIFO_COUNTL | R/W | FIFO_COUNT[7:0] | | | | | | | |
| 74 | 116 | FIFO_R_W | R/W | FIFO_DATA[7:0] | | | | | | | |
| 75 | 117 | WHO_AM_I | R | - | WHO_AM_I[6:1] | | | | | | - |

**Note:** Register Names ending in _H and _L contain the high and low bytes, respectively, of an internal register value.

In the detailed register tables that follow, register names are in capital letters, while register values are in capital letters and italicized. For example, the ACCEL_XOUT_H register (Register 59) contains the 8 most significant bits, ACCEL_XOUT[15:8], of the 16-bit X-Axis accelerometer measurement, ACCEL_XOUT.

The reset value is 0x00 for all registers other than the registers below.

- Register 107: 0x40.
- Register 117: 0x68.

---

## 4. Register Descriptions

This section describes the function and contents of each register within the MPU-60X0.

**Note:** The device will come up in sleep mode upon power-up.

### 4.1 Registers 13 to 16 – Self Test Registers

SELF_TEST_X, SELF_TEST_Y, SELF_TEST_Z, and SELF_TEST_A

**Type:** Read/Write

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 0D | 13 | XA_TEST[4-2] | | | | | XG_TEST[4-0] | | | |
| 0E | 14 | YA_TEST[4-2] | | | | | YG_TEST[4-0] | | | |
| 0F | 15 | ZA_TEST[4-2] | | | | | ZG_TEST[4-0] | | | |
| 10 | 16 | RESERVED | | XA_TEST[1-0] | | YA_TEST[1-0] | | ZA_TEST[1-0] | | |

These registers are used for gyroscope and accelerometer self-tests that permit the user to test the mechanical and electrical portions of the gyroscope and the accelerometer. The following sections describe the self-test process.

**Description:**

**1. Gyroscope Hardware Self-Test: Relative Method**

Gyroscope self-test permits users to test the mechanical and electrical portions of the gyroscope. Code for operating self-test is included within the MotionApps™ software provided by InvenSense. Please refer to the next section (Obtaining the Gyroscope Factory Trim (FT) Value) if not using MotionApps software.

When self-test is activated, the on-board electronics will actuate the appropriate sensor. This actuation will move the sensor's proof masses over a distance equivalent to a pre-defined Coriolis force. This proof mass displacement results in a change in the sensor output, which is reflected in the output signal. The output signal is used to observe the self-test response.

The self-test response (STR) is defined as follows:

```
SelfTest Response = Gyroscope Output with Self-Test Enabled − Gyroscope Output with Self-Test Disabled
```

This self test-response is used to determine whether the part has passed or failed self-test by finding the change from factory trim of the self-test response as follows:

```
Change from Factory Trim of the Self-Test Response (%) = (STR − FT) / FT
```

where,
FT = Factory trim value of selftest response, available via MotionApps software

This change from factory trim of the self-test response must be within the limits provided in the MPU-6000/MPU-6050 Product Specification document for the part to pass self-test. Otherwise, the part is deemed to have failed self-test.

**Obtaining the Gyroscope Factory Trim (FT) Value**

If InvenSense MotionApps software is not used, the procedure detailed below should be followed to obtain the Factory trim value of the self test response (FT) mentioned above. For the specific registers mentioned below, please refer to registers 13-15.

The Factory trim value of the self test response (FT) is calculated as shown below. FT[Xg], FT[Yg], and FT[Zg] refer to the factory trim (FT) values for the gyroscope X, Y, and Z axes, respectively. XG_TEST is the decimal version of XG_TEST[4-0], YG_TEST is the decimal version of YG_TEST[4-0], and ZG_TEST is the decimal version of ZG_TEST[4-0].

When performing self test for the gyroscope, the full-scale range should be set to ±250dps.

```
FT [Xg] = 25 * 131 * 1.046^(XG_TEST−1)     if XG_TEST ≠ 0
FT [Xg] = 0                                 if XG_TEST = 0

FT [Yg] = −25 * 131 * 1.046^(YG_TEST−1)     if YG_TEST ≠ 0
FT [Yg] = 0                                 if YG_TEST = 0

FT [Zg] = 25 * 131 * 1.046^(ZG_TEST−1)      if ZG_TEST ≠ 0
FT [Zg] = 0                                 if ZG_TEST = 0
```

**2. Accelerometer Hardware Self-Test: Relative Method**

Accelerometer self-test permits users to test the mechanical and electrical portions of the accelerometer. Code for operating self-test is included within the MotionApps software provided by InvenSense. Please refer to the next section (titled Obtaining the Accelerometer Factory Trim (FT) Value) if not using MotionApps software.

When self-test is activated, the on-board electronics will actuate the appropriate sensor. This actuation simulates an external force. The actuated sensor, in turn, will produce a corresponding output signal. The output signal is used to observe the self-test response.

The self-test response (STR) is defined as follows:

```
SelfTest Response = Accelerometer Output with Self-Test Enabled − Accelerometer Output with Self-Test Disabled
```

This self test-response is used to determine whether the part has passed or failed self-test by finding the change from factory trim of the self-test response as follows:

```
Change from Factory Trim of the Self-Test Response (%) = (STR − FT) / FT
```

where,
FT = Factory trim value of selftest response, available via MotionApps software

This change from factory trim of the self-test response must be within the limits provided in the MPU-6000/MPU-6050 Product Specification document for the part to pass self-test. Otherwise, the part is deemed to have failed self-test.

**Obtaining the Accelerometer Factory Trim (FT) Value**

If InvenSense MotionApps software is not used, the procedure detailed below should be followed to obtain the Factory trim value of the self test response (FT) mentioned above. For the specific registers mentioned below, please refer to registers 13-16.

The Factory trim value of the self test response (FT) is calculated as shown below. FT[Xa], FT[Ya], and FT[Za] refer to the factory trim (FT) values for the accelerometer X, Y, and Z axes, respectively. In the equations below, the factory trim values for the accel should be in decimal format, and they are determined by concatenating the upper accelerometer self test bits (bits 4-2) with the lower accelerometer self test bits (bits 1-0).

When performing accelerometer self test, the full-scale range should be set to ±8g.

```
FT[Xa] = 4096 * 0.34 * (0.92/0.34)^((XA_TEST−1)/2^5−2)     if XA_TEST ≠ 0.
FT[Xa] = 0                                                   if XA_TEST = 0.

FT[Ya] = 4096 * 0.34 * (0.92/0.34)^((YA_TEST−1)/2^5−2)     if YA_TEST ≠ 0.
FT[Ya] = 0                                                   if YA_TEST = 0.

FT[Za] = 4096 * 0.34 * (0.92/0.34)^((ZA_TEST−1)/2^5−2)     if ZA_TEST ≠ 0.
FT[Za] = 0                                                   if ZA_TEST = 0.
```

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| XA_TEST | 5-bit unsigned value. FT[Xa] is determined by using this value as explained above. |
| XG_TEST | 5-bit unsigned value. FT[Xg] is determined by using this value as explained above. |
| YA_TEST | 5-bit unsigned value. FT[Ya] is determined by using this value as explained above. |
| YG_TEST | 5-bit unsigned value. FT[Yg] is determined by using this value as explained above. |
| ZA_TEST | 5-bit unsigned value. FT[Za] is determined by using this value as explained above. |
| ZG_TEST | 5-bit unsigned value. FT[Zg] is determined by using this value as explained above. |

### 4.2 Register 25 – Sample Rate Divider

SMPRT_DIV

**Type:** Read/Write

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 19 | 25 | SMPLRT_DIV[7:0] | | | | | | | |

**Description:**

This register specifies the divider from the gyroscope output rate used to generate the Sample Rate for the MPU-60X0.

The sensor register output, FIFO output, DMP sampling and Motion detection are all based on the Sample Rate.

The Sample Rate is generated by dividing the gyroscope output rate by SMPLRT_DIV:

```
Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
```

where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled (see Register 26).

**Note:** For a diagram of the gyroscope and accelerometer signal paths, see Section 8 of the MPU-6000/MPU-6050 Product Specification document.

The accelerometer output rate is 1kHz. This means that for a Sample Rate greater than 1kHz, the same accelerometer sample may be output to the FIFO, DMP, and sensor registers more than once.

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| SMPLRT_DIV | 8-bit unsigned value. The Sample Rate is determined by dividing the gyroscope output rate by this value. |

### 4.3 Register 26 – Configuration

CONFIG

**Type:** Read/Write

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 1A | 26 | - | - | EXT_SYNC_SET[2:0] | | | DLPF_CFG[2:0] | | |

This register configures the external Frame Synchronization (FSYNC) pin sampling and the Digital Low Pass Filter (DLPF) setting for both the gyroscopes and accelerometers.

**Description:**

An external signal connected to the FSYNC pin can be sampled by configuring EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short strobes may be captured. The latched FSYNC signal will be sampled at the Sampling Rate, as defined in register 25. After sampling, the latch will reset to the current FSYNC signal state.

The sampled value will be reported in place of the least significant bit in a sensor data register determined by the value of EXT_SYNC_SET according to the following table.

| EXT_SYNC_SET | FSYNC Bit Location |
|--------------|-------------------|
| 0 | Input disabled |
| 1 | TEMP_OUT_L[0] |
| 2 | GYRO_XOUT_L[0] |
| 3 | GYRO_YOUT_L[0] |
| 4 | GYRO_ZOUT_L[0] |
| 5 | ACCEL_XOUT_L[0] |
| 6 | ACCEL_YOUT_L[0] |
| 7 | ACCEL_ZOUT_L[0] |

The DLPF is configured by DLPF_CFG. The accelerometer and gyroscope are filtered according to the value of DLPF_CFG as shown in the table below.

| DLPF_CFG | Accelerometer (Fs = 1kHz) Bandwidth (Hz) | Accelerometer Delay (ms) | Gyroscope Bandwidth (Hz) | Gyroscope Delay (ms) | Fs (kHz) |
|----------|------------------------------------------|--------------------------|--------------------------|----------------------|----------|
| 0 | 260 | 0 | 256 | 0.98 | 8 |
| 1 | 184 | 2.0 | 188 | 1.9 | 1 |
| 2 | 94 | 3.0 | 98 | 2.8 | 1 |
| 3 | 44 | 4.9 | 42 | 4.8 | 1 |
| 4 | 21 | 8.5 | 20 | 8.3 | 1 |
| 5 | 10 | 13.8 | 10 | 13.4 | 1 |
| 6 | 5 | 19.0 | 5 | 18.6 | 1 |
| 7 | RESERVED | RESERVED | | | 8 |

Bit 7 and bit 6 are reserved.

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| EXT_SYNC_SET | 3-bit unsigned value. Configures the FSYNC pin sampling. |
| DLPF_CFG | 3-bit unsigned value. Configures the DLPF setting. |

### 4.4 Register 27 – Gyroscope Configuration

GYRO_CONFIG

**Type:** Read/Write

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 1B | 27 | XG_ST | YG_ST | ZG_ST | FS_SEL[1:0] | | - | - | - |

This register is used to trigger gyroscope self-test and configure the gyroscopes' full scale range.

**Description:**

Gyroscope self-test permits users to test the mechanical and electrical portions of the gyroscope. The self-test for each gyroscope axis can be activated by controlling the XG_ST, YG_ST, and ZG_ST bits of this register. Self-test for each axis may be performed independently or all at the same time.

When self-test is activated, the on-board electronics will actuate the appropriate sensor. This actuation will move the sensor's proof masses over a distance equivalent to a pre-defined Coriolis force. This proof mass displacement results in a change in the sensor output, which is reflected in the output signal. The output signal is used to observe the self-test response.

The self-test response is defined as follows:

```
Self-test response = Sensor output with self-test enabled – Sensor output without self-test enabled
```

The self-test limits for each gyroscope axis is provided in the electrical characteristics tables of the MPU-6000/MPU-6050 Product Specification document. When the value of the self-test response is within the min/max limits of the product specification, the part has passed self test. When the self-test response exceeds the min/max values specified in the document, the part is deemed to have failed self-test.

FS_SEL selects the full scale range of the gyroscope outputs according to the following table.

| FS_SEL | Full Scale Range |
|--------|------------------|
| 0 | ± 250 °/s |
| 1 | ± 500 °/s |
| 2 | ± 1000 °/s |
| 3 | ± 2000 °/s |

Bits 2 through 0 are reserved.

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| XG_ST | Setting this bit causes the X axis gyroscope to perform self test. |
| YG_ST | Setting this bit causes the Y axis gyroscope to perform self test. |
| ZG_ST | Setting this bit causes the Z axis gyroscope to perform self test. |
| FS_SEL | 2-bit unsigned value. Selects the full scale range of gyroscopes. |

### 4.5 Register 28 – Accelerometer Configuration

ACCEL_CONFIG

**Type:** Read/Write

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 1C | 28 | XA_ST | YA_ST | ZA_ST | AFS_SEL[1:0] | | - | | |

This register is used to trigger accelerometer self test and configure the accelerometer full scale range. This register also configures the Digital High Pass Filter (DHPF).

**Description:**

Accelerometer self-test permits users to test the mechanical and electrical portions of the accelerometer. The self-test for each accelerometer axis can be activated by controlling the XA_ST, YA_ST, and ZA_ST bits of this register. Self-test for each axis may be performed independently or all at the same time.

When self-test is activated, the on-board electronics will actuate the appropriate sensor. This actuation simulates an external force. The actuated sensor, in turn, will produce a corresponding output signal. The output signal is used to observe the self-test response.

The self-test response is defined as follows:

```
Self-test response = Sensor output with self-test enabled – Sensor output without self-test enabled
```

The self-test limits for each accelerometer axis is provided in the electrical characteristics tables of the MPU-6000/MPU-6050 Product Specification document. When the value of the self-test response is within the min/max limits of the product specification, the part has passed self test. When the self-test response exceeds the min/max values specified in the document, the part is deemed to have failed self-test.

AFS_SEL selects the full scale range of the accelerometer outputs according to the following table.

| AFS_SEL | Full Scale Range |
|---------|------------------|
| 0 | ± 2g |
| 1 | ± 4g |
| 2 | ± 8g |
| 3 | ± 16g |

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| XA_ST | When set to 1, the X- Axis accelerometer performs self test. |
| YA_ST | When set to 1, the Y- Axis accelerometer performs self test. |
| ZA_ST | When set to 1, the Z- Axis accelerometer performs self test. |
| AFS_SEL | 2-bit unsigned value. Selects the full scale range of accelerometers. |

### 4.6 Register 31 – Motion Detection Threshold

MOT_THR

**Type:** Read/Write

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 1F | 31 | MOT_THR[7:0] | | | | | | | |

This register configures the detection threshold for Motion interrupt generation. The mg per LSB increment for MOT_THR can be found in the Electrical Specifications table of the MPU-6000/MPU-6050 Product Specification document.

**Description:**

Motion is detected when the absolute value of any of the accelerometer measurements exceeds this Motion detection threshold.

The Motion interrupt will indicate the axis and polarity of detected motion in MOT_DETECT_STATUS (Register 97).

For more details on the Motion detection interrupt, see Section 8.3 of the MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and 58 of this document.

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| MOT_THR | 8-bit unsigned value. Specifies the Motion detection threshold. |

### 4.7 Register 35 – FIFO Enable

FIFO_EN

**Type:** Read/Write

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 23 | 35 | TEMP_FIFO_EN | XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN | ACCEL_FIFO_EN | SLV2_FIFO_EN | SLV1_FIFO_EN | SLV0_FIFO_EN |

This register determines which sensor measurements are loaded into the FIFO buffer.

**Description:**

Data stored inside the sensor data registers (Registers 59 to 96) will be loaded into the FIFO buffer if a sensor's respective FIFO_EN bit is set to 1 in this register.

When a sensor's FIFO_EN bit is enabled in this register, data from the sensor data registers will be loaded into the FIFO buffer. The sensors are sampled at the Sample Rate as defined in Register 25.

For further information regarding sensor data registers, please refer to Registers 59 to 96.

When an external Slave's corresponding FIFO_EN bit (SLVx_FIFO_EN, where x=0, 1, or 2) is set to 1, the data stored in its corresponding data registers (EXT_SENS_DATA registers, Registers 73 to 96) will be written into the FIFO buffer at the Sample Rate. EXT_SENS_DATA register association with I2C Slaves is determined by the I2C_SLVx_CTRL registers (where x=0, 1, or 2; Registers 39, 42, and 45). For information regarding EXT_SENS_DATA registers, please refer to Registers 73 to 96.

Note that the corresponding FIFO_EN bit (SLV3_FIFO_EN) is found in I2C_MST_CTRL (Register 36). Also note that Slave 4 behaves in a different manner compared to Slaves 0-3. Please refer to Registers 49 to 53 for further information regarding Slave 4 usage.

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| TEMP_FIFO_EN | When set to 1, this bit enables TEMP_OUT_H and TEMP_OUT_L (Registers 65 and 66) to be written into the FIFO buffer. |
| XG_FIFO_EN | When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and 68) to be written into the FIFO buffer. |
| YG_FIFO_EN | When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and 70) to be written into the FIFO buffer. |
| ZG_FIFO_EN | When set to 1, this bit enables GYRO_ZOUT_H and GYRO_ZOUT_L (Registers 71 and 72) to be written into the FIFO buffer. |
| ACCEL_FIFO_EN | When set to 1, this bit enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) to be written into the FIFO buffer. |
| SLV2_FIFO_EN | When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96) associated with Slave 2 to be written into the FIFO buffer. |
| SLV1_FIFO_EN | When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96) associated with Slave 1 to be written into the FIFO buffer. |
| SLV0_FIFO_EN | When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96) associated with Slave 0 to be written into the FIFO buffer. |

**Note:** For further information regarding the association of EXT_SENS_DATA registers to particular slave devices, please refer to Registers 73 to 96.

### 4.8 Register 36 – I2C Master Control

I2C_MST_CTRL

**Type:** Read/Write

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 24 | 36 | MULT_MST_EN | WAIT_FOR_ES | SLV_3_FIFO_EN | I2C_MST_P_NSR | I2C_MST_CLK[3:0] | | | |

This register configures the auxiliary I2C bus for single-master or multi-master control. In addition, the register is used to delay the Data Ready interrupt, and also enables the writing of Slave 3 data into the FIFO buffer. The register also configures the auxiliary I2C Master's transition from one slave read to the next, as well as the MPU-60X0's 8MHz internal clock.

**Description:**

Multi-master capability allows multiple I2C masters to operate on the same bus. In circuits where multi-master capability is required, set MULT_MST_EN to 1. This will increase current drawn by approximately 30µA.

In circuits where multi-master capability is required, the state of the I2C bus must always be monitored by each separate I2C Master. Before an I2C Master can assume arbitration of the bus, it must first confirm that no other I2C Master has arbitration of the bus. When MULT_MST_EN is set to 1, the MPU-60X0's bus arbitration detection logic is turned on, enabling it to detect when the bus is available.

When the WAIT_FOR_ES bit is set to 1, the Data Ready interrupt will be delayed until External Sensor data from the Slave Devices are loaded into the EXT_SENS_DATA registers. This is used to ensure that both the internal sensor data (i.e. from gyro and accel) and external sensor data have been loaded to their respective data registers (i.e. the data is synced) when the Data Ready interrupt is triggered.

When the Slave 3 FIFO enable bit (SLV_3_FIFO_EN) is set to 1, Slave 3 sensor measurement data will be loaded into the FIFO buffer each time. EXT_SENS_DATA register association with I2C Slaves is determined by I2C_SLV3_CTRL (Register 48).

For further information regarding EXT_SENS_DATA registers, please refer to Registers 73 to 96.

The corresponding FIFO_EN bits for Slave 0, Slave 1, and Slave 2 can be found in Register 35.

The I2C_MST_P_NSR bit configures the I2C Master's transition from one slave read to the next slave read. If the bit equals 0, there will be a restart between reads. If the bit equals 1, there will be a stop followed by a start of the following read. When a write transaction follows a read transaction, the stop followed by a start of the successive write will be always used.

I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the MPU-60X0 internal 8MHz clock. It sets the I2C master clock speed according to the following table:

| I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider |
|-------------|------------------------|---------------------|
| 0 | 348 kHz | 23 |
| 1 | 333 kHz | 24 |
| 2 | 320 kHz | 25 |
| 3 | 308 kHz | 26 |
| 4 | 296 kHz | 27 |
| 5 | 286 kHz | 28 |
| 6 | 276 kHz | 29 |
| 7 | 267 kHz | 30 |
| 8 | 258 kHz | 31 |
| 9 | 500 kHz | 16 |
| 10 | 471 kHz | 17 |
| 11 | 444 kHz | 18 |
| 12 | 421 kHz | 19 |
| 13 | 400 kHz | 20 |
| 14 | 381 kHz | 21 |
| 15 | 364 kHz | 22 |

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| MUL_MST_EN | When set to 1, this bit enables multi-master capability. |
| WAIT_FOR_ES | When set to 1, this bit delays the Data Ready interrupt until External Sensor data from the Slave devices have been loaded into the EXT_SENS_DATA registers. |
| SLV3_FIFO_EN | When set to 1, this bit enables EXT_SENS_DATA registers associated with Slave 3 to be written into the FIFO. The corresponding bits for Slaves 0-2 can be found in Register 35. |
| I2C_MST_P_NSR | Controls the I2C Master's transition from one slave read to the next slave read. When this bit equals 0, there is a restart between reads. When this bit equals 1, there is a stop and start marking the beginning of the next read. When a write follows a read, a stop and start is always enforced. |
| I2C_MST_CLK | 4 bit unsigned value. Configures the I2C master clock speed divider. |

**Note:** For further information regarding the association of EXT_SENS_DATA registers to particular slave devices, please refer to Registers 73 to 96.

### 4.9 Registers 37 to 39 – I2C Slave 0 Control

I2C_SLV0_ADDR, I2C_SLV0_REG, and I2C_SLV0_CTRL

**Type:** Read/Write

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 25 | 37 | I2C_SLV0_RW | I2C_SLV0_ADDR[6:0] | | | | | | |
| 26 | 38 | I2C_SLV0_REG[7:0] | | | | | | | |
| 27 | 39 | I2C_SLV0_EN | I2C_SLV0_BYTE_SW | I2C_SLV0_REG_DIS | I2C_SLV0_GRP | I2C_SLV0_LEN[3:0] | | | |

These registers configure the data transfer sequence for Slave 0. Slaves 1, 2, and 3 also behave in a similar manner to Slave 0. However, Slave 4's characteristics differ greatly from those of Slaves 0-3. For further information regarding Slave 4, please refer to registers 49 to 53.

**Description:**

I2C slave data transactions between the MPU-60X0 and Slave 0 are set as either read or write operations by the I2C_SLV0_RW bit. When this bit is 1, the transfer is a read operation. When the bit is 0, the transfer is a write operation.

I2C_SLV0_ADDR is used to specify the I2C slave address of Slave 0.

Data transfer starts at an internal register within Slave 0. This address of this register is specified by I2C_SLV0_REG.

The number of bytes transferred is specified by I2C_SLV0_LEN. When more than 1 byte is transferred (I2C_SLV0_LEN > 1), data is read from (written to) sequential addresses starting from I2C_SLV0_REG.

In read mode, the result of the read is placed in the lowest available EXT_SENS_DATA register. For further information regarding the allocation of read results, please refer to the EXT_SENS_DATA register description (Registers 73 – 96).

In write mode, the contents of I2C_SLV0_DO (Register 99) will be written to the slave device.

I2C_SLV0_EN enables Slave 0 for I2C data transaction. A data transaction is performed only if more than zero bytes are to be transferred (I2C_SLV0_LEN > 0) between an enabled slave device (I2C_SLV0_EN = 1).

I2C_SLV0_BYTE_SW configures byte swapping of word pairs. When byte swapping is enabled, the high and low bytes of a word pair are swapped. Please refer to I2C_SLV0_GRP for the pairing convention of the word pairs. When this bit is cleared to 0, bytes transferred to and from Slave 0 will be written to EXT_SENS_DATA registers in the order they were transferred.

When I2C_SLV0_REG_DIS is set to 1, the transaction will read or write data only. When cleared to 0, the transaction will write a register address prior to reading or writing data. This bit should equal 0 when specifying the register address within the Slave device to/from which the ensuing data transaction will take place.

I2C_SLV0_GRP specifies the grouping order of word pairs received from registers. When cleared to 0, bytes from register addresses 0 and 1, 2 and 3, etc (even, then odd register addresses) are paired to form a word. When set to 1, bytes from register addresses are paired 1 and 2, 3 and 4, etc. (odd, then even register addresses) are paired to form a word.

I2C data transactions are performed at the Sample Rate, as defined in Register 25. The user is responsible for ensuring that I2C data transactions to and from each enabled Slave can be completed within a single period of the Sample Rate.

The I2C slave access rate can be reduced relative to the Sample Rate. This reduced access rate is determined by I2C_MST_DLY (Register 52). Whether a slave's access rate is reduced relative to the Sample Rate is determined by I2C_MST_DELAY_CTRL (Register 103).

The processing order for the slaves is fixed. The sequence followed for processing the slaves is Slave 0, Slave 1, Slave 2, Slave 3 and Slave 4. If a particular Slave is disabled it will be skipped.

Each slave can either be accessed at the sample rate or at a reduced sample rate. In a case where some slaves are accessed at the Sample Rate and some slaves are accessed at the reduced rate, the sequence of accessing the slaves (Slave 0 to Slave 4) is still followed. However, the reduced rate slaves will be skipped if their access rate dictates that they should not be accessed during that particular cycle. For further information regarding the reduced access rate, please refer to Register 52. Whether a slave is accessed at the Sample Rate or at the reduced rate is determined by the Delay Enable bits in Register 103.

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| I2C_SLV0_RW | When set to 1, this bit configures the data transfer as a read operation. When cleared to 0, this bit configures the data transfer as a write operation. |
| I2C_SLV0_ADDR | 7-bit I2C address of Slave 0. |
| I2C_SLV0_REG | 8-bit address of the Slave 0 register to/from which data transfer starts. |
| I2C_SLV0_EN | When set to 1, this bit enables Slave 0 for data transfer operations. When cleared to 0, this bit disables Slave 0 from data transfer operations. |
| I2C_SLV0_BYTE_SW | When set to 1, this bit enables byte swapping. When byte swapping is enabled, the high and low bytes of a word pair are swapped. Please refer to I2C_SLV0_GRP for the pairing convention of the word pairs. When cleared to 0, bytes transferred to and from Slave 0 will be written to EXT_SENS_DATA registers in the order they were transferred. |
| I2C_SLV0_REG_DIS | When set to 1, the transaction will read or write data only. When cleared to 0, the transaction will write a register address prior to reading or writing data. |
| I2C_SLV0_GRP | 1-bit value specifying the grouping order of word pairs received from registers. When cleared to 0, bytes from register addresses 0 and 1, 2 and 3, etc (even, then odd register addresses) are paired to form a word. When set to 1, bytes from register addresses are paired 1 and 2, 3 and 4, etc. (odd, then even register addresses) are paired to form a word. |
| I2C_SLV0_LEN | 4-bit unsigned value. Specifies the number of bytes transferred to and from Slave 0. Clearing this bit to 0 is equivalent to disabling the register by writing 0 to I2C_SLV0_EN. |

**Byte Swapping Example**

The following example demonstrates byte swapping for I2C_SLV0_BYTE_SW = 1, I2C_SLV0_GRP = 0, I2C_SLV0_REG = 0x01, and I2C_SLV0_LEN = 0x4:

1. The first byte, read from Slave 0 register 0x01, will be stored at EXT_SENS_DATA_00. Because I2C_SLV0_GRP = 0, bytes from even, then odd register addresses will be paired together as word pairs. Since the read operation started from an odd register address instead of an even address, only one byte is read.
2. The second and third bytes will be swapped, since I2C_SLV0_BYTE_SW = 1 and I2C_SLV0_REG[0] = 1. The data read from 0x02 will be stored at EXT_SENS_DATA_02, and the data read from 0x03 will be stored at EXT_SENS_DATA_01.
3. The last byte, read from address 0x04, will be stored at EXT_SENS_DATA_03. Because there is only one byte remaining in the read operation, byte swapping will not occur.

Slave 0 is accessed at the Sample Rate, while Slave 1 is accessed at half the Sample Rate. The other slaves are disabled. In the first cycle, both Slave 0 and Slave 1 will be accessed. However, in the second cycle, only Slave 0 will be accessed. In the third cycle, both Slave 0 and Slave 1 will be accessed. In the fourth cycle, only Slave 0 will be accessed. This pattern continues.

**Slave Access Example**

### 4.10 Registers 40 to 42 – I2C Slave 1 Control

I2C_SLV1_ADDR, I2C_SLV1_REG, and I2C_SLV1_CTRL

**Type:** Read/Write

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 28 | 40 | I2C_SLV1_RW | I2C_SLV1_ADDR[6:0] | | | | | | |
| 29 | 41 | I2C_SLV1_REG[7:0] | | | | | | | |
| 2A | 42 | I2C_SLV1_EN | I2C_SLV1_BYTE_SW | I2C_SLV1_REG_DIS | I2C_SLV1_GRP | I2C_SLV1_LEN[3:0] | | | |

These registers describe the data transfer sequence for Slave 1. Their functions correspond to those described for the Slave 0 registers (Registers 37 to 39).

### 4.11 Registers 43 to 45 – I2C Slave 2 Control

I2C_SLV2_ADDR, I2C_SLV2_REG, and I2C_SLV2_CTRL

**Type:** Read/Write

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 2B | 43 | I2C_SLV2_RW | I2C_SLV2_ADDR[6:0] | | | | | | |
| 2C | 44 | I2C_SLV2_REG[7:0] | | | | | | | |
| 2D | 45 | I2C_SLV2_EN | I2C_SLV2_BYTE_SW | I2C_SLV2_REG_DIS | I2C_SLV2_GRP | I2C_SLV2_LEN[3:0] | | | |

These registers describe the data transfer sequence for Slave 2. Their functions correspond to those described for the Slave 0 registers (Registers 37 to 39).

### 4.12 Registers 46 to 48 – I2C Slave 3 Control

I2C_SLV3_ADDR, I2C_SLV3_REG, and I2C_SLV3_CTRL

**Type:** Read/Write

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 2E | 46 | I2C_SLV3_RW | I2C_SLV3_ADDR[6:0] | | | | | | |
| 2F | 47 | I2C_SLV3_REG[7:0] | | | | | | | |
| 30 | 48 | I2C_SLV3_EN | I2C_SLV3_BYTE_SW | I2C_SLV3_REG_DIS | I2C_SLV3_GRP | I2C_SLV3_LEN[3:0] | | | |

These registers describe the data transfer sequence for Slave 3. Their functions correspond to those described for the Slave 0 registers (Registers 37 to 39).

### 4.13 Registers 49 to 53 – I2C Slave 4 Control

I2C_SLV4_ADDR, I2C_SLV4_REG, I2C_SLV4_DO, I2C_SLV4_CTRL, and I2C_SLV4_DI

**Type:** Read/Write

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 31 | 49 | I2C_SLV4_RW | I2C_SLV4_ADDR[6:0] | | | | | | |
| 32 | 50 | I2C_SLV4_REG[7:0] | | | | | | | |
| 33 | 51 | I2C_SLV4_DO[7:0] | | | | | | | |
| 34 | 52 | I2C_SLV4_EN | I2C_SLV4_INT_EN | I2C_SLV4_REG_DIS | I2C_MST_DLY[4:0] | | | | |
| 35 | 53 | I2C_SLV4_DI[7:0] | | | | | | | |

These registers describe the data transfer sequence for Slave 4. The characteristics of Slave 4 differ greatly from those of Slaves 0-3. For further information regarding the characteristics of Slaves 0-3, please refer to Registers 37 to 48.

**Description:**

I2C slave data transactions between the MPU-60X0 and Slave 4 are set as either read or write operations by the I2C_SLV4_RW bit. When this bit is 1, the transfer is a read operation. When the bit is 0, the transfer is a write operation.

I2C_SLV4_ADDR is used to specify the I2C slave address of Slave 4.

Data transfer starts at an internal register within Slave 4. This register address is specified by I2C_SLV4_REG.

In read mode, the result of the read will be available in I2C_SLV4_DI. In write mode, the contents of I2C_SLV4_DO will be written into the slave device.

A data transaction is performed only if the I2C_SLV4_EN bit is set to 1. The data transaction should be enabled once its parameters are configured in the _ADDR and _REG registers. For write, the _DO register is also required. I2C_SLV4_EN will be cleared after the transaction is performed once.

An interrupt is triggered at the completion of a Slave 4 data transaction if the interrupt is enabled. The status of this interrupt can be observed in Register 54.

When I2C_SLV4_REG_DIS is set to 1, the transaction will read or write data instead of writing a register address. This bit should equal 0 when specifying the register address within the Slave device to/from which the ensuing data transaction will take place.

I2C_MST_DLY configures the reduced access rate of I2C slaves relative to the Sample Rate. When a slave's access rate is decreased relative to the Sample Rate, the slave is accessed every 1 / (1 + I2C_MST_DLY) samples.

This base Sample Rate in turn is determined by SMPLRT_DIV (register 25) and DLPF_CFG (register 26). Whether a slave's access rate is reduced relative to the Sample Rate is determined by I2C_MST_DELAY_CTRL (register 103).

For further information regarding the Sample Rate, please refer to register 25.

Slave 4 transactions are performed after Slave 0, 1, 2 and 3 transactions have been completed. Thus the maximum rate for Slave 4 transactions is determined by the Sample Rate as defined in Register 25.

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| I2C_SLV4_RW | When set to 1, this bit configures the data transfer as a read operation. When cleared to 0, this bit configures the data transfer as a write operation. |
| I2C_SLV4_ADDR | 7-bit I2C address for Slave 4. |
| I2C_SLV4_REG | 8-bit address of the Slave 4 register to/from which data transfer starts. |
| I2C_SLV4_DO | This register stores the data to be written into the Slave 4. If I2C_SLV4_RW is set 1 (set to read), this register has no effect. |
| I2C_SLV4_EN | When set to 1, this bit enables Slave 4 for data transfer operations. When cleared to 0, this bit disables Slave 4 from data transfer operations. |
| I2C_SLV4_INT_EN | When set to 1, this bit enables the generation of an interrupt signal upon completion of a Slave 4 transaction. When cleared to 0, this bit disables the generation of an interrupt signal upon completion of a Slave 4 transaction. The interrupt status can be observed in Register 54. |
| I2C_SLV4_REG_DIS | When set to 1, the transaction will read or write data. When cleared to 0, the transaction will read or write a register address. |
| I2C_MST_DLY | Configures the decreased access rate of slave devices relative to the Sample Rate. |
| I2C_SLV4_DI | This register stores the data read from Slave 4. This field is populated after a read transaction. |

### 4.14 Register 54 – I2C Master Status

I2C_MST_STATUS

**Type:** Read Only

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 36 | 54 | PASS_THROUGH | I2C_SLV4_DONE | I2C_LOST_ARB | I2C_SLV4_NACK | I2C_SLV3_NACK | I2C_SLV2_NACK | I2C_SLV1_NACK | I2C_SLV0_NACK |

This register shows the status of the interrupt generating signals in the I2C Master within the MPU-60X0. This register also communicates the status of the FSYNC interrupt to the host processor.

**Description:**

Reading this register will clear all the status bits in the register.

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| PASS_THROUGH | This bit reflects the status of the FSYNC interrupt from an external device into the MPU-60X0. This is used as a way to pass an external interrupt through the MPU-60X0 to the host application processor. When set to 1, this bit will cause an interrupt if FSYNC_INT_EN is asserted in INT_PIN_CFG (Register 55). |
| I2C_SLV4_DONE | Automatically sets to 1 when a Slave 4 transaction has completed. This triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted and if the SLV_4_DONE_INT bit is asserted in the I2C_SLV4_CTRL register (Register 52). |
| I2C_LOST_ARB | This bit automatically sets to 1 when the I2C Master has lost arbitration of the auxiliary I2C bus (an error condition). This triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted. |
| I2C_SLV4_NACK | This bit automatically sets to 1 when the I2C Master receives a NACK in a transaction with Slave 4. This triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted. |
| I2C_SLV3_NACK | This bit automatically sets to 1 when the I2C Master receives a NACK in a transaction with Slave 3. This triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted. |
| I2C_SLV2_NACK | This bit automatically sets to 1 when the I2C Master receives a NACK in a transaction with Slave 2. This triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted. |
| I2C_SLV1_NACK | This bit automatically sets to 1 when the I2C Master receives a NACK in a transaction with Slave 1. This triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted. |
| I2C_SLV0_NACK | This bit automatically sets to 1 when the I2C Master receives a NACK in a transaction with Slave 0. This triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted. |

### 4.15 Register 55 – INT Pin / Bypass Enable Configuration

INT_PIN_CFG

**Type:** Read/Write

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 37 | 55 | INT_LEVEL | INT_OPEN | LATCH_INT_EN | INT_RD_CLEAR | FSYNC_INT_LEVEL | FSYNC_INT_EN | I2C_BYPASS_EN | - |

This register configures the behavior of the interrupt signals at the INT pins. This register is also used to enable the FSYNC Pin to be used as an interrupt to the host application processor, as well as to enable Bypass Mode on the I2C Master. This bit also enables the clock output.

**Description:**

FSYNC_INT_EN enables the FSYNC pin to be used as an interrupt to the host application processor. A transition to the active level specified in FSYNC_INT_LEVEL will trigger an interrupt. The status of this interrupt is read from the PASS_THROUGH bit in the I2C Master Status Register (Register 54).

When I2C_BYPASS_EN is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to 0, the host application processor will be able to directly access the auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host application processor will not be able to directly access the auxiliary I2C bus of the MPU-60X0 regardless of the state of I2C_MST_EN.

For further information regarding Bypass Mode, please refer to Section 7.11 and 7.13 of the MPU-6000/MPU-6050 Product Specification document.

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| INT_LEVEL | When this bit is equal to 0, the logic level for the INT pin is active high. When this bit is equal to 1, the logic level for the INT pin is active low. |
| INT_OPEN | When this bit is equal to 0, the INT pin is configured as push-pull. When this bit is equal to 1, the INT pin is configured as open drain. |
| LATCH_INT_EN | When this bit is equal to 0, the INT pin emits a 50us long pulse. When this bit is equal to 1, the INT pin is held high until the interrupt is cleared. |
| INT_RD_CLEAR | When this bit is equal to 0, interrupt status bits are cleared only by reading INT_STATUS (Register 58). When this bit is equal to 1, interrupt status bits are cleared on any read operation. |
| FSYNC_INT_LEVEL | When this bit is equal to 0, the logic level for the FSYNC pin (when used as an interrupt to the host processor) is active high. When this bit is equal to 1, the logic level for the FSYNC pin (when used as an interrupt to the host processor) is active low. |
| FSYNC_INT_EN | When equal to 0, this bit disables the FSYNC pin from causing an interrupt to the host processor. When equal to 1, this bit enables the FSYNC pin to be used as an interrupt to the host processor. |
| I2C_BYPASS_EN | When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to 0, the host application processor will be able to directly access the auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host application processor will not be able to directly access the auxiliary I2C bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106 bit[5]). |

### 4.16 Register 56 – Interrupt Enable

INT_ENABLE

**Type:** Read/Write

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 38 | 56 | - | MOT_EN | - | FIFO_OFLOW_EN | I2C_MST_INT_EN | - | - | DATA_RDY_EN |

This register enables interrupt generation by interrupt sources.

**Description:**

For information regarding the interrupt status for each interrupt generation source, please refer to Register 58. Further information regarding I2C Master interrupt generation can be found in Register 54.

Bits 2 and 1 are reserved.

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| MOT_EN | When set to 1, this bit enables Motion detection to generate an interrupt. |
| FIFO_OFLOW_EN | When set to 1, this bit enables a FIFO buffer overflow to generate an interrupt. |
| I2C_MST_INT_EN | When set to 1, this bit enables any of the I2C Master interrupt sources to generate an interrupt. |
| DATA_RDY_EN | When set to 1, this bit enables the Data Ready interrupt, which occurs each time a write operation to all of the sensor registers has been completed. |

### 4.17 Register 58 – Interrupt Status

INT_STATUS

**Type:** Read Only

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 3A | 58 | - | MOT_INT | - | FIFO_OFLOW_INT | I2C_MST_INT | - | - | DATA_RDY_INT |

This register shows the interrupt status of each interrupt generation source. Each bit will clear after the register is read.

**Description:**

For information regarding the corresponding interrupt enable bits, please refer to Register 56. For a list of I2C Master interrupts, please refer to Register 54.

Bits 2 and 1 are reserved.

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| MOT_INT | This bit automatically sets to 1 when a Motion Detection interrupt has been generated. The bit clears to 0 after the register has been read. |
| FIFO_OFLOW_INT | This bit automatically sets to 1 when a FIFO buffer overflow interrupt has been generated. The bit clears to 0 after the register has been read. |
| I2C_MST_INT | This bit automatically sets to 1 when an I2C Master interrupt has been generated. For a list of I2C Master interrupts, please refer to Register 54. The bit clears to 0 after the register has been read. |
| DATA_RDY_INT | This bit automatically sets to 1 when a Data Ready interrupt is generated. The bit clears to 0 after the register has been read. |

### 4.18 Registers 59 to 64 – Accelerometer Measurements

ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L

**Type:** Read Only

| Register (Hex) | Register (Decimal) | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
|----------------|-------------------|------|------|------|------|------|------|------|------|
| 3B | 59 | ACCEL_XOUT[15:8] | | | | | | | |
| 3C | 60 | ACCEL_XOUT[7:0] | | | | | | | |
| 3D | 61 | ACCEL_YOUT[15:8] | | | | | | | |
| 3E | 62 | ACCEL_YOUT[7:0] | | | | | | | |
| 3F | 63 | ACCEL_ZOUT[15:8] | | | | | | | |
| 40 | 64 | ACCEL_ZOUT[7:0] | | | | | | | |

These registers store the most recent accelerometer measurements.

**Description:**

Accelerometer measurements are written to these registers at the Sample Rate as defined in Register 25.

The accelerometer measurement registers, along with the temperature measurement registers, gyroscope measurement registers, and external sensor data registers, are composed of two sets of registers: an internal register set and a user-facing read register set.

The data within the accelerometer sensors' internal register set is always updated at the Sample Rate. Meanwhile, the user-facing read register set duplicates the internal register set's data values whenever the serial interface is idle. This guarantees that a burst read of sensor registers will read measurements from the same sampling instant. Note that if burst reads are not used, the user is responsible for ensuring a set of single byte reads correspond to a single sampling instant by checking the Data Ready interrupt.

Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS (Register 28). For each full scale setting, the accelerometers' sensitivity per LSB in ACCEL_xOUT is shown in the table below.

| AFS_SEL | Full Scale Range | LSB Sensitivity |
|---------|------------------|-----------------|
| 0 | ±2g | 16384 LSB/g |
| 1 | ±4g | 8192 LSB/g |
| 2 | ±8g | 4096 LSB/g |
| 3 | ±16g | 2048 LSB/g |

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| ACCEL_XOUT | 16-bit 2's complement value. Stores the most recent X axis accelerometer measurement. |
| ACCEL_YOUT | 16-bit 2's complement value. Stores the most recent Y axis accelerometer measurement. |
| ACCEL_ZOUT | 16-bit 2's complement value. Stores the most recent Z axis accelerometer measurement. |

---

*(The provided document content ends at page 31. The original PDF continues with sections 4.19 through 4.34, but they are not included in the available text layer.)*

---

**Note:** This Markdown file represents the available text content from the provided PDF. For the complete register map and descriptions, including sections 4.19 to 4.34, please refer to the original document.

