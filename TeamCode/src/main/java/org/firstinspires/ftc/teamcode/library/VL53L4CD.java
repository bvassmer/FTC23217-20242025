package org.firstinspires.ftc.teamcode.library;
import android.util.Log;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "VL53L4CD TOF Sensor", xmlTag = "VL53L4CD")
public class VL53L4CD extends I2cDeviceSynchDevice<I2cDeviceSynch>{
    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {
        return "VL53L4CD TOF Sensor";
    }

    public enum SensorState {
        BOOTING,
        WAIT_FOR_BOOT,
        POST_BOOT,
        WAIT,
        DATA_READY,
    };
    protected ElapsedTime bootTimer = new ElapsedTime();

    public VL53L4CDResult distanceData;
    SensorState tofSensorState = SensorState.BOOTING;
    public SensorState getState() {
        return tofSensorState;
    }

    final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x29);

    public VL53L4CD(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setLogging(true);

        Log.d("Rhinos", "i2c Heartbeat interval " + this.deviceClient.getHeartbeatInterval());
        Log.d("Rhinos", "i2c connection info " + this.deviceClient.getConnectionInfo());
        Log.d("Rhinos", "i2c health status " + this.deviceClient.getHealthStatus());
        Log.d("Rhinos", "i2c getDeviceName " + this.deviceClient.getDeviceName());
        Log.d("Rhinos", "i2c getManufacturer " + this.deviceClient.getManufacturer());
        Log.d("Rhinos", "i2c isWriteCoalescingEnabled " + this.deviceClient.isWriteCoalescingEnabled());

        // startTimers();

        // this.setOptimalReadWindow();

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged

        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        // this.deviceClient.engage();
        Log.d("Rhinos", "VL53L4CD I2C Engaged. Make it so.");
        Log.d("Rhinos", "i2c Heartbeat interval " + this.deviceClient.getHeartbeatInterval());
        Log.d("Rhinos", "i2c connection info " + this.deviceClient.getConnectionInfo());
        Log.d("Rhinos", "i2c health status " + this.deviceClient.getHealthStatus());
        Log.d("Rhinos", "i2c read window " + this.deviceClient.getReadWindow());
    }

    public enum Register
    {
        SYSTEM_START(0x87),
        RESULT_RANGE_STATUS(0x89), // 137
        RESULT_SPAD_NB(0x8C),
        RESULT_SIGNAL_RATE(0x8E),
        RESULT_AMBIENT_RATE(0x90),
        RESULT_SIGMA(0x92),
        RESULT_DISTANCE(0x96),
        FIRMWARE_STATUS(0xE5),
        IDENTIFICATION_MODEL_ID(0x10F),// 271
        GPIO_HV_MUX_CTRL(0x03),
        OSC_FREQUENCY(0x06),
        VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND(0x08),
        UNKNOWN_ONE(0x0B),
        UNKNOWN_TWO(0x24),
        CONFIG(0x2d),
        GPIO_TIO_HV_STATUS(0x31),
        RANGE_CONFIG_A(0x5E),
        RANGE_CONFIG_B(0x61),
        INTERMEASUREMENT_MS(0x6C),
        SYSTEM_INTERRUPT_CLEAR(0x86),
        RESULT_OSC_CALIBRATE_VAL(0xDE);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }

    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRMWARE_STATUS.bVal,
               1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    private void startTimers() {
        bootTimer.startTime();
    }
    protected void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected void writeInt(final Register reg, int value) {
        deviceClient.write(reg.bVal, TypeConversion.intToByteArray(value));
    }

    protected short readShort(Register reg)
    {
        Log.d("Rhinos", "VL53L4CD I2C readShort " + reg.toString() + " " + reg.bVal);
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    protected byte readByte(Register reg) {
        Log.d("Rhinos", "VL53L4CD I2C readByte " + reg.toString() + " " + reg.bVal);
        return deviceClient.read8(reg.bVal);
    }

    public void sensorStateMachine() {
        switch (tofSensorState) {
            case BOOTING:
                if (checkForBooted()) {
                   tofSensorState = SensorState.POST_BOOT;
                } else {
                    bootTimer.reset();
                    tofSensorState = SensorState.WAIT_FOR_BOOT;
                }
                break;
            case WAIT_FOR_BOOT:
                if (bootTimer.seconds() > 1) {
                    tofSensorState = SensorState.BOOTING;
                }
                break;
            case POST_BOOT:
                startVhv();
                loadConfig();
                tofSensorState = SensorState.WAIT;
                break;
            case WAIT:
                if (checkForDataReady()) {
                    tofSensorState = SensorState.DATA_READY;
                }
                break;
            case DATA_READY:
                // put data into variables
                distanceData = getResult();

                // clear interrupt
                clearInterrupt();
                break;
            default:
                // shouldn't get here.
                // TODO: error handling
        }
    }

    private boolean checkForDataReady() {
        short temp;
        short intPol;

        temp = readShort(Register.GPIO_HV_MUX_CTRL);
        temp = (short) (temp & 0x10);
        temp = (short) (temp >> 4);

        if (temp == 1) {
            intPol = 0;
        } else {
            intPol = 1;
        }

        temp = readShort(Register.GPIO_TIO_HV_STATUS);
        if ((temp & 1) == intPol) {
            return true;
        } else {
            return false;
        }
    }

    private void clearInterrupt() {
        writeShort(Register.SYSTEM_INTERRUPT_CLEAR, (short)1); // 0x01 == 1
    }

    public void startRanging()
    {
        // VL53L4CD_ERROR status = VL53L4CD_ERROR.VL53L4CD_ERROR_NONE;
        boolean dataReady;
        short continueLoop = 1;
        short i = 0;
        int temp;

        temp = readShort(Register.INTERMEASUREMENT_MS);

        if (temp == 0) {
            /* Sensor runs in continuous mode */
            writeShort(Register.SYSTEM_START, (short)33); // 0x21 == 33
        } else {
            /* Sensor runs in autonomous mode */
            writeShort(Register.SYSTEM_START, (short)64); // 0x40 == 64
        }

        /* do {
            dataReady = checkForDataReady();
            if (dataReady) { // Data ready
                continueLoop = 0;
            } else if (i < 1000) {  // Wait for answer
                i += 1; // TODO: better timer
            } else { // Timeout 1000ms reached
                continueLoop = 0;
                // status = 1; // TODO: better error system. VL53L4CD_ERROR_TIMEOUT;
            }
        } while (continueLoop == 1); */
        // clearInterrupt();
    }

    public void stopRanging() {
        writeShort(Register.SYSTEM_START, (short)0); // 0x00 == 0
    }

    public VL53L4CDResult getResult() {
        // VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
        VL53L4CDResult result = new VL53L4CDResult();
        short temp;
        short status_rtn[] = new short[] { 255, 255, 255, 5, 2, 4, 1, 7, 3,
                0, 255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
                255, 255, 11, 12 };

        temp = readShort(Register.RESULT_RANGE_STATUS);
        temp = (short)(temp & 0x1F);
        if (temp < 24) {
            temp = status_rtn[temp];
        }
        result.rangeStatus = temp;

        temp = readShort(Register.RESULT_SPAD_NB);
        result.numberOfSpad = temp / 256;

        temp = readShort(Register.RESULT_SIGNAL_RATE);
        result.signalRateKcps = temp * 8;

        temp = readShort(Register.RESULT_AMBIENT_RATE);
        result.ambientRateKcps = temp * 8;

        temp = readShort(Register.RESULT_SIGMA);
        result.sigmaMm = temp / 4;

        temp = readShort(Register.RESULT_DISTANCE);
        result.distanceMm = temp;

        result.signalPerSpadKcps = result.signalRateKcps / result.numberOfSpad;
        result.ambientPerSpadKcps = result.ambientRateKcps / result.numberOfSpad;

        return result;
    }

    private void setRangeTiming(int timingBudgetMs, int interMeasurementMs) {
        short clockPll, oscFreq, msByte;
        short status = 0;
        int macroPeriodUs = 0;
        int timingBudgetUs = 0;
        int lsByte, temp;
        float interMeasurementFactor = (float)1.055;

        oscFreq = readShort(Register.OSC_FREQUENCY);
        if (oscFreq != (short)0) {
            timingBudgetUs = timingBudgetMs * 1000;

            // macro_period_us = (uint32_t)((uint32_t)2304 * ((uint32_t)0x40000000 / (uint32_t)osc_frequency)) >> 6;
            macroPeriodUs = 2304 * (1073741824 / oscFreq) >> 6; // 0x40000000 == 1073741824
        } else {
            // ERROR: VL53L4CD_ERROR_INVALID_ARGUMENT
            status = 1; // TODO: replace with better error status
        }

        if (timingBudgetMs < (short)10 || (timingBudgetMs > (short)200 || status != (short)0)) {
            /* Timing budget check validity */
            // ERROR: VL53L4CD_ERROR_INVALID_ARGUMENT
            status = 1; // TODO: replace with better error status
        } else if (interMeasurementMs == 0) {
            /* Sensor runs in continuous mode */
            // VL53L4CD_WrDWord(dev, VL53L4CD_INTERMEASUREMENT_MS, 0);
            writeShort(Register.INTERMEASUREMENT_MS, (short)0);
            timingBudgetUs -= 2500;
        } else if (interMeasurementMs > timingBudgetMs) {
            /* Sensor runs in autonomous low power mode */
            clockPll = readShort(Register.RESULT_OSC_CALIBRATE_VAL);

            // clock_pll & (uint16_t)0x3FF;
            clockPll = (short)(clockPll & (short)1023); // 0x3FF == 1023

            interMeasurementFactor = interMeasurementFactor * interMeasurementMs * clockPll;

            // VL53L4CD_WrDWord(dev, VL53L4CD_INTERMEASUREMENT_MS, (uint32_t)inter_measurement_factor);
            writeInt(Register.INTERMEASUREMENT_MS, (int)interMeasurementFactor); // float cast to int?

            timingBudgetUs -= 4300;
            timingBudgetUs /= 2;
        } else {
            /* Invalid case */
            /* Timing budget check validity */
            // ERROR: VL53L4CD_ERROR_INVALID_ARGUMENT
            status = 1; // TODO: replace with better error status
        }

        if (status != 0) { // TODO: replace with better error status
            msByte = 0;

            timingBudgetUs = timingBudgetUs << 12;
            temp = macroPeriodUs * 16;
            lsByte = ((timingBudgetUs + ((temp >> 6) >> 1)) / (temp >> 6)) - 1;

            while ((lsByte & 0xFFFFFF00) > 0) { // Removed U (unsigned) suffix from hex and 0.
                lsByte = lsByte >> 1;
                msByte += 1;
            }

            msByte = (short)((msByte << 8) + (lsByte & 0xFF));
            writeShort(Register.RANGE_CONFIG_A, msByte);

            msByte = 0;
            temp = macroPeriodUs * 12;

            lsByte = ((timingBudgetUs + ((temp >> 6) >> 1)) / (temp >> 6)) - 1;
            while ((lsByte & 0xFFFFFF00) > 0) {
                lsByte = lsByte >> 1;
                msByte += 1;
            }
            msByte = (short)((msByte << 8) + (lsByte & 0xFF));
            writeShort(Register.RANGE_CONFIG_B, msByte);
        }
    }


    private boolean checkForBooted() {
        short tmp;

        tmp = readShort(Register.FIRMWARE_STATUS); // TODO: Check other registers? Maybe I am reading this wrong (try reading a byte?)?
        Log.d("Rhinos check for booted", Short.toString(tmp));
        if (tmp == (short)0x3) {
            // Sensor Booted
            return true;
        }
        return false;
        // TODO: Need a failsafe for hung boot.
    }

    private void loadConfig() {
        // load default configuration
        // test to see if we have to do this. hopefully not.
        // if we do we will have to figure out how to iterate over
        // bytes. leaning is fun.
        /*
        for (address = 0x20; address <= 0x87; address += 1) {
            writeShort(address, VL53L4CD_DEFAULT_CONFIGURATION);
        }
        */
        int config = TypeConversion.byteArrayToInt(VL53L4CD_DEFAULT_CONFIGURATION);
        writeInt(Register.CONFIG, config);
    }


    public void startSensor() {
        // VL53L4CD_ERROR status = VL53L4CD_ERROR_NONE;
        short address, tmp;
        int continue_loop = 1;
        int i = 0;

        clearInterrupt();
        stopRanging();

        // VL53L4CD_WrByte(dev, VL53L4CD_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, (uint8_t)0x09);
        writeShort(Register.VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, (short)9); // 0x09 == 9

        // Unknown task here...
        // VL53L4CD_WrByte(dev, 0x0B, (uint8_t)0);
        writeShort(Register.UNKNOWN_ONE, (short)0);

        // ANOTHER Unknown task here..
        // L53L4CD_WrWord(dev, 0x0024, 0x500);
        writeShort(Register.UNKNOWN_TWO, (short)1280); // 0x500 == 1280

        setRangeTiming(50, 0);
    }

    private void startVhv() {
        // start VHV (What is VHV?)
        writeShort(Register.SYSTEM_START, (short)64); // 0x40 == 64
    }

    private boolean waitForReady() {
        final int MAX_ITERATIONS = 100000;
        int continueLoop = 1;
        int i = 0;
        do {
            boolean dataReady = checkForDataReady();
            if (dataReady) {
                continueLoop = 0;
                return true;
            } else if (i < MAX_ITERATIONS) {  /* Wait for answer */
                i += 1;
            } else { /* Timeout 1000ms reached */
                continueLoop = 0;
                return false;
                // status = VL53L4CD_ERROR_TIMEOUT; // TODO: better error reporting
            }
        } while (continueLoop == 1);
        return false;
    }

    final byte[] VL53L4CD_DEFAULT_CONFIGURATION = new byte[]{
        0x12, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C),
else don't touch */
        0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1
(pull up at AVDD) */
        0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1
(pull up at AVDD) */
        0x11, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low
(bits 3:0 must be 0x1), use SetInterruptPolarity() */
        0x02, /* 0x31 : bit 1 = interrupt depending on the polarity,
use CheckForDataReady() */
        0x00, /* 0x32 : not user-modifiable */
        0x02, /* 0x33 : not user-modifiable */
        0x08, /* 0x34 : not user-modifiable */
        0x00, /* 0x35 : not user-modifiable */
        0x08, /* 0x36 : not user-modifiable */
        0x10, /* 0x37 : not user-modifiable */
        0x01, /* 0x38 : not user-modifiable */
        0x01, /* 0x39 : not user-modifiable */
        0x00, /* 0x3a : not user-modifiable */
        0x00, /* 0x3b : not user-modifiable */
        0x00, /* 0x3c : not user-modifiable */
        0x00, /* 0x3d : not user-modifiable */
        (byte)0xFF, /* 0x3e : not user-modifiable */
        0x00, /* 0x3f : not user-modifiable */
        0x0F, /* 0x40 : not user-modifiable */
        0x00, /* 0x41 : not user-modifiable */
        0x00, /* 0x42 : not user-modifiable */
        0x00, /* 0x43 : not user-modifiable */
        0x00, /* 0x44 : not user-modifiable */
        0x00, /* 0x45 : not user-modifiable */
        0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high,
2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
        0x0b, /* 0x47 : not user-modifiable */
        0x00, /* 0x48 : not user-modifiable */
        0x00, /* 0x49 : not user-modifiable */
        0x02, /* 0x4a : not user-modifiable */
        0x14, /* 0x4b : not user-modifiable */
        0x21, /* 0x4c : not user-modifiable */
        0x00, /* 0x4d : not user-modifiable */
        0x00, /* 0x4e : not user-modifiable */
        0x05, /* 0x4f : not user-modifiable */
        0x00, /* 0x50 : not user-modifiable */
        0x00, /* 0x51 : not user-modifiable */
        0x00, /* 0x52 : not user-modifiable */
        0x00, /* 0x53 : not user-modifiable */
        (byte)0xc8, /* 0x54 : not user-modifiable */
        0x00, /* 0x55 : not user-modifiable */
        0x00, /* 0x56 : not user-modifiable */
        0x38, /* 0x57 : not user-modifiable */
        (byte)0xff, /* 0x58 : not user-modifiable */
        0x01, /* 0x59 : not user-modifiable */
        0x00, /* 0x5a : not user-modifiable */
        0x08, /* 0x5b : not user-modifiable */
        0x00, /* 0x5c : not user-modifiable */
        0x00, /* 0x5d : not user-modifiable */
        0x01, /* 0x5e : not user-modifiable */
        (byte)0xcc, /* 0x5f : not user-modifiable */
        0x07, /* 0x60 : not user-modifiable */
        0x01, /* 0x61 : not user-modifiable */
        (byte)0xf1, /* 0x62 : not user-modifiable */
        0x05, /* 0x63 : not user-modifiable */
        0x00, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB),
use SetSigmaThreshold(), default value 90 mm  */
        (byte)0xa0, /* 0x65 : Sigma threshold LSB */
        0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB),
use SetSignalThreshold() */
        (byte)0x80, /* 0x67 : Min count Rate LSB */
        0x08, /* 0x68 : not user-modifiable */
        0x38, /* 0x69 : not user-modifiable */
        0x00, /* 0x6a : not user-modifiable */
        0x00, /* 0x6b : not user-modifiable */
        0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register,
use SetIntermeasurementInMs() */
        0x00, /* 0x6d : Intermeasurement period */
        0x0f, /* 0x6e : Intermeasurement period */
        (byte)0x89, /* 0x6f : Intermeasurement period LSB */
        0x00, /* 0x70 : not user-modifiable */
        0x00, /* 0x71 : not user-modifiable */
        0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB),
use SetD:tanceThreshold() */
        0x00, /* 0x73 : distance threshold high LSB */
        0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB),
use SetD:tanceThreshold() */
        0x00, /* 0x75 : distance threshold low LSB */
        0x00, /* 0x76 : not user-modifiable */
        0x01, /* 0x77 : not user-modifiable */
        0x07, /* 0x78 : not user-modifiable */
        0x05, /* 0x79 : not user-modifiable */
        0x06, /* 0x7a : not user-modifiable */
        0x06, /* 0x7b : not user-modifiable */
        0x00, /* 0x7c : not user-modifiable */
        0x00, /* 0x7d : not user-modifiable */
        0x02, /* 0x7e : not user-modifiable */
        (byte)0xc7, /* 0x7f : not user-modifiable */
        (byte)0xff, /* 0x80 : not user-modifiable */
        (byte)0x9B, /* 0x81 : not user-modifiable */
        0x00, /* 0x82 : not user-modifiable */
        0x00, /* 0x83 : not user-modifiable */
        0x00, /* 0x84 : not user-modifiable */
        0x01, /* 0x85 : not user-modifiable */
        0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
        0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(),
       If you want an automatic start after VL53L4CD_init() call,
            put 0x40 in location 0x87 */
    };
}


