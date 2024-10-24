package org.firstinspires.ftc.teamcode.library;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "I2CXLMaxSonarEZ4 Ultrasonic Sensor", xmlTag = "I2CXLMaxSonarEZ4")
public class AS726xColorSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> {
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
        return "AS726x Color Sensor";
    }

    final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x49);
    private AS726xColorSensorControlSetup _controlSetup;
    private AS726xColorSensorLedControlSetup _ledControlSetup;
    private boolean BOOTED = false;
    private final int BOOT_TIME = 1; // in seconds
    private ElapsedTime stateMachineTime = new ElapsedTime();
    public enum SensorState {
        WAIT,
        DATA_READY,
        BOOT,
        BOOTING,
    };

    SensorState colorSensorState = SensorState.BOOT;

    public AS726xColorSensor(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged

        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }

    public void boot() {
        _controlSetup.setRst((byte)1);
        writeByte(Register.AS726X_CONTROL_SETUP, _controlSetup.getControlSetup());
        _controlSetup.setRst((byte)0);
    }

    private void init() {
        // try to read the version reg to make sure we can connect
        byte version = readByte(Register.AS726X_HW_VERSION);

        if (version != 0x40) {
            // version mismatch
            // TODO: Trrow error.
        }

        enableInterrupt();

        setLedDrvCurrent(LedDriverLimit.LIMIT_12MA5);
        ledDrvOff();

        setIntegrationTime((byte)50);

        setGain(Gain.GAIN_64X);

        setConversionType(ConversionType.ONE_SHOT);
    }


    public void sensorStateMachine() {
        switch (colorSensorState) {
            case BOOT:
                boot();
                colorSensorState = SensorState.BOOTING;
                stateMachineTime.startTime();
                break;
            case BOOTING:
                if (stateMachineTime.seconds() > BOOT_TIME) {
                    // booting done.
                    init();
                    colorSensorState = SensorState.WAIT;
                }
                break;
            default:
                // shouldn't get here.
                // TODO: error handling
        }
    }

    private void enableInterrupt() {
        _controlSetup.setInt((byte)1);
        writeByte(Register.AS726X_CONTROL_SETUP, _controlSetup.getControlSetup());
    }

    private void disableInterrupt() {
        _controlSetup.setInt((byte)0);
        writeByte(Register.AS726X_CONTROL_SETUP, _controlSetup.getControlSetup());
    }

    private void setLedDrvCurrent(LedDriverLimit limit) {
        _ledControlSetup.setIclDrv(limit.getValue());
        writeByte(Register.AS726X_LED_CONTROL, _ledControlSetup.getLedControlSetup());
    }
    private void ledDrvOff() {
        _ledControlSetup.setLedDrv((byte)0);
        writeByte(Register.AS726X_LED_CONTROL, _ledControlSetup.getLedControlSetup());
    }

    private void ledDrvOn() {
        _ledControlSetup.setLedDrv((byte)1);
        writeByte(Register.AS726X_LED_CONTROL, _ledControlSetup.getLedControlSetup());
    }

    private void setIntegrationTime(byte time) {
        writeByte(Register.AS726X_INT_T, time);
    }

    private void setGain(Gain gain) {
        _controlSetup.setGain(gain.getValue());
        writeByte(Register.AS726X_CONTROL_SETUP, _controlSetup.getControlSetup());
    }

    private void setConversionType(ConversionType type) {
        _controlSetup.setBank(type.getValue());
        writeByte(Register.AS726X_CONTROL_SETUP, _controlSetup.getControlSetup());
    }


    private void readCalibratedValue(CalibratedChannel channel) {
        /*int val = 0;
        val = (readInt(channel) << 24) |
                (readInt(channel + 1) << 16) |
                (readInt(channel + 2) << 8) |
                readInt(channel + 3);

        float ret;
        memcpy(&ret, &val, 4);
        return ret; */

    }

    /*float Adafruit_AS726x::readCalibratedValue(uint8_t channel) {
        uint32_t val = 0;
        val = ((uint32_t)virtualRead(channel) << 24) |
                ((uint32_t)virtualRead(channel + 1) << 16) |
                ((uint32_t)virtualRead(channel + 2) << 8) |
                (uint32_t)virtualRead(channel + 3);

        float ret;
        memcpy(&ret, &val, 4);
        return ret;
    }

    float readCalibratedViolet() {
        return (readCalibratedValue(AS7262_VIOLET_CALIBRATED));
    }
    /*!
        @brief  Read calibrated blue color value (AS7262 only)
        @return the blue reading as a 32-bit floating point number
    */
    void readCalibratedBlue() {
        // return (readCalibratedValue(AS7262_BLUE_CALIBRATED));
    }
    /*!
        @brief  Read calibrated green color value (AS7262 only)
        @return the green reading as a 32-bit floating point number
    */
    void readCalibratedGreen() {
        // return (readCalibratedValue(AS7262_GREEN_CALIBRATED));
    }
    /*!
        @brief  Read calibrated yellow color value (AS7262 only)
        @return the yellow reading as a 32-bit floating point number
    */
    void readCalibratedYellow() {
        // return (readCalibratedValue(AS7262_YELLOW_CALIBRATED));
    }
    /*!
        @brief  Read calibrated orange color value (AS7262 only)
        @return the orange reading as a 32-bit floating point number
    */
    void readCalibratedOrange() {
       // return (readCalibratedValue(AS7262_ORANGE_CALIBRATED));
    }
    /*!
        @brief  Read calibrated red color value (AS7262 only)
        @return the red reading as a 32-bit floating point number
    */
    void readCalibratedRed() {
        // return (readCalibratedValue(AS7262_RED_CALIBRATED));
    }

    // void readCalibratedValues(float *buf, uint8_t num = AS726x_NUM_CHANNELS);

    private void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    private void writeByte(final Register reg, byte value) {
        deviceClient.write8(reg.bVal, value);
    }

    private int readInt(Register reg) {
        return TypeConversion.byteArrayToInt(deviceClient.read(reg.bVal, 8));
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    protected byte readByte(Register reg) {
        return deviceClient.read8(reg.bVal);
    }

    public boolean getBooted() { return BOOTED; }
    public enum Register
    {
        AS726X_HW_VERSION(0x00),
        AS726X_FW_VERSION(0x02),
        AS726X_CONTROL_SETUP(0x04),
        AS726X_INT_T(0x05),
        AS726X_DEVICE_TEMP(0x06),
        AS726X_LED_CONTROL(0x07),

        // for reading sensor data
        AS7262_V_HIGH(0x08),
        AS7262_V_LOW(0x09),
        AS7262_B_HIGH(0x0A),
        AS7262_B_LOW(0x0B),
        AS7262_G_HIGH(0x0C),
        AS7262_G_LOW(0x0D),
        AS7262_Y_HIGH(0x0E),
        AS7262_Y_LOW(0x0F),
        AS7262_O_HIGH(0x10),
        AS7262_O_LOW(0x11),
        AS7262_R_HIGH(0x12),
        AS7262_R_LOW(0x13),

        AS7262_V_CAL(0x14),
        AS7262_B_CAL(0x18),
        AS7262_G_CAL(0x1C),
        AS7262_Y_CAL(0x20),
        AS7262_O_CAL(0x24),
        AS7262_R_CAL(0x28),
        AS726X_SLAVE_STATUS_REG(0x00),
        AS726X_SLAVE_WRITE_REG(0x01),
        AS726X_SLAVE_READ_REG(0x02),
        AS726X_SLAVE_TX_VALID(0x02),
        AS726X_SLAVE_RX_VALID(0x01),
        AS7262_VIOLET(0x08),
        AS7262_BLUE(0x0A),
        AS7262_GREEN(0x0C),
        AS7262_YELLOW(0x0E),
        AS7262_ORANGE(0x10),
        AS7262_RED(0x12),
        LIMIT_1MA(0b00), // default // 0
        LIMIT_2MA(0b01), // 1
        LIMIT_4MA(0b10), // 2
        LIMIT_8MA(0b11); // 3
        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }

    private enum LedDriverLimit {
        LIMIT_12MA5((byte)0b00), // default // 0
        LIMIT_25MA((byte)0b01), // 1
        LIMIT_50MA((byte)0b10), // 2
        LIMIT_100MA((byte)0b11); // 3

        private final byte value;
        LedDriverLimit(byte value) {
            this.value = value;
        }
        private byte getValue() {
            return value;
        }
    }

    private enum Gain {
        GAIN_1X((byte)0b00), // default // 0
        GAIN_3X7((byte)0b01), // 1
        GAIN_16X((byte)0b10), // 2
        GAIN_64X((byte)0b11); // 3

        private final byte value;
        Gain(byte value) {
            this.value = value;
        }
        private byte getValue() {
            return value;
        }
    }

    private enum ConversionType {
        MODE_0((byte)0b00), // 0
        MODE_1((byte)0b01), // 1
        MODE_2((byte)0b10), // default // 2
        ONE_SHOT((byte)0b11); // 3

        private final byte value;
        ConversionType(byte value) {
            this.value = value;
        }
        private byte getValue() {
            return value;
        }
    }

    private enum CalibratedChannel {
        AS7262_VIOLET_CALIBRATED((byte)0x14),
        AS7262_BLUE_CALIBRATED((byte)0x18),
        AS7262_GREEN_CALIBRATED((byte)0x1C),
        AS7262_YELLOW_CALIBRATED((byte)0x20),
        AS7262_ORANGE_CALIBRATED((byte)0x24),
        AS7262_RED_CALIBRATED((byte)0x28);
        private final byte value;
        CalibratedChannel(byte value) {
            this.value = value;
        }
        private byte getValue() {
            return value;
        }

    }

}



