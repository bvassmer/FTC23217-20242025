package org.firstinspires.ftc.teamcode.library;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "I2CXLMaxSonarEZ4 Ultrasonic Sensor", xmlTag = "I2CXLMaxSonarEZ4")
public class I2CXLMaxSonarEZ4 extends I2cDeviceSynchDevice<I2cDeviceSynch>{
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
        return "I2CXLMaxSonarEZ4 Ultrasonic Sensor";
    }

    final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x70);

    public I2CXLMaxSonarEZ4(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);

        this.setOptimalReadWindow();

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged

        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }

    public enum Register
    {
        WRITE(0xE0),
        INIT_READING(0x51),
        REPORT_LAST_VALUE(0xE1);

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
                Register.REPORT_LAST_VALUE.bVal,
                1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    public void startReading() {
        short readingShort = (short)(81);
        writeShort(Register.INIT_READING, readingShort);
    }

    public short getDistance() {
        return readShort(Register.REPORT_LAST_VALUE);
    }

}


