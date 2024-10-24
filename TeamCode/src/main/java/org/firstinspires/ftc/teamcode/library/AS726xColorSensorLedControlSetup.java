package org.firstinspires.ftc.teamcode.library;


public class AS726xColorSensorLedControlSetup {
    // enable or disable indicator LED
    private byte LED_IND = 1;
    public byte getLedInd() {
        return LED_IND;
    }
    public void setLedInd(byte ledInd) {
        LED_IND = ledInd;
    }

    // indicator led current limit
    private byte ICL_IND = 2;
    public byte getIclInd() {
        return ICL_IND;
    }
    public void setIclInd(byte iclInd) {
        ICL_IND = iclInd;
    }

    // enable or disable led_drv
    private byte LED_DRV = 1;
    public byte getLedDrv() {
        return LED_DRV;
    }
    public void setLedDrv(byte ledDrv) {
        LED_DRV = ledDrv;
    }

    private byte ICL_DRV = 2;
    public byte getIclDrv() {
        return ICL_DRV;
    }
    public void setIclDrv(byte iclDrv) {
        ICL_DRV = iclDrv;
    }

    public byte getLedControlSetup() {
        return (byte)(LED_IND | (ICL_IND << 1) | (LED_DRV << 3) | (ICL_DRV << 4));
    };
}
