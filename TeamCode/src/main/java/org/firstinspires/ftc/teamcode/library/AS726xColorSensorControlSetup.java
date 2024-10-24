package org.firstinspires.ftc.teamcode.library;

public class AS726xColorSensorControlSetup {
    private byte unused = 1;

    /* 1: data ready to be read, sets int active if int is enabled */
    private byte DATA_RDY = 1;
    public void setDataReady(byte dataReady) {
        DATA_RDY = dataReady;
    }
    public byte getDataReady() {
        return DATA_RDY;
    }

    /* conversion type
     *  0b00 = Mode 0
     *  0b01 = Mode 1
     *  0b10 = Mode 2
     *  0b11 = Mode 3 One shot
     */
    private byte BANK = 2;
    public void setBank(byte bank) {
        BANK = bank;
    }
    public byte getBank() {
        return BANK;
    }

    /* Channel gain setting (all channels)
     *  0b00 = 1x
     *  0b01 = 3.7x
     *  0b10 = 16x
     *  0b11 = 64x
     */
    private byte GAIN = 2;
    public void setGain(byte gain) {
        GAIN = gain;
    }
    public byte getGain() {
        return GAIN;
    }

    /* enable or disable interrupt */
    private byte INT = 1;
    public void setInt(byte interrupt) {
        INT = interrupt;
    }
    public byte getInt() {
        return INT;
    }

    private byte RST = 1;
    public void setRst(byte rst) {
        RST = rst;
    }
    public byte getRst() {
        return RST;
    }

    public byte getControlSetup() {
        return (byte)((DATA_RDY << 1) | (BANK << 2) | (GAIN << 4) | (INT << 6) |
                (RST << 7));
    };
}
