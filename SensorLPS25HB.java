import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import jdk.dio.DeviceManager;
import jdk.dio.i2cbus.I2CDevice;
import jdk.dio.i2cbus.I2CDeviceConfig;

/**
 *
 * @author Леонид Бурдиков, leonid.b.d@gmail.com
 */
public class SensorLPS25HB {
    private I2CDevice dev;
    private final I2CDeviceConfig conf;
    
    private boolean oneshot = false;
    
    
    /**Constucts new instance of this class.
     * 
     * @param controllerNumber Number of I2C Bus controller (1 by default).
     * @param clockFrequency Frequency of the bus. Either 100000 or 400000 Hz.
     * @param address Address of the device on the bus. 92 or 93.
     */
    SensorLPS25HB(int controllerNumber, int clockFrequency, int address) {
        conf = new I2CDeviceConfig.Builder().
                setControllerNumber(controllerNumber).
                setAddress(address, I2CDeviceConfig.ADDR_SIZE_7).
                setClockFrequency(clockFrequency).build();
    }
    
    /**Constructs new instance of this class.
     * <p>Controller number is set to 1, clock frequency is set to 100000.
     * @param address Address of the device on the bus. 92 or 93.
     */
    SensorLPS25HB(int address){
        this(1, 100000, address);
    }
    
    /**Constructs new instance of this class. 
     * <p>Controller number is set to 1, clock frequency is set to 100000
     * and address of the device is set to 92.
     */
    SensorLPS25HB(){
        this(1, 100000, 92);
    }
    
    /**Reads who am i register of the device.
     * <p>LPS25HB returns -67. If it is not the case, try another I2C bus' address.
     * @return -67
     * @throws IOException
     */
    public byte whoAmI() throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x0F,1,buf);
        dev.close();
        return buf.get(0);
    }
    
    /**Sets the reference pressure value.
     * <p>This value will be subtracted from the output value on each ODR cycle
     * if the AutoZero bit is set to 1. Note: You should first enable autozero
     * function and wait for at least one ODR cycle to complete before setting
     * this value.
     *
     * @param pr Reference pressure in hPa. 0 - 4095.
     * @return
     * @throws IOException
     */
    public SensorLPS25HB setRefPressure(int pr) throws IOException {
        if (pr < 0 || pr > 4095) throw new IllegalArgumentException("Value out of range.");
        ByteBuffer buf = ByteBuffer.allocateDirect(4);
        buf.order(ByteOrder.LITTLE_ENDIAN)
            .putInt(0,pr*4096)
            .limit(3);
        dev = DeviceManager.open(conf);
        dev.write(0x88,1,buf);
        dev.close();
        return this;
    }

    public enum TemperatureResolution {
        _8(0),
        _16(1),
        _32(2),
        _64(3);

        public final int value;

        private TemperatureResolution(int value) {
            this.value = value;
        }
    }
    
    /**Sets the temperature resolution.
     * <p>Use TMP_RES_n static constants where n is the number of samples
     * averaged.
     *
     * @param samples One of the TemperatureResolution enum units
     * @return this
     * @throws IOException
     */
    public SensorLPS25HB setAVGT(TemperatureResolution samples) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x10,1,buf);
        byte reg = (byte) (buf.get(0) & 0b1111_0011 | (samples.value << 2));
        buf.put(0,reg).rewind();
        dev.write(0x10,1,buf);
        dev.close();
        return this;
    }
    
    public static final int PRS_RES_8 = 0;
    public static final int PRS_RES_32 = 1;
    public static final int PRS_RES_128 = 2;
    public static final int PRS_RES_256 = 3;
    
    /**Sets the pressure resolution.
     * <p>Use PRS_RES_n static constants where n is the number of samples
     * averaged.
     * 
     * @param avg One of the PRS_RES_n constants.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setAVGP(int avg) throws IOException {
        if (avg < 0 || avg > 3) throw new IllegalArgumentException("Value must be in range 0-3");
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x10,1,buf);
        byte reg = (byte) (buf.get(0) & 0b1111_1100 | avg);
        buf.put(0,reg).rewind();
        dev.write(0x10,1,buf);
        dev.close();
        return this;
    }
    
    /**Sets the Power control bit to the desired value.
     * <p>This bit allows the turn on of the device. The device is in power-down
     * mode after boot by default.
     * @param enable True - power on.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setPower(boolean enable) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x20,1,buf);
        byte reg = buf.get(0);
        if (enable) reg |= 0b1000_0000; else reg &= 0b0111_1111;
        buf.put(0,reg).rewind();
        dev.write(0x20,1,buf);
        dev.close();
        return this;
    }
    
    public static final int ODR_ONESHOT = 0;
    public static final int ODR_1 = 1;
    public static final int ODR_7 = 2;
    public static final int ODR_12p5 = 3;
    public static final int ODR_25 = 4;
    
    /**Sets the Output Data Rate of the device.
     * <p>Use the ODR_n static constants where n is the value in Hz.
     * <p>When ODR is set to one shot device is not gathering samples by itself.
     * @param rate One of the ODR_n constants.
     * @return
     * @throws IOException
     */
    public SensorLPS25HB setODR(int rate) throws IOException {
        if (rate < 0 || rate > 4) throw new IllegalArgumentException("Value must be in range 0-4");
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x20,1,buf);
        byte reg = (byte) (buf.get(0) & 0b1000_1111 | (rate << 4));
        buf.put(0,reg).rewind();
        dev.write(0x20,1,buf);
        dev.close();
        oneshot = rate == 0;
        return this;
    }
    
    /**Enables differrential interrupt generation.
     * <p>Enables device to generate interrupt when difference between output
     * value and reference pressure exceeds treshold.
     * 
     * @param enable True - int generation enabled.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setDiffIntGeneration(boolean enable) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x20,1,buf);
        byte reg = buf.get(0);
        if (enable) reg |= 0b0000_1000; else reg &= 0b1111_0111;
        buf.put(0,reg).rewind();
        dev.write(0x20,1,buf);
        dev.close();
        return this;
    }
    
    /**Sets the Block Data Update bit to the desired value.
     * <p>The BDU bit is used to inhibit the output register update between the reading of the upper
     * and lower register parts. In default mode (BDU = �?0’), the lower and upper register parts are
     * updated continuously. If it is not certain whether the read will be faster than output data rate,
     * it is recommended to set the BDU bit to �?1’. In this way, after the reading of the lower (upper)
     * register part, the content of that output register is not updated until the upper (lower) part is
     * read also.
     * 
     * <p>This feature prevents the reading of LSB and MSB related to different samples.
     * @param enable True - BDU enabled.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setBDU(boolean enable) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x20,1,buf);
        byte reg = buf.get(0);
        if (enable) reg |= 0b0000_0100; else reg &= 0b1111_1011;
        buf.put(0,reg).rewind();
        dev.write(0x20,1,buf);
        dev.close();
        return this;
    }
    
    /**Resets the AutoZero function of the device.
     * <p>Clears the reference pressure register and disables autozero function.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB resetAutoZero() throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x20,1,buf);
        byte reg = (byte) (buf.get(0) | 0b0000_0010);
        buf.put(0,reg).rewind();
        dev.write(0x20,1,buf);
        dev.close();
        return this;
    }
   
    /**Select the SPI serial interface mode.
     * <p>4-wire either 3-wire.
     * @param threeWire True - 3-wire.
     * @return
     * @throws IOException 
     */ 
    public SensorLPS25HB setSIM(boolean threeWire) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x20,1,buf);
        byte reg = buf.get(0);
        if (threeWire) reg |= 0b0000_0001; else reg &= 0b1111_1110;
        buf.put(0,reg).rewind();
        dev.write(0x20,1,buf);
        dev.close();
        return this;
    }
    
    /**Refresh the content of the internal registers.
     * 
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB reboot() throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x21,1,buf);
        byte reg = (byte) (buf.get(0) | 0b1000_0000);
        buf.put(0,reg).rewind();
        dev.write(0x21,1,buf);
        dev.close();
        return this;
    }
    
    /**Enable or disable FIFO using.
     * 
     * @param enable True - FIFO enabled.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setFIFO(boolean enable) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x21,1,buf);
        byte reg = buf.get(0);
        if (enable) reg |= 0b0100_0000; else reg &= 0b1011_1111;
        buf.put(0,reg).rewind();
        dev.write(0x21,1,buf);
        dev.close();
        return this;
    }
    
    /**Enable or disable FIFO Treshold flag setting.
     * <p>Device can set a flag when FIFO is filled to the watermark level.
     * This flag is set in FIFO_STATUS register and can be checked using
     * {@link getFIFOStatus()}.
     * @param enable
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setStopOnFTH(boolean enable) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x21,1,buf);
        byte reg = buf.get(0);
        if (enable) reg |= 0b0010_0000; else reg &= 0b1101_1111;
        buf.put(0,reg).rewind();
        dev.write(0x21,1,buf);
        dev.close();
        return this;
    }
    
    /**Decimate the ODR of the device to 1Hz while it in FIFO Mean mode.
     * <p>When this bit is �?1’, the output is decimated to 1 Hz as the moving average is being taken at
     * the rate of the ODR. Otherwise, averaged pressure data will be updated according to the
     * ODR defined.
     * @param enable
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setFifoMeanDec(boolean enable) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x21,1,buf);
        byte reg = buf.get(0);
        if (enable) reg |= 0b0001_0000; else reg &= 0b1110_1111;
        buf.put(0,reg).rewind();
        dev.write(0x21,1,buf);
        dev.close();
        return this;
    }
    
    /**Enable or disable using of I2C.
     * 
     * @param enable
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setI2C(boolean enable) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x21,1,buf);
        byte reg = buf.get(0);
        if (enable) reg |= 0b0000_1000; else reg &= 0b1111_0111;
        buf.put(0,reg).rewind();
        dev.write(0x21,1,buf);
        dev.close();
        return this;
    }
    
    /**Software reset.
     * <p>The device is reset to the power-on configuration after
     * SWRESET bit is set to '1'. The software reset process takes 4 μsec. When BOOT follows,
     * the recommended sequence is SWRESET first and then BOOT.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB swReset() throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x21,1,buf);
        byte reg = (byte) (buf.get(0) | 0b0000_0100);
        buf.put(0,reg).rewind();
        dev.write(0x21,1,buf);
        dev.close();
        return this;
    }
    
    /**Enable or disable autozero function.
     * <p>When enabled, the actual pressure output value is copied in REF_P_H (0Ah),
     * REF_P_L (09h) and REF_P_XL (08h). When this bit is enabled, the register content of
     * REF_P is subtracted from the pressure output value.
     * @param enable True - enabled.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setAutoZero(boolean enable) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x21,1,buf);
        byte reg = buf.get(0);
        if (enable) reg |= 0b0000_0010; else reg &= 0b1111_1101;
        buf.put(0,reg).rewind();
        dev.write(0x21,1,buf);
        dev.close();
        return this;
    }
    
    private void oneShot() throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        if (dev.isOpen()){
            dev.read(0x21,1,buf);
            byte reg = (byte) (buf.get(0) | 0b0000_0001);
            buf.put(0,reg).rewind();
            dev.write(0x21,1,buf);
        }
    }
    
    /**Set the active level on the int pin.
     * 
     * @param high True - high.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setIntHighLow(boolean high) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x22,1,buf);
        byte reg = buf.get(0);
        if (high) reg &= 0b0111_1111; else reg |= 0b1000_0000;
        buf.put(0,reg).rewind();
        dev.write(0x22,1,buf);
        dev.close();
        return this;
    }
    
    /**Set selection mode on int pin.
     * <p>Either push-pull or open drain.
     * @param pp True - push-pull.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setPP_OD(boolean pp) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x22,1,buf);
        byte reg = buf.get(0);
        if (pp) reg &= 0b1011_1111; else reg |= 0b0100_0000;
        buf.put(0,reg).rewind();
        dev.write(0x22,1,buf);
        dev.close();
        return this;
    } 
    
    public static final int INT_MD_DATA_SIG = 0;
    public static final int INT_MD_PHIGH = 1;
    public static final int INT_MD_PLOW = 2;
    public static final int INT_MD_PHIGH_OR_PLOW = 3;
    
    /**Data signal on interrupt pin control.
     * <p>Device can generate next signals on int pin:
     * <p>Data signal - data ready and FIFO events;
     * <p>Pressure high, pressure low, pressure high or low - when differential
     * pressure interrupt is generated.
     * @param mode One of the INT_MD static constants.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setIntMode(int mode) throws IOException {
        if (mode < 0 || mode > 3) throw new IllegalArgumentException("Value out of range.");
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x22,1,buf);
        byte reg = (byte) (buf.get(0) & 0b1111_1100 | mode);
        buf.put(0,reg).rewind();
        dev.write(0x22,1,buf);
        dev.close();
        return this;
    }
    
    public static final int INT_DS_F_EMPTY = 8;
    public static final int INT_DS_F_FTH = 4;
    public static final int INT_DS_F_OVR = 2;
    public static final int INT_DS_DRDY = 1;
    
    /**Specify events on which interrupt will be generated in Data Signal mode.
     * <p>Four different events can be specified: FIFO is empty, FIFO is filled
     * to the watermark level, FIFO is full if FIFO mode of overrun occured in
     * Stream mode, new data is available.
     * <p>Use this when INT_MD_DATA_SIG is set.
     * @param config Bitwise combination of one or more INT_DS static constants.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setIntDataSigConf(int config) throws IOException {
        if (config < 0 || config > 15) throw new IllegalArgumentException("Value out of range");
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        buf.put(0,(byte)config);
        dev = DeviceManager.open(conf);
        dev.write(0x23,1,buf);
        dev.close();
        return this;
    }
    
    /**Whether to latch interrupt request or not.
     * <p>When this bit is active an interrupt is latched to the INT_SOURCE
     * register and can be later viewed by {@link getIntSource()}. When this
     * bit is not active, the IA (interrupt active) is only remains high while
     * pressure exceeds the treshold, so that you can not know later whether
     * was an interrupt or not.
     * @param enable True - enabled.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setLIR(boolean enable) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x24,1,buf);
        byte reg = buf.get(0);
        if (enable) reg |= 0b0000_0100; else reg &= 0b1111_1011;
        buf.put(0,reg).rewind();
        dev.write(0x24,1,buf);
        dev.close();
        return this;
    }

    public static final int INT_DIF_NOINT = 0;
    public static final int INT_DIF_HIGH = 1;
    public static final int INT_DIF_LOW = 2;
    public static final int INT_DIF_HIGH_LOW = 3;
    
    /**Specify on which events differential pressure interrupt will be generated.
     * <p>Device can generate interrupts when differential pressure is lower than
     * treshold (low pressure event) or higher than treshold (high pressure event).
     * <p>Use this when other option then INT_MD_DATA_SIG is set.
     * @param mode One of the INT_DIF static constants.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setIntDiffConf(int mode) throws IOException {
        if (mode < 0 || mode > 3) throw new IllegalArgumentException("Value out of range");
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x24,1,buf);
        byte reg = (byte) (buf.get(0) & 0b1111_1100 | mode);
        buf.put(0,reg).rewind();
        dev.write(0x24,1,buf);
        dev.close();
        return this;
    }

    public enum FIFOMode {
        BYPASS(0),
        FIFO(1),
        STREAM(2),
        STR_TO_FIFO(3),
        BYP_TO_STR(4),
        MEAN(6),
        BYP_TO_FIFO(7);

        public final int value;

        private FIFOMode(int value) {
            this.value = value;
        }
    }
    
    /**Set FIFO operating mode.
     * <p>FIFO can operate in 7 different modes: bypass, FIFO, stream, stream-to-FIFO,
     * bypass-to-stream, FIFO mean mode and bypass-to-FIFO mode. Read datasheet
     * of the device to futher information.
     * @param mode One of the FIFO_MD static constants.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setFIFOMode(FIFOMode mode) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x2e,1,buf);
        byte reg = (byte) (buf.get(0) & 0b0001_1111 | (mode.value << 5));
        buf.put(0,reg).rewind();
        dev.write(0x2e,1,buf);
        dev.close();
        return this;
    }
    
    /**Set FIFO watermark level.
     * <p>Set the value on which a flag in FIFO_STATUS register will be set.
     * @param value Value in range 0-31.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setFIFOWatermark (int value) throws IOException {
        if (value < 0 || value > 31) throw new IllegalArgumentException("Value out of range");
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x2e,1,buf);
        byte reg = (byte) (buf.get(0) & 0b1110_0000 | value);
        buf.put(0,reg).rewind();
        dev.write(0x2e,1,buf);
        dev.close();
        return this;
    }
    
    /**Set the treshold for interrupt generation.
     * 
     * @param value Value in hPa in range 0-4095.
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setIntTreshold(int value) throws IOException {
        if (value < 0 || value > 4095) throw new IllegalArgumentException("Value out of range");
        ByteBuffer buf = ByteBuffer.allocateDirect(4);
        buf.order(ByteOrder.LITTLE_ENDIAN).putInt(0,value*16).limit(2);
        dev = DeviceManager.open(conf);
        dev.write(0xb0,1,buf);
        dev.close();
        return this;
    }
    
    /**Get FIFO_STATUS register value.
     * <p>Use this along with {@link getIntSource()} after you recieve an interrupt 
     * to differentiate between sources.
     * @return
     * @throws IOException 
     */
    public byte getFIFOStatus() throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x2f,1,buf);
        dev.close();
        return buf.get(0);
    }
    
    /**Get INT_SOURCE register value.
     * <p>Use this along with {@link getFIFOStatus()} after you recieve an interrupt
     * to differentiate between sources.
     * @return
     * @throws IOException 
     */
    public byte getIntSource() throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x25,1,buf);
        dev.close();
        return buf.get(0);
    }
    
    /**Gets all the data stored in the FIFO of the device.
     * <p>This method returns an array in which data is located in the next manner:
     * <p>res[0] = pressure data 1, res[1] = temp data 1, res[2] = pressure data 2...
     * and so on.
     * @return
     * @throws IOException 
     */
    public double[] getFIFOData() throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        
        dev = DeviceManager.open(conf);
        dev.read(0x2f,1,buf);
        
        int x = buf.get(0) & 0b0001_1111;
        double[] res = new double[2*x];
        
        ByteBuffer pbuf = ByteBuffer.allocateDirect(3);
        ByteBuffer tbuf = ByteBuffer.allocateDirect(2);
        ByteBuffer intbuf = ByteBuffer.allocateDirect(4);
        
        byte sign, val;
        
        for (int i = 0; i < x; i++) {
            dev.read(0xa8,1,pbuf);
            dev.read(0xab,1,tbuf);
            
            sign = (byte)(pbuf.get(2) & 0b1000_0000);
            val = (byte)(pbuf.get(2) & 0b0111_1111);
            
            if (sign == 0){
                intbuf.put(0,(byte)0);
                intbuf.put(1,val);
                intbuf.put(2,pbuf.get(1));
                intbuf.put(3,pbuf.get(0));

                res[i] = (double)intbuf.getInt(0) / 4096;
            }
            else {
                intbuf.put(0,(byte)0);
                intbuf.put(1,pbuf.get(2));
                intbuf.put(2,pbuf.get(1));
                intbuf.put(3,pbuf.get(0));

                res[i] = (double)((0b00000001_00000000_00000000_00000000 - intbuf.getInt(0)) * (-1)) / 4096;
            }
            
            i++;
            
            intbuf.put(3, tbuf.get(0));
            intbuf.put(2, tbuf.get(1));
            intbuf.put(1, (byte)0);
            intbuf.put(0, (byte)0);
            
            res[i] = (double)intbuf.getInt(0);
            
            pbuf.rewind();
            tbuf.rewind();
        }
        dev.close();
        return res;
    }
    
    /**Gets one set of the data.
     * <p>Returns an array which contains pressure data in its first cell and
     * temperature data in its second.
     * @return
     * @throws IOException
     * @throws InterruptedException 
     */
    public double[] getData() throws IOException, InterruptedException {
        ByteBuffer pbuf = ByteBuffer.allocateDirect(3);
        ByteBuffer tbuf = ByteBuffer.allocateDirect(2);
        ByteBuffer intbuf = ByteBuffer.allocateDirect(4);
        
        if (oneshot) {
            oneShot();
            Thread.sleep(50);
        }
        
        dev = DeviceManager.open(conf);
        dev.read(0xa8,1,pbuf);
        dev.read(0xab,1,tbuf);
        dev.close();
        
        double[] res = new double[2];
        
        byte sign, val;
        sign = (byte) (pbuf.get(2) & 0b1000_0000);
        val = (byte) (pbuf.get(2) & 0b0111_1111);
        if (sign == 0) {
            intbuf.put(0,(byte)0);
            intbuf.put(1,val);
            intbuf.put(2,pbuf.get(1));
            intbuf.put(3,pbuf.get(0));
            
            res[0] = (double)intbuf.getInt(0) / 4096;
        }
        else {
            intbuf.put(0,(byte)0);
            intbuf.put(1,pbuf.get(2));
            intbuf.put(2,pbuf.get(1));
            intbuf.put(3,pbuf.get(0));
            
            res[0] = (double)((0b00000001_00000000_00000000_00000000 - intbuf.getInt(0)) * (-1)) / 4096;
        }
        
        intbuf.put(3, tbuf.get(0));
        intbuf.put(2, tbuf.get(1));
        intbuf.put(1, (byte)0);
        intbuf.put(0, (byte)0);
        
        res[1] = (double)intbuf.getInt(0);
        
        return res;
    }
    
    /**Set pressure offset.
     * <p>This is used to implement one-point calibration.
     * @param offset
     * @return
     * @throws IOException 
     */
    public SensorLPS25HB setPresOffset(short offset) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(2);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        buf.putShort(0,offset);
        dev = DeviceManager.open(conf);
        dev.write(0xb9,1,buf);
        dev.close();
        return this;
    }
}
