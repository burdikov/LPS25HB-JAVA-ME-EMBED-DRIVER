import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import jdk.dio.DeviceManager;
import jdk.dio.i2cbus.I2CDevice;
import jdk.dio.i2cbus.I2CDeviceConfig;

/**
 *
 * @author Leonid Burdikov, leonid.b.d@gmail.com
 */
public class SensorLPS25HB {
    private I2CDevice dev;
    private final I2CDeviceConfig conf;
    
    private boolean oneshot = false;
    
    
    /**
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
    
    /**
     * <p>Controller number is set to 1, clock frequency is set to 100000.
     * @param address Address of the device on the bus. 92 or 93.
     */
    SensorLPS25HB(int address){
        this(1, 100000, address);
    }
    
    /**
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
     * @return this
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

    /**
     * Contains possible temperature resolution values measured in samples.
     */
    public enum TemperatureResolution {
        /**
         * 8 samples will be averaged at each data aquisition.
         */
        _8(0),
        /**
         * 16 samples will be averaged at each data aquisition.
         */
        _16(1),
        /**
         * 32 samples will be averaged at each data aquisition.
         */
        _32(2),
        /**
         * 64 samples will be averaged at each data aquisition.
         */
        _64(3);

        public final int value;

        private TemperatureResolution(int value) {
            this.value = value;
        }
    }
    
    /**Sets the temperature resolution.
     * <p>Defines number od samples averaged whenever a new data is acquired by
     * sensor.
     *
     * @param samples One of the {@link TemperatureResolution} enum units.
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
    
    /**
     * Contains possible pressure resolution values measured in samples.
     */
    public enum PressureResolution{
        /**
         * 8 samples will be averaged at each data aquisition.
         */
        _8(0),
        /**
         * 32 samples will be averaged at each data aquisition.
         */
        _32(1),
        /**
         * 128 samples will be averaged at each data aquisition.
         */
        _128(2),
        /**
         * 256 samples will be averaged at each data aquisition.
         */
        _256(3);
        
        public final int value;
        
        PressureResolution(int value){
            this.value = value;
        }
    }
    
    /**Sets the pressure resolution.
     * <p>Defines the number of samples averaged whenever a new data is aquired
     * by sensor.
     * 
     * @param samples One of the {@link PressureResolution} enum units.
     * @return this
     * @throws IOException 
     */
    public SensorLPS25HB setAVGP(PressureResolution samples) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x10,1,buf);
        byte reg = (byte) (buf.get(0) & 0b1111_1100 | samples.value);
        buf.put(0,reg).rewind();
        dev.write(0x10,1,buf);
        dev.close();
        return this;
    }
    
    /**Sets the Power control bit to the desired value.
     * <p>This bit allows the turn on of the device. The device is in power-down
     * mode after boot by default.
     * @param enable True - power on.
     * @return this
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
    
    /**
     * Contains possible ODR values in Hz.
     */
    public enum ODR{
        /**
         * Device is set to one shot mode.
         */
        ONESHOT(0),
        /**
         * Device is acquiring data at 1 Hz.
         */
        _1(1),
        /**
         * Device is acquiring data at 7 Hz.
         */
        _7(2),
        /**
         * Device is acquiring data at 12.5 Hz.
         */
        _12_5(3),
        /**
         * Device is acquiring data at 25 Hz.
         */
        _25(4);
    
        public final int value;
        
        ODR(int value){
            this.value = value;
        }
    }
    
    /**Sets the Output Data Rate of the device.
     * <p>When ODR is set to one shot device is not gathering samples by itself.
     * @param rate One of the {@link ODR} enum units.
     * @return this
     * @throws IOException
     */
    public SensorLPS25HB setODR(ODR rate) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x20,1,buf);
        byte reg = (byte) (buf.get(0) & 0b1000_1111 | (rate.value << 4));
        buf.put(0,reg).rewind();
        dev.write(0x20,1,buf);
        dev.close();
        oneshot = rate.value == 0;
        return this;
    }
    
    /**Enables differrential interrupt generation.
     * <p>Enables device to generate interrupt when difference between output
     * value and reference pressure exceeds treshold.
     * 
     * @param enable True - int generation enabled.
     * @return this
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
     * and lower register parts. In default mode (BDU = 0), the lower and upper register parts are
     * updated continuously. If it is not certain whether the read will be faster than output data rate,
     * it is recommended to set the BDU bit to 1. In this way, after the reading of the lower (upper)
     * register part, the content of that output register is not updated until the upper (lower) part is
     * read also.
     * 
     * <p>This feature prevents the reading of LSB and MSB related to different samples.
     * @param enable True - BDU enabled.
     * @return this
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
     * @return this
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
     * @return this
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
     * @return this
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
     * @return this
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
     * @return this
     * @throws IOException 
     */
    public SensorLPS25HB setStopOnFifoTreshold(boolean enable) throws IOException {
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
     * <p>When this bit is 1, the output is decimated to 1 Hz as the moving average is being taken at
     * the rate of the ODR. Otherwise, averaged pressure data will be updated according to the
     * ODR defined.
     * @param enable
     * @return this
     * @throws IOException 
     */
    public SensorLPS25HB setFifoMeanDecimate(boolean enable) throws IOException {
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
     * @return this
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
     * SWRESET bit is set to '1'. The software reset process takes 4 msec. When BOOT follows,
     * the recommended sequence is SWRESET first and then BOOT.
     * @return this
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
     * @return this
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
     * @return this
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
     * @return this
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
    
    /**
     * Contains the modes responsive for signal source on interrupt pin.
     */
    public enum InterruptPinMode{
        /**
         * In this mode device sends interrupts when one or more events 
         * configured by {@link setInterruptDataSignalConfig()} occur.
         */
        DATA_SIGNAL(0),
        /**
         * In this mode device sends interrupts when differential pressure high
         * event occurs.
         */
        PRESSURE_HIGH(1),
        /** 
         * In this mode device sends interrupts when differential pressure low
         * event occurs.
         */
        PRESSURE_LOW(2),
        /**
         * In this mode device sends interrupts when either differential pressure
         * low or differential pressure high event occur.
         */
        PRESSURE_HIGH_OR_LOW(3);
        
        public final int value;
        
        InterruptPinMode(int value){
            this.value = value;
        }
    }
    
    /**Decide which type of signals will be passed on interrupt pin.
     * <p>Device can pass next signals on int pin from internal sources:
     * <p>Data signal - data ready and FIFO events;
     * <p>Pressure high, pressure low, pressure high or low - when differential
     * pressure interrupt is generated.
     * @param mode One of the {@link InterruptPinMode} enum units.
     * @return this
     * @throws IOException 
     */
    public SensorLPS25HB setInterruptPinMode(InterruptPinMode mode) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x22,1,buf);
        byte reg = (byte) (buf.get(0) & 0b1111_1100 | mode.value);
        buf.put(0,reg).rewind();
        dev.write(0x22,1,buf);
        dev.close();
        return this;
    }
    
    /**
     * Contains events on which interrupt can be generated in data signal mode.
     */
    public enum InterruptDataSignalConfig{
        /**
         * Interrupt will be generated whenever FIFO is empty.
         */
        FIFO_EMPTY(8),
        /**
         * Interrupt will be generated when FIFO reaches specified watermark.
         */
        FIFO_TRESHOLD(4),
        /**
         * Interrupt will be generated when FIFO is full in FIFO mode or 
         * overrun occured in stream mode.
         */
        FIFO_OVERRUN(2),
        /**
         * Interrupt will be generated when a new data is available.
         */
        DATA_READY(1);
        
        public final int value;
        
        InterruptDataSignalConfig(int value){
            this.value = value;
        }
    }
    
    /**Specify events on which interrupt will be generated in Data Signal mode.
     * <p>Four different events can be specified: FIFO is empty, FIFO is filled
     * to the watermark level, FIFO is full if FIFO mode or overrun occured in
     * Stream mode, new data is available.
     * <p>Use this when device passing data signal interrupts ({@link setInterruptPinMode()}).
     * @param config Bitwise combination of one or more {@link InterruptDataSignalConfig}
     * unit.value's.
     * @return this
     * @throws IOException 
     */
    public SensorLPS25HB setInterruptDataSignalConfig(int config) throws IOException {
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
     * @return this
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

    /**
     * Contains modes responsive for differential pressure interrupts generation.
     */
    public enum DifferentialInterruptMode{
        /**
         * Interrupt generation is disabled.
         */
        NO_INTERRUPT(0),
        /**
         * Interrupt will be generated when differential pressure high event occur.
         */
        PRESSURE_HIGH(1),
        /**
         * Interrupt will be generated when differential pressure low event occur.
         */
        PRESSURE_LOW(2),
        /**
         * Interrupt will be generated when either differential pressure low or
         * differential pressure high events occur.
         */
        PRESSURE_HIGH_OR_LOW(3);
        
        public final int value;
        
        DifferentialInterruptMode(int value){
            this.value = value;
        }
    }
    
    /**Specify on which events differential pressure interrupt will be generated.
     * <p>Device can generate interrupts when differential pressure is lower than
     * treshold (low pressure event) or higher than treshold (high pressure event).
     * <p>Use this when other option then INT_MD_DATA_SIG is set.
     * @param mode One of the {@link DifferentialInterruptMode} enum units.
     * @return this
     * @throws IOException 
     */
    public SensorLPS25HB setDifferentialInterruptConfig(DifferentialInterruptMode mode) throws IOException {
        ByteBuffer buf = ByteBuffer.allocateDirect(1);
        dev = DeviceManager.open(conf);
        dev.read(0x24,1,buf);
        byte reg = (byte) (buf.get(0) & 0b1111_1100 | mode.value);
        buf.put(0,reg).rewind();
        dev.write(0x24,1,buf);
        dev.close();
        return this;
    }

    /**
     * Contains FIFO operating modes.
     */
    public enum FIFOMode {
        /**
         * In bypass mode, FIFO is not operational and remains empty.
         */
        BYPASS(0),
        /**
         * In FIFO mode, the data from output registers are stored in the FIFO.
         * <p>A watermark interrupt can be enabled (see {@link setStopOnFifoTreshold(boolean)}) in order
         * to be raised when FIFO is filled to the level specified by {@link setFIFOWatermark(int)}.
         * The FIFO continues filling until it's full (32 slots of data for output).
         * When full, the FIFO stops collecting data.
         * <p>The FIFO buffer can store up to 32 levels of data. The FIFO depth 
         * can be limited by setting the watermark interrupt and by selecting a 
         * watermark level.
         */
        FIFO(1),
        /**
         * In stream mode, the data from output registers are stored in the FIFO.
         * The FIFO continues filling until it's full. When full, the FIFO discards
         * the older data as the newer arrive. An interrupt can be enabled and set
         * as in FIFO mode.
         */
        STREAM(2),
        /**
         * In this mode FIFO is operating as in stream mode. An interrupt can be
         * set as in FIFO mode. Once a trigger event occurs, FIFO starts operating
         * in FIFO mode.
         * Interrupt request should be latched in order for proper triggering
         * (see {@link setLIR(boolean)}).
         */
        STREAM_TO_FIFO(3),
        /**
         * In this mode the FIFO remains inoperational till the triggering event
         * occurs and the FIFO starts operating in FIFO mode.
         */
        BYPASS_TO_STREAM(4),
        /**
         * In FIFO Mean mode the pressure data are not directly sent to the output
         * register but are stored first in the FIFO to calculate the average. In
         * this mode the FIFO is used to implement a moving average of the pressure
         * data with a 2, 4, 8, 16 or 32 sample set by changing the FIFO mean mode
         * sample size defined by the {@link setFIFOWatermark(int)}.
         * <p>There are two ways of providing the output pressure data averaged by FIFO:
         * <p>1. If the output is not decimated (altered by {@link setFifoMeanDecimate(boolean)}),
         * the output is at the same ODR of the data coming from the sensor.
         * <p>2. In other case, the output is decimated to the 1 Hz.
         * <p>Please note that when using the FIFO Mean mode it is not possible 
         * to access the FIFO content.
         * <p>Make sure that the watermark value is set to one of 2, 4, 8, 16 or 
         * before enabling this function.
         */
        MEAN(6),
        /**
         * In this mode the FIFO remains inoperational until the triggering event
         * occurs and the FIFO starts operating in FIFO mode.
         */
        BYPASS_TO_FIFO(7);

        public final int value;

        private FIFOMode(int value) {
            this.value = value;
        }
    }
    
    /**Set FIFO operating mode.
     * <p>FIFO can operate in 7 different modes.
     * <p>Note that to several modes which are sensitive to interrupts (e.g. 
     * Stream-to-FIFO, Bypass-to-Stream) latching interrupt request should be 
     * enabled to proper triggering (see {@link setLIR{boolean)}.
     * @param mode One of the FIFOMode enum units.
     * @return this
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
     * @return this
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
     * @return this
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
     * @return this
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
