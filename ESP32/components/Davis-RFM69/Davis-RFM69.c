#include "Davis-RFM69.h"
#include "Davis_Config.h"

#define TAG "DAVIS_RFM69"

#define LOW	0
#define HIGH 1


/// Array of instances connected to interrupts 0 and 1
//static RH_DAVIS_RFM69*		_deviceForInterrupt[];

/// Index of next interrupt number to use in _deviceForInterrupt
//static uint8_t		_interruptCount;

/// The configured interrupt pin connected to this instance
//static uint8_t				_interruptPin;

/// The index into _deviceForInterrupt[] for this device (if an interrupt is already allocated)
/// else 0xff
//static uint8_t				_myInterruptIndex;

static uint8_t _channel = 0;

/// The radio OP mode to use when mode is RHModeIdle
static uint8_t _idleMode; 

/// The reported device type
static uint8_t _deviceType;

/// The selected output power in dBm
static int8_t _power;

/// The message length in _buf
static uint8_t _bufLen;

/// Array of octets of teh last received message or the next to transmit message
static uint8_t _buf[RH_DAVIS_PACKET_LEN];

/// True when there is a valid message in the Rx buffer
static bool _rxBufValid;

/// Time in millis since the last preamble was received (and the last time the RSSI was measured)
static uint32_t	_lastPreambleTime;

#define RH_BROADCAST_ADDRESS 0xff

/// The current transport operating mode
static RH_DAVIS_RFM69_Mode _mode = RH_DAVIS_RFM69_ModeInitialising;

/// This node id
static uint8_t _thisAddress = RH_BROADCAST_ADDRESS;
	
/// Whether the transport is in promiscuous mode
static bool	_promiscuous;

/// TO header in the last received mesasge
static uint8_t _rxHeaderTo;

/// FROM header in the last received mesasge
static uint8_t _rxHeaderFrom;

/// ID header in the last received mesasge
static uint8_t _rxHeaderId;

/// FLAGS header in the last received mesasge
static uint8_t _rxHeaderFlags;

/// TO header to DavisRFM69_send in all messages
static uint8_t _txHeaderTo = RH_BROADCAST_ADDRESS;

/// FROM header to DavisRFM69_send in all messages
static uint8_t _txHeaderFrom = RH_BROADCAST_ADDRESS;

/// ID header to DavisRFM69_send in all messages
static uint8_t _txHeaderId = 0;

/// FLAGS header to DavisRFM69_send in all messages
static uint8_t _txHeaderFlags = 0;

/// The value of the last received RSSI value, in some transport specific units
static int16_t _lastRssi;

/// Count of the number of bad messages (eg bad checksum etc) received
//static uint16_t	_rxBad = 0;

/// Count of the number of successfully transmitted messaged
static uint16_t	_rxGood = 0;

/// Count of the number of bad messages (correct checksum etc) received
static uint16_t	_txGood = 0;
	
/// Channel activity detected
//static bool _cad;

/// Channel activity timeout in ms
//static unsigned int _cad_timeout = 0;

// SPI Stuff
#if CONFIG_SPI2_HOST
#define HOST_ID SPI2_HOST
#elif CONFIG_SPI3_HOST
#define HOST_ID SPI3_HOST
#endif
static spi_device_handle_t _handle;

/*
# if !CONFIG_DAVIS_RFM69_STATION_1 && !CONFIG_DAVIS_RFM69_STATION_2 && !CONFIG_DAVIS_RFM69_STATION_3 && !CONFIG_DAVIS_RFM69_STATION_4 \
		&& !CONFIG_DAVIS_RFM69_STATION_5 && !CONFIG_DAVIS_RFM69_STATION_6 && !CONFIG_DAVIS_RFM69_STATION_7 && !CONFIG_DAVIS_RFM69_STATION_8
#error NO STATION TO LISTEN!
#endif
*/
# if !CONFIG_DAVIS_RFM69_STATION_1
#define CONFIG_DAVIS_RFM69_STATION_1 0
#endif
# if !CONFIG_DAVIS_RFM69_STATION_2
#define CONFIG_DAVIS_RFM69_STATION_2 0
#endif
# if !CONFIG_DAVIS_RFM69_STATION_3
#define CONFIG_DAVIS_RFM69_STATION_3 0
#endif
# if !CONFIG_DAVIS_RFM69_STATION_4
#define CONFIG_DAVIS_RFM69_STATION_4 0
#endif
# if !CONFIG_DAVIS_RFM69_STATION_5
#define CONFIG_DAVIS_RFM69_STATION_5 0
#endif
# if !CONFIG_DAVIS_RFM69_STATION_6
#define CONFIG_DAVIS_RFM69_STATION_6 0
#endif
# if !CONFIG_DAVIS_RFM69_STATION_7
#define CONFIG_DAVIS_RFM69_STATION_7 0
#endif
# if !CONFIG_DAVIS_RFM69_STATION_8
#define CONFIG_DAVIS_RFM69_STATION_8 0
#endif

static bool stations[8];

static int8_t station = -1;

void DavisRFM69_reset(){
	// manual reset
	gpio_reset_pin(CONFIG_RST_GPIO);
	gpio_set_direction(CONFIG_RST_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(CONFIG_RST_GPIO, HIGH);
	vTaskDelay(100);
	gpio_set_level(CONFIG_RST_GPIO, LOW);
	vTaskDelay(100);
}

void DavisRFM69_hop_station(){
	station = (station + 1)%8;
	while (!stations[station]) {
		station = (station + 1)%8;
	}
}

int8_t DavisRFM69_listening_station(){

	return station;
}

void spi_init() {
	esp_err_t ret;

	gpio_reset_pin(CONFIG_NSS_GPIO);
	gpio_set_direction(CONFIG_NSS_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(CONFIG_NSS_GPIO, 1);
/*
	spi_bus_config_t buscfg = {
		.sclk_io_num = CONFIG_SCK_GPIO, // set SPI CLK pin
		.mosi_io_num = CONFIG_MOSI_GPIO, // set SPI MOSI pin
		.miso_io_num = CONFIG_MISO_GPIO, // set SPI MISO pin
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};


	ret = spi_bus_initialize( HOST_ID, &buscfg, SPI_DMA_CH_AUTO );
	ESP_LOGI(TAG, "spi_bus_initialize=%d",ret);
	assert(ret==ESP_OK);
*/
	spi_device_interface_config_t devcfg = {
		.clock_speed_hz = 5000000, // SPI clock is 5 MHz!
		//.clock_speed_hz = 1000000,
		.queue_size = 7,
		.mode = 0, // SPI mode 0
		.spics_io_num = -1, // we will use manual CS control
		.flags = (1<<6) //SPI_DEVICE_NO_DUMMY
	};

	ret = spi_bus_add_device( HOST_ID, &devcfg, &_handle);
	ESP_LOGI(TAG, "spi_bus_add_device=%d",ret);
	assert(ret==ESP_OK);
}

uint8_t spi_transfer(uint8_t address)
{
	uint8_t datain[1];
	uint8_t dataout[1];
	dataout[0] = address;
	//spi_write_byte(dev, dataout, 1 );
	//spi_read_byte(datain, dataout, 1 );

	spi_transaction_t SPITransaction;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 8;
	SPITransaction.tx_buffer = dataout;
	SPITransaction.rx_buffer = datain;
	spi_device_transmit( _handle, &SPITransaction );

	return datain[0];
}

uint8_t spiRead(uint8_t reg)
{
	uint8_t val;
	gpio_set_level(CONFIG_NSS_GPIO, LOW);
	spi_transfer(reg & ~RH_DAVIS_RFM69_SPI_WRITE_MASK); // DavisRFM69_send the address with the write mask off
	val = spi_transfer(0); // The written value is ignored, reg value is read
	gpio_set_level(CONFIG_NSS_GPIO, HIGH);
	return val;
}

uint8_t spiWrite(uint8_t reg, uint8_t val)
{
	uint8_t status = 0;
	gpio_set_level(CONFIG_NSS_GPIO, LOW);
	status = spi_transfer(reg | RH_DAVIS_RFM69_SPI_WRITE_MASK); // DavisRFM69_send the address with the write mask on
	spi_transfer(val); // New value follows
	gpio_set_level(CONFIG_NSS_GPIO, HIGH);
	return status;
}

uint8_t spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len)
{
	uint8_t status = 0;
	gpio_set_level(CONFIG_NSS_GPIO, LOW);
	status = spi_transfer(reg & ~RH_DAVIS_RFM69_SPI_WRITE_MASK); // DavisRFM69_send the start address with the write mask off
	while (len--) *dest++ = spi_transfer(0);
	gpio_set_level(CONFIG_NSS_GPIO, HIGH);
	return status;
}

uint8_t spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len)
{
	uint8_t status = 0;
	gpio_set_level(CONFIG_NSS_GPIO, LOW);
	status = spi_transfer(reg | RH_DAVIS_RFM69_SPI_WRITE_MASK); // DavisRFM69_send the start address with the write mask on
	while (len--) spi_transfer(*src++);
	gpio_set_level(CONFIG_NSS_GPIO, HIGH);
	return status;
}

void DavisRFM69_setIdleMode(uint8_t idleMode)
{
	_idleMode = idleMode;
}

bool DavisRFM69_init()
{
	//DavisRFM69_reset();
	_idleMode = RF_OPMODE_STANDBY;

	spi_init();

	// Get the device type and check it
	// This also tests whether we are really connected to a device
	// My test devices return 0x24
	_deviceType = spiRead(REG_VERSION);
	ESP_LOGI(TAG, "_deviceType=%x", _deviceType);
	if (_deviceType != 0x24) return false;
#if 0
	if (_deviceType == 00 ||
	_deviceType == 0xff)
	return false;
#endif

	  do {
		  spiWrite(REG_SYNCVALUE1, 0xaa);

		  vTaskDelay(10);
		  //ESP_LOGW(TAG, "REG_SYNCVALUE1: %d",spiRead(REG_SYNCVALUE1) );
		  //vTaskDelay(1000);
	  }while (spiRead(REG_SYNCVALUE1) != 0xaa);
	  do {
		  spiWrite(REG_SYNCVALUE1, 0x55);
		  vTaskDelay(10);
	  }
	  while (spiRead(REG_SYNCVALUE1) != 0x55);

	DavisRFM69_setModeIdle();

	spiWrite(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);
	spiWrite(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_10); // Davis uses Gaussian shaping with BT=0.5
	spiWrite(REG_BITRATEMSB, RF_BITRATEMSB_19200); // Davis uses a datarate of 19.2 KBPS
	spiWrite(REG_BITRATELSB, RF_BITRATELSB_19200);
	spiWrite(REG_FDEVMSB, RF_FDEVMSB_4800); // Davis uses a deviation of 4.8 kHz on Rx
	spiWrite(REG_FDEVLSB, RF_FDEVLSB_4800);
    // 0x07 to 0x09 are REG_FRFMSB to LSB. No sense setting them here. Done in main routine.
	spiWrite(REG_AFCCTRL, RF_AFCCTRL_LOWBETA_OFF); // TODO: Should use LOWBETA_ON, but having trouble getting it working
    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout=-18+OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout=-14+OutputPower (with PA1 and PA2)**
    // +20dBm formula: Pout=-11+OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, //over current protection (default is 95mA)
	spiWrite(REG_LNA, RF_LNA_ZIN_50 | RF_LNA_GAINSELECT_AUTO); // Not sure which is correct!
    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4khz)
	spiWrite(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_4); // Use 25 kHz BW (BitRate < 2 * RxBw)
	spiWrite(REG_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3); // Use double the bandwidth during AFC as reception
    /* 0x1B - 0x1D These registers are for OOK.  Not used */
	spiWrite(REG_AFCFEI, RF_AFCFEI_AFCAUTOCLEAR_ON | RF_AFCFEI_AFCAUTO_ON);
    /* 0x1F & 0x20 AFC MSB and LSB values, respectively */
    /* 0x21 & 0x22 FEI MSB and LSB values, respectively */
    /* 0x23 & 0x24 RSSI MSB and LSB values, respectively */
	spiWrite(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); //DIO0 is the only IRQ we're using
    /* 0x26 RegDioMapping2 */
    /* 0x27 RegIRQFlags1 */
	spiWrite(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN); // Reset the FIFOs. Fixes a problem I had with bad first packet.
	spiWrite(REG_RSSITHRESH, 170); //must be set to dBm = (-Sensitivity / 2) - default is 0xE4=228 so -114dBm
    /* 0x2a & 0x2b RegRxTimeout1 and 2, respectively */
    /* 0x2c RegPreambleMsb - use zero default */
	spiWrite(REG_PREAMBLELSB, 4); // Davis has four preamble bytes 0xAAAAAAAA
	spiWrite(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_2);  // Allow a couple errors in the sync word
	spiWrite(REG_SYNCVALUE1, 0xcb); // Davis ISS first sync byte. http://madscientistlabs.blogspot.ca/2012/03/first-you-get-sugar.html
	spiWrite(REG_SYNCVALUE2, 0x89); // Davis ISS second sync byte.
    /* 0x31 - 0x36  REG_SYNCVALUE3 - 8 not used */
	spiWrite(REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_OFF | RF_PACKET1_CRCAUTOCLEAR_OFF | RF_PACKET1_ADRSFILTERING_OFF); // Fixed packet length and we'll check our own CRC
	spiWrite(REG_PAYLOADLENGTH, RH_DAVIS_PACKET_LEN); // Davis sends 10 bytes of payload, including CRC that we check manually (Note: includes 2 byte re-transmit CRC).
    //* 0x39 */ { REG_NODEADRS, nodeID }, // Turned off because we're not using address filtering
    //* 0x3a */ { REG_BROADCASTADRS, RF_BROADCASTADDRESS_VALUE }, // Not using this
    /* 0x3b REG_AUTOMODES - Automatic modes are not used in this implementation. */
	spiWrite(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFOTHRESH | 0x09); // TX on FIFO having more than nine bytes - we'll implement the re-transmit CRC
	spiWrite(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF); //RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x3e - 0x4d  AES Key not used in this implementation */
	spiWrite(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0); // // TODO: Should use LOWBETA_ON, but having trouble getting it working
	spiWrite(REG_TESTAFC, 0); // AFC Offset for low mod index systems

#if CONFIG_DAVIS_RFM69_POWER_HIGH
	// If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
	// ishighpowermodule flag set like this:
	DavisRFM69_setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
	ESP_LOGW(TAG, "Set TX power high");
#else
	// +13dBm, same as power-on default
	DavisRFM69_setTxPower(13, RH_DAVIS_RFM69_DEFAULT_HIGHPOWER);
#endif


	  spiWrite(REG_OSC1, RF_OSC1_RCCAL_START);
	  while ((spiRead(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);

	return true;
}

void DavisRFM69_setChannel(uint8_t channel)
{
  _channel = (channel % DAVIS_FREQ_TABLE_LENGTH);
  //if (_channel > DAVIS_FREQ_TABLE_LENGTH - 1) _channel = 0;
  DavisRFM69_setModeIdle();

  spiWrite(REG_FRFMSB, FRF[_channel][0]);
  spiWrite(REG_FRFMID, FRF[_channel][1]);
  spiWrite(REG_FRFLSB, FRF[_channel][2]);


  ESP_LOGD(TAG,"REG_FRFMSB: %x", FRF[_channel][0]);
  ESP_LOGD(TAG,"REG_FRFMSB: %x", FRF[_channel][1]);
  ESP_LOGD(TAG,"REG_FRFMSB: %x", FRF[_channel][2]);
  DavisRFM69_receiveBegin();
}

void DavisRFM69_receiveBegin() {
  //_packetReceived = false;
  if (spiRead(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    spiWrite(REG_PACKETCONFIG2, (spiRead(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks

  spiWrite(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); //set DIO0 to "PAYLOADREADY" in receive mode
  DavisRFM69_setModeRx();
}

// Data bytes over the air from the ISS least significant bit first. Fix them as we go. From
// http://www.ocf.berkeley.edu/~wwu/cgi-bin/yabb/YaBB.cgi?board=riddles_cs;action=display;num=1103355188
uint8_t DavisRFM69_reverseBits(uint8_t b)
{
  b = ((b & 0b11110000) >>4 ) | ((b & 0b00001111) << 4);
  b = ((b & 0b11001100) >>2 ) | ((b & 0b00110011) << 2);
  b = ((b & 0b10101010) >>1 ) | ((b & 0b01010101) << 1);

  return(b);
}

void readDavisData(){
	gpio_set_level(CONFIG_NSS_GPIO, LOW);
	spi_transfer(REG_FIFO); // DavisRFM69_send the start address with the write mask off

		for (_bufLen = 0; _bufLen < RH_DAVIS_PACKET_LEN; _bufLen++)
			_buf[_bufLen] = DavisRFM69_reverseBits(spi_transfer(0));
		_rxGood++;
		_rxBufValid = true;
	gpio_set_level(CONFIG_NSS_GPIO, HIGH);
	// Any junk remaining in the FIFO will be cleared next time we go to receive mode.
}

int8_t DavisRFM69_temperatureRead()
{
	// Caution: must be ins standby.
//	  setModeIdle();
	spiWrite(REG_TEMP1, RF_TEMP1_MEAS_START); // Start the measurement
	while (spiRead(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING)
	; // Wait for the measurement to complete
	return 166 - spiRead(REG_TEMP2); // Very approximate, based on observation
}

bool DavisRFM69_setFrequency(float centre)
{
	// Frf = FRF / FSTEP
	uint32_t frf = (uint32_t)((centre * 1000000.0) / RH_DAVIS_RFM69_FSTEP);
	spiWrite(REG_FRFMSB, (frf >> 16) & 0xff);
	spiWrite(REG_FRFMID, (frf >> 8) & 0xff);
	spiWrite(REG_FRFLSB, frf & 0xff);

	// afcPullInRange is not used
	//(void)afcPullInRange;
	return true;
}

int8_t DavisRFM69_rssiRead()
{
	// Force a new value to be measured
	// Hmmm, this hangs forever!
#if 0
	spiWrite(RH_DAVIS_RFM69_REG_23_RSSICONFIG, RH_DAVIS_RFM69_RSSICONFIG_RSSISTART);
	while (!(spiRead(RH_DAVIS_RFM69_REG_23_RSSICONFIG) & RH_DAVIS_RFM69_RSSICONFIG_RSSIDONE))
	;
#endif
	return -((int8_t)(spiRead(REG_RSSIVALUE) >> 1));
}

void DavisRFM69_setOpMode(uint8_t mode)
{
	uint8_t opmode = spiRead(REG_OPMODE);
	opmode &= ~RF_OPMODE_MODE;
	opmode |= (mode & RF_OPMODE_MODE);
	spiWrite(REG_OPMODE, opmode);
	ESP_LOGD(TAG, "setOpMode=%x", opmode);

	// Wait for mode to change.
	while (!(spiRead(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY)) vTaskDelay(1)
	;

	// Verify new mode
	uint8_t _opmode = spiRead(REG_OPMODE);
	if (opmode != _opmode) {
		ESP_LOGE(TAG, "setOpMode fail. %x %x", opmode, _opmode);
	}
}

void DavisRFM69_setModeIdle()
{
	if (_mode != RH_DAVIS_RFM69_ModeIdle)
	{
		if (_power >= 18)
		{
			// If high power boost, return power amp to receive mode
			spiWrite(REG_TESTPA1, RF_TESTPA1_NORMAL);
			spiWrite(REG_TESTPA2, RF_TESTPA2_NORMAL);
		}
		DavisRFM69_setOpMode(_idleMode);
		_mode = RH_DAVIS_RFM69_ModeIdle;
	}
}

bool DavisRFM69_setSleep()
{
	if (_mode != RH_DAVIS_RFM69_ModeSleep)
	{
	spiWrite(REG_OPMODE, RF_OPMODE_SLEEP);
	_mode = RH_DAVIS_RFM69_ModeSleep;
	}
	return true;
}

void DavisRFM69_setModeRx()
{
	if (_mode != RH_DAVIS_RFM69_ModeRx)
	{
	if (_power >= 18)
	{
		// If high power boost, return power amp to receive mode
		spiWrite(REG_TESTPA1, RF_TESTPA1_NORMAL);
		spiWrite(REG_TESTPA2, RF_TESTPA2_NORMAL);
	}
	spiWrite(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // Set interrupt line 0 PayloadReady
	DavisRFM69_setOpMode(RF_OPMODE_RECEIVER); // Clears FIFO
	_mode = RH_DAVIS_RFM69_ModeRx;
	}
}

void DavisRFM69_setModeTx()
{
	if (_mode != RH_DAVIS_RFM69_ModeTx)
	{
	if (_power >= 18)
	{
		// Set high power boost mode
		// Note that OCP defaults to ON so no need to change that.
		spiWrite(REG_TESTPA1, RF_TESTPA1_BOOST);
		spiWrite(REG_TESTPA2, RF_TESTPA2_BOOST);
	}
	spiWrite(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // Set interrupt line 0 PacketSent
	DavisRFM69_setOpMode(RF_OPMODE_TRANSMITTER); // Clears FIFO
	_mode = RH_DAVIS_RFM69_ModeTx;
	}
}

void DavisRFM69_setTxPower(int8_t power, bool ishighpowermodule)
{
  _power = power;
  uint8_t palevel;

  if (ishighpowermodule)
  {
	if (_power < -2)
	  _power = -2; //RFM69HW only works down to -2.
	if (_power <= 13)
	{
	  // -2dBm to +13dBm
	  //Need PA1 exclusivelly on RFM69HW
	  palevel = RF_PALEVEL_PA1_ON | ((_power + 18) &
	  RH_PALEVEL_OUTPUTPOWER);
	}
	else if (_power >= 18)
	{
	  // +18dBm to +20dBm
	  // Need PA1+PA2
	  // Also need PA boost settings change when tx is turned on and off, see DavisRFM69_setModeTx()
	  palevel = RF_PALEVEL_PA1_ON
	| RF_PALEVEL_PA2_ON
	| ((_power + 11) & RH_PALEVEL_OUTPUTPOWER);
	}
	else
	{
	  // +14dBm to +17dBm
	  // Need PA1+PA2
	  palevel = RF_PALEVEL_PA1_ON
	| RF_PALEVEL_PA2_ON
	| ((_power + 14) & RH_PALEVEL_OUTPUTPOWER);
	}
  }
  else
  {
	if (_power < -18) _power = -18;
	if (_power > 13) _power = 13; //limit for RFM69W
	palevel = RF_PALEVEL_PA0_ON
	  | ((_power + 18) & RH_PALEVEL_OUTPUTPOWER);
  }
  spiWrite(REG_PALEVEL, palevel);
}

void DavisRFM69_setPreambleLength(uint16_t bytes)
{
	spiWrite(REG_PREAMBLEMSB, bytes >> 8);
	spiWrite(REG_PREAMBLELSB, bytes & 0xff);
}

void DavisRFM69_setSyncWords(const uint8_t* syncWords, uint8_t len)
{
	uint8_t syncconfig = spiRead(REG_SYNCCONFIG);
	if (syncWords && len && len <= 4)
	{
	spiBurstWrite(REG_SYNCVALUE1, syncWords, len);

#if 0
	uint8_t dest[8];
	spiBurstRead(REG_SYNCVALUE1, dest, 8);
	for (int i=0;i<8;i++) {
		ESP_LOGI(TAG, "REG_SYNCVALUE1[%d]=%x", i, dest[i]);
	}
#endif

	syncconfig |= RF_SYNC_ON;
	}
	else
	syncconfig &= ~RF_SYNC_ON;
	syncconfig &= ~RF_SYNC_SIZE;
	syncconfig |= (len-1) << 3;
	ESP_LOGD(TAG, "syncconfig=%x", syncconfig);
	spiWrite(REG_SYNCCONFIG, syncconfig);
}

bool DavisRFM69_available()
{
	// Get the interrupt cause
	uint8_t irqflags2 = spiRead(REG_IRQFLAGS2);
	ESP_LOGD(TAG, "available irqflags2=%x", irqflags2);
	//if (irqflags2 != 0) ESP_LOGW(TAG, "available irqflags2=%x", irqflags2);
	// Must look for PAYLOADREADY, not CRCOK, since only PAYLOADREADY occurs _after_ AES decryption
	// has been done
	if (irqflags2 & RF_IRQFLAGS2_PAYLOADREADY) {
		// A complete message has been received with good CRC
		_lastRssi = -((int8_t)(spiRead(REG_RSSIVALUE) >> 1));
		_lastPreambleTime = xTaskGetTickCount()*portTICK_PERIOD_MS;

		DavisRFM69_setModeIdle();
		// Save it in our buffer
		//readFifo();
		readDavisData();
		ESP_LOGD(TAG, "PAYLOADREADY");
		//ESP_LOGI(TAG, "_bufLen: %d", _bufLen);
	}
	DavisRFM69_setModeRx(); // Make sure we are receiving
	return _rxBufValid;
}

// Blocks until a valid message is received or timeout expires
// Return true if there is a message available
// Works correctly even on millis() rollover
bool DavisRFM69_waitAvailableTimeout(uint16_t timeout)
{
	unsigned long starttime = xTaskGetTickCount()*portTICK_PERIOD_MS;
	while ((xTaskGetTickCount()*portTICK_PERIOD_MS - starttime) < timeout)
	{
		if (DavisRFM69_available())
	{
		   return true;
	}
	vTaskDelay(1);
	}
	return false;
}


bool DavisRFM69_recv(uint8_t* buf)
{
	if (!DavisRFM69_available())
	return false;

	memcpy(buf, _buf, _bufLen);
	_rxBufValid = false; // Got the most recent message
	//ESP_LOGW("DavisRFM69_recv",": %d (%d)", *len,_bufLen);
	return true;
}

bool DavisRFM69_send(const uint8_t* data, uint8_t len)
{
	if (len > RH_DAVIS_PACKET_LEN)
	return false;

#if 0
	waitPacketSent(); // Make sure we dont interrupt an outgoing message
#endif
	DavisRFM69_setModeIdle(); // Prevent RX while filling the fifo

#if 0
	if (!waitCAD())
	return false;  // Check channel activity
#endif

	ESP_LOGD(TAG, "_txHeaderTo=%d", _txHeaderTo);
	ESP_LOGD(TAG, "_txHeaderFrom=%d", _txHeaderFrom);
	ESP_LOGD(TAG, "_txHeaderId=%d", _txHeaderId);
	ESP_LOGD(TAG, "_txHeaderFlags=%d", _txHeaderFlags);

	gpio_set_level(CONFIG_NSS_GPIO, LOW);
	spi_transfer(REG_FIFO | RH_DAVIS_RFM69_SPI_WRITE_MASK); // DavisRFM69_send the start address with the write mask on
	spi_transfer(len + RH_DAVIS_HEADER_LEN); // Include length of headers

	spi_transfer(_txHeaderTo);
	spi_transfer(_txHeaderFrom);
	spi_transfer(_txHeaderId);
	spi_transfer(_txHeaderFlags);
	// Now the payload
	while (len--)
	spi_transfer(*data++);
	gpio_set_level(CONFIG_NSS_GPIO, HIGH);

	DavisRFM69_setModeTx(); // Start the transmitter
	vTaskDelay(1);
	return true;
}

bool DavisRFM69_waitPacketSent()
{
	while (1) {
		// Get the interrupt cause
		uint8_t irqflags2 = spiRead(REG_IRQFLAGS2);
		ESP_LOGD(TAG, "waitPacketSent irqflags2=%x", irqflags2);
		if (irqflags2 & RF_IRQFLAGS2_PACKETSENT) {
			// A transmitter message has been fully sent
			DavisRFM69_setModeIdle(); // Clears FIFO
			_txGood++;
			ESP_LOGD(TAG, "PACKETSENT");
			break;
		}
		vTaskDelay(1);
	}
	return true;
}


uint8_t DavisRFM69_maxMessageLength()
{
	return RH_DAVIS_PACKET_LEN;
}

bool printRegister(uint8_t reg)
{
	printf("%x %x\n", reg, spiRead(reg));
	return true;
}

bool DavisRFM69_printRegisters()
{
	uint8_t i;
	for (i = 0; i < 0x50; i++)
	printRegister(i);
	// Non-contiguous registers
	printRegister(REG_TESTLNA);
	printRegister(REG_TESTDAGC);
	printRegister(REG_TESTAFC);

	return true;
}

uint8_t DavisRFM69_headerTo()
{
	return _rxHeaderTo;
}

uint8_t DavisRFM69_headerFrom()
{
	return _rxHeaderFrom;
}

uint8_t DavisRFM69_headerId()
{
	return _rxHeaderId;
}

uint8_t DavisRFM69_headerFlags()
{
	return _rxHeaderFlags;
}

int16_t DavisRFM69_lastRssi()
{
	return _lastRssi;
}

uint8_t DavisRFM69_channel(){
	return _channel;
}

uint16_t crc16_ccitt(const uint8_t *buf, uint8_t len, uint16_t initCrc)
{
  uint16_t crc = initCrc;
  while( len-- ) {
    crc ^= *(char *)buf++ << 8;
    for(uint8_t i = 0; i < 8; ++i ) {
      if( crc & 0x8000 )
        crc = (crc << 1) ^ 0x1021;
      else
        crc = crc << 1;
    }
  }
  return crc;
}

bool DavisRFM69_parseData(const uint8_t* data, t_DavisData* davis_data){
	//t_DavisData davis_data;

	//if (strlen((char)data) != RH_DAVIS_PACKET_LEN) return false;
	uint16_t crc;

	davis_data->id = data[0] & 0b00000111;
	davis_data->type = (data[0] & 0b11110000) >> 4;
	davis_data->battery = (data[0] & 0b00001000) >> 3;
	davis_data->wind_speed = data[1];
	davis_data->wind_direction = data[2] * (360/255.0);


	crc = crc16_ccitt(data, 6,0);

	davis_data->crc = ((crc == ((data[6] << 8) | data[7])) && (crc != 0))?  true: false;
	if (davis_data->crc){


	switch(davis_data->type){
		case 4:		// UV Index
			davis_data->active = data[3] == 0xFF?  false: true;
			davis_data->data = ((((data[3] << 8) + data[4]) >> 6) / 50.0);
			//ESP_LOGW(TAG,"UV Index (4): %f", davis_data->data);
			break;

		case 6:		// Solar Radiation
			davis_data->active = data[3] == 0xFF?  false: true;
			davis_data->data = ((((data[3] << 8) + data[4]) >> 6) * 1.757936);
			//ESP_LOGW(TAG,"Solar Radiation (6): %f", davis_data->data);
			break;

		case 8:		// Temperature (ºF)
			davis_data->active = true;
			davis_data->data = ((((((data[3] << 8) | (data[4])) / 160.0)-32)*5)/9);
			//ESP_LOGW(TAG,"Temperature (8): %f", davis_data->data);
			break;

		case 10:	// Humidity (%)
			davis_data->active = true;
			davis_data->data = (((data[4] >> 4) << 8) + data[3]) / 10.0;
			//ESP_LOGW(TAG,"Humidity (10): %f (%02hhX %02hhX)", davis_data->data, data[4], data[3]);
			break;

		case 14:	// Rain
			davis_data->active = true;
			davis_data->data = data[3];
			//ESP_LOGW(TAG," Rain (14): %f", davis_data->data);
			break;

		default:
			davis_data->active = false;
			davis_data->data = -1;
		}

	}

	return davis_data->crc;
}

void DavisRFM69_setStations(const bool new_stations[8]){
	memcpy(stations, new_stations, sizeof(bool) * 8);
}



