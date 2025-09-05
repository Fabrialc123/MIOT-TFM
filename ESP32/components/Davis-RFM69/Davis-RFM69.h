#ifndef RH_DAVIS_RFM69_h
#define RH_DAVIS_RFM69_h

#include "RFM69_Registers.h"
//#include "Davis_Config.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"

typedef struct DavisData{
	uint8_t type;			// Byte 0: 	0bXXXX0000
	bool 	battery;		// Byte 0:	0b0000X000
	uint8_t id;				// Byte 0:	0b00000XXX
	uint8_t wind_speed;		// Byte 1
	uint8_t wind_direction;	// Byte 2
	bool 	active;			// Byte 3: FALSE if 0xFF and type is 4 (UVIndex),  6 (SolarRad), otherwise is TRUE
	float 	data;			// Bytes 3-5: Depends of type
	bool 	crc;			// Bytes 6-7: TRUE if valid packet
} t_DavisData;


// The crystal oscillator frequency of the RF69 module
#define RH_DAVIS_RFM69_FXOSC 32000000.0

// The Frequency Synthesizer step = RH_DAVIS_RFM69_FXOSC / 2^^19
#define RH_DAVIS_RFM69_FSTEP  (RH_DAVIS_RFM69_FXOSC / 524288)

// This is the maximum number of interrupts the driver can support
// Most Arduinos can handle 2, Megas can handle more
#define RH_DAVIS_RFM69_NUM_INTERRUPTS 3

// This is the bit in the SPI address that marks it as a write
#define RH_DAVIS_RFM69_SPI_WRITE_MASK 0x80

// Max number of octets the RH_DAVIS_RFM69 Rx and Tx FIFOs can hold
#define RH_DAVIS_RFM69_FIFO_SIZE 66

// Maximum encryptable payload length the RF69 can support
#define RH_DAVIS_RFM69_MAX_ENCRYPTABLE_PAYLOAD_LEN 10

// Keep track of the mode the RF69 is in
#define RH_DAVIS_RFM69_MODE_IDLE         0
#define RH_DAVIS_RFM69_MODE_RX           1
#define RH_DAVIS_RFM69_MODE_TX           2

// This is the default node address,
#define RH_DAVIS_RFM69_DEFAULT_NODE_ADDRESS 0

// You can define the following macro (either by editing here or by passing it as a compiler definition
// to change the default value of the ishighpowermodule argument to setTxPower to true
// 
// #define RFM69_HW
#ifdef RH_DAVIS_RFM69_HW
#define RH_DAVIS_RFM69_DEFAULT_HIGHPOWER true
#else
#define RH_DAVIS_RFM69_DEFAULT_HIGHPOWER false
#endif

    /// \brief Defines register values for a set of modem configuration registers
    ///
    /// Defines register values for a set of modem configuration registers
    /// that can be passed to setModemRegisters() if none of the choices in
    /// ModemConfigChoice suit your need setModemRegisters() writes the
    /// register values from this structure to the appropriate RF69 registers
    /// to set the desired modulation type, data rate and deviation/bandwidth.
    typedef struct
    {
	uint8_t    reg_02;   ///< Value for register RH_DAVIS_RFM69_REG_02_DATAMODUL
	uint8_t    reg_03;   ///< Value for register RH_DAVIS_RFM69_REG_03_BITRATEMSB
	uint8_t    reg_04;   ///< Value for register RH_DAVIS_RFM69_REG_04_BITRATELSB
	uint8_t    reg_05;   ///< Value for register RH_DAVIS_RFM69_REG_05_FDEVMSB
	uint8_t    reg_06;   ///< Value for register RH_DAVIS_RFM69_REG_06_FDEVLSB
	uint8_t    reg_19;   ///< Value for register RH_DAVIS_RFM69_REG_19_RXBW
	uint8_t    reg_1a;   ///< Value for register RH_DAVIS_RFM69_REG_1A_AFCBW
	uint8_t    reg_37;   ///< Value for register RH_DAVIS_RFM69_REG_37_PACKETCONFIG1
    } RH_DAVIS_RFM69_ModemConfig;
  
    /// Choices for setModemConfig() for a selected subset of common
    /// modulation types, and data rates. If you need another configuration,
    /// use the register calculator.  and call setModemRegisters() with your
    /// desired settings.  
    /// These are indexes into MODEM_CONFIG_TABLE. We strongly recommend you use these symbolic
    /// definitions and not their integer equivalents: its possible that new values will be
    /// introduced in later versions (though we will try to avoid it).
    /// CAUTION: some of these configurations do not work corectly and are marked as such.
    typedef enum
    {
	FSK_Rb2Fd5 = 0,	   ///< FSK, Whitening, Rb = 2kbs,    Fd = 5kHz
	FSK_Rb2_4Fd4_8,    ///< FSK, Whitening, Rb = 2.4kbs,  Fd = 4.8kHz 
	FSK_Rb4_8Fd9_6,    ///< FSK, Whitening, Rb = 4.8kbs,  Fd = 9.6kHz 
	FSK_Rb9_6Fd19_2,   ///< FSK, Whitening, Rb = 9.6kbs,  Fd = 19.2kHz
	FSK_Rb19_2Fd38_4,  ///< FSK, Whitening, Rb = 19.2kbs, Fd = 38.4kHz
	FSK_Rb38_4Fd76_8,  ///< FSK, Whitening, Rb = 38.4kbs, Fd = 76.8kHz
	FSK_Rb57_6Fd120,   ///< FSK, Whitening, Rb = 57.6kbs, Fd = 120kHz
	FSK_Rb125Fd125,    ///< FSK, Whitening, Rb = 125kbs,  Fd = 125kHz
	FSK_Rb250Fd250,    ///< FSK, Whitening, Rb = 250kbs,  Fd = 250kHz
	FSK_Rb55555Fd50,   ///< FSK, Whitening, Rb = 55555kbs,Fd = 50kHz for RFM69 lib compatibility

	GFSK_Rb2Fd5,	    ///< GFSK, Whitening, Rb = 2kbs,    Fd = 5kHz
	GFSK_Rb2_4Fd4_8,    ///< GFSK, Whitening, Rb = 2.4kbs,  Fd = 4.8kHz
	GFSK_Rb4_8Fd9_6,    ///< GFSK, Whitening, Rb = 4.8kbs,  Fd = 9.6kHz
	GFSK_Rb9_6Fd19_2,   ///< GFSK, Whitening, Rb = 9.6kbs,  Fd = 19.2kHz
	GFSK_Rb19_2Fd38_4,  ///< GFSK, Whitening, Rb = 19.2kbs, Fd = 38.4kHz
	GFSK_Rb38_4Fd76_8,  ///< GFSK, Whitening, Rb = 38.4kbs, Fd = 76.8kHz
	GFSK_Rb57_6Fd120,   ///< GFSK, Whitening, Rb = 57.6kbs, Fd = 120kHz
	GFSK_Rb125Fd125,    ///< GFSK, Whitening, Rb = 125kbs,  Fd = 125kHz
	GFSK_Rb250Fd250,    ///< GFSK, Whitening, Rb = 250kbs,  Fd = 250kHz
	GFSK_Rb55555Fd50,   ///< GFSK, Whitening, Rb = 55555kbs,Fd = 50kHz

	OOK_Rb1Bw1,         ///< OOK, Whitening, Rb = 1kbs,    Rx Bandwidth = 1kHz. 
	OOK_Rb1_2Bw75,      ///< OOK, Whitening, Rb = 1.2kbs,  Rx Bandwidth = 75kHz. 
	OOK_Rb2_4Bw4_8,     ///< OOK, Whitening, Rb = 2.4kbs,  Rx Bandwidth = 4.8kHz. 
	OOK_Rb4_8Bw9_6,     ///< OOK, Whitening, Rb = 4.8kbs,  Rx Bandwidth = 9.6kHz. 
	OOK_Rb9_6Bw19_2,    ///< OOK, Whitening, Rb = 9.6kbs,  Rx Bandwidth = 19.2kHz. 
	OOK_Rb19_2Bw38_4,   ///< OOK, Whitening, Rb = 19.2kbs, Rx Bandwidth = 38.4kHz. 
	OOK_Rb32Bw64,       ///< OOK, Whitening, Rb = 32kbs,   Rx Bandwidth = 64kHz. 

//	Test,
    } RH_DAVIS_RFM69_ModemConfigChoice;


    /// \brief Defines different operating modes for the transport hardware
    ///
    /// These are the different values that can be adopted by the _mode variable and 
    /// returned by the mode() member function,
    typedef enum
    {
	RH_DAVIS_RFM69_ModeInitialising = 0, ///< Transport is initialising. Initial default value until init() is called..
	RH_DAVIS_RFM69_ModeSleep,            ///< Transport hardware is in low power sleep mode (if supported)
	RH_DAVIS_RFM69_ModeIdle,             ///< Transport is idle.
	RH_DAVIS_RFM69_ModeTx,               ///< Transport is in the process of transmitting a message.
	RH_DAVIS_RFM69_ModeRx,               ///< Transport is in the process of receiving a message.
	RH_DAVIS_RFM69_ModeCad               ///< Transport is in the process of detecting channel activity (if supported)
    } RH_DAVIS_RFM69_Mode;

    bool DavisRFM69_init();

    void DavisRFM69_setChannel(uint8_t channel);

    void DavisRFM69_receiveBegin();

    /// Reads the on-chip temperature sensor.
    /// The RF69 must be in Idle mode (= RF69 Standby) to measure temperature.
    /// The measurement is uncalibrated and without calibration, you can expect it to be far from
    /// correct.
    /// \return The measured temperature, in degrees C from -40 to 85 (uncalibrated)
    int8_t        DavisRFM69_temperatureRead();

    /// Sets the transmitter and receiver 
    /// centre frequency
    /// \param[in] centre Frequency in MHz. 240.0 to 960.0. Caution, RF69 comes in several
    /// different frequency ranges, and setting a frequency outside that range of your radio will probably not work
    /// \param[in] afcPullInRange Not used
    /// \return true if the selected frquency centre is within range
    bool        DavisRFM69_setFrequency(float centre);

    /// Reads and returns the current RSSI value. 
    /// Causes the current signal strength to be measured and returned
    /// If you want to find the RSSI
    /// of the last received message, use lastRssi() instead.
    /// \return The current RSSI value on units of 0.5dB.
    int8_t        DavisRFM69_rssiRead();

    /// Sets the parameters for the RF69 OPMODE.
    /// This is a low level device access function, and should not normally ned to be used by user code. 
    /// Instead can use stModeRx(), setModeTx(), setModeIdle()
    /// \param[in] mode RF69 OPMODE to set, one of RH_DAVIS_RFM69_OPMODE_MODE_*.
    void           DavisRFM69_setOpMode(uint8_t mode);

    /// If current mode is Rx or Tx changes it to Idle. If the transmitter or receiver is running, 
    /// disables them.
    void           DavisRFM69_setModeIdle();

    /// If current mode is Tx or Idle, changes it to Rx. 
    /// Starts the receiver in the RF69.
    void           DavisRFM69_setModeRx();

    /// If current mode is Rx or Idle, changes it to Rx. F
    /// Starts the transmitter in the RF69.
    void           DavisRFM69_setModeTx();

    /// Sets the transmitter power output level.
    /// Be a good neighbour and set the lowest power level you need.
    /// Caution: legal power limits may apply in certain countries.
    /// After init(), the power will be set to 13dBm for a low power module.
    /// If you are using a high p[ower modfule such as an RFM69HW, you MUST set the power level
    /// with the ishighpowermodule flag set to true. Else you wil get no measurable power output.
    /// Simlarly if you are not using a high power module, you must NOT set the ishighpowermodule
    /// (which is the default)
    /// \param[in] power Transmitter power level in dBm. For RF69W (ishighpowermodule = false),
    /// valid values are from -18 to +13.; Values outside this range are trimmed.
    /// For RF69HW (ishighpowermodule = true), valid values are from -2 to +20.
    /// Caution: at +20dBm, duty cycle is limited to 1% and a 
    /// maximum VSWR of 3:1 at the antenna port.
    /// \param ishighpowermodule Set to true if the connected module is a high power module RFM69HW
    void           DavisRFM69_setTxPower(int8_t power, bool ishighpowermodule);

    /// Sets all the registers required to configure the data modem in the RF69, including the data rate, 
    /// bandwidths etc. You can use this to configure the modem with custom configurations if none of the 
    /// canned configurations in ModemConfigChoice suit you.
    /// \param[in] config A ModemConfig structure containing values for the modem configuration registers.
    void           DavisRFM69_setModemRegisters(const RH_DAVIS_RFM69_ModemConfig* config);

    /// Select one of the predefined modem configurations. If you need a modem configuration not provided 
    /// here, use setModemRegisters() with your own ModemConfig. The default after init() is RH_DAVIS_RFM69::GFSK_Rb250Fd250.
    /// \param[in] index The configuration choice.
    /// \return true if index is a valid choice.
    bool        DavisRFM69_setModemConfig(RH_DAVIS_RFM69_ModemConfigChoice index);

    /// Starts the receiver and checks whether a received message is available.
    /// This can be called multiple times in a timeout loop
    /// \return true if a complete, valid message has been received and is able to be retrieved by
    /// recv()
    bool        DavisRFM69_available();

    /// Starts the receiver and blocks until a received message is available or a timeout
    /// \param[in] timeout Maximum time to wait in milliseconds.
    /// \return true if a message is available
    bool        DavisRFM69_waitAvailableTimeout(uint16_t timeout);

    /// Turns the receiver on if it not already on.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to available space in buf. Set to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    bool        DavisRFM69_recv(uint8_t* buf);

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is NOT permitted. 
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send (> 0)
    /// \return true if the message length was valid and it was correctly queued for transmit
    bool        DavisRFM69_send(const uint8_t* data, uint8_t len);

    /// Blocks until the transmitter 
    /// is no longer transmitting.
    bool        DavisRFM69_waitPacketSent();

    /// Sets the length of the preamble
    /// in bytes. 
    /// Caution: this should be set to the same 
    /// value on all nodes in your network. Default is 4.
    /// Sets the message preamble length in REG_0?_PREAMBLE?SB
    /// \param[in] bytes Preamble length in bytes.  
    void           DavisRFM69_setPreambleLength(uint16_t bytes);

    /// Sets the sync words for transmit and receive 
    /// Caution: SyncWords should be set to the same 
    /// value on all nodes in your network. Nodes with different SyncWords set will never receive
    /// each others messages, so different SyncWords can be used to isolate different
    /// networks from each other. Default is { 0x2d, 0xd4 }.
    /// Caution: tests here show that with a single sync word (ie where len == 1), 
    /// RFM69 reception can be unreliable.
    /// \param[in] syncWords Array of sync words, 1 to 4 octets long. NULL if no sync words to be used.
    /// \param[in] len Number of sync words to set, 1 to 4. 0 if no sync words to be used.
    void           DavisRFM69_setSyncWords(const uint8_t* syncWords, uint8_t len);

    /// Enables AES encryption and sets the AES encryption key, used
    /// to encrypt and decrypt all messages. The default is disabled.
    /// \param[in] key The key to use. Must be 16 bytes long. The same key must be installed
    /// in other instances of RF69, otherwise communications will not work correctly. If key is NULL,
    /// encryption is disabled, which is the default.
    void           DavisRFM69_setEncryptionKey(uint8_t* key);

    /// Returns the time in millis since the most recent preamble was received, and when the most recent
    /// RSSI measurement was made.
    uint32_t DavisRFM69_getLastPreambleTime();

    /// The maximum message length supported by this driver
    /// \return The maximum message length supported by this driver
    uint8_t DavisRFM69_maxMessageLength();

    /// Prints the value of a single register
    /// to the Serial device if RH_HAVE_SERIAL is defined for the current platform
    /// For debugging/testing only
    /// \return true if successful
    bool DavisRFM69_printRegister(uint8_t reg);

    /// Prints the value of all the RF69 registers
    /// to the Serial device if RH_HAVE_SERIAL is defined for the current platform
    /// For debugging/testing only
    /// \return true if successful
    bool DavisRFM69_printRegisters();

    /// Sets the radio operating mode for the case when the driver is idle (ie not
    /// transmitting or receiving), allowing you to control the idle mode power requirements
    /// at the expense of slower transitions to transmit and receive modes.
    /// By default, the idle mode is RH_DAVIS_RFM69_OPMODE_MODE_STDBY,
    /// but eg setIdleMode(RH_DAVIS_RFM69_OPMODE_MODE_SLEEP) will provide a much lower
    /// idle current but slower transitions. Call this function after init().
    /// \param[in] idleMode The chip operating mode to use when the driver is idle. One of RH_DAVIS_RFM69_OPMODE_*
    void DavisRFM69_setIdleMode(uint8_t idleMode);

    /// Sets the radio into low-power sleep mode.
    /// If successful, the transport will stay in sleep mode until woken by 
    /// changing mode it idle, transmit or receive (eg by calling send(), recv(), available() etc)
    /// Caution: there is a time penalty as the radio takes a finite time to wake from sleep mode.
    /// \return true if sleep mode was successfully entered.
    bool    DavisRFM69_setSleep();

    /// Return the integer value of the device type
    /// as read from the device in from RH_DAVIS_RFM69_REG_10_VERSION.
    /// Expect 0x24, depending on the type of device actually
    /// connected.
    /// \return The integer device type
    //uint16_t deviceType() {return _deviceType;};

    /// This is a low level function to handle the interrupts for one instance of RF69.
    /// Called automatically by isr*()
    /// Should not need to be called by user code.
    //void           handleInterrupt();

    /// Low level function to read the FIFO and put the received data into the receive buffer
    /// Should not need to be called by user code.
    void           DavisRFM69_readFifo();

    /// Low level interrupt service routine for RF69 connected to interrupt 0
    //static void         isr0();

    /// Low level interrupt service routine for RF69 connected to interrupt 1
    //static void         isr1();

    /// Low level interrupt service routine for RF69 connected to interrupt 1
    //static void         isr2();

    uint8_t DavisRFM69_headerTo();

    uint8_t DavisRFM69_headerFrom();

    uint8_t DavisRFM69_headerId();

    uint8_t DavisRFM69_headerFlags();

    int16_t DavisRFM69_lastRssi();

    uint8_t DavisRFM69_channel();

    void DavisRFM69_setChannel(uint8_t channel);

    bool DavisRFM69_parseData(const uint8_t* data, t_DavisData* davis_data);

    void DavisRFM69_hop_station();

    void DavisRFM69_reset();

    int8_t DavisRFM69_listening_station();

    void DavisRFM69_setStations(const bool new_stations[8]);

#endif

