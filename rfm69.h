                               // RFM69 register addresses:
#define RFM69_FIFO       0x00  // Rx and Tx FIFO: no auto-increment when reading or writing
#define RFM69_OPMODE     0x01  // operating mode
#define RFM69_DATAMODUL  0x02  // processing mode, modulation
#define RFM69_BITRATEMSB 0x03  // bit rate MSB (or rather bit period)
#define RFM69_BITRATELSB 0x04  // bit rate LSB [Xtal period] 0x140 => 100kbp (Xtal=32MHz)
#define RFM69_FDEVMSB    0x05  // frequency deviation MSB
#define RFM69_FDEVLSB    0x06  // frequency deviation LSB [Xtal/0x80000 = 61Hz @ Xtal=32MHz]
#define RFM69_FRFMSB     0x07  // carrier frequency MSB 0xD90000 => 868.000MHz
#define RFM69_FRFMID     0x08  // carrier frequency MID
#define RFM69_FRFLSB     0x09  // carrier frequency LSB [Xtal/0x80000 = 61Hz @ Xtal=32MHz]
#define RFM69_OSC1       0x0A  // RC oscillator calibration trigger
#define RFM69_AFCCTRL    0x0B  // AFC method
#define RFM69_LOWBAT     0x0C  // reserved or low-battery
#define RFM69_LISTEN1    0x0D  //
#define RFM69_LISTEN2    0x0E
#define RFM69_LISTEN3    0x0F
#define RFM69_VERSION    0x10  // chip version/revision
#define RFM69_PALEVEL    0x11  // enable power amplifier #0
#define RFM69_PARAMP     0x12  // power amplifier rise/fall time
#define RFM69_OCP        0x13  // current limit = 45 + 5 * reg.value
#define RFM69_AGCREF     0x14  //
#define RFM69_AGCTHRESH1 0x15
#define RFM69_AGCTHRESH2 0x16
#define RFM69_AGCTHRESH3 0x17
#define RFM69_LNA        0x18  // LNA input impedance and gain
#define RFM69_RXBW       0x19  // Rx bandwidth
#define RFM69_AFCBW      0x1A  // Rx AFC bandwidth
#define RFM69_OOKPEAK    0x1B  //
#define RFM69_OOKAVG     0x1C
#define RFM69_OOKFIX     0x1D
#define RFM69_AFCFEI     0x1E
#define RFM69_AFCMSB     0x1F
#define RFM69_AFCLSB     0x20
#define RFM69_FEIMSB     0x21
#define RFM69_FEILSB     0x22
#define RFM69_RSSICONFIG 0x23  // RSSI measurement control
#define RFM69_RSSIVALUE  0x24  // RSSI measured value
#define RFM69_DIOMAPPING1 0x25  // I/O mapping
#define RFM69_DIOMAPPING2 0x26  // and clock mapping
#define RFM69_IRQFLAGS1   0x27  // IRQ flags: mode
#define RFM69_IRQFLAGS2   0x28  // IRQ flags: FIFO, packet, CRC
#define RFM69_RSSITHRESH  0x29  // RSSI trigger level
#define RFM69_RXTIMEOUT1  0x2A  //
#define RFM69_RXTIMEOUT2  0x2B
#define RFM69_PREAMBLEMSB 0x2C  // preamble length MSB
#define RFM69_PREAMBLELSB 0x2D  // preamble length LSB
#define RFM69_SYNCCONFIG  0x2E  // SYNC size/tolerance/...
#define RFM69_SYNCVALUE1  0x2F  // SYNC bytes
#define RFM69_SYNCVALUE2  0x30
#define RFM69_SYNCVALUE3  0x31
#define RFM69_SYNCVALUE4  0x32
#define RFM69_SYNCVALUE5  0x33
#define RFM69_SYNCVALUE6  0x34
#define RFM69_SYNCVALUE7  0x35
#define RFM69_SYNCVALUE8  0x36
#define RFM69_PACKETCONFIG1 0x37  // packet config
#define RFM69_PAYLOADLENGTH 0x38  // packet size [bytes]
#define RFM69_NODEADRS      0x39  // node address
#define RFM69_BROADCASTADRS 0x3A  // broadcast address
#define RFM69_AUTOMODES     0x3B  //
#define RFM69_FIFOTHRESH    0x3C  // FIFO thresholds and TxStartCondition (on first byte or on full packet)
#define RFM69_PACKETCONFIG2 0x3D  //
#define RFM69_AESKEY1     0x3E  // AES key
#define RFM69_AESKEY2     0x3F
#define RFM69_AESKEY3     0x40
#define RFM69_AESKEY4     0x41
#define RFM69_AESKEY5     0x42
#define RFM69_AESKEY6     0x43
#define RFM69_AESKEY7     0x44
#define RFM69_AESKEY8     0x45
#define RFM69_AESKEY9     0x46
#define RFM69_AESKEY10    0x47
#define RFM69_AESKEY11    0x48
#define RFM69_AESKEY12    0x49
#define RFM69_AESKEY13    0x4A
#define RFM69_AESKEY14    0x4B
#define RFM69_AESKEY15    0x4C
#define RFM69_AESKEY16    0x4D
#define RFM69_TEMP1       0x4E  // temperature measurement control
#define RFM69_TEMP2       0x4F  // temperature measured value
#define RFM69_TESTLNA     0x58  // Sensitivity boost ?
#define RFM69_TESTPA1     0x5A  // only present on RFM69HW/SX1231H
#define RFM69_TESTPA2     0x5C  // only present on RFM69HW/SX1231H
#define RFM69_TESTDAGC    0x6F  // Fading margin improvement ?
#define RFM69_TESTAFC     0x71

                                       // operating modes:
#define RFM69_OPMODE_SLEEP   0x00
#define RFM69_OPMODE_STDBY   0x04
#define RFM69_OPMODE_SYNTH   0x08
#define RFM69_OPMODE_TX      0x0C
#define RFM69_OPMODE_RX      0x10
#define RFM69_OPMODE_MASK    0x1C

                                       // bits in IrqFlags1 and IrfFlags2
#define RFM69_IRQ_ModeReady     0x8000 // mode change done (between some modes)
#define RFM69_IRQ_RxReady       0x4000
#define RFM69_IRQ_TxReady       0x2000 // 
#define RFM69_IRQ_PllLock       0x1000 // 
#define RFM69_IRQ_Rssi          0x0800
#define RFM69_IRQ_Timeout       0x0400
#define RFM69_IRQ_AutoMode      0x0200
#define RFM69_IRQ_SyncAddrMatch 0x0100

#define RFM69_IRQ_FifoFull      0x0080 // 
#define RFM69_IRQ_FifoNotEmpty  0x0040 // at least one byte in the FIFO
#define RFM69_IRQ_FifoLevel     0x0020 // more bytes than FifoThreshold
#define RFM69_IRQ_FifoOverrun   0x0010 // write this bit to clear the FIFO
#define RFM69_IRQ_PacketSent    0x0008 // packet transmission was completed
#define RFM69_IRQ_PayloadReady  0x0004
#define RFM69_IRQ_CrcOk         0x0002
#define RFM69_IRQ_Unused        0x0001

#include "manchester.h"

class RFM69
{ public:                             // hardware access functions
   void (*Select)(void);              // activate SPI select
   void (*Deselect)(void);            // desactivate SPI select
   uint8_t (*TransferByte)(uint8_t);  // exchange one byte through SPI
   bool (*DIO0_isOn)(void);           // read DIO0 = packet is ready
   bool (*DIO4_isOn)(void);
   void (*RESET_On)(void);            // activate RF chip reset
   void (*RESET_Off)(void);           // desactive RF chip reset

   uint32_t BaseFrequency;            // [32MHz/2^19] base frequency = channel #0
    int32_t FrequencyCorrection;      // [32MHz/2^19] frequency correction (due to Xtal offset)
   uint32_t ChannelSpacing;           // [32MHz/2^19] spacing between channels
    int16_t Channel;                  // [   integer] channel being used rigth now

  private:
   uint8_t WriteByte(uint8_t Byte, uint8_t Addr=0) const   // write Byte
   { Select();
     TransferByte(Addr | 0x80);
     uint8_t Old=TransferByte(Byte);
     Deselect();
     return Old; }

   uint16_t WriteWord(uint16_t Word, uint8_t Addr=0) const  // write Word => two bytes
   { Select();
     TransferByte(Addr | 0x80);
     uint16_t Old=TransferByte(Word>>8);             // upper byte first
     Old = (Old<<8) | TransferByte(Word&0xFF);       // lower byte second
     Deselect();
     return Old; }

   void WriteBytes(const uint8_t *Data, uint8_t Len, uint8_t Addr=0) const
   { Select();
     TransferByte(Addr | 0x80);
     for(uint8_t Idx=0; Idx<Len; Idx++)
     { TransferByte(Data[Idx]); }
     Deselect(); }

   uint8_t ReadByte (uint8_t Addr=0) const
   { Select();
     TransferByte(Addr);
     uint8_t Byte=TransferByte(0);
     Deselect();
     return Byte; }

   uint16_t ReadWord (uint8_t Addr=0) const
   { Select();
     TransferByte(Addr);
     uint16_t Word=TransferByte(0);
     Word = (Word<<8) | TransferByte(0);
     Deselect();
     return Word; }

  public:
   uint32_t WriteFreq(uint32_t Freq) const             // [32MHz/2^19] Set center frequency
   { const uint8_t Addr = RFM69_FRFMSB;
     Select();
     TransferByte(Addr | 0x80);
     uint32_t Old  =  TransferByte(Freq>>16);
     Old = (Old<<8) | TransferByte(Freq>>8);
     Old = (Old<<8) | TransferByte(Freq);
     Deselect();
     return Old; }                                               // return the previously set frequency

   void setChannel(int16_t newChannel)
   { Channel=newChannel; WriteFreq(BaseFrequency+ChannelSpacing*Channel+FrequencyCorrection); }

   void WritePacket(const uint8_t *Data, uint8_t Len=26) const   // write the packet data (26 bytes)
   { const uint8_t Addr=RFM69_FIFO;                              // write to FIFO
     Select();
     TransferByte(Addr | 0x80);
     for(uint8_t Idx=0; Idx<Len; Idx++)
     { uint8_t Byte=Data[Idx];
       TransferByte(ManchesterEncode[Byte>>4]);                  // software manchester encode every byte
       TransferByte(ManchesterEncode[Byte&0x0F]);
     }
     Deselect();
   }

   void ReadPacket(uint8_t *Data, uint8_t *Err, uint8_t Len=26) const // read packet data from FIFO
   { const uint8_t Addr=RFM69_FIFO;
     Select();
     TransferByte(Addr);
     for(uint8_t Idx=0; Idx<Len; Idx++)
     { uint8_t ByteH = 0;
       ByteH = TransferByte(ByteH);
       ByteH = ManchesterDecode[ByteH]; uint8_t ErrH=ByteH>>4; ByteH&=0x0F; // decode manchester, detect (some) errors
       uint8_t ByteL = 0;
       ByteL = TransferByte(ByteL);
       ByteL = ManchesterDecode[ByteL]; uint8_t ErrL=ByteL>>4; ByteL&=0x0F;
       Data[Idx]=(ByteH<<4) | ByteL;
       Err [Idx]=(ErrH <<4) | ErrL ;
     }
     Deselect();
   }

   void WriteSYNC(uint8_t WriteSize, uint8_t SyncTol, const uint8_t *SyncData) const
   { if(SyncTol>7) SyncTol=7;
     if(WriteSize>8) WriteSize=8;
     WriteBytes(SyncData+(8-WriteSize), WriteSize, RFM69_SYNCVALUE1);        // write the SYNC, skip some initial bytes
     WriteByte(  0x80 | ((WriteSize-1)<<3) | SyncTol, RFM69_SYNCCONFIG);     // write SYNC length [bytes]
     WriteWord( 9-WriteSize, RFM69_PREAMBLEMSB); }                           // write preamble length [bytes]

   void    WriteMode(uint8_t Mode=RFM69_OPMODE_STDBY) const { WriteByte(Mode, RFM69_OPMODE); } // SLEEP/STDBY/FSYNTH/TX/RX
   uint8_t ReadMode (void) const { return ReadByte(RFM69_OPMODE); }
   uint8_t ModeReady(void) const { return ReadByte(RFM69_IRQFLAGS1)&0x80; }

   uint16_t ReadIrqFlags(void) const { return ReadWord(RFM69_IRQFLAGS1); }
   void ClearIrqFlags(void)    const { WriteWord(RFM69_IRQ_FifoOverrun | RFM69_IRQ_Rssi, RFM69_IRQFLAGS1); }

   void WriteTxPower_W(int8_t TxPower=10) const // [dBm] for RFM69W: -18..+13dBm
   { if(TxPower<(-18)) TxPower=(-18);           // check limits
     if(TxPower>  13 ) TxPower=  13 ;
     WriteByte(  0x80+(18+TxPower), RFM69_PALEVEL);
     WriteByte(  0x1A             , RFM69_OCP);
     WriteByte(  0x55             , RFM69_TESTPA1);
     WriteByte(  0x70             , RFM69_TESTPA2);
   }

   void WriteTxPower_HW(int8_t TxPower=10) const // [dBm] // for RFM69HW: -14..+20dBm
   { if(TxPower<(-14)) TxPower=(-14);            // check limits
     if(TxPower>  20 ) TxPower=  20 ;
     if(TxPower<=17)
     { WriteByte(  0x60+(14+TxPower), RFM69_PALEVEL);
       WriteByte(  0x1A             , RFM69_OCP);
       WriteByte(  0x55             , RFM69_TESTPA1);
       WriteByte(  0x70             , RFM69_TESTPA2);
     } else
     { WriteByte(  0x60+(11+TxPower), RFM69_PALEVEL);
       WriteByte(  0x0F             , RFM69_OCP);
       WriteByte(  0x5D             , RFM69_TESTPA1);
       WriteByte(  0x7C             , RFM69_TESTPA2);
     }
   }

   void WriteTxPower(int8_t TxPower, uint8_t isHW) const
   { if(isHW) WriteTxPower_HW(TxPower);
         else WriteTxPower_W (TxPower);  }

   void WriteTxPowerMin(void) const { WriteTxPower_W(-18); } // set minimal Tx power and setup for reception

   int Configure(int16_t Channel, const uint8_t *Sync)
   { WriteMode(RFM69_OPMODE_STDBY);          // mode = STDBY
     ClearIrqFlags();
     WriteByte(  0x02, RFM69_DATAMODUL);     // Packet mode, FSK, BT=0.5
     WriteWord(0x0140, RFM69_BITRATEMSB);    // bit rate = 100kbps
     WriteWord(0x0333, RFM69_FDEVMSB);       // FSK deviation = +/-50kHz
     setChannel(Channel);                    // operating channel
     WriteSYNC(8, 7, Sync);                  // SYNC pattern (setup for reception)
     WriteByte(  0x00, RFM69_PACKETCONFIG1); // Fixed size packet, no DC-free encoding, no CRC, no address filtering
     WriteByte(0x80+51, RFM69_FIFOTHRESH);   // TxStartCondition=FifoNotEmpty, FIFO threshold = 51 bytes
     WriteByte(  2*26, RFM69_PAYLOADLENGTH); // Packet size = 26 bytes Manchester encoded into 52 bytes
     WriteByte(  0x02, RFM69_PACKETCONFIG2); // disable encryption (it is permanent between resets !), AutoRxRestartOn=1
     WriteByte(  0x00, RFM69_AUTOMODES);
     WriteTxPowerMin();                      // TxPower (setup for reception)
     WriteByte(  0x08, RFM69_LNA);           // LNA input 50 or 200ohm ?
     WriteByte( 2*114, RFM69_RSSITHRESH);    // RSSI threshold = -114dBm
     WriteByte(  0x4A, RFM69_RXBW);          // +/-100kHz Rx bandwidth => p.27+67
     WriteByte(  0x89, RFM69_AFCBW);         // +/-200kHz Rx bandwidth while AFC
     WriteByte(  0xE4, RFM69_RSSITHRESH);
     WriteWord(0x4047, RFM69_DIOMAPPING1);   // DIO signals: DIO0=01, DIO4=01, ClkOut=OFF
                                             // RX: DIO0 = PayloadReady, DIO4 = Rssi
                                             // TX: DIO0 = TxReady,      DIO4 = TxReady
     WriteByte(  0x2D, RFM69_TESTLNA);       // enable LNA, sensitivity up by 3dB ?
     WriteByte(  0x20, RFM69_TESTDAGC);      // 0x20 when AfcLowBetaOn, 0x30 otherwise-> page 25
     WriteByte(  0x00, RFM69_AFCFEI);        // AfcAutoOn=0, AfcAutoclearOn=0
     WriteByte(  0x20, RFM69_AFCCTRL);       // AfcLowBetaOn=1 -> page 64 -> page 33
     WriteByte(   +10, RFM69_TESTAFC);       // [488Hz] if AfcLowBetaOn
     return 0; }

     uint8_t ReadVersion(void) const { return ReadByte(RFM69_VERSION); }           // normally returns: 0x24

     void    TriggerRSSI(void) const { WriteByte(0x01, RFM69_RSSICONFIG); }        // trigger measurement
     uint8_t ReadyRSSI(void)   const { return ReadByte(RFM69_RSSICONFIG) & 0x02; } // ready ?
     uint8_t ReadRSSI(void)    const { return ReadByte(RFM69_RSSIVALUE); }         // read value: RSS = -Value/2

     void    TriggerTemp(void) const { WriteByte(0x08, RFM69_TEMP1); }             // trigger measurement
     uint8_t RunningTemp(void) const { return ReadByte(RFM69_TEMP1) & 0x04; }      // still running ?
     uint8_t ReadTemp(void)    const { return ReadByte(RFM69_TEMP2); }             // read value: -1 deg/LSB

} ;


