#ifndef __UBX_H__
#define __UBX_H__

// UBX Class packet numbers
const uint8_t UBX_NAV = 0x01; // navigation
const uint8_t UBX_ACK = 0x05; // acknoledgement of configuration
const uint8_t UBX_CFG = 0x06; // configuration

class UBX_RxMsg // receiver for the UBX sentences
{ public:
   // most information in the UBX packets is already aligned to 32-bit boundary
   // thus it makes sense to have the packet so aligned when receiving it.
   static const uint8_t MaxWords=10;   // maximum number of 32-bit words (excl. head and tail)
   static const uint8_t MaxBytes=4*MaxWords; // max. number of bytes
   static const uint8_t SyncL=0xB5;    // UBX sync bytes
   static const uint8_t SyncH=0x62;

   uint32_t Word[MaxWords];            // here we store the UBX packet (excl. head and tail)
   uint8_t  Byte(uint8_t ByteIdx) const { return ((uint8_t*)Word)[ByteIdx]; } // get out given packet byte
   uint8_t  Class;                     // Class (01=NAV)
   uint8_t  ID;                        // ID
   uint8_t  Bytes;                     // number of bytes in the packet (excl. head and tail)

  private:
   uint8_t Padding;                  // just to make the structure size be a multiple of 4-bytes
   uint8_t State;                    // bits: 0:loading, 1:complete, 2:locked,
   uint8_t Idx;                      // loading index

   uint8_t CheckA;                   // UBX check sum (two bytes)
   uint8_t CheckB;
   void CheckPass(uint8_t Byte) { CheckA+=Byte; CheckB+=CheckA; } // pass a byte through the checksum

  public:
   inline void Clear(void) { Idx=0; State=0; CheckA=0; CheckB=0; }

   uint8_t isLoading(void) const
     { return State&0x01; }

   uint8_t isComplete(void) const
     { return State&0x02; }

   void ProcessByte(uint8_t Byte) // pass all bytes through this call and it will build the frame
     { 
       if(isComplete()) Clear(); // if already a complete frame, clear it
       switch(Idx)
       { case 0:  // expect SyncL
            if(Byte!=SyncL) { Clear(); return; }
            State=0x01; break;  // declare "isLoading" state
         case 1: // expect SyncH
            if(Byte!=SyncH) { Clear(); return; }
            break;
         case 2: // Class
            Class=Byte; CheckPass(Byte);
            break;
         case 3: // ID
            ID=Byte; CheckPass(Byte);
            break;
         case 4: // LSB of packet length
            Bytes=Byte; CheckPass(Byte); if(Bytes>MaxBytes) { Clear(); return; }
            break;
         case 5: // MSB of packet length (expect zero)
            CheckPass(Byte); if(Byte!=0) { Clear(); return; }
            break;
         default:                         // past the header, now load the packet content
            uint8_t ByteIdx=Idx-6;
            if(ByteIdx<Bytes)
            { ((uint8_t *)Word)[ByteIdx]=Byte; CheckPass(Byte); }
            else if(ByteIdx==Bytes)        // already past the content, now the first checksum byte
            { if(Byte!=CheckA) { Clear(); return; } }
            else if(ByteIdx==(Bytes+1))    // second checksum byte
            { if(Byte!=CheckB) { Clear(); return; }
              State=0x02; }                // declare "isComplete" state
            else
            { Clear(); return; }
            break;
       }
       Idx++;
     }

} ;

class UBX_NAV_STATUS
{ uint32_t iTOW;     // [ms] Time-of-Week
  uint8_t  gpsFix;   // Fix type: 0:none, 1=dead reckoning, 2:2-D, 3:3-D, 4:GPS+dead reckoning, 5:time-only
  uint8_t  flags;    // xxxxTWDF => T:Time-of-Week is valid, W:Week-Number is valid, D:Diff. GPS is used, F:Fix valid
  uint8_t  diffStat; // DD => 00:none, 01:PR+PRR corr., 10: PR+PRR+CP corr., 11: high accuracy PR+PRR+CP
  uint8_t  res;      // reserved          PR=Pseudo-Range, PRR=Pseudo-Range Rate
  uint32_t ttff;     // [ms] Time To First Fix
  uint32_t msss;     // [ms] Since Startup
} ;

class UBX_NAV_DOP
{ uint32_t iTOW;      // [ms] Time-of-Week
  uint16_t gDOP;      // [1/100] geometrical
  uint16_t pDOP;      // [1/100] position
  uint16_t tDOP;      // [1/100] time
  uint16_t vDOP;      // [1/100] vertical
  uint16_t hDOP;      // [1/100] horizontal
  uint16_t nDOP;      // [1/100] north-south
  uint16_t eDOP;      // [1/100] east-west
  uint16_t padding;   // padding for round size
} ;

class UBX_NAV_POSLLH // Position: Latitude/Longitude/Height
{ uint32_t iTOW;      // [ms] Time-of-Week
   int32_t lon;       // [1e-7 deg]
   int32_t lat;       // [1e-7 deg]
   int32_t height;    // [mm] height above elipsoid (GPS altitude)
   int32_t hMSL;      // [mm] height above Mean Sea Level
  uint32_t hAcc;      // [mm] horizontal accuracy
  uint32_t vAcc;      // [mm] vertical accuracy
} ;

class UBX_NAV_VELNED // Velocity: North/East/Down
{ uint32_t iTOW;      // [ms] Time-of-Week
   int32_t velN;      // [cm/s] velocity North
   int32_t velE;      // [cm/s] velocity East
   int32_t velD;      // [cm/s] velocity Down
  uint32_t Speed;     // [cm/s] velocity
  uint32_t gSpeed;    // [cm/s] ground speed (horizontal velocity)
   int32_t heading;   // [1e-5 deg] ground heading
  uint32_t sAcc;      // [cm/s] speed accuracy
  uint32_t cAcc;      // [1e-5 deg] heading accuracy
} ;

class UBX_NAV_TIMEGPS
{ public:
   uint32_t iTOW;      // [ms] Time-of-Week
    int32_t fTOW;      // [ns] reminder of Time-of-Week
   uint16_t week;
   uint8_t  leapS;
   uint8_t  valid;     // bits: 0:ToW, 1:week, 2:leapS
   uint32_t tAcc;      // [ns]

  public:
   static const uint32_t SecsPerWeek = 7*24*60*60;
   uint8_t Valid(void) const
   { return (valid&0x03)==0x03; }
   uint32_t UnixTime() const
   { return (iTOW+10)/1000 + week*SecsPerWeek + 315964785; } // http://www.andrews.edu/~tzs/timeconv/timedisplay.php
} ;

class UBX_NAV_TIMEUTC // Time in UTC
{ public:
   uint32_t iTOW;      // [ms] Time-of-Week
   uint32_t tAcc;      // [ns]
    int32_t nano;      // [ns]
   uint16_t year;
   uint8_t  month;
   uint8_t  day;
   uint8_t  hour;
   uint8_t  min;
   uint8_t  sec;
   uint8_t  valid;    // bits: 0:ToW, 1:WN, 2:UTC
} ;

#endif // __UBX_H__
