#ifndef __OGN_H__
#define __OGN_H__

#include <stdio.h>

#include <string.h>
#include <stdint.h>
#ifndef __AVR__
#include <time.h>
#endif

#include <math.h>

#include "bitcount.h"
#include "nmea.h"

#include "ldpc.h"

#include "format.h"

                          // the packet description here is how it look on the little-endian CPU before sending it to the RF chip
                          // nRF905, CC1101, SPIRIT1, RFM69 chips actually reverse the bit order within every byte
                          // thus on the air the bits appear MSbit first for every byte transmitted

class OGN_Packet          // Packet structure for the OGN tracker
{ public:

  union
  { uint32_t Word[7];     // OGN packet as 32-bit words
    uint8_t  Byte[26];    // OGN packet as  8-bit bytes

    struct                // OGN packet as Header+Position+FEC
    { uint32_t Header;     //    ECRR PMTT AAAA AAAA AAAA AAAA AAAA AAAA
                           // E=Emergency, C=enCrypt/Custom, RR=Relay count, P=Parity, M=isMeteo/Telemetry, TT=address Type, AA..=Address:24-bit
                           // When enCrypt/Custom is set the data (position or whatever) can only be decoded by the owner
                           // This option is indented to pass any type of custom data not foreseen otheriwse

      uint32_t Position[4];// 0: QQTT TTTT LLLL LLLL LLLL LLLL LLLL LLLL  QQ=fix Quality:2, TTTTTT=time:6, LL..=Latitude:20
                           // 1: MBDD DDDD LLLL LLLL LLLL LLLL LLLL LLLL  F=fixMode:1 B=isBaro:1, DDDDDD=DOP:6, LL..=Longitude:20
                           // 2: RRRR RRRR SSSS SSSS SSAA AAAA AAAA AAAA  RR..=turn Rate:8, SS..=Speed:10, AA..=Alt:14
                           // 3: BBBB BBBB YYYY PCCC CCCC CCDD DDDD DDDD  BB..=Baro altitude:8, YYYY=AcftType:4, P=Stealth:1, CC..=Climb:9, DD..=Heading:10

                           // meteo/telemetry types: Meteo conditions, Thermal wind/climb, Device telemetry, Precise time, 

                           // meteo report would transmit: Humidity, Barometric pressure, Temperature, wind Speed/Direction                          
                           // 2: HHHH HHHH SSSS SSSS SSAA AAAA AAAA AAAA
                           // 3: TTTT TTTT YYYY BBBB BBBB BBDD DDDD DDDD  YYYY = report tYpe (meteo, thermal, water level, other telemetry)

      uint32_t FEC[2];     // Gallager code: 48 check bits for 160 user bits
    } ;
  } ;

   uint8_t State;         // 
   uint8_t RxRSSI;        // [-0.5dBm]
   // int8_t RxFreqOfs;     // not used for now
   uint8_t RxErr;         // number of bit errors corrected upon reception
   uint8_t Rank;          // rank: low altitude and weak signal => high rank

/*                        // for relay decision:
    int16_t AltDelta;     // [m] altitude difference
   uint16_t Distance;     // [m] distance
*/
  public:

   void   clrReady(void)       {        State &= 0xFE; } // is ready for transmission
   void   setReady(void)       {        State |= 0x01; }
   uint8_t isReady(void) const { return State &  0x01; }

   void   clrSent(void)        {        State &= 0xFD; } // has already been transmitted out
   void   setSent(void)        {        State |= 0x02; }
   uint8_t isSent(void)  const { return State &  0x02; }

   void   clrAlloc(void)       {        State &= 0x7F; } // allocated = being used (when in a pipe)
   void   setAlloc(void)       {        State |= 0x80; }
   uint8_t isAlloc(void) const { return State &  0x80; }

   void recvBytes(const uint8_t *Packet) { memcpy(Byte, Packet, 26); } // load data bytes e.g. from a demodulator

#ifdef __AVR__

#endif

#ifndef __AVR__
   uint8_t calcErrorPattern(uint8_t *ErrPatt, const uint8_t *Packet)
   { uint8_t ByteIdx=0; uint32_t *WordPtr=&Header;
     for(uint8_t WordIdx=0; WordIdx<7; WordIdx++)
     { uint32_t Word=WordPtr[WordIdx];
       for(int Idx=0; Idx<4; Idx++)
       { if(ByteIdx>=26) break;
         ErrPatt[ByteIdx]=Packet[ByteIdx]^Word; ByteIdx++;
         Word>>=8; }
     }
     return 26; }

   void Dump(void) const
   { printf("%08lX: %08lX %08lX %08lX %08lX [%08lX %04lX] (%d)\n",
             (long int)Header, (long int)Position[0], (long int)Position[1],
             (long int)Position[2], (long int)Position[3], (long int)FEC[0],
             (long int)FEC[1], (int)checkFEC() ); }

   void DumpBytes(void) const
   { for(uint8_t Idx=0; Idx<26; Idx++)
     { printf(" %02X", Byte[Idx]); }
     printf(" (%d)\n", LDPC_Check(Byte)); }

   void Print(void) const
   { printf("%06lX:%c R%c %c%X %c",
            (long int)getAddress(), '0'+getAddrType(), '0'+getRelayCount(),
	    isStealth()?'s':' ', (int)getAcftType(), isEmergency()?'E':' ');
     printf("%d/%dD/%4.1f %02dsec: [%+10.6f, %+10.6f]deg %ldm %3.1fkt %05.1fdeg %+4.1fm/s %+4.1fdeg/s\n",
            (int)getFixQuality(), (int)getFixMode()+2, 0.1*(10+DecodeDOP()), (int)getTime(),
            0.0001/60*DecodeLatitude(), 0.0001/60*DecodeLongitude(), (long int)DecodeAltitude(),
            0.2*DecodeSpeed(), 0.1*DecodeHeading(), 0.1*DecodeClimbRate(), 0.1*DecodeTurnRate() ); }

#endif // __AVR__

   uint8_t WriteNMEA(char *NMEA)
   { uint8_t Len=0;
     Len+=Format_String(NMEA+Len, "$POGNT,");                      // sentence name
     Len+=Format_UnsDec(NMEA+Len, (uint16_t)getTime(), 2);         // [sec] time
     NMEA[Len++]=',';
     NMEA[Len++]=HexDigit(getAcftType());                          // [0..15] aircraft-type: 1=glider, 2=tow plane, etc.
     NMEA[Len++]=',';
     NMEA[Len++]='0'+getAddrType();                                // [0..3] address-type: 1=ICAO, 2=FLARM, 3=OGN
     NMEA[Len++]=',';
     uint32_t Addr = getAddress();                                 // [24-bit] address
     Len+=Format_Hex(NMEA+Len, (uint8_t)(Addr>>16));
     Len+=Format_Hex(NMEA+Len, (uint16_t)Addr);
     NMEA[Len++]=',';
     NMEA[Len++]='0'+getRelayCount();                              // [0..3] counts retransmissions
     NMEA[Len++]=',';
     NMEA[Len++]='0'+getFixQuality();                              // [] fix quality
     NMEA[Len++]='0'+getFixMode();                                 // [] fix mode
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, (uint16_t)(DecodeDOP()+10),2,1); // [] Dilution of Precision
     NMEA[Len++]=',';
     Len+=PrintLatitude(NMEA+Len, DecodeLatitude());               // [] Latitude
     NMEA[Len++]=',';
     Len+=PrintLongitude(NMEA+Len, DecodeLongitude());             // [] Longitude
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, (uint32_t)DecodeAltitude());     // [m] Altitude (by GPS)
     NMEA[Len++]=',';
     if(isBaro()) Len+=Format_SignDec(NMEA+Len, (int32_t)DecodeStdAltDiff()); // [m] Standard Pressure Altitude (by Baro)
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, DecodeSpeed()<<1, 2, 1);         // [kt] ground speed (by GPS)
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, DecodeHeading(), 4, 1);          // [deg] heading (by GPS)
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, DecodeClimbRate(), 2, 1);       // [m/s] climb/sink rate
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, -(int16_t)RxRSSI/2);            // [dBm] received signal level
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, (uint16_t)RxErr);                // [bits] corrected transmisison errors
     Len+=NMEA_AppendCheckCRNL(NMEA, Len);
     return Len; }

   uint8_t Print(char *Out)
   { uint8_t Len=0;
     Out[Len++]=HexDigit(getAcftType()); Out[Len++]=':';
     Out[Len++]='0'+getAddrType(); Out[Len++]=':';
     uint32_t Addr = getAddress();
     Len+=Format_Hex(Out+Len, (uint8_t)(Addr>>16));
     Len+=Format_Hex(Out+Len, (uint16_t)Addr);
     Out[Len++]=' ';
     Len+=Format_SignDec(Out+Len, -(int16_t)RxRSSI/2); Out[Len++]='d'; Out[Len++]='B'; Out[Len++]='m';
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint16_t)getTime(), 2);
     Out[Len++]=' ';
     Len+=PrintLatitude(Out+Len, DecodeLatitude());
     Out[Len++]=' ';
     Len+=PrintLongitude(Out+Len, DecodeLongitude());
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint32_t)DecodeAltitude()); Out[Len++]='m';
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, DecodeSpeed()<<1, 2, 1); Out[Len++]='k'; Out[Len++]='t';
     Out[Len++]=' ';
     Len+=Format_SignDec(Out+Len, DecodeClimbRate(), 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]='\n'; Out[Len]=0;
     return Len; }

   uint8_t PrintLatitude(char *Out, int32_t Lat)
   { uint8_t Len=0;
     char Sign='N';
     if(Lat<0) { Sign='S'; Lat=(-Lat); }
     uint32_t Deg=Lat/600000;
     Lat -= 600000*Deg;
     Len+=Format_UnsDec(Out+Len, Deg, 2, 0);
     Len+=Format_UnsDec(Out+Len, Lat, 6, 4);
     Out[Len++]=Sign;
     return Len; }

   uint8_t PrintLongitude(char *Out, int32_t Lon)
   { uint8_t Len=0;
     char Sign='E';
     if(Lon<0) { Sign='W'; Lon=(-Lon); }
     uint32_t Deg=Lon/600000;
     Lon -= 600000*Deg;
     Len+=Format_UnsDec(Out+Len, Deg, 3, 0);
     Len+=Format_UnsDec(Out+Len, Lon, 6, 4);
     Out[Len++]=Sign;
     return Len; }

   OGN_Packet() { Clear(); }
   void Clear(void) { Header=0; Position[0]=0; Position[1]=0; Position[2]=0; Position[3]=0; State=0; Rank=0; }

   // void calcFEC(void)                            { LDPC_Encode(&Header, FEC); }       // calculate the 48-bit parity check
   // void calcFEC(const uint32_t ParityGen[48][5]) { LDPC_Encode(&Header, FEC, ParityGen); }

   void calcFEC(void)                     { LDPC_Encode(Word); }       // calculate the 48-bit parity check
   int8_t checkFEC(void)    const  { return LDPC_Check(Word); }        // returns number of parity checks that fail (0 => no errors, all fine)
   // void Whiten  (void) { TEA_Encrypt(Position, OGN_WhitenKey, 4); TEA_Encrypt(Position+2, OGN_WhitenKey, 4); } // whiten the position
   // void Dewhiten(void) { TEA_Decrypt(Position, OGN_WhitenKey, 4); TEA_Decrypt(Position+2, OGN_WhitenKey, 4); } // de-whiten the position
   void Whiten  (void) { TEA_Encrypt_Key0(Position, 8); TEA_Encrypt_Key0(Position+2, 8); } // whiten the position
   void Dewhiten(void) { TEA_Decrypt_Key0(Position, 8); TEA_Decrypt_Key0(Position+2, 8); } // de-whiten the position

   int BitErr(OGN_Packet &RefPacket) const // return number of different data bits between this Packet and RefPacket
   { return Count1s(Header^RefPacket.Header)
           +Count1s(Position[0]^RefPacket.Position[0])
           +Count1s(Position[1]^RefPacket.Position[1])
           +Count1s(Position[2]^RefPacket.Position[2])
           +Count1s(Position[3]^RefPacket.Position[3])
           +Count1s(FEC[0]^RefPacket.FEC[0])
           +Count1s((FEC[1]^RefPacket.FEC[1])&0xFFFF); }

   bool isEmergency(void)   const { return Header &  0x80000000; } // emergency declared or detected (high-g shock ?)
   void setEmergency(void)        {        Header |= 0x80000000; }
   void clrEmergency(void)        {        Header &= 0x7FFFFFFF; }

   bool  isEncrypted(void) const  { return Header &  0x40000000; } // position can be encrypted with a public key (competitions, etc.)
   void setEncrypted(void)        {        Header |= 0x40000000; } // when in Emergency it must not be encrypted
   void clrEncrypted(void)        {        Header &= 0xBFFFFFFF; }

   uint8_t getRelayCount(void) const { return (Header>>28)&0x03; } // how many time the packet has been relayed
   void    setRelayCount(uint8_t Count) { Header = (Header&0xCFFFFFFF) | ((uint32_t)(Count&0x03)<<28); }
   void    incRelayCount(void)       { Header+=0x10000000; }

   bool goodAddrParity(void) const  { return ((Count1s(Header&0x0FFFFFFF)&1)==0); }  // Address parity should be EVEN
   void calcAddrParity(void)        { if(!goodAddrParity()) Header ^= 0x08000000; }  // if not correct parity, flip the parity bit

   bool  isOther(void) const  { return Header &  0x04000000; }          // this is a meteo or other report: sends wind speed/direction, pressure, temperatue and humidity
   void setOther(void)        {        Header |= 0x04000000; }
   void clrOther(void)        {        Header &= 0xFBFFFFFF; }

   uint8_t getAddrType(void) const   { return (Header>>24)&0x03; }      // Address type: 0 = Random, 1 = ICAO, 2 = FLARM, 3 = OGN
   void    setAddrType(uint8_t Type) { Header = (Header&0xFCFFFFFF) | ((uint32_t)(Type&0x03)<<24); }

   uint32_t getAddress(void) const { return Header&0x00FFFFFF; }        // Address: 24-bit
   void setAddress(uint32_t Address) { Header = (Header&0xFF000000) | (Address&0x00FFFFFF); }

   uint32_t getAddressAndType(void) const { return Header&0x03FFFFFF; } // Address with address-type: 26-bit

   bool  isStealth(void) const  { return Position[3] &  0x00080000; }   // position not to be displayed on public webpages
   void setStealth(uint8_t Stealth) { clrStealth(); if(Stealth) setStealth(); }
   void setStealth(void)        {        Position[3] |= 0x00080000; }
   void clrStealth(void)        {        Position[3] &= 0xFFF7FFFF; }

   uint8_t  getAcftType(void) const { return (Position[3]>>20)&0x0F; }
   void     setAcftType(uint8_t Type) { Position[3] = (Position[3]&0xFF0FFFFF) | ((uint32_t)(Type&0x0F)<<20); }

   uint8_t  getTime(void) const { return (Position[0]>>24)&0x3F; }              // 6 lower bits of the UnitTime or the second counter ?
   void     setTime(uint8_t Time) { Position[0] = (Position[0]&0xC0FFFFFF) | ((uint32_t)(Time&0x3F)<<24); }

   uint8_t  getFixMode(void) const { return (Position[1]>>31)&0x01; }           // 0 = 2-D, 1 = 3-D
   void     setFixMode(uint8_t Mode) { Position[1] = (Position[1]&0x7FFFFFFF) | ((uint32_t)(Mode&0x01)<<31); }

   bool  isBaro(void) const  { return Position[1] &  0x40000000; } // climb and alitude are takens with the barometer
   void setBaro(void)        {        Position[1] |= 0x40000000; } // after processing and calibration with the GPS
   void clrBaro(void)        {        Position[1] &= 0xBFFFFFFF; }

   uint8_t  getFixQuality(void) const { return (Position[0]>>30)&0x03; }        // 0 = no fix, 1 = GPS, 2 = diff. GPS, 3 = other
   void     setFixQuality(uint8_t Qual) { Position[0] = (Position[0]&0x3FFFFFFF) | ((uint32_t)(Qual&0x03)<<30); }

   // int32_t static RoundDiv(int32_t Value, int32_t Div)
   // { return Value>0 ? (Value+Div/2)/Div : (Value-Div/2)/Div; }

                                                                        // calc. rank for retransmission: higher rank, better chance to retransmit
   void calcRelayRank(int32_t RxAltitude)                               // [0.1m] altitude of reception
   { if(isEmergency()) { Rank=0xFF; return; }                           // emergency packets always highest rank
     Rank=0;
     if(isOther()) return;                                              // only relay position packets
     if(getTime()>=60) return;
     if(getRelayCount()>0) return;                                      // no rank for relayed packets (only single relay)
     if(RxRSSI>32)                                                      // [-0.5dB] weaker signal => higher rank
       Rank += (RxRSSI-32)>>2;                                          // 1 point/2dB lower signal
     RxAltitude -= 10*DecodeAltitude();                                 // [0.1m] lower altitude => higher rank
     if(RxAltitude>0)
       Rank += RxAltitude>>10;                                          // 1 point/100m of altitude below
     int16_t ClimbRate = DecodeClimbRate();                             // [0.1m/s] higher sink rate => higher rank
     if(ClimbRate<0)
       Rank += (-ClimbRate)>>3;                                         // 1 point/0.8m/s of sink
   }

   void EncodeLatitude(int32_t Latitude)                                // encode Latitude: units are 0.0001/60 degrees
   { Latitude>>=3; Position[0] = (Position[0]&0xFF000000) | (Latitude&0x00FFFFFF); } // scaled by 8 thus 1.5m resolution

   int32_t DecodeLatitude(void) const
   { int32_t Latitude=Position[0]&0x00FFFFFF;
     if(Latitude&0x00800000) Latitude|=0xFF000000;
     Latitude = (Latitude<<3)+4; return Latitude; }

   void EncodeLongitude(int32_t Longitude)                             // encode Longitude: units are 0.0001/60 degrees
   { Longitude>>=4; Position[1] = (Position[1]&0xFF000000) | (Longitude&0x00FFFFFF); } // scaled down by 16 thus 3m resolution on the equator

   int32_t DecodeLongitude(void) const
   { int32_t Longitude=Position[1]&0x00FFFFFF;
     if(Longitude&0x00800000) Longitude|=0xFF000000;
     Longitude = (Longitude<<4)+8; return Longitude; }

   void EncodeAltitude(int32_t Altitude)                               // encode altitude in meters
   {      if(Altitude<0)      Altitude=0;
     else if(Altitude<0x1000) { }
     else if(Altitude<0x3000) Altitude = 0x1000 | ((Altitude-0x1000)>>1);
     else if(Altitude<0x7000) Altitude = 0x2000 | ((Altitude-0x3000)>>2);
     else if(Altitude<0xF000) Altitude = 0x3000 | ((Altitude-0x7000)>>3);
     else                     Altitude = 0x3FFF;
     Position[2] = (Position[2]&0xFFFFC000) | (Altitude&0x3FFF); }

   int32_t DecodeAltitude(void) const                                   // return Altitude in meters
   { int32_t Altitude =  Position[2]     &0x0FFF;
     int32_t Range    = (Position[2]>>12)&0x0003;
     if(Range==0) return         Altitude;       // 0000..0FFF
     if(Range==1) return 0x1001+(Altitude<<1);   // 1000..2FFE
     if(Range==2) return 0x3002+(Altitude<<2);   // 3000..6FFC
                  return 0x7004+(Altitude<<3); } // 7000..EFF8 => max. altitude: 61432 meters

   void EncodeDOP(uint8_t DOP)
   {      if(DOP<0)    DOP=0;
     else if(DOP<0x10) { }
     else if(DOP<0x30) DOP = 0x10 | ((DOP-0x10)>>1);
     else if(DOP<0x70) DOP = 0x20 | ((DOP-0x30)>>2);
     else if(DOP<0xF0) DOP = 0x30 | ((DOP-0x70)>>3);
     else              DOP = 0x3F;
     Position[1] = (Position[1]&0xC0FFFFFF) | ((uint32_t)DOP<<24); }

   uint8_t DecodeDOP(void) const
   { uint8_t DOP   =  (Position[1]>>24)&0x0F;
     int8_t Range =  (Position[1]>>28)&0x03;
     if(Range==0) return       DOP;              // 00..0F
     if(Range==1) return 0x11+(DOP<<1);          // 10..2E
     if(Range==2) return 0x31+(DOP<<2);          // 30..6C
                  return 0x74+(DOP<<4); }        // 70..E8 => max. DOP = 232*0.1=23.2

   void EncodeSpeed(int16_t Speed)                                       // speed in 0.2 knots
   {      if(Speed<0)     Speed=0;
     else if(Speed<0x100) { }
     else if(Speed<0x300) Speed = 0x100 | ((Speed-0x100)>>1);
     else if(Speed<0x700) Speed = 0x200 | ((Speed-0x300)>>2);
     else if(Speed<0xF00) Speed = 0x300 | ((Speed-0x700)>>3);
     else                 Speed = 0x3FF;
     Position[2] = (Position[2]&0xFF003FFF) | ((uint32_t)Speed<<14); }

   int16_t DecodeSpeed(void) const           // return speed in 0.2 knots units (0.2 knots is about 0.1 m/s)
   { int16_t Speed = (Position[2]>>14)&0x00FF;
     int8_t  Range = (Position[2]>>22)&0x0003;
     if(Range==0) return Speed;              // 000..0FF
     if(Range==1) return 0x101+(Speed<<1);   // 100..2FE
     if(Range==2) return 0x302+(Speed<<2);   // 300..6FC
                  return 0x704+(Speed<<3); } // 700..EF8 => max. speed: 3832*0.2 = 766 knots

   void EncodeTurnRate(int16_t Turn)
   { int8_t Sign=0; if(Turn<0) { Turn=(-Turn); Sign=0x80; }
          if(Turn<0x020) { }
     else if(Turn<0x060) Turn = 0x020 | ((Turn-0x020)>>1);
     else if(Turn<0x0E0) Turn = 0x040 | ((Turn-0x060)>>2);
     else if(Turn<0x1E0) Turn = 0x060 | ((Turn-0x0E0)>>3);
     else                Turn = 0x07F;
     Turn |= Sign;
     Position[2] = (Position[2]&0x00FFFFFF) | ((uint32_t)Turn<<24); }

   int16_t DecodeTurnRate(void) const
   { int8_t Sign =(Position[2]>>31)&0x01;
     int8_t Range=(Position[2]>>29)&0x03;
     int16_t Turn =(Position[2]>>24)&0x1F;
          if(Range==0) { }                          // 000..01F
     else if(Range==1) { Turn = 0x021+(Turn<<1); }  // 020..05E
     else if(Range==2) { Turn = 0x062+(Turn<<2); }  // 060..0DC
     else              { Turn = 0x0E4+(Turn<<3); }  // 0E0..1D8 => max. turn rate = +/- 472*0.1 = +/- 47.2 deg/s
     return Sign ? -Turn:Turn; }

   int16_t DecodeHeading(void) const         // return Heading in 0.1 degree units
   { int32_t Heading = Position[3]&0x3FF;
     return (Heading*3600+512)>>10; }

   void EncodeHeading(int16_t Heading)
   { Heading = (((int32_t)Heading<<10)+180)/3600;
     Position[3] = (Position[3]&0xFFFFFC00) | ((Heading&0x3FF)); }

   void EncodeClimbRate(int16_t Climb)
   { int16_t Sign=0; if(Climb<0) { Climb=(-Climb); Sign=0x100; }
          if(Climb<0x040) { }
     else if(Climb<0x0C0) Climb = 0x040 | ((Climb-0x040)>>1);
     else if(Climb<0x1C0) Climb = 0x080 | ((Climb-0x0C0)>>2);
     else if(Climb<0x3C0) Climb = 0x0C0 | ((Climb-0x1C0)>>3);
     else                 Climb = 0x0FF;
     Climb |= Sign;
     Position[3] = (Position[3]&0xFFF803FF) | ((int32_t)Climb<<10); }

   int16_t DecodeClimbRate(void) const
   { int32_t Sign =(Position[3]>>18)&0x01;
     int32_t Range=(Position[3]>>16)&0x03;
     int32_t Climb=(Position[3]>>10)&0x3F;
          if(Range==0) { }                            // 000..03F
     else if(Range==1) { Climb = 0x041+(Climb<<1); }  // 040..0BE
     else if(Range==2) { Climb = 0x0C2+(Climb<<2); }  // 0C0..1BC
     else              { Climb = 0x1C4+(Climb<<3); }  // 1C0..3B8 => max. climb rate = +/- 952*0.1 = +/- 95.2 m/s
     return Sign ? -Climb:Climb; }

   void EncodeStdAltDiff(int8_t AltDiff)
   { Position[3] = (Position[3]&0x00FFFFFF) | ((int32_t)AltDiff<<24) ; }

   int8_t DecodeStdAltDiff(void) const
   { return (Position[3]>>24)&0xFF; }

/*
   void EncodeTemperature(int16_t Temp)             // [0.5 degC]
   { int8_t Sign=0; if(Temp<0) { Temp=(-Temp); Sign=0x80; }
          if(Temp<0x020) { }
     else if(Temp<0x060) Temp = 0x020 | ((Temp-0x020)>>1);
     else if(Temp<0x0E0) Temp = 0x040 | ((Temp-0x060)>>2);
     else if(Temp<0x1E0) Temp = 0x060 | ((Temp-0x0E0)>>3);
     else                Temp = 0x07F;
     Temp |= Sign;
     Position[3] = (Position[3]&0x00FFFFFF) | ((int32_t)Temp<<24); }

   int16_t DecodeTemperature(void) const
   { int8_t Sign =(Position[3]>>31)&0x01;
     int8_t Range=(Position[3]>>29)&0x03;
     int16_t Temp =(Position[3]>>24)&0x1F;
          if(Range==0) { }                          // 000..01F
     else if(Range==1) { Temp = 0x021+(Temp<<1); }  // 020..05E
     else if(Range==2) { Temp = 0x062+(Temp<<2); }  // 060..0DC
     else              { Temp = 0x0E4+(Temp<<3); }  // 0E0..1D8 => max. temperature = +/- 472*0.5 = +/- 236 degC
     return Sign ? -Temp:Temp; }
*/
   static void TEA_Encrypt (uint32_t* Data, const uint32_t *Key, int Loops=4)
   { uint32_t v0=Data[0], v1=Data[1];                         // set up
     const uint32_t delta=0x9e3779b9; uint32_t sum=0;         // a key schedule constant
     uint32_t k0=Key[0], k1=Key[1], k2=Key[2], k3=Key[3];     // cache key
     for (int i=0; i < Loops; i++)                            // basic cycle start
     { sum += delta;
       v0 += ((v1<<4) + k0) ^ (v1 + sum) ^ ((v1>>5) + k1);
       v1 += ((v0<<4) + k2) ^ (v0 + sum) ^ ((v0>>5) + k3); }  // end cycle
     Data[0]=v0; Data[1]=v1;
   }

   void TEA_Decrypt (uint32_t* Data, const uint32_t *Key, int Loops=4)
   { uint32_t v0=Data[0], v1=Data[1];                           // set up
     const uint32_t delta=0x9e3779b9; uint32_t sum=delta*Loops; // a key schedule constant
     uint32_t k0=Key[0], k1=Key[1], k2=Key[2], k3=Key[3];       // cache key
     for (int i=0; i < Loops; i++)                              // basic cycle start */
     { v1 -= ((v0<<4) + k2) ^ (v0 + sum) ^ ((v0>>5) + k3);
       v0 -= ((v1<<4) + k0) ^ (v1 + sum) ^ ((v1>>5) + k1);
       sum -= delta; }                                          // end cycle
     Data[0]=v0; Data[1]=v1;
   }

   static void TEA_Encrypt_Key0 (uint32_t* Data, int Loops=4)
   { uint32_t v0=Data[0], v1=Data[1];                          // set up
     const uint32_t delta=0x9e3779b9; uint32_t sum=0;          // a key schedule constant
     for (int i=0; i < Loops; i++)                             // basic cycle start
     { sum += delta;
       v0 += (v1<<4) ^ (v1 + sum) ^ (v1>>5);
       v1 += (v0<<4) ^ (v0 + sum) ^ (v0>>5); }  // end cycle
     Data[0]=v0; Data[1]=v1;
   }

   void TEA_Decrypt_Key0 (uint32_t* Data, int Loops=4)
   { uint32_t v0=Data[0], v1=Data[1];                           // set up
     const uint32_t delta=0x9e3779b9; uint32_t sum=delta*Loops; // a key schedule constant
     for (int i=0; i < Loops; i++)                              // basic cycle start */
     { v1 -= (v0<<4) ^ (v0 + sum) ^ (v0>>5);
       v0 -= (v1<<4) ^ (v1 + sum) ^ (v1>>5);
       sum -= delta; }                                          // end cycle
     Data[0]=v0; Data[1]=v1;
   }

} ;

template<uint8_t Size=8>
 class OGN_PrioQueue
{ public:
   // static const uint8_t Size = 8;            // number of packets kept
   OGN_Packet           Packet[Size];        // OGN packets
   uint16_t             Sum;                 // sum of all ranks
   uint8_t              Low, LowIdx;         // lowest rank and index

  public:
   void Clear(void)                                                           // clear (reset) the queue
   { for(uint8_t Idx=0; Idx<Size; Idx++)                                      // clear every packet
     { Packet[Idx].Clear(); }
     Sum=0; Low=0; LowIdx=0; }                                                // clear the rank sum, lowest rank

   OGN_Packet * operator [](uint8_t Idx) { return Packet+Idx; }

   uint8_t getNew(void)                                                       // get (index of) a free or lowest rank packet
   { Sum-=Packet[LowIdx].Rank; Packet[LowIdx].Rank=0; Low=0; return LowIdx; } // remove old packet from the rank sum

   void addNew(uint8_t NewIdx)                                                // add the new packet to the queue
   { uint32_t AddressAndType = Packet[NewIdx].getAddressAndType();            // get ID of this packet: ID is address-type and address (2+24 = 26 bits)
     for(uint8_t Idx=0; Idx<Size; Idx++)                                      // look for other packets with same ID
     { if(Idx==NewIdx) continue;                                              // avoid the new packet
       if(Packet[Idx].getAddressAndType() == AddressAndType)                  // if another packet with same ID:
       { Sum-=Packet[Idx].Rank; Packet[Idx].Rank=0; Low=0; LowIdx=Idx; }      // then remove it: set rank to zero
     }
     uint8_t Rank=Packet[NewIdx].Rank; Sum+=Rank;                             // add the new packet to the rank sum
     if(NewIdx!=LowIdx)                                                       // 
     { if(Rank<=Low) { Low=Rank; LowIdx=NewIdx; } }
     else reCalc();
   }

   uint8_t getRand(uint32_t Rand)                                             // get a position by random selection but probabilities prop. to ranks
   { if(Sum==0) return Rand%Size;
     uint16_t RankIdx = Rand%Sum;
     uint8_t Idx; uint16_t RankSum=0;
     for(Idx=0; Idx<Size; Idx++)
     { uint8_t Rank=Packet[Idx].Rank; if(Rank==0) continue; 
       RankSum+=Rank; if(RankSum>=RankIdx) break; }
     return Idx; }

   void reCalc(void)                                                           // find the lowest rank and calc. the sum of all ranks
   { Sum=Low=Packet[0].Rank; LowIdx=0;
     for(uint8_t Idx=1; Idx<Size; Idx++)
     { Sum+=Packet[Idx].Rank; 
       uint8_t Rank=Packet[Idx].Rank;
       if(Rank<Low) { Low=Rank; LowIdx=Idx; }
     }
   }

   void cleanTime(uint8_t Time)                                                // clean up packets of given Time
   { for(int Idx=0; Idx<Size; Idx++)
     { if( (Packet[Idx].Rank) && (Packet[Idx].getTime()==Time) )
       { Sum-=Packet[Idx].Rank; Packet[Idx].Rank=0; Low=0; LowIdx=0; }
     }
   }

   void decrRank(uint8_t Idx, uint8_t Decr=1)
   { uint8_t Rank=Packet[Idx].Rank; if(Rank==0) return;
     if(Decr>Rank) Decr=Rank;
     Rank-=Decr; Sum-=Decr;
     if(Rank<Low) { Low=Rank; LowIdx=Idx; }
     Packet[Idx].Rank=Rank; }

   uint8_t Print(char *Out)
   { uint8_t Len=0;
     for(uint8_t Idx=0; Idx<Size; Idx++)
     { uint8_t Rank=Packet[Idx].Rank;
       Out[Len++]=' '; Len+=Format_Hex(Out+Len, Rank);
       if(Rank)
       { Out[Len++]='/'; Len+=Format_Hex(Out+Len, Packet[Idx].getAddressAndType() ); }
     }
     Out[Len++]=' '; Len+=Format_Hex(Out+Len, Sum);
     Out[Len++]='/'; Len+=Format_Hex(Out+Len, LowIdx);
     Out[Len++]='\n'; Out[Len]=0; return Len; }

} ;

class OgnPosition
{ public:
  uint8_t Flags;                // bit #0 = GGA and RMC had same Time
   int8_t FixQuality;           // 0 = none, 1 = GPS, 2 = Differential GPS (can be WAAS)
   int8_t FixMode;              // 0 = not set (from GSA) 1 = none, 2 = 2-D, 3 = 3-D
   int8_t Satellites;           // number of active satellites

   int8_t  Year, Month, Day;    // Date (UTC) from GPS
   int8_t  Hour, Min, Sec;      // Time-of-day (UTC) from GPS
   int8_t  FracSec;             // [1/100 sec] some GPS-es give second fraction with the time-of-day

   uint8_t PDOP;                // [0.1] dilution of precision
   uint8_t HDOP;                // [0.1] horizontal dilution of precision
   uint8_t VDOP;                // [0.1] vertical dilution of precision

   int16_t Speed;               // [0.1 knot] speed-over-ground
   int16_t Heading;             // [0.1 deg]  heading-over-ground

   int16_t ClimbRate;           // [0.1 meter/sec)
   int16_t TurnRate;            // [0.1 deg/sec]

   int16_t GeoidSeparation;     // [0.1 meter] difference between Geoid and Ellipsoid 
   int32_t Altitude;            // [0.1 meter] height above Geoid (sea level)

   int32_t Latitude;            // [0.0001/60 deg] about 0.018m accuracy (to convert to u-Blox GPS 1e-7deg units mult by. 5/3)
   int32_t Longitude;           // [0.0001/60 deg]

   int32_t StdAltitude;         // [0.1 meter] standard pressure altitude (from the pressure sensor)
   int16_t Temperature;         // [0.1 degC]

  public:

   OgnPosition() { Clear(); }

   void Clear(void)
   { Flags=0; FixQuality=0; FixMode=0; PDOP=0; HDOP=0; VDOP=0;
     setDefaultDate(); setDefaultTime();
     Latitude=0; Longitude=0; Altitude=0; GeoidSeparation=0;
     Speed=0; Heading=0; ClimbRate=0; TurnRate=0;
     StdAltitude=0; Temperature=0; }

   void setDefaultDate() { Year=00; Month=1; Day=1; }
   void setDefaultTime() { Hour=0;  Min=0;   Sec=0; FracSec=0; }

   // void CopyTime(OgnPosition *Previous) { }

   bool  isComplete(void) const { return Flags&0x01; } // have both RMC and GGA sentences been received and for same time ?
   void setComplete(void)       { Flags|=0x01; }
   void clrComplete(void)       { Flags&=0xFE; }

   bool hasBaro(void)     const { return Flags&0x02; } // has data from pressure sensor
   void setBaro(void)           { Flags|=0x02; }
   void clrBaro(void)           { Flags&=0xFD; }

   bool isTimeValid(void) const                      // is the GPS time-of-day valid ?
   { return (Hour>=0) && (Min>=0) && (Sec>=0); }     // all data must have been correctly read: negative means not correctly read)

   bool isDateValid(void) const                      // is the GPS date valid ?
   { return (Year>=0) && (Month>=0) && (Day>=0); }

   bool isValid(void) const                          // is GPS lock there ?
   { if(!isTimeValid()) return 0;                    // is GPS time valid/present ?
     if(!isDateValid()) return 0;                    // is GPS date valid/present ?
     if(FixQuality==0) return 0;                     // Fix quality must be 1=GPS or 2=DGPS
     if(FixMode==1) return 0;                        // if GSA says "no lock" (when GSA is not there, FixMode=0)
     if(Satellites<=0) return 0;                     // if number of satellites none or invalid
     return 1; }

   void copyTime(OgnPosition &RefPosition)           // copy HH:MM:SS.SSS from another record
   { FracSec = RefPosition.FracSec;
     Sec     = RefPosition.Sec;
     Min     = RefPosition.Min;
     Hour    = RefPosition.Hour; }

   void copyDate(OgnPosition &RefPosition)           // copy YY:MM:DD from another record
   { Day     = RefPosition.Day;
     Month   = RefPosition.Month;
     Year    = RefPosition.Year; }

   void copyTimeDate(OgnPosition &RefPosition) { copyTime(RefPosition); copyDate(RefPosition); }

   uint8_t incrTime(void)                            // increment HH:MM:SS by one second
   { Sec++; if(Sec<60) return 0;
     Sec=0;
     Min++; if(Min<60) return 0;
     Min=0;
     Hour++; if(Hour<24) return 0;
     Hour=0;
     return 1; }                                     // return 1 if date needs to be incremented

   uint8_t MonthDays(void)                           // number of days per month
   { const uint16_t Table = 0x0CD5;
     // const uint8_t Table[12] = { 31,28,31,30, 31,30,31,31, 30,31,30,31 };
     if( (Month<1) || (Month>12) ) return 0;
     if( Month==2) return 28+isLeapYear();
     return 30 + ((Table>>(Month-1))&1); }

   void incrDate(void)                               // increment YY:MM:DD by one day
   { Day++; if(Day<MonthDays()) return;
     Day=1; Month++; if(Month<12) return;
     Month=1; Year++; }

   void incrTimeData(void) { if(incrTime()) incrDate(); }

#ifndef __AVR__ // there is not printf() with AVR
   void PrintDateTime(void) const { printf("%02d.%02d.%04d %02d:%02d:%05.2f", Day, Month, 2000+Year, Hour, Min, Sec+0.01*FracSec ); }
   void PrintTime(void)     const { printf("%02d:%02d:%05.2f", Hour, Min, Sec+0.01*FracSec ); }

   int PrintDateTime(char *Out) const { return sprintf(Out, "%02d.%02d.%04d %02d:%02d:%02d.%02d", Day, Month, Year, Hour, Min, Sec, FracSec ); }
   int PrintTime(char *Out)     const { return sprintf(Out, "%02d:%02d:%02d.%02d", Hour, Min, Sec, FracSec ); }

   void Print(void) const
   { printf("Time/Date = "); PrintDateTime(); printf("\n"); // printf(" = %10ld.%03dsec\n", (long int)UnixTime, mSec);
     printf("FixQuality/Mode=%d/%d: %d satellites DOP/H/V=%3.1f/%3.1f/%3.1f\n", FixQuality, FixMode, Satellites, 0.1*PDOP, 0.1*HDOP, 0.1*VDOP);
     printf("FixQuality=%d: %d satellites HDOP=%3.1f\n", FixQuality, Satellites, 0.1*HDOP);
     printf("Lat/Lon/Alt = [%+10.6f,%+10.6f]deg %+3.1f(%+3.1f)m\n", 0.0001/60*Latitude, 0.0001/60*Longitude, 0.1*Altitude, 0.1*GeoidSeparation);
     printf("Speed/Heading = %4.2fkt %06.2fdeg\n", 0.1*Speed, 0.1*Heading);
   }

   int Print(char *Out) const
   { int Len=0;
     Len+=sprintf(Out+Len, "Time/Date = "); Len+=PrintDateTime(Out+Len); printf("\n"); // Len+=sprintf(Out+Len, " = %10ld.%02dsec\n", (long int)UnixTime, FracSec);
     Len+=sprintf(Out+Len, "FixQuality/Mode=%d/%d: %d satellites DOP/H/V=%3.1f/%3.1f/%3.1f\n", FixQuality, FixMode, Satellites, 0.1*PDOP, 0.1*HDOP, 0.1*VDOP);
     Len+=sprintf(Out+Len, "Lat/Lon/Alt = [%+10.6f,%+10.6f]deg %+3.1f(%+3.1f)m\n", 0.0001/60*Latitude, 0.0001/60*Longitude, 0.1*Altitude, 0.1*GeoidSeparation);
     Len+=sprintf(Out+Len, "Speed/Heading = %4.2fkt %06.2fdeg\n", 0.1*Speed, 0.1*Heading);
     return Len; }

   void PrintLine(void) const
   { PrintTime();
     printf(" %d/%d/%02d/%4.1f/%4.1f/%4.1f", FixQuality, FixMode, Satellites, 0.1*PDOP, 0.1*HDOP, 0.1*VDOP);
     printf(" [%+10.6f,%+10.6f]deg %+3.1f(%+3.1f)m", 0.0001/60*Latitude, 0.0001/60*Longitude, 0.1*Altitude, 0.1*GeoidSeparation);
     printf(" %4.1fkt %05.1fdeg", 0.1*Speed, 0.1*Heading);
     printf("\n"); }

   int PrintLine(char *Out) const
   { int Len=PrintDateTime(Out);
     Len+=sprintf(Out+Len, " %d/%d/%02d", FixQuality, FixMode, Satellites);
     Out[Len++]='/'; Len+=Format_UnsDec(Out+Len, PDOP, 2, 1);
     Out[Len++]='/'; Len+=Format_UnsDec(Out+Len, HDOP, 2, 1);
     Out[Len++]='/'; Len+=Format_UnsDec(Out+Len, VDOP, 2, 1);
     Out[Len++]=' ';
     Out[Len++]='['; Len+=Format_SignDec(Out+Len, Latitude/60, 6, 4);
     Out[Len++]=','; Len+=Format_SignDec(Out+Len, Longitude/60, 7, 4);
     Out[Len++]=']'; Out[Len++]='d'; Out[Len++]='e'; Out[Len++]='g';
     Out[Len++]=' '; Len+=Format_SignDec(Out+Len, Altitude, 4, 1); Out[Len++]='m';
     Out[Len++]='/'; Len+=Format_SignDec(Out+Len, GeoidSeparation, 4, 1); Out[Len++]='m';
     Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, Speed/10  , 2, 1); Out[Len++]='k'; Out[Len++]='t';
     Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, Heading/10, 4, 1); Out[Len++]='d'; Out[Len++]='e'; Out[Len++]='g';
     Out[Len++]='\n'; Out[Len++]=0; return Len; }
#endif // __AVR__

   int8_t ReadNMEA(NMEA_RxMsg &RxMsg)
   {      if(RxMsg.isGPGGA()) return ReadGGA(RxMsg);
     else if(RxMsg.isGPRMC()) return ReadRMC(RxMsg);
     else if(RxMsg.isGNRMC()) return ReadRMC(RxMsg);
     else if(RxMsg.isGPGSA()) return ReadGSA(RxMsg);
     else if(RxMsg.isGNGSA()) return ReadGSA(RxMsg);
     else return 0; }

   int8_t ReadNMEA(const char *NMEA)
   { int Err=0;
     Err=ReadGGA(NMEA); if(Err!=(-1)) return Err;
     Err=ReadGSA(NMEA); if(Err!=(-1)) return Err;
     Err=ReadRMC(NMEA); if(Err!=(-1)) return Err;
     return 0; }

   int8_t ReadGGA(NMEA_RxMsg &RxMsg)
   { if(RxMsg.Parms<14) return -1;                                                        // no less than 14 paramaters
     if(ReadTime((const char *)RxMsg.ParmPtr(0))>0) setComplete(); else clrComplete();    // read time and check if same as the RMC says
     FixQuality =Read_Dec1(*RxMsg.ParmPtr(5)); if(FixQuality<0) FixQuality=0;             // fix quality: 0=invalid, 1=GPS, 2=DGPS
     Satellites=Read_Dec2((const char *)RxMsg.ParmPtr(6));                                // number of satellites
     if(Satellites<0) Satellites=Read_Dec1(RxMsg.ParmPtr(6)[0]);
     if(Satellites<0) Satellites=0;
     ReadHDOP((const char *)RxMsg.ParmPtr(7));                                            // horizontal dilution of precision
     ReadLatitude(*RxMsg.ParmPtr(2), (const char *)RxMsg.ParmPtr(1));                     // Latitude
     ReadLongitude(*RxMsg.ParmPtr(4), (const char *)RxMsg.ParmPtr(3));                    // Longitude
     ReadAltitude(*RxMsg.ParmPtr(9), (const char *)RxMsg.ParmPtr(8));                     // Altitude
     ReadGeoidSepar(*RxMsg.ParmPtr(11), (const char *)RxMsg.ParmPtr(10));                 // Geoid separation
     return 1; }

   int8_t ReadGGA(const char *GGA)
   { if(memcmp(GGA, "$GPGGA", 6)!=0) return -1;                                           // check if the right sequence
     uint8_t Index[20]; if(IndexNMEA(Index, GGA)<14) return -2;                           // index parameters and check the sum
     if(ReadTime(GGA+Index[0])>0) setComplete(); else clrComplete();
     FixQuality =Read_Dec1(GGA[Index[5]]); if(FixQuality<0) FixQuality=0;                 // fix quality
     Satellites=Read_Dec2(GGA+Index[6]);                                                  // number of satellites
     if(Satellites<0) Satellites=Read_Dec1(GGA[Index[6]]);
     if(Satellites<0) Satellites=0;
     ReadHDOP(GGA+Index[7]);                                                              // horizontal dilution of precision
     ReadLatitude( GGA[Index[2]], GGA+Index[1]);                                          // Latitude
     ReadLongitude(GGA[Index[4]], GGA+Index[3]);                                          // Longitude
     ReadAltitude(GGA[Index[9]], GGA+Index[8]);                                           // Altitude
     ReadGeoidSepar(GGA[Index[11]], GGA+Index[10]);                                       // Geoid separation
     return 1; }

   int8_t ReadGSA(NMEA_RxMsg &RxMsg)
   { if(RxMsg.Parms<17) return -1;
     FixMode =Read_Dec1(*RxMsg.ParmPtr(1)); if(FixMode<0) FixMode=0;                       // fix mode
     ReadPDOP((const char *)RxMsg.ParmPtr(14));                                           // total dilution of precision
     ReadHDOP((const char *)RxMsg.ParmPtr(15));                                           // horizontal dilution of precision
     ReadVDOP((const char *)RxMsg.ParmPtr(16));                                           // vertical dilution of precision
     return 1; }

   int8_t ReadGSA(const char *GSA)
   { if(memcmp(GSA, "$GPGSA", 6)!=0) return -1;                                           // check if the right sequence
     uint8_t Index[20]; if(IndexNMEA(Index, GSA)<17) return -2;                           // index parameters and check the sum
     FixMode =Read_Dec1(GSA[Index[1]]); if(FixMode<0) FixMode=0;
     ReadPDOP(GSA+Index[14]);
     ReadHDOP(GSA+Index[15]);
     ReadVDOP(GSA+Index[16]);
     return 1; }

   int ReadRMC(NMEA_RxMsg &RxMsg)
   { if(RxMsg.Parms<12) return -1;                                                        // no less than 12 parameters
     if(ReadTime((const char *)RxMsg.ParmPtr(0))>0) setComplete(); else clrComplete();    // read time and check if same as the GGA says
     if(ReadDate((const char *)RxMsg.ParmPtr(8))<0) setDefaultDate();                     // date
     ReadLatitude(*RxMsg.ParmPtr(3), (const char *)RxMsg.ParmPtr(2));                     // Latitude
     ReadLongitude(*RxMsg.ParmPtr(5), (const char *)RxMsg.ParmPtr(4));                    // Longitude
     ReadSpeed((const char *)RxMsg.ParmPtr(6));                                           // Speed
     ReadHeading((const char *)RxMsg.ParmPtr(7));                                         // Heading
     return 1; }

   int8_t ReadRMC(const char *RMC)
   { if(memcmp(RMC, "$GPRMC", 6)!=0) return -1;                                           // check if the right sequence
     uint8_t Index[20]; if(IndexNMEA(Index, RMC)<12) return -2;                           // index parameters and check the sum
     if(ReadTime(RMC+Index[0])>0) setComplete(); else clrComplete();
     if(ReadDate(RMC+Index[8])<0) setDefaultDate();
     ReadLatitude( RMC[Index[3]], RMC+Index[2]);
     ReadLongitude(RMC[Index[5]], RMC+Index[4]);
     ReadSpeed(RMC+Index[6]);
     ReadHeading(RMC+Index[7]);
     return 1; }

   int8_t calcDifferences(OgnPosition &RefPos) // calculate climb rate and turn ratewith an earlier reference position
   { ClimbRate=0; TurnRate=0;
     if(RefPos.FixQuality==0) return 0;
     int TimeDiff=Sec-RefPos.Sec; if(TimeDiff<(-30)) TimeDiff+=60;
     if(TimeDiff==0) return 0;
     ClimbRate=(Altitude-RefPos.Altitude)/TimeDiff;
     TurnRate=Heading-RefPos.Heading;
     if(TurnRate>1800) TurnRate-=3600; else if(TurnRate<(-1800)) TurnRate+=3600;
     TurnRate=TurnRate/TimeDiff;
     if(hasBaro() && RefPos.hasBaro() && (abs(Altitude-StdAltitude)<2500) )
     { ClimbRate=(StdAltitude-RefPos.StdAltitude)/TimeDiff; }
     return TimeDiff; }

   int8_t Encode(OGN_Packet &Packet) const
   { Packet.setFixQuality(FixQuality<3 ? FixQuality:3);
     if((FixQuality>0)&&(FixMode>=2)) Packet.setFixMode(FixMode-2);
                                 else Packet.setFixMode(0);
     if(PDOP>0) Packet.EncodeDOP(PDOP-10);                              // encode PDOP from GSA
           else Packet.EncodeDOP(HDOP-10);                              // or if no GSA: use HDOP
     int ShortTime=Sec;
     if(FracSec>=50) { ShortTime+=1; if(ShortTime>=60) ShortTime-=60; }
     Packet.setTime(ShortTime);
     Packet.EncodeLatitude(Latitude);
     Packet.EncodeLongitude(Longitude);
     Packet.EncodeSpeed((Speed+1)>>1);
     Packet.EncodeHeading(Heading);
     Packet.EncodeClimbRate(ClimbRate);
     Packet.EncodeTurnRate(TurnRate);
     Packet.EncodeAltitude((Altitude+5)/10);
     if(hasBaro())
     { int32_t AltDiff = (StdAltitude+5)/10-Packet.DecodeAltitude();;
       if(AltDiff>127) AltDiff=127;
       else if(AltDiff<(-127)) AltDiff=(-127);
       Packet.EncodeStdAltDiff(AltDiff);
       Packet.setBaro(); }
     else
     { Packet.EncodeStdAltDiff(0);
       Packet.clrBaro(); }
     return 0; }

  private:

   int8_t ReadLatitude(char Sign, const char *Value)
   { int8_t Deg=Read_Dec2(Value); if(Deg<0) return -1;
     int8_t Min=Read_Dec2(Value+2); if(Min<0) return -1;
     if(Value[4]!='.') return -1;
     int16_t FracMin=Read_Dec4(Value+5); if(FracMin<0) return -1;
     // printf("Latitude: %c %02d %02d %04d\n", Sign, Deg, Min, FracMin);
     Latitude = Times60((int16_t)Deg) + Min;
     Latitude = Latitude*(int32_t)10000 + FracMin;
     // printf("Latitude: %d\n", Latitude);
     if(Sign=='S') Latitude=(-Latitude);
     else if(Sign!='N') return -1;
     // printf("Latitude: %d\n", Latitude);
     return 0; }                                    // Latitude units: 0.0001/60 deg

   int8_t ReadLongitude(char Sign, const char *Value)
   { int16_t Deg=Read_Dec3(Value); if(Deg<0) return -1;
     int8_t Min=Read_Dec2(Value+3); if(Min<0) return -1;
     if(Value[5]!='.') return -1;
     int16_t FracMin=Read_Dec4(Value+6); if(FracMin<0) return -1;
     Longitude = Times60((int16_t)Deg) + Min;
     Longitude = Longitude*(int32_t)10000 + FracMin;
     if(Sign=='W') Longitude=(-Longitude);
     else if(Sign!='E') return -1;
     return 0; }                                    // Longitude units: 0.0001/60 deg

   int8_t ReadAltitude(char Unit, const char *Value)
   { if(Unit!='M') return -1;
     return Read_Float1(Altitude, Value); }          // Altitude units: 0.1 meter

   int8_t ReadGeoidSepar(char Unit, const char *Value)
   { if(Unit!='M') return -1;
     return Read_Float1(GeoidSeparation, Value); }   // GeoidSepar units: 0.1 meter

   int8_t ReadSpeed(const char *Value)
   { return Read_Float1(Speed, Value); }             // Speed units: 0.1 knots

   int8_t ReadHeading(const char *Value)
   { return Read_Float1(Heading, Value); }           // Heading units: 0.1 degree

   int8_t ReadPDOP(const char *Value)
   { int16_t DOP;
     if(Read_Float1(DOP, Value)<1) return -1;
     if(DOP<10) DOP=10;
     else if(DOP>255) DOP=255;
     PDOP=DOP; return 0; }

   int ReadHDOP(const char *Value)
   { int16_t DOP;
     if(Read_Float1(DOP, Value)<1) return -1;
     if(DOP<10) DOP=10;
     else if(DOP>255) DOP=255;
     HDOP=DOP; return 0; }

   int ReadVDOP(const char *Value)
   { int16_t DOP;
     if(Read_Float1(DOP, Value)<1) return -1;
     if(DOP<10) DOP=10;
     else if(DOP>255) DOP=255;
     VDOP=DOP; return 0; }

   int8_t ReadTime(const char *Value)
   { int8_t Prev; int8_t Same=1;
     Prev=Hour;
     Hour=Read_Dec2(Value);  if(Hour<0) return -1; // read hour (two digits)
     if(Prev!=Hour) Same=0;
     Prev=Min;
     Min=Read_Dec2(Value+2); if(Min<0)  return -1; // read minute (two digits)
     if(Prev!=Min) Same=0;
     Prev=Sec;
     Sec=Read_Dec2(Value+4); if(Sec<0)  return -1; // read second (two digits)
     if(Prev!=Sec) Same=0;
     Prev=FracSec;
     if(Value[6]=='.')                            // is there a second fraction ?
     { FracSec=Read_Dec2(Value+7); if(FracSec<0) return -1; }
     if(Prev!=FracSec) Same=0;
     return Same; }                             // return 1 when time did not change (both RMC and GGA were for same time)

   int8_t ReadDate(const char *Param)
   { Day=Read_Dec2(Param);     if(Day<0)   return -1; // read calendar year (two digits - thus need to be extended to four)
     Month=Read_Dec2(Param+2); if(Month<0) return -1; // read calendar month
     Year=Read_Dec2(Param+4);  if(Year<0)  return -1; // read calendar day
     return 0; }

   int8_t static IndexNMEA(uint8_t Index[20], const char *Seq) // index parameters and verify the NMEA checksum
   { if(Seq[0]!='$') return -1;
     if(Seq[6]!=',') return -1;
     uint8_t Check=Seq[1]^Seq[2]^Seq[3]^Seq[4]^Seq[5]^Seq[6];
     Index[0]=7; int8_t Params=1; int8_t Ptr;
     for(Ptr=7; ; )
     { char ch=Seq[Ptr++]; if(ch<' ') return -1;
       if(ch=='*') break;
       Check^=ch;
       if(ch==',') { Index[Params++]=Ptr; }
     }
     if(Seq[Ptr++]!=HexDigit(Check>>4)  ) { /* printf("H:%c:%c <=> %02X\n", Seq[Ptr-1],Seq[Ptr  ], Check); */ return -2; }
     if(Seq[Ptr++]!=HexDigit(Check&0x0F)) { /* printf("L:%c:%c <=> %02X\n", Seq[Ptr-2],Seq[Ptr-1], Check); */ return -2; }
     // printf("%s => [%d]\n", Seq, Params);
     return Params; }

  public:

   uint32_t getUnixTime(void)                               // return the Unix timestamp (tested 2000-2099)
   { uint16_t Days = DaysSince00() + DaysSimce1jan();
     return Times60(Times60(Times24((uint32_t)(Days+10957)))) + Times60((uint32_t)(Times60((uint16_t)Hour) + Min)) + Sec; }

   uint32_t getFatTime(void)                                // return timestamp in FAT format
   { uint16_t Date = ((uint16_t)(Year+20)<<9) | ((uint16_t)Month<<5) | Day;
     uint16_t Time = ((uint16_t)Hour<<11) | ((uint16_t)Min<<5) | (Sec>>1);
     return ((uint32_t)Date<<16) | Time; }

  private:

   uint8_t isLeapYear(void) { return (Year&3)==0; }

#ifdef __AVR__
   int16_t DaysSimce1jan(void)
   { static const uint8_t DaysDiff[12] PROGMEM = { 0, 3, 3, 6, 8, 11, 13, 16, 19, 21, 24, 26 } ;
     uint16_t Days = Times28((uint16_t)(Month-(int8_t)1)) + pgm_read_byte(DaysDiff+(Month-1)) + Day - 1;
     if(isLeapYear() && (Month>2) ) Days++;
     return Days; }
#else
   int16_t DaysSimce1jan(void)
   { static const uint8_t DaysDiff[12] = { 0, 3, 3, 6, 8, 11, 13, 16, 19, 21, 24, 26 } ;
     uint16_t Days = Times28((uint16_t)(Month-(int8_t)1)) + DaysDiff[Month-1] + Day - 1;
     if(isLeapYear() && (Month>2) ) Days++;
     return Days; }
#endif

   uint16_t DaysSince00(void)
   { uint16_t Days = 365*Year;
     if(Year>0) Days += ((Year-1)>>2)+1;
     return Days; }

   template <class Type>
     static Type Times60(Type X) { return ((X<<4)-X)<<2; }

   template <class Type>
     static Type Times28(Type X) { X+=(X<<1)+(X<<2); return X<<2; }

   template <class Type>
     static Type Times24(Type X) { X+=(X<<1);        return X<<3; }

} ;

#endif // of __OGN_H__

