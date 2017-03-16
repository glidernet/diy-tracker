#include <stdint.h>

class FreqPlan
{ public:
   uint8_t  Plan;        // 1=Europe, 2=USA/Canada, 3=Australia/Chile, 4=New Zeeland
   uint8_t  Channels;    // number of channels
   uint32_t BaseFreq;    // [Hz] base channel (#0) frequency
   uint32_t ChanSepar;   // [Hz] channel spacing
   static const uint8_t MaxChannels=65;

  public:
   void setPlan(uint8_t NewPlan=0) // preset for a given frequency plan
   { Plan=NewPlan;
          if(Plan==2) { BaseFreq=902200000; ChanSepar=400000; Channels=65; } // USA
     else if(Plan==3) { BaseFreq=917000000; ChanSepar=400000; Channels=24; } // Australia and South America
     else if(Plan==4) { BaseFreq=869250000; ChanSepar=200000; Channels= 1; } // New Zeeland
     else             { BaseFreq=868200000; ChanSepar=200000; Channels= 2; } // Europe
   }

   uint8_t getChannel  (uint32_t Time, uint8_t Slot=0, uint8_t OGN=1) const // OGN-tracker or FLARM, UTC time, slot: 0 or 1
   { if(Channels<=1) return 0;                                         // if single channel (New Zeeland)
     if(Plan>=2)                                                       // Hopping plans
     { int8_t Channel = FreqHopHash((Time<<1)+Slot) % Channels;
       if(OGN)                                                         // for OGN tracker
       { if(Slot) { Channel--; if(Channel<0)         Channel+=2; }     // for 2nd slot choose a lower channel
             else { Channel++; if(Channel>=Channels) Channel-=2; }     // for 1st slot choose a higher channel
       }
       return Channel; }                                               // return 0..Channels-1 for USA/CA or Australia.
     return Slot^OGN; }                                                // return 0 or 1 for EU freq. plan

   uint32_t getFrequency(uint32_t Time, uint8_t Slot=0, uint8_t OGN=1) const
   { uint8_t Channel=getChannel(Time, Slot, OGN); return BaseFreq+ChanSepar*Channel; } // return frequency [Hz] for given UTC time and slot

  private:
   static uint32_t FreqHopHash(uint32_t Time)
   { Time  = (Time<<15) + (~Time);
     Time ^= Time>>12;
     Time += Time<<2;
     Time ^= Time>>4;
     Time *= 2057;
     return Time ^ (Time>>16); }

} ;
