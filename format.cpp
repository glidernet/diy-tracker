#include "format.h"

// ------------------------------------------------------------------------------------------

char HexDigit(uint8_t Val) { return Val+(Val<10?'0':'A'-10); }

// ------------------------------------------------------------------------------------------

void Format_Bytes( void (*Output)(char), const uint8_t *Bytes, uint8_t Len)
{ for( ; Len; Len--)
    (*Output)(*Bytes++);
}

void Format_String( void (*Output)(char), const char *String)
{ for( ; ; )
  { uint8_t ch = (*String++); if(ch==0) break;
    if(ch=='\n') (*Output)('\r');
    (*Output)(ch); }
}

uint8_t Format_String(char *Str, const char *String)
{ uint8_t OutLen=0;
  for( ; ; )
  { char ch = (*String++); if(ch==0) break;
    if(ch=='\n') Str[OutLen++]='\r';
    Str[OutLen++]=ch; }
  return OutLen; }

uint8_t Format_String(char *Str, const char *String, uint8_t Len)
{ uint8_t OutLen=0;
  for(uint8_t Idx=0; Idx<Len; Idx++)
  { char ch = (*String++);
    if(ch=='\n') Str[OutLen++]='\r';
    Str[OutLen++]=ch; }
  return OutLen; }

void Format_Hex( void (*Output)(char), uint8_t Byte )
{ (*Output)(HexDigit(Byte>>4)); (*Output)(HexDigit(Byte&0x0F)); }

void Format_Hex( void (*Output)(char), uint16_t Word )
{ Format_Hex(Output, (uint8_t)(Word>>8)); Format_Hex(Output, (uint8_t)Word); }

void Format_Hex( void (*Output)(char), uint32_t Word )
{ Format_Hex(Output, (uint8_t)(Word>>24)); Format_Hex(Output, (uint8_t)(Word>>16));
  Format_Hex(Output, (uint8_t)(Word>>8));  Format_Hex(Output, (uint8_t)Word); }

void Format_UnsDec( void (*Output)(char), uint16_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ uint16_t Base; uint8_t Pos;
  for( Pos=5, Base=10000; Base; Base/=10, Pos--)
  { uint8_t Dig;
    if(Value>=Base)
    { Dig=Value/Base; Value-=Dig*Base; }
    else
    { Dig=0; }
    if(Pos==DecPoint) (*Output)('.');
    if( (Pos<=MinDigits) || (Dig>0) || (Pos<=DecPoint) )
    { (*Output)('0'+Dig); MinDigits=Pos; }
  }
}

void Format_SignDec( void (*Output)(char), int16_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ if(Value<0) { (*Output)('-'); Value=(-Value); }
         else { (*Output)('+'); }
  Format_UnsDec(Output, (uint16_t)Value, MinDigits, DecPoint); }

void Format_UnsDec( void (*Output)(char), uint32_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ uint32_t Base; uint8_t Pos;
  for( Pos=10, Base=1000000000; Base; Base/=10, Pos--)
  { uint8_t Dig;
    if(Value>=Base)
    { Dig=Value/Base; Value-=Dig*Base; }
    else
    { Dig=0; }
    if(Pos==DecPoint) (*Output)('.');
    if( (Pos<=MinDigits) || (Dig>0) || (Pos<=DecPoint) )
    { (*Output)('0'+Dig); MinDigits=Pos; }
  }
}

void Format_SignDec( void (*Output)(char), int32_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ if(Value<0) { (*Output)('-'); Value=(-Value); }
         else { (*Output)('+'); }
  Format_UnsDec(Output, (uint32_t)Value, MinDigits, DecPoint); }

// ------------------------------------------------------------------------------------------

uint8_t Format_UnsDec(char *Str, uint32_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ uint32_t Base; uint8_t Pos, Len=0;
  for( Pos=10, Base=1000000000; Base; Base/=10, Pos--)
  { uint8_t Dig;
    if(Value>=Base)
    { Dig=Value/Base; Value-=Dig*Base; }
    else
    { Dig=0; }
    if(Pos==DecPoint) { (*Str++)='.'; Len++; }
    if( (Pos<=MinDigits) || (Dig>0) || (Pos<=DecPoint) )
    { (*Str++)='0'+Dig; Len++; MinDigits=Pos; }
  }
  return Len; }

uint8_t Format_SignDec(char *Str, int32_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ if(Value<0) { (*Str++)='-'; Value=(-Value); }
         else { (*Str++)='+'; }
  return 1+Format_UnsDec(Str, Value, MinDigits, DecPoint); }

uint8_t Format_Hex( char *Output, uint8_t Byte )
{ (*Output++) = HexDigit(Byte>>4); (*Output++)=HexDigit(Byte&0x0F); return 2; }

uint8_t Format_Hex( char *Output, uint16_t Word )
{ Format_Hex(Output, (uint8_t)(Word>>8)); Format_Hex(Output+2, (uint8_t)Word); return 4; }

uint8_t Format_Hex( char *Output, uint32_t Word )
{ Format_Hex(Output  , (uint8_t)(Word>>24)); Format_Hex(Output+2, (uint8_t)(Word>>16));
  Format_Hex(Output+4, (uint8_t)(Word>> 8)); Format_Hex(Output+6, (uint8_t) Word     ); return 8; }

uint8_t Format_Hex( char *Output, uint32_t Word, uint8_t Digits)
{ for(uint8_t Idx=Digits; Idx>0; )
  { Output[--Idx]=HexDigit(Word&0x0F);
    Word>>=4; }
  return Digits; }

// ------------------------------------------------------------------------------------------

   int8_t Read_Dec1(char Digit)                   // convert single digit into an integer
   { if(Digit<'0') return -1;                     // return -1 if not a decimal digit
     if(Digit>'9') return -1;
     return Digit-'0'; }

   int8_t Read_Dec2(const char *Inp)              // convert two digit decimal number into an integer
   { int8_t High=Read_Dec1(Inp[0]); if(High<0) return -1;
     int8_t Low =Read_Dec1(Inp[1]); if(Low<0)  return -1;
     return Low+10*High; }

   int16_t Read_Dec3(const char *Inp)             // convert three digit decimal number into an integer
   { int8_t High=Read_Dec1(Inp[0]); if(High<0) return -1;
     int8_t Mid=Read_Dec1(Inp[1]);  if(Mid<0) return -1;
     int8_t Low=Read_Dec1(Inp[2]);  if(Low<0) return -1;
     return (int16_t)Low + (int16_t)10*(int16_t)Mid + (int16_t)100*(int16_t)High; }

   int16_t Read_Dec4(const char *Inp)             // convert three digit decimal number into an integer
   { int16_t High=Read_Dec2(Inp  ); if(High<0) return -1;
     int16_t Low =Read_Dec2(Inp+2); if(Low<0) return -1;
     return Low + (int16_t)100*(int16_t)High; }

