#ifndef  __FORMAT_H__
#define  __FORMAT_H__

// ------------------------------------------------------------------------------------------

char inline HexDigit(uint8_t Val) { return Val+(Val<10?'0':'A'-10); }

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

void Format_Hex( void (*Output)(char), uint8_t Byte )
{ (*Output)(HexDigit(Byte>>4)); (*Output)(HexDigit(Byte&0x0F)); }

void Format_Hex( void (*Output)(char), uint16_t Word )
{ Format_Hex(Output, (uint8_t)(Word>>8)); Format_Hex(Output, (uint8_t)Word); }

void Format_Hex( void (*Output)(char), uint32_t Word )
{ Format_Hex(Output, (uint8_t)(Word>>24)); Format_Hex(Output, (uint8_t)(Word>>16));
  Format_Hex(Output, (uint8_t)(Word>>8));  Format_Hex(Output, (uint8_t)Word); }

void Format_UnsDec( void (*Output)(char), uint16_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0)
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

void Format_SignDec( void (*Output)(char), int16_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0)
{ if(Value<0) { (*Output)('-'); Value=(-Value); }
         else { (*Output)('+'); }
  Format_UnsDec(Output, (uint16_t)Value, MinDigits, DecPoint); }

void Format_UnsDec( void (*Output)(char), uint32_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0)
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

void Format_SignDec( void (*Output)(char), int32_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0)
{ if(Value<0) { (*Output)('-'); Value=(-Value); }
         else { (*Output)('+'); }
  Format_UnsDec(Output, (uint32_t)Value, MinDigits, DecPoint); }

// ------------------------------------------------------------------------------------------

uint8_t Format_UnsDec(char *Str, uint32_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0)
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

uint8_t static Format_SignDec(char *Str, int32_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0)
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

// ------------------------------------------------------------------------------------------

#endif //  __FORMAT_H__
