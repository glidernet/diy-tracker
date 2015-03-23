#ifndef __LDPC_H__
#define __LDPC_H__

#include <stdint.h>
#include <stdlib.h>

#include "bitcount.h"

#ifndef __AVR__
#include <math.h>
#endif

#ifdef __AVR__
#include <avr/pgmspace.h>
#endif

// FindVectors65432bit(10,20,23, 500) => 208, Delta=2579

// every row represents a parity check to be performed on the received codeword
static const uint32_t LDPC_ParityCheck[48][7]
#ifdef __AVR__
PROGMEM
#endif
= { // parity check vectors: 48 vectors for 48 parity checks
    // Eaech vector applied to the data packet should yield even number of bits
 { 0x00000805, 0x00000020, 0x04000000, 0x20000000, 0x00000040, 0x00044020, 0x00000000 },
 { 0x00000001, 0x00800800, 0x00000000, 0x00000000, 0x00000000, 0x10010000, 0x00008C98 },
 { 0x00004001, 0x01000080, 0x80000400, 0x00000000, 0x08000200, 0x00200000, 0x00000005 },
 { 0x00000101, 0x20000200, 0x00000022, 0x00000000, 0x00000000, 0xCC008000, 0x00005002 },
 { 0x00000401, 0x00000000, 0x00004900, 0x00000020, 0x00000000, 0x20C00349, 0x00000020 },
 { 0x03140001, 0x00000002, 0x00000000, 0x40000001, 0x41534100, 0x00102C00, 0x00002000 },
 { 0x04008800, 0x82000642, 0x00000000, 0x00000020, 0x88040020, 0x03000010, 0x00000400 },
 { 0x00000802, 0x20000000, 0x02000014, 0x01200000, 0x04000403, 0x00800004, 0x0000A004 },
 { 0x02020820, 0x00000000, 0x80020820, 0x10190040, 0x30000000, 0x00000002, 0x00000900 },
 { 0x40804950, 0x00090000, 0x00000000, 0x00021204, 0x40001000, 0x10001100, 0x00000000 },
 { 0x08000A00, 0x00020008, 0x00040000, 0x02400010, 0x01002000, 0x40280280, 0x00000010 },
 { 0x00000000, 0x00008010, 0x118000A0, 0x00040080, 0x01000084, 0x00040100, 0x00000444 },
 { 0x20040108, 0x18000000, 0x08608800, 0x0000000A, 0x08000010, 0x00040080, 0x00008000 },
 { 0x00004080, 0x00422201, 0x00010000, 0x0000A400, 0x00400800, 0x00840000, 0x00000800 },
 { 0x00000000, 0x60200000, 0x80100240, 0x08000021, 0x02800000, 0x100C0000, 0x00000000 },
 { 0x00001000, 0x01010002, 0x00082001, 0x04000000, 0x00000001, 0x00040002, 0x00004030 },
 { 0x00002300, 0x04000000, 0xA0080000, 0x20004000, 0x00028000, 0x00800000, 0x00000400 },
 { 0x00004000, 0x00104100, 0x40041028, 0x24000020, 0x00200000, 0x00100000, 0x00008000 },
 { 0x08011000, 0x20040000, 0x00000000, 0xA0800000, 0x08090000, 0x00000100, 0x00000A00 },
 { 0x10180000, 0x00000204, 0x00002800, 0x20400800, 0x00000000, 0x10000000, 0x00000004 },
 { 0x00000000, 0xC0000000, 0x10200000, 0x20028000, 0x20000000, 0x80000008, 0x00002011 },
 { 0x82004000, 0x20000000, 0x04202000, 0x00000000, 0x00000000, 0x00020200, 0x00000400 },
 { 0x08600000, 0x00001200, 0x94000000, 0x00000000, 0x40000008, 0x00000000, 0x00008020 },
 { 0x04040000, 0x04010000, 0x04100000, 0x00000100, 0x00200000, 0x40000008, 0x00000804 },
 { 0x00000200, 0x00000110, 0x04000100, 0x00000000, 0x28400400, 0x10000000, 0x00004000 },
 { 0x00080000, 0x00000080, 0x04001000, 0x01882007, 0x00008024, 0x04000001, 0x00000010 },
 { 0x20200000, 0x00000020, 0x00010040, 0x81000800, 0x10001000, 0x00300008, 0x00004400 },
 { 0x90000010, 0x89841021, 0x00000118, 0x08080000, 0x00020000, 0x40000000, 0x00000040 },
 { 0x04C20000, 0x10404034, 0x00000000, 0x00004000, 0x00810001, 0x04000200, 0x00000009 },
 { 0x40102000, 0x020020A0, 0x40100000, 0x00100080, 0x00080400, 0x80030080, 0x00000020 },
 { 0x00010000, 0x04020920, 0x00000200, 0x00060000, 0x00000218, 0x01002007, 0x00001000 },
 { 0x00020008, 0x00A08040, 0x00080000, 0x40001400, 0x04200040, 0x80200001, 0x00000200 },
 { 0x40000402, 0x01100000, 0x20808000, 0x00008000, 0x10100060, 0x00080000, 0x00001008 },
 { 0x200010A0, 0x00000000, 0x01040100, 0x00000104, 0x02040042, 0x08012000, 0x00000001 },
 { 0x01000000, 0x50000880, 0x00000092, 0x14400000, 0x00001840, 0x02400000, 0x00000000 },
 { 0x00000010, 0x02000000, 0x00014000, 0x00200018, 0x00000240, 0x04000800, 0x00000180 },
 { 0x00008000, 0x00880008, 0x08000044, 0x00100000, 0x00000004, 0x00400820, 0x00001001 },
 { 0x01000000, 0x00002000, 0x02004001, 0x00000042, 0x00000000, 0x09201020, 0x00000048 },
 { 0x00800000, 0x01000400, 0x00400002, 0xC0002000, 0x00002080, 0x00010064, 0x00000100 },
 { 0x00000400, 0x08400840, 0x00000400, 0x00000890, 0x00008102, 0x00000020, 0x00000002 },
 { 0x00200040, 0x00000081, 0x00000000, 0x02050000, 0x04940000, 0x20008020, 0x00000080 },
 { 0x00000404, 0x00800000, 0x00001000, 0x00014000, 0x00082200, 0x0A000400, 0x00000000 },
 { 0x0000A024, 0x00000000, 0x00000402, 0x08A01000, 0x00004010, 0x20000000, 0x00000008 },
 { 0x00480046, 0x00008000, 0x00000208, 0x00000048, 0x00000000, 0x00410010, 0x00000002 },
 { 0x0000008C, 0x00044C00, 0x00824004, 0x00000200, 0x00000000, 0x00028000, 0x00000000 },
 { 0x10010004, 0x00080000, 0x43008000, 0x10000400, 0x80000100, 0x00000040, 0x00000080 },
 { 0x80000000, 0x0020000C, 0x20420480, 0x00000100, 0x00000008, 0x00005410, 0x00000080 },
 { 0x00000000, 0x00101000, 0x08000001, 0x02000200, 0x82004A80, 0x00004000, 0x00000202 }
} ;


static const uint8_t LDPC_ParityCheckIndex[48][24]
#ifdef __AVR__
PROGMEM
#endif
= { // number of, indicies to bits to be taken for parity checks
 { 10,   0,   2,  11,  37,  90, 125, 134, 165, 174, 178 },
 { 11,   0,  43,  55, 176, 188, 195, 196, 199, 202, 203, 207 },
 { 11,   0,  14,  39,  56,  74,  95, 137, 155, 181, 192, 194, },
 { 14,   0,   8,  41,  61,  65,  69, 175, 186, 187, 190, 191, 193, 204, 206 },
 { 15,   0,  10,  72,  75,  78, 101, 160, 163, 166, 168, 169, 182, 183, 189, 197 },
 { 21,   0,  18,  20,  24,  25,  33,  96, 126, 136, 142, 144, 145, 148, 150, 152, 158, 170, 171, 173, 180, 205 },
 { 18,  11,  15,  26,  33,  38,  41,  42,  57,  63, 101, 133, 146, 155, 159, 164, 184, 185, 202 },
 { 17,   1,  11,  61,  66,  68,  89, 117, 120, 128, 129, 138, 154, 162, 183, 194, 205, 207 },
 { 18,   5,  11,  17,  25,  69,  75,  81,  95, 102, 112, 115, 116, 124, 156, 157, 161, 200, 203 },
 { 18,   4,   6,   8,  11,  14,  23,  30,  48,  51,  98, 105, 108, 113, 140, 158, 168, 172, 188 },
 { 17,   9,  11,  27,  35,  49,  82, 100, 118, 121, 141, 152, 167, 169, 179, 181, 190, 196 },
 { 17,  36,  47,  69,  71,  87,  88,  92, 103, 114, 130, 135, 152, 168, 178, 194, 198, 202 },
 { 18,   3,   8,  18,  29,  59,  60,  75,  79,  85,  86,  91,  97,  99, 132, 155, 167, 178, 207 },
 { 16,   7,  14,  32,  41,  45,  49,  54,  80, 106, 109, 111, 139, 150, 178, 183, 203 },
 { 15,  53,  61,  62,  70,  73,  84,  95,  96, 101, 123, 151, 153, 178, 179, 188 },
 { 14,  12,  33,  48,  56,  64,  77,  83, 122, 128, 161, 178, 196, 197, 206 },
 { 13,   8,   9,  13,  58,  83,  93,  95, 110, 125, 143, 145, 183, 202 },
 { 15,  14,  40,  46,  52,  67,  69,  76,  82,  94, 101, 122, 125, 149, 180, 207 },
 { 14,  12,  16,  27,  50,  61, 119, 125, 127, 144, 147, 155, 168, 201, 203 },
 { 12,  19,  20,  28,  34,  41,  75,  77, 107, 118, 125, 188, 194 },
 { 13,  62,  63,  85,  92, 111, 113, 125, 157, 163, 191, 192, 196, 205 },
 { 10,  14,  25,  31,  61,  77,  85,  90, 169, 177, 202 },
 { 12,  21,  22,  27,  41,  44,  90,  92,  95, 131, 158, 197, 207 },
 { 12,  18,  26,  48,  58,  84,  90, 104, 149, 163, 190, 194, 203 },
 { 11,   9,  36,  40,  72,  90, 138, 150, 155, 157, 188, 206 },
 { 17,  19,  39,  76,  90,  96,  97,  98, 109, 115, 119, 120, 130, 133, 143, 160, 186, 196 },
 { 15,  21,  29,  37,  70,  80, 107, 120, 127, 140, 156, 163, 180, 181, 202, 206 },
 { 19,   4,  28,  31,  32,  37,  44,  50,  55,  56,  59,  63,  67,  68,  72, 115, 123, 145, 190, 198 },
 { 18,  17,  22,  23,  26,  34,  36,  37,  46,  54,  60, 110, 128, 144, 151, 169, 186, 192, 195 },
 { 18,  13,  20,  30,  37,  39,  45,  57,  84,  94, 103, 116, 138, 147, 167, 176, 177, 191, 197 },
 { 18,  16,  37,  40,  43,  49,  58,  73, 113, 114, 131, 132, 137, 160, 161, 162, 173, 184, 204 },
 { 17,   3,  17,  38,  47,  53,  55,  83, 106, 108, 126, 134, 149, 154, 160, 181, 191, 201 },
 { 16,   1,  10,  30,  52,  56,  79,  87,  93, 111, 133, 134, 148, 156, 179, 195, 204 },
 { 17,   5,   7,  12,  29,  72,  82,  88,  98, 104, 129, 134, 146, 153, 173, 176, 187, 192 },
 { 16,  24,  39,  43,  60,  62,  65,  68,  71, 118, 122, 124, 134, 139, 140, 182, 185 },
 { 13,   4,  57,  78,  80,  99, 100, 117, 134, 137, 171, 186, 199, 200 },
 { 14,  15,  35,  51,  55,  66,  70,  91, 116, 130, 165, 171, 182, 192, 204 },
 { 14,  24,  45,  64,  78,  89,  97, 102, 165, 172, 181, 184, 187, 195, 198 },
 { 15,  23,  42,  56,  65,  86, 109, 126, 127, 135, 141, 162, 165, 166, 176, 200 },
 { 14,  10,  38,  43,  54,  59,  74, 100, 103, 107, 129, 136, 143, 165, 193 },
 { 15,   6,  21,  32,  39, 112, 114, 121, 146, 148, 151, 154, 165, 175, 189, 199 },
 { 12,   2,  10,  55,  76, 110, 112, 137, 141, 147, 170, 185, 187 },
 { 14,   2,   5,  13,  15,  65,  74, 108, 117, 119, 123, 132, 142, 189, 195 },
 { 14,   1,   2,   6,  19,  22,  47,  67,  73,  99, 102, 164, 176, 182, 193 },
 { 14,   2,   3,   7,  42,  43,  46,  50,  66,  78,  81,  87, 105, 175, 177 },
 { 14,   2,  16,  28,  51,  79,  88,  89,  94, 106, 124, 136, 159, 166, 199 },
 { 16,  31,  34,  35,  53,  71,  74,  81,  86,  93, 104, 131, 164, 170, 172, 174, 199 },
 { 15,  44,  52,  64,  91, 105, 121, 135, 137, 139, 142, 153, 159, 174, 193, 201 }
  } ;

static const uint8_t LDPC_BitWeight[208] // weight of parity checks for every codeword bit
#ifdef __AVR__
PROGMEM
#endif
= { 6, 3, 6, 3, 3, 3, 3, 3, 4, 3, 4, 6, 3, 3, 5, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    3, 3, 3, 3, 3, 6, 3, 5, 3, 5, 3, 5, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 5, 5, 3, 3, 3, 3, 5, 3, 3,
    3, 4, 3, 3, 3, 4, 3, 3, 4, 3, 4, 4, 3, 3, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 6, 3, 3, 3, 3, 5,
    3, 3, 3, 3, 3, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 6, 3, 3,
    3, 3, 3, 3, 3, 3, 6, 3, 3, 5, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 5, 3, 3, 3, 3,
    4, 3, 3, 4, 3, 6, 3, 3, 4, 4, 3, 3, 3, 3, 3, 3, 5, 3, 6, 3, 3, 5, 4, 4, 3, 3, 4, 4, 5, 3, 4, 4,
    5, 4, 5, 5, 5, 4, 3, 5, 3, 3, 6, 5, 4, 3, 4, 5 } ;

// every row represents the generator for a parity bit
static const uint32_t LDPC_ParityGen[48][5] 
#ifdef __AVR__
PROGMEM
#endif
= { // Parity bits generator: 48 vectors to generate 48 parity bits
    // Each vector applied to the user data yields a corresponding parity bit
 { 0x40A90281, 0x9159D249, 0xCE9D516B, 0x2FDEED0B, 0xD9267CD4  },
 { 0xCCBC0FC3, 0xCC4FA4BC, 0x811EC3D0, 0xB07EC1B3, 0xA3B8E8D8  },
 { 0x66418D56, 0x3B85ADFF, 0xD2A6532E, 0x48CF52E4, 0x6A16586D  },
 { 0x44C71240, 0x2C94631F, 0x15F15A4A, 0x7459D901, 0x037863CC  },
 { 0x7386D718, 0x7F6C9623, 0x738E2E0C, 0xD2351593, 0xEF358669  },
 { 0x7BF87232, 0x9E4CCD68, 0xBB82590E, 0x9C9292EA, 0x4CE2AEB9  },
 { 0xAA8436BC, 0x94A61C4D, 0x1DA89B11, 0x72EAF204, 0x34D3A041  },
 { 0xBCE77760, 0x229935B2, 0xAAF85CE3, 0xFFE7B602, 0xF26BCC64  },
 { 0xD0C371D0, 0xA553D12F, 0xA0685BF2, 0x5C553C81, 0x0218EB48  },
 { 0x8D29034D, 0xEB20A394, 0x1A8C82A3, 0x41B4DA0C, 0x8632F81E  },
 { 0x15A50876, 0x9BC10F59, 0xF979D1E0, 0xCFF6BD88, 0x88FE5895  },
 { 0x037A9ED5, 0xFA5DB837, 0x61395ACA, 0xE65B5839, 0x9A2D9D02  },
 { 0xEE70D18F, 0x8AE909C6, 0x8AF5BECA, 0x66968559, 0x1BD9B5E7  },
 { 0xC56397AC, 0xD8FF6A30, 0x8E165AF3, 0xC01686B9, 0xEC26BEDC  },
 { 0xB2D63859, 0xFACA8CFD, 0x7EE85EB7, 0x19BFDA46, 0xC8C1CA52  },
 { 0xD7EB4D94, 0x426104DA, 0x124FBD54, 0xBF610A1D, 0x0E615094  },
 { 0x68EFE180, 0x15A1549C, 0x18D20289, 0xBD28AD44, 0x8DADDAEC  },
 { 0xD7EB4D18, 0x426548DA, 0x12CDFD50, 0xBF61081D, 0x0E615094  },
 { 0xC92E426E, 0x648641B5, 0xC16A07B9, 0xA52D48AC, 0x842364AB  },
 { 0xB71DAB61, 0x2B15995C, 0x6BE7E0C1, 0x97ECE351, 0xDF622A04  },
 { 0x55CD7406, 0x5E0F3507, 0x23F6C372, 0x7ECAFE84, 0x7E68A8DF  },
 { 0x97DB831C, 0xD46D648F, 0x14FA22B3, 0x4F875648, 0x94C23936  },
 { 0x60D940EC, 0xFCC18797, 0xD0DE7383, 0xF38F22E5, 0x2E7A733E  },
 { 0xD8C22D55, 0x8D45EB4E, 0xAC695FF3, 0xDED59211, 0x8851288A  },
 { 0xCE9D11A1, 0xD8E8F438, 0xAF3102EE, 0xCB2FE547, 0xC11845BD  },
 { 0x61D940EC, 0xACC18F17, 0xD0DE7311, 0xE7CF22E5, 0x2E7A6B7E  },
 { 0x66AD8025, 0x493D883C, 0x538E9261, 0x5F0E116B, 0xB17492FA  },
 { 0x747C4C9E, 0x3780804E, 0x29A7B2F1, 0x2838DF6D, 0xA68C11EB  },
 { 0x7E33E90F, 0x2FB3D8E9, 0x2A9DE538, 0x3AC1ABDC, 0x59C14EAF  },
 { 0x16B6095E, 0x4883D57E, 0xF765FF4B, 0x431C6EF3, 0xF2C45F6C  },
 { 0x3F04D4F4, 0xEEA73108, 0x567ECF38, 0x15200560, 0x56AB6942  },
 { 0x1E5ECFFE, 0x29426F53, 0x17057060, 0xA774ED7F, 0x4FE7EACB  },
 { 0xF9F02A12, 0xFADEBEE2, 0xBE67EB8B, 0x5506F594, 0xC5037599  },
 { 0x7BF87632, 0x960CC528, 0xBB825D0E, 0x9C929A7A, 0x4CE22FBB  },
 { 0x6E2BE90F, 0x2FB3DAED, 0x2A9DCD38, 0x1A81A3DC, 0x59C14EAF  },
 { 0x16B6A97A, 0x4883D57E, 0xF765FB49, 0x4BBC7EF3, 0xF2C41F7C  },
 { 0x260C82A4, 0xD8645AF5, 0x9913D30A, 0x7158DC67, 0x68526E0A  },
 { 0x5DAD3406, 0x5E1F6607, 0xF7F2D35A, 0x5ACAFEA4, 0x3E48A8D7  },
 { 0xAF04D4E4, 0x67232129, 0x567ECE20, 0x1D280560, 0x56A96942  },
 { 0xBA8536B8, 0x94AE1C4D, 0x5EA81B11, 0x62EAF604, 0xB4D3A141  },
 { 0xDF522858, 0x25CE2C46, 0x6C1E93BA, 0xDB9FBF4E, 0x9F8AACF9  },
 { 0xC92E4E6B, 0x6CD659D5, 0xCD6A03B8, 0x872D423C, 0x0623AF69  },
 { 0xD8C20E55, 0x8945EB4E, 0x0C615FF3, 0xFED5D211, 0x8853A88A  },
 { 0x11EC2FBB, 0xE98188FA, 0x6D02584A, 0x7BF87EBD, 0x0C324421  },
 { 0xE1AB0619, 0x62864C22, 0xBC029B88, 0xDC501DA2, 0x3DB63518  },
 { 0x85657508, 0xE76CE85B, 0x35A012AB, 0xD7719D8D, 0xC1CE9294  },
 { 0x7E33EB0F, 0x2FB3D9F9, 0x2E9DE438, 0x3AC1ABDC, 0x71814AAF  },
 { 0x55CD3406, 0x5E1F7407, 0x63F2D35A, 0x5ACAFEA4, 0x7E48A8DF  }
} ;

#ifdef __AVR__

// encode Parity from Data: Data is 5x 32-bit words = 160 bits, Parity is 1.5x 32-bit word = 48 bits
void LDPC_Encode(const uint32_t *Data, uint32_t *Parity, const uint32_t ParityGen[48][5])
{ uint8_t ParIdx=0; Parity[ParIdx]=0; uint32_t Mask=1;
  for(uint8_t Row=0; Row<48; Row++)
  { uint8_t Count=0;
    const uint32_t *Gen=ParityGen[Row];
    for(uint8_t Idx=0; Idx<5; Idx++)
    { Count+=Count1s(Data[Idx]&pgm_read_dword(Gen+Idx)); }
    if(Count&1) Parity[ParIdx]|=Mask; Mask<<=1;
    if(Mask==0) { ParIdx++; Parity[ParIdx]=0; Mask=1; }
  }
}

void LDPC_Encode(const uint32_t *Data, uint32_t *Parity)
{ LDPC_Encode(Data, Parity, LDPC_ParityGen); }

// encode Parity from Data: Data is 20 bytes = 160 bits, Parity is 6 bytes = 48 bits
void LDPC_Encode(const uint8_t *Data, uint8_t *Parity, const uint32_t ParityGen[48][5])
{ uint8_t ParIdx=0; Parity[ParIdx]=0; uint8_t Mask=1;
  for(uint8_t Row=0; Row<48; Row++)
  { uint8_t Count=0;
    const uint8_t *Gen = (uint8_t *)ParityGen[Row];
    for(uint8_t Idx=0; Idx<20; Idx++)
    { Count+=Count1s(Data[Idx]&pgm_read_byte(Gen+Idx)); }
    if(Count&1) Parity[ParIdx]|=Mask; Mask<<=1;
    if(Mask==0) { ParIdx++; Parity[ParIdx]=0; Mask=1; }
  }
}

void LDPC_Encode(const uint8_t *Data, uint8_t *Parity)
{ LDPC_Encode(Data, Parity, LDPC_ParityGen); }

void LDPC_Encode(uint8_t *Data)
{ LDPC_Encode(Data, Data+20, LDPC_ParityGen); }

// check Data against Parity (run 48 parity checks) - return number of failed checks
int8_t LDPC_Check(const uint8_t *Data) // 20 data bytes followed by 6 parity bytes
{ uint8_t Errors=0;
  for(uint8_t Row=0; Row<48; Row++)
  { uint8_t Count=0;
    const uint8_t *Check = (uint8_t *)LDPC_ParityCheck[Row];
    for(uint8_t Idx=0; Idx<26; Idx++)
    { Count+=Count1s(Data[Idx] & pgm_read_byte(Check+Idx)); }
    if(Count&1) Errors++; }
  return Errors; }

int8_t LDPC_Check(const uint32_t *Packet)
{ return LDPC_Check( (uint8_t *)Packet ); }

#else // if not 8-bit AVR

void LDPC_Encode(const uint8_t *Data, uint8_t *Parity, const uint32_t ParityGen[48][5])
{ uint8_t ParIdx=0; uint8_t ParByte=0; uint8_t Mask=1;
  for(uint8_t Row=0; Row<48; Row++)
  { uint8_t Count=0;
    const uint8_t *Gen = (uint8_t *)(ParityGen[Row]);
    for(uint8_t Idx=0; Idx<20; Idx++)
    { Count+=Count1s((uint8_t)(Data[Idx]&Gen[Idx])); }
    if(Count&1) ParByte|=Mask; Mask<<=1;
    if(Mask==0) { Parity[ParIdx++]=ParByte; Mask=1; ParByte=0; }
  }
  // if(Mask!=1) Parity[ParIdx]=ParByte;
}

void LDPC_Encode(const uint8_t *Data, uint8_t *Parity)
{ LDPC_Encode(Data, Parity, LDPC_ParityGen); }

void LDPC_Encode(uint8_t *Data)
{ LDPC_Encode(Data, Data+20); }

// encode Parity from Data: Data is 5x 32-bit words = 160 bits, Parity is 1.5x 32-bit word = 48 bits
void LDPC_Encode(const uint32_t *Data, uint32_t *Parity, const uint32_t ParityGen[48][5])
{ // printf("LDPC_Encode: %08X %08X %08X %08X %08X", Data[0], Data[1], Data[2], Data[3], Data[4] );
  uint8_t ParIdx=0; Parity[ParIdx]=0; uint32_t Mask=1;
  for(uint8_t Row=0; Row<48; Row++)
  { uint8_t Count=0;
    const uint32_t *Gen=ParityGen[Row];
    for(uint8_t Idx=0; Idx<5; Idx++)
    { Count+=Count1s(Data[Idx]&Gen[Idx]); }
    if(Count&1) Parity[ParIdx]|=Mask; Mask<<=1;
    if(Mask==0) { ParIdx++; Parity[ParIdx]=0; Mask=1; }
  }
  // printf(" => %08X %08X\n", Parity[0], Parity[1] );
}

void LDPC_Encode(const uint32_t *Data, uint32_t *Parity)
{ LDPC_Encode(Data, Parity, LDPC_ParityGen); }

// check Data against Parity (run 48 parity checks) - return number of failed checks
int LDPC_Check(const uint32_t *Data, const uint32_t *Parity) // Data and Parity are 32-bit words
{ int Errors=0;
  for(int Row=0; Row<48; Row++)
  { int Count=0;
    const uint32_t *Check=LDPC_ParityCheck[Row];
    int Idx;
    for(Idx=0; Idx<5; Idx++)
    { Count+=Count1s(Data[Idx]&Check[Idx]); }
    Count+=Count1s(Parity[0]&Check[Idx++]);
    Count+=Count1s((Parity[1]&Check[Idx++])&0xFFFF);
    if(Count&1) Errors++; }
  return Errors; }

int LDPC_Check(const uint32_t *Packet) { return LDPC_Check(Packet, Packet+5); }
/*
int LDPC_Check(const uint8_t *Data) // Data and Parity are 8-bit bytes
{ int Errors=0;
  for(int Row=0; Row<48; Row++)
  { int Count=0;
    const uint32_t *Check=LDPC_ParityCheck[Row];
    uint32_t ParityWord = 0;
    for(int Idx=0; Idx<26; Idx++)
    { if((Idx&3)==0) { ParityWord = *Check++; }
      uint8_t And=Data[Idx]&ParityWord; Count+=Count1s(And);
      ParityWord>>=8; }
    if(Count&1) Errors++; }
  return Errors; }
*/
int8_t LDPC_Check(const uint8_t *Data) // 20 data bytes followed by 6 parity bytes
{ uint8_t Errors=0;
  for(uint8_t Row=0; Row<48; Row++)
  { uint8_t Count=0;
    const uint8_t *Check = (uint8_t *)LDPC_ParityCheck[Row];
    for(uint8_t Idx=0; Idx<26; Idx++)
    { uint8_t And = Data[Idx]&Check[Idx]; Count+=Count1s(And); }
    if(Count&1) Errors++; }
  return Errors; }
  
#endif // __AVR__

#ifndef __AVR__

class LDPC_Decoder
{ public:
   const static uint8_t UserBits   = 160;                 // 5 32-bit bits = 20 bytes
   const static uint8_t UserWords  = UserBits/32;
   const static uint8_t ParityBits =  48;                 // 6 bytes (total packet is 26 bytes)
   const static uint8_t CodeBits   = UserBits+ParityBits; // 160+48 = 208 code bits = 26 bytes
   const static uint8_t CodeBytes  = (CodeBits+ 7)/ 8;    //
   const static uint8_t CodeWords  = (CodeBits+31)/32;    // 
   const static uint8_t MaxCheckWeight = 24;
   // const static uint8_t MaxBitWeight   =  8;

  public:
/*
   uint8_t BitWeight[CodeBits];                           // How many checks involved for every bit
   uint8_t CheckWeight[ParityBits];                       // How many bits in every parity check
   uint8_t ParityCheck[ParityBits][MaxCheckWeight];       // For every parity check: list of bits to be taken from the codeword
*/
  public:
/*
   LDPC_Decoder() { Configure(); }

   int8_t Configure(void) { return Configure(LDPC_ParityCheck); }

   int8_t Configure(const uint32_t ParityCheckConfig[ParityBits][CodeWords])
   { for(uint8_t Bit=0; Bit<CodeBits; Bit++)
       BitWeight[Bit]=0;
     for(uint8_t Row=0; Row<ParityBits; Row++)
     { uint8_t Weight=0;
       uint32_t Mask=1; uint8_t Idx=0; uint32_t Word=ParityCheckConfig[Row][Idx];
       for(uint8_t Bit=0; Bit<CodeBits; Bit++)
       { if(Word&Mask) { ParityCheck[Row][Weight++]=Bit; BitWeight[Bit]++; }
         Mask<<=1; if(Mask==0) { Idx++; Word=ParityCheckConfig[Row][Idx]; Mask=1; }
       }
       CheckWeight[Row]=Weight;
     }
     // for(int Bit=0; Bit<CodeBits; Bit++)
     // { printf(BitWeight[Bit]); }
     return 0; }

   void PrintCode(void)
   { printf("LDPC_ParityCheckIndex[%d][%d] = {\n", ParityBits, MaxCheckWeight);
     for(uint8_t Row=0; Row<ParityBits; Row++)
     { printf(" {");
       for(uint8_t Bit=0; Bit<CheckWeight[Row]; Bit++)
       { printf(" %3d,", ParityCheck[Row][Bit]); }
       printf(" },\n"); }
     printf(" } ;\n");
   }

   void PrintConfig(void) const
   { printf("LDPC_Decoder[%d,%d] Check index table:\n", UserBits, CodeBits);
     for(uint8_t Row=0; Row<ParityBits; Row++)
     { printf("Check[%d]:", CheckWeight[Row]);
       for(uint8_t Bit=0; Bit<CheckWeight[Row]; Bit++)
       { printf(" %3d", ParityCheck[Row][Bit]); }
       printf("\n");
     }
     printf("BitWeight[%d]:\n", CodeBits);
     uint8_t Bit;
     for(Bit=0; Bit<CodeBits; Bit++)
     { if((Bit&0x1F)==0x00) printf("%03d:", Bit);
       printf(" %d", BitWeight[Bit]);
       if((Bit&0x1F)==0x1F) printf("\n"); }
     if((Bit&0x1F)!=0x00) printf("\n");
   }
*/
  public:

   int8_t  InpBit[CodeBits]; // a-priori bits
   int16_t OutBit[CodeBits]; // a-posteriori bits

   void PrintInpBit(void)
   { printf("InpBit[%d]\n", CodeBits);
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { if((Bit&0xF)==0x0) printf("%03d:", Bit);
       printf(" %+5d", InpBit[Bit]);
       if((Bit&0xF)==0xF) printf("\n"); }
   }

   void Input(const uint8_t *Data, uint8_t *Err)
   { uint8_t Mask=1; uint8_t Idx=0; uint8_t DataByte=0; uint8_t ErrByte=0;
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { if(Mask==1) { DataByte=Data[Idx];  ErrByte=Err[Idx]; }
       int8_t Inp;
       if(ErrByte&Mask) Inp=0;
                   else Inp=(DataByte&Mask) ? +32:-32;
       InpBit[Bit] = Inp;
       Mask<<=1; if(Mask==0) { Idx++; Mask=1; }
     }
   }

   void Input(const uint32_t Data[CodeWords])
   { uint32_t Mask=1; uint8_t Idx=0; uint32_t Word=Data[Idx];
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { InpBit[Bit] = (Word&Mask) ? +32:-32;
       Mask<<=1; if(Mask==0) { Word=Data[++Idx]; Mask=1; }
     }
   }

   void Input(const float *Data, float RefAmpl=1.0)
   { for(int Bit=0; Bit<CodeBits; Bit++)
     { int Inp = floor(32*Data[Bit^7]/RefAmpl+0.5);
       if(Inp>127) Inp=127; else if(Inp<(-127)) Inp=(-127);
       InpBit[Bit] = Inp; }
   }

   void Output(uint32_t Data[CodeWords])
   { uint32_t Mask=1; uint8_t Idx=0; uint32_t Word=0;
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { if(InpBit[Bit]>0) Word|=Mask;
       Mask<<=1; if(Mask==0) { Data[Idx++]=Word; Word=0; Mask=1; }
     } if(Mask>1) Data[Idx++]=Word;
   }

   void Output(uint8_t Data[CodeBytes])
   { uint8_t Mask=1; uint8_t Idx=0; uint8_t Byte=0;
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { if(InpBit[Bit]>0) Byte|=Mask;
       Mask<<=1; if(Mask==0) { Data[Idx++]=Byte; Byte=0; Mask=1; }
     } if(Mask>1) Data[Idx++]=Byte;
   }

   int8_t ProcessChecks(void)
   { for(uint8_t Bit=0; Bit<CodeBits; Bit++)
       OutBit[Bit]=0;
     uint8_t Count=0;
     for(uint8_t Row=0; Row<ParityBits; Row++)
     { int8_t Ret=ProcessCheck(Row); 
       if(Ret<=0) Count++; }
     // printf("%d parity checks fail\n", Count);
     if(Count==0) return 0;
     for(uint8_t Bit=0; Bit<CodeBits; Bit++)
     { int16_t Ampl = ((uint16_t)4*InpBit[Bit] + OutBit[Bit])/(uint16_t)5;
       if(Ampl<(-128)) Ampl=(-127);
       else if(Ampl>127) Ampl=127;
       InpBit[Bit] = Ampl; }
     // PrintInpBit();
     return Count; }

   int8_t ProcessCheck(uint8_t Row)
   { int8_t MinAmpl=127; uint8_t MinBit=0; int8_t MinAmpl2=127; uint32_t Word=0; uint32_t Mask=1;
     const uint8_t *CheckIndex = LDPC_ParityCheckIndex[Row];
     uint8_t CheckWeight = *CheckIndex++;
     for(uint8_t Bit=0; Bit<CheckWeight; Bit++)
     { uint8_t BitIdx=CheckIndex[Bit]; int8_t Ampl=InpBit[BitIdx];
       if(Ampl>0) Word|=Mask;
       Mask<<=1;
       if(Ampl<0) Ampl=(-Ampl);
       if(Ampl<MinAmpl) { MinAmpl2=MinAmpl; MinAmpl=Ampl; MinBit=Bit; }
       else if(Ampl<MinAmpl2) { MinAmpl2=Ampl; }
     }
     uint8_t CheckFails = Count1s(Word)&1;
     Mask=1;
     for(uint8_t Bit=0; Bit<CheckWeight; Bit++)
     { uint8_t BitIdx=CheckIndex[Bit];
       int8_t Ampl = Bit==MinBit ? MinAmpl2 : MinAmpl;
       if(CheckFails) Ampl=(-Ampl);
       // if( (BitIdx==166) || (BitIdx==183) )
       //   printf("OutBit[%d] += %+d\n", BitIdx, (Word&Mask) ? Ampl:-Ampl);
       OutBit[BitIdx] += (Word&Mask) ? Ampl:-Ampl;
       Mask<<=1; }
     return CheckFails?-MinAmpl:MinAmpl; }

} ;

#endif // __AVR__

#endif // of __LDPC_H__
