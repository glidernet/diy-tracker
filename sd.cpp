// the "driver" for the microSD card to make FatFS (ff.c) work.

#include <stdint.h>

#include "spi2.h"
#include "diskio.h"

// here is the "glue" to the SPI interface
#define spi_init    SPI2_Configuration     // SPI initialize
#define spi_fast    SPI2_Fast              // switch to a fast SPI clock
#define spi_slow    SPI2_Slow              // switch to a slow SPI clock
#define spi_cs_low  SPI2_Select            // SPI Chip-Select control for the SD card reader
#define spi_cs_high SPI2_Deselect          // 
#define spi_txrx    SPI2_TransferByte      // SPI byte exchange

// Command definitions in SPI bus mode 
#define GO_IDLE_STATE           0
#define SEND_OP_COND            1
#define SWITCH_FUNC             6
#define SEND_IF_COND            8
#define SEND_CSD                9
#define SEND_CID                10
#define STOP_TRANSMISSION       12
#define SEND_STATUS             13
#define SET_BLOCKLEN            16
#define READ_SINGLE_BLOCK       17
#define READ_MULTIPLE_BLOCK     18
#define WRITE_SINGLE_BLOCK      24
#define WRITE_MULTIPLE_BLOCK    25
#define APP_CMD                 55
#define READ_OCR                58
#define CRC_ON_OFF              59

/* Application specific commands supported by SD.
All these commands shall be preceded with APP_CMD (CMD55). */
#define SD_STATUS               13
#define SD_SEND_OP_COND         41

// R1 response bit flag definition
#define R1_NO_ERROR         0x00
#define R1_IN_IDLE_STATE    0x01
#define R1_ERASE_RESET      0x02
#define R1_ILLEGAL_CMD      0x04
#define R1_COM_CRC_ERROR    0x08
#define R1_ERASE_SEQ_ERROR  0x10
#define R1_ADDRESS_ERROR    0x20
#define R1_PARA_ERROR       0x40
#define R1_MASK             0x7F

const uint8_t  sd_head_bytes =    10;   // [SPI bytes] send before CS=LOW
const uint8_t  sd_tail_bytes =    10;   // [SPI bytes] send before CS=HIGH
const uint8_t  sd_resp_wait  =    20;   // [SPI bytes] wait for R1/R2/R7 response
const uint16_t sd_data_wait  = 60000;   // [SPI bytes] wait for data block read/write

static uint8_t crc7(uint8_t crc, uint8_t byte)
{ const uint8_t g = 0x89;
  crc ^= byte;
  for (uint8_t i=0; i<8; i++)
  { if (crc & 0x80) crc ^= g;
    crc <<= 1; }
  return crc; }

static uint8_t crc7(const uint8_t *data, int len)          // CRC for data blocks
{ uint8_t crc = 0;
  for (int j=0; j<len; j++)
  { crc = crc7(crc, data[j]); }
  return crc>>1; }

static uint16_t crc16_ccitt(uint16_t crc, uint8_t byte)
{ crc  = (uint8_t)(crc >> 8) | (crc << 8);
  crc ^= byte;
  crc ^= (uint8_t)(crc & 0xff) >> 4;
  crc ^= (crc << 8) << 4;
  crc ^= ((crc & 0xff) << 4) << 1;
  return crc; }

static uint16_t crc16(const u8 *data, int len)   // CRC for commands
{ uint16_t crc = 0;
  for (int i=0; i<len; i++)
  { crc = crc16_ccitt(crc, data[i]); }
  return crc; }

// http://elm-chan.org/docs/mmc/mmc_e.html

static void sd_cmd(uint8_t cmd, uint32_t arg=0)  // send command+argument+CRC to the SD card (CS already active)
{ uint8_t crc = 0; uint8_t byte;
  byte = 0x40 | cmd; spi_txrx(byte); crc = crc7(crc, byte);
  byte = arg >> 24;  spi_txrx(byte); crc = crc7(crc, byte);
  byte = arg >> 16;  spi_txrx(byte); crc = crc7(crc, byte);
  byte = arg >>  8;  spi_txrx(byte); crc = crc7(crc, byte);
  byte = arg      ;  spi_txrx(byte); crc = crc7(crc, byte);
  spi_txrx(crc | 0x01); }

static uint8_t sd_get_r1(void)                  // wait for the R1: 1-byte response
{ uint8_t resp=0xff;                            // keep sending the clock with data high
  for(uint8_t wait=sd_resp_wait; wait; wait--)
  { resp = spi_txrx(0xff);
    if((resp&0x80)==0) break; }                 // when bit 7 is low, we got the response
  return resp; }

static uint16_t sd_get_r2(void)                // wait for R2: 2-byte response
{ uint16_t resp = sd_get_r1(); resp<<=8;       // first wait for R1
  if(resp&0x8000) return resp;
  return resp | spi_txrx(0xff); }              // then read one more byte

static uint8_t sd_get_r7(uint32_t &resp)       // wait for R7: 1+4 byte response
{ uint8_t ret=sd_get_r1();                     // first wait for R1
  if(ret&0x80) return ret;
  resp  = spi_txrx(0xff) << 24;                // then read the 32-bit word (big endian)
  resp |= spi_txrx(0xff) << 16;
  resp |= spi_txrx(0xff) << 8;
  resp |= spi_txrx(0xff);
  return ret; }

static uint8_t sd_get_data(uint8_t *data, int len)          // read block of data that comes after R1 response
{ uint8_t ret;
  for(uint16_t wait=sd_data_wait; wait; wait--)             // wait for block
  { ret=spi_txrx(0xff); if(ret==0xfe) break; }              // which starts with 0xFE
  if(ret!=0xfe) return 0xff;
  for(int i=0; i<len; i++)                                  // read block data
    data[i]=spi_txrx(0xff);
  uint16_t crc=spi_txrx(0xff);                              // read two bytes of CRC
  crc = (crc<<8) | spi_txrx(0xff);
  if(crc!=crc16(data, len)) return 0x40;                    // check if CRC correct
  return 0; }

static uint8_t sd_put_data(const uint8_t *data, int len) // write block of data after R1 response
{ uint8_t ret1, ret2;
  spi_txrx(0xfe);                           // data block starts with 0xFE
  for(int i=0; i<len; i++)
    spi_txrx(data[i]);                      // send block bytes
  uint16_t crc=crc16(data, len);            // calc. CRC
  spi_txrx(crc>>8);                         // send CRC
  spi_txrx(crc);
  for(uint8_t wait=sd_resp_wait ; wait; wait--)  // wait for no-FF
  { ret1=spi_txrx(0xff);                    // this is the EFSL ?
    if(ret1!=0xff) break; }
  if(ret1==0xff) return 0xff;
  for(uint16_t wait=sd_data_wait; wait; wait--) // wait for FF again
  { ret2=spi_txrx(0xff); if(ret2==0xff) break; }
  if(ret2!=0xff) return 0xff;
  return ret1; }

static void sd_idle_bytes(uint8_t count)
{ for (uint8_t count=sd_head_bytes; count; count--)
    spi_txrx(0xff);
}

static uint8_t sd_wait_ready(void)
{ spi_cs_high();
  sd_idle_bytes(sd_head_bytes);
  spi_cs_low();
  uint8_t ret;
  for(uint16_t wait=sd_data_wait; wait; wait--) // wait for FF
  { ret=spi_txrx(0xff); if(ret==0xff) break; }
  spi_cs_high();
  if(ret!=0xff) return 0xff;
  return 0; }

static uint8_t sd_cmd_r1(uint8_t cmd=0x00, uint32_t arg=0)
{ spi_cs_high();
  sd_idle_bytes(sd_head_bytes);
  spi_cs_low();
  sd_cmd(cmd, arg);                // send command
  uint8_t resp = sd_get_r1();      // wait for the R1 response
  sd_idle_bytes(sd_tail_bytes);
  spi_cs_high();
  return resp; }                   // response should be 0x01 = "In Idle State" and no errors

static uint16_t sd_cmd_r2(uint8_t cmd=0x00, uint32_t arg=0)
{ spi_cs_high();
  sd_idle_bytes(sd_head_bytes);
  spi_cs_low();
  sd_cmd(cmd, arg);                // send command
  uint16_t resp = sd_get_r2();     // wait for the R2 response
  sd_idle_bytes(sd_tail_bytes);
  spi_cs_high();
  return resp; }                   // response should be 0x01 = "In Idle State" and no errors

static inline uint8_t sd_reset(void) // send GO_IDLE_STATE command
{ return sd_cmd_r1(0, 0); } // response should be 0x01 = "In Idle State" and no errors

static uint8_t sd_cmd_r7(uint32_t &resp, uint8_t cmd, uint32_t arg=0, uint8_t dummy=10, int wait=16)
{ spi_cs_high();
  sd_idle_bytes(sd_head_bytes);
  spi_cs_low();
  sd_cmd(cmd, arg);                 // send command
  uint8_t ret = sd_get_r7(resp);    // wait for the R7 response
  sd_idle_bytes(sd_tail_bytes);
  spi_cs_high();
  return ret; }

static inline uint8_t sd_cmd8(uint32_t &resp, uint32_t arg=0x000001AA) { return sd_cmd_r7(resp, 0x08, arg); }

static uint8_t sd_acmd(uint8_t cmd=41, uint32_t arg=0x40000000)
{ uint8_t ret1, ret2;
  for(uint16_t wait=sd_data_wait; wait; wait--)
  { ret1 = sd_cmd_r1(55, 0);
    if(ret1&0x80) break;
    ret2 = sd_cmd_r1(41, 0x40000000);
    if(ret2!=0x01) break; }
  if(ret1&0x80) return ret1;
  return ret2; }

static inline uint8_t  sd_set_sector_size(uint32_t len=512) { return sd_cmd_r1(16, len); }

static inline uint8_t  sd_enable_crc(void)            { return sd_cmd_r1(59,   0); }
static inline uint16_t sd_read_status(void)           { return sd_cmd_r2(13,   0); }

static uint8_t sd_cmd_r1b(uint8_t cmd, uint32_t arg, uint8_t *data, int len)
{ spi_cs_high();
  sd_idle_bytes(sd_head_bytes);
  spi_cs_low();
  sd_cmd(cmd, arg);                // send command
  uint8_t resp = sd_get_r1();      // wait for the R1 response
  if( (resp&80) || (resp&0x01) )
  { sd_idle_bytes(sd_tail_bytes); spi_cs_high(); return resp; }
  resp = sd_get_data(data, len);
  sd_idle_bytes(sd_tail_bytes);
  spi_cs_high();
  return resp; }

static inline uint8_t sd_read_csd(uint8_t *data) { return sd_cmd_r1b( 9, 0, data, 16); }
static inline uint8_t sd_read_cid(uint8_t *data) { return sd_cmd_r1b(10, 0, data, 16); }

class sd_csd
{ public:
   uint8_t data[16];
   uint8_t version(void) const { return data[0]>>6; } // CSD format version: 0 = SD, 1 = SDHC
   uint32_t capacity(void) const
   { uint32_t capacity;
     if(version()==1)
     { capacity = data[9] + ((uint16_t)data[8] << 8) + ((uint32_t)(data[7] & 63) << 16) + 1;
       capacity <<= 10; }
     else
     { capacity = (data[8] >> 6) + ((uint16_t)data[7] << 2) + ((uint16_t)(data[6] & 3) << 10) + 1;
       capacity <<= (data[5] & 15) + ((data[10] & 128) >> 7) + ((data[9] & 3) << 1) + 2 -9; }
       // capacity = ( (data[6]&0x03)<<10 | data[7]<<2 | data[8]>>6) +1)  <<  (2+(((data[9]&0x03) << 1) | data[10]>>7);
       // capacity <<= (data[5] & 0x0f) - 9; }
     return capacity; }
} ;

static inline uint8_t sd_read_csd(sd_csd &csd)   { return sd_read_csd(csd.data); }

class sd_cid
{ public:
   uint8_t data[16];
   uint8_t manuf_id(void) const { return data[0]; }
   // uint16_t appl_id(void) const { return data[]; }
} ;

static inline uint8_t sd_read_cid(sd_cid &cid)   { return sd_read_cid(cid.data); }

static inline uint8_t sd_read_sector(uint32_t addr, uint8_t *data, int len) { return sd_cmd_r1b( 17, addr, data, len); }

static uint8_t sd_write_sector(uint32_t addr, const uint8_t *data, int len)
{ const uint8_t cmd=24;
  spi_cs_high();
  sd_idle_bytes(sd_head_bytes);
  spi_cs_low();
  sd_cmd(cmd, addr);               // send command
  uint8_t resp = sd_get_r1();      // wait for the R1 response
  if( (resp&80) || (resp&0x01) ) { sd_idle_bytes(sd_tail_bytes); spi_cs_high(); return resp; }
  resp = sd_put_data(data, len);
  sd_idle_bytes(sd_tail_bytes);
  spi_cs_high(); // return resp; }
  return (resp&0x1f)==0x05 ? 0x00 : 0xff; }

// ======================================================================================================

static const uint32_t sector_size = 512;
static sd_csd   csd;
static DSTATUS  card_stat = STA_NOINIT;
static uint8_t  card_type=0;            // 0 = SD, 1 = SDHC
static uint32_t capacity=0;             // [blocks]

DSTATUS disk_initialize (BYTE drv)
{ card_stat = STA_NOINIT;
  if (drv) return STA_NOINIT;
  spi_init();
  spi_slow();
  uint8_t ret, stat; uint32_t resp;
  ret = sd_reset();                if(ret!=0x01) return STA_NOINIT;
  ret = sd_cmd8(resp, 0x000001AA);
  // Response 0x05 means "illigal command error". The card is SDC version 1 or
  // MMC version 3. In this case we can continue with initialization
  if (ret != 0x05) {
    if( (ret!=0x01) || (resp!=0x000001AA) ) return STA_NOINIT;
  }
  ret = sd_acmd(0x41, 0x40000000); if(ret)  return STA_NOINIT;
  ret = sd_set_sector_size(sector_size);  if(ret)  return STA_NOINIT;
  ret = sd_enable_crc();           if(ret)  return STA_NOINIT;
  spi_fast();
  stat = sd_read_status();         if(stat) return STA_NOINIT;
  ret = sd_read_csd(csd);          if(ret)  return STA_NOINIT;
  card_type = csd.version();       // 0 = SD, 1 = SDHC
  capacity  = csd.capacity();
  card_stat = 0; return 0; }

DSTATUS disk_status (BYTE drv)
{ if (drv) return STA_NOINIT;
  return card_stat; }

DRESULT disk_read (
        BYTE drv,               /* Physical drive number (0) */
        BYTE *buff,             /* Pointer to the data buffer to store read data */
        DWORD sector,           /* Start sector number (LBA) */
        UINT count )            /* Number of sectors to read (1..128) */
{ if (drv || !count ) return RES_PARERR;           /* Check parameter */
  if (card_stat & STA_NOINIT) return RES_NOTRDY;       /* Check if drive is ready */

  if(card_type==0) sector*=sector_size;
  for(UINT i=0; i<count; i++)
  { if(sd_read_sector(sector, buff, sector_size) ) return RES_ERROR;
    if(card_type==0) sector += sector_size;
                     sector += 1;
    buff+=sector_size; }

  return RES_OK; }

#if _USE_WRITE
DRESULT disk_write (
        BYTE drv,               /* Physical drive number (0) */
        const BYTE *buff,       /* Ponter to the data to write */
        DWORD sector,           /* Start sector number (LBA) */
        UINT count )            /* Number of sectors to write (1..128) */
{ if (drv || !count) return RES_PARERR;           /* Check parameter */
  if (card_stat & STA_NOINIT) return RES_NOTRDY;       /* Check if drive is ready */
  if (card_stat & STA_PROTECT) return RES_WRPRT;       /* Check write protect */

  if(card_type==0) sector*=sector_size;
  for(UINT i=0; i<count; i++)
  { if(sd_write_sector(sector, buff, sector_size)) return RES_ERROR;
    if(card_type==0) sector += sector_size;
                     sector += 1;
    buff+=sector_size; }

  return RES_OK; }
#endif

#if _USE_IOCTL
DRESULT disk_ioctl (
        BYTE drv,               /* Physical drive number (0) */
        BYTE cmd,               /* Control command code */
        void *buff )            /* Pointer to the conrtol data */
{ if (drv) return RES_PARERR;                     /* Check parameter */
  if (card_stat & STA_NOINIT) return RES_NOTRDY;       /* Check if drive is ready */

  DRESULT res = RES_ERROR;
  
  switch(cmd)
  { case CTRL_SYNC:  /* Wait for end of internal write process of the drive */
      sd_wait_ready();
      break;
    case GET_SECTOR_COUNT: /* Get drive capacity in unit of sector (DWORD) */
      *(DWORD *)buff = 0;
      if(sd_read_csd(csd)) break;
      card_type = csd.version();
      capacity = csd.capacity();
      *(DWORD *)buff = capacity;
      res=RES_OK; break;
  }
  return res; }
#endif
