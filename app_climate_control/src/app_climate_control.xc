#include <xs1.h>
#include <debug_print.h>
#include <xclib.h>
#include <string.h>
#include <timer.h>

#include "i2c.h"
#include "i2c_wrapper.h"

#include "lcd_data.h"

//Must all be powers of 2
#define SCREEN_BUFFER_X_PIX     128
#define SCREEN_BUFFER_Y_PIX     64
#define SCREEN_BUFFER_BITS_PP   1   //Maximum 8bpp
#define SCREEN_BUFFER_BYTES     ((SCREEN_BUFFER_X_PIX * SCREEN_BUFFER_Y_PIX * SCREEN_BUFFER_BITS_PP) / 8)

port p_scl = XS1_PORT_1A;
port p_sda = XS1_PORT_1B;
port reset = XS1_PORT_32A;

unsigned char scrn_buff[SCREEN_BUFFER_BYTES] = {0};


//Looks up the value of a pixel at posn x,y in a screen buffer
static inline unsigned get_pixel_from_buffer(const unsigned char buff[SCREEN_BUFFER_BYTES], unsigned x, unsigned y){
  unsigned idx, read_word;
  if (x < SCREEN_BUFFER_X_PIX && y < SCREEN_BUFFER_Y_PIX){
      idx = (x % SCREEN_BUFFER_X_PIX) + (y/8) * SCREEN_BUFFER_X_PIX;
      read_word = (0x01 << (y % 8));
    if (read_word & buff[idx]) return 1;
    else return 0;
  }
  return 0;
}

//Sets a pixel at posn x,y in a screen buffer
static inline void set_pixel_in_buffer(unsigned char buff[SCREEN_BUFFER_BYTES], unsigned x, unsigned y, unsigned val){
  unsigned idx, write_word;
    if (x < SCREEN_BUFFER_X_PIX && y < SCREEN_BUFFER_Y_PIX){
      idx = (x % SCREEN_BUFFER_X_PIX) + (y/8) * SCREEN_BUFFER_X_PIX;
      write_word = (0x01 << (y % 8));
      if (val) buff[idx] |= write_word;
      else buff[idx] &= ~write_word;
    }
}

//Function for taking the stripped 1b DIB output from paint and converting to top-left origin bitmap
void convert_to_useful(const unsigned char buff[SCREEN_BUFFER_BYTES]){
  for (int row_ctr=SCREEN_BUFFER_Y_PIX - 1; row_ctr>-1; row_ctr--){
    for (int i=0; i<(((SCREEN_BUFFER_X_PIX /8)  * SCREEN_BUFFER_BITS_PP)) ; i++){
      unsigned tmp = buff[(row_ctr * (SCREEN_BUFFER_X_PIX / 8) * SCREEN_BUFFER_BITS_PP) + i];
      tmp = (bitrev(tmp) >> 24);
      tmp = ~tmp;
      tmp &= 0xff;
      debug_printf("0x");
      if (tmp < 16) debug_printf("0");
      debug_printf("%d,",tmp);
    }
    debug_printf("\n");
  }
}

//Dumps whole screen buffer to console to capture in c array suitable for mem_cpy'ing
void dump_buffer_to_console(const unsigned char buff[SCREEN_BUFFER_BYTES]){
  unsigned char new[SCREEN_BUFFER_BYTES];
  for (int y=0; y<SCREEN_BUFFER_Y_PIX; y++){
    for (int x=0; x<SCREEN_BUFFER_X_PIX; x++){
      set_pixel_in_buffer(new, x, y, get_pixel_from_buffer(buff, x, y));
      }
    }
  for (unsigned idx=0; idx<SCREEN_BUFFER_BYTES; idx++){
    if ((idx & 0xf) == 0) debug_printf("\n", idx);
    debug_printf("0x%x,", new[idx]);
    delay_microseconds(20);
  }
}

typedef enum{
  OVEREWRITE_ALL,
  WRITE_OR,
  WRITE_AND,
  WRITE_EXOR
} print_mode_t;

#define WRAP_TEXT     0   //Wrap if x cursor goes off end off screen

void putchar(unsigned char buff[SCREEN_BUFFER_BYTES], char char_to_print, print_mode_t mode){
  static unsigned char_cursor_x = 0;
  static unsigned char_cursor_y = 0;

  //check for out of printable range character
  if(char_to_print < FONT_OFFSET || (char_to_print - FONT_OFFSET) > FONT_NUM_CHARS){
    switch (char_to_print){
    case 0x0a:  //Line Feed
      char_cursor_x = 0;
      char_cursor_y++;
      if (char_cursor_y >= SCREEN_BUFFER_Y_PIX / 8){
        char_cursor_y = (SCREEN_BUFFER_Y_PIX / 8) - 1;
        memcpy(buff, buff + SCREEN_BUFFER_X_PIX, SCREEN_BUFFER_BYTES - SCREEN_BUFFER_X_PIX); //Scroll up one line
        memset(buff + (SCREEN_BUFFER_BYTES - SCREEN_BUFFER_X_PIX), 0, SCREEN_BUFFER_X_PIX);  //Clear the bottom line
      }
      return;
    break;

    case 0x0d:  //Carriage return
      char_cursor_x = 0;
      return;
    break;

    default:    //Print control character
    char_to_print = '!';
    break;
    }
  }


  unsigned buffer_offset = (char_cursor_x * FONT_PITCH) + (SCREEN_BUFFER_X_PIX * char_cursor_y);
#if WRAP_TEXT
  memcpy(&buff[buffer_offset], &font[(char_to_print - FONT_OFFSET) * FONT_WIDTH], FONT_WIDTH);
#else
  if (char_cursor_x < SCREEN_BUFFER_X_PIX / FONT_PITCH){
    memcpy(&buff[buffer_offset], &font[(char_to_print - FONT_OFFSET) * FONT_WIDTH], FONT_WIDTH);
  }
#endif
  char_cursor_x++;
#if WRAP_TEXT
  if (char_cursor_x >= SCREEN_BUFFER_X_PIX / FONT_PITCH){
    char_cursor_x = 0;
    char_cursor_y++;
    if (char_cursor_y >= SCREEN_BUFFER_Y_PIX / 8){
      char_cursor_y = (SCREEN_BUFFER_Y_PIX / 8) - 1;
      memcpy(buff, buff + SCREEN_BUFFER_X_PIX, SCREEN_BUFFER_BYTES - SCREEN_BUFFER_X_PIX); //Scroll up one line
      memset(buff + (SCREEN_BUFFER_BYTES - SCREEN_BUFFER_X_PIX), 0, SCREEN_BUFFER_X_PIX);  //Clear the bottom line
    }
  }
#endif
}

// Override the weak symbol used for print messages, so general prints go to oled
int _write(int fd, const unsigned char *data, size_t len) {
  for(int i=0; i<len; i++){
    putchar(scrn_buff, *data++, OVEREWRITE_ALL);
  }
  return len;
}

//draw line from x,y to new x,y
void draw_line(unsigned char buff[SCREEN_BUFFER_BYTES], unsigned x1, unsigned y1, unsigned x2, unsigned y2, unsigned val)
{
  unsigned tmp;
  unsigned x,y;
  unsigned dx, dy;
  int err;
  int ystep;
  unsigned swapxy = 0;

  if (x1 > x2) dx = x1-x2; else dx = x2-x1;
  if (y1 > y2) dy = y1-y2; else dy = y2-y1;

  if (dy > dx)
  {
    swapxy = 1;
    tmp = dx; dx =dy; dy = tmp;
    tmp = x1; x1 =y1; y1 = tmp;
    tmp = x2; x2 =y2; y2 = tmp;
  }
  if (x1 > x2)
  {
    tmp = x1; x1 =x2; x2 = tmp;
    tmp = y1; y1 =y2; y2 = tmp;
  }
  err = dx >> 1;
  if (y2 > y1) ystep = 1; else ystep = -1;
  y = y1;

  if (x2 == 0xffff)
    x2--;

  for(x = x1; x <= x2; x++)
  {
    if (swapxy == 0)
      set_pixel_in_buffer(buff, x, y, val);
    else
      set_pixel_in_buffer(buff, y, x, val);
    err -= (uint8_t)dy;
    if (err < 0)
    {
      y += (unsigned)ystep;
      err += (unsigned)dx;
    }
  }
}


//Calculates a (very) approximate dbfs value from the PCM input
int lin_to_dbfs(unsigned pcm)
{
  int dbfs;
  dbfs = 6 + (4 * (1 - clz(pcm)));
  return dbfs;
}

//Calculates the y position of the meter arc based on x input
unsigned calc_arc(unsigned x)
{
  unsigned y;
  y = 6 + ((32-x)*(32-x))/95;
  return y;
}

//copyies a sprite to a location within the framebuffer pixel by pixel
void copy_sprite(unsigned char *buff, unsigned x_buff, unsigned y_buff,
                 const unsigned char *src_buff, unsigned x_src_size, unsigned y_src_size)
{
  for (int y=0; y<y_src_size; y++){
    for (int x=0; x<x_src_size; x++){
      if ((0x01 << (x % (8/SCREEN_BUFFER_BITS_PP))) & src_buff[(x + y * y_src_size) / (8/SCREEN_BUFFER_BITS_PP)]) {
        set_pixel_in_buffer(buff, x_buff + x, y_buff + y, 1);
      }
      else{
        set_pixel_in_buffer(buff, x_buff + x, y_buff + y, 0);
      }
    }
  }
}


static const uint8_t ssd1306_128x64_init_seq[] = {
  0xae,           /* display off, sleep mode */
  0xd5, 0x80,     /* clock divide ratio (0x0=1) and oscillator frequency (0x8) */
  0xa8, 0x3f,     /* Multiplex */

  0xd3, 0x00,     /* Display offset, none */

  0x40,           /* start line 0*/

  0x8d, 0x14,     /*charge pump setting: 0x14 enable, 0x10 disable */

  0x20, 0x00,     /* 0x02 page addressing mode, 0x00 horiz, 0x00, vert */
  0xa1,           /* segment remap a0/a1*/
  0xc8,           /* c0: scan dir normal, c8: reverse */
  0xda, 0x12,     /* com pin HW config, sequential com pin config (bit 4), disable left/right remap (bit 5) */
  0x81, 0xcf,     /* [2] set contrast control */
  0xd9, 0xf1,     /* [2] pre-charge period 0x22/f1*/
  0xdb, 0x40,     /* vcomh deselect level */

  0x2e,           /* Deactivate scroll */
  0xa4,           /* output ram to display */
  0xa6,           /* none inverted normal display mode */
  0xaf,           /* display on */
};

static const unsigned char ssd1306_128x64_reset_cursor[] = {
  0x21,    //Set column addr
  0x00,    //column start address (0 = reset)
  SCREEN_BUFFER_X_PIX - 1,    //display width
  0x22,    //Set page addr
  0x00,    //start address
  0x7      //page end address
};


void test(client i2c_master_if i_i2c){
  //convert_to_useful(vu_3_raw);
  //print_buffer_to_console(vu_3);

  //Runme before any I2C access, else expect exceptions!
  i2c_setup_glob_interface(i_i2c);


  i2c_write(I2C_CMD_MODE, sizeof(ssd1306_128x64_init_seq), ssd1306_128x64_init_seq);
  i2c_write(I2C_CMD_MODE, sizeof(ssd1306_128x64_reset_cursor), ssd1306_128x64_reset_cursor);
  memset(scrn_buff, 0x00, 1024);
  i2c_write(I2C_DATA_MODE, 1024, scrn_buff);




  while(1){
    for (int i=0; i<64; i++){
      draw_line(scrn_buff, 32 + 64, 42, i + 64, calc_arc(i), 1);
      draw_line(scrn_buff, 32, 42, (64-i), calc_arc(i), 1);
      i2c_write(I2C_CMD_MODE, sizeof(ssd1306_128x64_reset_cursor), ssd1306_128x64_reset_cursor);
      //putchar(scrn_buff, '!', OVEREWRITE_ALL);
      debug_printf("Sum calculator %d * %d = %d\r", i , i , i*i);
      i2c_write(I2C_DATA_MODE, 1024, scrn_buff);

    }
  }
}


int main(void){
  i2c_master_if i_i2c;

  par{
    i2c_master(&i_i2c, 1, p_scl, p_sda, 400);
    test(i_i2c);
  }
  return 0;
}

