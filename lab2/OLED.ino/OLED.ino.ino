# include <Arduino.h>
# include <U8x8lib.h> //Display library
# include <Wire.h> //I2C protocol library (the display uses I2C to interact with MCU)
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ PIN_WIRE_SCL, /* data=*/
PIN_WIRE_SDA, /* reset=*/ U8X8_PIN_NONE); // OLEDs without Reset of the Display
void setup( void) {
  u8x8.begin();
  u8x8.setFlipMode( 1 ); // set number from 1 to 3, the screen word will rotary 180
}
void loop( void) {
  u8x8.setFont(u8x8_font_chroma48medium8_r); //try u8x8_font_px437wyse700a_2x2_r
  u8x8.setCursor( 0 , 0 ); // It will start printing from (0,0) location
  u8x8.print("Welcome to");
  u8x8.setCursor( 0 , 1); // (columns, row)
  u8x8.print("my World!! ");
  u8x8.setCursor( 0 , 2 );
  u8x8.print("Won't you");
  u8x8.setCursor( 0 , 3 );
  u8x8.print("come on in?");
}