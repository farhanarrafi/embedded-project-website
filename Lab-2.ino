# include <Arduino.h>
# include <U8x8lib.h> //Display library
# include <Wire.h> //I2C protocol library (the display uses I2C to interact with MCU)
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ PIN_WIRE_SCL, /* data=*/ PIN_WIRE_SDA, /* reset=*/ U8X8_PIN_NONE); // OLEDs without Reset of the Display

const int buttonPin = 1;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status

void setup( void) {
  u8x8.begin();
  u8x8.setFlipMode( 1 ); // set number from 1 to 3, the screen word will rotary 180

  // initialize the LED pin as an output:
  pinMode(LED_BUILTIN, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP);
}
void loop( void) {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // remove text
    u8x8.setFont(u8x8_font_chroma48medium8_r); //try u8x8_font_px437wyse700a_2x2_r
    u8x8.setCursor( 0 , 0 ); // It will start printing from (0,0) location
    u8x8.print("                    ");
    u8x8.setCursor( 0 , 1); // (columns, row)
    u8x8.print("                    ");
    u8x8.setCursor( 0 , 2 );
    u8x8.print("                    ");
    u8x8.setCursor( 0 , 3 );
    u8x8.print("                    ");
  } else {
    // show text
    u8x8.setFont(u8x8_font_chroma48medium8_r); //try u8x8_font_px437wyse700a_2x2_r
    u8x8.setCursor( 0 , 0 ); // It will start printing from (0,0) location
    u8x8.print("Welcome to");
    u8x8.setCursor( 0 , 1); // (columns, row)
    u8x8.print("my World!");
    u8x8.setCursor( 0 , 2 );
    u8x8.print("Won't you ");
    u8x8.setCursor( 0 , 3 );
    u8x8.print("come on in?");
  }

  
}