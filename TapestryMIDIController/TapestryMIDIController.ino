/*
 * Compile as:
 * Board: Teensy 3.2 / 3.1 
 * USB Type: Serial
 * 
 * References:
 * Implementing simple high and low pass filters in C
 * http://stackoverflow.com/questions/13882038/implementing-simple-high-and-low-pass-filters-in-c
 */

/*
 *                      Teensy 3.2
 *                        _______
 *                    ___|       |___
 *               Gnd | o |_______| o | Vcc
 * (interior) SS PB0 | o         o o | AGND
 *  PE6     SCLK PB1 | o           o | 3.3V (100 mA max)
 *  AIN0    MOSI PB2 | o         o o | 23
 *  INT6    MISO PB3 | o         o o | 22
 * RTS OC1C OC0A PB7 | o         o o | 21
 * OC0B SCL INT0 PD0 | o           o | 20
 *      SDA INT1 PD1 | o           o | 19
 *     RXD1 INT2 PD2 | o           o | 18
 *     TXD1 INT3 PD3 | o           o | PB4 ADC11
 *    !OC4A OC3A PC6 | o           o | PD7 ADC10 T0 OC4D
 *                   | o           o |
 *                   | o           o |
 *     OC4A ICP3 PC7 | o o o o o o o | PD6 ADC9  T1 !OC4D (LED on PD6)
 *                   |_______________|
 *                       | | | | |
 *            CTS XCK1 PD5 | | | PD4 ADC8 ICP1
 *                       Vcc | RST
 *                          GND
 * 
 *                        _______
 *                    ___|       |___
 *               Gnd | o |_______| o | Vcc
 *    Touch RX1    0 | o         o o | AGND
 *    Touch TX1    1 | o           o | 3.3V (100 mA max)
 *                 2 | o         o o | 23 A9 Touch  PWM
 *                 3 | o         o o | 22 A8 Touch  PWM
 *                 4 | o         o o | 21 A7        PWM
 *                 5 | o           o | 20 A6        PWM
 *                 6 | o           o | 19 A5        Touch
 *                 7 | o           o | 18 A4        Touch
 *                 8 | o           o | 17 A3        Touch
 *                 9 | o           o | 16 A2        Touch
 *                10 | o           o | 15 A1        Touch
 *                11 | o           o | 14 A0
 *                12 | o o o o o o o | 13 (LED)
 *                   |_______________|
 *                       | | | | |
 *            CTS XCK1 PD5 | | | PD4 ADC8 ICP1
 *                       Vcc | RST
 *                          GND
 */




#define SMOOTHINGLENGTH 16  // Number of samples over which to smooth the raw reading from touchRaad()
#define PINS 8              // Number of touch pins used on the Teensy 3.2

#define CUTOFF 10           // Cutoff frequency of highpass filter in Hz
#define SAMPLE_RATE 100     // (Approximate) sampling rate of the program in Hz

#define NP 0                // No print
#define HPP 1               // Highpass print
#define TP 2                // Touch print

int ledstate = 0;

// Array of the enabled state of each touch pin/solenoid.
// The order of the array follows the definition/pairing of the touchpins[] and solenoidpins[] arrays.
// If a pin is enabled, the solenoid will respond to a touch input.
// If a pin is disabled, the solenoid will not respond to a touch input, however, it will still be read by the program.
boolean enabled[] = { true, true, true, true, true, true, true, true };

// Print the output  { 23, 22, 19, 18, 17, 16, 15,  1,  0 }
int printenabled[] = { HPP, HPP, HPP, HPP, HPP, HPP, HPP, HPP, HPP };

// Highpass threshold
float hp_threshold[] = { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 };

// Array of the "fired" state of each solenoid.
// When a solenoid is fired the corresponding value in the array will change to TRUE, indicating that a note has been played.
// The value will only reset when the program detects a negative dip in the high pass filter output.
// The negative dip corresponds to the key being released by the user, which creates a negative change in the raw capacitance reading.
// Checking this value prevents the solenoid from firing continuously when only one touch has occured.
boolean fired[PINS] = { false, };

// Filter gains
float dt = 1.0/SAMPLE_RATE;     // Time change
float RC = 1.0/(2.0*PI*CUTOFF); // "RC" constant of system
float alpha = RC/(RC + dt);     // High pass filter gain

int frame = 50;                 // The number of the "frame" (i.e. loop iteration) on which to send serial data.
int framecount = 0;             // The frame counter that increments every loop.

// When used directly, the data from the touch pins is polluted by electrical noise.
// To smooth the input from the touch pins, we use a windowed moving average that compares the data over time and eliminates spikes in the readings.
// The mathematical operation is the sum of all data points within the "window" divided by the number of data points within the "window".
// To keep track of the samples over time, we use a buffer to store the desired number of raw data values.
// The smoothing window sum keeps a tally of the overall sum of the buffer. This value divided by the number of samples is the current average.
// The index variable keeps track of the raw value to subtract and overwrite when the data increments. This allows us to cycle through the buffer without needing to move any elements.
int smoothingwindow[PINS][SMOOTHINGLENGTH] = { { 0, }, { 0, } };        // The buffer to keep track of the smoothing 
long int smoothingwindowsum[PINS] = { 0, };                             // The summation 
int idx = 0;                                                            // The moving average index.

//23,22,21,20,10,9,6,5,4,3,2 PWM
//23,22,19,18,17,16,15,1,0 Touch
int touchpins[] = { 0, 1, 23, 19, 18, 17, 16, 15 };
int solenoidpins[] = { 5, 6, 9, 10, 22, 21, 20, 25 };

// Delay time in milliseconds of the solenoid between firing and releasing.
int delayinterval[] = { 10, 10, 10, 10, 10, 10, 10, 10, 10 };

int touchreadings[PINS] = { 0, };                                               // A buffer to store the raw readings from the touch pins.

// The values set in the threshold buffers are observed from test data but are otherwise arbitrary.
// The values should be hand-tuned from observation to be high enough such that the pins are not triggered from data noise fluctuations but low enough such that they will trigger from intended input.
// When the high pass filter encounters a positive shift in readings (from a touch), the filter data will show a positive spike and settle to zero if the touch persists.
// When the touch is released, the high pass filter data will show a negative spike due to the raw readings dropping lower.
// Both events are used to control the behavior of the solenoid pins as to fire only when an initial touch has occurred and not after any residual contact (e.g. holding down the key).
float highpassreadingsthreshold[] = { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 };       // A buffer of the thresholds of the high pass reading corresponding to each touch pin.
float highpassresetthreshold[] = { -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0 }; // The high pass threshold that resets the pin to the firing state.

float smoothreadings[PINS][2] = { { 0, }, { 0, } };                     // A buffer to store the current and previous smoothed readings.
float highpass[PINS][2] = { { 0, }, { 0, } };                           // A buffer to store the current and previous high pass filter data.

int ledpin = LED_BUILTIN;    // Define the LED pin

// message data union
union {
  uint32_t u32;
  float f32;
} data;

// Teensy 1
//uint8_t channel[] = { 1, 1, 2, 2, 2, 2, 2, 2 };
//uint8_t note[] = { 60, 61, 60, 61, 62, 63, 64, 65 };

// Teensy 2
uint8_t channel[] = { 3, 4, 5, 6, 7, 8, 9, 10 };
uint8_t note[] = { 60, 60, 60, 60, 60, 60, 60, 60 };


void setup() {
  // put your setup code here, to run once:
  // Start the serial port at 115200 baud.
  Serial.begin(115200);
  // Wait for the serial port to connect.
  //while(!Serial);
  // Set the LED pin as a digital output.
  pinMode(ledpin, OUTPUT);

  // Define each solenoid pin in the buffer as an output.
  for (int i = 0; i < PINS; i++) {
    pinMode(solenoidpins[i], OUTPUT);
  }

  // This code segment flashes the LED for one second and turns it off.
  // The Teensy 3.2 has no other LED indicators for power or data, so this routine indicates that the program is starting.
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  // For each of the touch pins:
  // 1. Read the data from the pin using touchRead() into the touchreadings buffer.
  // 2. Add the reading to the smoothingwindowsum buffer that keeps track of the entire sum of the smoothing window and subtract the previous reading from the smoothingwindow buffer.
  // 3. Overwrite the previous value in the smoothingwindow buffer with the current touchreadings value.
  // 4. Get the smoothed (low pass filtered) readings by dividing the smoothingwindowsum by the number of data points in the buffer.
  // 5. Calculate the high pass filter reading from the current and previous smoothreadings values and the previous highpass value. 0 indicates the current reading and 1 indicates the previous reading.
  for (int i = 0; i < PINS; i++) {
    touchreadings[i] = touchRead(touchpins[i]);                                             // 1.
    smoothingwindowsum[i] += touchreadings[i] - smoothingwindow[i][idx];                    // 2.
    smoothingwindow[i][idx] = touchreadings[i];                                             // 3.
    smoothreadings[i][0] = smoothingwindowsum[i]/SMOOTHINGLENGTH;                           // 4.
    highpass[i][0] = alpha*(highpass[i][1] + smoothreadings[i][0] - smoothreadings[i][1]);  // 5.
    //Serial.print(smoothreadings[i][0]);
    //Serial.print(" ");
    Serial.print(highpass[i][0]);
    Serial.print(" ");
    analogWrite(solenoidpins[i], map(smoothreadings[i][0], 800, 5000, 0, 255));
  }
  Serial.println();
  

  // For each element in the highpass and smoothreadings buffers:
  // Shift the data such that the current reading (zeroth element) is now the previous reading (first element) in each respective buffer.
  for (int i = 0; i < PINS; i++) {
    highpass[i][1] = highpass[i][0];
    smoothreadings[i][1] = smoothreadings[i][0];
  }

  // For each highpass buffer element:
  // Check whether the value in the buffer has met or exceeded the threshold defined by the user AND is enabled by the user AND has not already fired.
  // If the above conditions are met, turn on the solenoid pin for the delay interval duration and release the pin.
  // After the solenoid has fired, set the fired state of the pin to TRUE.
  for (int i = 0; i < PINS; i++) {
    if ((highpass[i][0] >= highpassreadingsthreshold[i]) && enabled[i] && !fired[i]) {
      //analogWrite(solenoidpins[i], 255);
      //delay(delayinterval[i]);
      //analogWrite(solenoidpins[i], 0);
      digitalWrite(LED_BUILTIN, HIGH);
      fired[i] = true;
      //#ifdef USB_MIDI
        usbMIDI.sendNoteOn(note[i], 100, channel[i]);
      //#endif
    }
  }

  // For each of the highpass buffer elements:
  // Check whether the value in the buffer has dropped below the reset threshold defined by the user.
  // If the above condition is met, reset the fired state of the pin to FALSE.
  for (int i = 0; i < PINS; i++) {
    if (highpass[i][0] <= highpassresetthreshold[i]) {
        fired[i] = false;
        digitalWrite(LED_BUILTIN, LOW);
        //#ifdef USB_MIDI
          usbMIDI.sendNoteOff(note[i], 100, channel[i]);
        //#endif
    }
  }

  // Increment the index of the smoothed readings. If the index exceeds the length of the buffer, reset the vaue of the index to zero.
  idx++;
  if (idx >= SMOOTHINGLENGTH) {
    idx = 0;
  }
  
  // Increment the frame counter and reset to zero if it has overflowed.
  framecount ++;
  if (framecount >= frame) {
    framecount = 0;
  }

  // On the zeroth frame, write the data to the serial port for debugging.
  // The data can be viewed using the Serial Monitor or Serial Plotter under Tools in the Arduino IDE menu.
  if (framecount == 0) {
    ledstate = 0;
    for (int i = 0; i < PINS - 1; i++) {
      // Send data if the pin is enabled.
      if (printenabled[i] == HPP) {
//        Serial.print(highpass[i][0]);
//        Serial.print(" ");
      } else if (printenabled[i] == TP) {
        ledstate |= (highpass[i][0] > hp_threshold[i])? 1 : 0;
//        Serial.print((highpass[i][0] > hp_threshold[i])? 1 : 0);
//        Serial.print("\t");
      }
    }
    if (printenabled[PINS] == HPP) {
//      Serial.print(highpass[PINS][0]);
    } else if (printenabled[PINS] == TP) {
      ledstate |= (highpass[PINS][0] > hp_threshold[PINS])? 1 : 0;
//      Serial.print((highpass[PINS][0] > hp_threshold[PINS])? 1 : 0);
    }
    //Serial.println();
    
    digitalWrite(LED_BUILTIN, ledstate);
  }
  delay(10);
}






// Format the pin address, event type, and data into a 6-Byte buffer
void format_message(uint8_t message[], uint8_t button_idx, uint8_t event, uint32_t data) {
  message[0] = 0x80 | (button_idx & 0x7F); // Header byte
  message[1] = 0x3F & (((event & 0x03) << 4) | ((data & 0xF0000000) >> 28));
  message[2] = 0x7F & ((data & 0xFE00000) >> 21);
  message[3] = 0x7F & ((data & 0x1FC000) >> 14);
  message[4] = 0x7F & ((data & 0x3F80) >> 7);
  message[5] = 0x7F & data;
}









