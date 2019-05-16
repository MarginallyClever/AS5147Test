// AS5147 test
// dan@marginallyclever.com
// 2019-05-16
// @see https://ams.com/documents/20143/36005/AS5147_DS000307_2-00.pdf

#define PIN_SELECT  2
#define PIN_CLOCK   3
#define PIN_MISO    4
#define PIN_MOSI    5

#define BOTTOM_14_MASK 0x3FFF

//#define VERBOSE
#define GRAPHABLE

void setup() {
  Serial.begin(57600);
  pinMode(PIN_SELECT,OUTPUT);
  pinMode(PIN_CLOCK,OUTPUT);
  pinMode(PIN_MISO,INPUT);
  pinMode(PIN_MOSI,OUTPUT);

  digitalWrite(PIN_SELECT,HIGH);
  digitalWrite(PIN_CLOCK,LOW);
#ifdef VERBOSE
  Serial.println("\n\n** START **\n");
#endif
}


void loop() {
#ifdef VERBOSE
  Serial.print(millis());
  Serial.print(' ');
#endif
  
  uint16_t rawValue;
  
  if(getSensorRawValue(rawValue)==0) {
    // raw value received ok
    float angle = extractAngleFromRawValue(rawValue);
#ifdef GRAPHABLE
    Serial.print('\t');
    Serial.print(angle,3);
#endif
  }
  
  Serial.println();
}

/**
 * @param rawValue 16 bit value from as4157 sensor, including parity and EF bit
 * @return degrees calculated from bottom 14 bits.
 */
float extractAngleFromRawValue(uint16_t rawValue) {
  return (float)(rawValue & BOTTOM_14_MASK) * 360.0 / (float)(1<<14);
}

/**
 * @param result where to store the returned value.  may be changed even if method fails.
 * @return 0 on fail, 1 on success.
 * @see https://ams.com/documents/20143/36005/AS5147_DS000307_2-00.pdf/ figure 12
 */
boolean getSensorRawValue(uint16_t &result) {
  result=0;
  uint8_t input,parity=0;

  // send the request for the angle value (command 0xFFFF)
  digitalWrite(PIN_MOSI,HIGH);

  // Collect the 16 bits of data from the sensor
  digitalWrite(PIN_SELECT,LOW);
  
  for(int i=0;i<16;++i) {
    digitalWrite(PIN_CLOCK,HIGH);
    // this is here to give a little more time to the clock going high.
    // only needed if the arduino is *very* fast.  I'm feeling generous.
    result <<= 1;
    digitalWrite(PIN_CLOCK,LOW);
    
    input = digitalRead(PIN_MISO);
#ifdef VERBOSE
    Serial.print(input,DEC);
#endif
    result |= input;
    if(i>0) parity += input;  // XOR
  }

  digitalWrite(PIN_SELECT,HIGH);
  
  // check the parity bit
  return ( (parity&1) != (result>>15) );
}
