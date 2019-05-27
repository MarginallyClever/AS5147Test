// AS5147 test
// dan@marginallyclever.com
// 2019-05-16
// @see https://ams.com/documents/20143/36005/AS5147_DS000307_2-00.pdf

#define PIN_SENSOR_CSEL_0   11
#define  PIN_SENSOR_CLK_0   10
#define PIN_SENSOR_MOSI_0   9
#define PIN_SENSOR_MISO_0   8

#define PIN_SENSOR_CSEL_1   5
#define  PIN_SENSOR_CLK_1   4
#define PIN_SENSOR_MOSI_1   3
#define PIN_SENSOR_MISO_1   2

#define PIN_SENSOR_CSEL_2   14
#define  PIN_SENSOR_CLK_2   15
#define PIN_SENSOR_MOSI_2   16
#define PIN_SENSOR_MISO_2   17

#define PIN_SENSOR_CSEL_3   18
#define  PIN_SENSOR_CLK_3   19
#define PIN_SENSOR_MOSI_3   20
#define PIN_SENSOR_MISO_3   21

#define PIN_SENSOR_CSEL_4   45
#define  PIN_SENSOR_CLK_4   43
#define PIN_SENSOR_MOSI_4   41
#define PIN_SENSOR_MISO_4   39

#define PIN_SENSOR_CSEL_5   29
#define  PIN_SENSOR_CLK_5   27
#define PIN_SENSOR_MOSI_5   25
#define PIN_SENSOR_MISO_5   23


#define PIN_SELECT  2
#define PIN_CLOCK   3
#define PIN_MOSI    4
#define PIN_MISO    5
/*
#define PIN_SELECT  PIN_SENSOR_CSEL_4
#define PIN_CLOCK    PIN_SENSOR_CLK_4
#define PIN_MOSI    PIN_SENSOR_MOSI_4
#define PIN_MISO    PIN_SENSOR_MISO_4
*/

#define BOTTOM_14_MASK       (0x3FFF)
#define SENSOR_TOTAL_BITS    (16)
#define SENSOR_DATA_BITS     (15)
#define SENSOR_ANGLE_BITS    (14)

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
  digitalWrite(PIN_MOSI,HIGH);
#ifdef VERBOSE
  Serial.println("\n\n** START **\n");
#endif
}


void loop() {
#ifdef VERBOSE
  Serial.print(millis());
  Serial.print('\t');
#endif
  
  uint16_t rawValue;
  
  if(getSensorRawValue(rawValue)==0) {
    // raw value received ok
    float angle = extractAngleFromRawValue(rawValue);
#ifdef GRAPHABLE
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
  return (float)(rawValue & BOTTOM_14_MASK) * 360.0 / (float)(1<<SENSOR_ANGLE_BITS);
}

/**
 * @param result where to store the returned value.  may be changed even if method fails.
 * @return 0 on fail, 1 on success.
 * @see https://ams.com/documents/20143/36005/AS5147_DS000307_2-00.pdf/ figure 12
 */
boolean getSensorRawValue(uint16_t &result) {
  result=0;
  uint8_t input,parity=0;

  // Send the request for the angle value (command 0xFFFF)
  // at the same time as receiving an angle.

  // Collect the 16 bits of data from the sensor
  digitalWrite(PIN_SELECT,LOW);
  
  for(int i=0;i<SENSOR_TOTAL_BITS;++i) {
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
    parity ^= (i>0) & input;
  }
#ifdef VERBOSE
  Serial.print('\t');
#endif

  digitalWrite(PIN_SELECT,HIGH);
  
  // check the parity bit
  return ( parity != (result>>SENSOR_DATA_BITS) );
}
