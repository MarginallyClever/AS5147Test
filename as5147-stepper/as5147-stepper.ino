
// https://ams.com/documents/20143/36005/AS5147_DS000307_2-00.pdf/6921a55b-7cba-bf20-78c0-660d62bd0a5b

#define MOTOR_0_DIR           (16)
#define MOTOR_0_STEP          (17)
#define MOTOR_0_ENABLE        (48)
#define MOTOR_STEPS_PER_TURN  (200*16)

// SENSORS
#define RUMBA_X_POS   (37)
#define RUMBA_X_NEG   (36)
#define RUMBA_Y_POS   (35)
#define RUMBA_Y_NEG   (34)

#define AS5147_CSEL   (RUMBA_X_POS)
#define AS5147_CLK    (RUMBA_X_NEG)
#define AS5147_MOSI   (RUMBA_Y_POS)
#define AS5147_MISO   (RUMBA_Y_NEG)

// sensor bits, flags, and masks
#define BOTTOM_14_MASK       (0x3FFF)
#define SENSOR_TOTAL_BITS    (16)
#define SENSOR_DATA_BITS     (15)
#define SENSOR_ANGLE_BITS    (14)
#define SENSOR_ANGLE_PER_BIT (360.0/(float)((long)1<<SENSOR_ANGLE_BITS))  // 0.00549316406

// wrap all degrees to within -180...180.
#define WRAP_DEGREES(NN)     (fmod( (NN+360), 360 ))



void setup() {
  Serial.begin(57600);
  Serial.println("\n\n** START **\n\n");

  pinMode(AS5147_CSEL,OUTPUT);
  pinMode(AS5147_CLK ,OUTPUT);
  pinMode(AS5147_MOSI,OUTPUT);
  pinMode(AS5147_MISO,INPUT );

  digitalWrite(AS5147_CSEL,HIGH);  // csel
  digitalWrite(AS5147_MOSI,HIGH);  // mosi
    
  pinMode(MOTOR_0_DIR   ,OUTPUT);
  pinMode(MOTOR_0_STEP  ,OUTPUT);
  pinMode(MOTOR_0_ENABLE,OUTPUT);

  digitalWrite(MOTOR_0_ENABLE,LOW);
}



/**
 * @param index the sensor to read
 * @param result where to store the returned value.  may be changed even if method fails.
 * @return 0 on fail, 1 on success.
// @see https://ams.com/documents/20143/36005/AS5147_DS000307_2-00.pdf
 */
boolean getSensorRawValue(uint16_t &result) {
  uint8_t j, parity=0;

  result=0;
  
  // Send the request for the angle value (command 0xFFFF)
  // at the same time as receiving an angle.

  // Collect the 16 bits of data from the sensor
  digitalWrite(AS5147_CSEL,LOW);  // csel
  
  for(int i=0;i<SENSOR_TOTAL_BITS;++i) {
    digitalWrite(AS5147_CLK,HIGH);  // clk
    // this is here to give a little more time to the clock going high.
    // only needed if the arduino is *very* fast.  I'm feeling generous.
    result <<= 1;
    digitalWrite(AS5147_CLK,LOW);  // clk
    
    j = digitalRead(AS5147_MISO);  // miso
#ifdef VERBOSE
    Serial.print(j,DEC);
#endif
    result |= j;
    parity ^= (i>0) & j;
  }

  digitalWrite(AS5147_CSEL,HIGH);  // csel
  
  // check the parity bit
  return ( parity != (result>>SENSOR_DATA_BITS) );
}


/**
 * @param rawValue 16 bit value from as4157 sensor, including parity and EF bit
 * @return degrees calculated from bottom 14 bits.
 */
float extractAngleFromRawValue(uint16_t rawValue) {
  return (float)(rawValue & BOTTOM_14_MASK) * 360.0 / (float)(1<<SENSOR_ANGLE_BITS);
}


void sensorUpdate(float &sensorAngle) {
  uint16_t rawValue;
  if(getSensorRawValue(rawValue)) return;
  sensorAngle = extractAngleFromRawValue(rawValue);
}


void fullCircle(uint16_t wait,float first) {
  float angleNow;
  
  for(int i=0;i<MOTOR_STEPS_PER_TURN;++i) {
    digitalWrite(MOTOR_0_STEP,HIGH);
    digitalWrite(MOTOR_0_STEP,LOW);
    if((i%20)==0) {
      sensorUpdate(angleNow);
      Serial.println(WRAP_DEGREES(angleNow-first));
    }
    delayMicroseconds(wait + abs(i-MOTOR_STEPS_PER_TURN/2)/10);
  }
}


void loop() {
  float first=0;
  sensorUpdate(first);
      
  digitalWrite(MOTOR_0_DIR,LOW);
  fullCircle(100,first);
  digitalWrite(MOTOR_0_DIR,HIGH);
  fullCircle(100,first);
}
