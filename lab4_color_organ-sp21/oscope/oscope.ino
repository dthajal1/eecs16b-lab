/*
 * oscope.ino
 *
 * Displays voltage output, peak-to-peak voltage, and
 * a constant threshold voltage on the serial plotter.
 *
 * EECS16B Spring 2021
 * Rafael Calleja
 *
 */


#define BAUD_RATE         9600
// can't use 6.2 and 7.0
#define MIC_IN            P6_3
#define PEAK_DTCTR        P6_0 // not sure this is working // eyeball it (just focus on blue plot -- attenuates)
#define CALIB_PIN         P6_5 
#define BUTTON            PUSH1 //P2.1, bottom left of board

#define CALIBRATION_TIME  1000
#define THRESHOLD         2.5
#define DEFAULT_P2P       0
#define MAX_VOLTAGE       3.45 //3.3V voltage regulator reading from your multimeter (will probably be between 3.3-3.45V
#define MAX_PEAK          2.3

int buttonState = 0;
float conversion_factor = 0;
float peak_to_peak = 0;
float half_max = MAX_VOLTAGE/2;


void setup() {

  Serial.begin(BAUD_RATE);

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_IN, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  reset_blinker();
  conversion_factor = calibrate();
  Serial.println();
  Serial.println();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  buttonState = digitalRead(BUTTON);


  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == LOW) {
    conversion_factor = calibrate();

  }
  else {
    // turn LED off:
    digitalWrite(RED_LED, LOW);


    float val = read_mic(conversion_factor, MIC_IN);

    int t_init = millis();
    int t = millis();
    float max_val = 1;
    float min_val = MAX_VOLTAGE;
    val = read_mic(conversion_factor, MIC_IN);
    peak_to_peak = read_mic(conversion_factor, PEAK_DTCTR);
    
    plot(val, peak_to_peak);
    peak_to_peak = max_val-min_val;
  }



}

float calibrate(){
  int running_out = 3*CALIBRATION_TIME/4;

  // turn LED on:
  digitalWrite(RED_LED, HIGH);
  int t_init = millis();
  int t = millis();
  while (t < t_init+running_out){
    conversion_factor = get_conv();
    digitalWrite(GREEN_LED, HIGH);
    delay(200);
    digitalWrite(GREEN_LED, LOW);
    delay(200);
    t = millis();
   }


   while (t < t_init+CALIBRATION_TIME){
    conversion_factor = get_conv();
    digitalWrite(GREEN_LED, HIGH);
    delay(100);
    digitalWrite(GREEN_LED, LOW);
    delay(100);
    t = millis();
  }

  return conversion_factor;

}

float get_conv() {
  float max_sensor = analogRead(CALIB_PIN);
  float conversion_factor = MAX_VOLTAGE/max_sensor;
  return conversion_factor;
  }

float read_mic(float conv, int pin) {
  float v_out_int = analogRead(pin);
  float v_out = v_out_int*conv;
  return v_out;

}

void plot(float val, float peak_to_peak) {
  float p2p = max(2*peak_to_peak - MAX_VOLTAGE+0.6, 0)*MAX_VOLTAGE/MAX_PEAK;
  Serial.print("MIC_OUT:"); Serial.print(val - half_max,2); Serial.print(",");
  Serial.print("2.5V_THRESHOLD:"); Serial.print(THRESHOLD,2); Serial.print(",");
  Serial.print("P2P-"); Serial.print(p2p,2), Serial.print(":"); Serial.println(p2p,2);
//  Serial.print("PEAK-to-PEAK-"); Serial.print(peak_to_peak,2), Serial.print(":"); Serial.println(peak_to_peak-4,2);
}

void reset_blinker() {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}
