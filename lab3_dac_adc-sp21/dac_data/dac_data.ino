/* Set the serial connection speed */
#define BAUD_RATE       9600

/* Set which bits correspond to which pins */
#define BIT_3           P3_2
#define BIT_2           P2_7
#define BIT_1           P4_2
#define BIT_0           P4_1
#define READ_PIN        P6_0

/* Set number of bits */
//#define BITS            3
#define BITS            4

/* Set how long to delay in between digital outputs */
#define DELAY           500

int val = 0;
int added = 1;
int loop_counter = 0;
float max_voltage;
float conversion_factor;
int plot_flag = 1;

void setup()
{
  // Start a serial connection to send data to the computer
  Serial.begin(BAUD_RATE);

  // Set pins to output mode
  pinMode(BIT_3, OUTPUT);
  pinMode(BIT_2, OUTPUT);
  pinMode(BIT_1, OUTPUT);
  pinMode(BIT_0, OUTPUT);
  pinMode(READ_PIN, INPUT);
  max_voltage = 3.3*((1<<BITS) - 1)/(1<<BITS);
  reset_blinker();
}

void loop()
{
   int max_loops = get_max_loops();

  // Calibrate sensor values to actual voltages
  conversion_factor = calibrate();

  int start_loop = millis();
  // If LSB is set in val, turn bit0 high
  if (val & 1)
    digitalWrite(BIT_0, HIGH);
  else
    digitalWrite(BIT_0, LOW);

  // If bit 1 is set in val, turn bit1 high
  if (val & 2)
    digitalWrite(BIT_1, HIGH);
  else
    digitalWrite(BIT_1, LOW);

  // If bit 2 is set in val, turn bit2 high
  if (val & 4)
    digitalWrite(BIT_2, HIGH);
  else
    digitalWrite(BIT_2, LOW);


  if (BITS == 4){
    // If bit 3 is set in val, turn bit3 high
    if(val & 8)
    digitalWrite(BIT_3, HIGH);
    else
    digitalWrite(BIT_3, LOW);

    // Range of values is 0 to 15 for 4 bit DAC
    if(val == 15)
    added = -1;
  }


  if (BITS == 3){
    // Range of values is 0 to 7 for 4 bit DAC
    if (val == 7)
      added = -1;
  }


  if (val == 0)
    added = 1;

  val = val + added;


  // Plot results
  if (loop_counter == 0 && plot_flag != 1){
    Serial.println("Values: ");
    Serial.print("[");
  }


  if (loop_counter<max_loops){
    plot();
  }

  loop_counter += 1;

    if (loop_counter == max_loops && plot_flag != 1){
    Serial.println("0.0000000000]");
    loop_counter = 0;
  }

  delay(DELAY);
  //Serial.print("loops: ");
  //Serial.println(loop_counter,DEC);

}

// Reset success indicator
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

float calibrate() {
  digitalWrite(BIT_0, HIGH);
  digitalWrite(BIT_1, HIGH);
  digitalWrite(BIT_2, HIGH);
  digitalWrite(BIT_3, HIGH);
  delay(100);
  float max_sensor = analogRead(READ_PIN);
  digitalWrite(BIT_0, LOW);
  digitalWrite(BIT_1, LOW);
  digitalWrite(BIT_2, LOW);
  digitalWrite(BIT_3, LOW);
  return conversion_factor = max_voltage/max_sensor;
  }

  void plot() {
    float v_out_int = analogRead(READ_PIN);
    float v_out = v_out_int*conversion_factor;
    if (plot_flag == 1) {
      for (int i = 0; i < 20; i+=1) {
    		Serial.println(v_out,DEC);
    	}
    } else {
    	for (int i = 0; i < 20; i+=1) {
        Serial.print(v_out, DEC);
        Serial.print(", ");
      }
    }
  }

int get_max_loops(){
    if (BITS == 3){
      return 15;
    }
    else if (BITS == 4){
      return 31;
    }
}
