/*
 * turning.ino
 *
 * Implementing turns in SIXT33N
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_1
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_3
#define PUSH_START                  PUSH1

#define SAMPLING_INTERVAL           100

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1

#define NUM_COMMANDS                8 //4 // YOUR CODE HERE
#define DRIVE_STRAIGHT              0
#define DRIVE_LEFT                  1
#define DRIVE_RIGHT                 2
#define STOP                        3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode;
int program_count;
int sample_lens[NUM_COMMANDS] = {0};

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*    From closed_loop.ino   */
/*---------------------------*/

float theta_left = 0.1371;
float theta_right = 0.1688;
float beta_left = -53.96;
float beta_right = -54.68;
float v_star = 76.8;

// PWM inputs to jolt the car straight
int left_jolt = 195;
int right_jolt = 165;

// Control gains
float k_left = 0.5;
float k_right = 0.5;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*    From closed_loop.ino   */
/*---------------------------*/

float driveStraight_left(float delta) {
  float u_left = ( ((v_star + beta_left) / theta_left) - ((delta * k_left) / theta_left) );
  return theta_left * u_left - beta_left;
}

float driveStraight_right(float delta) {
  float u_right = ( ((v_star + beta_right) / theta_right) + ((delta * k_right) / theta_right) );
  return theta_right * u_right - beta_right;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*    From closed_loop.ino   */
/*---------------------------*/

float delta_ss = 0;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 91 // in cm - 6 feet diameter
// #define TURN_RADIUS                 60 // in cm - 4 feet diameter

/*---------------------------*/
/*    PREPROGRAMMED PATH     */
/*---------------------------*/
//int run_times[NUM_COMMANDS] = {7000, 5000, 2500, 5000}; // length of commands roughly in ms
int run_times[NUM_COMMANDS] = {1000, 2500, 1000, 2500, 1000, 2500, 1000, 2500}; // length of commands roughly in ms
int drive_modes[NUM_COMMANDS] = {DRIVE_STRAIGHT, DRIVE_LEFT, DRIVE_STRAIGHT, DRIVE_RIGHT, DRIVE_STRAIGHT, DRIVE_LEFT, DRIVE_STRAIGHT, DRIVE_RIGHT}; // commands: [DRIVE_STRAIGHT, DRIVE_LEFT, DRIVE_RIGHT]

float delta_reference(int n) {
  // YOUR CODE HERE
  //  use TURN_RADIUS, v_star, CAR_WIDTH, n
  float v_star_over_m = v_star / 5;
  float theta = (v_star_over_m * n) / TURN_RADIUS;
  float delta_n = CAR_WIDTH * theta;
  
  if (drive_mode == DRIVE_RIGHT) {
    return delta_n;
  }
  else if (drive_mode == DRIVE_LEFT) {
    return -1 * delta_n;
  }
  else { // DRIVE_STRAIGHT
    return 0;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int n) {
  // YOUR CODE HERE
  return 0; // Replace this line
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  for (int i = 0; i < NUM_COMMANDS; i++) {
    sample_lens[i] = run_times[i] / SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
  setTimer(); // Set timer for timestep
  program_count = 0;
  drive_mode = drive_modes[0];
  start_drive_mode();
}


void loop(void) {
  check_encoders();
  if (program_count == NUM_COMMANDS) {
    if (!digitalRead(PUSH_START)) {
      program_count = 0;
      drive_mode = drive_modes[0];
      start_drive_mode();
    }
    else {
      idle_blinker();
    }
  }
  else if (loop_mode == MODE_LISTEN) {
    // In the integration phase of the project, this section will listen
    // to the microphone and switch to the specified mode.
    // For now, we simply cycle through them.
    program_count++;
    drive_mode = drive_modes[program_count];

    start_drive_mode();
  }

  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta - delta_reference(step_num) - straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta);
      int right_cur_pwm = driveStraight_right(delta);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[program_count]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }

    do_loop = 0;
  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void start_drive_mode(void) {
  loop_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
}

void start_listen_mode(void) {
  write_pwm(0, 0);
  delay(3000);
  loop_mode = MODE_LISTEN;
}

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
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

void idle_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(500);
  digitalWrite(RED_LED, LOW);
  delay(500);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.1*4096))
#define HIGH_THRESH                 ((int) (0.4*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use B0 to free up all other PWM ports
void setTimer(void) {
  TB0CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
  TB0CCTL0 = CCIE; // enable interrupts for Timer B
  __bis_SR_register(GIE);
  TB0CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
}

// ISR for timestep
#pragma vector=TIMER0_B0_VECTOR    // Timer B ISR
__interrupt void Timer0_B0_ISR(void) {
  if (loop_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
