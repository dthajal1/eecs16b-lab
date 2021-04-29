/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_1
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_3

#define SAMPLING_INTERVAL           100
int sample_lens[4] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1
int timer_mode = MODE_LISTEN;

#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

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
/*      From turning.ino     */
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
/*      From turning.ino     */
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
/*      From turning.ino     */
/*---------------------------*/

float delta_ss = 0;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 91 // in cm - 6 feet diameter = 3 tiles in 125 Cory
// #define TURN_RADIUS                 60 // in cm - 4 feet diameter = 2 tiles in 125 Cory

int run_times[4] = {7000, 2700, 2500, 2700};
//int run_times[4] = {7000, 5000, 2500, 5000};

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
  else { // DRIVE_FAR, DRIVE_CLOSE
    return 0;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*      From turning.ino     */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int n) {
  // YOUR CODE HERE
  return 0;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

#define MIC_INPUT                   P6_0

#define SIZE                        3200
#define SIZE_AFTER_FILTER           200
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*     From classify.ino     */
/*---------------------------*/

// Enveloping and threshold constants
//#define SNIPPET_SIZE                40
//#define PRELENGTH                   5
//#define THRESHOLD                   0.5
//
//#define EUCLIDEAN_THRESHOLD            0
//#define LOUDNESS_THRESHOLD          0
#define SNIPPET_SIZE                80
#define PRELENGTH                   5
#define THRESHOLD                   0.4


#define EUCLIDEAN_THRESHOLD         0.03
#define LOUDNESS_THRESHOLD          100 //if the Launchpad classifies noise, increase this constant. If it misses a lot of speech (i.e. thinks your word is noise), decrease this constant.

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/
float pca_vec1[SNIPPET_SIZE] = {0.03869854543639116, 0.036428853892088164, 0.039431791670065386, 0.046912839131675876, 0.05986599291958975, 0.10820645559799724, 0.21371533618177915, 0.22700575980425633, 0.22106302914386766, 0.2252424838461879, 0.21990648897432538, 0.20453769112578365, 0.2295284661781375, 0.21221847759357557, 0.2203646426543762, 0.2103656136358604, 0.19900549333166692, 0.19620886314458044, 0.16090562756946128, 0.16189453104905288, 0.15008451246367252, 0.09876213645969847, 0.091308088019744, 0.0746846854133438, 0.03369514901199983, 0.009245538982940676, 0.005024378733595692, -0.023013809301157297, -0.03373278004971827, -0.018340276509715828, -0.0333217028023679, -0.032186264940939735, -0.04172639050159669, -0.0374444196866863, -0.046445524590973446, -0.042442334434348604, -0.03785650122588442, -0.0336733080693098, -0.044086414110481566, -0.05677041578868925, -0.07048909985745218, -0.08041031000829295, -0.0952910578716107, -0.09582379634871434, -0.09256313240036676, -0.09028609327968044, -0.08972006637316449, -0.08501993521530227, -0.07785396221601026, -0.07425860914832175, -0.05831567989207656, -0.0547455114638402, -0.043071555134237334, -0.04456022764550297, -0.03574114351602143, -0.050096862512550716, -0.040086091135675996, -0.03372801427209096, -0.03539192069241568, -0.028179133561590006, -0.03140600114226702, -0.04536842121508989, -0.04895597912571438, -0.05936136590882844, -0.05822329099807959, -0.06498452160907107, -0.0690394960361357, -0.10156853786048053, -0.11780650176240186, -0.09314684143941307, -0.12287721360140863, -0.13881597398017895, -0.13401610031352487, -0.13463110957977623, -0.1225602595304251, -0.13176178514250433, -0.12673473300133345, -0.12527398685206065, -0.12535925647295526, -0.08574775183727777};
float pca_vec2[SNIPPET_SIZE] = {-0.00039459896029524867, 0.008156628721781908, 0.036461570212846274, 0.05119965157220456, -0.005853160063432047, -0.004957859344678628, -0.10669994953351658, -0.10928373916645097, -0.10016625000589528, -0.07264101387544418, -0.07728538132669636, -0.05580315797705868, -0.029084894020159874, -0.040610940494735, -0.007075695011159175, -0.0009368378522737041, 0.009806417951902275, 0.046020194192319605, 0.06129533923225646, 0.08757275590220208, 0.15194884750117188, 0.13205525741503563, 0.12070938654696618, 0.153279993234988, 0.14696331241651775, 0.1212670132457813, 0.13174392619837727, 0.1389503424458853, 0.12148044318754715, 0.1332581234934547, 0.1381894899582177, 0.10268949934996414, 0.12718703209741503, 0.13179846043967872, 0.09068134736670472, 0.1108037469100203, 0.10218784455891065, 0.06860033806842043, 0.04321269835271787, 0.0032444509534110652, -0.06424004245331931, -0.09993021356726932, -0.13872065934589264, -0.19053343877731663, -0.20360634505180936, -0.18840912931558004, -0.2207480554306236, -0.2080587001034464, -0.18123190442341755, -0.2142089166453975, -0.2019129171492834, -0.17178632888089726, -0.21169634363804227, -0.17922449963462475, -0.14762667486568248, -0.16227411393273977, -0.13714851894422772, -0.10857903062860053, -0.08215638625085071, -0.03418957045127231, -0.009142550443042484, -0.012955442481844758, 0.027770064698225853, 0.017910631765545287, 0.01926551145424295, 0.03840881874444651, 0.04093968714857265, 0.07843763642348131, 0.09872985477523062, 0.040866089965841075, 0.09309331198455645, 0.10360557502644908, 0.09481314464142258, 0.08884186433134632, 0.08302854582372358, 0.09049874827671002, 0.0875003829600066, 0.07456339416366531, 0.08109159333672222, 0.049044293000089556};
float projected_mean_vec[2] = {0.04269475792594029, 0.003505860759593858};
float centroid1[2] = {-0.009545886283275613, 0.03152919443592906}; // DRIVE_FAR
float centroid2[2] = {-0.0018890863503692305, -0.03486408357121838}; // DRIVE_LEFT
float centroid3[2] = {0.05617889021407822, 0.0025346866493150657}; // DRIVE_CLOSE
float centroid4[2] = {-0.05468701037608525, 0.0009780252606351838}; // DRIVE_RIGHT
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------------------------------*/
/*---------------------------------------------------*/
/*---------------------------------------------------*/

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);

  for (int i = 0; i < 4; i++) {
    sample_lens[i] = run_times[i] / SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
  start_listen_mode();
}

void loop(void) {
  check_encoders();
  if (timer_mode == MODE_LISTEN && re_pointer == SIZE){
    // Stop motor
    write_pwm(0, 0);
    digitalWrite(RED_LED, LOW);

    // if enveloped data is above some preset value
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*     From classify.ino     */
      /*     with more changes     */
      /*---------------------------*/

      // Project 'result' on the principal components
      // YOUR CODE HERE
      // the call to envelope leaves the data vector of your recording in the array called result.
      for (int i = 0; i < SNIPPET_SIZE; i++) {
          proj1 += result[i] * pca_vec1[i];
          proj2 += result[i] * pca_vec2[i];
      }

      // Demean the projection
      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];

      // Classification
      // Use the function l2_norm defined above
      // ith centroid: centroids[i]
      // YOUR CODE HERE
      float best_dist = 999999;
      int best_index = -1;
      // YOUR CODE HERE
      for (int i = 0; i < 4; i++) {
        float dist = l2_norm(proj1, proj2, centroids[i]);
        if (dist < best_dist) {
          best_dist = dist;
          best_index = i;
        }
      }


      // Check against EUCLIDEAN_THRESHOLD and execute identified command
      // YOUR CODE HERE
      if (best_dist < EUCLIDEAN_THRESHOLD) {
        drive_mode = best_index; // from 0-3, inclusive
        start_drive_mode();
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    delay(2000);
    re_pointer = 0; // start recording from beginning if we don't start driving
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

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }

    do_loop = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

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

void start_listen_mode(void) {
  re_pointer = 0;
  write_pwm(0, 0);
  delay(3000); // 3 seconds buffer for mic cap settling
  timer_mode = MODE_LISTEN;
  setTimer(MODE_LISTEN);
}

void start_drive_mode(void) {
  timer_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
  setTimer(MODE_DRIVE);
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
void setTimer(boolean mode) {
  if (mode == MODE_LISTEN) {
    // Set the timer based on 25MHz clock
    TB0CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
    TB0CCTL0 = CCIE;
    __bis_SR_register(GIE);
    TB0CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  }
  else if (mode == MODE_DRIVE) {
    TB0CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
    TB0CCTL0 = CCIE; // enable interrupts for Timer B
    __bis_SR_register(GIE);
    TB0CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  timer_mode = mode;
}

// ISR for timestep
#pragma vector=TIMER0_B0_VECTOR    // Timer B ISR
__interrupt void Timer0_B0_ISR(void) {
  if (timer_mode == MODE_LISTEN) {
    if (re_pointer < SIZE) {
      digitalWrite(RED_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
      re_pointer += 1;
    }
  }
  else if (timer_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
