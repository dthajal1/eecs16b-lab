/*
 * classify.ino
 *
 * EE16B Spring 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

// Enveloping and K-means constants
//#define SNIPPET_SIZE                90
//#define PRELENGTH                   4
//#define THRESHOLD                   0.4
#define SNIPPET_SIZE                80
#define PRELENGTH                   5
#define THRESHOLD                   0.4


#define EUCLIDEAN_THRESHOLD         0.03
#define LOUDNESS_THRESHOLD          100
//if the Launchpad classifies noise, increase this constant. If it misses a lot of speech (i.e. thinks your word is noise), decrease this constant.

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/

float pca_vec1[80] = {0.03869854543639116, 0.036428853892088164, 0.039431791670065386, 0.046912839131675876, 0.05986599291958975, 0.10820645559799724, 0.21371533618177915, 0.22700575980425633, 0.22106302914386766, 0.2252424838461879, 0.21990648897432538, 0.20453769112578365, 0.2295284661781375, 0.21221847759357557, 0.2203646426543762, 0.2103656136358604, 0.19900549333166692, 0.19620886314458044, 0.16090562756946128, 0.16189453104905288, 0.15008451246367252, 0.09876213645969847, 0.091308088019744, 0.0746846854133438, 0.03369514901199983, 0.009245538982940676, 0.005024378733595692, -0.023013809301157297, -0.03373278004971827, -0.018340276509715828, -0.0333217028023679, -0.032186264940939735, -0.04172639050159669, -0.0374444196866863, -0.046445524590973446, -0.042442334434348604, -0.03785650122588442, -0.0336733080693098, -0.044086414110481566, -0.05677041578868925, -0.07048909985745218, -0.08041031000829295, -0.0952910578716107, -0.09582379634871434, -0.09256313240036676, -0.09028609327968044, -0.08972006637316449, -0.08501993521530227, -0.07785396221601026, -0.07425860914832175, -0.05831567989207656, -0.0547455114638402, -0.043071555134237334, -0.04456022764550297, -0.03574114351602143, -0.050096862512550716, -0.040086091135675996, -0.03372801427209096, -0.03539192069241568, -0.028179133561590006, -0.03140600114226702, -0.04536842121508989, -0.04895597912571438, -0.05936136590882844, -0.05822329099807959, -0.06498452160907107, -0.0690394960361357, -0.10156853786048053, -0.11780650176240186, -0.09314684143941307, -0.12287721360140863, -0.13881597398017895, -0.13401610031352487, -0.13463110957977623, -0.1225602595304251, -0.13176178514250433, -0.12673473300133345, -0.12527398685206065, -0.12535925647295526, -0.08574775183727777};
float pca_vec2[80] = {-0.00039459896029524867, 0.008156628721781908, 0.036461570212846274, 0.05119965157220456, -0.005853160063432047, -0.004957859344678628, -0.10669994953351658, -0.10928373916645097, -0.10016625000589528, -0.07264101387544418, -0.07728538132669636, -0.05580315797705868, -0.029084894020159874, -0.040610940494735, -0.007075695011159175, -0.0009368378522737041, 0.009806417951902275, 0.046020194192319605, 0.06129533923225646, 0.08757275590220208, 0.15194884750117188, 0.13205525741503563, 0.12070938654696618, 0.153279993234988, 0.14696331241651775, 0.1212670132457813, 0.13174392619837727, 0.1389503424458853, 0.12148044318754715, 0.1332581234934547, 0.1381894899582177, 0.10268949934996414, 0.12718703209741503, 0.13179846043967872, 0.09068134736670472, 0.1108037469100203, 0.10218784455891065, 0.06860033806842043, 0.04321269835271787, 0.0032444509534110652, -0.06424004245331931, -0.09993021356726932, -0.13872065934589264, -0.19053343877731663, -0.20360634505180936, -0.18840912931558004, -0.2207480554306236, -0.2080587001034464, -0.18123190442341755, -0.2142089166453975, -0.2019129171492834, -0.17178632888089726, -0.21169634363804227, -0.17922449963462475, -0.14762667486568248, -0.16227411393273977, -0.13714851894422772, -0.10857903062860053, -0.08215638625085071, -0.03418957045127231, -0.009142550443042484, -0.012955442481844758, 0.027770064698225853, 0.017910631765545287, 0.01926551145424295, 0.03840881874444651, 0.04093968714857265, 0.07843763642348131, 0.09872985477523062, 0.040866089965841075, 0.09309331198455645, 0.10360557502644908, 0.09481314464142258, 0.08884186433134632, 0.08302854582372358, 0.09049874827671002, 0.0875003829600066, 0.07456339416366531, 0.08109159333672222, 0.049044293000089556};
float projected_mean_vec[2] = {0.04269475792594029, 0.003505860759593858};
float centroid1[2] = {-0.009545886283275613, 0.03152919443592906};
float centroid2[2] = {-0.0018890863503692305, -0.03486408357121838};
float centroid3[2] = {0.05617889021407822, 0.0025346866493150657};
float centroid4[2] = {-0.05468701037608525, 0.0009780252606351838};
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

  pinMode(MIC_INPUT, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  re_pointer = 0;
  reset_blinker();
  setTimer();
}

void loop(void) {
  if (re_pointer == SIZE) {
    digitalWrite(RED_LED, LOW);

    // Apply enveloping function and get snippet with speech.
    // Do classification only if loud enough.
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*---------------------------*/

      // Project 'result' onto the principal components
      // Hint: 'result' is an array
      // Hint: do this entire operation in 1 loop by replacing the '...'
      // YOUR CODE HERE
      // the call to envelope leaves the data vector of your recording in the array called result.
      for (int i = 0; i < SNIPPET_SIZE; i++) {
          proj1 += result[i] * pca_vec1[i];
          proj2 += result[i] * pca_vec2[i];
      }
      

      // Demean the projection
      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];
//      Serial.println(proj1);
//      Serial.println(proj2);

      // Classification
      // Use the function 'l2_norm' defined above
      // ith centroid: 'centroids[i]'
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
//      Serial.println(best_dist);


      // Compare 'best_dist' against the 'EUCLIDEAN_THRESHOLD' and print the result
      // If 'best_dist' is less than the 'EUCLIDEAN_THRESHOLD', the recording is a word
      // Otherwise, the recording is noise
      // YOUR CODE HERE
      if (best_dist >= EUCLIDEAN_THRESHOLD) {
        
        Serial.println("MyError: This threshold was not satisfied! The recording is noise");
      } else {
        Serial.println(best_index);
        Serial.println("Threshold satisfied, the recording is a word!");
      }


      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }
    else {
      Serial.println("Below LOUDNESS_THRESHOLD.");
    }


    delay(2000);
    re_pointer = 0;
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

void reset_blinker(void) {
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

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (re_pointer < SIZE) {
    digitalWrite(RED_LED, HIGH);
    re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(void) {
  // Set the timer based on 25MHz clock
  TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
  TA2CCTL0 = CCIE;
  __bis_SR_register(GIE);
  TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
}
