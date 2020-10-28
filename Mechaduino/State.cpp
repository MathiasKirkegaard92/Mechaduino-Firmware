//Contains the declaration of the state variables for the control loop

#include "State.h"
#include "math.h"

#ifdef SAMPLE_ON_REQUEST
const int BUFLENGTH = 2;
#else
const int BUFLENGTH =  fmin(FPID_FREQ / HAPTIC_LOOP_FREQ,4);
#endif

volatile int U = 0;       //control effort (abs)
volatile float r = 0.0;   //setpoint
volatile float y = 0.0;   // me		asured angle
volatile float v = 0.0;  // estimated velocity  (velocity loop)
volatile float yw = 0.0;  // "wrapped" angle (not limited to 0-360)
volatile float yw_1 = 0.0;
volatile float e = 0.0;   // e = r-y (error)
volatile float p = 0.0;   // proportional effort
volatile float i = 0.0;   // integral effort


volatile float u = 0.0;     //real control effort (not abs)
volatile float u_1 = 0.0;   //value of u at previous time step, etc...
volatile float e_1 = 0.0;   //these past values can be useful for more complex controllers/filters
volatile float u_2 = 0.0;
volatile float e_2 = 0.0;
volatile float u_3 = 0.0;
volatile float e_3 = 0.0;
volatile long counter = 0;


volatile long wrap_count = 0;  //keeps track of how many revolutions the motor has gone though (so you can command angles outside of 0-360)
volatile float ybuf[BUFLENGTH] = {0};
volatile int itr_count = 0;

// Fixed point variables
volatile fixed_point_t ybuf_fixed[BUFLENGTH] = {0};
volatile fixed_point_t v_fixed = 0; 
volatile fixed_point_t yw_fixed = 0;
volatile fixed_point_t yw_1_fixed = 0;


volatile long step_count = 0;  //For step/dir interrupt (closed loop)
int stepNumber = 0; // open loop step number (used by 's' and for cal routine)

volatile float ITerm;
volatile float DTerm;


char mode;
int dir = false;

bool print_yw = false;      //for step response, under development...
bool print_v = false;
