#include "msp.h"


/**
 * EGR 326 Andrew Kruse 9/11/2020
 * This programs debounce is based in interrupts.
 * When the push button is pressed, the port interrupt is triggered.
 * This routine, starts the SysTick timer.
 * Every time the timer completes, it triggers an interrupt.
 * They systick interrupt routine, calls the debounce state shift function.
 * if the button press is good, the state machine proceeds
 * to next state.
 *
 */
#define DBTIME 5 //for 5ms each check of timer

#define PB1Port P1
#define PB1Pin BIT1
#define PB2Port P1
#define PB2Pin BIT4

#define LEDR_Port P2
#define LEDR_Pin BIT0
#define LEDG_Port P2
#define LEDG_Pin BIT1
#define LEDB_Port P2
#define LEDB_Pin BIT2

//possible states
enum states {ALL_OFF,RED,GREEN,BLUE};

//structure used to hold previous current and next state.
typedef struct {
    int prevState;
    int currState;
    int nextState;
}C_States;

//prototyping
void configTimer32_1(int delay);
void LED_Init();
void push_Button_Init();
void debounceSwitch();
void stateAdvance(C_States *);
void stateReverse(C_States *);
void stateInitialize(C_States *);
void stateAction(C_States *);
void A_OFF();
void G_ON();
void G_OFF();
void R_ON();
void R_OFF();
void B_ON();
void B_OFF();

//global variable for state machine structure
C_States STATE_MACHINE;

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    __disable_interrupt();

    //initalize input/output pins
    LED_Init();
    push_Button_Init();

    //configure state machine structure
    stateInitialize(&STATE_MACHINE);

    //change input output based upon statemachine structure
    stateAction(&STATE_MACHINE);

    //enable push button interrupt routines
    NVIC->ISER[1] = 1 << ((PORT1_IRQn) & 31);
    NVIC->ISER[0] = 1 << ((T32_INT1_IRQn));

    //enable global interrupts
    __enable_interrupt();

    //main processing loop, all work done by interrupts and peripherals.
    while(1);
}

void LED_Init(){
    LEDR_Port->SEL0 &= ~LEDR_Pin;
    LEDR_Port->SEL1 &= ~LEDR_Pin;
    LEDR_Port->DIR |= LEDR_Pin;

    LEDG_Port->SEL0 &= ~LEDG_Pin;
    LEDG_Port->SEL1 &= ~LEDG_Pin;
    LEDG_Port->DIR |= LEDG_Pin;

    LEDB_Port->SEL0 &= ~LEDB_Pin;
    LEDB_Port->SEL1 &= ~LEDB_Pin;
    LEDB_Port->DIR |= LEDB_Pin;

    A_OFF();
}

void push_Button_Init(){
   PB1Port->SEL0 &= ~PB1Pin;        //set GPIO
   PB1Port->SEL1 &= ~PB1Pin;        //set GPIO
   PB1Port->DIR &= ~PB1Pin;         //set low aka input
   PB1Port->REN |= PB1Pin;          //set pull up resistor enabled
   PB1Port->OUT |= PB1Pin;          //set to assert when high to low
   PB1Port->IES |= PB1Pin;          //Interrupt when assert low occurs
   PB1Port->IE  |= PB1Pin;          //Enable interrupt on this pin
   PB1Port->IFG = 0;                //clear flag;

   PB2Port->SEL0 &= ~PB2Pin;        //set GPIO
   PB2Port->SEL1 &= ~PB2Pin;        //set GPIO
   PB2Port->DIR &= ~PB2Pin;         //set low aka input
   PB2Port->REN |= PB2Pin;          //set pull up resistor enabled
   PB2Port->OUT |= PB2Pin;          //set to assert when high to low
   PB2Port->IES |= PB2Pin;          //Interrupt when assert low occurs.
   PB2Port->IE  |= PB2Pin;          //Enable interrupt on this pin
   PB2Port->IFG = 0;                //Clear Flag;
}

void sys_config(int delay){
    SysTick->LOAD= (delay * 3000 -1);
    SysTick->CTRL=7;
}

void configTimer32_1(int delay){
    TIMER32_1->CONTROL &= ~BIT0; //wrapping mode, continuous operation
    TIMER32_1->CONTROL |= BIT1;//32-bit counter
    TIMER32_1->CONTROL &= ~(BIT2|BIT3);//no prescaler
    TIMER32_1->CONTROL |= BIT5;
    TIMER32_1->CONTROL |= BIT6; //periodic mode, looks at load register

    //set load register to delay number of ms
    TIMER32_1->LOAD = (delay * 3000 -1);

    //Turn on Timer32_1
    TIMER32_1->CONTROL |= BIT7; //enables timer to begin counting.
}

void T32_INT1_IRQHandler(void){
    if(TIMER32_1->RIS & BIT0){

        //shift the debounce registers
    debounceSwitch();
    }
    TIMER32_1->INTCLR = 0;
}

void debounceSwitch(){

    static uint16_t StatePB1 = 0xfc00;
    static uint16_t StatePB2 = 0xfc00;

    //every 5ms as triggered by Systic interrupt, this register will shift,
    //after 10 shifts, should read 0b1111 1100 0000 0000 for a good bounce.
    StatePB1 = (StatePB1<<1) | (PB1Port->IN & PB1Pin) >> 1 | 0xf800;
    StatePB2 = (StatePB2<<1) | (PB2Port->IN & PB2Pin) >> 4 | 0xf800;


    //if state is 0xffff in both buttons, not a good press, stop timer
    if((StatePB1 == 0xffff) && (StatePB2 == 0xffff)){
        TIMER32_1->CONTROL = 0;
        StatePB1 = 0xffff;
        StatePB2 = 0xffff;
    }

    //if both buttons are good and pressed at same time, turn off lights
    else if(StatePB1 == 0xfc00 && StatePB2 == 0xfc00){
        stateInitialize(&STATE_MACHINE);
        stateAction(&STATE_MACHINE);
        TIMER32_1->CONTROL = 0;
        StatePB1 = 0xffff;
        StatePB2 = 0xffff;
    }

    //advance state is pb1 is a good button
    else if(StatePB1 == 0xfc00){
        stateAdvance(&STATE_MACHINE);
        stateAction(&STATE_MACHINE);
        TIMER32_1->CONTROL = 0;
        StatePB1 = 0xffff;
        StatePB2 = 0xffff;
    }

    //reverse state if pb2 is a good button
    else if(StatePB2 == 0xfc00){
        stateReverse(&STATE_MACHINE);
        stateAction(&STATE_MACHINE);
        TIMER32_1->CONTROL = 0;
        StatePB1 = 0xffff;
        StatePB2 = 0xffff;
    }

    //else 50ms not elapsed/ no good/ bad button finalized
}

//this is the push button interrupt handler
//responsible for intializing the Systick Timer
//to debounce buttons.
void PORT1_IRQHandler(){

    //debounce either input button
    if((PB1Port->IFG & PB1Pin)){
        PB1Port->IFG = 0;
        configTimer32_1(DBTIME);
    }
    else if(PB1Port->IFG & PB2Pin){
        PB1Port->IFG = 0;
        configTimer32_1(DBTIME);
    }
    PB1Port->IFG = 0;
}
//if pb1 is pushed, state will advance through sequence
//red, green, blue, red green blue etc. etc. on each
//subsequent pb1 button press
void stateAdvance(C_States *s1){
    s1->prevState = s1->currState;
    s1->currState = s1->nextState;
    if(s1->nextState == BLUE){
        s1->nextState = RED;
    }
    else{
        s1->nextState++;
    }
}

//if pb2 is pushed, state will reverse through sequence
//blue, green, red, blue green red etc. etc. on each
//subsequent pb2 button press
void stateReverse(C_States *s1){
    s1->nextState = s1->currState;
    s1->currState = s1->prevState;
    if(s1->prevState == RED){
        s1->prevState = BLUE;
    }
    else{
        s1->prevState--;
    }
}

//this is the reset state, all lights off
//Blue is previous, if Red is advanced
void stateInitialize(C_States *s1){
    s1->prevState = BLUE;
    s1->currState = ALL_OFF;
    s1->nextState = RED;
}

//this function processes all the outputs associated with each state
//transition, based upon the state machine's current structures
//actions are processed using a case statement for each
//enumerated state type calling functions to manipulate
//port registers
void stateAction(C_States *s1){

    switch(s1->currState){
    case ALL_OFF:
        A_OFF();
        break;
    case RED:
        A_OFF();
        R_ON();
        break;
    case GREEN:
        A_OFF();
        G_ON();
        break;
    case BLUE:
        A_OFF();
        B_ON();
        break;
    }
}

//manipulates registers to turn on green light
void G_ON(){
    LEDG_Port->OUT |= LEDG_Pin;
}

//manipulates registers to turn off green light
void G_OFF(){
    LEDG_Port->OUT &= ~LEDG_Pin;
}

//manipulates registers to turn on red light
void R_ON(){
    LEDR_Port->OUT |= LEDR_Pin;
}

//manipulates registers to turn off red light
void R_OFF(){
    LEDR_Port->OUT &= ~LEDR_Pin;
}

//manipulates registers to turn on blue light
void B_ON(){
    LEDB_Port->OUT |= LEDB_Pin;
}

//manipulates registers to turn off blue light
void B_OFF(){
    LEDB_Port->OUT &= ~LEDB_Pin;
}

//function used to turn of all lights
//by calling register manipulation functions
//associated with that task
void A_OFF(){
    G_OFF();
    R_OFF();
    B_OFF();
}
