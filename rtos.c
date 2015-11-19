// RTOS Framework for Project 1
// Fall 2014
// Jason Losh

//-----------------------------------------------------------------------------
// Objectives and notes             
//-----------------------------------------------------------------------------

// Target uC:       33FJ128MC802
// Devices used:    LEDs and PBs

// Hardware description:
// Red LED
//   anode connected through 100ohm resistor to RB5 (pin 14), cathode grounded
// Green LED
//   anode connected through 100ohm resistor to RB4 (pin 11), cathode grounded
// Yellow LED
//   anode connected through 100ohm resistor to RB3 (pin 7), cathode grounded
// Orange LED
//   anode connected through 100ohm resistor to RB2 (pin 6), cathode grounded
// Push Buttons
//   push button 0 connected between RB12 (pin 23) and ground
//   push button 1 connected between RB13 (pin 24) and ground
//   push button 2 connected between RB14 (pin 25) and ground
//   push button 3 connected between RB15 (pin 26) and ground

//-----------------------------------------------------------------------------
// Device includes and assembler directives             
//-----------------------------------------------------------------------------

#include <p33FJ128MC802.h>
#include <stdio.h>
#define FCY 40000000UL                       // instruction cycle rate
#include <libpic30.h>                        // __delay32
                                             // __delay_ms (max value is 268)
                                             // __delay_us

#define PIN_YELLOW LATBbits.LATB2            // define i/o... change as needed
#define PIN_ORANGE LATBbits.LATB3
#define PIN_GREEN LATBbits.LATB4
#define PIN_RED LATBbits.LATB5
#define PIN_PB0 PORTBbits.RB12
#define PIN_PB1 PORTBbits.RB13
#define PIN_PB2 PORTBbits.RB14
#define PIN_PB3 PORTBbits.RB15

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables                
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

#define TRUE  1
#define FALSE 0

// semaphore
#define MAX_QUEUE_SIZE 10	

struct semaphore
{
  unsigned int count;
  unsigned int queue_size;
  unsigned int process_queue[MAX_QUEUE_SIZE]; 	// store task index here
} *s, key_pressed, key_released, flash_req;


// task 
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // ready, but never run
#define STATE_READY      2 // ready to run
#define STATE_BLOCKED    3 // has run, but now blocked by semaphore
#define STATE_DELAYED    4 // has run, but now awaiting timer

#define MAX_TASKS 10       // maximum number of valid tasks
int task_current = 0;      // index of last dispatched task
int task_count = 0;        // total number of valid tasks

int rtos_mode;             // rtos mode
#define MODE_COOPERATIVE 0
#define MODE_PREEMPTIVE  1


#define SHORT_RESTORE 0    // task switching restore type
#define LONG_RESTORE 1


int *SP = (int*)0x1E;	   // Pointer to stack register WREG15


/* #################################################################################### */
struct _tcb
{
  	unsigned int state;            	// see STATE_ values above
  	unsigned int pid;              	// used to uniquely identify process
  	unsigned int sp;               	// location of stack pointer for process
  	unsigned int priority;         	// 0=lowest, 7=highest
  	unsigned int current_priority;
  	unsigned int ticks;            	// ticks until sleep complete
	unsigned int sp_limit;			// used to store SPLIM value
	unsigned int skip_cntr;			// used for Prioritization
	unsigned int restore_type;		// used to save push-pop restore type
	void* waitOnSemaphore_add;		// Keep track of semaphores task is using

} tcb[MAX_TASKS];

unsigned int stack[MAX_TASKS][256];


//-----------------------------------------------------------------------------
// RTOS Kernel                
//-----------------------------------------------------------------------------

void rtos_init(int mode)
{
  int i;
  rtos_mode = mode;
  // no tasks running
  task_count = 0;
  // clear out tcb records
  for (i = 0; i < MAX_TASKS; i++)
  {
    tcb[i].state = STATE_INVALID;
    tcb[i].pid = 0;
  }
  // Setup timer 3 in a separate function (timer3_init)
}


/* #################################################################################### */
int rtos_scheduler()
{
  	static int ok;
  	static int task = 0xFF;
  	ok = FALSE;
  	while (!ok)
  	{
    	task++;
    	if (task >= MAX_TASKS)
      		task = 0;
		if((tcb[task].state == STATE_READY) || (tcb[task].state == STATE_UNRUN))
    	{
			if(tcb[task].skip_cntr <= 0)
			{
				ok = TRUE;
			}
			else
			{
				tcb[task].skip_cntr--;
			}
		}
  	}
	tcb[task].skip_cntr = 7-tcb[task].priority;
  	return task;
}


/* #################################################################################### */
int create_process(_fn fn, int priority)
{
  int ok = FALSE;
  int i = 0;
  int found = FALSE;
  IEC0bits.T3IE = 0; 
  // save starting address if room in task list
  if (task_count < MAX_TASKS)
  {
    // make sure fn not already in list (prevent reentrancy)
    while (!found && (i < MAX_TASKS))
    {
      found = (tcb[i++].pid == (unsigned int) fn);
    }
    if (!found)
    {
      // find first available tcb record
      i = 0;
      while (tcb[i].state != STATE_INVALID) {i++;}
	  tcb[i].state = STATE_UNRUN;
      tcb[i].pid = (unsigned int) fn;
      tcb[i].sp = (int)stack[i];
	  tcb[i].sp_limit = (int)&stack[i][253];
      tcb[i].priority = priority;   
	  tcb[i].skip_cntr = 7-priority; 
      tcb[i].current_priority = priority; 
	  tcb[i].waitOnSemaphore_add = 0;
      // increment task count
      task_count++;
      ok = TRUE;
    }
  }
  IEC0bits.T3IE = 1;
  return ok;
}


/* #################################################################################### */
// REQUIRED: modify this function to destroy a process
int destroy_process(_fn fn)
{
	int i,j,k;								// index variables for three 'for' loops
	for(i=0; i<task_count;i++)
	{
		if(tcb[i].pid == (unsigned int) fn) // Check if we have any task with that same argument
		{
			if(i == task_current)			// Check if we are trying to delete the task that called this func.
				return 0;
			task_count--;
			tcb[i].pid = 0;
			tcb[i].state = STATE_INVALID;


			// Check semaphores queue 
			if(tcb[i].waitOnSemaphore_add != 0)
			{
				s = tcb[i].waitOnSemaphore_add;
				for(j=0; j<s->queue_size;j++)
				{
					if(s->process_queue[j] == i)
					{
						s->queue_size--;
						for(k=j;k<s->queue_size;k++)
							s->process_queue[k] = s->process_queue[k+1];
						break;
					}
				}
			}
			break;
		}
	}	
	return 1;
}


/* #################################################################################### */
void rtos_start()
{
  	_fn fn;

	task_current =  rtos_scheduler();
  	*SP = tcb[task_current].sp;
	SPLIM = tcb[task_current].sp_limit;
  	fn = (_fn)tcb[task_current].pid;
	tcb[task_current].state = STATE_READY;

	IFS0bits.T3IF = 0;		// Clear the flag
	IEC0bits.T3IE = 1;		// Enable timer 3 interrupts
	T3CONbits.TON = 1;		// Timer3 ON
  	(*fn)();				// Call the first task
}



/* #################################################################################### */
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt (void)
{	
	int task_previous;

	if(rtos_mode == MODE_PREEMPTIVE)
	{
		asm("ulnk");
    	asm("POP.D W6");
		asm("POP.D W4");
    	asm("POP.D W2");
    	asm("POP.D W2");
     	asm("POP RCOUNT");
     	asm("lnk #0x6");
		asm("PUSH CORCON");
		asm("PUSH SR");
		asm("PUSH DCOUNT");
		asm("PUSH RCOUNT");
		asm("PUSH PSVPAG");
		asm("PUSH TBLPAG");
		asm("PUSH W0");
		asm("PUSH W1");
		asm("PUSH W2");
		asm("PUSH W3");
		asm("PUSH W4");
		asm("PUSH W5");
		asm("PUSH W6");
		asm("PUSH W7");
		asm("PUSH W8");
		asm("PUSH W9");
		asm("PUSH W10");
		asm("PUSH W11");
		asm("PUSH W12");
		asm("PUSH W13");
		asm("PUSH W14");
		tcb[task_current].sp = *SP;
		tcb[task_current].restore_type = LONG_RESTORE;
		task_previous = task_current;
	}

	
	int task=0;
	while(task<MAX_TASKS)
	{
		if(tcb[task].state == STATE_DELAYED)
		{
			if(tcb[task].ticks != 0)
				tcb[task].ticks--;
			else if(tcb[task].ticks == 0) 
				tcb[task].state = STATE_READY;
		} 
		task++;		
	}

	if(rtos_mode == MODE_PREEMPTIVE)
	{
		task_current = rtos_scheduler();
		
		if(task_current >= task_previous)
		{
			SPLIM = tcb[task_current].sp_limit;
			*SP = tcb[task_current].sp;
		}
		else
		{
			*SP = tcb[task_current].sp;
			SPLIM = tcb[task_current].sp_limit;
		}


		IFS0bits.T3IF = 0;    // Clear the Flag   [because, after that we will jump to other functions. Also for short_restore if we lower the CPU priority (IPL) then we better fiorst clear the flag. Otherwise this interrupt func will be called recursively]


		if(tcb[task_current].state == STATE_UNRUN)
		{
			_fn fn;
			fn = (_fn)tcb[task_current].pid;
			tcb[task_current].state = STATE_READY;
			SRbits.IPL = 0;
	  		(*fn)();
		}
		else
		{
			if(tcb[task_current].restore_type == LONG_RESTORE)
			{
				asm("POP W14");
				asm("POP W13");
				asm("POP W12");
				asm("POP W11");
				asm("POP W10");
				asm("POP W9");
				asm("POP W8");
				asm("POP W7");
				asm("POP W6");
				asm("POP W5");
				asm("POP W4");
				asm("POP W3");
				asm("POP W2");
				asm("POP W1");
				asm("POP W0");
				asm("POP TBLPAG");
				asm("POP PSVPAG");
				asm("POP RCOUNT");
				asm("POP DCOUNT");
				asm("POP SR");
				asm("POP CORCON");
				asm("ULNK");
				asm("RETFIE");
			}
			else if(tcb[task_current].restore_type == SHORT_RESTORE)
			{
				SRbits.IPL = 0;
				asm("POP W14");
				asm("POP W13");
				asm("POP W12");
				asm("POP W11");
				asm("POP W10");
				asm("POP W9");
				asm("POP W8");
				asm("ULNK");
				asm("RETURN");
			}
		}
	}
	IFS0bits.T3IF = 0;    // Clear the Flag
}


/* #################################################################################### */
void init_semaphore(void* p, int count)
{
  	s = p;
  	s->count = count;  
  	s->queue_size = 0;
}


/* #################################################################################### */
void yield()
{
	IEC0bits.T3IE = 0;
	// push registers, call scheduler, pop registers, return to new function

	asm("PUSH W8");
	asm("PUSH W9");
	asm("PUSH W10");
	asm("PUSH W11");
	asm("PUSH W12");
	asm("PUSH W13");
	asm("PUSH W14");

	tcb[task_current].sp = *SP;
	tcb[task_current].restore_type = SHORT_RESTORE;
	int task_previous = task_current;
	task_current = rtos_scheduler();
	
	if(task_current >= task_previous)
	{
		SPLIM = tcb[task_current].sp_limit;
		*SP = tcb[task_current].sp;
	}
	else
	{
		*SP = tcb[task_current].sp;
		SPLIM = tcb[task_current].sp_limit;
	}
	if(tcb[task_current].state == STATE_UNRUN)
	{
		_fn fn;
		fn = (_fn)tcb[task_current].pid;
		tcb[task_current].state = STATE_READY;
		IEC0bits.T3IE = 1;
  		(*fn)();
	}
	else
	{
		if(tcb[task_current].restore_type == LONG_RESTORE)
		{
			asm("POP W14");
			asm("POP W13");
			asm("POP W12");
			asm("POP W11");
			asm("POP W10");
			asm("POP W9");
			asm("POP W8");
			asm("POP W7");
			asm("POP W6");
			asm("POP W5");
			asm("POP W4");
			asm("POP W3");
			asm("POP W2");
			asm("POP W1");
			asm("POP W0");
			asm("POP TBLPAG");
			asm("POP PSVPAG");
			asm("POP RCOUNT");
			asm("POP DCOUNT");
			asm("POP SR");
			asm("POP CORCON");
			IEC0bits.T3IE = 1;
			asm("ULNK");
			asm("RETFIE");
		}
		else if(tcb[task_current].restore_type == SHORT_RESTORE)
		{
			//SRbits.IPL = 0;
			asm("POP W14");
			asm("POP W13");
			asm("POP W12");
			asm("POP W11");
			asm("POP W10");
			asm("POP W9");
			asm("POP W8");
			IEC0bits.T3IE = 1;
			asm("ULNK");
			asm("RETURN");
		}
	}
}


/* #################################################################################### */
void sleep(unsigned int tick)
{
	tcb[task_current].state = STATE_DELAYED;
	tcb[task_current].ticks = tick;
	// push registers, set state to delayed, store timeout, call scheduler, pop registers, 
	// return to new function (separate unrun or ready processing)
	yield();
}


/* #################################################################################### */
// return if avail (separate unrun or ready processing), else yield to scheduler
void wait(void* p)
{
	IEC0bits.T3IE = 0;
	s = p;
	if(s->count > 0)
	{
		s->count--;
	}
	else
	{	
		s->process_queue[s->queue_size++]=task_current;
		tcb[task_current].state = STATE_BLOCKED;
		tcb[task_current].waitOnSemaphore_add = s;
		yield();
	}
	IEC0bits.T3IE = 1;
}


/* #################################################################################### */
void signal(void* p)
{
	IEC0bits.T3IE = 0;
	s = p;

	s->count++;

	if(s->queue_size > 0)
	{
		s->count--;
		tcb[s->process_queue[0]].state = STATE_READY;
		tcb[s->process_queue[0]].waitOnSemaphore_add = 0;   // Clear the waiting address
		s->queue_size--;
		int i;
		for(i=0; i<s->queue_size; i++)
			s->process_queue[i] = s->process_queue[i+1];
	}
	IEC0bits.T3IE = 1;
}

//-----------------------------------------------------------------------------
// Subroutines                
//-----------------------------------------------------------------------------

// Initialize Hardware
void init_hw()
{
  PLLFBDbits.PLLDIV = 38;                    	// pll feedback divider = 40;
  CLKDIVbits.PLLPRE = 0;                     	// pll pre divider = 2
  CLKDIVbits.PLLPOST = 0;                    	// pll post divider = 2


  AD1PCFGLbits.PCFG4 = 1;                    // make selected pins digital
  AD1PCFGLbits.PCFG5 = 1;
  LATBbits.LATB2 = 0;                        // write 0 into output latches
  LATBbits.LATB3 = 0;
  LATBbits.LATB4 = 0;
  LATBbits.LATB5 = 0;
  TRISBbits.TRISB2 = 0;                      // led pins outputs
  TRISBbits.TRISB3 = 0;
  TRISBbits.TRISB4 = 0;
  TRISBbits.TRISB5 = 0;
  CNPU1bits.CN11PUE = 1;                     // enable pull-ups for push buttons
  CNPU1bits.CN12PUE = 1;
  CNPU1bits.CN13PUE = 1;
  CNPU1bits.CN14PUE = 1;
}


/* #################################################################################### */
void timer3_init(unsigned int period)
{
  TMR3 = 0;				// Clear counter
  // Clock timer 3 with internal 40/64 MHz clock 
  T3CONbits.TCS = 0;	// Select Fcy
  T3CONbits.TCKPS = 2;	// Prescaler 64
  //T3CONbits.TON = 1;	// Timer on. Done in RTOS_START 
  PR3 = period;
  IFS0bits.T3IF = 0;	// Clear the flag
  //IEC0bits.T3IE = 1;	// Enable timer 3 interrupts  [this is done in RTOS_START function] 
}


/* #################################################################################### */
int read_pbs()
{
  return (~PORTB >> 12);
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose

void idle()
{
  while(TRUE) 
  { 
    PIN_ORANGE = 1;  
    __delay_ms(1);
    PIN_ORANGE = 0;
    yield();
  }
}


/* #################################################################################### */
void flash_4hz()
{
  while(TRUE)
  {
    PIN_GREEN ^= 1;
    sleep(125);
  }
}


/* #################################################################################### */
void one_shot()
{
  while(TRUE)
  {
    wait(&flash_req);
    PIN_YELLOW = 1;
    sleep(1000);
    PIN_YELLOW = 0;
  }
}


/* #################################################################################### */
void part_of_lengthy_fn()
{
  // represent some lengthy operation
  __delay_ms(1);
  // give another process a chance
  yield();
}

void lengthy_fn()
{
  long i;
  while(TRUE)
  {
    for (i = 0; i < 4000; i++)
    {
      part_of_lengthy_fn();
    }
    PIN_RED ^= 1;
  }
}


/* #################################################################################### */
void read_keys()
{
  int buttons;
  while(TRUE)
  {
    wait(&key_released);
    buttons = 0;
    while (buttons == 0)
    {
      buttons = read_pbs();
      yield();
    }
    signal(&key_pressed);
    if ((buttons & 1) != 0)
    {
      PIN_YELLOW ^= 1;
      PIN_RED = 1;
    }
    if ((buttons & 2) != 0)
    {
      signal(&flash_req);
      PIN_RED = 0;
    }
    if ((buttons & 4) != 0)
    {
      create_process(flash_4hz, 7);
	  //create_process(one_shot, 4);
    }
    if ((buttons & 8) != 0)
    {
      destroy_process(flash_4hz);
	  //destroy_process(one_shot);
	}

    yield();
  }
}


/* #################################################################################### */
void debounce()
{
  int count;
  while(TRUE)
  {
    wait(&key_pressed);
    count = 10;
    while (count != 0)
    {  
      sleep(10);
      if (read_pbs() == 0)
        count--;
      else
        count = 10;
    }
    signal(&key_released);
  }
}


/* #################################################################################### */
void uncooperative()
{
  while(TRUE)
  {
    while (read_pbs() == 8)
    {
    }
    yield();
  }
}

//-----------------------------------------------------------------------------
// Main                
//-----------------------------------------------------------------------------

int main(void)
{
  	int ok;
  	int pb;

  	
  	init_hw(); 					// initialize hardware
	timer3_init(625);			// initialize Timer 3 // (625*64)/40M = 1ms 
	

  	// power-up flash
  	PIN_RED = 1;
  	__delay32(10000000);
  	PIN_RED = 0;
  	__delay32(10000000);

  	// init semaphores
  	init_semaphore(&key_pressed, 0);
  	init_semaphore(&key_released, 1);
  	init_semaphore(&flash_req, 5);

  // initialize selected RTOS
  ok = FALSE;
  while (!ok)
  {
    pb = read_pbs();
    if (pb & 4) 
    {
      	ok = TRUE;
      	rtos_init(MODE_COOPERATIVE);
		while(read_pbs() & 4);			// wait till button release
    }
    if (pb & 8) 
    {
      	ok = TRUE;
      	rtos_init(MODE_PREEMPTIVE);
		while(read_pbs() & 8)			// wait till button release
		{
			__delay32(400000);
		}
    }
  }

  // add required idle process
  ok =  create_process(idle, 0) > 0;

  // add other processes
  ok &= create_process(flash_4hz, 7) > 0;
  ok &= create_process(lengthy_fn, 1) > 0;
  ok &= create_process(one_shot, 4) > 0;
  ok &= create_process(read_keys, 6) > 0;
  ok &= create_process(debounce, 4) > 0;
  ok &= create_process(uncooperative, 2) > 0;

  // start up rtos
  if (ok) 
    rtos_start(); // never returns
  else
    PIN_RED = 1;

  return 0;
  // don't delete this unreachable code
  // if a function is only called once in your code, it will be
  // accessed with two goto instructions instead of call-return,
  // so any stack-based code will not function correctly
  yield(); sleep(0); wait(0); signal(0);
}
