/***
 *      _        _    ____    _____ 
 *     | |      / \  | __ )  |___ / 
 *     | |     / _ \ |  _ \    |_ \ 
 *     | |___ / ___ \| |_) |  ___) |
 *     |_____/_/   \_\____/  |____/ 
 *                                  
 */

#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_emc.h"
#include "pin_mux.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <cmath>

#include "eGFX.h"
#include "eGFX_Driver.h"
#include "FONT_5_7_1BPP.h"
#include "OCR_A_Extended__20px__Bold__SingleBitPerPixelGridFit_1BPP.h"
#include "pin_mux.h"
#include "fsl_device_registers.h"
#include "fsl_i2c.h"
#include "fsl_i2s.h"
#include "fsl_wm8904.h"
#include "Audio.h"

#include "Ravie__26px__Regular__SystemDefault_1BPP.h" 

#include "Sprites_16BPP_565.h"
/*
	  arm_math.h has all of the prototypes for the CMSIS DSP functions
*/

#include "arm_math.h"
/*



*/
/***
 *      ____ ___ _   _  ____             ____   ___  _   _  ____               
 *     |  _ \_ _| \ | |/ ___|           |  _ \ / _ \| \ | |/ ___|              
 *     | |_) | ||  \| | |  _    _____   | |_) | | | |  \| | |  _               
 *     |  __/| || |\  | |_| |  |_____|  |  __/| |_| | |\  | |_| |              
 *     |_|__|___|_| \_|\____|__ _____ __|_|    \___/|_|_\_|\____|_ _   _ ____  
 *     | __ )| | | |  ___|  ___| ____|  _ \     / ___|| ____|_   _| | | |  _ \ 
 *     |  _ \| | | | |_  | |_  |  _| | |_) |    \___ \|  _|   | | | | | | |_) |
 *     | |_) | |_| |  _| |  _| | |___|  _ <      ___) | |___  | | | |_| |  __/ 
 *     |____/ \___/|_|   |_|   |_____|_| \_\    |____/|_____| |_|  \___/|_|    
 *                                                                             
 */
 /*
 *   This example sets up a ping pong buffer to acquire
 *   data from the DMIC and display it to the screen
 *   This is not the neccessarily the most efficient implementation
 *   but was written to show all of the operations
 */

//Let's Start with 1k Buffers
#define  BUFFER_SIZE  									 4096

volatile float * LeftBuffer;						//This is a pointer to Data we can use
volatile float * BackgroundLeftBuffer; //This is a pointer to a buffer that the DMIC IRQ routine is filling up

uint32_t BackgroundBufferIdx = 0;     //This variable is what we use to track where we are in the background buffer

volatile float LeftBuffer1[BUFFER_SIZE];  //We need 2 buffers in memory to work with.  One to store data an one to have for working on in the main loop
volatile float LeftBuffer2[BUFFER_SIZE];
volatile uint32_t NextBufferReady = 0;	//This is a flag  that the DMIC IRQ routine will use to tell us when a buffer of data is ready.


	uint32_t BuffersCaptured = 0;  /*We are using this to keep track of how many data buffers we have captured for display*/




volatile uint32_t NextSampleOut = 0;



/*
	This union will be used to split 32-bit fifo data into 2 int16_t samples.
	Unions are uses to overlay several variable across the same memory.
*/
typedef union 
{
	uint32_t Data;
	int16_t Channel[2];
	
}I2S_FIFO_Data_t;




/***
 *      ___ ____  ____    _______  __  ___       _                             _   
 *     |_ _|___ \/ ___|  |_   _\ \/ / |_ _|_ __ | |_ ___ _ __ _ __ _   _ _ __ | |_ 
 *      | |  __) \___ \    | |  \  /   | || '_ \| __/ _ \ '__| '__| | | | '_ \| __|
 *      | | / __/ ___) |   | |  /  \   | || | | | ||  __/ |  | |  | |_| | |_) | |_ 
 *     |___|_____|____/    |_| /_/\_\ |___|_| |_|\__\___|_|  |_|   \__,_| .__/ \__|
 *                                                                      |_|        
 */
 
void FLEXCOMM6_DriverIRQHandler(void)
{
    if (I2S0->FIFOINTSTAT & I2S_FIFOINTSTAT_TXLVL_MASK)
    {
        /*
					NextSampleOut Holds the last value from the I2S RX Interrupt.
				  It is also ready in the "packed" FIFO format
			  */
				I2S0->FIFOWR = NextSampleOut;
		
				 /* Clear TX level interrupt flag */
        I2S0->FIFOSTAT = I2S_FIFOSTAT_TXLVL(1U);
		}
}

/***
 *      ___ ____  ____    ____  __  __  ___       _                             _   
 *     |_ _|___ \/ ___|  |  _ \ \ \/ / |_ _|_ __ | |_ ___ _ __ _ __ _   _ _ __ | |_ 
 *      | |  __) \___ \  | |_) | \  /   | || '_ \| __/ _ \ '__| '__| | | | '_ \| __|
 *      | | / __/ ___) | |  _ <  /  \   | || | | | ||  __/ |  | |  | |_| | |_) | |_ 
 *     |___|_____|____/  |_| \_\/_/\_\ |___|_| |_|\__\___|_|  |_|   \__,_| .__/ \__|
 *                                                                       |_|        
 */
void FLEXCOMM7_DriverIRQHandler(void)
{
		register float LeftChannel;
	
	  I2S_FIFO_Data_t FIFO_Data; 
	
     /* Clear RX level interrupt flag */
     I2S1->FIFOSTAT = I2S_FIFOSTAT_RXLVL(1U);
	
	   /*
				Read the Recieve FIFO.   Data is packed as two samples in one 32-bit word.  We will immediately store the data
				in a variable that is used is the transmit routine to send incoming data back out.
		 */
	    FIFO_Data.Data = I2S1->FIFORD;
	    NextSampleOut = FIFO_Data.Data; //dump the data back out!
	
	  /*
			In the configuration for this lab,  2 channels of data are packed
			in one 32-bit word.  The Right Channel is in the upper 16-bits and the Left-Channel in the lower
		  Notice between we can use a "union" (I2S_FIFO_Data_t) to read the data in as 32-bit and access it as two 16-bit signed numbers.
	  */
	   
	   LeftChannel = (float)(FIFO_Data.Channel[0])/32768.0f;
	  

		/*
			Do something with the Left and Right channel here
		
		*/
		// SWAP BUFFER
		
		//We will do this by cast our sample to a float and then dividing by the max value of a signal 16-bit number (32767 or 1<<15)
		
		
		
    BackgroundLeftBuffer[BackgroundBufferIdx] =  (float)((LeftChannel))  /  ((float)(1<<15));
		
		/*We Just put a sample into the background buffer.  Increment our index and see if the buffer is filled*/
		BackgroundBufferIdx++;

		if(BackgroundBufferIdx == BUFFER_SIZE)
		{
				if(BackgroundLeftBuffer == LeftBuffer1)
						{
							BackgroundLeftBuffer = LeftBuffer2;
							LeftBuffer = LeftBuffer1;
						}
				else
						{
							BackgroundLeftBuffer = LeftBuffer1;
							LeftBuffer = LeftBuffer2;
						}

						/*Set a flag to inidicate that the buffer is ready!*/
				if(NextBufferReady == 0)
						NextBufferReady = 1;
						
						/*Reset our index that will fill up the background buffer*/
						BackgroundBufferIdx=0;
			}
		
}
//**************************************************************************************************

//--------------------------------------------------------------------------------------------------

#define HPS 3

float HPS_average;

int HPS_bin;

int hps(float* spectrum, int spectrumSize, int harmonics) 
{

int i, j, maxSearchIndex, maxBin;
maxSearchIndex = spectrumSize/harmonics;

maxBin = 1;
for (j=1; j<=maxSearchIndex; j++) {
    for (i=1; i<=harmonics; i++) { 
        spectrum[j] *= spectrum[j*i];
    }
    if (spectrum[j] > spectrum[maxBin]) {
        maxBin = j;
    }
}

// Fixing octave too high errors    
int correctMaxBin = 1;
int maxsearch = maxBin * 3 / 4;
for (i=2; i<maxsearch; i++) {
    if (spectrum[i] > spectrum[correctMaxBin]) {
        correctMaxBin = i;
    }
}
if (abs(correctMaxBin * 2 - maxBin) < 4) {
    if ((double)spectrum[correctMaxBin]/spectrum[maxBin] > 0.2) {
        maxBin = correctMaxBin;
    }
}

return maxBin;
}
//--------------------------------------------------------------------------------------------------
const char *notename[] = { "C", "C#", "D", "Eb", "E", "F", "F#", "G", "G#", "A",	"Bb", "B"};

const float notes[7][12] = {
			
			{16.35, 17.32,  18.35,	19.45,	20.60,	21.83,	23.12,	24.50,	25.96,	27.50,	29.14,	30.87},
			{32.70,	34.65,	36.71,	38.89,	41.20,	43.65,	46.25,	49.00,	51.91,	55.00,	58.27,	61.74},
			{65.41,	69.30,	73.42,	77.78,	82.41,	87.31,	92.50,	98.00,	103.8,	110.0,	116.5,	123.5},
			{130.8,	138.6,	146.8,	155.6,	164.8,	174.6,	185.0,	196.0,	207.7,	220.0,	233.1,	246.9},
			{261.6,	277.2,	293.7,	311.1,	329.6,	349.2,	370.0,	392.0,	415.3,	440.0,	466.2,	493.9},
			{523.3,	554.4,	587.3,	622.3,	659.3,	698.5,	740.0,	784.0,	830.6,	880.0,	932.3,	987.8},
			{1046.5, 1108.73, 1174.66, 1244.51, 1318.51, 1396.91, 1479.98, 1567.98, 1661.22, 1760, 1864.66, 1975.53}
			
		};


double diff;
int finder;
int noteRow;
double idealNote; 
int note(float freq)
	{
		//char letter[2] = "  ";
		// tempered scale with = 440;
		double min = 40;
								for(int r = 0; r < 7; r++)
								{
									for(int c = 0; c<12;c++)
									{
										diff = fabs(notes[r][c] - freq);
										
										if (diff < min)
										{
											min = diff;
											finder = c;
											noteRow = r;
										}							
									}
								}
		      
								return finder;
		
	}
	
	
const char *freq2notestring(float freq)
{
	int index = note(freq);
	if (index >= sizeof(notename)/sizeof(char*) )		
		return "nahhhh"; 
	else
		return notename[index];
		
}

void printthis(float freq)
{
	int number = note(freq);
	
	switch(number)
			{
								case 0:
												eGFX_printf(&eGFX_BackBuffer,
																		435,63,   //The x and y coordinate of where to draw the text.
																		&FONT_5_7_1BPP,   //Long font name!
															" C ");
								break;
								case 1:
												eGFX_printf(&eGFX_BackBuffer,
																		440,63,   //The x and y coordinate of where to draw the text.
																		&FONT_5_7_1BPP,   //Long font name!
															" C# ");
								break;
								case 2:
												eGFX_printf(&eGFX_BackBuffer,
																		435,51,   //The x and y coordinate of where to draw the text.
																		&FONT_5_7_1BPP,   //Long font name!
															" D ");
								break;
								case 3:
													eGFX_printf(&eGFX_BackBuffer,
																		435,41,   //The x and y coordinate of where to draw the text.
																		&FONT_5_7_1BPP,   //Long font name!
															" Eb ");
								break;
								case 4:
													eGFX_printf(&eGFX_BackBuffer,
																		435,41,   //The x and y coordinate of where to draw the text.
																		&FONT_5_7_1BPP,   //Long font name!
															" E ");
								break;
								case 5:
												eGFX_printf(&eGFX_BackBuffer,
																		435,103,   //The x and y coordinate of where to draw the text.
																		&FONT_5_7_1BPP,   //Long font name!
															" F ");
								break;
								case 6:
												eGFX_printf(&eGFX_BackBuffer,
																		435,103,   //The x and y coordinate of where to draw the text.
																		&FONT_5_7_1BPP,   //Long font name!
															" F# ");
								break;
								
								case 7:
												eGFX_printf(&eGFX_BackBuffer,
																		435,92,   //The x and y coordinate of where to draw the text.
																		&FONT_5_7_1BPP,   //Long font name!
															" G ");
								break;
								
								case 8:
												eGFX_printf(&eGFX_BackBuffer,
																		435,92,   //The x and y coordinate of where to draw the text.
																		&FONT_5_7_1BPP,   //Long font name!
															" G# ");
								break;
								case 9:
												eGFX_printf(&eGFX_BackBuffer,
																		435,82,   //The x and y coordinate of where to draw the text.
																		&FONT_5_7_1BPP,   //Long font name!
															" A ");
								break;
								case 10:
												eGFX_printf(&eGFX_BackBuffer,
																		435,72,   //The x and y coordinate of where to draw the text.
																		&FONT_5_7_1BPP,   //Long font name!
															" Bb ");
								break;
								case 11:
												eGFX_printf(&eGFX_BackBuffer,
																		435,72,   //The x and y coordinate of where to draw the text.
																		&FONT_5_7_1BPP,   //Long font name!
															" B ");
								break;
			
						}
	
									
}
/***************************************************************************************************
 *      _____ _____ _____  __     __         _       _     _           
 *     |  ___|  ___|_   _| \ \   / /_ _ _ __(_) __ _| |__ | | ___  ___ 
 *     | |_  | |_    | |    \ \ / / _` | '__| |/ _` | '_ \| |/ _ \/ __|
 *     |  _| |  _|   | |     \ V / (_| | |  | | (_| | |_) | |  __/\__ \
 *     |_|   |_|     |_|      \_/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/
 *                                                                     
 */

/*

	We are going use the floating point real valued FFT in the CMSIS library.
  If you unzip the CMSIS source code,  there is an "index.html" file in the CMSIS folder.
  This is the root webpage for the CMSIS library documentation.  There is a "DSP" tab at the top.

  You can also find the latest documentation here:

  http://www.keil.com/pack/doc/CMSIS/DSP/html/index.html

	The real FFT requires a configuration "instance" structure.   MyFFT is the struct
	we will use for our FFT

*/

arm_rfft_fast_instance_f32 MyFFT;

/*
		We are going to allocate arrays to store the output of the FFT and the power spectrum.  The
		raw output are the real/imaginary components of the output.   The "power spectrum" array is to store
		the magnitude squared values of the FFT output.
*/

float	 FFT_RawDataOut[BUFFER_SIZE];

float  FFT_PowerSpectrum[BUFFER_SIZE];


//float average[BUFFER_SIZE/2];	



float FFT_PS[BUFFER_SIZE];

float FFT_PS2[BUFFER_SIZE];





//***************************************************************************************************



void InitLeftBuffers()
{
	for (int i=0;i<BUFFER_SIZE;i++)
	{
		LeftBuffer1[i] = 0;
		LeftBuffer2[i] = 0;
	}

	LeftBuffer = LeftBuffer1;

	BackgroundLeftBuffer = LeftBuffer2;

	NextBufferReady = 0;
}
float idealNote_bin;
	
int main(void)
{
	float Point1 = 0;

    CLOCK_EnableClock(kCLOCK_InputMux);
		
    CLOCK_EnableClock(kCLOCK_Iocon);
	
    CLOCK_EnableClock(kCLOCK_Gpio0);
  
    CLOCK_EnableClock(kCLOCK_Gpio1);

  	/* USART0 clock */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* Initialize the rest */
    BOARD_InitPins();
	
    BOARD_BootClockRUN();
  
    BOARD_InitDebugConsole();

		BOARD_InitSDRAM();

		eGFX_InitDriver();
		
		InitLeftBuffers();

      /*
				This function initializes the WM8904 CODEC and two I2S ports to send and recieve audio.
				Read through the comments in the function. (See Audio.c)
			*/

   	InitAudio_CODEC();
				
		arm_rfft_fast_init_f32(&MyFFT,BUFFER_SIZE);	
		
		while(1)
		{
			
			float peak = 0;
			int bin = 0;
			float freq = 	1;
			
			while(NextBufferReady == 0)
			{
			}
			NextBufferReady = 0;
			
		
			
				  arm_rfft_fast_f32(&MyFFT,
													  (float *)LeftBuffer,
													  FFT_RawDataOut,0
														);
														
				 arm_cmplx_mag_squared_f32(FFT_RawDataOut,
																	 FFT_PowerSpectrum,
																	 BUFFER_SIZE
																	);
														
				arm_cmplx_mag_squared_f32(FFT_RawDataOut,
																	 FFT_PS,
																	 BUFFER_SIZE
																	);
																							
		
				 eGFX_ImagePlane_Clear(&eGFX_BackBuffer);
		
	//--------------------------------------------------------------------------
														
			for(int x = 0; x < BUFFER_SIZE;x++)
			{
					
						if (peak < FFT_PowerSpectrum[x])
										{
											peak = FFT_PowerSpectrum[x];			
											bin = x;
											freq = bin *(8000)/(BUFFER_SIZE);
			
										}		
			}
	//----------------------------------------------------------------------------
			
		
			
				idealNote = notes[noteRow][finder];
			
				idealNote_bin = (int)(idealNote*BUFFER_SIZE/8000);
				
				HPS_average =   8000 * ((double) hps(FFT_PS, BUFFER_SIZE, HPS))/(BUFFER_SIZE);
				
				HPS_bin = (int)(HPS_average*BUFFER_SIZE/8000);
						
						
				if (HPS_average >= freq)
				{
					HPS_average = freq;
				}
			
	//----------------------------------------------------------------------------
						for(int i=0;i<eGFX_PHYSICAL_SCREEN_SIZE_X;i++)
								{
								
								Point1 = (1000000000 * FFT_PowerSpectrum[i]/12);
							 
									
											if((i == idealNote_bin)&(i == HPS_bin))
											{
											eGFX_DrawVline(&eGFX_BackBuffer,
																(eGFX_PHYSICAL_SCREEN_SIZE_Y-1)/1.2,
																0,
																i,
																eGFX_RGB888_TO_RGB565(255,0,0));	
												
																eGFX_Blit(&eGFX_BackBuffer,
																i,  //x coordinate of where to put the monkey head
																75,																			 //y coordinate of where to put the pic
																&Sprite_16BPP_565_fonz);
												
												}
											else if (i == HPS_bin) 
												{
												eGFX_DrawVline(&eGFX_BackBuffer,
																(eGFX_PHYSICAL_SCREEN_SIZE_Y-1)/1.2,
																0,
																i,
																eGFX_RGB888_TO_RGB565(0,0,255));	
													eGFX_Blit(&eGFX_BackBuffer,
																i,  //x coordinate of where to put the monkey head
																75,																			 //y coordinate of where to put the pic
																&Sprite_16BPP_565_troll);
													
						
												}
											else if (i == idealNote_bin)
											{
													eGFX_DrawVline(&eGFX_BackBuffer,
																(eGFX_PHYSICAL_SCREEN_SIZE_Y-1)/1.2,
																0,
																i,
																eGFX_RGB888_TO_RGB565(255,0,255));	
												
												
											}
											else
											{
											
												eGFX_DrawVline(&eGFX_BackBuffer,
																(eGFX_PHYSICAL_SCREEN_SIZE_Y-1)/1.2,
																(eGFX_PHYSICAL_SCREEN_SIZE_Y-1)/1.2 - Point1,
																i,
																eGFX_RGB888_TO_RGB565(0,255,255));								
											}
							}
	
		
		eGFX_printf_Colored(&eGFX_BackBuffer,
										50,238,   //The x and y coordinate of where to draw the text.
											&OCR_A_Extended__20px__Bold__SingleBitPerPixelGridFit_1BPP,   //Long font name!
											eGFX_RGB888_TO_RGB565(0,0,255),
							"Frequency: %.1f Hz", HPS_average);	
								
		eGFX_printf_Colored(&eGFX_BackBuffer,
										250,238,   //The x and y coordinate of where to draw the text.
											&OCR_A_Extended__20px__Bold__SingleBitPerPixelGridFit_1BPP,   //Long font name!
											eGFX_RGB888_TO_RGB565(255,0,255),
							"Ideal Note: %.1f Hz", idealNote);	
							
		eGFX_printf(&eGFX_BackBuffer,
										390,160,   //The x and y coordinate of where to draw the text.
											&OCR_A_Extended__20px__Bold__SingleBitPerPixelGridFit_1BPP,   //Long font name!
							       "Pitch: %s", freq2notestring(HPS_average) );	
				
								
								
								
					  eGFX_printf_Colored(&eGFX_BackBuffer,
											5, 
											5,
											&Ravie__26px__Regular__SystemDefault_1BPP,   
											eGFX_RGB888_TO_RGB565(213,15,37),
											"FREQ'Y TUNER");
		
						eGFX_Blit(&eGFX_BackBuffer,
								360,  //x coordinate of where to put the monkey head
								10,																			 //y coordinate of where to put the pic
								&Sprite_16BPP_565_staffs);	
		
						printthis(HPS_average);
		
		
		
								
				eGFX_Dump(&eGFX_BackBuffer);
				
			
		}
}

