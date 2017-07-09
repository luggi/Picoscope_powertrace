/*******************************************************************************
 *
 * Filename: ps3000acon.c
 *
 * Description:
 *   This is a console mode program that demonstrates how to use the
 *   PicoScope 3000 Series (ps3000a) driver functions.
 *
 *	Supported PicoScope models:
 *
 *		PicoScope 3204A/B/D
 *		PicoScope 3205A/B/D
 *		PicoScope 3206A/B/D
 *		PicoScope 3207A/B
 *		PicoScope 3204 MSO & D MSO
 *		PicoScope 3205 MSO & D MSO
 *		PicoScope 3206 MSO & D MSO
 *		PicoScope 3404A/B/D/D MSO
 *		PicoScope 3405A/B/D/D MSO
 *		PicoScope 3406A/B/D/D MSO
 *
 * Examples:
 *    Collect a block of samples immediately
 *    Collect a block of samples when a trigger event occurs
 *	  Collect a block of samples using Equivalent Time Sampling (ETS)
 *    Collect samples using a rapid block capture with trigger
 *    Collect a stream of data immediately
 *    Collect a stream of data when a trigger event occurs
 *    Set Signal Generator, using standard or custom signals
 *    Change timebase & voltage scales
 *    Display data in mV or ADC counts
 *    Handle power source changes (PicoScope 34XXA/B, 32XX D/D MSO, & 
 *		34XX D/D MSO devices only)
 *
 * Digital Examples (MSO variants only):  
 *    Collect a block of digital samples immediately
 *    Collect a block of digital samples when a trigger event occurs
 *    Collect a block of analogue & digital samples when analogue AND digital trigger events occurs
 *    Collect a block of analogue & digital samples when analogue OR digital trigger events occurs
 *    Collect a stream of digital data immediately
 *    Collect a stream of digital data and show aggregated values
 *
 *
 *	To build this application:
 *
 *		If Microsoft Visual Studio (including Express) is being used:
 *
 *			Select the solution configuration (Debug/Release) and platform (x86/x64)
 *			Ensure that the 32-/64-bit ps3000a.lib can be located
 *			Ensure that the ps3000aApi.h and PicoStatus.h files can be located
 *		
 *		Otherwise:
 *
 *			 Set up a project for a 32-/64-bit console mode application
 *			 Add this file to the project
 *			 Add 32-/64-bit ps3000a.lib to the project
 *			 Add ps3000aApi.h and PicoStatus.h to the project
 *			 Build the project
 *
 *  Linux platforms:
 *
 *		Ensure that the libps3000a driver package has been installed using the
 *		instructions from https://www.picotech.com/downloads/linux
 *
 *		Place this file in the same folder as the files from the linux-build-files
 *		folder. In a terminal window, use the following commands to build
 *		the ps3000acon application:
 *
 *			./autogen.sh <ENTER>
 *			make <ENTER>
 *
 * Copyright (C) 2011 - 2017 Pico Technology Ltd. See LICENSE file for terms.
 *
 ******************************************************************************/

#include <stdio.h>

/* Headers for Windows */
#ifdef _WIN32
#include "windows.h"
#include <conio.h>
#include "ps3000aApi.h"
#else
#include <sys/types.h>
#include <string.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>

#include <libps3000a-1.1/ps3000aApi.h>
#ifndef PICO_STATUS
#include <libps3000a-1.1/PicoStatus.h>
#endif

#include <fcntl.h>
#include <sys/signal.h>
#include <sys/time.h>
#include <stdint.h>
#include <time.h>

#define Sleep(a) usleep(1000*a)
#define scanf_s scanf
#define fscanf_s fscanf
#define memcpy_s(a,b,c,d) memcpy(a,c,d)

typedef enum enBOOL{FALSE,TRUE} BOOL;
int serial_port = 0;


/* opens a com-port and returns the file pointer */
int open_port (char * PORT, int speed)
{
    struct termios tio;
    struct sigaction saio;           /* definition of signal action */
    int fd; // Filedeskriptor
    fd = open (PORT, O_RDWR | O_NOCTTY ); // O_NONBLOCK

    if (fd < 0) // negative Zahl bei fehler!
    {
        fprintf(stderr, "Open_port: unable to open %s\n", PORT);
        exit (1);   // irreparebel, verlasse Programm
    }

    memset( &tio, 0, sizeof(tio) );
    tio.c_cflag = CS8|CREAD|CLOCAL;     /* 8n1 */
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if(speed == 9600) {
        cfsetospeed( &tio, B9600 );
        cfsetispeed( &tio, B9600 );
    } else if (speed == 19200) {
        cfsetospeed( &tio, B19200 );
        cfsetispeed( &tio, B19200 );
    } else if (speed == 115200) {
        cfsetospeed( &tio, B115200 );
        cfsetispeed( &tio, B115200 );
    } else {
        printf("Baudrate not supported!\n");
    }
    cfmakeraw(&tio);
    tcflush( fd, TCIFLUSH );
    tcsetattr( fd, TCSANOW, &tio );  

    return fd;
}




/* A function to detect a keyboard press on Linux */
int32_t _getch()
{
	struct termios oldt, newt;
	int32_t ch;
	int32_t bytesWaiting;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	setbuf(stdin, NULL);
	do {
		ioctl(STDIN_FILENO, FIONREAD, &bytesWaiting);
		if (bytesWaiting)
			getchar();
	} while (bytesWaiting);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
}

int32_t _kbhit()
{
	struct termios oldt, newt;
	int32_t bytesWaiting;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	setbuf(stdin, NULL);
	ioctl(STDIN_FILENO, FIONREAD, &bytesWaiting);

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return bytesWaiting;
}

int32_t fopen_s(FILE ** a, const char * b, const char * c)
{
	FILE * fp = fopen(b,c);
	*a = fp;
	return (fp>0)?0:-1;
}

/* A function to get a single character on Linux */
#define max(a,b) ((a) > (b) ? a : b)
#define min(a,b) ((a) < (b) ? a : b)
#endif

#define PREF4 __stdcall

int32_t cycles = 0;

#define TRACES_TO_SKIP  50
#define BUFFER_SIZE 	200000
#define TRACE_COUNT     3000

#define QUAD_SCOPE		4
#define DUAL_SCOPE		2

// AWG Parameters

#define AWG_DAC_FREQUENCY			20e6		
#define AWG_DAC_FREQUENCY_PS3207B	100e6
#define	AWG_PHASE_ACCUMULATOR		4294967296.0

typedef enum
{
	ANALOGUE,
	DIGITAL,
	AGGREGATED,
	MIXED
}MODE;


typedef struct
{
	int16_t DCcoupled;
	int16_t range;
	int16_t enabled;
}CHANNEL_SETTINGS;

typedef enum
{
	SIGGEN_NONE = 0,
	SIGGEN_FUNCTGEN = 1,
	SIGGEN_AWG = 2
} SIGGEN_TYPE;

typedef struct tTriggerDirections
{
	PS3000A_THRESHOLD_DIRECTION channelA;
	PS3000A_THRESHOLD_DIRECTION channelB;
	PS3000A_THRESHOLD_DIRECTION channelC;
	PS3000A_THRESHOLD_DIRECTION channelD;
	PS3000A_THRESHOLD_DIRECTION ext;
	PS3000A_THRESHOLD_DIRECTION aux;
}TRIGGER_DIRECTIONS;

typedef struct tPwq
{
	PS3000A_PWQ_CONDITIONS_V2 * conditions;
	int16_t nConditions;
	PS3000A_THRESHOLD_DIRECTION direction;
	uint32_t lower;
	uint32_t upper;
	PS3000A_PULSE_WIDTH_TYPE type;
}PWQ;

typedef struct
{
	int16_t					handle;
	int8_t					model[8];
	PS3000A_RANGE			firstRange ;
	PS3000A_RANGE			lastRange;
	int16_t					channelCount;
	int16_t					maxValue;
	int16_t					sigGen;
	int16_t					ETS;
	int32_t					AWGFileSize;
	CHANNEL_SETTINGS		channelSettings [PS3000A_MAX_CHANNELS];
	int16_t					digitalPorts;
}UNIT;

uint32_t	timebase = 5;
int16_t     oversample = 1;
BOOL		scaleVoltages = TRUE;

uint16_t inputRanges [PS3000A_MAX_RANGES] = {10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000};

BOOL     	g_ready = FALSE;
int32_t 	g_times [PS3000A_MAX_CHANNELS] = {0, 0, 0, 0};
int16_t     g_timeUnit;
int32_t     g_sampleCount;
uint32_t	g_startIndex;
int16_t		g_autoStopped;
int16_t		g_trig = 0;
uint32_t	g_trigAt = 0;

char BlockFile[200]		= "block.txt";
char DigiBlockFile[200]	= "digiBlock.txt";
char StreamFile[200]		= "stream.txt";
char Filename_custom[200] = "test.txt";

typedef struct tBufferInfo
{
	UNIT * unit;
	MODE mode;
	int16_t **driverBuffers;
	int16_t **appBuffers;
	int16_t **driverDigBuffers;
	int16_t **appDigBuffers;

} BUFFER_INFO;


/****************************************************************************
* Streaming callback
* Used by ps3000a data streaming collection calls, on receipt of data.
* Used to set global flags etc. checked by user routines
****************************************************************************/
void PREF4 callBackStreaming(	int16_t handle,
	int32_t		noOfSamples,
	uint32_t	startIndex,
	int16_t		overflow,
	uint32_t	triggerAt,
	int16_t		triggered,
	int16_t		autoStop,
	void		*pParameter)
{
	int32_t channel;
	BUFFER_INFO * bufferInfo = NULL;

	if (pParameter != NULL)
	{
		bufferInfo = (BUFFER_INFO *) pParameter;
	}

	// used for streaming
	g_sampleCount	= noOfSamples;
	g_startIndex	= startIndex;
	g_autoStopped	= autoStop;

	// flag to say done reading data
	g_ready = TRUE;

	// flags to show if & where a trigger has occurred
	g_trig = triggered;
	g_trigAt = triggerAt;

	if (bufferInfo != NULL && noOfSamples)
	{
		if (bufferInfo->mode == ANALOGUE)
		{
			for (channel = 0; channel < bufferInfo->unit->channelCount; channel++)
			{
				if (bufferInfo->unit->channelSettings[channel].enabled)
				{
					if (bufferInfo->appBuffers && bufferInfo->driverBuffers)
					{
						if (bufferInfo->appBuffers[channel * 2]  && bufferInfo->driverBuffers[channel * 2])
						{
							memcpy_s (&bufferInfo->appBuffers[channel * 2][startIndex], noOfSamples * sizeof(int16_t),
								&bufferInfo->driverBuffers[channel * 2][startIndex], noOfSamples * sizeof(int16_t));
						}
						if (bufferInfo->appBuffers[channel * 2 + 1] && bufferInfo->driverBuffers[channel * 2 + 1])
						{
							memcpy_s (&bufferInfo->appBuffers[channel * 2 + 1][startIndex], noOfSamples * sizeof(int16_t),
								&bufferInfo->driverBuffers[channel * 2 + 1][startIndex], noOfSamples * sizeof(int16_t));
						}
					}
				}
			}
		}
		else if (bufferInfo->mode == AGGREGATED)
		{
			for (channel = 0; channel < bufferInfo->unit->digitalPorts; channel++)
			{
				if (bufferInfo->appDigBuffers && bufferInfo->driverDigBuffers)
				{
					if (bufferInfo->appDigBuffers[channel * 2] && bufferInfo->driverDigBuffers[channel * 2])
					{
						memcpy_s (&bufferInfo->appDigBuffers[channel * 2][startIndex], noOfSamples * sizeof(int16_t),
							&bufferInfo->driverDigBuffers[channel * 2][startIndex], noOfSamples * sizeof(int16_t));
					}
					if (bufferInfo->appDigBuffers[channel * 2 + 1] && bufferInfo->driverDigBuffers[channel * 2 + 1])
					{
						memcpy_s (&bufferInfo->appDigBuffers[channel * 2 + 1][startIndex], noOfSamples * sizeof(int16_t),
							&bufferInfo->driverDigBuffers[channel * 2 + 1][startIndex], noOfSamples * sizeof(int16_t));
					}
				}
			}
		}
		else if (bufferInfo->mode == DIGITAL)
		{
			for (channel = 0; channel < bufferInfo->unit->digitalPorts; channel++)
			{
				if (bufferInfo->appDigBuffers && bufferInfo->driverDigBuffers)
				{
					if (bufferInfo->appDigBuffers[channel] && bufferInfo->driverDigBuffers[channel])
					{
						memcpy_s (&bufferInfo->appDigBuffers[channel][startIndex], noOfSamples * sizeof(int16_t),
							&bufferInfo->driverDigBuffers[channel][startIndex], noOfSamples * sizeof(int16_t));
					}
				}
			}
		}
	}
}

/****************************************************************************
* Block Callback
* used by ps3000a data block collection calls, on receipt of data.
* used to set global flags etc checked by user routines
****************************************************************************/
void PREF4 callBackBlock( int16_t handle, PICO_STATUS status, void * pParameter)
{
	if (status != PICO_CANCELLED)
	{
		g_ready = TRUE;
	}
}

/****************************************************************************
* setDefaults - restore default settings
****************************************************************************/
void setDefaults(UNIT * unit)
{
	int32_t i;
	PICO_STATUS status;

	status = ps3000aSetEts(unit->handle, PS3000A_ETS_OFF, 0, 0, NULL);	// Turn off ETS
	printf(status?"SetDefaults:ps3000aSetEts------ 0x%08lx \n":"", status);

	for (i = 0; i < unit->channelCount; i++) // reset channels to most recent settings
	{
		status = ps3000aSetChannel(unit->handle, (PS3000A_CHANNEL)(PS3000A_CHANNEL_A + i),
			unit->channelSettings[PS3000A_CHANNEL_A + i].enabled,
			(PS3000A_COUPLING)unit->channelSettings[PS3000A_CHANNEL_A + i].DCcoupled,
			(PS3000A_RANGE)unit->channelSettings[PS3000A_CHANNEL_A + i].range, 0);

		printf(status?"SetDefaults:ps3000aSetChannel------ 0x%08lx \n":"", status);
	}
}

/****************************************************************************
* setDigitals - enable or disable Digital Channels
****************************************************************************/
PICO_STATUS setDigitals(UNIT *unit, int16_t state)
{
	PICO_STATUS status;

	int16_t logicLevel;
	float logicVoltage = 1.5;
	int16_t maxLogicVoltage = 5;

	int16_t timebase = 4;
	int16_t port;


	// Set logic threshold
	logicLevel =  (int16_t) ((logicVoltage / maxLogicVoltage) * PS3000A_MAX_LOGIC_LEVEL);

	// Enable Digital ports
	for (port = PS3000A_DIGITAL_PORT0; port <= PS3000A_DIGITAL_PORT1; port++)
	{
		status = ps3000aSetDigitalPort(unit->handle, (PS3000A_DIGITAL_PORT)port, state, logicLevel);
		printf(status?"SetDigitals:PS3000ASetDigitalPort(Port 0x%X) ------ 0x%08lx \n":"", port, status);
	}

	return status;
}

/****************************************************************************
* disableAnalogue - Disable Analogue Channels
****************************************************************************/
PICO_STATUS disableAnalogue(UNIT *unit)
{
	PICO_STATUS status;
	int16_t ch;

	// Turn off analogue channels, keeping settings
	for (ch = 0; ch < unit->channelCount; ch++)
	{
		unit->channelSettings[ch].enabled = FALSE;

		status = ps3000aSetChannel(unit->handle, (PS3000A_CHANNEL) ch, unit->channelSettings[ch].enabled, (PS3000A_COUPLING) unit->channelSettings[ch].DCcoupled, 
			(PS3000A_RANGE) unit->channelSettings[ch].range, 0);
		
		if(status != PICO_OK)
		{
			printf("disableAnalogue:ps3000aSetChannel(channel %d) ------ 0x%08lx \n", ch, status);
		}
	}
	return status;
}

/****************************************************************************
* restoreAnalogueSettings - Restores Analogue Channel settings
****************************************************************************/
PICO_STATUS restoreAnalogueSettings(UNIT *unit)
{
	PICO_STATUS status;
	int16_t ch;

	// Turn on analogue channels using previous settings
	for (ch = 0; ch < unit->channelCount; ch++)
	{
		status = ps3000aSetChannel(unit->handle, (PS3000A_CHANNEL) ch, unit->channelSettings[ch].enabled, (PS3000A_COUPLING) unit->channelSettings[ch].DCcoupled, 
			(PS3000A_RANGE) unit->channelSettings[ch].range, 0);

		if(status != PICO_OK)
		{
			printf("restoreAnalogueSettings:ps3000aSetChannel(channel %d) ------ 0x%08lx \n", ch, status);
		}
	}
	return status;
}


/****************************************************************************
* adc_to_mv
*
* Convert an 16-bit ADC count into millivolts
****************************************************************************/
int32_t adc_to_mv(int32_t raw, int32_t ch, UNIT * unit)
{
	return (raw * inputRanges[ch]) / unit->maxValue;
}

/****************************************************************************
* mv_to_adc
*
* Convert a millivolt value into a 16-bit ADC count
*
*  (useful for setting trigger thresholds)
****************************************************************************/
int16_t mv_to_adc(int16_t mv, int16_t ch, UNIT * unit)
{
	return (mv * unit->maxValue) / inputRanges[ch];
}

/****************************************************************************************
* changePowerSource - function to handle switches between +5 V supply, and USB-only power
* Only applies to PicoScope 34xxA/B/D/D MSO units 
******************************************************************************************/
PICO_STATUS changePowerSource(int16_t handle, PICO_STATUS status)
{
	char ch;

	switch (status)
	{
		case PICO_POWER_SUPPLY_NOT_CONNECTED:			// User must acknowledge they want to power via USB

			do
			{
				printf("\n5 V power supply not connected");
				printf("\nDo you want to run using USB only Y/N?\n");
				ch = toupper(_getch());
				if(ch == 'Y')
				{
					printf("\nPowering the unit via USB\n");
					status = ps3000aChangePowerSource(handle, PICO_POWER_SUPPLY_NOT_CONNECTED);		// Tell the driver that's ok

					if (status == PICO_POWER_SUPPLY_UNDERVOLTAGE)
					{
						status = changePowerSource(handle, status);
					}
				}
			}
			while(ch != 'Y' && ch != 'N');

			printf(ch == 'N'?"Please use the +5 V power supply to power this unit\n":"");
			break;

		case PICO_POWER_SUPPLY_CONNECTED:

			printf("\nUsing +5V power supply voltage\n");
			status = ps3000aChangePowerSource(handle, PICO_POWER_SUPPLY_CONNECTED);	// Tell the driver we are powered from +5 V supply
			break;

		case PICO_USB3_0_DEVICE_NON_USB3_0_PORT:

			printf("\nUSB 3.0 device on non-USB 3.0 port.\n");
			status = ps3000aChangePowerSource(handle, PICO_USB3_0_DEVICE_NON_USB3_0_PORT);	// Tell the driver that it is ok for the device to be on a non-USB 3.0 port
				
			break;

		case PICO_POWER_SUPPLY_UNDERVOLTAGE:

			do
			{
				printf("\nUSB not supplying required voltage");
				printf("\nPlease plug in the +5 V power supply\n");
				printf("\nHit any key to continue, or Esc to exit...\n");
				ch = _getch();

				if (ch == 0x1B)	// ESC key
				{
					exit(0);
				}
				else
				{
					status = ps3000aChangePowerSource(handle, PICO_POWER_SUPPLY_CONNECTED);		// Tell the driver that's ok
				}
			}
			while (status == PICO_POWER_SUPPLY_REQUEST_INVALID);
			break;

	}
	return status;
}

/****************************************************************************
* clearDataBuffers
*
* stops GetData writing values to memory that has been released
****************************************************************************/
PICO_STATUS clearDataBuffers(UNIT * unit)
{
	int32_t i;
	PICO_STATUS status;

	for (i = 0; i < unit->channelCount; i++) 
	{
		if((status = ps3000aSetDataBuffers(unit->handle, (PS3000A_CHANNEL) i, NULL, NULL, 0, 0, PS3000A_RATIO_MODE_NONE)) != PICO_OK)
		{
			printf("ClearDataBuffers:ps3000aSetDataBuffers(channel %d) ------ 0x%08lx \n", i, status);
		}
	}


	for (i= 0; i < unit->digitalPorts; i++) 
	{
		if((status = ps3000aSetDataBuffer(unit->handle, (PS3000A_CHANNEL) (i + PS3000A_DIGITAL_PORT0), NULL, 0, 0, PS3000A_RATIO_MODE_NONE))!= PICO_OK)
		{
			printf("ClearDataBuffers:ps3000aSetDataBuffer(port 0x%X) ------ 0x%08lx \n", i + PS3000A_DIGITAL_PORT0, status);
		}
	}

	return status;
}

/****************************************************************************
* blockDataHandler
* - Used by all block data routines
* - acquires data (user sets trigger mode before calling), displays 10 items
*   and saves all to block.txt
* Input :
* - unit : the unit to use.
* - text : the text to display before the display of data slice
* - offset : the offset into the data buffer to start the display's slice.
****************************************************************************/
void blockDataHandler(UNIT * unit, char * text, int32_t offset, MODE mode)
{
	int16_t retry;
	int16_t bit;

	uint16_t bitValue;
	uint16_t digiValue;
	
	int16_t * buffers[PS3000A_MAX_CHANNEL_BUFFERS];
	int16_t * digiBuffer[PS3000A_MAX_DIGITAL_PORTS];

	int32_t i, j;
	int32_t timeInterval;
	int32_t sampleCount = BUFFER_SIZE;
	int32_t maxSamples;
	int32_t timeIndisposed;
    static uint32_t blocks_written=0;
    static char plaintext[18] = {0,0,0,0, 
                                 0,0,0,0,
                                 0,0,0,0,
                                 0,0,0,0,'\n', 0};
    
	FILE * fp = NULL;
    FILE * plaintext_fp = NULL;
	FILE * digiFp = NULL;
	
	PICO_STATUS status;
	PS3000A_RATIO_MODE ratioMode = PS3000A_RATIO_MODE_NONE;
    
    

	if (mode == ANALOGUE || mode == MIXED)		// Analogue or (MSO Only) MIXED 
	{
		for (i = 0; i < unit->channelCount; i++) 
		{
			if(unit->channelSettings[i].enabled)
			{
				buffers[i * 2] = (int16_t*) calloc(sampleCount, sizeof(int16_t));
				buffers[i * 2 + 1] = (int16_t*) calloc(sampleCount, sizeof(int16_t));

				status = ps3000aSetDataBuffers(unit->handle, (PS3000A_CHANNEL)i, buffers[i * 2], buffers[i * 2 + 1], sampleCount, 0, ratioMode);

				printf(status?"BlockDataHandler:ps3000aSetDataBuffers(channel %d) ------ 0x%08lx \n":"", i, status);
			}
		}
	}

	if (mode == DIGITAL || mode == MIXED)		// (MSO Only) Digital or MIXED
	{
		for (i= 0; i < unit->digitalPorts; i++) 
		{
			digiBuffer[i] = (int16_t*) calloc(sampleCount, sizeof(int16_t));

			status = ps3000aSetDataBuffer(unit->handle, (PS3000A_CHANNEL) (i + PS3000A_DIGITAL_PORT0), digiBuffer[i], sampleCount, 0, ratioMode);

			printf(status?"BlockDataHandler:ps3000aSetDataBuffer(port 0x%X) ------ 0x%08lx \n":"", i + PS3000A_DIGITAL_PORT0, status);
		}
	}

	/* Find the maximum number of samples and the time interval (in nanoseconds) */
	while (ps3000aGetTimebase(unit->handle, timebase, sampleCount, &timeInterval, oversample, &maxSamples, 0))
	{
		timebase++;
	}

	printf("\nTimebase: %lu  Sample interval: %ld ns \n", timebase, timeInterval);

	/* Start the device collecting, then wait for completion*/
	g_ready = FALSE;


	do
	{
		retry = 0;

		status = ps3000aRunBlock(unit->handle, 0, sampleCount, timebase, oversample, &timeIndisposed, 0, callBackBlock, NULL);

		if (status != PICO_OK)
		{
			if (status == PICO_POWER_SUPPLY_CONNECTED || status == PICO_POWER_SUPPLY_NOT_CONNECTED || 
				status == PICO_POWER_SUPPLY_UNDERVOLTAGE)       // PicoScope 340XA/B/D/D MSO devices...+5 V PSU connected or removed
			{
				status = changePowerSource(unit->handle, status);
				retry = 1;
			}
			else
			{
				printf("BlockDataHandler:ps3000aRunBlock ------ 0x%08lx \n", status);
				return;
			}
		}
	}
	while (retry);

	printf("Waiting for trigger...\n");
    uint8_t response[16];
    char plaintextfile[100];
    
    blocks_written++;
    sprintf(plaintextfile, "%s_%dsamples_%dtraces_%dns_Plaintext.txt",Filename_custom,BUFFER_SIZE, TRACE_COUNT, timeInterval); 
    fopen_s(&plaintext_fp, plaintextfile, "a");
    
    if(plaintext_fp != NULL)
    {
        
        for(int i=0; i<16; i++)
        {
            plaintext[i] = ' ' + rand() % (127-' '); // generate random ascii characters
        }
        
        write(serial_port, plaintext, 17);
        usleep(30000);
        read(serial_port, response, 16);
        
        for(int i=0;i<16;i++) {
            printf("%d %d\n", plaintext[i], response[i]);
        }

        if(!memcmp(plaintext, response, 16))
            printf("------------------------data valid-----------------------------\n");
        if(blocks_written > TRACES_TO_SKIP)
            fprintf(plaintext_fp, "%s", plaintext);
    }

	while (!g_ready) // while (!g_ready && !_kbhit()) including keypress wait
	{
		Sleep(0);
	}

	if (g_ready) 
	{
		status = ps3000aGetValues(unit->handle, 0, (uint32_t*) &sampleCount, 1, ratioMode, 0, NULL);

		if (status != PICO_OK)
		{
			if (status == PICO_POWER_SUPPLY_CONNECTED || status == PICO_POWER_SUPPLY_NOT_CONNECTED || status == PICO_POWER_SUPPLY_UNDERVOLTAGE)
			{
				if (status == PICO_POWER_SUPPLY_UNDERVOLTAGE)
				{
					changePowerSource(unit->handle, status);
				}
				else
				{
					printf("\nPower Source Changed. Data collection aborted.\n");
				}
			}
			else
			{
				printf("BlockDataHandler:ps3000aGetValues ------ 0x%08lx \n", status);
			}
		}
		else
		{

			if (mode == ANALOGUE || mode == MIXED)		// If we're doing analogue or MIXED
			{
				sampleCount = min(sampleCount, BUFFER_SIZE);

                sprintf(BlockFile, "%s_%dsamples_%dtraces_%dns.txt",Filename_custom,BUFFER_SIZE, TRACE_COUNT, timeInterval); 
				fopen_s(&fp, BlockFile, "a");
                
                
				if (fp != NULL && blocks_written > TRACES_TO_SKIP) // skip the first few traces to account for the offset change
				{
                    printf("Writing Block %d\n", blocks_written);

					for (i = 0; i < sampleCount; i++) 
					{
						if (unit->channelSettings[0].enabled) // Print channel A to file (powertrace)
							{
								fprintf(	fp,
									"%d",
									buffers[0][i]);
							}
						fprintf(fp, ",");
					}
					fprintf(fp, "\n");
				}
				else
				{
					printf(	"Cannot open the file %s for writing.\n"
						"Please ensure that you have permission to access.\n", BlockFile);
				}
			}

		} 
	}
	else 
	{
		printf("\nData collection aborted.\n");
		_getch();
	}

	if ((status = ps3000aStop(unit->handle)) != PICO_OK)
	{
		printf("BlockDataHandler:ps3000aStop ------ 0x%08lx \n", status);
	}

	if (fp != NULL)
	{
		fclose(fp);
	}

	if (digiFp != NULL)
	{
		fclose(digiFp);
	}
	
	if (plaintext_fp != NULL)
        fclose(plaintext_fp);
        

	if (mode == ANALOGUE || mode == MIXED)		// Only if we allocated these buffers
	{
		for (i = 0; i < unit->channelCount; i++) 
		{
			if(unit->channelSettings[i].enabled)
			{
				free(buffers[i * 2]);
				free(buffers[i * 2 + 1]);

			}
		}
	}

	if (mode == DIGITAL || mode == MIXED)		// Only if we allocated these buffers
	{
		for (i = 0; i < unit->digitalPorts; i++) 
		{
			free(digiBuffer[i]);
		}
	}
	
	clearDataBuffers(unit);
}




/****************************************************************************
* setTrigger
*
* - Used to call aall the functions required to set up triggering
*
***************************************************************************/
PICO_STATUS setTrigger(	UNIT * unit,

						struct tPS3000ATriggerChannelProperties * channelProperties,
							int16_t nChannelProperties,
							PS3000A_TRIGGER_CONDITIONS_V2 * triggerConditionsV2,
							int16_t nTriggerConditions,
							TRIGGER_DIRECTIONS * directions,

						struct tPwq * pwq,
							uint32_t delay,
							int16_t auxOutputEnabled,
							int32_t autoTriggerMs,
							PS3000A_DIGITAL_CHANNEL_DIRECTIONS * digitalDirections,
							int16_t nDigitalDirections)
{
	PICO_STATUS status;

	if ((status = ps3000aSetTriggerChannelProperties(unit->handle,
		channelProperties,
		nChannelProperties,
		auxOutputEnabled,
		autoTriggerMs)) != PICO_OK) 
	{
		printf("SetTrigger:ps3000aSetTriggerChannelProperties ------ Ox%08lx \n", status);
		return status;
	}

	if ((status = ps3000aSetTriggerChannelConditionsV2(unit->handle, triggerConditionsV2, nTriggerConditions)) != PICO_OK) 
	{
		printf("SetTrigger:ps3000aSetTriggerChannelConditions ------ 0x%08lx \n", status);
		return status;
	}

	if ((status = ps3000aSetTriggerChannelDirections(unit->handle,
		directions->channelA,
		directions->channelB,
		directions->channelC,
		directions->channelD,
		directions->ext,
		directions->aux)) != PICO_OK) 
	{
		printf("SetTrigger:ps3000aSetTriggerChannelDirections ------ 0x%08lx \n", status);
		return status;
	}

	if ((status = ps3000aSetTriggerDelay(unit->handle, delay)) != PICO_OK) 
	{
		printf("SetTrigger:ps3000aSetTriggerDelay ------ 0x%08lx \n", status);
		return status;
	}

	if((status = ps3000aSetPulseWidthQualifierV2(unit->handle, 
		pwq->conditions,
		pwq->nConditions, 
		pwq->direction,
		pwq->lower, 
		pwq->upper, 
		pwq->type)) != PICO_OK)
	{
		printf("SetTrigger:ps3000aSetPulseWidthQualifier ------ 0x%08lx \n", status);
		return status;
	}

	if (unit->digitalPorts)					
	{
		if((status = ps3000aSetTriggerDigitalPortProperties(unit->handle, digitalDirections, nDigitalDirections)) != PICO_OK) 
		{
			printf("SetTrigger:ps3000aSetTriggerDigitalPortProperties ------ 0x%08lx \n", status);
			return status;
		}
	}
	return status;
}

/****************************************************************************
* collectBlockTriggered
*  this function demonstrates how to collect a single block of data from the
*  unit, when a trigger event occurs.
****************************************************************************/
void collectBlockTriggered(UNIT * unit)
{

    /*
    PS3000A_50MV:  50 mV
    PS3000A_100MV: 100 mV
    PS3000A_200MV: 200 mV
    PS3000A_500MV: 500 mV
    PS3000A_1V:    1 V
    PS3000A_2V:    2 V
    PS3000A_5V:    5 V
    PS3000A_10V:   10 V
    PS3000A_20V:   20 V
     */
    
    unit->channelSettings[PS3000A_CHANNEL_A].enabled = 1;
	unit->channelSettings[PS3000A_CHANNEL_A].DCcoupled = FALSE;
    unit->channelSettings[PS3000A_CHANNEL_A].range = PS3000A_50MV;
    
    unit->channelSettings[PS3000A_CHANNEL_B].enabled = 1;
	unit->channelSettings[PS3000A_CHANNEL_B].DCcoupled = TRUE;
    unit->channelSettings[PS3000A_CHANNEL_B].range = PS3000A_1V;
    
	int16_t triggerVoltage = mv_to_adc(300, unit->channelSettings[PS3000A_CHANNEL_B].range, unit);

	struct tPS3000ATriggerChannelProperties sourceDetails = {	triggerVoltage,
																256 * 10,
																triggerVoltage,
																256 * 10,
																PS3000A_CHANNEL_B,
																PS3000A_LEVEL};

	struct tPS3000ATriggerConditionsV2 conditions = {	PS3000A_CONDITION_DONT_CARE,  // trigger on channel A
														PS3000A_CONDITION_TRUE,       // trigger on channel B
														PS3000A_CONDITION_DONT_CARE,
														PS3000A_CONDITION_DONT_CARE,
														PS3000A_CONDITION_DONT_CARE,
														PS3000A_CONDITION_DONT_CARE,
														PS3000A_CONDITION_DONT_CARE,
														PS3000A_CONDITION_DONT_CARE};

	struct tPwq pulseWidth;

	struct tTriggerDirections directions = {	PS3000A_NONE,
												PS3000A_RISING,  // trigger on channel B rising edge
												PS3000A_NONE,
												PS3000A_NONE,
												PS3000A_NONE,
												PS3000A_NONE };

	memset(&pulseWidth, 0, sizeof(struct tPwq));

	printf("Collect block triggered...\n");
	printf("Collects when value rises past %d", scaleVoltages?
		adc_to_mv(sourceDetails.thresholdUpper, unit->channelSettings[PS3000A_CHANNEL_B].range, unit)	// If scaleVoltages, print mV value
		: sourceDetails.thresholdUpper);																// else print ADC Count

	printf(scaleVoltages?"mV\n" : "ADC Counts\n");
    
	setDefaults(unit);

	/* Trigger enabled
	* Rising edge
	* Threshold = 1000mV */
	setTrigger(unit, &sourceDetails, 1, &conditions, 1, &directions, &pulseWidth, 0, 0, 0, 0, 0);

	blockDataHandler(unit, "Ten readings after trigger:\n", 0, ANALOGUE);
}


/****************************************************************************
* Initialise unit' structure with Variant specific defaults
****************************************************************************/
void get_info(UNIT * unit)
{
	char description [11][25]= { "Driver Version",
		"USB Version",
		"Hardware Version",
		"Variant Info",
		"Serial",
		"Cal Date",
		"Kernel",
		"Digital H/W",
		"Analogue H/W",
		"Firmware 1",
		"Firmware 2"};

	int16_t i, r = 0;
	int8_t line [80];
	
	// Variables used for arbitrary waveform parameters
	int16_t			minArbitraryWaveformValue = 0;
	int16_t			maxArbitraryWaveformValue = 0;
	uint32_t		minArbitraryWaveformSize = 0;
	uint32_t		maxArbitraryWaveformSize = 0;

	PICO_STATUS status = PICO_OK;

	//Initialise default unit properties and change when required
	unit->sigGen		= SIGGEN_FUNCTGEN;
	unit->firstRange	= PS3000A_50MV;
	unit->lastRange		= PS3000A_20V;
	unit->channelCount	= DUAL_SCOPE;
	unit->ETS			= FALSE;
	unit->AWGFileSize	= MIN_SIG_GEN_BUFFER_SIZE;
	unit->digitalPorts	= 0;

	if (unit->handle) 
	{
		for (i = 0; i < 11; i++) 
		{
			status = ps3000aGetUnitInfo(unit->handle, line, sizeof (line), &r, i);

			if (i == 3) 
			{
				memcpy(unit->model, line, strlen((char*)(line))+1);

				// If 4 (analogue) channel variant 
				if (line[1] == '4')
				{
					unit->channelCount = QUAD_SCOPE;
				}

				// If ETS mode is enabled
				if (strlen(line) == 8 || line[3] != '4')
				{
					unit->ETS	= TRUE;
				}

				
				if (line[4] != 'A')
				{
					// Set Signal generator type if the device is not an 'A' model
					unit->sigGen = SIGGEN_AWG;

					// PicoScope 3000D and 3000D MSO models have a lower range of +/- 20mV
					if (line[4] == 'D')
					{
						unit->firstRange = PS3000A_20MV;
					}
				}

				// Check if MSO device
				if (strlen(line) >= 7)
				{
					line[4] = toupper(line[4]);
					line[5] = toupper(line[5]);
					line[6] = toupper(line[6]);

					if(strcmp(line + 4, "MSO") == 0 || strcmp(line + 5, "MSO") == 0 )
					{
						unit->digitalPorts = 2;
						unit->sigGen = SIGGEN_AWG;
					}
				}

				// If device has Arbitrary Waveform Generator, find the maximum AWG buffer size
				if (unit->sigGen == SIGGEN_AWG)
				{
					ps3000aSigGenArbitraryMinMaxValues(unit->handle, &minArbitraryWaveformValue, &maxArbitraryWaveformValue, &minArbitraryWaveformSize, &maxArbitraryWaveformSize);
					unit->AWGFileSize = maxArbitraryWaveformSize;
				}
			}
			printf("%s: %s\n", description[i], line);
		}		
	}
}






/****************************************************************************
* displaySettings 
* Displays information about the user configurable settings in this example
* Parameters 
* - unit        pointer to the UNIT structure
*
* Returns       none
***************************************************************************/
void displaySettings(UNIT *unit)
{
	int32_t ch;
	int32_t voltage;

	printf("\n\nReadings will be scaled in (%s)\n\n", (scaleVoltages)? ("mV") : ("ADC counts"));

	for (ch = 0; ch < unit->channelCount; ch++)
	{
		if (!(unit->channelSettings[ch].enabled))
		{
			printf("Channel %c Voltage Range = Off\n", 'A' + ch);
		}
		else
		{
			voltage = inputRanges[unit->channelSettings[ch].range];
			printf("Channel %c Voltage Range = ", 'A' + ch);

			if (voltage < 1000)
			{
				printf("%dmV\n", voltage);
			}
			else
			{
				printf("%dV\n", voltage / 1000);
			}
		}
	}

	printf("\n");

	if(unit->digitalPorts > 0)
	{
		printf("Digital ports switched off.");
	}

	printf("\n");
}

/****************************************************************************
* openDevice 
* Parameters 
* - unit        pointer to the UNIT structure, where the handle will be stored
*
* Returns
* - PICO_STATUS to indicate success, or if an error occurred
***************************************************************************/
PICO_STATUS openDevice(UNIT *unit, uint32_t timebase)
{
	int16_t value = 0;
	int32_t i;
	struct tPwq pulseWidth;
	struct tTriggerDirections directions;
	
	PICO_STATUS status = ps3000aOpenUnit(&(unit->handle), NULL);

	if (status == PICO_POWER_SUPPLY_NOT_CONNECTED || status == PICO_USB3_0_DEVICE_NON_USB3_0_PORT )
	{
		status = changePowerSource(unit->handle, status);
	}

	printf("\nHandle: %d\n", unit->handle);

	if (status != PICO_OK) 
	{
		printf("Unable to open device\n");
		printf("Error code : 0x%08lx\n", status);
		while(!_kbhit());
		exit(99); // exit program
	}

	printf("Device opened successfully, cycle %d\n\n", ++cycles);

	// setup devices
	get_info(unit);
	timebase = timebase;

	ps3000aMaximumValue(unit->handle, &value);
	unit->maxValue = value;

	for ( i = 0; i < unit->channelCount; i++) 
	{
		unit->channelSettings[i].enabled = TRUE;
		unit->channelSettings[i].DCcoupled = TRUE;
		unit->channelSettings[i].range = PS3000A_5V;
	}

	memset(&directions, 0, sizeof(struct tTriggerDirections));
	memset(&pulseWidth, 0, sizeof(struct tPwq));

	setDefaults(unit);

	/* Trigger disabled	*/
	setTrigger(unit, NULL, 0, NULL, 0, &directions, &pulseWidth, 0, 0, 0, 0, 0);

	return status;
}


/****************************************************************************
* closeDevice 
****************************************************************************/
void closeDevice(UNIT *unit)
{
	ps3000aCloseUnit(unit->handle);
}

/****************************************************************************
* main
* 
***************************************************************************/
int32_t main(int argc, char *argv[])
{
	PICO_STATUS status;
	UNIT unit;
    int32_t timeInterval;
    int32_t maxSamples;
    float megasamples_per_second;

    
    if(argc > 2)
    {
        megasamples_per_second = atof(argv[1]);
        if(megasamples_per_second > 1000.0f)
            megasamples_per_second = 1000.0f;
        else if (megasamples_per_second < 1.0f)
            megasamples_per_second = 1.0f;
        
        if(megasamples_per_second > 250.0f)
            timebase = (int32_t)(1000.0f/megasamples_per_second) - 1;
        else if(megasamples_per_second > 125.0f)
            timebase = 2;
        else if(megasamples_per_second <= 125.0f)
            timebase = 125.0f/megasamples_per_second + 2;
        
        
        strcpy(Filename_custom, argv[2]);
        printf("timebase: %d\n", timebase);
        printf("filenameprefix: %s", Filename_custom);
    }
    else
    {
        printf("usage: %s [megasamples_per_second] [filename] [ttydevice]", argv[0]);
    }
    
    
    sleep(1);
    srand(time(NULL));   // should only be called once

	printf("PicoScope 3000 Series powertrace capture program\n");
	printf("\nOpening the device...\n");

    serial_port = open_port(argv[3],115200);
    
	status = openDevice(&unit, timebase);

    do
	{
		status = ps3000aGetTimebase(unit.handle, timebase, BUFFER_SIZE, &timeInterval, 1, &maxSamples, 0);
		
		if (status != PICO_OK)
		{
			if(status == PICO_INVALID_CHANNEL)
			{
				printf("ps3000aGetTimebase: Status Error 0x%x\n", status);
				printf("Please enable an analogue channel (option V from the main menu).");
				return;

			}
			else
			{
				timebase++;  // Increase timebase if the one specified can't be used. 
			}
		}
	}
	while(status != PICO_OK);

	printf("Timebase used %lu = %ld ns sample interval\n", timebase, timeInterval);

    for(int counter = 0;counter < TRACE_COUNT + TRACES_TO_SKIP; counter++)
    {
        collectBlockTriggered(&unit);
        usleep(5000); // short delay for stability
    }
	
	closeDevice(&unit);

	return 1;
}
