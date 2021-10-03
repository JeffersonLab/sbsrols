/*************************************************************************
 *
 *  ti_master_list.c - Library of routines for readout and buffering of
 *                     events using a JLAB Trigger Interface V3 (TI) with
 *                     a Linux VME controller in CODA 3.0.
 *
 *                     This for a TI in Master Mode controlling multiple ROCs
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   1024*64      /* Size in Bytes */

/* Define TI Type (TI_MASTER or TI_SLAVE) */
#define TI_MASTER
/* EXTernal trigger source (e.g. front panel ECL input), POLL for available data */
#define TI_READOUT TI_READOUT_EXT_POLL
/* TI VME address, or 0 for Auto Initialize (search for TI by slot) */
#define TI_ADDR  0
/* Skip the firmware check while we're testing 3v11.1 */
#define TI_FLAG TI_INIT_SKIP_FIRMWARE_CHECK


/* Measured longest fiber length in system */
#define FIBER_LATENCY_OFFSET 0x50

#include "dmaBankTools.h"   /* Macros for handling CODA banks */
#include "tiprimary_list.c" /* Source required for CODA readout lists using the TI */
#include "sdLib.h"

/* Library to pipe stdout to daLogMsg */
#include "dalmaRolLib.h"

#include "usrstrutils.c"

/* Define initial blocklevel and buffering level */
#define BLOCKLEVEL 1
#define BUFFERLEVEL 5

/*
  Global to configure the trigger source
      0 : tsinputs
      1 : TI random pulser
      2 : TI fixed pulser

  Set with rocSetTriggerSource(int source);
*/
int rocTriggerSource = 0;
void rocSetTriggerSource(int source); // routine prototype

/*

  Read the user flags/configuration file.

*/
typedef struct
{
  int slave1;
  int slave2;
  int slave3;
  int slave4;
} TI_SLAVE_CONFIG;

TI_SLAVE_CONFIG tiSlaveConfig = {0,0,0,0};

void
readUserFlags()
{
  int i;

  printf("%s: Reading user flags file.",
	 __func__);
  init_strings();

  tiSlaveConfig.slave1 = getflag("slave1");
  if(tiSlaveConfig.slave1 == 2)
    {
      tiSlaveConfig.slave1 = getint("slave1");
    }

  tiSlaveConfig.slave2 = getflag("slave2");
  if(tiSlaveConfig.slave2 == 2)
    {
      tiSlaveConfig.slave2 = getint("slave2");
    }

  tiSlaveConfig.slave3 = getflag("slave3");
  if(tiSlaveConfig.slave3 == 2)
    {
      tiSlaveConfig.slave3 = getint("slave3");
    }

  tiSlaveConfig.slave4 = getflag("slave4");
  if(tiSlaveConfig.slave4 == 2)
    {
      tiSlaveConfig.slave4 = getint("slave4");
    }

  printf("%s:  tiSlaveConfig = { %d, %d, %d, %d }\n",
	 __func__,
	 tiSlaveConfig.slave1,
	 tiSlaveConfig.slave2,
	 tiSlaveConfig.slave3,
	 tiSlaveConfig.slave4);

}

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  int stat;

  /* Read in the init flags using usrstrutils.c */
  readUserFlags();

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,5,1);

  /* Define BLock Level */
  blockLevel = BLOCKLEVEL;


  /*****************
   *   TI SETUP
   *****************/

  /*
   * Set Trigger source
   *    For the TI-Master, valid sources:
   *      TI_TRIGGER_FPTRG     2  Front Panel "TRG" Input
   *      TI_TRIGGER_TSINPUTS  3  Front Panel "TS" Inputs
   *      TI_TRIGGER_TSREV2    4  Ribbon cable from Legacy TS module
   *      TI_TRIGGER_PULSER    5  TI Internal Pulser (Fixed rate and/or random)
   */
  if(rocTriggerSource == 0)
    {
      tiSetTriggerSource(TI_TRIGGER_TSINPUTS); /* TS Inputs enabled */
    }
  else
    {
      tiSetTriggerSource(TI_TRIGGER_PULSER); /* Internal Pulser */
    }

  /* Set needed TS input bits */
  tiEnableTSInput( TI_TSINPUT_1 );
  tiSetTSInputDelay(1,73);// 0 = 8 ns - 4 ns steps - need 300 ns
  /* Load the trigger table that associates
   *  pins 21/22 | 23/24 | 25/26 : trigger1
   *  pins 29/30 | 31/32 | 33/34 : trigger2
   */
  tiLoadTriggerTable(0);

  // MPD 1 sample readout = 3.525 us = 8 * 480
  tiSetTriggerHoldoff(1,30,1); /* 1 trigger in 20*480ns window */
  //tiSetTriggerHoldoff(1,60,1); /* 1 trigger in 20*480ns window */
  tiSetTriggerHoldoff(2,0,0);  /* 2 trigger in don't care window */

  tiSetTriggerHoldoff(3,0,0);  /* 3 trigger in don't care window */
  tiSetTriggerHoldoff(4,20,1);  /* 4 trigger in 20*3840ns window */

  /* Set initial number of events per block */
  tiSetBlockLevel(blockLevel);

  /* Set Trigger Buffer Level */
  tiSetBlockBufferLevel(BUFFERLEVEL);

  tiSetTriggerPulse(1,0,25,0);

  /* Set prompt output width (127 + 2) * 4 = 516 ns */
  tiSetPromptTriggerWidth(127);


  tiSetOutputPort(1,1,0,0);

  /* Init the SD library so we can get status info */
  stat = sdInit(0);
  if(stat==0)
    {
      sdSetActiveVmeSlots(0);
      sdStatus(0);
    }

  if(tiSlaveConfig.slave1)
    {
      printf("%s: Adding Slave to port 1 (%d)\n",
	     __func__, tiSlaveConfig.slave1);
      tiAddSlave(1);
    }
  if(tiSlaveConfig.slave2)
    {
      printf("%s: Adding Slave to port 2 (%d)\n",
	     __func__, tiSlaveConfig.slave2);
      tiAddSlave(2);
    }
  if(tiSlaveConfig.slave3)
    {
      printf("%s: Adding Slave to port 3 (%d)\n",
	     __func__, tiSlaveConfig.slave3);
      tiAddSlave(3);
    }
  if(tiSlaveConfig.slave4)
    {
      printf("%s: Adding Slave to port 4 (%d)\n",
	     __func__, tiSlaveConfig.slave4);
      tiAddSlave(4);
    }


  tiStatus(0);

  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{

  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);

  tiSetPrescale(0);
  tiSetOutputPort(1,1,0,0);

  tiStatus(0);

  printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{

  /* Print out the Run Number and Run Type (config id) */
  printf("rocGo: Activating Run Number %d, Config id = %d\n",
	 rol->runNumber,rol->runType);

  tiSetOutputPort(0,1,0,0);

  /* Get the current Block Level */
  blockLevel = tiGetCurrentBlockLevel();
  printf("rocGo: Block Level set to %d\n",blockLevel);

  tiStatus(0);

  /* Enable/Set Block Level on modules, if needed, here */
  if(rocTriggerSource != 0)
    {
      printf("************************************************************\n");
      daLogMsg("INFO","TI Configured for Internal Pulser Triggers");
      printf("************************************************************\n");

      if(rocTriggerSource == 1)
	{
	  /* Enable Random at rate 500kHz/(2^7) = ~3.9kHz */
	  	  tiSetRandomTrigger(1,0xd);
	  //tiSetRandomTrigger(1,0x4);
	}

      if(rocTriggerSource == 2)
	{
	  /*    Enable fixed rate with period (ns)
		120 +30*700*(1024^0) = 21.1 us (~47.4 kHz)
		- arg2 = 0xffff - Continuous
		- arg2 < 0xffff = arg2 times
	  */
	  tiSoftTrig(1,0xffff,100,0);
	}
    }

}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{

  if(rocTriggerSource == 1)
    {
      /* Disable random trigger */
      tiDisableRandomTrigger();
    }

  if(rocTriggerSource == 2)
    {
      /* Disable Fixed Rate trigger */
      tiSoftTrig(1,0,100,0);
    }

  tiSetOutputPort(1,1,0,0);
  tiStatus(0);

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int dCnt;

  /* Set TI output 1 high for diagnostics */
  tiSetOutputPort(1,0,0,0);

  /* Readout the trigger block from the TI
     Trigger Block MUST be readout first */
  dCnt = tiReadTriggerBlock(dma_dabufp);

  if(dCnt<=0)
    {
      printf("No TI Trigger data or error.  dCnt = %d\n",dCnt);
    }
  else
    { /* TI Data is already in a bank structure.  Bump the pointer */
      dma_dabufp += dCnt;
    }

  /* Set TI output 0 low */
  tiSetOutputPort(0,0,0,0);

}

void
rocLoad()
{
  dalmaInit(1);
}
void
rocCleanup()
{

  printf("%s: Reset all Modules\n",__FUNCTION__);
  tiResetSlaveConfig();
  dalmaClose();

}

void
rocSetTriggerSource(int source)
{
#ifdef TI_MASTER
  if(TIPRIMARYflag == 1)
    {
      printf("%s: ERROR: Trigger Source already enabled.  Ignoring change to %d.\n",
	     __func__, source);
    }
  else
    {
      rocTriggerSource = source;

      if(rocTriggerSource == 0)
	{
	  tiSetTriggerSource(TI_TRIGGER_TSINPUTS); /* TS Inputs enabled */
	}
      else
	{
	  tiSetTriggerSource(TI_TRIGGER_PULSER); /* Internal Pulser */
	}

      daLogMsg("INFO","Setting trigger source (%d)", rocTriggerSource);
    }
#else
  printf("%s: ERROR: TI is not Master  Ignoring change to %d.\n",
	 __func__, source);
#endif
}
