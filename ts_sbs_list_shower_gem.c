/*************************************************************************
 *
 *  ts_sbslist.c - Library of routines for readout and buffering of
 *                events using a JLAB Pipeline Trigger Supervisor (TS) with
 *                a Linux VME controller in CODA 3.0
 *
 *
 *     This is for the SBS TS ~ August 2021
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     100
#define MAX_EVENT_LENGTH   1152*32      /* Size in Bytes */
#define AUTOSELECT
/* Define maximum number of words in the event block
   MUST be less than MAX_EVENT_LENGTH/4   */
#define MAX_WORDS 4000

/* Define Interrupt source and address */
#define TS_READOUT TS_READOUT_EXT_POLL  /* Poll for available data, external triggers */
/* #define TS_ADDR    (20<<19)           GEO slot 20  for ELMA backplane*/
#define TS_ADDR  0                 /* 0 for Autoscan for TS */


/* make useful TD library variables available*/
extern int tdID[21];
extern int nTD;

#include "dmaBankTools.h"
#include "tsprimary_list.c" /* source required for CODA */
#include "sdLib.h"
#include "tdLib.h"

#define BLOCKLEVEL  1
#define BUFFERLEVEL 4
#define SYNC_INTERVAL 100

/* Set to Zero to define a Front Panel Trigger source
   RANDOM_RATE defines the Pulser rate
   = 0    500.00kHz
     1    250.00kHz
     2    125.00kHz
     3     62.50kHz
     4     31.25kHz
     5     15.63kHz
     6      7.81kHz
     7      3.91kHz   */
#define RANDOM_RATE  5

/*
  Global to configure the trigger source
      0 : external inputs
      1 : internal random pulser
      2 : internal fixed rate pulser

  Set with rocSetTriggerSource(int source);
*/
int rocTriggerSource = 0;
void rocSetTriggerSource(int source); // routine prototype

/* Global Flag for debug printing */
int usrDebugFlag=0;


/* function prototype */
void rocTrigger(int arg);

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{

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
  bufferLevel = BUFFERLEVEL;


  /*****************
   *   TS SETUP
   *****************/

  if(rocTriggerSource == 0)
    {
      tsSetTriggerSource(TS_TRIGSRC_EXT);
    }
  else
    {
      tsSetTriggerSource(TS_TRIGSRC_PULSER);
    }

  /* Enable/Disable specific inputs */
  tsSetFPInput(0x7);
  tsSetFPDelay(1,0);
  tsSetGTPInput(0x0);

  /* Set Time stamp format - 48 bits */
  tsSetEventFormat(3);
  /* tsSetGTPInputReadout(1);  /\* 1 enables output of GTP trigger word into trigger event*\/ */
  tsSetFPInputReadout(1);

  /* Load the default trigger table */
  tsLoadTriggerTable();

  /*
   * Trigger Holdoff rules:
   *
   *   Originals taken from GEM ti_master_list.so
   */
  // MPD 1 sample readout = 3.525 us = 8 * 480
  
  tsSetTriggerHoldoff(1,30,1); /* 1 trigger in 20*480ns window */
  //tsSetTriggerHoldoff(1,60,1); /* 1 trigger in 20*480ns window */
 tsSetTriggerHoldoff(1,60,1); /* 1 trigger in 20*480ns window */
  tsSetTriggerHoldoff(2,0,0);  /* 2 trigger in don't care window */

  tsSetTriggerHoldoff(3,0,0);  /* 3 trigger in don't care window */
  tsSetTriggerHoldoff(4,20,1);  /* 4 trigger in 20*3840ns window */

  /*
   * Set the Block Buffer Level
   *  0:  Pipeline mode
   *  1:  One Block per readout - "ROC LOCK" mode
   *  2-255:  "Buffered" mode.
   */
  tsSetBlockBufferLevel(BUFFERLEVEL);

  /* Set a Maximum Block count before trigger autmatically disables  (0 disables block limit)*/
  tsSetBlockLimit(0);

  /* Override the busy source set in tsInit (only if TS crate running alone) */

 /* Setup TDs - */
  tdInit(0,0,0,0);
  tdGSetBlockBufferLevel(BUFFERLEVEL);

  /* No Need to add TD slaves here */
  /*tdAddSlave(TD_SLOT_1,4);  */   /* TI Slave - Slot 4 only */

  /* Init SD Board. and add TD Slaves */
  sdInit(0);
  sdSetActiveVmeSlots(tdSlotMask());
  sdStatus(0);


  /* Reset Active ROC Masks on all TD modules */
  tsTriggerReadyReset();


/*   tsSetPrescale(0); */

  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{
  unsigned short iflag;
  unsigned int ival = SYNC_INTERVAL;
  int stat;
  int islot;


  /* Set number of events per block */
  tsSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level to be broadcasted: %d\n",blockLevel);
  /* On TD's too */
  tdGSetBlockLevel(blockLevel);


  /* Reset Active ROC Masks on all TD modules */
  tsTriggerReadyReset();

  /* Set Sync Event Interval  (0 disables sync events, max 65535) */
  tsSetSyncEventInterval(ival);
  printf("rocPrestart: Set Sync interval to %d Blocks\n",ival);

  /* Print Status info */
  tdGStatus(0);
  tsStatus(0);

  printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{

  int ii, islot, tmask;

#ifdef AUTOSELECT
  /* Enable TD module Ports that have indicated they are active */
  for (ii=0;ii<nTD;ii++) {
    tdResetSlaveConfig(tdID[ii]);
    tmask = tdGetTrigSrcEnabledFiberMask(tdID[ii]);
    printf("TD (Slot %d) Source Enable Mask = 0x%x\n",tdID[ii],tmask);
    if(tmask!=0) tdAddSlaveMask(tdID[ii],tmask);
  }
#else
  for (ii=0;ii<nTD;ii++) {
    tdResetSlaveConfig(tdID[ii]);
  }
  tdAddSlave(20, 6); // bbshower6
  tdAddSlave(20, 2); // bbgem19

#endif

  tdGStatus(0);

  usrDebugFlag=0;


  /* Enable modules, if needed, here */
  tsStatus(0);

  if(rocTriggerSource != 0)
    {
      printf("************************************************************\n");
      daLogMsg("INFO","TS Configured for Internal Pulser Triggers");
      printf("************************************************************\n");

      if(rocTriggerSource == 1)
	{
	  /* Enable Random at rate 500kHz/(2^7) = ~3.9kHz */
	  //	  tiSetRandomTrigger(1,0xd);
	  tsSetRandomTrigger(1,RANDOM_RATE);
	}

      if(rocTriggerSource == 2)
	{
	  /*    Enable fixed rate with period (ns)
		120 +30*700*(1024^0) = 21.1 us (~47.4 kHz)
		- arg2 = 0xffff - Continuous
		- arg2 < 0xffff = arg2 times
	  */
	  tsSoftTrig(1,0xffff,100,0);
	}
    }

}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{

  int islot;

  if(rocTriggerSource == 1)
    {
      /* Disable random trigger */
      tsDisableRandomTrigger();
    }

  if(rocTriggerSource == 2)
    {
      /* Disable Fixed Rate trigger */
      tsSoftTrig(1,0,100,0);
    }

  tsStatus(0);

  printf("rocEnd: Ended after %d blocks\n",tsGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int evntno)
{
  int ii, islot;
  int stat, dCnt, len=0, idata;
  int timeout;


  /* Check if this is a Sync Event */
  /*  stat = tsGetSyncEventFlag(); */
  stat = syncFlag;
  if(stat) {
    printf("rocTrigger: Got Sync Event!! Block # = %d\n",evntno);
    usrDebugFlag=0;
  }

  /* Set Output port bit 0  */
  tsSetOutputPort(1,0,0,0,0,0);

  /* Readout the trigger block from the TS
     Trigger Block MUST be reaodut first */
  dCnt = tsReadTriggerBlock(dma_dabufp);
  if(dCnt<=0)
    {
      logMsg("No data or error.  dCnt = %d\n",dCnt);
    }
  else
    { /* TS Data is already in a bank structure.  Bump the pointer */
      if(stat) {
	printf("rocTrigger: Sync Event data: 0x%08x 0x%08x 0x%08x 0x%08x\n",
	       *dma_dabufp, *(dma_dabufp+1), *(dma_dabufp+2), *(dma_dabufp+3));
      }
      dma_dabufp += dCnt;
    }

  if(stat) {
    /* Set new block level if it has changed */
    idata = tsGetCurrentBlockLevel();
    if((idata != blockLevel)&&(idata<255)) {
      blockLevel = idata;
      printf("rocTrigger: Block Level changed to %d\n",blockLevel);
    }

    /* Clear/Update Modules here */

  }

  /* Clear all output register bits */
  tsSetOutputPort(0,0,0,0,0,0);

}

void
rocSetTriggerSource(int source)
{
  if(TSPRIMARYflag == 1)
    {
      printf("%s: ERROR: Trigger Source already enabled.  Ignoring change to %d.\n",
	     __func__, source);
    }
  else
    {
      rocTriggerSource = source;

      if(rocTriggerSource == 0)
	{
	  tsSetTriggerSource(TS_TRIGSRC_EXT); /* External inputs */
	}
      else
	{
	  tsSetTriggerSource(TS_TRIGSRC_PULSER);
	}

      daLogMsg("INFO","Setting trigger source (%d)", rocTriggerSource);
    }
}

void
rocCleanup()
{
  int islot=0;

}

/*
  Local Variables:
  compile-command: "make ts_sbs_list.so"
  End:
 */
