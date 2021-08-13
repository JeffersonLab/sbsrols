/*************************************************************************
 *
 *  ts_list.c - Library of routines for readout and buffering of
 *                events using a JLAB Pipeline Trigger Supervisor (TS) with
 *                a Linux VME controller in CODA 3.0
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     100
#define MAX_EVENT_LENGTH   1152*32      /* Size in Bytes */

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

#define BLOCKLEVEL 40
#define BUFFERLEVEL 4
#define SYNC_INTERVAL 100000

extern unsigned int tsTriggerSource;

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
#define RANDOMPULSER
int usePulser=0;

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

  if(usePulser)
    tsSetTriggerSource(TS_TRIGSRC_PULSER);
  else
    tsSetTriggerSource(TS_TRIGSRC_EXT);

  /* Enable/Disable specific inputs */
  tsSetFPInput(0x7);
  tsSetGTPInput(0x0);

  /* Set Time stamp format - 48 bits */
  tsSetEventFormat(3);
  tsSetGTPInputReadout(1);  /* 1 enables output of GTP trigger word into trigger event*/
  tsSetFPInputReadout(1);

  /* Load the default trigger table */
  tsLoadTriggerTable();

  /*
   * Trigger Holdoff rules:
   *
   *
   */
  tsSetTriggerHoldoff(1,6,0);   /* 1 trigger DT = 32ns + 6*16 = 124ns  */
  tsSetTriggerHoldoff(2,0,0);  /* Disable */
  tsSetTriggerHoldoff(3,0,0);  /* Disable */
  tsSetTriggerHoldoff(4,63,0);  /* max 4 triggers in 64ns + 64ns*63 = 4.032 microsec */

  /* Set the sync delay width to 0x40*32 = 2.048us */
  tsSetSyncDelayWidth(0x30, 0x40, 1);

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

#define USETD
#ifdef USETD

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

#else
  tsSetBusySource(TS_BUSY_LOOPBACK,1);
  tsAddSlave(1);
#endif

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

  /* Enable TD module Ports that have indicated they are active */
  for (ii=0;ii<nTD;ii++) {
    tdResetSlaveConfig(tdID[ii]);
    tmask = tdGetTrigSrcEnabledFiberMask(tdID[ii]);
    printf("TD (Slot %d) Source Enable Mask = 0x%x\n",tdID[ii],tmask);
    if(tmask!=0) tdAddSlaveMask(tdID[ii],tmask);
  }


  tdGStatus(0);

  usrDebugFlag=0;


  /* Enable modules, if needed, here */
  tsStatus(0);

  /* Example: How to start internal pulser trigger */
  if(usePulser)
    {
#ifdef RANDOMPULSER
      /* Enable Random at rate 500kHz/(2^7) = ~3.9kHz */
      tsSetRandomTrigger(1,RANDOM_RATE);
#else
      /* Enable fixed rate with period (ns) 120 +30*val*(1024^0) = 21.1 us (~47.4 kHz)
	 - Generated 1000 times */
      tsSoftTrig(1,1000,0x7fff,1);
#endif
    }

}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{

  int islot;

  /* Example: How to stop internal pulser trigger */
  if(usePulser)
    {
#ifdef RANDOMPULSER
      /* Disable random trigger */
      tsDisableRandomTrigger();
#else
      /* Disable Fixed Rate trigger */
      tsSoftTrig(1,0,700,0);
#endif
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
rocCleanup()
{
  int islot=0;

}
