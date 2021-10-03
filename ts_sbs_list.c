/*************************************************************************
 *
 *  ts_sbs_list.c - Library of routines for readout and buffering of
 *                events using a JLAB Pipeline Trigger Supervisor (TS) with
 *                a Linux VME controller in CODA 3.0
 *
 *
 *     This is for the SBS TS ~ August 2021
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

#define SCALERS 1
#ifdef SCALERS
int scaler_inhibit=0;
#endif

#include "dmaBankTools.h"
#include "tsprimary_list.c" /* source required for CODA */
#include "sdLib.h"
#include "tdLib.h"

/* Library to pipe stdout to daLogMsg */
#include "dalmaRolLib.h"

#include "usrstrutils.c"

#define BLOCKLEVEL  1
/* override this setting with 'bufferlevel' user string */
#define BUFFERLEVEL 5
#define SYNC_INTERVAL 10000

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
#define RANDOM_RATE  6

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

/*
  Hardcode ROC names to their TD ports
  -- someday (tm) read them in with a config file - BM 10sept21
*/
typedef struct
{
  int enable;
  int slot;
  int port;
  int arm;
  char rocname[64];
} TD_SLAVE_MAP;

enum sbsARMS
  {
   armHCAL = 0,
   armLHRS,
   armBIGBITE,
   armSCALER,
   nArms
  };

char *armNames[nArms] =
  {
   "HCAL",
   "LHRS",
   "BIGBITE",
   "SCALER"
  };

enum sbsSlaves
  {
   nhcalROC16 = 0,
   nsbsvme29ROC1,
   nhcalROC17,
   nlhrsROC10,
   nbbgemROC19,
   ngrinchROC7,
   nbbshowerROC6,
   nbbhodoROC5,
   nSlaves
  };

TD_SLAVE_MAP tdSlaveConfig[nSlaves] =
  {
   { 0,  19,   1,  armHCAL,  "hcalROC16"},
   { 0,  19,   2,  armSCALER, "sbsvme29ROC1"},
   { 0,  19,   4,  armHCAL,  "hcalROC17"},
   { 0,  19,   8,  armLHRS,  "lhrsROC10"},
   { 0,  19,   6,  armBIGBITE, "bbgemROC19"},
   { 0,  20,   3,  armBIGBITE, "grinchROC7"},
   { 0,  20,   6,  armBIGBITE, "bbshowerROC6"},
   { 0,  19,   7,  armBIGBITE, "bbhodoROC5"}
  };


/* prescale factors gathered here */
#define NPSF 8
int psfact[NPSF];

/*
  Read the user flags/configuration file.
  10sept21 - BM
    - Support for
      all,
      HCAL, LHRS, BIGBITE
      ROCs listed in tdSlaveConfig
*/
void
readUserFlags()
{
  int flag = 0, flagval = 0;
  int i;

  printf("%s: Reading user flags file.",
	 __func__);
  init_strings();

  char *fstring = getstr("ffile");
  if(fstring == NULL)
    {
      /* Load a default */
    }

  /*
   *
   *   Set prescales factors from prescale.dat
   *   - this TS can have 32 inputs but for now we only use 8 for now
   *   - ps factor of -1 disables an input
   *   - otherwise the prescaling is 2^ps.
   *       For example ps=0 means 1 and ps=3 means 8
   */

  psfact[0] = getint(PS1);
  psfact[1] = getint(PS2);
  psfact[2] = getint(PS3);
  psfact[3] = getint(PS4);
  psfact[4] = getint(PS5);
  psfact[5] = getint(PS6);
  psfact[6] = getint(PS7);
  psfact[7] = getint(PS8);

  int jj;
  printf("\n****** Prescale factors : ");
  for (jj = 0; jj < NPSF; jj++) {
    printf("T%d=%d ; ",jj+1,psfact[jj]);
  }
  printf("\n\n");

  /* contruct mask for allowing TS inputs */

  unsigned int mask=0;
  for (jj = 0; jj<NPSF; jj++) {
    if (psfact[jj] > -1) mask |= (1<<jj);
  }

  printf("Enabled inputs mask = 0x%x \n",mask);

  /* Enable/Disable specific inputs */
  tsSetFPInput(mask);

  for (jj = 0; jj<NPSF; jj++) {
    if(psfact[jj]>0) tsSetTriggerPrescale(2,jj,psfact[jj]);
  }
  // tsSetFPInput(0x10);
  // tsSetTriggerPrescale(2,4,0);
  //tsSetTriggerPrescale(2,5,0);
  /* bufferLevel */
  flag = getflag("bufferlevel");
  if(flag)
    {
      bufferLevel = 1;

      if(flag > 1)
	{
	  bufferLevel = getint("bufferlevel");
	}

    }
  else
    {
      bufferLevel = BUFFERLEVEL;
    }
  printf("%s: Setting bufferlevel = %d\n",
	 __func__, bufferLevel);
  tsSetBlockBufferLevel(bufferLevel);

  // 30sept2021 8pm: Test turning off bufferlevel on TDs
  tdGSetBlockBufferLevel(0);

  /* Order of operations..
     - check 'all'
     - check 'arm'
     - check 'rocname'
  */

  /* enable 'all', 'all=1'
     disable 'all=0'
     pass the entire flag, if specified for future non-binary flag support.
  */

  flagval = 0;
  flag = getflag("all");

  if(flag)
    {
      flagval = 1;

      if(flag > 1)
	flagval = getint("all");

      for(i = 0; i < nSlaves; i++)
	tdSlaveConfig[i].enable = flagval;
    }

  /* enable 'BIGBITE', 'BIGBITE=1'
     disable 'BIGBITE=0'
     similar for 'LHRS' and 'HCAL'
  */
  int iarm=0;
  for(iarm = 0; iarm < nArms; iarm++)
    {
      flagval = 0;
      flag = getflag(armNames[iarm]);

      if(flag)
	{
	  flagval = 1;

	  if(flag > 1)
	    flagval = getint(armNames[iarm]);

	  for(i = 0; i < nSlaves; i++)
	    {
	      if(tdSlaveConfig[i].arm == iarm)
		tdSlaveConfig[i].enable = flagval;
	    }
	}
    }

  /* enable 'bbshowerROC6', 'bbshowerROC6=1'
     disable 'bbshowerROC6=0'
     similar for the rest of the crates
  */
  int islave=0;
  for(islave = 0; islave < nSlaves; islave++)
    {
      flagval = 0;
      flag = getflag(tdSlaveConfig[islave].rocname);

      if(flag)
	{
	  flagval = 1;

	  if(flag > 1)
	    flagval = getint(tdSlaveConfig[islave].rocname);

	  tdSlaveConfig[islave].enable = flagval;
	}

    }

  /* Print config */
  printf("%s\n TD Slave Config from usrstringutils\n",
	 __func__);
  printf("  Enable      Slot      Port       Arm       Roc\n");
  printf("|----------------------------------------------------------|\n");

  for(islave = 0; islave < nSlaves; islave++)
    {
      printf("       %d  ",
	     tdSlaveConfig[islave].enable);

      printf("      %2d ",
	     tdSlaveConfig[islave].slot);

      printf("       %d  ",
	     tdSlaveConfig[islave].port);

      printf(" %-10s",
	     armNames[tdSlaveConfig[islave].arm]);

      printf(" %-20s",
	     tdSlaveConfig[islave].rocname);

      printf("\n");
    }

  printf("\n");

}

#ifdef SCALERS
/* Remex function to toggle on or off the hardware scaler inhibit */
void setScalerInhibit(int inhibit) {
  scaler_inhibit = (inhibit!=0);
  /* Tweak bits 2-4.  Do just one bit once we figure out where
     these outputs are on the TS */
  tsSetOutputPort(0,0,scaler_inhibit,scaler_inhibit,0,0);
  if(scaler_inhibit) {
    printf("Scalers inhibited\n");
  } else {
    printf("Scalers enabled\n");
  }
}
#endif

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
   tsSetFPInput(0x1f);
  // tsSetFPInput(0x2);
  tsSetTriggerPrescale(2,0,0);
  tsSetTriggerPrescale(2,1,0);
  tsSetTriggerPrescale(2,2,0);
  tsSetTriggerPrescale(2,3,0);
  tsSetTriggerPrescale(2,4,0);
  tsSetTriggerPrescale(2,5,0);
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
  // 30sept2021 8pm: Turn off bufferlevel on TDs
  tdGSetBlockBufferLevel(0);
  /* Reset Active ROC Masks on all TD modules */
  int islot;
  for (islot = 0; islot < nTD; islot++)
    {
      tdTriggerReadyReset(tdSlot(islot));
    }

  /* Init SD Board. and set the initialzed TD Slots */
  sdInit(0);
  sdSetActiveVmeSlots(tdSlotMask());
  sdStatus(0);


#ifdef SCALERS
  /* Make sure scalers are not inhbited */
  setScalerInhibit(0);
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

#ifdef SCALERS
  /* Inhibit scalers */
  setScalerInhibit(1);
#endif

  /* Set number of events per block */
  tsSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level to be broadcasted: %d\n",blockLevel);
  /* On TD's too */
  tdGSetBlockLevel(blockLevel);

  /* Read User Flags with usrstringutils
     What's set
     - prescale factors
     - TD Slave Ports
     - bufferLevel
   */
  readUserFlags();

  /* Reset Active ROC Masks on all TD modules */
  for (islot = 0; islot < nTD; islot++)
    {
      tdTriggerReadyReset(tdSlot(islot));
    }

  /* Set Sync Event Interval  (0 disables sync events, max 65535) */
  tsSetSyncEventInterval(ival);
  printf("rocPrestart: Set Sync interval to %d Blocks\n",ival);

  /* Print Status info */
  DALMAGO;
  tdGStatus(0);
  tsStatus(0);
  DALMASTOP;

  printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{

  int ii, islot, tmask;

  /* Reset all TD slave configurations */
  for (ii=0;ii<nTD;ii++) {
    tdResetSlaveConfig(tdID[ii]);
  }

  /* Enable TD module ports that have been flaged from usrstringutils */
  int stringEnabled = 0;
  for(ii = 0; ii < nSlaves; ii++)
    {
      if(tdSlaveConfig[ii].enable)
	{
	  tdAddSlave(tdSlaveConfig[ii].slot,
		     tdSlaveConfig[ii].port);
	  stringEnabled=1;
	}
    }

  /* If none were enabled with usrstringutils, assume the auto method is preferred */
  if(!stringEnabled)
    {
      /* Enable TD module Ports that have indicated they are active */
      for (ii=0;ii<nTD;ii++) {
	tmask = tdGetTrigSrcEnabledFiberMask(tdID[ii]);
	printf("TD (Slot %d) Source Enable Mask = 0x%x\n",tdID[ii],tmask);
	if(tmask!=0) tdAddSlaveMask(tdID[ii],tmask);
      }

    }

  DALMAGO;
  tdGStatus(0);
  tsStatus(0);
  DALMASTOP;

  usrDebugFlag=0;


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

#ifdef SCALERS
  /* Enable scalers */
  setScalerInhibit(0);
#endif

}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{

  int islot;

#ifdef SCALERS  /* Inhibit scalers */
  setScalerInhibit(1);
  /* A script on the host will reenable scalers */
#endif


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

  for (islot = 0; islot < nTD; islot++)
    {
      tdLatchTimers(tdSlot(islot));
    }

  DALMAGO;
  tdGPrintBusyCounters();
  tdGStatus(0);
  tsStatus(0);
  DALMASTOP;

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
  stat = tsGetSyncEventFlag();
  if(stat) {
    printf("rocTrigger: Got Sync Event!! Block # = %d\n",evntno);
    usrDebugFlag=0;
  }

  /* Set Output port bit 0  */
#ifdef SCALERS
  tsSetOutputPort(0,0,scaler_inhibit,scaler_inhibit,0,0);
#else
  tsSetOutputPort(1,0,0,0,0,0);
#endif

  /* Readout the trigger block from the TS
     Trigger Block MUST be reaodut first */
  dCnt = tsReadTriggerBlock(dma_dabufp);
  if(dCnt<=0)
    {
      logMsg("No data or error.  dCnt = %d\n",dCnt);
    }
  else
    { /* TS Data is already in a bank structure.  Bump the pointer */
#ifdef DEBUGSYNCEVENT
      if(stat) {
	printf("rocTrigger: Sync Event data: 0x%08x 0x%08x 0x%08x 0x%08x\n",
	       *dma_dabufp, *(dma_dabufp+1), *(dma_dabufp+2), *(dma_dabufp+3));
      }
#endif
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

  /* Clear all output register bit 0 */
#ifdef SCALERS
  //  tsSetOutputPort(scaler_inhibit,scaler_inhibit,scaler_inhibit,scaler_inhibit,0,0);
#else
  tsSetOutputPort(0,0,0,0,0,0);
#endif

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
rocLoad()
{
  dalmaInit(1);
}

void
rocCleanup()
{
  int islot=0;

  /* Reset all TD slave configurations */
  for (islot=0;islot<nTD;islot++) {
    tdResetSlaveConfig(tdID[islot]);
  }

  dalmaClose();
}

/*
  Local Variables:
  compile-command: "make ts_sbs_list.so"
  End:
 */
