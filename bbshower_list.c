/*************************************************************************
 *
 *  bbshower_list.c - Library of routines for config and readout of
 *                    fadc250s and f1tdc-v1, using a JLab pipeline TI
 *                    as a source for trigger and syncreset.
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   1024*60      /* Size in Bytes */

#ifdef TI_MASTER
/* EXTernal trigger source (e.g. front panel ECL input), POLL for available data */
#define TI_READOUT TI_READOUT_EXT_POLL
#else
/* TS trigger source (e.g. fiber), POLL for available data */
#define TI_READOUT TI_READOUT_TS_POLL
#endif
#define TI_ADDR    (21<<19)          /* GEO slot 21 */

#define FIBER_LATENCY_OFFSET 0x10  /* measured longest fiber length */

#include "dmaBankTools.h"
#include "tiprimary_list.c" /* source required for CODA */
#include "sdLib.h"
#include "fadcLib.h"        /* library of FADC250 routines */
#include "fadc250Config.h"
#include "f1tdcLib.h"       /* library of f1tdc routines */

#define BUFFERLEVEL 1

/* FADC Library Variables */
extern int fadcA32Base, nfadc;
#define NFADC     4
/* Address of first fADC250 */
#define FADC_ADDR (3<<19)
/* Increment address to find next fADC250 */
#define FADC_INCR (1<<19)
#define FADC_BANK 0x3

#define FADC_READ_CONF_FILE {			\
    fadc250Config("");				\
    if(rol->usrConfig)				\
      fadc250Config(rol->usrConfig);		\
  }

/* F1TDC Specifics */
extern int f1tdcA32Base;
#define F1_ADDR  (2<<19)

/* for the calculation of maximum data words in the block transfer */
unsigned int MAXFADCWORDS=0;

/* SD variables */
static unsigned int sdScanMask = 0;

/*
  enable triggers (random or fixed rate) from internal pulser
 */
//#define INTRANDOMPULSER
/* #define INTFIXEDPULSER */


/* function prototype */
void rocTrigger(int arg);

void
rocDownload()
{
  unsigned short iflag;
  int ifa, stat;

#ifdef TI_MASTER
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
#if (defined (INTFIXEDPULSER) | defined(INTRANDOMPULSER))
  tiSetTriggerSource(TI_TRIGGER_PULSER); /* TS Inputs enabled */
#else
  tiSetTriggerSource(TI_TRIGGER_TSINPUTS); /* TS Inputs enabled */
#endif

  /* Enable set specific TS input bits (1-6) */
  tiEnableTSInput( TI_TSINPUT_1 | TI_TSINPUT_2 );

  /* Load the trigger table that associates
   *    - TS#1,2,3,4,5,6 : Physics trigger,
   */
  tiLoadTriggerTable(3);

  tiSetTriggerHoldoff(1,10,0);
  tiSetTriggerHoldoff(2,10,0);

  /* Set the SyncReset width to 4 microSeconds */
  tiSetSyncResetType(1);

  /* Set initial number of events per block */
  tiSetBlockLevel(blockLevel);

  /* Set Trigger Buffer Level */
  tiSetBlockBufferLevel(BUFFERLEVEL);

  /* Sync event every 1000 blocks */
  tiSetSyncEventInterval(1000);

  /* Set L1A prescale ... rate/(x+1) */
  tiSetPrescale(0);

  /* Set TS input #1 prescale rate/(2^(x-1) + 1)*/
  tiSetInputPrescale(1, 0);

  /* Add trigger latch pattern to datastream */
  tiSetFPInputReadout(1);
#endif

  /* Init the SD library so we can get status info */
  sdScanMask = 0;
  stat = sdInit(0);
  if (stat != OK)
    {
      printf("%s: WARNING: sdInit() returned %d\n",__func__, stat);
      daLogMsg("ERROR","SD not found");
    }

  /*****************
   *   FADC SETUP
   *****************/

  /* FADC Initialization flags */
  iflag = 0; /* NO SDC */
  iflag |= (1<<0);  /* VXS sync-reset */
  iflag |= FA_INIT_VXS_TRIG;  /* VXS trigger source */
  iflag |= FA_INIT_VXS_CLKSRC;  /* VXS 250MHz Clock source */

  fadcA32Base = 0x09000000;

  vmeSetQuietFlag(1);
  faInit(FADC_ADDR, FADC_INCR, NFADC, iflag);
  vmeSetQuietFlag(0);

  /* Just one FADC250 */
  if(nfadc == 1)
    faDisableMultiBlock();
  else
    faEnableMultiBlock(1);

  /* configure all modules based on config file */
  FADC_READ_CONF_FILE;

  for(ifa = 0; ifa < nfadc; ifa++)
    {
      /* Bus errors to terminate block transfers (preferred) */
      faEnableBusError(faSlot(ifa));

      /*trigger-related*/
      faResetMGT(faSlot(ifa),1);
      faSetTrigOut(faSlot(ifa), 7);

      /* Enable busy output when too many events are being processed */
      faSetTriggerBusyCondition(faSlot(ifa), 3);
    }

  sdSetActiveVmeSlots(faScanMask()); /* Tell the sd where to find the fadcs */

  f1tdcA32Base = 0x08800000;
  /* Setup the F1TDC */
  f1ConfigReadFile("/home/sbs-onl/cfg/f1tdc/bbshower_f1tdc.cfg");

  iflag = 4;  /* read chip config from file */

  f1Init(F1_ADDR, 1<<19, NF1TDC, iflag);

  /* For multiple boards we enable multiblock, which also enables
     the bus error on the last board. When not in multiblock we can
     enable multiblock on the single board ourselves.
  */
  if(NF1TDC>1) {
    f1EnableMultiBlock();
  } else {
    f1GEnableBusError();
  }

  f1GStatus(0);

  f1GEnableData(F1_ALL_CHIPS); /* Enable data on all chips */
  printf("\n\n\n * * * * BEFORE Sending ReSync (to lock resolution) \n\n\n");
  f1GStatus(0);

  tiStatus(0);
  sdStatus(0);
  faGStatus(0);

  printf("rocDownload: User Download Executed\n");

}

void
rocPrestart()
{
  int ifa;

  /* Program/Init VME Modules Here */
  for(ifa=0; ifa < nfadc; ifa++)
    {
      faSoftReset(faSlot(ifa),0);
      faResetToken(faSlot(ifa));
      faResetTriggerCount(faSlot(ifa));
      faEnableSyncReset(faSlot(ifa));
    }

#ifdef TI_MASTER
  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);
#endif
  tiSetTriggerPulse(1,25,3,0); // delay is second argument in units of 16ns
  tiStatus(0);
  faGStatus(0);

  /* FIXME: Need a way to send syncreset to f1tdc here */
  f1GStatus(0);

  printf("rocPrestart: User Prestart Executed\n");

}

void
rocGo()
{
  int fadc_mode = 0;
  unsigned int pl=0, ptw=0, nsb=0, nsa=0, np=0;

  /* Get the current block level */
  blockLevel = tiGetCurrentBlockLevel();
  printf("%s: Current Block Level = %d\n",
	 __FUNCTION__,blockLevel);

  faGSetBlockLevel(blockLevel);

  /* Get the FADC mode and window size to determine max data size */
  faGetProcMode(faSlot(0), &fadc_mode, &pl, &ptw,
		&nsb, &nsa, &np);

  /* Set Max words from fadc (proc mode == 1 produces the most)
     nfadc * ( Block Header + Trailer + 2  # 2 possible filler words
               blockLevel * ( Event Header + Header2 + Timestamp1 + Timestamp2 +
	                      nchan * (Channel Header + (WindowSize / 2) )
             ) +
     scaler readout # 16 channels + header/trailer
   */
  MAXFADCWORDS = nfadc * (4 + blockLevel * (4 + 16 * (1 + (ptw / 2))) + 18);

  /*  Enable FADC */
  faGEnable(0, 0);

#ifdef TI_MASTER
#if (defined (INTFIXEDPULSER) | defined(INTRANDOMPULSER))
  printf("************************************************************\n");
  printf("%s: TI Configured for Internal Pulser Triggers\n",
	 __func__);
  printf("************************************************************\n");
#endif

  /* Example: How to start internal pulser trigger */
#ifdef INTRANDOMPULSER
  /* Enable Random at rate 500kHz/(2^7) = ~3.9kHz */
  tiSetRandomTrigger(1,0x7);
#elif defined (INTFIXEDPULSER)
  /*
    Enable fixed rate with period (ns)
    120 +30*700*(1024^0) = 21.1 us (~47.4 kHz)
     - arg2 = 0xffff - Continuous
     - arg2 < 0xffff = arg2 times
  */
  tiSoftTrig(1,0xffff,700,0);
#endif
#endif
  /* Interrupts/Polling enabled after conclusion of rocGo() */
}

void
rocEnd()
{

#ifdef TI_MASTER
  /* Example: How to stop internal pulser trigger */
#ifdef INTRANDOMPULSER
  /* Disable random trigger */
  tiDisableRandomTrigger();
#elif defined (INTFIXEDPULSER)
  /* Disable Fixed Rate trigger */
  tiSoftTrig(1,0,700,0);
#endif
#endif

  /* FADC Disable */
  faGDisable(0);

  /* FADC Event status - Is all data read out */
  faGStatus(0);

  /* F1TDC Event status - Is all data read out */
  f1GStatus(0);
  int islot = 0;
  for(islot = 0; islot<NF1TDC; islot++) {
    f1Reset(f1Slot(islot),0);
  }

  tiStatus(0);

  printf("rocEnd: Ended after %d events\n",tiGetIntCount());

}

void
rocTrigger(int arg)
{
  int ifa = 0, stat, nwords, dCnt;
  unsigned int datascan, scanmask;
  int roType = 2, roCount = 0, blockError = 0;

  roCount = tiGetIntCount();

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,5,1);

  dCnt = tiReadTriggerBlock(dma_dabufp);
  if(dCnt<=0)
    {
      printf("No data or error.  dCnt = %d\n",dCnt);
    }
  else
    {
      dma_dabufp += dCnt;
    }

  /* fADC250 Readout */
  BANKOPEN(FADC_BANK,BT_UI4,0);

  /* Mask of initialized modules */
  scanmask = faScanMask();
  /* Check scanmask for block ready up to 100 times */
  datascan = faGBlockReady(scanmask, 100);
  stat = (datascan == scanmask);

  if(stat)
    {
      if(nfadc == 1)
	roType = 1;   /* otherwise roType = 2   multiboard reaodut with token passing */
      nwords = faReadBlock(0, dma_dabufp, MAXFADCWORDS, roType);

      /* Check for ERROR in block read */
      blockError = faGetBlockError(1);

      if(blockError)
	{
	  printf("ERROR: Slot %d: in transfer (event = %d), nwords = 0x%x\n",
		 faSlot(ifa), roCount, nwords);

	  for(ifa = 0; ifa < nfadc; ifa++)
	    faResetToken(faSlot(ifa));

	  if(nwords > 0)
	    dma_dabufp += nwords;
	}
      else
	{
	  dma_dabufp += nwords;
	  faResetToken(faSlot(0));
	}
    }
  else
    {
      printf("ERROR: Event %d: Datascan != Scanmask  (0x%08x != 0x%08x)\n",
	     roCount, datascan, scanmask);
    }
  BANKCLOSE;


  /* Check for SYNC Event */
  if(tiGetSyncEventFlag() == 1)
    {
      /* Check for data available */
      int davail = tiBReady();
      if(davail > 0)
	{
	  printf("%s: ERROR: TI Data available (%d) after readout in SYNC event \n",
		 __func__, davail);

	  while(tiBReady())
	    {
	      vmeDmaFlush(tiGetAdr32());
	    }
	}

      for(ifa = 0; ifa < nfadc; ifa++)
	{
	  davail = faBready(faSlot(ifa));
	  if(davail > 0)
	    {
	      printf("%s: ERROR: fADC250 Data available (%d) after readout in SYNC event \n",
		     __func__, davail);

	      while(faBready(faSlot(ifa)))
		{
		  vmeDmaFlush(faGetA32(faSlot(ifa)));
		}
	    }
	}
    }

}

void
rocCleanup()
{

  printf("%s: Reset all FADCs\n",__FUNCTION__);
  faGReset(1);

#ifdef TI_MASTER
  tiResetSlaveConfig();
#endif

}

/*
  Local Variables:
  compile-command: "make -B bbshower_list.so bbshower_slave_list.so"
  End:
 */
