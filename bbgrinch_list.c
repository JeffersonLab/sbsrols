/*************************************************************************
 *
 *  bbgrinch_list.c - GRINCH readout list
 *             Configure: 4 VETROC, 2 c792s, TI, SD
 *             Readout:   4 VETROC, 2 c792s, TI
 *
 *     TI delivers accepted Triggers, Clocks, and SyncReset to
 *       SD -> VETROC
 *
 *     SD combines BUSY from VETROC and sends to TI (SWB_BUSY)
 *
 *************************************************************************/

/* Define initial blocklevel and buffering level */
#define BLOCKLEVEL  1
#define BUFFERLEVEL 1

/* Event Buffer definitions */
#define MAX_EVENT_LENGTH 5*10240 /* in Bytes */
#define MAX_EVENT_POOL   400   /* in number of events */

/* TI_MASTER / TI_SLAVE defined in Makefile */

#ifdef TI_MASTER
/* EXTernal trigger source (e.g. front panel ECL input), POLL for available data */
#define TI_READOUT TI_READOUT_EXT_POLL
#else
/* TS trigger source (e.g. fiber), POLL for available data */
#define TI_READOUT TI_READOUT_TS_POLL
#endif
#define TI_ADDR  0 /* Auto initialize (search for TI by slot */

/* Measured longest fiber length in system */
#define FIBER_LATENCY_OFFSET 0x50

/* Include */
#include "dmaBankTools.h"   /* Macros for handling CODA banks */
#include "tiprimary_list.c" /* Source required for CODA readout lists using the TI */

/* DMA config definitions */
#define A24DMA     1,2,0
#define A24DMA2    1,5,2
#define A32DMA1    2,5,1
#define A32DMA2    2,5,2

/* SD variables */
#include "sdLib.h"
static unsigned int sdScanMask = 0;

/* VETROC variables */
#include "vetrocLib.h"      /* VETROC library */
#include "vetrocConfig.h"
#define USE_VETROC
#define MAXVETROCDATA 1200*BLOCKLEVEL
#define VETROC_SLOT 3				/* slot of first vetroc */
#define VETROC_SLOT_INCR 1			/* slot increment */
#define NVETROC	4								/* number of vetrocs used */
#define VETROC_ROMODE 1  /* Readout Mode: 0 = SCT, 1 = Single Board DMA, 2 = MultiBoard DMA */
#define VETROC_READ_CONF_FILE {			\
    vetrocConfig("");				\
    if(rol->usrConfig)				\
      vetrocConfig(rol->usrConfig);		\
  }
#define VETROC_BANK 526

static unsigned int vetrocSlotMask=0;
int nvetroc=0;		// number of vetrocs in the crate
extern int vetrocA32Base;                      /* Minimum VME A32 Address for use by VETROCs */

/* c792 variables */
#include "c792Lib.h"        /* v792 library */
#define MAX_ADC_DATA 44
#define use792      0
#define V792_NMOD   2
// V792_ADR1 is the HW address for the leftmost v792 in the crate
#define V792_ADR1   0x11510000 //was  0x11410000 (CA)
// V792_OFF is the increment for each subsequent v792 (ie. next module is 0x180000)
#define V792_OFF    0x00100000
#define C792_BANKID 792
extern int Nc792;

#include <time.h> /* CARLOS STUFF FOR DEBUGGING PURPOSES */

/*
  enable triggers (random or fixed rate) from internal pulser
 */
/* #define INTRANDOMPULSER */
/* #define INTFIXEDPULSER */

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  int ifa, stat;
  unsigned short faflag;

  /* Define BLock Level */

  blockLevel = BLOCKLEVEL;

  /*****************
   *   TI SETUP
   *****************/
#ifdef TI_MASTER
  /*
   * Set Trigger source
   *    For the TI-Master, valid sources:
   *      TI_TRIGGER_FPTRG     2  Front Panel "TRG" Input
   *      TI_TRIGGER_TSINPUTS  3  Front Panel "TS" Inputs
   *      TI_TRIGGER_TSREV2    4  Ribbon cable from Legacy TS module
   *      TI_TRIGGER_PULSER    5  TI Internal Pulser (Fixed rate and/or random)
   */
#ifdef INTRANDOMPULSER
  tiSetTriggerSource(TI_TRIGGER_PULSER);
#else
  tiSetTriggerSource(TI_TRIGGER_TSINPUTS);  //TS Inputs trigger;
#endif

  /* Enable set specific TS input bits (1-6) */
  tiEnableTSInput(TI_TSINPUT_1|TI_TSINPUT_2|TI_TSINPUT_3|TI_TSINPUT_4|TI_TSINPUT_5);

  tiSetTriggerLatchOnLevel(1);

  /* Load the trigger table that associates
   *  pins 21/22 | 23/24 | 25/26 : trigger1
   *  pins 29/30 | 31/32 | 33/34 : trigger2
   */
  tiLoadTriggerTable(0);

  // Lower deadtime trigger rules
  tiSetTriggerHoldoff(1,31,0);	// no more than 1 triggers in 31*16ns  -
  tiSetTriggerHoldoff(4,3,1);	// no more than 4 triggers in 3*3840ns
#endif
  tiFakeTriggerBankOnError(0);
  tiSetFPInputReadout(1);

  /* Set initial number of events per block */
  tiSetBlockLevel(blockLevel);

  /* Set Trigger Buffer Level */
  tiSetBlockBufferLevel(BUFFERLEVEL);

  /* BR: enable busy when buffer level is exceeded */
//  tiBusyOnBufferLevel(1);

  /* Init the SD library so we can get status info */
  sdScanMask = 0;
  stat = sdInit(0);
  if (stat != OK)
    {
      printf("%s: WARNING: sdInit() returned %d\n",__func__, stat);
    }
  else
    {
      printf("Will try to use SD in Switch Slot\n");
      sdSetActiveVmeSlots(0);	// clear active slots
    }

  sdStatus(0);

  if(use792)
    {
      c792Init(V792_ADR1,V792_OFF,V792_NMOD,0);
      int iadc;
      for(iadc = 0; iadc < Nc792; iadc++)
	{
	  c792SetGeoAddress(iadc, iadc+17);
	}
      c792GStatus(0);
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
  int ivt;
  unsigned short vtflag;

  /*****************
   *   VETROC SETUP
   *****************/
#ifdef USE_VETROC

  /* 0 = software synch-reset, FP input 1, internal clock */
  //	vtflag = 0x20;  /* FP 1  0x020;  MAY NEED TO BE CHANGED*/
  vtflag = 0x111; /* vxs sync-reset, trigger, clock */

  vetrocA32Base = 0x09000000;
  nvetroc = vetrocInit((VETROC_SLOT<<19),(VETROC_SLOT_INCR<<19) , NVETROC, vtflag);
  if (nvetroc <= 0) {
    printf("ERROR: no VETROC !!! \n");
  }

  for(ivt=0; ivt<nvetroc; ivt++)
    {
      vetrocSlotMask |= (1<<vetrocSlot(ivt)); /* Add it to the mask */
    }
  printf("vetrocSlotMask=0x%08x\n", vetrocSlotMask);

  sdScanMask |= vetrocScanMask();
  sdSetActiveVmeSlots(sdScanMask); /* Tell the sd where to find the vetrocs */

  for(ivt=0; ivt<nvetroc; ivt++)
    {
      vetrocLinkReset(vetrocSlot(ivt));
      vetrocClear(vetrocSlot(ivt));
    }

  /* configure all modules based on config file */
  VETROC_READ_CONF_FILE;


#if(VETROC_ROMODE==2)
  vetrocEnableMultiBlock();
#endif

#endif
  /* Print status for all boards */
#ifdef USE_VETROC
  vetrocGStatus(0);
#endif

  /* Setup ADCs (no sparcification, enable berr for block reads) */
  if(use792)
    {
      int iadc;
      for(iadc = 0; iadc < Nc792; iadc++)
	{
	  c792Sparse(iadc,0,0);
	  c792Clear(iadc);
	  c792EnableBerr(iadc);
      c792BitSet2(iadc, 1<<14);
      c792EventCounterReset(iadc);
	}
      c792GStatus(0);
    }

#ifdef TI_MASTER
  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);
#endif

  sdStatus(0);
  tiStatus(1);

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

  /* Get the current Block Level */
  blockLevel = tiGetCurrentBlockLevel();
  printf("rocGo: Block Level set to %d\n",blockLevel);

#ifdef USE_VETROC
  vetrocGSetBlockLevel(blockLevel);
#endif

  if(use792)
    {
      int iadc;
      for(iadc = 0; iadc < Nc792; iadc++)
	c792Enable(iadc);
    }

  /* Interrupts/Polling enabled after conclusion of rocGo() */

#ifdef TI_MASTER
  /* Example: How to start internal pulser trigger */
#ifdef INTRANDOMPULSER
  /* Enable Random at rate 500kHz/(2^N): N=7: ~3.9kHz, N=3: ~62kHz  */
  tiSetRandomTrigger(1,0xf);
#elif defined (INTFIXEDPULSER)
  /* Enable fixed rate with period (ns) 120 +30*700*(1024^0) = 21.1 us (~47.4 kHz)
     - Generated 1000 times */
  tiSoftTrig(1,1000,700,0);
#endif
#endif

  tiSetBlockLimit(0);

  tiStatus(1);
}

/****************************************
 *  END
 ****************************************/
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

  /* Print status for all boards */
#ifdef USE_VETROC
  vetrocGStatus(0);
#endif

  if(use792)
    {
      int iadc;
      for(iadc = 0; iadc < Nc792; iadc++)
	{
	  c792Disable(iadc);
	}
      c792GStatus(0);
    }

  sdStatus(0);
  tiStatus(1);

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int ii, gbready, itime, read_stat, stat;
  int ivt = 0, ifa, nwords_fa, nwords_vt, blockError, dCnt;
  unsigned int val;
  unsigned int datascan, scanmask, roCount;

  /* Set TI output 1 high for diagnostics */
  tiSetOutputPort(1,0,0,0);

  roCount = tiGetIntCount(); //Get the TI trigger count

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */

  vmeDmaConfig(2,5,1);


  /* Readout the trigger block from the TI
     Trigger Block MUST be reaodut first */

  dCnt = tiReadTriggerBlock(dma_dabufp);

  if(dCnt<=0)
    {
      printf("%d: No TI Trigger data or error.  dCnt = %d\n",roCount,dCnt);
      tiSetBlockLimit(1);
    }
  else
    { /* TI Data is already in a bank structure.  Bump the pointer */
      dma_dabufp += dCnt;
    }

#ifdef USE_VETROC
  /* Bank for VETROC data */
  BANKOPEN(VETROC_BANK,BT_UI4,0);
  dCnt = 0;

  /* Check for valid data in VETROC */
  read_stat = 0;

  for(itime=0; itime<1000; itime++)
    {
      gbready = vetrocGBready();
      read_stat = (gbready == vetrocSlotMask);

      if (read_stat>0)
	{
	  break;
	}
    }

  if(read_stat>0)
    { /* read the data here */
#if(VETROC_ROMODE==2)
      ivt = 0;
#else
      for(ivt=0; ivt<nvetroc; ivt++)
#endif
	{
	  /* skip 1 word so nwords_vt is written before buffer to keep same format as before */
	  nwords_vt = vetrocReadBlock(vetrocSlot(ivt), dma_dabufp + 1,
				      MAXVETROCDATA, VETROC_ROMODE);
	  *dma_dabufp++ = LSWAP(nwords_vt);

          dma_dabufp+= nwords_vt;
	}
    }
  else
    {
      printf("Missed VETROC event data: gbready=0x%08X, vetrocSlotMask=0x%08X\n", gbready, vetrocSlotMask);
      fflush(stdout);
      vetrocGStatus(1);
      tiStatus(1);
      tiSetBlockLimit(1);
    }
  BANKCLOSE;
#endif

  if(use792)
    {
      unsigned int scanmask = 0, datascan = 0;
      int iadc;

      /* Check if an Event is available */
      scanmask = c792ScanMask();
      datascan = c792GDReady(scanmask, 1000);

      BANKOPEN(C792_BANKID,BT_UI4,blockLevel);
      if(datascan==scanmask)
	{
	  for(iadc = 0; iadc < Nc792; iadc++)
	    {
	      vmeDmaConfig(2,3,0);
	      dCnt = c792ReadBlock(iadc,dma_dabufp,MAX_ADC_DATA+40);
	      if(dCnt <= 0)
		{
		  logMsg("%4d: ERROR: ADC %2d Read Failed - Status 0x%x\n",
			 tiGetIntCount(),
			 iadc, dCnt,0,0,0,0);
		  *dma_dabufp++ = LSWAP(iadc);
		  *dma_dabufp++ = LSWAP(0xda00bad1);
		  c792Clear(iadc);
		}
	      else
		{
		  dma_dabufp += dCnt;
		}
	    }

	}
      else
	{
	  logMsg("%4d: ERROR: datascan != scanmask for ADC  (0x%08x != 0x%08x)\n",
		 tiGetIntCount(),
		 datascan,scanmask,0,0,0,0);
	  *dma_dabufp++ = LSWAP(datascan);
	  *dma_dabufp++ = LSWAP(scanmask);
	  *dma_dabufp++ = LSWAP(0xda00bad2);

	  for(iadc = 0; iadc < Nc792; iadc++)
	    c792Clear(iadc);
	}
      BANKCLOSE;
    }

  /* Set TI output 0 low */
  tiSetOutputPort(0,0,0,0);

}

void
rocCleanup()
{
#ifdef TI_MASTER
  tiResetSlaveConfig();
#endif
}

/*
  Local Variables:
  compile-command: "make -k bbgrinch_list.so bbgrinch_slave_list.so"
  End:
 */
