/*************************************************************************
 *
 *  bbhodo_list.c - Readout list for bigbite hodoscope
 *
 *     Readout 2 Caen 792 ADCs
 *             2 Caen 1190 TDCs
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     400
#define MAX_EVENT_LENGTH   1024*10      /* Size in Bytes */

#ifdef TI_MASTER
/* EXTernal trigger source (e.g. front panel ECL input), POLL for available data */
#define TI_READOUT TI_READOUT_EXT_POLL
#else
/* TS trigger source (e.g. fiber), POLL for available data */
#define TI_READOUT TI_READOUT_TS_POLL
#endif

/* TI VME address, or 0 for Auto Initialize (search for TI by slot) */
/* GEO slot 21 */
#define TI_ADDR    (21 << 19)

/* Measured longest fiber length in system */
#define FIBER_LATENCY_OFFSET 0x50

#include "dmaBankTools.h"   /* Macros for handling CODA banks */
#include "tiprimary_list.c" /* Source required for CODA readout lists using the TI */
#include "c792Lib.h"
#include "c1190Lib.h"

/* Define initial blocklevel and buffering level */
#define BLOCKLEVEL 1
#define BUFFERLEVEL 1

/*
  enable triggers (random or fixed rate) from internal pulser
 */
/* #define INTRANDOMPULSER */
/* #define INTFIXEDPULSER */

/* CAEN 792 specific definitions */
#define C792_BANKID 792
#define ADC_ID 0
#define MAX_ADC_DATA 34
extern int Nc792;


/* CAEN 1190/1290 specific definitions */
#define NUM_V1190 2
#define C1190_BANKID 1190
// 0: CBLT   1: LL-DMA
#define C1190_ROMODE C1190_CBLT

#define useTDC_SCALERS 1
int scaler_period=2;
struct timespec last_time;
#include "../scaler_server/scale32LibNew.c"
#include "../scaler_server/linuxScalerLib.c"

/* function prototype */
void rocTrigger(int arg);

void
rocDownload()
{
  // FIXME: This is debug stuff.  remove when satisfied.
  vmeSetQuietFlag(0);


  /* Define BLock Level */
  blockLevel = BLOCKLEVEL;

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
  tiSetTriggerSource(TI_TRIGGER_PULSER); /* Pulser enabled */
#else
  tiSetTriggerSource(TI_TRIGGER_FPTRG); /* Front Panel TRG enabled */
#endif

  /* Enable set specific TS input bits (1-6) */
  tiEnableTSInput( TI_TSINPUT_1 | TI_TSINPUT_2 );

  /* Load the trigger table that associates
   *  pins 21/22 | 23/24 | 25/26 : trigger1
   *  pins 29/30 | 31/32 | 33/34 : trigger2
   */
  tiLoadTriggerTable(0);

  tiSetTriggerHoldoff(1,10,0);
  tiSetTriggerHoldoff(2,10,0);

  /* Set initial number of events per block */
  tiSetBlockLevel(blockLevel);

  /* Set Trigger Buffer Level */
  tiSetBlockBufferLevel(BUFFERLEVEL);

  tiSetSyncEventInterval(1000);

  /* Set prompt trigger width to 100ns = (23 + 2) * 4ns */
  tiSetPromptTriggerWidth(23);
#endif

  tiStatus(0);

  c792Init(0x200000,0x80000,2,0);
  int iadc;
  for(iadc = 0; iadc < Nc792; iadc++)
    {
      c792SetGeoAddress(iadc, iadc+4);
    }

  if(useTDC_SCALERS > 0) {
    if(fadcscaler_init_crl()) {
      printf("Scalers initialized\n");
    } else {
      printf("Failed to initialize scalers\n");
    }
    set_runstatus(0);
  }

  printf("rocDownload: User Download Executed\n");

}

void
rocPrestart()
{

  /* Suspend scaler task */
  if(useTDC_SCALERS > 0) {
    set_runstatus(1);
    register_tdc_info(450,BLOCKLEVEL);
    clock_gettime(CLOCK_REALTIME, &last_time);
  }

  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);

  tiStatus(0);

  /* Program/Init VME Modules Here */
  /* Setup ADCs (no sparcification, enable berr for block reads) */
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

  int itdc;

  /* INIT C1190/C1290 - Must be A32 for 2eSST */
  UINT32 list[NUM_V1190] = {0x100000,0x180000};

  tdc1190InitList(list,NUM_V1190,2);

  unsigned int mcstaddr = 0x0a000000;
  if(C1190_ROMODE==0)  tdc1190InitMCST(mcstaddr);
  tdc1190GSetTriggerMatchingMode();
  tdc1190GSetEdgeResolution(100);
  tdc1190GSetEdgeDetectionConfig(3);
  tdc1190GSetWindowWidth(550); // ns
 tdc1190GSetWindowOffset(-600); // ns
 tdc1190GEnableTriggerTimeSubtraction(); // Uses the beginning of the match window instead of the latest bunch reset

 tdc1190GTriggerTime(1); // flag = 1 enable (= 0 disable) writing out of the Extended Trigger Time Tag in the output buffer.
tdc1190ConfigureGReadout(C1190_ROMODE);
  for(itdc=0; itdc<NUM_V1190; itdc++)
    {
      tdc1190SetTriggerMatchingMode(itdc);
      tdc1190SetEdgeResolution(itdc,100);
      tdc1190SetEdgeDetectionConfig(itdc,3);
  tdc1190SetWindowWidth(itdc,450); // ns
 tdc1190SetWindowOffset(itdc,-500); // ns
     tdc1190SetGeoAddress(itdc, list[itdc] >> 19);
    }

  tdc1190GStatus(1);

  printf("rocPrestart: User Prestart Executed\n");

}

void
rocGo()
{
  tiSetBlockLimit(0);
  /* Print out the Run Number and Run Type (config id) */
  printf("rocGo: Activating Run Number %d, Config id = %d\n",
	 rol->runNumber,rol->runType);

  /* Get the current Block Level */
  blockLevel = tiGetCurrentBlockLevel();
  printf("rocGo: Block Level set to %d\n",blockLevel);

  /* Enable/Set Block Level on modules, if needed, here */
  int iadc;
  for(iadc = 0; iadc < Nc792; iadc++)
    c792Enable(iadc);


  tdc1190GSetBLTEventNumber(blockLevel);

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

  if (useTDC_SCALERS>0) {
    clear_scalers();
    printf("tdc scalers cleared\n");
    enable_scalers();
  }

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

  int iadc;
  for(iadc = 0; iadc < Nc792; iadc++)
    {
      c792Disable(iadc);
    }
  c792GStatus(0);

  tdc1190GStatus(0);

  tiStatus(0);

  if(useTDC_SCALERS) {
    disable_scalers();
    set_runstatus(0);		/* Tell Stand alone scaler task to resume  */
  }

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());

}

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

  unsigned int scanmask = 0, datascan = 0;

  BANKOPEN(C1190_BANKID,BT_UI4,blockLevel);
  /* Check for valid data here */
  datascan = tdc1190GBlockReady(scanmask, BLOCKLEVEL, 1000);

  if(datascan==scanmask)
    {
      /* Get the TDC data from all modules... rflag=2 for Linked List DMA
	 "64" is ignored in Linux */
#if(C1190_ROMODE==C1190_CBLT)
      vmeDmaConfig(2,3,2);
      dCnt = tdc1190CBLTReadBlock(0,dma_dabufp,2000,2);
#else
      vmeDmaConfig(2,3,2);
      dCnt = tdc1190ReadBlock(0,dma_dabufp,64000,2|(BLOCKLEVEL<<8));
#endif

      if(dCnt < 0)
	{
	  printf("ERROR: in c1190 transfer (event = %d), status = 0x%x\n",
		 tiGetIntCount(),dCnt);
	  *dma_dabufp++ = LSWAP(0xda000bad);
	}
      else
	{
	  if(useTDC_SCALERS) {
	    struct timespec now;
	    accumulate_tdc_scalers(dCnt, dma_dabufp);
	    clock_gettime(CLOCK_REALTIME, &now);
	    if( (now.tv_sec - last_time.tv_sec
		 + ((double)now.tv_nsec - (double)last_time.tv_nsec)/1000000000L)
		>= scaler_period) {
	      read_tdc_scalers(0,0); /* Update scaler server */
	      read_ti_scalers(0,0);
	      last_time = now;
	      read_clock_channels(); /* Make sure clock updates */
	    }
	  }
	  dma_dabufp += dCnt;
	}
    }
  else
    {
      printf("ERROR: Data not ready in event %d.. 0x%08x != 0x%08x\n",
	     tiGetIntCount(), datascan, scanmask);
      *dma_dabufp++ = LSWAP(datascan);
      *dma_dabufp++ = LSWAP(scanmask);
      *dma_dabufp++ = LSWAP(0xda000bad);
    }
  BANKCLOSE;

  int iadc;

  vmeDmaConfig(1,2,0);

  /* Check if an Event is available */
  scanmask = c792ScanMask();
  datascan = c792GDReady(scanmask, 1000);

  BANKOPEN(C792_BANKID,BT_UI4,blockLevel);
  if(datascan==scanmask)
    {
      for(iadc = 0; iadc < Nc792; iadc++)
	{
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

  /* Check for SYNC Event or Bufferlevel == 1 */
  if((tiGetSyncEventFlag() == 1) || (tiGetBlockBufferLevel() == 1))
    {
      /* Check for data available */
      int davail = tiBReady();
      if(davail > 0)
	{
	  printf("%4d: ERROR: TI Data available (%d) after readout! \n",
		 tiGetIntCount(), davail);

	  while(tiBReady())
	    {
	      vmeDmaFlush(tiGetAdr32());
	    }
	}

      /* Check for ANY data in 1190 TDCs */
      datascan = tdc1190GDready(1);
      if(datascan > 0)
	{
	  printf("%4d: ERROR: C1190 Data available (0x%x) after readout!\n",
		 tiGetIntCount(), datascan);
	  int itdc;
	  for(itdc = 0; itdc < 32; itdc++)
	    {
	      if((1 << itdc) & datascan)
		{
		  tdc1190Clear(itdc);
		}
	    }
	}

      /* Check for ANY data in 792 ADCs */
      for(iadc = 0; iadc < Nc792; iadc++)
	{
	  datascan = c792Dready(iadc);
	  if(datascan)
	    {
	      printf("%4d: ERROR: C792 Data availble (%d) after readout!\n",
		     tiGetIntCount(), datascan);
	      c792Clear(iadc);
	    }

	}

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
  compile-command: "make -k -B bbhodo_list.so bbhodo_slave_list.so"
  End:
 */
