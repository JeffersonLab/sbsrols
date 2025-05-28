/*************************************************************************
 *
 *  ti_list.c - Library of routines for readout and buffering of
 *                     events using a JLAB Trigger Interface V3 (TI) with
 *                     a Linux VME controller in CODA 3.0.
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   1024*64*16      /* Size in Bytes */

/* TI_MASTER / TI_SLAVE defined in Makefile */

#ifdef TI_MASTER
/* EXTernal trigger source (e.g. front panel ECL input), POLL for available data */
#define TI_READOUT TI_READOUT_EXT_POLL
#else
/* TS trigger source (e.g. fiber), POLL for available data */
#define TI_READOUT TI_READOUT_TS_POLL
//#define TI_FLAG TI_INIT_SLAVE_FIBER_5


#endif
#define TI_ADDR  0 /* Auto initialize (search for TI by slot */

/* Measured longest fiber length in system */
#define FIBER_LATENCY_OFFSET 0x50

#include "dmaBankTools.h"   /* Macros for handling CODA banks */
#include "tiprimary_list.c" /* Source required for CODA readout lists using the TI */
#include "sdLib.h"
static unsigned int sdScanMask = 0;
#include "vfTDCLib.h"

/* Define initial blocklevel and buffering level */
#define BLOCKLEVEL 1
#define BUFFERLEVEL 10

/* Library to pipe stdout to daLogMsg */
#include "dalmaRolLib.h"

/*
  enable triggers (random or fixed rate) from internal pulser
 */
/*#define INTRANDOMPULSER*/
/* #define INTFIXEDPULSER */


/*
  Global to configure the trigger source
      0 : tsinputs
      1 : TI random pulser
      2 : TI fixed pulser

  Set with rocSetTriggerSource(int source);*/

static unsigned int vetrocSlotMask=0;


int rocTriggerSource = 0;
int multiboard_read=0;

void rocSetTriggerSource(int source); // routine prototype




/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  int stat;
  int itdc = 0, roFlag = 1;
  extern int nvfTDC;
  dalmaInit(1);
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
#ifdef TI_MASTER
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
  tiEnableTSInput( TI_TSINPUT_1 | TI_TSINPUT_2 | TI_TSINPUT_6);

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
#endif
  tiSetSlavePort(1);
  /* Init the SD library so we can get status info */
  stat = sdInit(0);
  if(stat==0)
    {
      sdSetActiveVmeSlots(0);
      sdStatus(0);
    }

  /*************************************************************/
  /* VFTDC initialization                                      */
  /*************************************************************/
  extern unsigned int vfTDCA32Base;

  vfTDCA32Base=0x09000000;
  //
  // Looking at ~/linuxvme/vfTDC/vfTDCLib.c, the way that the following statement works
  // is that the first argument is the first slot to read out (shifted 19 bits), the
  // second argument is the INCREMENT between TDC slots, and the
  // third argument is the total number of TDCs to read out.
  //
  // vfTDCInit(8<<19, 11<<19, 2, ...) will start at slot 8, increment by 11, and read 2 TDCs.  So, Slot 8 and Slot 19
  //
  // vfTDCInit(3<<19, 1<<19, 17, ...) will start at slot 3, increment by 1, and read 17 TDCs.  So, Slots 3 to 19, inclusive.
  //
  vfTDCInit(3<<19, 1<<19, 18,
	    VFTDC_INIT_VXS_SYNCRESET |
	    VFTDC_INIT_VXS_TRIG      |
	    VFTDC_INIT_VXS_CLKSRC);


  int window_width   = 13; /* 13 = 13*4ns = 52 ns */
  int window_latency = 804; /* 804 = 804*4ns =  3216 ns = 3.216 us */

  // Calling vfTDCSetWindoParamters with first argument 0 will loop through all TDC's that have been initialized
  vfTDCSetWindowParamters(0, window_latency, window_width);

  // Note:  the call above to vfTDCInit will initialize the TDCs, and within that, nvfTDC will get incremented
  //  for each slot.  So, now we can use that as a loop termination variable!
  //
  for(itdc = 0; itdc < nvfTDC; itdc++)
    {
      vfTDCSetEdgeReadout(vfTDCSlot(itdc),1,1);
      vetrocSlotMask |= (1<<vfTDCSlot(itdc)); /* Add it to the mask */
      //vfTDCSetWindowParamters(itdc, window_latency, window_width);
      vfTDCStatus(vfTDCSlot(itdc),0);
      vfTDCResetToken(vfTDCSlot(itdc));
    }

  int ref_window_width = 50; // define a window width of 200ns  for readout of Slot 20, reference TDC
  int ref_window_latency = 25; // define a latency of 100 ns for Slot 20
  vfTDCSetWindowParamters(20, ref_window_latency, ref_window_width);
  vfTDCGStatus(0);

  printf("vetrocSlotMask=0x%08x\n", vetrocSlotMask);

  sdScanMask |= vetrocSlotMask;
  sdSetActiveVmeSlots(sdScanMask);
  sdSetBusyVmeSlots(sdScanMask, 0);

  tiStatus(0);

  sdStatus(0);

  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{

  DALMAGO;
  vfTDCGStatus(0);
  tiStatus(0);
  DALMASTOP;

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

  int bufferLevel = 0;
  /* Get the current buffering settings (blockLevel, bufferLevel) */
  blockLevel = tiGetCurrentBlockLevel();
  bufferLevel = tiGetBroadcastBlockBufferLevel();
  printf("%s: Block Level = %d,  Buffer Level (broadcasted) = %d (%d)\n",
	 __func__,
	 blockLevel,
	 tiGetBlockBufferLevel(),
	 bufferLevel);

#ifdef TI_SLAVE
  /* In case of slave, set TI busy to be enabled for full buffer level */

  /* Check first for valid blockLevel and bufferLevel */
  if((bufferLevel > 10) || (blockLevel > 1))
    {
      daLogMsg("ERROR","Invalid blockLevel / bufferLevel received: %d / %d",
	       blockLevel, bufferLevel);
      tiUseBroadcastBufferLevel(0);
      tiSetBlockBufferLevel(1);

      /* Cannot help the TI blockLevel with the current library.
	 modules can be spared, though
      */
      blockLevel = 1;
    }
  else
    {
      tiUseBroadcastBufferLevel(1);
    }
#endif

  /* Enable/Set Block Level on modules, if needed, here */


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

  DALMAGO;
  vfTDCGStatus(0);
  tiStatus(0);
  DALMASTOP;

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int ii,slot;
  int itdc = 0, roFlag = 1;
  uint32_t maxdata = 0;
  extern int nvfTDC;

  int stat, dCnt, len=0, idata, blkReady=0,timeout=0;
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

  /* EXAMPLE: How to open a bank (name=5, type=ui4) and add data words by hand */
  BANKOPEN(5,BT_UI4,blockLevel);
  *dma_dabufp++ = tiGetIntCount();
  *dma_dabufp++ = 0xdead;
  *dma_dabufp++ = 0xcebaf111;
  *dma_dabufp++ = 0xcebaf222;
  BANKCLOSE;

  /* Readout vfTDC data */
  BANKOPEN(9,BT_UI4,0);

  /* e.g. Max number of words = Blocklevel * (10 hits per channel + 10 other words) */
  maxdata = (10*1024)>>2;

  if(multiboard_read==1)
    {
      maxdata *= nvfTDC;
      roFlag = 2;
    }

  uint32_t scan_mask = vfTDCScanMask();
  uint32_t slots_ready = vfTDCGBlockReady(scan_mask, 100);

  if(slots_ready != scan_mask)
    {
      printf("VFTDC slots_ready != scan_mask (0x%08x != 0x%08x)\n",
	     slots_ready, scan_mask);
    }
  else
    {
      if(multiboard_read == 1)
	vfTDCResetToken(0);

      for(itdc = 0; itdc < nvfTDC; itdc++)
	{
	  dCnt = vfTDCReadBlock(vfTDCSlot(itdc), dma_dabufp, maxdata, roFlag);
	  if(dCnt<=0)
	    {
	      printf("vfTDCReadBlock: ERROR: No data or error.  dCnt = %d\n",dCnt);
	    }
	  else
	    {
	      dma_dabufp += dCnt;
	    }
	  if(roFlag == 2)
	    break;
	}
    }
  BANKCLOSE;


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
#ifdef TI_MASTER
  tiResetSlaveConfig();
#endif
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



/*
  Local Variables:
  compile-command: "make -k -B ti_list.so ti_slave_list.so"
  End:
 */
