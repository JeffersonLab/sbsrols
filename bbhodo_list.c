/*************************************************************************
 *
 *  bbhodo_list.c - Readout list for bigbite hodoscope
 *
 *             6 FADC
 *             2 Caen 1190 TDCs
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     100
#define MAX_EVENT_LENGTH   1024*240      /* Size in Bytes */

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
#include "c1190Lib.h"

/* Library to pipe stdout to daLogMsg */
#include "dalmaRolLib.h"

/* Define initial blocklevel and buffering level */
#define BLOCKLEVEL 1
#define BUFFERLEVEL 1

#define USE_FADC 1
#ifdef USE_FADC
/* FADC Library Variables */
#include "fadcLib.h"        /* library of FADC250 routines */
#include "fadc250Config.h"
extern int nfadc;
extern unsigned int  fadcA32Base;
#define NFADC     6
/* Address of first fADC250 */
#define FADC_ADDR (14<<19)
/* Increment address to find next fADC250 */
#define FADC_INCR (1<<19)
#define FADC_BANK 250

#define FADC_READ_CONF_FILE {			\
    fadc250Config("/home/sbs-onl/cfg/intelbbhodo.cnf");	\
    if(rol->usrConfig)				\
      fadc250Config(rol->usrConfig);		\
  }
/* for the calculation of maximum data words in the block transfer */
unsigned int MAXFADCWORDS=0;
#endif /* USE_FADC */

/* CAEN 1190/1290 specific definitions */
#define NUM_V1190 2
#define C1190_BANKID 1190
// 0: CBLT   1: LL-DMA
#define C1190_ROMODE C1190_CBLT

/* Clear counter at 1190 syncevent */
unsigned int c1190ClearCounter = 0;

/*
  Global to configure the trigger source
      0 : tsinputs
      1 : TI random pulser
      2 : TI fixed pulser

  Set with rocSetTriggerSource(int source);
*/
int rocTriggerSource = 0;
void rocSetTriggerSource(int source); // routine prototype

void
rocDownload()
{
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

  if(rocTriggerSource == 0)
    {
      tiSetTriggerSource(TI_TRIGGER_FPTRG); /* FP TRG enabled */
    }
  else
    {
      tiSetTriggerSource(TI_TRIGGER_PULSER); /* Internal Pulser */
    }

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

#endif
  /* Set prompt trigger width to 100ns = (23 + 2) * 4ns */
  tiSetPromptTriggerWidth(23);
  /*
0: SWA
1: SWB
2: P2
3: FP-FTDC
4: FP-FADC
5: FP
6: Unused
7: Loopack
8-15: Fiber 1-8
   */
  tiSetBusySource(0x14,0);
  tiStatus(0);
#ifdef USE_FADC
  /* FADC library init */
  faInit(FADC_ADDR, FADC_INCR, NFADC, FA_INIT_SKIP);

  faGStatus(0);

#endif


  printf("rocDownload: User Download Executed\n");

}

void
rocPrestart()
{
  int ifa;

  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);

  tiStatus(0);

#ifdef USE_FADC
 /*****************
   *   FADC SETUP
   *****************/

  /* FADC Initialization flags */
  int iflag = 0xea00;  /* FASDC address */
  iflag |= FA_INIT_EXT_SYNCRESET;
  iflag |= FA_INIT_FP_TRIG;
  iflag |= FA_INIT_FP_CLKSRC;

  fadcA32Base = 0x0a000000;

  faInit(FADC_ADDR, FADC_INCR, NFADC, iflag);

  /* Disable multiblock when using faSDC */
  faDisableMultiBlock();

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

  for(ifa=0; ifa < nfadc; ifa++)
    {
      faSoftReset(faSlot(ifa),0);
      faResetTriggerCount(faSlot(ifa));
      faEnableSyncReset(faSlot(ifa));
    }

  faSDC_Config(2,0x3F); // with all signals hooked up (trig,clk,sync,busy)

  faSDC_Status(0);

  // SYNC IS ISSUED FROM TI - BM 26sept2021
  //  faSDC_Sync();

#endif /* USE_FADC */
   int itdc;

   typedef struct
   {
     int EdgeResolution;
     int EdgeDetectionConfig;
     int WindowWidth;
     int WindowOffset;
   } tdc1190_config;

   tdc1190_config Common1190Config =
     {
      .EdgeResolution = 100,
      .EdgeDetectionConfig = 3,
      .WindowWidth = 2000,
      .WindowOffset = -2000
     };


  /* INIT C1190/C1290 - Must be A32 for 2eSST */
 UINT32 list[NUM_V1190] = {0x100000,0x180000};
 // UINT32 list[NUM_V1190] = {0x100000};
  tdc1190InitList(list,NUM_V1190,2);

  unsigned int mcstaddr = 0x09000000;
  if(C1190_ROMODE==0)  tdc1190InitMCST(mcstaddr);

  tdc1190GSetTriggerMatchingMode();
  tdc1190GSetEdgeResolution(Common1190Config.EdgeResolution);
  tdc1190GSetEdgeDetectionConfig(Common1190Config.EdgeDetectionConfig);
  tdc1190GSetWindowWidth(Common1190Config.WindowWidth); // ns
  tdc1190GSetWindowOffset(Common1190Config.WindowOffset); // ns
  tdc1190GEnableTriggerTimeSubtraction(); // Uses the beginning of the match window instead of the latest bunch reset

  tdc1190GTriggerTime(1); // flag = 1 enable (= 0 disable) writing out of the Extended Trigger Time Tag in the output buffer.
  tdc1190ConfigureGReadout(C1190_ROMODE);

  for(itdc=0; itdc<NUM_V1190; itdc++)
    {
#ifdef WRITE_TWICE
      tdc1190SetTriggerMatchingMode(itdc);
      tdc1190SetEdgeResolution(itdc,100);
      tdc1190SetEdgeDetectionConfig(itdc,3);
      tdc1190SetWindowWidth(itdc,2000); // ns
      tdc1190SetWindowOffset(itdc,-2000); // ns
#endif
      tdc1190SetGeoAddress(itdc, list[itdc] >> 19);
    }

  DALMAGO;
#ifdef USE_FADC
  faGStatus(0);
#endif
  tdc1190GStatus(1);
  tiStatus(0);
  DALMASTOP;

  printf("rocPrestart: User Prestart Executed\n");

}

void
rocGo()
{
  int fadc_mode = 0;
  unsigned int pl=0, ptw=0, nsb=0, nsa=0, np=0;
  tiSetBlockLimit(0);
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

#ifdef USE_FADC
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
  faGStatus(0);

  /*  Enable FADC */
  faGEnable(0, 0);
#endif


  tdc1190GSetBLTEventNumber(blockLevel);

#ifdef TI_MASTER
  if(rocTriggerSource != 0)
    {
      printf("************************************************************\n");
      daLogMsg("INFO","TI Configured for Internal Pulser Triggers");
      printf("************************************************************\n");

      if(rocTriggerSource == 1)
	{
	  /* Enable Random at rate 500kHz/(2^7) = ~3.9kHz */
	  tiSetRandomTrigger(1,7);
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
#endif

  /* Reset c1190 clear counter */
  c1190ClearCounter = 0;

  /* Interrupts/Polling enabled after conclusion of rocGo() */
}

void
rocEnd()
{

#ifdef TI_MASTER
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

#endif

#ifdef USE_FADC
  /* FADC Disable */
  faGDisable(0);
#endif

  DALMAGO;
  tdc1190GStatus(0);
  printf("   c1190 syncevent clear (c1190ClearCounter) = %d\n",
	 c1190ClearCounter);
#ifdef USE_FADC
  faGStatus(0);
#endif

  tiStatus(0);
  DALMASTOP;

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());

}

void
rocTrigger(int arg)
{
  int ifa = 0, stat, nwords, dCnt;
  unsigned int datascan=0, scanmask=0;
  int roType = 2, roCount = 0, blockError = 0;

  roCount = tiGetIntCount();

  /* Set TI output 1 high for diagnostics */
  tiSetOutputPort(1,0,0,0);

  /* Readout the trigger block from the TI
     Trigger Block MUST be readout first */
  dCnt = tiReadTriggerBlock(dma_dabufp);

  if(dCnt<=0)
    {
      daLogMsg("ERROR","No TI Trigger data or error.  event = %d\n",
	       tiGetIntCount());
    }
  else
    { /* TI Data is already in a bank structure.  Bump the pointer */
      dma_dabufp += dCnt;
    }


#ifdef USE_FADC
 /* fADC250 Readout */
  BANKOPEN(FADC_BANK,BT_UI4,blockLevel);

  /* Mask of initialized modules */
  scanmask = faScanMask();
  /* Check scanmask for block ready up to 100 times */
  datascan = faGBlockReady(scanmask, 100);
  stat = (datascan == scanmask);

  if(stat == 0)
    {
      daLogMsg("ERROR","Event %d: fadc Datascan != Scanmask  (0x%08x != 0x%08x)\n",
	     roCount, datascan, scanmask);
    }

  if(datascan)
    {
      for(ifa = 0; ifa < nfadc; ifa++)
	{
	  if( ((1 << faSlot(ifa)) & datascan) == 0)
	    continue;

	  vmeDmaConfig(2,5,1);

	  nwords = faReadBlock(faSlot(ifa), dma_dabufp, MAXFADCWORDS, 1);

	  /* Check for ERROR in block read */
	  blockError = faGetBlockError(1);

	  if(blockError)
	    {
	      daLogMsg("ERROR","fadc Slot %d: in transfer (event = %d), nwords = 0x%x\n",
		       faSlot(ifa), roCount, nwords);

	      if(nwords > 0)
		dma_dabufp += nwords;
	    }
	  else
	    {
	      dma_dabufp += nwords;
	    }
	}
    }

  BANKCLOSE;
#endif

  BANKOPEN(C1190_BANKID,BT_UI4,blockLevel);
  /* Check for valid data here */
  scanmask = tdc1190ScanMask();
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
	  daLogMsg("ERROR","c1190 error in transfer (event = %d), status = 0x%x\n",
		 tiGetIntCount(),dCnt);
	  *dma_dabufp++ = LSWAP(0xda000bad);
	}
      else
	{
	  dma_dabufp += dCnt;
	}
    }
  else
    {
      daLogMsg("ERROR","c1190 Data not ready in event %d.. 0x%08x != 0x%08x\n",
	     tiGetIntCount(), datascan, scanmask);
      *dma_dabufp++ = LSWAP(datascan);
      *dma_dabufp++ = LSWAP(scanmask);
      *dma_dabufp++ = LSWAP(0xda000bad);
    }
  BANKCLOSE;

  /* Check for SYNC Event or Bufferlevel == 1 */
  if((tiGetSyncEventFlag() == 1) || (tiGetBlockBufferLevel() == 1))
    {
      int iflush = 0, maxflush = 10;
      /* Check for data available */
      int davail = tiBReady();
      if(davail > 0)
	{
	  daLogMsg("ERROR","%4d: TI Data available (%d) after readout! \n",
		 tiGetIntCount(), davail);

	  iflush = 0;
	  while(tiBReady() && (++iflush < maxflush))
	    {
	      vmeDmaFlush(tiGetAdr32());
	    }
	}

#ifdef USE_FADC
      for(ifa = 0; ifa < nfadc; ifa++)
	{
	  davail = faBready(faSlot(ifa));
	  if(davail > 0)
	    {
	      daLogMsg("ERROR", "fADC250 Data available (%d) after readout in SYNC event (%d)\n",
		     davail, tiGetIntCount());

	      iflush = 0;
	      while(faBready(faSlot(ifa)) && (++iflush < maxflush))
		{
		  vmeDmaFlush(faGetA32(faSlot(ifa)));
		}
	    }
	}
#endif

      /* Check for ANY data in 1190 TDCs */
      datascan = tdc1190GDready(1);
      static int max_complaints = 10;
      static int complaints = 0;
      if(datascan > 0)
	{
	  c1190ClearCounter++;

	  if(++complaints < max_complaints)
	    daLogMsg("ERROR","%4d: C1190 Data available (0x%x) after readout! %d\n",
		     tiGetIntCount(), datascan, complaints);

	  if(complaints == max_complaints)
	    daLogMsg("ERROR","%4d: C1190 Data available (0x%x) after readout! MAX SHOWN\n",
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
#ifdef USE_FADC
  printf("%s: Reset all FADCs\n",__FUNCTION__);
  faGReset(1);
#endif

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
  compile-command: "make -k -B bbhodo_list.so bbhodo_slave_list.so"
  End:
 */
