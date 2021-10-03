/*************************************************************************
 *
 *  bbshower_list.c - Library of routines for config and readout of
 *                    fadc250s and f1tdc-v1, using a JLab pipeline TI
 *                    as a source for trigger and syncreset.
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   1024*240      /* Size in Bytes */

#ifdef TI_MASTER
/* EXTernal trigger source (e.g. front panel ECL input), POLL for available data */
#define TI_READOUT TI_READOUT_EXT_POLL
#else
/* TS trigger source (e.g. fiber), POLL for available data */
#define TI_READOUT TI_READOUT_TS_POLL
#endif
#define TI_ADDR    (21<<19)          /* GEO slot 21 */

#define FIBER_LATENCY_OFFSET 0x50  /* measured longest fiber length */

#include <unistd.h>
#include "dmaBankTools.h"
#include "tiprimary_list.c" /* source required for CODA */
#include "sdLib.h"
#include "fadcLib.h"        /* library of FADC250 routines */
#include "fadc250Config.h"
#ifdef ENABLE_F1
#include "f1tdcLib.h"       /* library of f1tdc routines */
#endif

/* Library to pipe stdout to daLogMsg */
#include "dalmaRolLib.h"

#define BUFFERLEVEL 1

/* FADC Library Variables */
extern int fadcA32Base, nfadc;
#define NFADC     18
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

#ifdef ENABLE_F1
/* F1TDC Specifics */
extern int f1tdcA32Base;
extern int nf1tdc;
#define F1_ADDR  (0x100000)
#define F1TDC_BANK 0x4
#endif

/* for the calculation of maximum data words in the block transfer */
unsigned int MAXFADCWORDS=0;

/* SD variables */
static unsigned int sdScanMask = 0;

#ifdef FADC_SCALERS
/* FADC Scalers */
// Define this to include scaler data in coda events
// #define FADC_SCALER_BANKS
int scaler_period=2;
struct timespec last_time;
#include "../scaler_server/scale32LibNew.c"
#include "../scaler_server/linuxScalerLib.c"
#endif

/*
  Global to configure the trigger source
      0 : tsinputs
      1 : TI random pulser
      2 : TI fixed pulser

  Set with rocSetTriggerSource(int source);
*/
int rocTriggerSource = 0;
void rocSetTriggerSource(int source); // routine prototype


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
  if(rocTriggerSource == 0)
    {
      tiSetTriggerSource(TI_TRIGGER_TSINPUTS); /* TS Inputs enabled */
    }
  else
    {
      tiSetTriggerSource(TI_TRIGGER_PULSER); /* Internal Pulser */
    }

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

  /* FADC library init */
  faInit(FADC_ADDR, FADC_INCR, NFADC, FA_INIT_SKIP);

  faGStatus(0);
#ifdef ENABLE_F1
  /* Setup the F1TDC */
  f1tdcA32Base = 0x08800000;

  /*
    Configure first with local clock, normal res, trigger matching
    Configure with system clock in PRESTART
  */
  iflag = 2;

  f1Init(F1_ADDR, 1<<19, 1, iflag);

  if(nf1tdc > 1) {
    f1EnableMultiBlock();
  } else {
    f1GEnableBusError();
  }
#endif

#ifdef FADC_SCALERS
  if(fadcscaler_init_crl()) {
    printf("Scalers initialized\n");
  } else {
    printf("Failed to initialize scalers\n");
  }
  set_runstatus(0);
#endif

  tiStatus(0);
  sdStatus(0);
#ifdef ENABLE_F1
  f1GStatus(0);
#endif
  faGStatus(0);

  printf("rocDownload: User Download Executed\n");

}

void
rocPrestart()
{
  int ifa, if1;

#ifdef FADC_SCALERS
  /* Suspend scaler task */
  set_runstatus(1);
  clock_gettime(CLOCK_REALTIME, &last_time);
#endif

  /* Program/Init VME Modules Here */
  /*****************
   *   FADC SETUP
   *****************/

  /* FADC Initialization flags */
  int iflag = 0; /* NO SDC */
  iflag |= (1<<0);  /* VXS sync-reset */
  iflag |= FA_INIT_VXS_TRIG;  /* VXS trigger source */
  iflag |= FA_INIT_VXS_CLKSRC;  /* VXS 250MHz Clock source */

  fadcA32Base = 0x09000000;

  faInit(FADC_ADDR, FADC_INCR, NFADC, iflag);

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


#ifdef ENABLE_F1
  /* Use 31.25MHz clock from TI */
  f1SetClockPeriod(32);

  for(if1 = 0; if1 < nf1tdc; if1++)
    {
      f1SetInputPort(f1Slot(if1), 0); /* Front panel clock */
      f1DisableClk(f1Slot(if1), 0);   /* Disable internal 40MHz clock */

      f1EnableClk(f1Slot(if1), 1);    /* Use external clock */
    }

  f1ConfigReadFile("/home/sbs-onl/cfg/intelbbshower_f1tdc_3125.cfg");
  for(if1 = 0; if1 < nf1tdc; if1++)
    {
      f1SetConfig(f1Slot(if1), 4, 0); /* Configure f1chips with file */
    }

  usleep(50000);
  for(if1 = 0; if1 < nf1tdc; if1++)
    {
      f1EnableData(f1Slot(if1),0xff);
    }
  f1GClear();
  f1GStatus(0);
#endif


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

  DALMAGO;
  sdStatus(0);
#ifdef ENABLE_F1
  f1GStatus(0);
#endif
  tiStatus(0);
  faGStatus(0);
  DALMASTOP;

  printf("rocPrestart: User Prestart Executed\n");

}

void
rocGo()
{
  int fadc_mode = 0;
  unsigned int pl=0, ptw=0, nsb=0, nsa=0, np=0;

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

#ifdef ENABLE_F1
#ifdef F1_SOFTTRIG
  f1GEnableSoftTrig();
#endif
#endif

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
  /* Interrupts/Polling enabled after conclusion of rocGo() */

#ifdef FADC_SCALERS
  /* Clear and enable FADC scalers */
  clear_scalers();
  printf("fadc scalers cleared\n");
  enable_scalers();
#endif

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

  /* FADC Disable */
  faGDisable(0);

#ifdef ENABLE_F1
  /* F1TDC Event status - Is all data read out */
  int islot = 0;
#ifdef F1_SOFTTRIG
  for(islot = 0; islot < nf1tdc; islot++)
    f1DisableSoftTrig(f1Slot(islot));
#endif

  for(islot = 0; islot < nf1tdc; islot++) {
    f1DisableData(f1Slot(islot));
    /* f1Reset(f1Slot(islot),0); */
  }
#endif

  DALMAGO;
  sdStatus(0);
#ifdef ENABLE_F1
  f1GStatus(0);
#endif
  tiStatus(0);
  faGStatus(0);
  DALMASTOP;

#ifdef FADC_SCALERS
  /* Resume stand alone scaler server */
  disable_scalers();
  set_runstatus(0);		/* Tell Stand alone scaler task to resume  */
#endif

  printf("rocEnd: Ended after %d events\n",tiGetIntCount());

}

void
rocTrigger(int arg)
{
  int ifa = 0, stat, nwords, dCnt;
  unsigned int datascan, scanmask;
  int roType = 2, roCount = 0, blockError = 0;
  int ii, islot;

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
      daLogMsg("ERROR","No TI Trigger data or error.  dCnt = %d\n",dCnt);
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
	  daLogMsg("ERROR","fadc Slot %d: in transfer (event = %d), nwords = 0x%x\n",
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
      daLogMsg("ERROR","Event %d: fadc Datascan != Scanmask  (0x%08x != 0x%08x)\n",
	     roCount, datascan, scanmask);
    }
  BANKCLOSE;

  int roflag = 1;

#ifdef ENABLE_F1
  if(nf1tdc <= 1) {
    roflag = 1; /* DMA Transfer */
  } else {
    roflag = 2; /* Multiple DMA Transfer */
  }

  int f1maxdata = 100*nf1tdc*64;
  /* Set DMA for A32 - BLT64 */
  vmeDmaConfig(2,3,0);
  BANKOPEN(F1TDC_BANK,BT_UI4,0);

#ifdef F1_SOFTTRIG
  f1Trig(f1Slot(0));
#endif

  for(ii=0;ii<100;ii++)
    {
      datascan = f1Dready(f1Slot(0));
      if (datascan>0)
	{
	  break;
	}
    }

  if(datascan>0)
    {
      for(islot = 0; islot < nf1tdc; islot++) {
       nwords = f1ReadEvent(f1Slot(islot),dma_dabufp,f1maxdata,roflag);

      if(nwords < 0)
	{
	  daLogMsg("ERROR","f1ReadEvent error.  f1Slot = %d  event = %d , status = 0x%x\n",
		   f1Slot(islot), tiGetIntCount(),nwords);
	  *dma_dabufp++ = LSWAP(0xda000bad);
	  f1GClear();
	}
      else if (nwords >= f1maxdata)
	{
	  dma_dabufp += nwords;
	  *dma_dabufp++ = LSWAP(0xda000bad);
	  daLogMsg("ERROR","MAX f1 data.  f1slot = %d  event = %d\n",
		   f1Slot(islot), tiGetIntCount());
	  f1GClear();
	}
      else
	{
	  dma_dabufp += nwords;
	}
      }
    }
  else
    {
      daLogMsg("ERROR","Data not ready in event %d, F1TDC slot %d\n",
	       tiGetIntCount(), f1Slot(0));
      *dma_dabufp++ = LSWAP(0xda000bad);
    }
  *dma_dabufp++ = LSWAP(0xda0000ff); /* Event EOB */
  BANKCLOSE;
#endif

  /* Check for SYNC Event */
  if(tiGetSyncEventFlag() == 1)
    {
      int iflush = 0, maxflush = 10;
      /* Check for data available */
      int davail = tiBReady();
      if(davail > 0)
	{
	  daLogMsg("ERROR","TI Data available (%d) after readout in SYNC event (%d)\n",
		   davail, tiGetIntCount());

	  iflush = 0;
	  while(tiBReady() && (++iflush < maxflush))
	    {
	      vmeDmaFlush(tiGetAdr32());
	    }
	}

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

#ifdef ENABLE_F1
      for(islot = 0; islot < nf1tdc; islot++)
	{
	  davail = f1Dready(f1Slot(islot));
	  if(davail > 0)
	    {
	      daLogMsg("ERROR","f1tdc Data available (%d) after readout in SYNC event (%d)\n",
		     davail, tiGetIntCount());
#ifdef F1DMAFLUSH
	      iflush = 0;
	      while(f1Dready(f1Slot(islot)) && (++iflush < maxflush))
		{
		  vmeDmaFlush(f1GetA32(f1Slot(islot)));
		}
#else
	      f1Clear(f1Slot(islot));
#endif
	    }
	}

      /* Check for f1 errors */
      stat = f1GErrorStatus(0);
      if (stat)
	{
	  f1GClearStatus(0); /* Clear Latched Status again */
	  stat = f1GErrorStatus(0);
	  if(stat)
	    {
	      daLogMsg("ERROR",
		       "F1 TDCs have active error condition.  stat = 0x%x  SYNC event (%d)\n",
		       stat, tiGetIntCount());

	    }
	}
#endif

    }
#ifdef FADC_SCALERS
  if (scaler_period > 0) {
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    if((scaler_period>0 &&
	((now.tv_sec - last_time.tv_sec
	  + ((double)now.tv_nsec - (double)last_time.tv_nsec)/1000000000L) >= scaler_period))) {
#define FADC_SCALER_BANKS
#ifdef FADC_SCALER_BANKS
      BANKOPEN(9250,BT_UI4,0);
      read_fadc_scalers(&dma_dabufp,0);
      BANKCLOSE;
      BANKOPEN(9001,BT_UI4,syncFlag);
      read_ti_scalers(&dma_dabufp,0);
      BANKCLOSE;
#else
      read_fadc_scalers(0,0);
      read_ti_scalers(0,0);
#endif
      last_time = now;
      read_clock_channels();
    }
  }
#endif


}

void
rocLoad()
{
  dalmaInit(1);
}

void
rocCleanup()
{

  printf("%s: Reset all FADCs\n",__FUNCTION__);
  faGReset(1);

#ifdef ENABLE_F1
  printf("%s: Reset all F1TDCs\n",__FUNCTION__);
  int islot;
  for(islot=0; islot<nf1tdc; islot++)
    {
      f1HardReset(f1Slot(islot)); /* Reset, and DO NOT restore A32 settings (1) */
    }
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
  compile-command: "make -B bbshower_list.so bbshower_slave_list.so"
  End:
 */
