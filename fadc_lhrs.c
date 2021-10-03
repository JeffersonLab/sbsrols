/*************************************************************************
 *
 *  Readout list for the LHRS
 *  This is based on fadc_example and then we added a few things.
 *  see the revision history
 *
 *  fadc_lhrs.c - Library of routines for config and readout of
 *                    fadc250s, using a JLab pipeline TI
 *                    as a source for trigger and syncreset.
 *
 *   build with
 *    make -B fadc_lhrs.so fadc_slave5_lhrs.so
 *
 *   TI_MASTER and TI_SLAVE defined in Makefile
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
/* Define port 5 as the Slave port */
#define TI_FLAG (TI_INIT_SLAVE_FIBER_5|TI_INIT_SKIP_FIRMWARE_CHECK)
#endif
/* TI VME address, or 0 for Auto Initialize (search for TI by slot) */
#define TI_ADDR  0

#define FIBER_LATENCY_OFFSET 0x50  /* measured longest fiber length */

#include <unistd.h>
#include "dmaBankTools.h"
#include "tiprimary_list.c" /* source required for CODA */
#include "sdLib.h"
#include "fadcLib.h"        /* library of FADC250 routines */
#include "fadc250Config.h"

/*  Configuration file parsing */
#define INTERNAL_FLAGS "ffile=/home/sbs-onl/cfg/helscaler.flags,hsactive=1,hsaddr=0xa10000"
#define HELSCAL_ACTIVE "hsactive"
#define HELSCAL_ADDR   "hsaddr"
#include "usrstrutils.c"

/* Library to pipe stdout to daLogMsg */
#include "dalmaRolLib.h"

/*  Headers for the helicity scaler */
#include "crateSharedMem.h"
#include "SIS.h"
#include "SIS3801.h"

#define BUFFERLEVEL 1

/* FADC Library Variables */
extern int fadcA32Base, nfadc;
#define NFADC     18
/* Address of first fADC250 */
#define FADC_ADDR (3<<19)
/* Increment address to find next fADC250 */
#define FADC_INCR (1<<19)
#define FADC_BANK 250

/* Use config file defined in COOL DB */
#define FADC_READ_CONF_FILE {			\
    fadc250Config("");				\
    if(rol->usrConfig)				\
      fadc250Config(rol->usrConfig);		\
  }

/* Flag to enable/disable reading the helicity scaler */
static int helScalerActive = 1;

/* for the calculation of maximum data words in the block transfer */
unsigned int MAXFADCWORDS=0;

/* SD variables */
static unsigned int sdScanMask = 0;

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

  printf("Hello !  rocTriggerSource = %d\n",rocTriggerSource);

  if(rocTriggerSource == 0)
    {
      tiSetTriggerSource(TI_TRIGGER_TSINPUTS); /* TS Inputs enabled */
    }
  else
    {
      tiSetTriggerSource(TI_TRIGGER_PULSER); /* Internal Pulser */
    }
  printf("Setting TI_TRIGGER_FPTRG \n");
  tiSetTriggerSource(TI_TRIGGER_FPTRG);

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

  /*****************
   *   HELICITY SCALER SETUP
   *****************/
  init_strings();
  if (getflag(HELSCAL_ADDR)==2){
    char * tmpptr;
    unsigned long newaddr = strtol(getstr(HELSCAL_ADDR),&tmpptr,0);
    SISSetRelativeAddress(0,newaddr);
  }
  initSIS(0);  // Don't do DMA initialization
  cshmSetScalerDebug(0);

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

  for(ifa=0; ifa < nfadc; ifa++)
    {
      faSoftReset(faSlot(ifa),0);
      faResetToken(faSlot(ifa));
      faResetTriggerCount(faSlot(ifa));
      faEnableSyncReset(faSlot(ifa));
    }

  /* Initialize the helicity scaler */
  init_strings();
  helScalerActive = getint(HELSCAL_ACTIVE);
  if (helScalerActive){
    SIS3801_Prestart();
    SISSetByteOrder(1);
  }

#ifdef TI_MASTER
  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);
#endif

  DALMAGO;
  sdStatus(0);
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

  /*  Start the SIS3801 */
  if (helScalerActive){
    SIS3801_Go();
  }

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

#ifdef F1_SOFTTRIG
  f1GEnableSoftTrig();
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

  SIS3801_End();  ///  Disable the helicity scaler LNE; always disable, without checking helScalerActive

  DALMAGO;
  sdStatus(0);
  tiStatus(0);
  faGStatus(0);
  DALMASTOP;

  printf("rocEnd: Ended after %d events\n",tiGetIntCount());

}

void
rocTrigger(int arg)
{
  int ifa = 0, stat, nwords, dCnt;
  unsigned int datascan, scanmask;
  int roType = 2, roCount = 0, blockError = 0;
  int ii, islot;
  int n, i, j;
  UINT32 value;

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

  /***************************
   *  Helicity scaler banks
   ***************************/
  //  Check for new data in the helicity scaler and output it to the data stream
  if (helScalerActive){
    SISInterrupt(0);
    n = NumRing();
    if (n>0) {
      BANKOPEN(0xab01,BT_UI4,0);
      *dma_dabufp++ = LSWAP(n);
      for (i = 1; i <= n; i++) {
	// logMsg("Has ring entry\n",0,0,0,0,0,0);
	for (j = 0; j < NRING_CHANNEL+2; j++) {
	  value = ReadRing(i,j);
	  *dma_dabufp++ = LSWAP(value);
	}
      }
      ResetRing(n);
      BANKCLOSE;
    }
    //  Bank for the new event-trigger projected helicity info
    BANKOPEN(0xab11,BT_UI4,0);
    //  *dma_dabufp++ = LSWAP(0xab0000ab);
    dma_dabufp += ReportNextHelicityReading(dma_dabufp);
    BANKCLOSE;
    //  Bank for new scaler summary block
    BANKOPEN(0xab12,BT_UI4,0);
    dma_dabufp += ReportLastScalerSummary(dma_dabufp);
    BANKCLOSE;
  }

  /* Check for SYNC Event */
  if(tiGetSyncEventFlag() == 1)
    {
      /* Check for data available */
      int davail = tiBReady();
      if(davail > 0)
	{
	  daLogMsg("ERROR","TI Data available (%d) after readout in SYNC event \n",
		 davail);

	  while(tiBReady())
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
	      daLogMsg("ERROR", "fADC250 Data available (%d) after readout in SYNC event \n",
		     davail);

	      while(faBready(faSlot(ifa)))
		{
		  vmeDmaFlush(faGetA32(faSlot(ifa)));
		}
	    }
	}
#endif /* USE_FADC */

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

  printf("%s: Reset all FADCs\n",__FUNCTION__);
  faGReset(1);

  SIS3801_End();  ///  Disable the helicity scaler LNE; always disable, without checking helScalerActive

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
  compile-command: "make -B fadc_lhrs.so fadc_slave5_lhrs.so"
  End:
 */
