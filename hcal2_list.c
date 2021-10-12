/*************************************************************************
 *
 *  hcal2_list.c - Readout list for intelsbshcal2
 *                   Read out: TI, fadc250s, f1tdcs
 *
 */

/* Decide which components we want to enable. Comment out to disable */
#define ENABLE_F1 /* F1TDCs */
#define ENABLE_FADC  /*  FADC 250s */

/* Decide which readouts we want enabled */
#define READOUT_FADC   /* Readout FADCs */
#define READOUT_F1      /* Readout F1TDCs */

/* Comment out the following to re-enable the MultiBlock readout
   On our (July 2019) test VME64x crate we don't have the necessary jumpers
   installed to do multiblock readout.
 */
#define DISABLE_MULTIBLOCK_READOUT

/* Specify the banks that will contain the data here */
#define BANK_FADC        3
#define BANK_TI          4
#define BANK_TEST        5
#define BANK_HCAL_PULSER 6
#define BANK_F1          7

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   1024*60      /* Size in Bytes */

/* TI_MASTER / TI_SLAVE defined in Makefile */

#ifdef TI_SLAVE5
#define TI_SLAVE
#define TI_FLAG TI_INIT_SLAVE_FIBER_5
#endif



#ifdef TI_MASTER
/* EXTernal trigger source (e.g. front panel ECL input), POLL for available data */
#define TI_READOUT TI_READOUT_EXT_POLL
#else
/* TS trigger source (e.g. fiber), POLL for available data */
#define TI_READOUT TI_READOUT_TS_POLL
#endif
#define TI_ADDR  0 /* Auto initialize (search for TI by slot */

#define FIBER_LATENCY_OFFSET 0x10  /* measured longest fiber length */

#include <unistd.h>
#include "dmaBankTools.h"
#include "tiprimary_list.c" /* source required for CODA */
#include "sdLib.h"

/* Library to pipe stdout to daLogMsg */
#include "dalmaRolLib.h"

#ifdef ENABLE_FADC
#include "fadcLib.h"        /* library of FADC250 routines */
#include "fadc250Config.h"
#endif /* ENABLE_FADC */

#ifdef ENABLE_F1
#include "f1tdcLib.h"       /* library of f1tdc routines */
#endif /* ENABLE_F1 */

#include "hcal_usrstrutils.c" /* So we can read from a flags file */

#ifdef ENABLE_HCAL_PULSER
#include "hcalLib.h"
#endif

#define BUFFERLEVEL 5

#ifdef ENABLE_FADC
/* FADC Library Variables */
extern int fadcA32Base, nfadc;
#define NFADC     3
/* Address of first fADC250 */
#define FADC_ADDR (18<<19)
/* Increment address to find next fADC250 */
#define FADC_INCR (1<<19)

#define FADC_READ_CONF_FILE {			\
    fadc250Config("");				\
    if(rol->usrConfig)				\
      fadc250Config(rol->usrConfig);		\
  }
#endif /* ENABLE_FADC */


#ifdef ENABLE_F1
/* F1TDC Specifics */
extern int f1tdcA32Base;
int F1_SLOT;
extern int f1ID[F1_MAX_BOARDS];
//extern unsigned int f1AddrList[F1_MAX_BOARDS]; /* Array of slot numbers, discovered by the library */
#define F1_ADDR  4<<19
#define F1_SD_ADDR 0xdd0  /*Just needs a unique address. Slot doesn't matter.*/
int NF1TDC = 5;
/* Settings for cosmic tests Dec 2020 */
#define F1_WINDOW 800
#define F1_LATENCY 800 //Can't be smaller than F1 window.
#endif


/* HCal FLAGS */
#define NINPUTS 6 /* 6 trigger inputs on the new TI */
int flag_prescale[NINPUTS];
/* Flags for HCal pulser */
int flag_pulserTriggerInput;
int flag_LED_NSTEPS; /* Number of steps in sequence */
int flag_LED_STEP[50]; /* Step LED configuration */
int flag_LED_NSTEP[50]; /* Number of triggers in this step */


/* for the calculation of maximum data words in the block transfer */
unsigned int MAXFADCWORDS=0;
unsigned int MAXF1WORDS   = 0;

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

/* function prototypes */
void rocTrigger(int arg);
void readUserFlags();

void
rocDownload()
{
  unsigned int iflag;
  int ifa, stat;

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
  tiSetTriggerSource(TI_TRIGGER_TSINPUTS); /* TS Inputs enabled */

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
#else
  tiSetFiberSyncDelay(0x9);
#endif /* TI_MASTER */
  tiSetBusySource(0xA,0); // adding F1 busy AC 10/01/2021

  /* Init the SD library so we can get status info */
  sdScanMask = 0;
  stat = sdInit(0);
  if (stat != OK)
    {
      printf("%s: WARNING: sdInit() returned %d\n",__func__, stat);
      daLogMsg("ERROR","SD not found");
    }

#ifdef FADC_SCALERS
  if(fadcscaler_init_crl()) {
    printf("Scalers initialized\n");
  } else {
    printf("Failed to initialize scalers\n");
  }
  set_runstatus(0);
#endif

  tiStatus(1);
  sdStatus(0);

  printf("rocDownload: User Download Executed\n");

}

void
rocPrestart()
{
  int ifa, iflag;
  int i,ti_input_triggers;

#ifdef FADC_SCALERS
  /* Suspend scaler task */
  set_runstatus(1);
  clock_gettime(CLOCK_REALTIME, &last_time);
#endif

  /* Program/Init VME Modules Here */

  /* Read in the user flag files */
  readUserFlags();

#ifdef TI_MASTER
  /* TI Master configuration */

  /* Set needed TS input bits */
  ti_input_triggers = 0;
  printf("prescales: ");
  for(i = 0; i < NINPUTS; i++) {
    ti_input_triggers |= (flag_prescale[i]>=0)<<i;
    printf("ps%d=%d ",i+1,flag_prescale[i]);
  }
  printf("\n");
  tiEnableTSInput( ti_input_triggers );

  /* Setup the prescales */
  for(i = 0; i < NINPUTS; i++) {
    if(flag_prescale[i]>=0) {
      tiSetInputPrescale(1<<i,flag_prescale[i]);
    }
  }
#endif


#ifdef ENABLE_FADC
  /*****************
   *   FADC SETUP
   *****************/

  /* FADC Initialization flags */
  iflag = FA_INIT_VXS_TRIG | FA_INIT_VXS_CLKSRC | FA_INIT_EXT_SYNCRESET;

  fadcA32Base = 0x0a000000;

  vmeSetQuietFlag(1);
  faInit(FADC_ADDR, FADC_INCR, NFADC, iflag);
  vmeSetQuietFlag(0);

  sdSetActiveVmeSlots(faScanMask()); /* Tell the sd where to find the fadcs */


  if(nfadc == 1)
    faDisableMultiBlock();
  else
    faEnableMultiBlock(1);

  /* configure all modules based on config file */
  FADC_READ_CONF_FILE;

  for(ifa=0; ifa < nfadc; ifa++)
    {
      faEnableBusError(faSlot(ifa));
      faResetMGT(faSlot(ifa),1);
      faSetTrigOut(faSlot(ifa), 7);

      faSetTriggerBusyCondition(faSlot(ifa), 3);

      faSoftReset(faSlot(ifa),0);
      faResetToken(faSlot(ifa));
      faResetTriggerCount(faSlot(ifa));
      faEnableSyncReset(faSlot(ifa));
    }

  /***************************************
   *   SD SETUP
   ***************************************/
  sdInit(0);   /* Initialize the SD library */
  sdSetActiveVmeSlots(faScanMask()); /* Use the fadcSlotMask to configure the SD */
#endif /* ENABLE_FADC */

#ifdef ENABLE_F1
  f1tdcA32Base = 0x08800000;
  /* Setup the F1TDC */
  f1ConfigReadFile("/home/sbs-onl/cfg/f1tdc/hcal_f1tdc_3125.cfg");

  int islot = 0;
  iflag = F1_SD_ADDR; /* with SD address */
  iflag |= 4;  /* read from file */
  iflag |= 0x10000; /* SDC mode = 1 for external clock and syncreset */
  printf("iflag = 0x%x\n",iflag);

  /* f1AddrList[0] = 4<<19; */
  /* f1AddrList[1] = 5<<19; */
  /* f1AddrList[2] = 6<<19; */
  /* f1AddrList[3] = 7<<19; */
  /* f1AddrList[4] = 8<<19; */
  f1SetClockPeriod(32);
  f1Init(F1_ADDR,1<<19,NF1TDC,iflag);
  usleep(30000);

  /* f1Init(F1_ADDR,0,NF1TDC,iflag); */
  islot = 0;
  printf("\n\n\n - - - - AFTER f1Init - - - - - \\n\n\n");
  for(islot = 0; islot < NF1TDC; islot++) {
    printf("Found slot in %d\n",f1ID[islot]);

    /* Should we reset here? */
    /*  f1Reset(f1ID[islot],0); */
  }
  /*  f1Init(F1_ADDR,1<<19,NF1TDC,iflag); */
  F1_SLOT = f1ID[0];
  printf("\n\n\n\n\n");


  /* For multiple boards we enable multiblock, which also enables
     the bus error on the last board. When not in multiblock we can
     enable multiblock on the single board ourselves.
  */
#ifndef DISABLE_MULTIBLOCK_READOUT
  if(NF1TDC>1)
    f1EnableMultiBlock();
#else /* Multiblock disabled? Enable the bus error on each F1TDC */
  f1DisableMultiBlock();
  /* This is a test: */
#endif /* End: !DISABLE_MULTIBLOCK_READOUT */

  f1GEnableData(F1_ALL_CHIPS); /* Enable data on all chips */
  f1GEnableBusError();
  f1GSetBlockLevel(1);   /* fix default block level at 1 */
  f1GClear();
  F1_SLOT = f1ID[0];

  /*
  for(islot = 0; islot < NF1TDC; islot++)
    {
      f1SetWindow(f1ID[islot],F1_WINDOW,F1_LATENCY,0);
    }
  */
#endif /* End: ENABLE_F1 */

#ifdef ENABLE_HCAL_PULSER
  if(flag_PULSER_ENABLED) {
    HCAL_LED_ITER=0;
    HCAL_LED_COUNT=0;
    printf("HCAL Pulser sequence: ");
    for(i = 0 ; i < flag_LED_NSTEPS; i++) {
      printf(" %d/%d",flag_LED_STEP[i],flag_LED_NSTEP[i]);
    }
    printf("\n");
    /* Clock in the first setting */
    HCAL_LED_C_STEP = flag_LED_STEP[0];
    HCAL_LED_C_NSTEP = flag_LED_NSTEP[0];
    /*BQ     hcalClkIn(HCAL_LED_C_STEP);*/
  } else {
    /*BQ      hcalClkIn(0);*/
  }

#endif

#ifdef ENABLE_F1
  printf("\n\n\n * * * * BEFORE Sending ReSync (to lock resolution) \n\n\n");
  f1SDC_Sync();
  printf("\n\n\n * * * * AFTER  Sending ReSync (to lock resolution) \n\n\n");
  f1GStatus(0);
#endif

#ifdef TI_MASTER
  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);
#endif /* TI_MASTER */

  DALMAGO;
#ifdef ENABLE_FADC
  /* FADC Event status - Is all data read out */
  faGStatus(0);
#endif /* ENABLE_FADC */
#ifdef ENABLE_F1
  f1GStatus(0);
#endif /* End: ENABLE_F1 */
  tiStatus(1);
  sdStatus();
  DALMASTOP;

  printf("rocPrestart: User Prestart Executed\n");

}

void
rocGo()
{
  int fadc_mode = 0;
  unsigned int pl=0, ptw=0, nsb=0, nsa=0, np=0;

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

#ifdef ENABLE_FADC
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
#endif /* ENABLE_FADC */

#ifdef ENABLE_F1
  f1GSetBlockLevel(blockLevel);
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

#ifdef ENABLE_FADC
  /* FADC Disable */
  faGDisable(0);
#endif /* ENABLE_FADC */

#ifdef ENABLE_F1
  /* F1TDC Event status - Is all data read out */
  f1GStatus(0);
  int islot = 0;
  for(islot = 0; islot<NF1TDC; islot++) {
    F1_SLOT=f1ID[islot];
    f1Reset(F1_SLOT,0);
  }
#endif /* End: ENABLE_F1 */

  DALMAGO;
#ifdef ENABLE_FADC
  /* FADC Event status - Is all data read out */
  faGStatus(0);
#endif /* ENABLE_FADC */
#ifdef ENABLE_F1
  f1GStatus(0);
#endif /* End: ENABLE_F1 */
  tiStatus(1);
  sdStatus();
  DALMASTOP;

#ifdef ENABLE_HCAL_PULSER
  /*BQ  hcalClkIn(0);  // Turn off LEDs at the end  */
#endif
  /* Turn off all output ports */
  /*BQ  tiSetOutputPort(0,0,0,0);*/

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
  unsigned int datascan, scanmask, f1scanmask, islot, nf1tdc;
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
      printf("ERROR: No data or error in tiReadTriggerBlock().  dCnt = %d  readout# = %d\n",dCnt,roCount);
    }
  else
    {
      dma_dabufp += dCnt;
    }

#ifdef ENABLE_FADC
#ifdef READOUT_FADC

  /* fADC250 Readout */
  BANKOPEN(BANK_FADC,BT_UI4,0);

  /* Mask of initialized modules */
  scanmask = faScanMask();
  /* Check scanmask for block ready up to 100 times */
  datascan = faGBlockReady(scanmask, 10);
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
#endif /* READOUT_FADC */
#endif /* ENABLE_FADC */

#ifdef ENABLE_F1
#ifdef READOUT_F1
  int roflag = 1;

  if(NF1TDC <= 1) {
    roflag = 1; /* DMA Transfer */
  } else {
    roflag = 2; /* Multiple DMA Transfer */
  }
#ifdef DISABLE_MULTIBLOCK_READOUT
  roflag=1; /* Test, Alex says not to use MultiBlock */
#endif
  /* Set DMA for A32 - BLT64 */
  vmeDmaConfig(2,3,0);

  BANKOPEN(BANK_F1,BT_UI4,0);

  /* Insert the trigger count here */
  *dma_dabufp++ = LSWAP(roCount);
  /* Check for valid data here */
  F1_SLOT = f1ID[0];  /* First F1 board slot */
  f1scanmask = f1ScanMask();

  int ii, islot;
  for(ii=0;ii<10;ii++)
    {
      datascan = f1DataScan(0);
      if (datascan == f1scanmask)
	{
	  break;
	}
    }

  if(datascan == f1scanmask)   /* All F1 say they have data */
    {
      for(islot = 0; islot < NF1TDC; islot++) {
       F1_SLOT = f1ID[islot];
       /* This one was just a test, but it breaks the decoder, so only uncomment it for special tests */
       /*  *dma_dabufp++ = LSWAP(0xda000000|(F1_SLOT<<12)|0xadc); */
       nwords = f1ReadEvent(F1_SLOT,dma_dabufp,2*NF1TDC*64,roflag);
      /* TEST, print out only */
      /* nwords = 0;
      f1GPrintEvent(0);
      */


      if(nwords < 0)
	{
	  printf("ERROR: in f1ReadEvent() transfer (event = %d), status = 0x%x\n", roCount,nwords);
	  *dma_dabufp++ = LSWAP(0xda000bad);
	}
      else
	{
          /*  printf("Evt %10d: TDC in slot=%d read nwords=%d\n",tiGetIntCount(),F1_SLOT,nwords); */
	  dma_dabufp += nwords;
	}
      }
    }
  else
    {
      printf("ERROR: Data not ready for event %d. F1TDC data ready mask 0x%x is not equal to 0x%x\n",
	     roCount,datascan, f1scanmask);

      *dma_dabufp++ = LSWAP(0xda000bad);

      /* Just clear the F1 boards here worry about syncronizing later */
      f1Clear(F1_SLOT);

    }

  *dma_dabufp++ = LSWAP(0xda0000ff); /* Event EOB */
  BANKCLOSE;

  unsigned int scaler1 = 0, scaler2 = 0;
  BANKOPEN(0x0F10, BT_UI4, 0);
  for(islot = 0; islot < NF1TDC; islot++)
    {
      scaler1 = 0; scaler2 = 0;
      f1GetScalers(f1Slot(islot), &scaler1, &scaler2);
      *dma_dabufp++ = f1Slot(islot);
      *dma_dabufp++ = scaler1;
      *dma_dabufp++ = scaler2;
    }
  BANKCLOSE;

#ifdef DISABLE_MULTIBLOCK_READOUT
  /* vmeDmaConfig(2,5,1); */
#endif

#endif /* READOUT_F1 */
#endif /* ENABLE_F1 */

  /* Check for SYNC Event */
  if((tiGetSyncEventFlag() == 1) || (tiGetBlockBufferLevel() == 1))
    {
      int iflush = 0, maxflush = 10;
      /* Check for data available */
      int davail = tiBReady();
      if(davail > 0)
	{
	  daLogMsg("ERROR","TI Data still available (%d) after SYNC event ($d)\n",
		   davail,roCount);

	  iflush = 0;
	  /*
	  while(tiBReady() && (++iflush < maxflush))
	    {
	      vmeDmaFlush(tiGetAdr32());
	      }*/
	}

#ifdef ENABLE_FADC
#ifdef READOUT_FADC

      for(ifa = 0; ifa < nfadc; ifa++)
	{
	  davail = faBready(faSlot(ifa));
	  if(davail > 0)
	    {
	      daLogMsg("ERROR",
		       "fADC250 (slot %d) Data available (%d) after SYNC event\n",
		       faSlot(ifa), davail);

	      iflush = 0;
	      while(faBready(faSlot(ifa)) && (++iflush < maxflush))
		{
		  vmeDmaFlush(faGetA32(faSlot(ifa)));
		}
	    }
	}

#endif /* READOUT_FADC */
#endif /* ENABLE_FADC */

#ifdef ENABLE_F1
#ifdef READOUT_F1
      for(islot = 0; islot < NF1TDC; islot++)
	{
	  davail = f1Dready(f1Slot(islot));
	  if(davail > 0)
	    {
	      daLogMsg("ERROR",
		       "f1TDC (slot %d) Data available (%d) after SYNC event\n",
		       f1Slot(islot), davail);

	      iflush = 0;
	      /*
	      while(f1Dready(f1Slot(islot)) && (++iflush < maxflush))
		{
		  vmeDmaFlush(f1GetA32(f1Slot(islot)));
		  }*/
	      /* Just clear the boards here */
	      f1Clear(f1Slot(islot));

#ifdef OLDDMAFLUSH
	      unsigned int trash[512];
	      int timeout = 0;
	      while(f1Dready(f1Slot(islot)))
		{
		  nwords = f1ReadEvent(f1Slot(islot),trash,512,0);
		  printf("%s: dumped %d words\n", __func__, nwords);
		  if((nwords < 0) || (timeout++ > 10))
		    break;
		}
#endif

	    }
	}
#endif /* READOUT_F1 */
#endif /* ENABLE_F1 */

    }

#ifdef FADC_SCALERS
  if (scaler_period > 0) {
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    if((scaler_period>0 &&
	((now.tv_sec - last_time.tv_sec
	  + ((double)now.tv_nsec - (double)last_time.tv_nsec)/1000000000L) >= scaler_period))) {
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

#ifdef TI_MASTER
  tiResetSlaveConfig();
#endif
  dalmaClose();

#ifdef ENABLE_FADC
  printf("%s: Reset all FADCs\n",__FUNCTION__);
  faGReset(1);
#endif /* ENABLE_FADC */

#ifdef ENABLE_F1
  int islot;
  for(islot=0; islot<NF1TDC; islot++)
    {
      f1HardReset(f1ID[islot]); /* Reset, and DO NOT restore A32 settings (1) */
    }
#endif /* ENABLE_F1 */

}

/*  Read the user flags/configuration file. */
void readUserFlags()
{
  int i;
  char pstext[10];

  printf("Reading user flags file.");
  init_strings();
  flag_pulserTriggerInput=getint("pulserTrigger");
  /* Read the FADC configuration */
  /* flag_FADC_WINDOW_LAT=getint("fadc_window_lat"); */
  /* flag_FADC_WINDOW_WIDTH=getint("fadc_window_width"); */

  /* Read prescales */
  printf("prescales: ");
  for(i = 0; i < NINPUTS; i++) {
    sprintf(pstext,"ps%d",i);
    flag_prescale[i] = getint(pstext);
    printf("%s=%d, ",pstext,flag_prescale[i]);
  }
  printf("\n");

#ifdef ENABLE_HCAL_PULSER
  /* Read the hcal pulser step info */
  if(flag_pulserTriggerInput >= 0 && flag_pulserTriggerInput <6) {
    flag_PULSER_ENABLED = flag_prescale[flag_pulserTriggerInput];
    if(flag_PULSER_ENABLED) {
      flag_LED_NSTEPS=getint("pulser_nsteps");
      char ptext[50];
      for(i = 0; i < flag_LED_NSTEPS; i++) {
        sprintf(ptext,"pulser_step%d",i);
        flag_LED_STEP[i]  = getint(ptext);
        sprintf(ptext,"pulser_nstep%d",i);
        flag_LED_NSTEP[i] = getint(ptext);
      }
    }
  } else {
    flag_PULSER_ENABLED=0;
  }
#endif
}

/*
  Local Variables:
  compile-command: "make -k hcal2_slave_list.so hcal2_slave5_list.so"
  End:
 */
