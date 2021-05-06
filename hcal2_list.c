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

#define BUFFERLEVEL 1

#ifdef ENABLE_FADC
/* FADC Library Variables */
extern int fadcA32Base, nfadc;
#define NFADC     2
/* Address of first fADC250 */
#define FADC_ADDR (19<<19)
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
#endif /* TI_MASTER */


  /* Init the SD library so we can get status info */
  sdScanMask = 0;
  stat = sdInit(0);
  if (stat != OK)
    {
      printf("%s: WARNING: sdInit() returned %d\n",__func__, stat);
      daLogMsg("ERROR","SD not found");
    }

  tiStatus(0);
  sdStatus(0);

  printf("rocDownload: User Download Executed\n");

}

void
rocPrestart()
{
  int ifa, iflag;
  int i,ti_input_triggers;

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

  faGStatus(0);

  /***************************************
   *   SD SETUP
   ***************************************/
  sdInit(0);   /* Initialize the SD library */
  sdSetActiveVmeSlots(faScanMask()); /* Use the fadcSlotMask to configure the SD */
  sdStatus();
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
  if(NF1TDC>1) {
    f1EnableMultiBlock();
  } else {
    /*  f1EnableBusError(F1_SLOT); */
    f1GEnableBusError();
  }
#else /* Multiblock disabled? Enable the bus error on each F1TDC */
  f1DisableMultiBlock();
  f1GEnableBusError();
  /* This is a test: */
#endif /* End: !DISABLE_MULTIBLOCK_READOUT */
  f1GStatus(0);

  F1_SLOT = f1ID[0];
  f1GEnableData(F1_ALL_CHIPS); /* Enable data on all chips */
  printf("\n\n\n * * * * BEFORE Sending ReSync (to lock resolution) \n\n\n");
  f1GStatus(0);
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
    hcalClkIn(HCAL_LED_C_STEP);
  } else {
    hcalClkIn(0);
  }

#endif

#ifdef ENABLE_F1
  f1SDC_Sync();

  printf("\n\n\n * * * * AFTER  Sending ReSync (to lock resolution) \n\n\n");
  f1GStatus(0);
#endif

#ifdef TI_MASTER
  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);
#endif /* TI_MASTER */


  tiStatus(0);

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

  /* Interrupts/Polling enabled after conclusion of rocGo() */
}

void
rocEnd()
{

#ifdef ENABLE_FADC
  /* FADC Disable */
  faGDisable(0);

  /* FADC Event status - Is all data read out */
  faGStatus(0);
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

  tiStatus(0);
  sdStatus();
#ifdef ENABLE_HCAL_PULSER
  hcalClkIn(0); /* Turn off LEDs at the end */
#endif
  /* Turn off all output ports */
  tiSetOutputPort(0,0,0,0);

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

#ifdef ENABLE_FADC
#ifdef READOUT_FADC

  /* fADC250 Readout */
  BANKOPEN(BANK_FADC,BT_UI4,0);

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
  *dma_dabufp++ = LSWAP(tiGetIntCount());
  /* Check for valid data here */
  F1_SLOT = f1ID[0];

  int ii, islot;
  for(ii=0;ii<100;ii++)
    {
      datascan = f1Dready(F1_SLOT);
      if (datascan>0)
	{
	  break;
	}
    }

  if(datascan>0)
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
	  printf("ERROR: in transfer (event = %d), status = 0x%x\n", tiGetIntCount(),nwords);
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
      printf("ERROR: Data not ready in event %d, F1TDC slot %d\n",tiGetIntCount(),F1_SLOT);
      *dma_dabufp++ = LSWAP(0xda000bad);
    }
  *dma_dabufp++ = LSWAP(0xda0000ff); /* Event EOB */
  BANKCLOSE;

#ifdef DISABLE_MULTIBLOCK_READOUT
  /* vmeDmaConfig(2,5,1); */
#endif

#endif /* READOUT_F1 */
#endif /* ENABLE_F1 */

  /* Check for SYNC Event */
  if((tiGetSyncEventFlag() == 1) || (tiGetBlockBufferLevel() == 1))
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

#ifdef ENABLE_FADC
#ifdef READOUT_FADC

      for(ifa = 0; ifa < nfadc; ifa++)
	{
	  davail = faBready(faSlot(ifa));
	  if(davail > 0)
	    {
	      printf("%s: ERROR: fADC250 Data available after readout in SYNC event \n",
		     __func__);

	      while(faBready(faSlot(ifa)))
		{
		  vmeDmaFlush(faGetA32(faSlot(ifa)));
		}
	    }
	}

#endif /* READOUT_FADC */
#endif /* ENABLE_FADC */

    }

}

void
rocCleanup()
{

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
  compile-command: "make -k hcal2_list.so hcal2_slave_list.so"
  End:
 */
