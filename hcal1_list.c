/*************************************************************************
 *
 *  hcal1_list.c - Readout list for intelsbshcal1
 *                   Read out: TI, fadc250s
 *
 */

/* Decide which components we want to enable. Comment out to disable */
#define ENABLE_F1 /* F1TDCs */
#define ENABLE_FADC  /*  FADC 250s */
#define ENABLE_HCAL_PULSER /* Enable the HCAL Pulser logic */

/* Decide which readouts we want enabled */
#define READOUT_FADC   /* Readout FADCs */
#define READOUT_F1      /* Readout F1TDCs */

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

#include "dmaBankTools.h"
#include "tiprimary_list.c" /* source required for CODA */
#include "sdLib.h"

/* Library to pipe stdout to daLogMsg */
#include "dalmaRolLib.h"

/* Routines to add string buffers to banks */
#include "/adaqfs/home/sbs-onl/rol_common/rocUtils.c"

#ifdef ENABLE_FADC
#include "fadcLib.h"        /* library of FADC250 routines */
#include "fadc250Config.h"
#endif /* ENABLE_FADC */
#include "hcal_usrstrutils.c" /* So we can read from a flags file */

#ifdef ENABLE_HCAL_PULSER
#include "hcalLib.h"   /* Source of hcalClkIn is here (../linuxvme/include/hcalLib.h??  and ../linuxvme/hcal_pulser/hcalLib.c   */
#endif

#define BUFFERLEVEL 5

#ifdef ENABLE_FADC
/* FADC Library Variables */
extern int fadcA32Base, nfadc;
#define NFADC     16
/* Address of first fADC250 */
#define FADC_ADDR (3<<19)
/* Increment address to find next fADC250 */
#define FADC_INCR (1<<19)

#define FADC_READ_CONF_FILE {			\
    fadc250Config("");				\
    if(rol->usrConfig)				\
      fadc250Config(rol->usrConfig);		\
  }
#endif /* ENABLE_FADC */

/* HCal FLAGS */
#define NINPUTS 6 /* 6 trigger inputs on the new TI */
int flag_prescale[NINPUTS];
/* Flags for HCal pulser */
int flag_pulserTriggerInput;
int flag_LED_NSTEPS; /* Number of steps in sequence */
int flag_LED_PATT[128]; /* Step's LED pattern configuration */
int flag_LED_NPULSE[128]; /* Number of triggers in this step */
int flag_MODE=0;  /* 0 normal; 1 one box at a time (usual running mode); 2 all boxes at once (pulser only mode)*/
int jdum=0;  /* count 7 pairs of 0's after each value clocked in for MODE 1*/
int flag_VTP_readout = 0; /* 0: VME readout, 1: VTP readout */

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

#ifdef ENABLE_HCAL_PULSER
#define HCAL_LED_NLIST 3 /* Number of steps in sequence */
int flag_PULSER_ENABLED; /* Determined based on ps and nsteps */
unsigned int HCAL_LED_COUNT = 0;
unsigned int HCAL_LED_ITER=0;
unsigned int HCAL_LED_PATT;
unsigned int HCAL_LED_NPULSE;
#endif

int pulser_TRIG, cosmic_TRIG, hcal_sum_TRIG;


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

  /* Add HCAL2 to port 1*/
  tiAddSlave(1);
#else
  tiSetFiberSyncDelay(0x9);
#endif /* TI_MASTER */


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
  int ifa;
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
    printf("ps%d=%d \n",i+1,flag_prescale[i]);
    //printf("*******************ti_input_triggers = %d\n",ti_input_triggers);
  }
  printf("\n");
  tiEnableTSInput( ti_input_triggers );

  /* Setup the prescales */
  for(i = 0; i < NINPUTS; i++) {
    if(flag_prescale[i]>=0) {
      //tiSetInputPrescale(1<<i,flag_prescale[i]);
      //printf("****tiSetInputPrescale(%d,%d)\n",1<<i,flag_prescale[i]);
      tiSetInputPrescale(i+1,flag_prescale[i]);
      printf("****tiSetInputPrescale(%d,%d)\n",i+1,flag_prescale[i]);
    }
  }
#endif


#ifdef ENABLE_FADC
  /*****************
   *   FADC SETUP
   *****************/

  /* FADC Initialization flags */
  int iflag = FA_INIT_VXS_TRIG | FA_INIT_VXS_CLKSRC | FA_INIT_EXT_SYNCRESET;

  fadcA32Base = 0x09000000;

  vmeSetQuietFlag(1);
  faInit(FADC_ADDR, FADC_INCR, NFADC+2, iflag);
  vmeSetQuietFlag(0);

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

      if(flag_VTP_readout)
	{
	  faSetVXSReadout(faSlot(ifa), 1);
	}
    }

  /***************************************
   *   SD SETUP
   ***************************************/
  sdInit(0);   /* Initialize the SD library */
  sdSetActiveVmeSlots(faScanMask()); /* Use the fadcSlotMask to configure the SD */
#endif /* ENABLE_FADC */

#ifdef ENABLE_HCAL_PULSER
  if(flag_PULSER_ENABLED) {
    HCAL_LED_ITER=0;
    HCAL_LED_COUNT=0;
    printf("HCAL Pulser sequence: ");
    for(i = 0 ; i < flag_LED_NSTEPS; i++) {
      printf(" %d/%d",flag_LED_PATT[i],flag_LED_NPULSE[i]);
    }
    printf("\n");
    int idum;
    /* Clock in the first setting */
    HCAL_LED_PATT = flag_LED_PATT[0];
    HCAL_LED_NPULSE = flag_LED_NPULSE[0];
    for(idum=0;idum<14;idum++){hcalClkIn(0);}
    hcalClkIn(HCAL_LED_PATT);
    hcalClkIn(HCAL_LED_PATT);  /* into both boards of first box */
   /*   printf("Re-enabling pulser\n");*/
      hcalClkIn(69);  /* re-enable pulser (suspended during clock-in) */
      if(flag_MODE==1)jdum=0;
  } else {
    int idum;
    printf("Pulser_enabled is *False*");
    for(idum=0;idum<16;idum++){hcalClkIn(0);}
  }

#endif

#ifdef TI_MASTER
  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);
#endif /* TI_MASTER */



  DALMAGO;
  tiStatus(1);
  sdStatus();
#ifdef ENABLE_FADC
  faGStatus(0);
#endif /* ENABLE_FADC */
  DALMASTOP;

  /* Add configuration files to user event type 137 */
  int maxsize = MAX_EVENT_LENGTH-128, inum = 0, nwords = 0;

  if(rol->usrConfig)
    {
      UEOPEN(137, BT_BANK, 0);
      nwords = rocFile2Bank(rol->usrConfig,
			    (uint8_t *)rol->dabufp,
			    ROCID, inum++, maxsize);

      if(nwords > 0)
	rol->dabufp += nwords;

      UECLOSE;
    }

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

  DALMAGO;
#ifdef ENABLE_FADC
  /* FADC Event status - Is all data read out */
  faGStatus(0);
#endif /* ENABLE_FADC */
  tiStatus(1);
  sdStatus();
  DALMASTOP;

#ifdef ENABLE_HCAL_PULSER
  int idum;
    for(idum=0;idum<16;idum++){hcalClkIn(0);} /* Turn off LEDs at the end */
    printf("Cleared pulser boxes\n");
#endif
  /* Turn off all output ports */
    /*BQ  tiSetOutputPort(0,0,0,0);   */

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

 int TRIG_word=*(dma_dabufp+2);
 /* printf("TRIG_word %8X %8x %2X\n",TRIG_word,TRIG_word&0XFF000000,(TRIG_word&0XFF000000)>>24);*/
 /*For TS triggers, first Hex digit is 2 (or at least has 2 set?) for front
panel triggers and 2nd Hex digit is number of input
1  Shower
2  Hcal Sum
3  Coincidence
4  HRS
5  Grinch LED
6  HCal LED pulser
7  HCal cosmic paddle     */
 int TRIG_type=(TRIG_word&0x0F000000)>>24;
 pulser_TRIG=(TRIG_type==6);
 cosmic_TRIG=(TRIG_type==7);
 hcal_sum_TRIG=(TRIG_type==2);
 /* printf("TRIG_type: %X  pulser: %i   cosmic: %i   sum:  %i\n",TRIG_type,pulser_TRIG,cosmic_TRIG,hcal_sum_TRIG);*/

 /*Next 3 lines for TI in Roc 16 crate
 pulser_TRIG=(TRIG_word0X2000000)>>25;
 cosmic_TRIG=(TRIG_word&0X1000000)>>24;
 hcal_sum_TRIG=(TRIG_word&0X4000000)>>26;*/

 /* if(pulser_TRIG){printf("pulser\n");}
 if(cosmic_TRIG){printf("cosmic\n");}
 if(hcal_sum_TRIG){printf("hcal_sum\n");}*/
  /*
   int idum;
    printf("TI says trigger block length is %d \n",dCnt);
    for(idum=0;idum<dCnt;idum++){printf("%i  %9X\n",idum,*(dma_dabufp+idum));}

    printf("Trig wd 2:  %16X\n",*(dma_dabufp+2));

  */

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

  if(flag_VTP_readout == 0)
    {
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
    }

#endif /* READOUT_FADC */
#endif /* ENABLE_FADC */


  /* Check for SYNC Event */
  if(tiGetSyncEventFlag() == 1)
    {
      int iflush = 0, maxflush = 10;
      /* Check for data available */
      int davail = tiBReady();
      if(davail > 0)
	{
	  daLogMsg("ERROR","TI Data available (%d) after SYNC event \n",
		   davail);

	  iflush = 0;
	  while(tiBReady() && (++iflush < maxflush))
	    {
	      vmeDmaFlush(tiGetAdr32());
	    }
	}

#ifdef ENABLE_FADC
#ifdef READOUT_FADC
      if(flag_VTP_readout == 0)
	{
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
	}
#endif /* READOUT_FADC */
#endif /* ENABLE_FADC */

    }

#ifdef ENABLE_HCAL_PULSER
  if(flag_PULSER_ENABLED && pulser_TRIG)
{
    HCAL_LED_COUNT++;
    BANKOPEN(BANK_HCAL_PULSER,BT_UI4,0);
    /**dma_dabufp++ = LSWAP(tiGetIntCount());*/
        *dma_dabufp++ = LSWAP(HCAL_LED_ITER);
    *dma_dabufp++ = LSWAP(HCAL_LED_PATT);
    *dma_dabufp++ = LSWAP(HCAL_LED_COUNT);
    *dma_dabufp++ = LSWAP((HCAL_LED_ITER<<22)|(HCAL_LED_PATT<<16)|HCAL_LED_COUNT);
    int idum;
    if(flag_MODE==0){idum=0;}else if(flag_MODE==1){idum=jdum+1;}else{idum=9;}
    *dma_dabufp++ = LSWAP(idum);
    BANKCLOSE;

    /* Run the HCAL pulser clock-in code */
 if(flag_MODE==0){
    if(HCAL_LED_COUNT>=HCAL_LED_NPULSE)
    {
      HCAL_LED_COUNT=0;
      HCAL_LED_ITER++;
      if(HCAL_LED_ITER>=flag_LED_NSTEPS) {
        HCAL_LED_ITER=0;
      }
      HCAL_LED_PATT=flag_LED_PATT[HCAL_LED_ITER];
      HCAL_LED_NPULSE=flag_LED_NPULSE[HCAL_LED_ITER];
      printf("Clocking in HCAL LED: %2d, %2d (tircount:%d)\n",
        HCAL_LED_ITER,HCAL_LED_PATT,tiGetIntCount());
      hcalClkIn(HCAL_LED_PATT);
      hcalClkIn(HCAL_LED_PATT);   /* into both boards of one box */
     /* printf("Re-enabling pulser\n");  */
       hcalClkIn(69);  /* re-enable pulser (suspended during clock-in */
    }
 }else if(flag_MODE==1){
/* mode 1, one box at a time, the usual running mode */
    if(HCAL_LED_COUNT>=HCAL_LED_NPULSE)
    {
    if(jdum<7){jdum++;
               HCAL_LED_COUNT=0;
               hcalClkIn(0);
               hcalClkIn(0);
	       hcalClkIn(69);  /* move non-zero values along to next box.. and re-enable */
              }else{
      jdum=0;
      HCAL_LED_COUNT=0;
      HCAL_LED_ITER++;
      if(HCAL_LED_ITER>=flag_LED_NSTEPS) {
        HCAL_LED_ITER=0;
      }
      HCAL_LED_PATT=flag_LED_PATT[HCAL_LED_ITER];
      HCAL_LED_NPULSE=flag_LED_NPULSE[HCAL_LED_ITER];
      printf("Clocking in HCAL LED: %2d, %2d (tircount:%d)\n",
        HCAL_LED_ITER,HCAL_LED_PATT,tiGetIntCount());
      hcalClkIn(HCAL_LED_PATT);
      hcalClkIn(HCAL_LED_PATT);   /* into both boards of one box */
     /* printf("Re-enabling pulser\n");  */
       hcalClkIn(69);  /* re-enable pulser (suspended during clock-in */
       		   }
    }
 }else{
    if(HCAL_LED_COUNT>=HCAL_LED_NPULSE)
    {
      HCAL_LED_COUNT=0;
      HCAL_LED_ITER++;
      if(HCAL_LED_ITER>=flag_LED_NSTEPS) {
        HCAL_LED_ITER=0;
      }
      HCAL_LED_PATT=flag_LED_PATT[HCAL_LED_ITER];
      HCAL_LED_NPULSE=flag_LED_NPULSE[HCAL_LED_ITER];
      printf("Clocking in HCAL LED: %2d, %2d (tircount:%d)\n",
        HCAL_LED_ITER,HCAL_LED_PATT,tiGetIntCount());
      int idum;
      for(idum=0;idum<16;idum++){hcalClkIn(HCAL_LED_PATT);}  /* Clock into all 16 boards */
     /* printf("Re-enabling pulser\n");  */
       hcalClkIn(69);  /* re-enable pulser (suspended during clock-in */
    }
 }
}
#endif

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

}

/*  Read the user flags/configuration file. */
void readUserFlags()
{
  int i;
  char pstext[10];

  printf("Reading user flags file.");
  init_strings();
  flag_pulserTriggerInput=getint("pulserTrigger");

  /* REMOVED : Read the FADC configuration */
  /* This is set in ~/cfg/intelsbshal1.cnf */
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

  /* VTP readout */
  flag_VTP_readout = getint("vtp");
  printf("vtp=%d: %s\n",
	 flag_VTP_readout,
	 (flag_VTP_readout) ? "VTP readout" : "VME readout");

#ifdef ENABLE_HCAL_PULSER
  /* Read the hcal pulser step info */
  /* disable if prescale=-1 */
  if(flag_pulserTriggerInput >= 0 && flag_pulserTriggerInput <6) {
    flag_PULSER_ENABLED = flag_prescale[flag_pulserTriggerInput]+1;
    /*	  printf("flag_prescale[%d]=%d\n",flag_pulserTriggerInput,flag_prescale[flag_pulserTriggerInput]);  */
    flag_MODE=getint("LED_pulser_MODE");
    if(flag_PULSER_ENABLED) {
      flag_LED_NSTEPS=getint("pulser_nsteps");
      char ptext[50];
      for(i = 0; i < flag_LED_NSTEPS; i++) {
        sprintf(ptext,"pulser_step%d",i);
        flag_LED_PATT[i]  = getint(ptext);
        sprintf(ptext,"pulser_nstep%d",i);
        flag_LED_NPULSE[i] = getint(ptext);
      }
    }
  } else {
    flag_PULSER_ENABLED=0;
  }
#endif
}

/*
  Local Variables:
  compile-command: "make -k hcal1_list.so hcal1_slave5_list.so"
  End:
 */
