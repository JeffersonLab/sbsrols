/*************************************************************************
 *
 *  vme_dual.c - TI-based triggered readout of VETROC + v792 QDC
 *    - This is a version of vme_block1.c extended by brads
 *    -- Bob M.  Here, I allow to turn off elements like VETROC.
 *    -- Evan    Here, I implement multiple VETROCs
 *  Note from vme_block1.c:
 *    - Ideally we'd use vetrocReadBlock instead of
 *      vetrocReadFIFO.
 *    -- Carlos A. Here, I set some flags for debugging
 */

#define MAX_EVENT_LENGTH 5*10240 /* in Bytes */
#define MAX_EVENT_POOL   400   /* in number of events */

#define BLOCKLEVEL 1          /* in events */
#define BUFFERLEVEL 1          /* in number of blocks */

#define MAXVETROCDATA 10000
#define FIRST_VETROC_SLOT 3
// assume VETROCs are in sequential slots
#define N_VETROCs 4 // was 1 (CA)

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

#define FIBER_LATENCY_OFFSET 0x4A  /* measured longest fiber length */

/*
  enable triggers (random or fixed rate) from internal pulser
 */
#define INTRANDOMPULSER
/* #define INTFIXEDPULSER */


#include "dmaBankTools.h"
#include "tiprimary_list.c" /* source required for CODA */

#ifdef USE_SCALER
#include "SIS3800.h"        /* scaler library */
#define SCAL_ADDR 0xab2000
static struct SIS3800CSREG *pScaler;
#endif


#include "vetrocLib.h"      /* VETROC library */
#include "gtpLib.h"         /* GTP library */
#include "c792Lib.h"        /* v792 library */

#define MAX_ADC_DATA 34

#include <time.h> /* CARLOS STUFF FOR DEBUGGING PURPOSES */

static unsigned int vetrocSlotMask=0;
int nvetroc=0;

#define use_vetroc 1


/********************************************************************************/
/* v792 setup */
/********************************************************************************/
#define use792      1
#define V792_NMOD   2 //was 3 (CA)
// V792_ADR1 is the HW address for the leftmost v792 in the crate
#define V792_ADR1   0x11510000 //was  0x11410000 (CA)
// V792_OFF is the increment for each subsequent v792 (ie. next module is 0x180000)
#define V792_OFF    0x00100000
/********************************************************************************/

/********************************************************************************
 ** F250 ADC requires 2,5,1 DMA transfers (well, A32 anyway), but v775 and v792
 **  can't talk in that mode
 ********************************************************************************/
/* FIXME: DMA switching in rocTrigger() is still a bit scrambled.  Moving
 * towards switching to relevant mode inside each module's readout block.
 * I've been assured that there is no meaningful performance cost for this.
 */
#define A24DMA     1,2,0
#define A24DMA2    1,5,2
#define A32DMA1    2,5,1
#define A32DMA2    2,5,2

/* function prototype */
void rocTrigger(int arg);

/****************************************
 *  DOWNLOAD
 *****************Front Panel TRG***********************/
void
rocDownload()
{


  int vi, i, res, mode, iflag;
  unsigned long laddr;

  /* Set up DMA */
  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */

  /* Stay in this mode by default, switch to A24DMA when talking to v792/v775 */
  vmeDmaConfig(A32DMA1);

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
  tiSetTriggerSource(TI_TRIGGER_FPTRG); /* FP TRG enabled */
#endif

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

  tiStatus(0);

  /* scalers */

#ifdef USE_SCALER
  pScaler = 0;

  printf("Setting up pScaler - address 0x%x\n",SCAL_ADDR);

  res = vmeBusToLocalAdrs(0x39,(char *)SCAL_ADDR,(char **)&laddr);

  if (res != 0) {
    printf("Scaler: ERROR:  vmeBusToLocalAdrs: scaler at offset 0x%x address= 0x%x\n",SCAL_ADDR,laddr);
  } else {
    pScaler = (struct SIS3800CSREG *)laddr;

    vmeWrite32(&pScaler->reset,1);
    vmeWrite32(&pScaler->csr,0x7000fd00);
    vmeWrite32(&pScaler->enclk,1);
    mode = 1;
    vmeWrite32(&pScaler->csr,0x00000C00);
    vmeWrite32(&pScaler->csr,mode<<2);
    vmeWrite32(&pScaler->clear,1);

    printf("Scaler: Have setup and cleared \n");


  }
#endif


/********************************************************************************/
/* for VETROC */
/********************************************************************************/

  if (use_vetroc) {

  int VETROC_SLOT = 0;
  vetrocSlotMask=0;

  for(vi=0;vi<N_VETROCs;vi++) {
    VETROC_SLOT = FIRST_VETROC_SLOT + vi;
    vetrocSlotMask |= (1<<VETROC_SLOT); /* Add it to the mask */
    printf("vetrocSlotMask=0x%08x\n",vetrocSlotMask);
  }

  /* 0 = software synch-reset, FP input 1, internal clock */
  iflag = 0x111;  /* VXS 0x111, FP 1  0x020;  */
  nvetroc = vetrocInit((FIRST_VETROC_SLOT<<19), 0x80000, N_VETROCs, iflag);

  if (nvetroc <= 0) {
    printf("ERROR: no VETROC !!! \n");
  }
  else {
    printf("Initialized %i VETROCs \n",nvetroc);
  }



  //First parameter shifts data left or right in TDC time. Second parameter sets upper bound for TDC window (0-XXXX ns).

  //Window must be <= Latency (lookBack) (CA)

    //unsigned int lookBack = 1400;//was 1700 (CA)
    //unsigned int windowWidth = 1800;//was 2500 (CA)

    //unsigned int lookBack = 600; // (Measured trigger-TDC difference ~ 570ns. -BY)
    //unsigned int windowWidth = 1000; //(TDC window size -BY)

    // This works, but not optimized (window too large) -BY
    // Pulser signal location: [262-319]
    //unsigned int lookBack = 1600;
    //unsigned int windowWidth = 3000;

    // This works (TDC will be ~260-321)
    //unsigned int lookBack = 1600;
    //unsigned int windowWidth = 1000;

    // THIS WORKS BETTER TDC will be ~160-200
    //unsigned int lookBack = 1500;
    //unsigned int windowWidth = 1000;

    // This is only for PMT trigger at weldment (+300ns to trigger) TDC will be ~350 -BY
    //  unsigned int lookBack = 2000;
    //  unsigned int windowWidth = 1000;

    // For reduced trigger delay (ADC cable length 100m)
    unsigned int lookBack = 1100;
    unsigned int windowWidth = 1000;

  for(vi=0;vi<N_VETROCs;vi++) {
    VETROC_SLOT = FIRST_VETROC_SLOT + vi;


  // vetrocGSetProcMode(2000,2000);
  // vetrocGSetProcMode(1200,400);       //test 2017-09-08
  // vetrocGSetProcMode(1200,1000);      //test 2018-02-16
    vetrocGSetProcMode(lookBack, windowWidth);       //test 2018-11-15

    vetrocSetBlockLevel(VETROC_SLOT, BLOCKLEVEL);
    vetrocTriggerPulseWidth(VETROC_SLOT, 8000);
    vetrocLinkReset(VETROC_SLOT);

    printf("VETROC. lookBack: %i ns; windowWidth: %i ns \n",lookBack, windowWidth);

    //vetrocReset(VETROC_SLOT, 0);
    //vetrocClear(VETROC_SLOT);

    //  vetrocSync(VETROC_SLOT);

    // Invert all TDC channels when using NINO discriminators as inputs
    //0xFFFF = inverted, 0X0000 = not inverted.
    vetrocChanInvert(VETROC_SLOT, 0, 0x0000);   //0-15
    vetrocChanInvert(VETROC_SLOT, 1, 0x0000);   //16-31
    vetrocChanInvert(VETROC_SLOT, 2, 0x0000);   //32-47
    vetrocChanInvert(VETROC_SLOT, 3, 0x0000);   //48-63
    vetrocChanInvert(VETROC_SLOT, 4, 0x0000);   //0x0001 inverts just channel 64.
    vetrocChanInvert(VETROC_SLOT, 5, 0x0000);   //80-95
    vetrocChanInvert(VETROC_SLOT, 6, 0x0000);   //96-111
    vetrocChanInvert(VETROC_SLOT, 7, 0x0000);   //112-127

    vetrocStatus(VETROC_SLOT,0);

    }
  }

/********************************************************************************/
/* for GTP */
/********************************************************************************/

/* Initialize GTP.  NULL (arg 2) specifies library to access GTP through TI (I2C)
     to determine it's network hostname */
  printf("***GTP initializing setup*** \n");//(CAG)

  gtpInit(0,NULL);

  /* Clock source is Internal before prestart */
/*  gtpSetClockSource(GTP_CLK_CTRL_P0); */
/*  gtpSetSyncSource(GTP_SD_SRC_SEL_SYNC); */
/*  gtpSetTrig1Source(GTP_SD_SRC_SEL_TRIG1); */
/*  gtpSetSource(2, 32); */
/*  gtpSetSource(2, 33); */
/*  gtpSetSource(2, 34); */
/*  gtpSetSource(2, 35); */
/*  gtpSetTriggerLatency(0); */

gtpSetTriggerClusterThreshold(1);
  /* mask = 1<<(PP# - 1)
     PP15 = 0x4000 (VME SLOT 3)
     PP13 = 0x1000 (VME SLOT 4)
     PP11 = 0x0400 ...
     PP9  = 0x0100
     PP7  = 0x0040
     PP5  = 0x0010
     PP3  = 0x0004
     PP1  = 0x0001
     PP2  = 0x0002 (VME SLOT 13)
     PP4  = 0x0008 (VME SLOT 14)
     PP6  = 0x0020 ...
     PP8  = 0x0080
     PP10 = 0x0200
     PP12 = 0x0800
     PP14 = 0x2000
     PP16 = 0x8000 (VME SLOT 20)
   */
  /*gtpEnableVmeSlotMask(vetrocSlotMask); - need to add this function to gtpLib, for no hardcoded to vme slot 6 (PP9) */
 gtpPayloadTriggerEnable(0x4000);

  printf("***Finished with gtp setup*** \n");



/********************************************************************************/
/* for v792 */
/********************************************************************************/

  if (use792>0) {

    printf("about to initialize CAEN 792 ADC \n");
    c792Init(V792_ADR1,V792_OFF,V792_NMOD,0);
    for (i=0; i<V792_NMOD ; i++) {
      c792Reset(i);
      c792Clear(i);
      c792ClearThresh(i);

      c792Sparse(i,0,0);    /* pedestal supression DISABLED */

      c792EnableBerr(i); /* for 32bit block transfer */

      c792Status(i,0,0);

      /* FIXME: make sure this is accurate */
      c792SetGeoAddress(i, i+1);
    }
    c792GStatus(0);
  }

  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{
#ifdef TI_MASTER
  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);

  tiSetBlockLimit(0);
#endif

  tiStatus(0);

  int vi, i, ii;

  if (use_vetroc) {
    for(vi=0;vi<N_VETROCs;vi++) {
      int VETROC_SLOT = FIRST_VETROC_SLOT + vi;
      vetrocReset(VETROC_SLOT, 0);
      vetrocClear(VETROC_SLOT);
      vetrocLinkReset(VETROC_SLOT);
    }
  }

  if (use792>0) {
    for (i=0; i<V792_NMOD ; i++) {
      /* Pedestal suppression set here (should be read in by file) */
      c792ClearThresh(i);
      //for (ii=0;ii<32;ii++) { c792p[0]->threshold[ii]=0x0; }
      c792Clear(i);
    }
  }

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

  /* Enable/Set Block Level on modules, if needed, here */
  int i, vi;

  if (use_vetroc) {
    for(vi=0;vi<N_VETROCs;vi++) {
      int VETROC_SLOT = FIRST_VETROC_SLOT + vi;
      vetrocStatus(VETROC_SLOT,0);
    }
    gtpStatus(0);
    /* Show the current block level */
    printf("%s: Current Block Level = %d\n",__FUNCTION__,tiGetCurrentBlockLevel());

    if (use792>0) {
      for (i=0; i<V792_NMOD ; i++) {
	c792Enable(i);
      }
    }

  }

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

  int i, islot;

  if (use792>0) {
    for (i=0; i<V792_NMOD ; i++) {
      c792Disable(i);
    }
    c792GStatus(0);
  }

  tiStatus(0);

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int evnum)
{

  time_t current_time; /* CARLOS STUFF FOR DEBUGGING PURPOSES */
  current_time = time(NULL); /* CARLOS STUFF FOR DEBUGGING PURPOSES */
  char* c_time_string; /* CARLOS STUFF FOR DEBUGGING PURPOSES */
  c_time_string = ctime(&current_time);  /* CARLOS STUFF FOR DEBUGGING PURPOSES */

  int i, wait, nevt, vi;
  int ii, islot, intcnt, type=0;
  int dCnt, idata, nwords=0;
  int itime, gbready, read_stat;
  unsigned int busytime,livetime;
  int blkcnt, evtcnt;
  static int defblk = 400;
  static int defevt = 20000;
  static int mycount=1;
  int debug=0;

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


  if (use_vetroc) {

    /* Bank for VETROC data */

    BANKOPEN(3,BT_UI4,blockLevel);
    vmeDmaConfig(A32DMA1);


    /* Check for valid data in VETROCs */

    read_stat = 0;

    for(itime=0; itime<1000; itime++) {
      gbready = vetrocGBready();
      read_stat = (gbready == vetrocSlotMask);
      if (read_stat>0) break;
    }

    if (debug) {
      printf("gbready = %d  read_stat = %d \n",gbready, read_stat);
      printf("vetrocSlotMask = %d \n",vetrocSlotMask);
    }

    if(read_stat>0) { /* read the data here */

      for(vi=0;vi<N_VETROCs;vi++) {
        int VETROC_SLOT = FIRST_VETROC_SLOT + vi;
        *dma_dabufp++ = LSWAP(0xb0b0b0b4+vi); // To Do: Make last hexit give N_VETROCs -- REM -- 2018-03-27
	//Could be as simple as +vi, but we need to check the decoder afterwards


        nwords = vetrocReadBlock(VETROC_SLOT,dma_dabufp, MAXVETROCDATA, 1);
        *dma_dabufp = LSWAP(nwords);
        dma_dabufp   += nwords; //empty histos no 0xffffffff
        //dma_dabufp   += nwords; //empty histos one 0xffffffff
        //*dma_dabufp++ = LSWAP(nwords);
        //dma_dabufp   += nwords; //histos look good two 0xffffffff
        //*dma_dabufp++ = LSWAP(read_stat);
        //dma_dabufp   += nwords; //histos look good three 0xffffffff
        //*dma_dabufp++ = LSWAP(vetrocDready(VETROC_SLOT,1));
        //dma_dabufp   += nwords; //histos look good four 0xffffffff
        //*dma_dabufp++ = LSWAP(vetrocDready(VETROC_SLOT,0));
        //dma_dabufp   += nwords; //histos look good 5 0xffffffff
      }

    } else {
      printf("ERROR (VETROC ROL): Data not ready in event %d\n at %s",tiGetIntCount(), c_time_string); /* CARLOS STUFF FOR DEBUGGING PURPOSES (time)*/
      *dma_dabufp++ = LSWAP(0xda000bad);
    }
    BANKCLOSE;
  }

  /* v792 Readout */
  if (use792>0) {
    unsigned int scanmask = 0, datascan = 0;
    int iadc;

    vmeDmaConfig(A24DMA);

    BANKOPEN(6,BT_UI4,blockLevel);
    /* Check if an Event is available */
    scanmask = c792ScanMask();
    datascan = c792GDReady(scanmask, 1000);

    if(datascan==scanmask)
      {
	for(iadc = 0; iadc < V792_NMOD; iadc++)
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
#ifdef TI_MASTER
		tiSetBlockLimit(20);
#endif
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
#ifdef TI_MASTER
	tiSetBlockLimit(20);
#endif

	for(iadc = 0; iadc < V792_NMOD; iadc++)
	  c792Clear(iadc);
      }

    BANKCLOSE;
  }

  BANKOPEN(5,BT_UI4,0);

  *dma_dabufp++ = LSWAP(0xfb1b1b1b);
  mycount++;
  *dma_dabufp++ = LSWAP(mycount);
  *dma_dabufp++ = LSWAP(tiGetIntCount());
  busytime = tiGetBusyTime();
  livetime = tiGetLiveTime();
  *dma_dabufp++ = LSWAP(busytime);
  *dma_dabufp++ = LSWAP(livetime);

#ifdef USE_SCALER
  if (pScaler) {

     *dma_dabufp++ = LSWAP(0xb0b0b444);
     *dma_dabufp++ = LSWAP(vmeRead32(&pScaler->readCounter[8]));
     *dma_dabufp++ = LSWAP(vmeRead32(&pScaler->readCounter[9]));
     *dma_dabufp++ = LSWAP(vmeRead32(&pScaler->readCounter[10]));

  }
#endif

  BANKCLOSE;

  tiSetOutputPort(0,0,0,0);

  //  blkcnt = vetrocDready(VETROC_SLOT,1);
  //  evtcnt = vetrocDready(VETROC_SLOT,0);
  //  Tried this; it does not work
  //  if (blkcnt < 50 || evtcnt < 1000) {
  //    vetrocWriteCnt(VETROC_SLOT, defblk, defevt);
  //  }


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
  compile-command: "make -k -B grinch_list.so grinch_slave_list.so"
  End:
 */
