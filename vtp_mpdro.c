/*************************************************************************
 *
 *  vtp_mpdro_list.c - Library of routines for readout and buffering of
 *                events using a VTP as a readout path for the MPD
 *
 *     To be used with the vtp HW ROC readout list
 *
 */
#define VTP
#include "vtp.h"
#include "mpdLib.h"
#include "mpdConfig.h"
#include "vtpMpdConfig.h"

extern pthread_mutex_t   vtpMutex;
#define VLOCK     if(pthread_mutex_lock(&vtpMutex)<0) perror("pthread_mutex_lock");
#define VUNLOCK   if(pthread_mutex_unlock(&vtpMutex)<0) perror("pthread_mutex_unlock");
extern volatile ZYNC_REGS *vtp;

/*
  Global to configure pedestal subtraction mode
      0 : subtraction mode DISABLED
      1 : subtraction mode ENABLED 

  Set with vtpSetPedSubtractionMode(int enable)
*/
int vtpPedSubtractionMode = 1;
void vtpSetPedSubtractionMode(int enable); // routine prototype

/* vtp defs */
/* extern int vtpSoftReset(int id); */

/*MPD Definitions*/
extern uint32_t mpdRead32(volatile uint32_t * reg);
extern int I2C_SendStop(int id);

int fnMPD=0;
int mpd_evt[22];

extern volatile struct mpd_struct *MPDp[(MPD_MAX_BOARDS+1)]; /* pointers to MPD memory map */

// End of MPD definition

/* Buffer to store daLogMsg's */
int dalma_rval, dalma_tot;

#define DALMA_INIT {				\
  dalma_tot = 0;				\
  bufp = (char *) &(dalmabuffer[0]);		\
  dalma_rval = sprintf (bufp,"\n");					\
  if(dalma_rval > 0) {bufp += dalma_rval; dalma_tot += dalma_rval;}}

#define DALMA_MSG(x...)	{						\
  dalma_rval = sprintf(bufp, x);					\
  if(dalma_rval > 0) {bufp += dalma_rval; dalma_tot += dalma_rval;}}

#define DALMA_LOG {				\
    if(dalma_tot > 1)				\
      daLogMsg("INFO",dalmabuffer);}

char *dalmabuffer;
char *apvbuffer;
char *bufp;


int
vtpMpdDalogStatus(unsigned int fmask)
{
  MPDFIBER_REGS *mr;
  int impd=0;

  mr = (MPDFIBER_REGS *)malloc(32*sizeof(MPDFIBER_REGS));
  printf("fmask = 0x%08x\n", fmask);

  for(impd=0; impd<32; impd++)
    {
      mr[impd].gtx_ctrl   = vtp->v7.mpdFiber[impd].gtx_ctrl;
      mr[impd].gtx_status = vtp->v7.mpdFiber[impd].gtx_status;
      mr[impd].eb_ctrl = vtp->v7.mpdFiber[impd].eb_ctrl;
    }

  DALMA_INIT;

  DALMA_MSG("\n");
  DALMA_MSG("                           MPD Settings and Status\n\n");
  DALMA_MSG("                  Channel  TX   ResetDone   -------ERRORS------     Event\n");
  DALMA_MSG("MPD  Ctrl Status    Up    Lock   TX  RX     HARD   FRAME    CNT     Builder\n");
  DALMA_MSG("--------------------------------------------------------------------------------\n");

  for(impd=0; impd<32; impd++)
    {
      if( (impd % 8) == 0 )
	{
	  DALMA_LOG;
	  DALMA_INIT;
	}
      if( (fmask & (1 << impd)) == 0) continue;

      DALMA_MSG("%2d   ",impd);

      DALMA_MSG("%04x   ",mr[impd].gtx_ctrl);

      DALMA_MSG("%04x   ",mr[impd].gtx_status & 0xffff);

      DALMA_MSG("%s     ",(mr[impd].gtx_status & MPD_GTX_STATUS_FIBER_CHANNEL_UP)?" UP ":"DOWN");

      DALMA_MSG("%s     ",(mr[impd].gtx_status & MPD_GTX_STATUS_TX_LOCK)?"1":"0");

      DALMA_MSG("%s   ",(mr[impd].gtx_status & MPD_GTX_STATUS_TX_RESETDONE)?"1":"0");

      DALMA_MSG("%s      ",(mr[impd].gtx_status & MPD_GTX_STATUS_RX_RESETDONE)?"1":"0");

      DALMA_MSG("%s     ",(mr[impd].gtx_status & MPD_GTX_STATUS_FIBER_HARD_ERR)?"ERR":"---");

      DALMA_MSG("%s    ",(mr[impd].gtx_status & MPD_GTX_STATUS_FIBER_FRAME_ERR)?"ERR":"---");

      if(mr[impd].gtx_status & MPD_GTX_STATUS_FIBER_ERR_CNT)
	{
	  DALMA_MSG("%3d     ",(mr[impd].gtx_status & MPD_GTX_STATUS_FIBER_FRAME_ERR)>>8);
	}
      else
	DALMA_MSG("---     ");

      DALMA_MSG("%s",(mr[impd].eb_ctrl & MPD_EBCTRL_ENABLE)?"ENABLED ":"DISABLED");

      DALMA_MSG("\n");

    }

  DALMA_LOG;

  if(mr)
    free(mr);

  return OK;
}

void
vtpPrintMPD_OB_STATUS(int dalogFlag)
{
  struct output_buffer_struct r;
  int k;
  int rval = 0;

  DALMA_INIT;

  DALMA_MSG("                              Output Buffer Status\n");
  DALMA_MSG("           Event Builder\n");
  DALMA_MSG("         OutFIFO   Full Flags       \n");
  DALMA_MSG("Slot   nWrds  F E    O E C T   Blks    Events     Trigs    Missed  Incoming\n");
  DALMA_MSG("--------------------------------------------------------------------------------\n");

  for (k=0;k<fnMPD;k++) { // only active mpd set
    r.evb_fifo_word_count = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.evb_fifo_word_count);
    r.block_count = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.block_count);
    r.event_count = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.event_count);
    r.trigger_count = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.trigger_count);
    r.missed_trigger = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.missed_trigger);
    r.incoming_trigger = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.incoming_trigger);

    if( (k % 8) == 0 )
      {
	DALMA_LOG;
	DALMA_INIT;
      }


    DALMA_MSG(" %2d     ", mpdSlot(k));

    DALMA_MSG("%4d  ",
		    r.evb_fifo_word_count & 0xFFFF);

    DALMA_MSG("%d %d    ",
		    (r.evb_fifo_word_count & (1<<17) ? 1 : 0),
		    (r.evb_fifo_word_count & (1<<16) ? 1 : 0));

    DALMA_MSG("%d %d %d %d    ",
		    (r.evb_fifo_word_count & (1<<27) ? 1 : 0),
		    (r.evb_fifo_word_count & (1<<26) ? 1 : 0),
		    (r.evb_fifo_word_count & (1<<25) ? 1 : 0),
		    (r.evb_fifo_word_count & (1<<24) ? 1 : 0));

    DALMA_MSG("%3d  ",
		    r.block_count & 0xFF);

    DALMA_MSG("%8d  ",
		    r.event_count & 0xFFFFFF);

    DALMA_MSG("%8d  ",
		    r.trigger_count);

    DALMA_MSG("%8d  ",
		    r.missed_trigger);

    DALMA_MSG("%8d",
		    r.incoming_trigger);

    DALMA_MSG("\n");

  }

  if(dalogFlag)
    {
      DALMA_LOG;
    }
  else
    {
      printf("%s",dalmabuffer);
      printf("\n");
    }


  DALMA_INIT;

  DALMA_MSG("       ---------------------- SDRAM --------------------   -- OBUF -    Latched\n");
  DALMA_MSG("                  FIFO Addr                        Word         Word   APV  PROC\n");
  DALMA_MSG("Slot          WR OK         RD OK     Overrun      Count   F E Count  Full  Full\n");
  DALMA_MSG("--------------------------------------------------------------------------------\n");

  for (k=0;k<fnMPD;k++) { // only active mpd set

    r.sdram_flag_wc = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.sdram_flag_wc);
    r.output_buffer_flag_wc = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.output_buffer_flag_wc);

    if( (k % 8) == 0 )
      {
	DALMA_LOG;
	DALMA_INIT;
      }

    DALMA_MSG(" %2d    ", mpdSlot(k));

    DALMA_MSG("0x%7x  %d  ",
		    r.sdram_fifo_wr_addr & 0x1FFFFFF,
		    (r.sdram_fifo_wr_addr & (1<<31)) ? 1 : 0
		    );

    DALMA_MSG("0x%7x  %d         ",
		    r.sdram_fifo_rd_addr & 0x1FFFFFF,
		    (r.sdram_fifo_rd_addr & (1<<31)) ? 1 : 0
		    );

    DALMA_MSG("%s  ",
		    (r.sdram_flag_wc & (1<<31)) ? "YES" : " no"
		    );

    DALMA_MSG("0x%7x   ",
		    r.sdram_flag_wc & 0x1FFFFFF
		    );

    DALMA_MSG("%d %d  %4d  ",
		    (r.output_buffer_flag_wc & (1<<31) ) ? 1 : 0,
		    (r.output_buffer_flag_wc & (1<<30) ) ? 1 : 0,
		    r.output_buffer_flag_wc & 0x1FFF
		    );

    DALMA_MSG("0x%8x",
		    r.latched_full
		    );

    DALMA_MSG("\n");
  }

  if(dalogFlag)
    {
      DALMA_LOG;
    }
  else
    {
      printf("%s",dalmabuffer);
      printf("\n");
    }
}

void vtp_mpd_setup()
{
  static int just_once = 0;
  if( (just_once) > 0)
    {
      daLogMsg("INFO", " %d run%s since last vtp_mpd_setup\n",
	       just_once, (just_once>1)?"s":"");
      daLogMsg("INFO",apvbuffer);
      just_once++;
      return;
    }

  /*****************
   *   VTP SETUP
   *****************/
  int iFlag = 0;
  int vtpFiberBit = 0;
  uint32_t vtpFiberMaskToInit;


  if(vtpMpdConfigInit("/home/sbs-onl/cfg/vtp_config.cfg") == ERROR)
    {
      daLogMsg("ERROR","Error in configuration file");
      return;
    }

  vtpMpdConfigLoad();

  vtpMpdFiberReset();
  vtpMpdFiberLinkReset(0xffffffff);


  vtpMpdDisable(0xffffffff);

  vtpFiberMaskToInit = mpdGetVTPFiberMask();
  printf("VTP fiber mask: 0x%08x", vtpFiberMaskToInit);

  while(vtpFiberMaskToInit != 0){
    if((vtpFiberMaskToInit & 0x1) == 1)
      vtpMpdEnable( 0x1 << vtpFiberBit);
    vtpFiberMaskToInit = vtpFiberMaskToInit >> 1;
    ++vtpFiberBit;
  }


  int build_all_samples = 0;
  //1 => For pedestal run, will write ADC samples from the APV (i.e. disable zero suppression)
  //0 => will apply the threshold and peak position logic to decide if data is written to the event (i.e. zero suppression enabled)

  int build_debug_headers = 0;
  //1 => will write extra debugging info headers about the common-mode processing (which APV chip reported data, avg A/B values and counts for each APV)
  //0 => disables extra debug info headers
  int enable_cm = 1;
  //1 => enables the common-mode subtraction logic
  //0 => disables the common-mode subtraction logic (so raw ADC samples only have the channel offset applied)
  int noprocessing_prescale = 100;
  //0 => prescaling disabled (all events are processed)
  //1-65535 => every Nth event has common-mode subtract and zero suppression disabled

  vtpMpdEbSetFlags(build_all_samples, build_debug_headers,
		   enable_cm, noprocessing_prescale);

  //char* mpdSlot[10],apvId[10],cModeMin[10],cModeMax[10];
  int apvId, cModeMin, cModeMax, cModeRocId;
  int fiberID = -1, last_mpdSlot = -1;
  //FILE *fcommon   = fopen("/home/sbs-onl/cfg/CommonModeRange.txt","r");
  //FILE *fcommon   = NULL;
  FILE *fcommon   = fopen("/home/sbs-onl/cfg/CommonModeRange_220.txt","r");

  //valid pedestal file => will load APV offset file and subtract from APV samples
  //NULL => will load 0's for all APV offsets
  //FILE *fpedestal = fopen("/home/sbs-onl/cfg/pedestal.txt","r");
  //FILE *fpedestal = fopen("/home/sbs-onl/cfg/pedestal_test.txt","r");    //Test file with offset set to -1000 in fiber 15, apv 11, channel 10
  FILE *fpedestal = fopen("/home/sbs-onl/cfg/gem_ped_220.dat","r");//all GEMs
  //FILE *fpedestal = NULL;

  // Load pedestal & threshold file settings
  int stripNo;
  float ped_offset, ped_rms;
  char buf[10];
  int i1, i2, i3, n;
  char *line_ptr = NULL;
  size_t line_len;
  fiberID = -1, last_mpdSlot = -1;
  if((fpedestal==NULL) || (vtpPedSubtractionMode == 0)) {

    /* Correct vtpPedSubtractionMode if fpedestal == NULL */
    if(fpedestal==NULL) vtpPedSubtractionMode = 0;

    for(fiberID=0; fiberID<16; fiberID++)
      {
	for(apvId=0; apvId<15; apvId++)
          {
            for(stripNo=0; stripNo<128; stripNo++)
	      {
		vtpMpdSetApvOffset(fiberID, apvId, stripNo, 0);
		vtpMpdSetApvThreshold(fiberID, apvId, stripNo, 0);
	      }
          }
      }
    printf("no pedestal file or vtpPedSubtractionmode == 0\n");
  }else{
    printf("trying to read pedestal \n");

    while(!feof(fpedestal))
      {
	getline(&line_ptr, &line_len, fpedestal);

	n = sscanf(line_ptr, "%10s %d %d %d", buf, &i1, &i2, &i3);
	if( (n == 4) && !strcmp("APV", buf))
          {
            cModeRocId = i1; 
            fiberID = i2;
            apvId = i3;
            continue;
          }


	n = sscanf(line_ptr, "%d %f %f", &stripNo, &ped_offset, &ped_rms);

      	// TODO: need to replace vtpRocId with the ROC ID for this VTP to see if settings are for us
        //if(cModeRocId != vtpRocId)
	//  continue;
	//
	if(n == 3)
          {
	    //            printf(" fiberID: %2d, apvId %2d, stripNo: %3d, ped_offset: %4.0f ped_rms: %4.0f \n", fiberID, apvId, stripNo, ped_offset, ped_rms);
            vtpMpdSetApvOffset(fiberID, apvId, stripNo, (int)ped_offset);
	    vtpMpdSetApvThreshold(fiberID, apvId, stripNo, 5*(int)ped_rms);
          }
      }
    fclose(fpedestal);
  }

  // Load common-mode file settings
  if(fcommon==NULL){
    printf("no commonMode file\n");
  }else{
    printf("trying to read commonMode\n");
    while(fscanf(fcommon, "%d %d %d %d %d", &cModeRocId, &fiberID, &apvId, &cModeMin, &cModeMax)==5){
      printf("fiberID %d %d %d %d %d \n", cModeRocId, fiberID, apvId, cModeMin, cModeMax);

      //	  cModeMin = 200;
      //	  cModeMax = 800;
      //	  cModeMin = 0;
      //	  cModeMax = 4095;

      // TODO: need to replace vtpRocId with the ROC ID for this VTP to see if settings are for us
      //if(cModeRocId != vtpRocId)
      //  continue;

      vtpMpdSetAvg(fiberID, apvId, cModeMin, cModeMax);
    }
    fclose(fcommon);
  }

  vtpRocMigReset(1);
  vtpRocMigReset(0);
  //  vtpPrintMigStatus(0);

  /*****************
   *   MPD SETUP
   *****************/
  int rval = OK;
  unsigned int errSlotMask = 0;
  /* Index is the mpd / fiber... value mask if ADCs with APV config errors */
  uint32_t apvConfigErrorMask[32];
  uint32_t apvErrorTypeMask[32]; /* 0 : mpd init, 1: apv not found , 2: config */

  memset(apvConfigErrorMask, 0 , sizeof(apvConfigErrorMask));
  memset(apvErrorTypeMask, 0 , sizeof(apvErrorTypeMask));

  mpdSetPrintDebug(0);

  // discover MPDs and initialize memory mapping

  // In VTP mode, par1(fiber mask) and par3(number of mpds) are not used in mpdInit(par1, par2, par3, par4)
  // Instead, they come from the configuration file
  unsigned int chanmask = vtpMpdGetChanUpMask();

  chanmask = mpdGetVTPFiberMask();
  mpdInitVTP(chanmask, MPD_INIT_FIBER_MODE | MPD_INIT_NO_CONFIG_FILE_CHECK);
  /* mpdInit(chanmask, 0,32,MPD_INIT_FIBER_MODE | MPD_INIT_NO_CONFIG_FILE_CHECK); */
  fnMPD = mpdGetNumberMPD();


  //fnMPD = 1;
  if (fnMPD<=0) { // test all possible vme slot ?
    printf("ERR: no MPD discovered, cannot continue\n");
    return;
  }

  printf(" MPD discovered = %d\n",fnMPD);

  // APV configuration on all active MPDs
  int error_status = OK;
  int k, i;
  for (k=0;k<fnMPD;k++) { // only active mpd set
    i = mpdSlot(k);

    int try_cnt = 0;

    /* mpdHISTO_MemTest(i); */

  retry:
    error_status = OK;

    printf(" Try initialize I2C mpd in slot %d\n",i);
    if (mpdI2C_Init(i) != OK) {
      printf("WRN: I2C fails on MPD %d\n",i);
    }

    printf("Try APV discovery and init on MPD slot %d\n",i);
    if (mpdAPV_Scan(i)<=0 && try_cnt < 3 ) { // no apd found, skip next
      try_cnt++;
      printf("failing retrying\n");
      goto retry;
    }



    if( try_cnt == 3 )
      {
	printf("APV blind scan failed for %d TIMES !!!!\n\n", try_cnt);
	errSlotMask |= (1 << i);
	continue;
      }

    printf(" - APV Reset\n");
    fflush(stdout);
    if (mpdI2C_ApvReset(i) != OK)
      {
	printf(" * * FAILED\n");
	error_status = ERROR;
	errSlotMask |= (1 << i);
      }

    usleep(10);
    I2C_SendStop(i);


    // board configuration (APV-ADC clocks phase)
    printf("Do DELAY setting on MPD slot %d\n",i);
    mpdDELAY25_Set(i, mpdGetAdcClockPhase(i,0), mpdGetAdcClockPhase(i,1));



    // apv configuration
    printf("Configure %d APVs on MPD slot %d\n",mpdGetNumberAPV(i),i);


    // apv configuration
    mpdSetPrintDebug(0);
    printf(" - Configure Individual APVs\n");
    printf(" - - ");
    fflush(stdout);
    int itry, badTry = 0, iapv, saveError = error_status;
    error_status = OK;
    for (itry = 0; itry < 3; itry++)
      {
	if(badTry)
	  {
	    printf(" ******** RETRY ********\n");
	    printf(" - - ");
	    fflush(stdout);
	    error_status = OK;
	  }
	badTry = 0;
	for (iapv = 0; iapv < mpdGetNumberAPV(i); iapv++)
	  {
	    printf("%2d ", iapv);
	    fflush(stdout);

	    if (mpdAPV_Config(i, iapv) != OK)
	      {
		printf(" * * FAILED for APV %2d\n", iapv);
		if(iapv < (mpdGetNumberAPV(i) - 1))
		  printf(" - - ");
		fflush(stdout);
		error_status = ERROR;
		apvConfigErrorMask[i] |= (1 << mpdApvGetAdc(i,iapv));
		apvErrorTypeMask[i] |= (1 << 2);
		badTry = 1;
	      }
	    else
	      {
		apvConfigErrorMask[i] &= ~(1 << mpdApvGetAdc(i,iapv));
	      }
	  }
	printf("\n");
	fflush(stdout);
	if(badTry)
	  {
	    printf(" ***** APV RESET *****\n");
	    fflush(stdout);
	    mpdI2C_ApvReset(i);
	  }
	else
	  {
	    if(itry > 0)
	      {
		printf(" ****** SUCCESS!!!! ******\n");
		error_status = OK;
		apvConfigErrorMask[i] = 0;
		apvErrorTypeMask[i] &= ~(1 << 2);
		fflush(stdout);
	      }
	    break;
	  }

      }

    error_status |= saveError;

    if(error_status == ERROR)
      errSlotMask |= (1 << i);

    // configure adc on MPD
    printf("Configure ADC on MPD slot %d\n",i);
    mpdADS5281_Config(i);

    // configure fir
    // not implemented yet

    // 101 reset on the APV
    printf("Do 101 Reset on MPD slot %d\n",i);
    mpdAPV_Reset101(i);

    // <- MPD+APV initialization ends here
    sleep(1);
  } // end loop on mpds
  //END of MPD configure

  // summary report
  bufp = (char *) &(apvbuffer[0]);

  rval = sprintf(bufp, "\n");
  if(rval > 0)
    bufp += rval;
  rval = sprintf(bufp, "Configured APVs (ADC 15 ... 0)            --------------------ERRORS------------------\n");
  if(rval > 0)
    bufp += rval;

  int ibit;
  int ifiber, id, iapv;
  for (ifiber = 0; ifiber < 32; ifiber++)
    {
      if(ifiber == 16)
	{
	  daLogMsg("INFO",apvbuffer);
	  bufp = (char *) &(apvbuffer[0]);
	}

      id = ifiber;

      if( ((1 << id) & mpdGetVTPFiberMask()) == 0)
	continue;


      /* Build the ADCmask of those in the config file */
      uint32_t configAdcMask = 0;
      for (iapv = 0; iapv < mpdGetNumberAPV(id); iapv++)
	{
	  if(mpdApvGetAdc(id,iapv) > -1)
	    {
	      configAdcMask |= (1 << mpdApvGetAdc(id,iapv));
	      apvErrorTypeMask[ifiber] |= (1 << 1);
	    }
	}

      if(mpdGetFpgaRevision(ifiber) == 0)
	apvErrorTypeMask[ifiber] = (1 << 0);

      /* if (mpdGetApvEnableMask(id) != 0) */
	{
	  rval = sprintf(bufp, "  MPD %2d : ", id);
	  if(rval > 0)
	    bufp += rval;
	  iapv = 0;
	  for (ibit = 15; ibit >= 0; ibit--)
	    {
	      if (((ibit + 1) % 4) == 0)
		{
		  rval = sprintf(bufp, " ");
		  if(rval > 0)
		    bufp += rval;
		}
	      if (mpdGetApvEnableMask(id) & (1 << ibit))
		{
		  if(apvConfigErrorMask[id] & (1 << ibit))
		    {
		      rval = sprintf(bufp, "C");
		    }
		  else
		    {
		      rval = sprintf(bufp, "1");
		    }
		  if(rval > 0)
		    bufp += rval;
		  iapv++;
		}
	      else if(configAdcMask & (1 << ibit))
		{
		  rval = sprintf(bufp, "E");
		  errSlotMask |= (1 << id);
		  if(rval > 0)
		    bufp += rval;
		}
	      else
		{
		  rval = sprintf(bufp, ".");
		  if(rval > 0)
		    bufp += rval;
		}
	    }
	  rval = sprintf(bufp, " (#APV %2d)", iapv);
	  if(rval > 0)
	    bufp += rval;
	  if(errSlotMask & (1 << id))
	    {
	      rval = sprintf(bufp, " %s  %s  %s\n",
			     (apvErrorTypeMask[id] & 0x1) ? "*MPD NotFound*" :
			     "              ",
			     (apvErrorTypeMask[id] & 0x2) ? "*APV NotFound*" :
			     "              ",
			     (apvErrorTypeMask[id] & 0x4) ? "*APV Config*" :
			     "");
	      if(rval > 0)
		bufp += rval;
	    }
	  else
	    {
	      rval = sprintf(bufp, "\n");
	      if(rval > 0)
		bufp += rval;
	    }
	}
      /* else */
      /* 	{ */
      /* 	  rval = sprintf(bufp, */
      /* 			 "  MPD %2d :                                INIT ERRORS\n", id); */
      /* 	  if(rval > 0) */
      /* 	    bufp += rval; */
      /* 	} */
    }
  rval = sprintf(bufp, "\n");
  if(rval > 0)
    bufp += rval;

  daLogMsg("INFO",apvbuffer);

  if ((errSlotMask != 0) || (error_status != OK))
    daLogMsg("ERROR", "MPD initialization errors");
}

/****************************************
 *  DOWNLOAD
 ****************************************/
void
vtpMpdDownload()
{
#ifdef PEDSUB_USRSTRING
  /* Check usrString for pedestal subtraction mode */
  if(strcmp("PedSub",rol->usrString) == 0)
    {
      vtpSetPedSubtractionMode(1);
    }
  else
    {
      vtpSetPedSubtractionMode(0);
    }
#endif

  dalmabuffer = (char *)malloc(1024*50*sizeof(char));
  apvbuffer = (char *)malloc(1024*50*sizeof(char));

  printf("rocDownload: User Download Executed\n");
}

/****************************************
 *  PRESTART
 ****************************************/
void
vtpMpdPrestart()
{
  // Setup in Prestart since TI clock glitches at end of Download()
  vtp_mpd_setup();

}

/****************************************
 *  PAUSE
 ****************************************/
void
vtpMpdPause()
{
}

/****************************************
 *  GO
 ****************************************/
void
vtpMpdGo()
{
  int UseSdram, FastReadout;

  /*Enable MPD*/
  UseSdram = mpdGetUseSdram(mpdSlot(0)); // assume sdram and fastreadout are the same for all MPDs
  FastReadout = mpdGetFastReadout(mpdSlot(0));
  printf(" UseSDRAM= %d , FastReadout= %d\n",UseSdram, FastReadout);
  int k, i;
  for (k=0;k<fnMPD;k++) { // only active mpd set
    i = mpdSlot(k);

    // mpd latest configuration before trigger is enabled
    mpdSetAcqMode(i, "process");

    // load pedestal and thr default values
    mpdPEDTHR_Write(i);

    // enable acq
    mpdDAQ_Enable(i);

    mpdTRIG_Enable(i);
    mpd_evt[i]=0;
  }

  vtpMpdDalogStatus(mpdGetVTPFiberMask());

}

/****************************************
 *  END
 ****************************************/
void
vtpMpdEnd()
{
  int k;
  for (k=0;k<fnMPD;k++) { // only active mpd set
    mpdTRIG_Disable(mpdSlot(k));
  }

  vtpPrintMPD_OB_STATUS(1);
}

void
vtpMpdReset()
{
  int k;
  for (k=0;k<fnMPD;k++) { // only active mpd set
    mpdTRIG_Disable(mpdSlot(k));
  }
};



void
vtpMpdCleanup()
{
  if(dalmabuffer)
    free(dalmabuffer);

  if(apvbuffer)
    free(apvbuffer);
}

/*
  Routine to configure pedestal subtraction mode
      0 : subtraction mode DISABLED
      1 : subtraction mode ENABLED
*/

void
vtpSetPedSubtractionMode(int enable)
{
  vtpPedSubtractionMode = (enable) ? 1 : 0;

  daLogMsg("INFO","Setting Pedestral Subtraction Mode (%d)", vtpPedSubtractionMode);
}

/*
  Local Variables:
  compile-command: "make -k vtp_roc_mpdro.so"
  End:
 */
