/*************************************************************************
 *
 *  vtp_mpdro_list.c - Library of routines for readout and buffering of
 *                events using a VTP as a readout path for the MPD
 *
 *     To be used with the vtp HW ROC readout list
 *
 */
#define VTP
#include <errno.h>
#include "vtp.h"
#include "mpdLib.h"
#include "mpdConfig.h"
#include "vtpMpdConfig.h"

#define PEDESTAL_FILENAME    "/home/sbs-onl/cfg/CommonModeRange_220.txt"
#define COMMON_MODE_FILENAME "/home/sbs-onl/cfg/gem_ped_220.dat"

/*
  Global to configure pedestal subtraction mode
      0 : subtraction mode DISABLED
      1 : subtraction mode ENABLED

  Set with vtpSetPedSubtractionMode(int enable)
*/
int vtpPedSubtractionMode = 0;
void vtpSetPedSubtractionMode(int enable); // routine prototype

/* vtp defs */

/*MPD Definitions*/
extern int I2C_SendStop(int id);

int fnMPD=0;

extern volatile struct mpd_struct *MPDp[(MPD_MAX_BOARDS+1)]; /* pointers to MPD memory map */

// End of MPD definition


char *apvbuffer;
char *bufp;


int
vtpMpdDalogStatus(unsigned int fmask)
{
  DALMAGO;
  vtpMpdPrintStatus(fmask, 0);
  DALMASTOP;
  return 0;
}


void
vtpPrintMPD_OB_STATUS(int dalogFlag)
{
  DALMAGO;
  mpdGStatus(0);
  DALMASTOP;
}



void vtp_mpd_setup()
{
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

  int build_all_samples = 1;
  //1 => For pedestal run, will write ADC samples from the APV (i.e. disable zero suppression)
  //0 => will apply the threshold and peak position logic to decide if data is written to the event (i.e. zero suppression enabled)

  int build_debug_headers = 1;
  //1 => will write extra debugging info headers about the common-mode processing (which APV chip reported data, avg A/B values and counts for each APV)
  //0 => disables extra debug info headers
  int enable_cm = 0;
  //1 => enables the common-mode subtraction logic
  //0 => disables the common-mode subtraction logic (so raw ADC samples only have the channel offset applied)
  int noprocessing_prescale = 9;
  //0 => prescaling disabled (all events are processed)
  //1-65535 => every Nth event has common-mode subtract and zero suppression disabled
  int allow_peak_any_time = 1;
  //0 => strip ADC peak sample must be neither first or last sample
  //1 => strip ADC peak sample can be at any time sample
  int min_avg_samples = 20;
  //minimum number of samples for a single APV frame that must fall within pedestal averaging region for common-mode to be applied. If less than this number are found in average, then common-mode will be disabled for that APV for all time ssamples of that event and a flag will be reported indicating this so offline can know it needs to perform the correction.

  vtpMpdEbSetFlags(build_all_samples, build_debug_headers,
		   enable_cm, noprocessing_prescale, allow_peak_any_time,
                   min_avg_samples);

  //char* mpdSlot[10],apvId[10],cModeMin[10],cModeMax[10];
  int apvId, cModeMin, cModeMax, cModeRocId;
  int fiberID = -1, last_mpdSlot = -1;
  FILE *fcommon   = fopen(COMMON_MODE_FILENAME,"r");



  if(fcommon == NULL)
    {
      perror("fopen");
      daLogMsg("ERROR","Failed to open GEM CommonModeRange file");
    }

  //valid pedestal file => will load APV offset file and subtract from APV samples
  FILE *fpedestal = fopen(PEDESTAL_FILENAME,"r");//all GEMs

  if(fpedestal == NULL)
    {
      perror("fopen");
      daLogMsg("ERROR","Failed to open GEM Pedestal file");
    }

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

        // if settings are for us
        if(cModeRocId != ROCID)
        {
          printf("Skipping pedestal settings not for us (us=%d, file=%d)\n", ROCID, cModeRocId);
          continue;
        }

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

      // if settings are for us
      if(cModeRocId != ROCID)
      {
        printf("Skipping common-mode settings not for us (us=%d, file=%d)\n", ROCID, cModeRocId);
        continue;
      }

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
    }
  rval = sprintf(bufp, "\n");
  if(rval > 0)
    bufp += rval;

  DALMAGO;
  printf("%s",apvbuffer);
  DALMASTOP;

  if ((errSlotMask != 0) || (error_status != OK))
    daLogMsg("ERROR", "MPD initialization errors");
}

void
vtpMpdApvConfigStatus()
{
  printf("%s",apvbuffer);
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
  }

  DALMAGO;
  printf("vtpPedSubtractionMode = %d\n", vtpPedSubtractionMode);
  printf("MPD PEDESTAL FILENAME: %s\n", PEDESTAL_FILENAME);
  printf("COMMON_MODE_FILENAME: %s\n", COMMON_MODE_FILENAME);
  DALMASTOP;
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