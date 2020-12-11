#ifndef _THRESHOLD_INCLUDED
#define _THRESHOLD_INCLUDED
#include "usrstrutils.c"
#undef DEBUG
/* #define DEBUG */

#define FASTBUS 1

#define NTH_SUBADD 64
#define MAX_TH_SLOTS 19
#define BASE_ADDR 0xc0000000
#define TAG_THRESHOLDS 0x10


#define TYPE_LONG 1
#define TYPE_SEGMENT 0x20


unsigned int thresholds[MAX_TH_SLOTS][NTH_SUBADD];
unsigned int readback_thresholds[MAX_TH_SLOTS][NTH_SUBADD];
unsigned int slots[MAX_TH_SLOTS];

int nmodules;

void load_thresholds()
/* Look for tfile= to get filename with thresholds */
{
  char *fname;
  FILE *fd;
  char s[256];
  int imodule, subadd;
  int slot;

  nmodules=0;			/* Thresh ev will be created if this is set */

  fname = getstr("tfile");

  fd = fopen(fname,"r");
  if(!fd) {
    printf("Failed to open %s\n",fname);
  }
  printf("Reading ADC thresholds from %s\n",fname);
  imodule=-1;
  slot = -1;
  subadd = NTH_SUBADD;
  while(fgets(s,255,fd)) {
    char *arg;
    /*   printf("f:%s",s); */
    arg = strchr(s,COMMENT_CHAR);
    if(arg) *arg = '\0'; /* Blow away comments */
    if( (arg=strstr(s,"slot=")) ) {
      if(subadd < NTH_SUBADD) {
	printf("Not enough thresholds for slot %d, setting all to zero\n"
	       ,slot);
	for(subadd=0;subadd<NTH_SUBADD;subadd++){
	  thresholds[imodule][subadd] = 0;
	}
      }
      sscanf(arg+5,"%d",&slot);
      imodule++;
      subadd = 0;
      slots[imodule] = slot;
    } else if(slot >= 0) {
      if(subadd >= NTH_SUBADD) {
	/*printf("Ignoring excess values: %s\n",s);*/
      } else {
	arg = strtok(s,", \n");
	while(arg && subadd<NTH_SUBADD) {
	  /*  printf("arg[%d]=%s\n",subadd,arg); */
	  thresholds[imodule][subadd++] = strtol(arg,0,0);
	  arg = strtok(0,", \n");
	}
      }
    }
  }

  nmodules = imodule+1;
  printf("Done reading, nmodules = %d \n",nmodules);
  fclose(fd);

  free(fname);


  for(imodule=0;imodule<nmodules;imodule++) {
    printf("%d ",slots[imodule]);
  }
  printf("\n nth_subadd = %d \n",NTH_SUBADD);
  for(subadd=0;subadd<NTH_SUBADD;subadd++){
    for(imodule=0;imodule<nmodules;imodule++) {
      printf("%d ",thresholds[imodule][subadd]);
    }
    printf("\n");
  }
}


void set_thresholds()
#ifdef FASTBUS
/* Copy the loaded thresholds in the to Fastbus ADC modules.
   Modules must be reset before this routine is called */
{
  int imodule;
  int ichan;
  int slot, padr, sadr;
  unsigned int temp;

  /* Set the thresholds */
  for(imodule=0;imodule<nmodules;imodule++){
    slot = slots[imodule];

#ifdef DEBUG
    printf("Loading thresholds into slot %d\n",slot);
#endif

    /* address geographic  control slot */
    padr = slot;

    for(ichan=0;ichan<NTH_SUBADD;ichan++){
      sadr = BASE_ADDR + ichan;
#ifdef DEBUG
      printf("%d %d %x %d \n",thresholds[imodule][ichan],padr,sadr,ichan);
#endif

      /* write thresholds[imodule][ichan] to padr,sadr*/
      //if (ichan < 6){
      //temp = 0;

      //}
      //else
      temp = thresholds[imodule][ichan];
      /*      fb_fwc_1(padr,sadr,temp,1,1,0,0,0,1,1); */
      fpwc(padr,sadr,temp);
      sfi_error_decode(0);
    }
#ifdef DEBUG
    printf("\n");
#endif
    /* Note: sparcification turned on in crl code ! */

#ifdef DEBUG
    printf("Done loading thresholds into slot %d\n",slot);
#endif
  }
  /* Reading back the thresholds */
  for(imodule=0;imodule<nmodules;imodule++){
    slot = slots[imodule];

#ifdef DEBUG
    printf("Reading back thresholds from slot %d\n",slot);
#endif
    /* address geographic  control slot */
    padr = slot;

    for(ichan=0;ichan<NTH_SUBADD;ichan++){
      sadr = BASE_ADDR + ichan;
      /* read threshold from padr,sadr */
      fprc(padr,sadr,&temp);
      sfi_error_decode(0);


#ifdef DEBUG
      printf("Value of threshold %d %d %d %x %d \n",imodule,ichan,
	     padr,sadr,temp);
#endif

      readback_thresholds[imodule][ichan] = temp;
      //if(temp != thresholds[imodule][ichan]) {
      //printf("Slot %d, Chan %d: Wrote %d, Read %d\n",
      //slot,ichan,thresholds[imodule][ichan],temp);
      //}
    }

#ifdef DEBUG
    printf("Done reading back thresholds from slot %d\n",slot);
#endif
  }

  return;
  /* fooy: */
  /*  daLogMsg(FB_ERRTXT[(((unsigned long)(fb_errno) &0x00003fff)>>3)],fb_errno,
      global_routine[global_env_depth],fb_rtn,padr,sadr);*/
  return;
}
#else
{/* Camac Discriminator threshold setting routine */ }
#endif

#ifdef FASTBUS
void reset_adc(int slot)
/* Reset an adc and set all it's thresholds to zero */
{
  int ichan, sadr;

  /* address geographic control ADCSLOT1 */

  fb_frc_1(slot ,0,0,1,1,0,1,1,1,1);
  /* if (fb_errno != 0x800BC119){
     fb_rtn = "fpgac" ;
     goto fooy;
     } */
  /*   write hex 40000000  */
  fb_fwd_1(0,0,0x40000000,0,1,1,1,0,1,1);
  /* if (fb_errno != 0x800BC119){
     fb_rtn = "fpw" ;
     goto fooy;
     } */

  for(ichan=0;ichan<NTH_SUBADD;ichan++){
    sadr = BASE_ADDR + ichan;
    /* secondary address sadr */
    fb_frd_1(0,sadr,0,0,1,1,0,1,1,1);
    /* if (fb_errno != 0x800BC119){
       fb_rtn = "fpsaw" ;
       goto fooy;
       } */
    /* write 0 */
    fb_fwd_1(0,0,0,0,1,1,1,0,1,1);
    /* if (fb_errno != 0x800BC119){
       fb_rtn = "fpw" ;
       goto fooy;
       } */
  }

  /* release */
  fprel();
  /*  fb_frd_1(0,0,0,1,1,1,1,1,0,0); */
  /* if (fb_errno != 0x800BC119){
     fb_rtn = "fprel" ;
     goto fooy;
     } */
  return;
  /* fooy: */
  /* daLogMsg(FB_ERRTXT[(((unsigned long)(fb_errno) &0x00003fff)>>3)],fb_errno,
     global_routine[global_env_depth],fb_rtn,slot,sadr); */

  return;
}
#endif

#endif
