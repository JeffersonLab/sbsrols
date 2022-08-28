/**
 * @copyright Copyright 2022, Jefferson Science Associates, LLC.
 *            Subject to the terms in the LICENSE file found in the
 *            top-level directory.
 *
 * @author    Bryan Moffit
 *            moffit@jlab.org                   Jefferson Lab, MS-12B3
 *            Phone: (757) 269-5660             12000 Jefferson Ave.
 *            Fax:   (757) 269-5800             Newport News, VA 23606
 *
 * @file      extraBusyCheck.c
 *
 * @brief     Library of routines for checking the busy status of VME
 *            modules when 'tsLive' is called.
 *
 */


static uint32_t previousBusy = 0;  /* Busy slot busy mask from previous call */
static uint32_t *slotBusyCount=NULL;    /* Count of repeated busy for each slot */

int32_t
extraBusyCheckInit(int32_t iflag)
{
  int32_t rval = 0;
  previousBusy = 0;

  if(slotBusyCount == NULL)
    {
      slotBusyCount = (uint32_t *) malloc((MAX_VME_SLOTS + 1) * sizeof(uint32_t));
      if(slotBusyCount == NULL)
	{
	  printf("%s: ERROR allocating memory for slotBusyCount\n",
		 __func__);
	  rval = -1;
	}
      memset(slotBusyCount, 0, (MAX_VME_SLOTS + 1) * sizeof(uint32_t));
    }

  return rval;
}

int32_t
extraBusyCheckReset(int32_t rflag)
{
  if(slotBusyCount != NULL)
    free(slotBusyCount);

  return 0;
}

static void
warnSlotBusy(int32_t slotID)
{

}


static void
updateSlotCounts(uint32_t busyMask)
{
  int32_t islot;

  /* Bail if the array is not allocated */
  if(slotBusyCount == NULL)
    return;

  for(islot = 2; islot < (VME_MAX_SLOTS + 1); islot++)
    {
      if(busyMask & (1 << islot))
	{
	  slotBusyCount[islot]++;

	  if(slotBusyCount[islot] > slotBusyWarnThreshold)
	    {
	      daLogMsg("WARN","ROC %d SLOT %d BUSY",
		       ROCID, islot);
	    }
	}
    }
}

int32_t
extraBusyCheck(int32_t flag)
{
  int32_t tiLiveTime = 0;
  uint32_t currentBusy = 0;          /* Busy slot busy mask to get now */
  const int32_t tiLowLimit = 10;     /* 10 => 1% */

  tiLiveTime = tiLive(flag);
  if(tiLiveTime < tiLowLimit)
    {
      /* assume the TI is in slot 21 */
      currentBusy |= (1 << 21);
    }


  if((currentBusy & previousBusy) != 0)
    {
      updateSlotCounts(currentBusy & previousBusy);
    }

  /* prep for next call */
  previousBusy = currentBusy;

  return tiLiveTime;
}
