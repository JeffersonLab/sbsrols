/**
 *  Copyright (c) 2022        Southeastern Universities Research Association, *
 *                            Thomas Jefferson National Accelerator Facility  *
 *                                                                            *
 *    This software was developed under a United States Government license    *
 *    described in the NOTICE file included as part of this distribution.     *
 *                                                                            *
 *    Authors: Bryan Moffit                                                   *
 *             moffit@jlab.org                   Jefferson Lab, MS-12B3       *
 *             Phone: (757) 269-5660             12000 Jefferson Ave.         *
 *             Fax:   (757) 269-5800             Newport News, VA 23606       *
 *
 * Description:
 *
 *  Routines to check consistency of the syncFlag sources from the TI library
 *
 */

#include "tiLib.h"

/*

  tiSyncEventCheck()

  Routine to compare the syncFlag derived from hardware registers versus
  the syncFlag decoded from the most recent call the tiReadTriggerBlock(...)

  returns 0 if they are equal, otherwise -1

*/

int
tiSyncEventCheck()
{
  int reg_syncFlag = 0, data_syncFlag = 0;

  reg_syncFlag = tiGetSyncEventFlag();
  data_syncFlag = tiGetBlockSyncFlag();

  if(reg_syncFlag != data_syncFlag)
    {
      daLogMsg("ERROR","reg_syncFlag != data_syncFlag  (%d != %d)",
	       reg_syncFlag, data_syncFlag);
      return -1;
    }

  return 0;
}
