/*
 *
 *  A few helpful routines to help do stuff in the ROC readout list
 *
 */

#include <byteswap.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Routine to read in a file (as text) and create a String Bank
   (uchar*) in a buffer
*/

int
rocFile2Bank(const char *fname, uint8_t *buf,
	      uint16_t banktag, uint8_t banknum,
	      int32_t maxbytes)
{

  int ii, c, ilen, rem;
  FILE *fid;
  int maxb = 1024 * 1024 * 4;	/* max output buffer size */
  uint32_t bank_header[2];

  if(fname == NULL)
    {
      printf("%s: ERROR: No filename was specified\n", __func__);
      return -1;
    }

  if((maxbytes == 0) || (maxbytes > maxb))
    maxbytes = maxb;		/* Default to 4 MB */


  fid = fopen(fname, "r");
  if(fid == NULL)
    {
      printf("%s: ERROR: The file %s does not exist \n", __func__, fname);
      return -1;
    }
  else
    {
      printf("%s: INFO: Opened file %s for reading into Bank (%x,%x) \n",
	     __func__, fname, banktag, banknum);
    }

  /* Read in the file to the buffer */
  ilen = 8;			/* leave the header words */

  while((ilen < maxbytes) && ((c = getc(fid)) != EOF))
    {
      buf[ilen++] = c;
    }
  fclose(fid);
  printf("%s: INFO: Read %d bytes into buffer\n", __func__, ilen);

  /* Make sure we null terminate the buffer and pad it out to an
     integal number of words */
  rem = 4 - (ilen & 0x3);
  if(rem < 4)
    {
      for(ii = 0; ii < rem; ii++)
	buf[(ilen + ii)] = 0;
      ilen += ii;
      //      rem = 0;
    }
  else
    {
      rem = 0;			/* no padding necessary just NULL terminate */
      buf[ilen] = 0;
    }

  /* Write the header info in the buffer */
  /* Bank Tag */
  bank_header[0] = (ilen>>2) - 1; /* divide by 4  and subtract 1 */
  bank_header[1] = (banktag<<16)|(rem<<14)|(3<<8)| banknum;
  memcpy(&buf[0],&bank_header[0],8);

  /*Swap the bytes going to the Async Event 32 bit Fifo */
#ifdef BYTESWAPIT
  if(0)
    {
      uint8_t aa, bb;
      for(ii = 8; ii <= (bank_header[0] << 2); ii += 4)
	{
	  aa = buf[ii];
	  bb = buf[ii + 1];
	  buf[ii] = buf[ii + 3];
	  buf[ii + 1] = buf[ii + 2];
	  buf[ii + 2] = bb;
	  buf[ii + 3] = aa;
	}
    }
  else
    {
      for(ii = 0; ii <= (bank_header[0]); ii++)
	u32p[ii] = bswap_32(u32p[ii]);
    }
#endif /* BYTESWAPIT */


  return (bank_header[0] + 1);

}

int
rocBuffer2Bank(const char *inbuf, uint8_t *buf,
	      uint16_t banktag, uint8_t banknum,
	      int32_t nbytes)
{

  int ii, c, ilen, ibyte = 0, rem;
  int maxb = 1024 * 1024 * 4;	/* max output buffer size */
  uint32_t bank_header[2];

  if(nbytes > maxb)
    nbytes = maxb;		/* Default to 4 MB */

  /* Read in the file to the buffer */
  ilen = 8;			/* leave the header words */
  ibyte = 0;
  while(ibyte < nbytes)
    {
      buf[ilen++] = inbuf[ibyte++];
    }
  printf("%s: INFO: Read %d bytes into buffer\n", __func__, ilen);

  /* Make sure we null terminate the buffer and pad it out to an
     integal number of words */
  rem = 4 - (ilen & 0x3);
  if(rem < 4)
    {
      for(ii = 0; ii < rem; ii++)
	buf[(ilen + ii)] = 0;
      ilen += ii;
    }
  else
    {
      rem = 0;			/* no padding necessary just NULL terminate */
      buf[ilen] = 0;
    }

  /* Write the header info in the buffer */
  /* Bank Tag */
  bank_header[0] = (ilen>>2) - 1; /* divide by 4  and subtract 1 */
  bank_header[1] = (banktag<<16)|(rem<<14)|(3<<8)| banknum;
  memcpy(&buf[0],&bank_header[0],8);

  /*Swap the bytes going to the Async Event 32 bit Fifo */
#ifdef BYTESWAPIT
  if(0)
    {
      uint8_t aa, bb;
      for(ii = 8; ii <= (bank_header[0] << 2); ii += 4)
	{
	  aa = buf[ii];
	  bb = buf[ii + 1];
	  buf[ii] = buf[ii + 3];
	  buf[ii + 1] = buf[ii + 2];
	  buf[ii + 2] = bb;
	  buf[ii + 3] = aa;
	}
    }
  else
    {
      for(ii = 0; ii <= (bank_header[0]); ii++)
	u32p[ii] = bswap_32(u32p[ii]);
    }
#endif /* BYTESWAPIT */


  return (bank_header[0] + 1);

}
