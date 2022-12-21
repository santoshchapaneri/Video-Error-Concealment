// MHL fixed bugs Oct 07 1999
// MHL Oct 14 1999 Changed such that '0' represents a dropped packet and '1' or greater
//                 represents a received packet

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <time.h>
#ifdef WIN32
#include <Winsock2.h>
#else
#include <netinet/in.h>
#endif

#define MAXPACKETSIZE 65000
#define LOSSPERCENT   10

int keep_packet(int loss_percent)
{
  int rnd;
  if (loss_percent>100)
    return 1;
  if (loss_percent<=0)
    return 0;
  
  //rnd = int (100 * (((float) rand()) / RAND_MAX));
  rnd = (int)(100 * (((float) rand()) / RAND_MAX));

  //return (rnd >= loss_percent );
  return 1;
}

int looseit(FILE *err) {
	int c;

	do {
		c = getc (err);
		if (c == EOF) {
			printf ("End of Error pattern\n");
			exit (0);
		}
		else if (c == '0') 
			return 0;
		else if (c >= '1') 
			return 1;
	} while (1);
}

int main(int ac, char *av[])
{
    FILE *in, *out, *err, *stat;
	unsigned char buf[MAXPACKETSIZE];
	unsigned psize, bufsize;
	int intime, pacno = 0, version, drop=0, flag;

	int loss_percent = LOSSPERCENT;
	srand(time(0));

	if (ac != 4) {
		printf ("Usage: %s <inpackets> <outpackets> <error-file>\n", av[0]);
		exit (-1);
	}

    if (NULL == (in = fopen(av[1],"rb"))) {
		perror("read input open");
		exit(1);
	}
    
    if (NULL == (out = fopen(av[2],"wb"))) {
		perror("write output open");
		exit(2);
	}

	if (NULL == (err = fopen (av[3], "rb"))) {
		perror ("read err open");	
		exit (3);
	}

	if (NULL == (stat = fopen ("stat.txt", "w"))) {
		perror ("write stat open");	
		exit (3);
	}
/*    while ((1 == fread (&psize, 4, 1, in))) 
	{
		if (psize != fread (buf, 1, psize, in)) 
		{
			printf ("can't read %d bytes from input file, exiting \n", psize);
			exit (-5);
		}
		if (looseit(err)) 
		{ 
			// MHL changed from !looseit(err) so that '0' means dropped packet
			fwrite (&psize, 4, 1, out);
			fwrite (buf, psize, 1, out);
		}
	}
*/
	
	while(1)
	{
		if (4 != fread (&bufsize, 1, 4, in))
		  return 0;
		if (4 != fread (&intime, 1, 4, in))
		  return -1;
		printf ("\n\npacket #%4d containing %5d bytes\n", pacno, bufsize);
		if (bufsize != fread (buf, 1, bufsize, in))
		{
		  printf ("Problems while reading buffer, exit\n");
		  return -3;
		}

		//Want to lose packet #6 i.e. 1st slice of 2nd I frame


		//SPS, PPS and first IDR frame never gets lost, an assumption
		//SPS and PPS can be send via reliable out-of-band transmission

		//if ((looseit(err) || pacno==0 || pacno==1 || pacno==2) && pacno!=6) 
		//if ((looseit(err) || pacno==0 || pacno==1 || pacno==2) && pacno!=22 && pacno!=42) 
		//if (pacno!=22 || pacno!=42) 

		//if ((looseit(err) || pacno==0 || pacno==1 || pacno==2))// || pacno==3 || pacno==4 || pacno==5 || pacno==6))
		//if ((keep_packet(loss_percent) || pacno==0 || pacno==1))// || pacno==2))// || pacno==3 || pacno==4 || pacno==5 || pacno==6))
		if ((keep_packet(loss_percent) || pacno==0 || pacno==1))
		{
			//if (pacno == 12 || pacno == 16 || pacno == 19 || pacno == 23)
			//if (pacno == 5 || pacno == 6 || pacno == 7)
			if (pacno == 7)
			{
				printf("\n\nPacket #%4d dropped\n", pacno);
				fprintf(stat,"Packet #%4d dropped\n", pacno);
				pacno++;
				continue;
			}
			// MHL changed from !looseit(err) so that '0' means dropped packet
			fwrite (&bufsize, 4, 1, out);
			fwrite (&intime, 4, 1, out);
			fwrite (buf, bufsize, 1, out);
		}
		else
		{
			printf("\n\nPacket #%4d dropped\n", pacno);
			fprintf(stat,"Packet #%4d dropped\n", pacno);
		}

		pacno++;
	}

	//MHL close files..
	fclose(in);
	fclose(out);
	fclose(err);
	return 0;
}
