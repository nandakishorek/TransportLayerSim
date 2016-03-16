#include "../include/simulator.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PAYLOAD_SIZE 20
#define TIMEOUT 10.0

/* ******************************************************************
 ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.1  J.F.Kurose

   This code should be used for PA2, unidirectional data transfer 
   protocols (from A to B). Network properties:
   - one way network delay averages five time units (longer if there
     are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
     or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
     (although some can be lost).
**********************************************************************/

/**
* Nandakishore Krishna
* nandakis@buffalo.edu
*
*/

/* A's state variables*/
static int winsize_a;
static int base_a;
static int nextseqnum;
static struct pkt *sndpkt = NULL;

/* B's state variables*/
static int expseqnum;
static struct pkt packet_b;

/**
* Checksum function.
* Returns checksum - sum of seqnum, acksum, payload chars as integers.
*
* @param data packet to be checksum'ed
*/
static int checksum(struct pkt *data) {
    int cksum = 0;
    cksum += data->seqnum + data->acknum;

    for (int i = 0; i < PAYLOAD_SIZE; ++i) {
        cksum += (int)data->payload[i];
    }
    return cksum;
}

/**
* Function to check the packet integrity
*
* @return 1 on corrupt, 0 otherwise
*/
static int corrupt(struct pkt *packet) {
    if (checksum(packet) != packet->checksum) {
        return 1;
    }
    return 0;
}

/* called from layer 5, passed the data to be sent to other side */
void A_output(message)
  struct msg message;
{
    if (nextseqnum < (base_a + winsize_a)) {
        // create packet
        memset(&sndpkt[nextseqnum], 0, sizeof(struct pkt));
        sndpkt[nextseqnum].seqnum = nextseqnum;
        memcpy(&sndpkt[nextseqnum].payload, &message.data, PAYLOAD_SIZE);
        sndpkt[nextseqnum].checksum = checksum(&sndpkt[nextseqnum]);

        // send packet
        tolayer3(0, sndpkt[nextseqnum]);

        // if sending first packet in window, start timer
        if (base_a == nextseqnum) {
            starttimer(0, TIMEOUT);
        }

        // increment seq num
        ++nextseqnum;
    }
    // else DROP message
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(packet)
  struct pkt packet;
{
    if (!corrupt(&packet)) {
        // slide the window forward
        base_a = packet.acknum + 1;

        if (base_a == nextseqnum) {
            // all packets ACK'ed
            stoptimer(0);
        } else {
            // restart the timer for base packet
            stoptimer(0);
            starttimer(0, TIMEOUT);
        }
    }
}

/* called when A's timer goes off */
void A_timerinterrupt()
{
    // start the timer
    starttimer(0, TIMEOUT);

    // resend all the un-ACK'ed packets
    for (int i = base_a; i < nextseqnum; ++i) {
        tolayer3(0, sndpkt[i]);
    }
}  

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init()
{
    // get the window size
    winsize_a = getwinsize();

    // set the base and nextseqnum
    base_a = 1;
    nextseqnum = 1;

    // allocate buffers
    if (sndpkt == NULL) {
        sndpkt = malloc(winsize_a * sizeof(struct pkt));
    }
}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(packet)
  struct pkt packet;
{
    if (!corrupt(&packet) && packet.seqnum == expseqnum) {
        // deliver packet
        tolayer5 (1, packet.payload);

        // create ACK packet
        memset(&packet_b, 0, sizeof(struct pkt));
        packet_b.acknum = expseqnum;
        packet_b.checksum = checksum(&packet_b);

        // send ACK
        tolayer3(1, packet_b);

        // increment expected seqnum
        ++expseqnum;
    } else {
        // send duplicate ACK and drop this packet
        tolayer3(1, packet_b);
    }
}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{
    expseqnum = 1;
}
