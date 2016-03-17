#include "../include/simulator.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#define PAYLOAD_SIZE 20
#define TIMEOUT 10.0
#define NUM_MSGS 1500

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

/* software timer queue */
typedef struct timeout {
    float start_time; // the time when the timer started
    int seqnum;
    struct timeout *next;
} timeout_t;

timeout_t *head = NULL;

/* A's state variables*/
static int winsize_a;
static int base_a;
static int nextseqnum;
static struct pkt *sndpkt = NULL;

/* B's state variables*/
static int winsize_b;
static int base_b;
static int expseqnum;
static struct pkt *recvpkt = NULL;
static int *undelivered = NULL;
static int *received = NULL;

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
* Function to set timer for a packet with seqnum
*
* @param seqnum sequence number of the packet
*/
void start_timer(int seqnum) {
    printf("%s: seqnum:%d\n", __func__, seqnum);

    // create an entry
    timeout_t *new = malloc(sizeof(timeout_t));
    new->start_time = get_sim_time();
    new->seqnum = seqnum;
    new->next = NULL;

    // add it to the queue
    if (head == NULL) {
        head = new;

        // start the HW timer
        starttimer(0, TIMEOUT);
        printf("%s: started HW timer\n", __func__);
    } else {
        timeout_t *iter = head;
        while(iter->next != NULL) {
            iter = iter->next;
        }
        iter->next = new;
        printf("%s: queued seqnum:%d\n", __func__, seqnum);
    }
}

/**
* Function to stop the timer for a packet with seqnum
*
* @param seqnum sequence number of the packet
*/
void stop_timer(int seqnum) {
    printf("%s: seqnum:%d\n", __func__, seqnum);
    if (head != NULL) {
        if (head->seqnum == seqnum) {
            // stop the HW timer
            stoptimer(0);
            printf("%s: stopped HW timer\n", __func__);

            // remove from queue
            timeout_t *front = head;
            head = head->next;
            printf("%s: dequeued seqnum %d\n", __func__, seqnum);
            free(front);
        } else {
            // search and remove from the queue
            timeout_t *iter = head;
            while (iter->next != NULL) {
                if (iter->next->seqnum == seqnum) {
                    timeout_t *temp = iter->next;
                    iter->next = temp->next;
                    printf("%s: dequeued seqnum %d\n", __func__, seqnum);
                    free(temp);
                    break;
                }
                iter = iter->next;
            }
        }
    } else {
        fprintf(stderr, "%s, timer queue was empty\n", __func__);
    }
}

/**
* Function to determine the least unacked packet seqnum
*/
int get_next_unacked() {
    int min = INT_MAX;
    timeout_t *iter = head;
    while(iter != NULL) {
        if (iter->seqnum < min) {
            min = iter->seqnum;
        }
        iter = iter->next;
    }
    if (min != INT_MAX) {
        return min;
    }
    return base_a + 1;
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
        printf("%s: sent %.20s base_a:%d seqnum:%d\n", __func__, message.data, base_a, nextseqnum);

        // start the timer for this packet
        start_timer(nextseqnum);

        // increment seq num
        ++nextseqnum;
    } else {
        // DROP message
        printf("%s: message %.20s dropped\n", __func__, message.data);
    }
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(packet)
  struct pkt packet;
{
    if (!corrupt(&packet) && packet.acknum >= base_a && packet.acknum < (base_a + winsize_a)) {
        printf("%s: acknum:%d base_a:%d\n", __func__, packet.acknum, base_a);

        // mark packet as received by stopping the timer
        stop_timer(packet.acknum);

        // if the ACK is for base_a then slide the window forward
        if (base_a == packet.acknum) {
            base_a = get_next_unacked();
            printf("%s move base_a to %d\n",__func__, base_a);
        }
    } else {
        printf("%s: packet corrupt or out of the window\n", __func__);
    }
}

/**
* Callback function for the software timer interrupt
*
* @param seqnum seqnum corresponding to the timedout packet
*/
static void timeout_callback(int seqnum) {
    printf("%s: seqnum:%d\n", __func__, seqnum);

    // resend the packet
    tolayer3(0, sndpkt[seqnum]);

    // restart the timer
    start_timer(seqnum);
}

/* called when A's timer goes off */
void A_timerinterrupt()
{
    // timeout happened for the seqnum in the front of the queue
    if (head != NULL) {
        // remove it and call the callback func
        timeout_t *front = head;
        timeout_callback(front->seqnum);
        head = head->next;

        // start the timer for the next seqnum in the queue
        starttimer(0, head->start_time - front->start_time);
        printf("%s: started HW timer\n", __func__);

        free(front);
    } else {
        fprintf(stderr, "%s: timer queue was empty", __func__);
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
        sndpkt = malloc(NUM_MSGS * sizeof(struct pkt));
    }
}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(packet)
  struct pkt packet;
{
    printf("%s: seqnum %d\n", __func__, packet.seqnum);
    if (!corrupt(&packet)) {
        if (packet.seqnum >= base_b && packet.seqnum < (base_b + winsize_b)) {
            // create ACK
            struct pkt ackpkt;
            memset(&ackpkt, 0, sizeof(struct pkt));
            ackpkt.acknum = packet.seqnum;
            ackpkt.checksum = checksum(&ackpkt);

            // send ACK
            tolayer3(1, ackpkt);

            if (!received[packet.seqnum]) {
                // mark as received
                received[packet.seqnum] = 1;

                // buffer the packet
                memcpy(&recvpkt[packet.seqnum], &packet, sizeof(struct pkt));

                // mark for delivery
                undelivered[packet.seqnum] = 1;

                if (packet.seqnum == expseqnum) {
                    // in order packet
                    int i;
                    for (i = packet.seqnum; i < (base_b + winsize_b); ++i) {
                        if (undelivered[i]) {
                            printf("%s: delivered seqnum %d\n", __func__, recvpkt[i].seqnum);
                            tolayer5(1, recvpkt[i].payload);
                            undelivered[i] = 0;
                        } else {
                            break;
                        }
                    }
                    expseqnum = i;
                }
            }
        } else if (packet.seqnum >= (base_b - winsize_b) && packet.seqnum < (winsize_b - 1)) {
             // create ACK
            struct pkt ackpkt;
            memset(&ackpkt, 0, sizeof(struct pkt));
            ackpkt.acknum = packet.seqnum;

            // send ACK
            tolayer3(1, ackpkt);
        }
        // else drop packet
    }
}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{
    // get the window size
    winsize_b = getwinsize();

    // set the base and expseqnum
    base_b = 1;
    expseqnum = 1;

    // allocate buffers
    if (recvpkt == NULL) {
        recvpkt = malloc(NUM_MSGS * sizeof(struct pkt));
    }

    // allocate memory for delivered flags
    if (undelivered == NULL) {
        undelivered = malloc(NUM_MSGS * sizeof(int));
    }
    memset(undelivered, 0, NUM_MSGS * sizeof(int));

    // allocate memory for received flags
    if (received == NULL) {
        received = malloc(NUM_MSGS * sizeof(int));
    }
    memset(received, 0, NUM_MSGS * sizeof(int));
}
