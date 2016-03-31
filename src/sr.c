#include "../include/simulator.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>

#define PAYLOAD_SIZE 20
#define TIMEOUT 12.0
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
static int end_a;
static int nextseqnum;
static struct pkt *sndpkt = NULL;
static float timeout = TIMEOUT;
static float est_to;
static float devrtt;
static float *start_time = NULL;

/* B's state variables*/
static int winsize_b;
static int base_b;
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
        starttimer(0, timeout);
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

            if (head != NULL) {
                // start the timer for the next seqnum in the queue
                starttimer(0, timeout + head->start_time - get_sim_time());
                printf("%s: started HW timer\n", __func__);
            }

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
        printf("%s: timer queue was empty\n", __func__);
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
    return end_a + 1;
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
    // create packet
    memset(&sndpkt[nextseqnum], 0, sizeof(struct pkt));
    sndpkt[nextseqnum].seqnum = nextseqnum;
    memcpy(&sndpkt[nextseqnum].payload, &message.data, PAYLOAD_SIZE);
    sndpkt[nextseqnum].checksum = checksum(&sndpkt[nextseqnum]);

    if (nextseqnum < (base_a + winsize_a)) {
        // send packet
        tolayer3(0, sndpkt[nextseqnum]);
        printf("%s: sent %.20s base_a:%d seqnum:%d\n", __func__, message.data, base_a, nextseqnum);

        // start the timer for this packet
        start_timer(nextseqnum);

        // set the last sent message seqnum
        end_a = nextseqnum;

        // start sampling
        start_time[end_a] = get_sim_time();
        printf("sampling seqnum %d, start time %f\n", end_a, start_time[end_a]);
    } else {
        // buffer message
        printf("%s: message %.20s with seqnum %d buffered\n", __func__, message.data, nextseqnum);
    }

    // increment seq num
    ++nextseqnum;
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(packet)
  struct pkt packet;
{
    if (!corrupt(&packet) && packet.acknum >= base_a && packet.acknum < (base_a + winsize_a)) {
        printf("%s: acknum:%d base_a:%d\n", __func__, packet.acknum, base_a);

        // mark packet as received by stopping the timer
        stop_timer(packet.acknum);

        // stop sampling
        if (start_time[packet.acknum] != 0.0f) {
            est_to = (0.875f * est_to) + (0.125f * (get_sim_time() - start_time[packet.acknum]));
            printf("estimated rtt %f, sample rtt %f\n", est_to, (get_sim_time() - start_time[packet.acknum]));
            devrtt = (0.75f * devrtt) + 0.25f * fabs((get_sim_time() - start_time[packet.acknum]) - est_to);
            printf("sampling before New timeout %f seqnum %d devrtt %f\n", est_to, packet.acknum, devrtt);
            timeout = est_to + 4.0f * devrtt;
            printf("sampling New timeout %f seqnum %d end time %f\n", timeout, packet.acknum, get_sim_time());
            start_time[packet.acknum] = 0.0f;
        }

        // if the ACK is for base_a then slide the window forward
        if (base_a == packet.acknum) {
            base_a = get_next_unacked();
            printf("%s move base_a to %d\n",__func__, base_a);

            // if there are buffered messages, send them
            for (int i = end_a + 1; (i < nextseqnum && i < (base_a + winsize_a)); ++i) {
                printf("sending buffered message %.20s with deq num %d", sndpkt[i].payload, sndpkt[i].seqnum);
                tolayer3(0, sndpkt[i]);
                end_a = i;

                // start the timer for this packet
                start_timer(end_a);
            }
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

    // stop and reset sampling
    start_time[seqnum] = 0.0f;
    printf("seqnum %d stopped sampling\n", seqnum);
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
        printf("%s: timer queue was empty", __func__);
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

   // allocate memory for timeout sampling
    if (start_time == NULL) {
        start_time = malloc(NUM_MSGS * sizeof(float));
    }
    memset(start_time, 0, NUM_MSGS * sizeof(float));
}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(packet)
  struct pkt packet;
{
    if (!corrupt(&packet)) {
        if (packet.seqnum >= base_b && packet.seqnum < (base_b + winsize_b)) {
            printf("%s: packet in current window - seqnum %d\n", __func__, packet.seqnum);

            // create ACK
            struct pkt ackpkt;
            memset(&ackpkt, 0, sizeof(struct pkt));
            ackpkt.acknum = packet.seqnum;
            ackpkt.checksum = checksum(&ackpkt);

            // send ACK
            tolayer3(1, ackpkt);
            printf("%s: sent acknum %d\n", __func__, packet.seqnum);

            if (!received[packet.seqnum]) {
                // mark as received
                received[packet.seqnum] = 1;

                // buffer the packet
                memcpy(&recvpkt[packet.seqnum], &packet, sizeof(struct pkt));

                // mark for delivery
                undelivered[packet.seqnum] = 1;

                if (packet.seqnum == base_b) {
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
                    base_b = i;
                }
            }
        } else if (packet.seqnum >= (base_b - winsize_b) && packet.seqnum < (base_b - 1)) {
            printf("%s: packet in previous window - seqnum %d\n", __func__, packet.seqnum);

            // create ACK
            struct pkt ackpkt;
            memset(&ackpkt, 0, sizeof(struct pkt));
            ackpkt.acknum = packet.seqnum;
            ackpkt.checksum = checksum(&ackpkt);

            // send ACK
            tolayer3(1, ackpkt);
            printf("%s: sent acknum %d\n", __func__, packet.seqnum);
        } else {
            // drop packet
            printf("%s: dropped seqnum %d\n", __func__, packet.seqnum);
        }
    } else {
        printf("%s: packet corrupt\n", __func__);
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
