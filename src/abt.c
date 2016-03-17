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

/* A's state variables */
static int state_a; // 0 - 3
static int seq_num_a;
static struct pkt packet_a;

/* B's state variables */
static int state_b;
static struct pkt packet_b;

/**
* Function to move A to next state
*/
void next_state_a() {
    state_a = (state_a + 1) % 4;
}


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

/**
* Function to handle an incoming message in state 0
* @param message message from layer 5
*
*/
static void handle_senda_st_zero_a(struct msg *message) {
    memset(&packet_a, 0, sizeof(struct pkt));

    // copy payload
    memcpy(&packet_a.payload, message, PAYLOAD_SIZE);

    // generate checksum
    packet_a.checksum = checksum(&packet_a);

    // pass the packet to layer 3
    tolayer3(0, packet_a);

    // change state to 1
    next_state_a();

    // start timer
    starttimer(0, TIMEOUT);
}

/**
* Function to handle an incoming message in state 2
* @param message message from layer 5
*
*/
static void handle_senda_st_two_a(struct msg *message) {
    memset(&packet_a, 0, sizeof(struct pkt));

    // copy payload
    memcpy(&packet_a.payload, message, PAYLOAD_SIZE);

    // set seq num
    packet_a.seqnum = 1;

    // generate checksum
    packet_a.checksum = checksum(&packet_a);

    // pass the packet to layer 3
    tolayer3(0, packet_a);

    // change state to 1
    next_state_a();

    // start timer
    starttimer(0, TIMEOUT);
}

/**
* Functiojn to 
*
*
*/
static void handle_recv_a(struct pkt *packet, int acknum) {
    if(!corrupt(packet) && packet->acknum == acknum) {
        stoptimer(0);
        next_state_a();
    }
}

static void handle_timeout_a() {
    // resend the last packet
    tolayer3(0, packet_a);

    // start the timer again
    starttimer(0, TIMEOUT);
}

/* called from layer 5, passed the data to be sent to other side */
void A_output(message)
  struct msg message;
{
    switch(state_a) {
    case 0:
        handle_senda_st_zero_a(&message);
        break;
    case 1:
        // Drop message
        fprintf(stderr, "sender: message dropped - %.20s\n", message.data);
        break;
    case 2:
        handle_senda_st_two_a(&message);
        break;
    case 3:
        // Drop message
        fprintf(stderr, "sender: message dropped - %.20s\n", message.data);
        break;
    default:
        fprintf(stderr, "sender: invalid state\n");
        break;
    }
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(packet)
  struct pkt packet;
{
    switch(state_a) {
    case 0:
        // NO OP
        break;
    case 1:
        handle_recv_a(&packet, 0);
        break;
    case 2:
        // NO OP
        break;
    case 3:
        handle_recv_a(&packet, 1);
        break;
    default:
        fprintf(stderr, "sender: invalid state\n");
        break;
    }
}

/* called when A's timer goes off */
void A_timerinterrupt()
{
    switch(state_a) {
    case 0:
        // NO OP
        break;
    case 1:
        handle_timeout_a();
        break;
    case 2:
        // NO OP
        break;
    case 3:
        handle_timeout_a();
        break;
    default:
        fprintf(stderr, "sender: invalid state\n");
        break;
    }
}  

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init()
{

}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/**
* Function to send ack
*
* @param acknum ACK num
*/
static void send_ack(int acknum) {
    // create packet
    memset(&packet_b, 0, sizeof(struct pkt));

    // set the acknum
    packet_b.acknum = acknum;

    // checksum packet
    packet_b.checksum = checksum(&packet_b);

    // send the ACK packet
    tolayer3(1, packet_b);
}

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(packet)
  struct pkt packet;
{
    switch(state_b) {
    case 0:
        if (!corrupt(&packet) && packet.seqnum == 0) {
            // deliver the packet
            tolayer5 (1, packet.payload);

            // send ack for packet with seqnum 0
            send_ack(0);

            // change state
            state_b = 1;
        } else {
            // send duplicate ACK
            send_ack(1);
        }
        break;
    case 1:
        if (!corrupt(&packet) && packet.seqnum == 1) {
            // deliver the packet
            tolayer5 (1, packet.payload);

            // send ack for packet with seqnum 0
            send_ack(1);

            // change state
            state_b = 0;
        } else {
            // send duplicate ACK
            send_ack(0);
        }
        break;
    default:
        fprintf(stderr, "receiver: invalid state\n");
        break;
    }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{

}
