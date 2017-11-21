#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/bcm.h>

int main(void) {

	int s;
	int nbytes;
	struct sockaddr_can addr;
	struct ifreq ifr;

	struct bcm_msg_head {
		int opcode;                   
		int flags;                    
		int count;                   
		struct timeval ival1, ival2; 
		canid_t can_id;              
		int nframes;                 
	};

    struct {
      struct bcm_msg_head msg_head;
      struct can_frame frame[2];
    } msg;

	const char *ifname = "vcan0";

	if((s = socket(PF_CAN, SOCK_DGRAM, CAN_BCM)) < 0) {
		perror("Error while opening socket");
		return -1;
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(s, SIOCGIFINDEX, &ifr);
	
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	
	printf("%s at index %d\n", ifname, ifr.ifr_ifindex);
	connect(s, (struct sockaddr *)&addr, sizeof(addr));

	msg.msg_head.opcode  = TX_SETUP;
	msg.msg_head.flags   = 0;
	msg.msg_head.can_id  = 0;
	msg.msg_head.nframes = 2;
	msg.msg_head.count = 0;
    msg.msg_head.ival1.tv_sec = 0;
    msg.msg_head.ival1.tv_usec = 0;
    msg.msg_head.ival2.tv_sec = 0;
    msg.msg_head.ival2.tv_usec = 0;
	
	msg.frame[0].can_id    = 0x202;
	msg.frame[0].can_dlc   = 2;
	msg.frame[0].data[0]   = 0x07;
	msg.frame[0].data[1]   = 0x00;

	msg.frame[1].can_id    = 0x203;
	msg.frame[1].can_dlc   = 2;
	msg.frame[1].data[0]   = 0x07;
	msg.frame[1].data[1]   = 0x00;
	
	write(s, &msg, sizeof(msg));
	
	return 0;
}
