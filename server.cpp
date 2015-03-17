#include <sys/types.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>

#define WIDTH 640
#define HEIGHT 480
#define SCALE 2
#define BUFLEN WIDTH/SCALE*sizeof(XnDepthPixel)+2*sizeof(unsigned int)
#define DEFAULT_IP "127.0.0.1"
#define DEFAULT_PORT 12111

using namespace xn;

int main(int argc, char **argv) {

	char IP[16];
	int PORT = -1;
	switch (argc) {
		case 1:
			strcpy(IP, DEFAULT_IP);
			PORT = DEFAULT_PORT;
			printf("No target IP or Port specified, assuming %s:%d\n", IP, PORT);
			break;
		case 2:
			strcpy(IP, argv[1]);
			PORT = DEFAULT_PORT;
			printf("No Port specified, assuming %d\n", PORT);
			break;
		case 3:
			strcpy(IP, argv[1]);
			PORT = atoi(argv[2]);
			break;
	}


	struct sockaddr_in si_other;
	unsigned char buffer[BUFLEN];

	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	memset((char *) &si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(PORT);
	if (inet_aton(IP, &si_other.sin_addr) == 0) {
		fprintf(stderr, "Invalid IP address %s", IP);
		exit(-1);
	}
	
	XnStatus nRetVal = XN_STATUS_OK;
	xn::Context context;
	nRetVal = context.Init();


	xn::DepthGenerator depth;
	nRetVal = depth.Create(context);
	nRetVal = context.StartGeneratingAll();

	DepthMetaData depthMD;
	depth.GetMetaData(depthMD);

	const XnDepthPixel* pDepthMap;
	uint16_t minDist = 65535;
	
	while (true) {
		nRetVal = context.WaitOneUpdateAll(depth);
		if (nRetVal != XN_STATUS_OK) {
			fprintf(stderr,"Failed updating Kinect data: %s\n", xnGetStatusString(nRetVal));
			exit(-1);
		}

		depth.GetMetaData(depthMD);
		pDepthMap = depth.GetDepthMap();
		
		for (unsigned int j = 0; j<depthMD.YRes(); j+=SCALE) {
			*(unsigned int*)buffer = depthMD.FrameID();
			*(unsigned int*)(buffer + sizeof(unsigned int)) = j;
			for (unsigned int i = 0; i<depthMD.XRes();i+=SCALE) {
				*(unsigned int*)(buffer + 2*sizeof(unsigned int) + sizeof(XnDepthPixel)*(i/SCALE)) = pDepthMap[(depthMD.XRes()*j + i)];
				if (pDepthMap[(depthMD.XRes()*j + i)] > 0 && pDepthMap[(depthMD.XRes()*j + i)] < minDist) {
					minDist = pDepthMap[(depthMD.XRes()*j + i)];
				}
			}
			if (sendto(sock, buffer, BUFLEN, 0, (const sockaddr*)&si_other, sizeof(si_other)) < 0) {
				fprintf(stderr, "Error sending UDP packet to %s:%d (size %d, errno %d)\n", IP, PORT,BUFLEN,errno);
				exit(-1);
			}
		}
		printf("Broadcasting frame %d (min dist %d)\n", depthMD.FrameID(), minDist);
		minDist = 65535;
	}
	close(sock);

}
