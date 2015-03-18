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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define RGB_WIDTH 320
#define RGB_HEIGHT 240
#define DEPTH_WIDTH 320
#define DEPTH_HEIGHT 240
/* Comment out DEPTH_SCALE to transmit entire depth buffer */
#define DEPTH_SCALE 2
#ifndef DEPTH_SCALE
	#define DEPTH_BUFFER_LENGTH DEPTH_WIDTH*sizeof(XnDepthPixel)+2*sizeof(unsigned int)
#else
	#define DEPTH_BUFFER_LENGTH DEPTH_WIDTH/DEPTH_SCALE*sizeof(XnDepthPixel)+2*sizeof(unsigned int)
#endif
#define DEFAULT_IP "127.0.0.1"
#define DEFAULT_PORT 12111

using namespace xn;
using namespace cv;

int main(int argc, char **argv) {
	/* Parse Input for IP and Port options */
	char ip[16];
	unsigned short depthPort = 0, rgbPort = 0;
	switch (argc) {
		case 1:
			strcpy(ip, DEFAULT_IP);
			depthPort = DEFAULT_PORT;
			rgbPort = DEFAULT_PORT+1;
			printf("No target IP or Ports specified, using defaults\n");
			break;
		case 2:
			strcpy(ip, argv[1]);
			depthPort = DEFAULT_PORT;
			rgbPort = DEFAULT_PORT+1;
			printf("No Ports specified, using defaults\n");
			break;
		case 3:
			strcpy(ip, argv[1]);
			depthPort = atoi(argv[2]);
			rgbPort = depthPort + 1;
			printf("No RGB Port specified, using DEPTH_PORT+1\n");
			break;
	}
	printf("Depth: %s:%d\nRGB: %s:%d\n", ip, depthPort, ip, rgbPort);
	
	/* Create sockets for Depth and RGB Data */
	int depthSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (depthSocket < 0) {
		fprintf(stderr, "Failed to create socket for depth data (errno: %d)\n", errno);
		exit(-1);
	}
	
	int rgbSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (rgbSocket < 0) {
		fprintf(stderr, "Failed to create socket for RGB data (errno: %d)\n", errno);
		exit(-1);
	}
	
	/* Set up destination information for each socket */
	struct sockaddr_in socketInfoDepth;
	memset((char *) &socketInfoDepth, 0, sizeof(socketInfoDepth));
	socketInfoDepth.sin_family = AF_INET;
	socketInfoDepth.sin_port = htons(depthPort);
	if (inet_aton(ip, &socketInfoDepth.sin_addr) == 0) {
		fprintf(stderr, "Invalid IP address %s", ip);
		exit(-1);
	}
	
	struct sockaddr_in socketInfoRGB;
	memset((char *) &socketInfoRGB, 0, sizeof(socketInfoRGB));
	socketInfoRGB.sin_family = AF_INET;
	socketInfoRGB.sin_port = htons(rgbPort);
	if (inet_aton(ip, &socketInfoRGB.sin_addr) == 0) {
		fprintf(stderr, "Invalid IP address %s", ip);
		exit(-1);
	}
	
	/* Set up the Kinect device */
	XnStatus nRetVal = XN_STATUS_OK;
	Context context;
	nRetVal = context.Init();
	if (nRetVal != XN_STATUS_OK) {
		fprintf(stderr, "Failed to initialise OpenNI (%s)\n", xnGetStatusString(nRetVal));
		exit(-1);
	}

	/* Create depth and image generators */
	DepthGenerator depth;
	nRetVal = depth.Create(context);
	if (nRetVal != XN_STATUS_OK) {
		fprintf(stderr, "Failed to create OpenNI Depth Generator (%s)\n", xnGetStatusString(nRetVal));
		exit(-1);
	}
	
	ImageGenerator image;
	nRetVal = image.Create(context);
	if (nRetVal != XN_STATUS_OK) {
		fprintf(stderr, "Failed to create OpenNI Image Generator (%s)\n", xnGetStatusString(nRetVal));
		exit(-1);
	}
	
	/* Configure depth and image generators */
	XnMapOutputMode depthMapMode; 
	depthMapMode.nXRes = DEPTH_WIDTH; 
	depthMapMode.nYRes = DEPTH_HEIGHT; 
	depthMapMode.nFPS = 30; 
	nRetVal = depth.SetMapOutputMode(depthMapMode);
	if (nRetVal != XN_STATUS_OK) {
		fprintf(stderr, "Failed to set Depth Map Mode (%s)\n", xnGetStatusString(nRetVal));
		exit(-1);
	}
	
	XnMapOutputMode rgbMapMode; 
	rgbMapMode.nXRes = RGB_WIDTH; 
	rgbMapMode.nYRes = RGB_HEIGHT; 
	rgbMapMode.nFPS = 30; 
	nRetVal = image.SetMapOutputMode(rgbMapMode);
	if (nRetVal != XN_STATUS_OK) {
		fprintf(stderr, "Failed to set RGB Map Mode (%s)\n", xnGetStatusString(nRetVal));
		exit(-1);
	}

	/* Start generating data */
	nRetVal = context.StartGeneratingAll();
	if (nRetVal != XN_STATUS_OK) {
		fprintf(stderr, "Failed to start OpenNI generators (%s)\n", xnGetStatusString(nRetVal));
		exit(-1);
	}
	
	
	/* Start inifite loop to output data to sockets */
	DepthMetaData depthMD;
	unsigned char depthBuffer[DEPTH_BUFFER_LENGTH];
	const XnDepthPixel* pDepthMap;
	const XnRGB24Pixel* pImage;
	
	while (true) {
		/* Get latest frame */
		nRetVal = context.WaitOneUpdateAll(depth);
		if (nRetVal != XN_STATUS_OK) {
			fprintf(stderr,"Failed updating Kinect data: %s\n", xnGetStatusString(nRetVal));
			exit(-1);
		}

		/* Get depth data */
		depth.GetMetaData(depthMD);
		pDepthMap = depth.GetDepthMap();
		
		/* Output depth data one line at a time so we don't get oversized packets */
		/* TODO: Should check consistency of depthMD.YRes() with DEPTH_WIDTH */
#ifndef DEPTH_SCALE		
		for (unsigned int j = 0; j<DEPTH_HEIGHT; j++) {
#else
		for (unsigned int j = 0; j<DEPTH_HEIGHT; j+=DEPTH_SCALE) {
#endif
			/* First two ints in the buffer are Frame ID and Line Number
			 *	this should allow for easy reconstruction of depth map */
			*(unsigned int*)depthBuffer = depthMD.FrameID();
			*(unsigned int*)(depthBuffer + sizeof(unsigned int)) = j/DEPTH_SCALE;

#ifndef DEPTH_SCALE
			/* Fill the rest of the buffer with a line of depth data */
			memcpy(depthBuffer + 2*sizeof(unsigned int),
					pDepthMap+j*DEPTH_WIDTH,
					sizeof(XnDepthPixel)*DEPTH_WIDTH);
#else
			for (unsigned int i = 0; i<DEPTH_WIDTH; i+=DEPTH_SCALE) {
				*(XnDepthPixel*)(depthBuffer + 2*sizeof(unsigned int) + i/DEPTH_SCALE*sizeof(XnDepthPixel)) =
						pDepthMap[j*DEPTH_WIDTH + i];
			}
#endif
			
			/* Transmit the packet */
			if (sendto(depthSocket,
					depthBuffer,
					DEPTH_BUFFER_LENGTH,
					0,
					(const sockaddr*)&socketInfoDepth,
					sizeof(socketInfoDepth)) < 0) {
				fprintf(stderr, "Error sending depth packet to %s:%d (size %d, errno %d)\n", ip, depthPort,DEPTH_BUFFER_LENGTH,errno);
				exit(-1);
			}
		}
		
		/* Get RGB data */
		pImage = image.GetRGB24ImageMap();
		
		/* Map the RGB data to an OpenCV Mat */
		Mat rgbMat = Mat(RGB_HEIGHT,RGB_WIDTH,CV_8UC3,(void*)pImage);
		
		/* Encode the RGB image as a JPEG */
		vector<uchar> jpeg;
		vector<int> param = vector<int>(2);
        param[0]=CV_IMWRITE_JPEG_QUALITY;
        param[1]=75;
        imencode(".jpg",rgbMat,jpeg,param);
		
		/* Transmit the JPEG */
		/* TODO: Check that jpeg.size() is less than maximum UDP packet size
		 *		 otherwise this call to sendto will fail. Not a problem for
		 *		 small resolutions, possibly a problem with larger ones */
		if (sendto(rgbSocket,
				&jpeg[0],
				jpeg.size(),
				0,
				(const sockaddr*)&socketInfoRGB,
				sizeof(socketInfoRGB)) < 0) {
			fprintf(stderr, "Error sending UDP packet to %s:%d (size %d, errno %d)\n", ip, rgbPort,jpeg.size(),errno);
			exit(-1);
		}
		
		/* Let the user know how we're doing */
		printf("Frame %d broadcast\n", depthMD.FrameID());
	}
}
