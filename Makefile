server: server.cpp
	g++ -I/usr/include/ni server.cpp -lOpenNI `pkg-config --libs opencv` -oserver
