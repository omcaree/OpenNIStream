#!/bin/sh

g++ -I/usr/include/ni server.cpp -lOpenNI `pkg-config --libs opencv` -oserver
