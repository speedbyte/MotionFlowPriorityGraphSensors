// ExampleConsole.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include "RDBHandler.hh"
#include "main.h"

char  szServer[128];             // Server to connect to
int   iPort     = DEFAULT_PORT;  // Port on server to connect to

// function prototypes
void sendTrigger( int & sendSocket, const double & simTime, const unsigned int & simFrame );


void sendTrigger( int & sendSocket, const double & simTime, const unsigned int & simFrame )
{
    Framework::RDBHandler myHandler;

    myHandler.initMsg();

    RDB_TRIGGER_t *myTrigger = ( RDB_TRIGGER_t* ) myHandler.addPackage( simTime, simFrame, RDB_PKG_ID_TRIGGER );

    if ( !myTrigger )
        return;

    myTrigger->frameNo = simFrame + 1;
    myTrigger->deltaT  = 0.043;

    fprintf( stderr, "sendTrigger: sending trigger: \n" );

    myHandler.printMessage( myHandler.getMsg() );

    int retVal = send( sendSocket, ( const char* ) ( myHandler.getMsg() ), myHandler.getMsgTotalSize(), 0 );

    if ( !retVal )
        fprintf( stderr, "sendTrigger: could not send trigger\n" );

}


