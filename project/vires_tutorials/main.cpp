//#include "ImageUtils.h"
#include <boost/bind.hpp>
#include "vires/RDBHandler.hh"

#include <sys/ipc.h>
#include <sys/shm.h>

#include <boost/filesystem.hpp>

#include <iostream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <netdb.h>

#include "main.h"
#include "kbhit.h"

extern unsigned int mShmPtr_writer;
extern unsigned int mShmKey_writer;
extern bool mVerbose;
extern unsigned int mShmPtr_reader;
extern unsigned int mShmKey_reader;
extern double       mFrameTime;
extern unsigned int mCheckMask;
extern int mForceBuffer;

extern void ValidateArgs_writer(int argc, char **argv);
extern void openShm_writer();
extern int  writeTriggerToShm();
extern void ValidateArgs_reader(int argc, char **argv);
extern void openShm_reader();
extern int checkShm();

extern char  szServer[128];             // Server to connect to
extern int   iPort;  // Port on server to connect to

extern void sendTrigger( int & sendSocket, const double & simTime, const unsigned int & simFrame );

/**
* information about usage of the software
* this method will exit the program
*/
void usage_reader()
{
    printf("usage: shmReader [-k:key] [-c:checkMask] [-v] [-f:bufferId]\n\n");
    printf("       -k:key        SHM key that is to be addressed\n");
    printf("       -c:checkMask  mask against which to check before reading an SHM buffer\n");
    printf("       -f:bufferId   force reading of a given buffer (0 or 1) instead of checking for a valid checkMask\n");
    printf("       -v            run in verbose mode\n");
    exit(1);
}

/**
* validate the arguments given in the command line
*/
void ValidateArgs_reader(int argc, char **argv)
{
    for( int i = 1; i < argc; i++)
    {
        if ((argv[i][0] == '-') || (argv[i][0] == '/'))
        {
            switch (tolower(argv[i][1]))
            {
                case 'k':        // shared memory key
                    if ( strlen( argv[i] ) > 3 )
                        mShmKey_reader = atoi( &argv[i][3] );
                    break;

                case 'c':       // check mask
                    if ( strlen( argv[i] ) > 3 )
                        mCheckMask = atoi( &argv[i][3] );
                    break;

                case 'f':       // force reading a given buffer
                    if ( strlen( argv[i] ) > 3 )
                        mForceBuffer = atoi( &argv[i][3] );
                    break;

                case 'v':       // verbose mode
                    mVerbose = true;
                    break;

                default:
                    usage_reader();
                    break;
            }
        }
    }

    fprintf( stderr, "ValidateArgs: key = 0x%x, checkMask = 0x%x, mForceBuffer = %d\n",
             mShmKey_reader, mCheckMask, mForceBuffer );
}


/**
* information about usage of the software
* this method will exit the program
*/
void usage_writer()
{
    printf("usage: shmWriter [-k:key]\n\n");
    printf("       -k:key        SHM key that is to be addressed\n");
    printf("       -v            run in verbose mode\n");
    exit(1);
}


/**
* validate the arguments given in the command line
*/
void ValidateArgs_writer(int argc, char **argv)
{
    for( int i = 2; i < argc; i++)
    {
        if ((argv[i][0] == '-') || (argv[i][0] == '/'))
        {
            switch (tolower(argv[i][1]))
            {
                case 'k':        // shared memory key
                    if ( strlen( argv[i] ) > 3 )
                        mShmKey_writer = atoi( &argv[i][3] );
                    break;

                case 'v':       // verbose mode
                    mVerbose = true;
                    break;

                default:
                    usage_writer();
                    break;
            }
        }
    }

    fprintf( stderr, "ValidateArgs: key = 0x%x\n", mShmKey_writer );
}


//
// Function: usage:
//
// Description:
//    Print usage information and exit
//
void usage_trigger()
{
    printf("usage: client [-p:x] [-s:IP]\n\n");
    printf("       -p:x      Remote port to send to\n");
    printf("       -s:IP     Server's IP address or hostname\n");
    exit(1);
}

//
// Function: ValidateArgs
//
// Description:
//    Parse the command line arguments, and set some global flags
//    to indicate what actions to perform
//
void ValidateArgs_trigger(int argc, char **argv)
{
    int i;

    strcpy( szServer, "127.0.0.1" );

    for(i = 1; i < argc; i++)
    {
        if ((argv[i][0] == '-') || (argv[i][0] == '/'))
        {
            switch (tolower(argv[i][1]))
            {
                case 'p':        // Remote port
                    if (strlen(argv[i]) > 3)
                        iPort = atoi(&argv[i][3]);
                    break;
                case 's':       // Server
                    if (strlen(argv[i]) > 3)
                        strcpy(szServer, &argv[i][3]);
                    break;
                default:
                    usage_trigger();
                    break;
            }
        }
    }
}

/**
* main program with high frequency loop for checking the shared memory;
* does nothing else
*/

int main(int argc, char* argv[])
{
    // Parse the command line
    //
    if ( strcmp(argv[1], "write") == 0 ) {
        ValidateArgs_writer(argc, argv);

        // first: open the shared memory (try to attach without creating a new segment)

        fprintf( stderr, "attaching to shared memory....\n" );

        while ( !mShmPtr_writer )
        {
            openShm_writer();
            usleep( 1000 );     // do not overload the CPU
        }

        fprintf( stderr, "...attached! Triggering now...\n" );

        // now write the trigger to the SHM for the time being
        while ( 1 )
        {
            writeTriggerToShm();

            usleep( ( unsigned int ) ( 1.e6 * mFrameTime ) );    // wait for frame time (not very precise!)
        }
    }
    else if ( strcmp(argv[1], "read") == 0 ) {
        // Parse the command line
        //
        ValidateArgs_reader(argc, argv);

        // first: open the shared memory (try to attach without creating a new segment)

        fprintf( stderr, "attaching to shared memory....\n" );

        while ( !mShmPtr_reader )
        {
            openShm_reader();
            usleep( 100000 );     // do not overload the CPU
        }

        fprintf( stderr, "...attached! Reading now...\n" );

        // now check the SHM for the time being
        while ( 1 )
        {
            checkShm();

            usleep( 1000 );
        }
    }
    else if ( strcmp(argv[1], "trigger") == 0 )  {
        int           sClient;
        char* szBuffer = new char[DEFAULT_BUFFER];  // allocate on heap
        int           ret;
        struct sockaddr_in server;
        struct hostent    *host = NULL;
        static bool sSendTrigger = true;

        // Parse the command line
        //
        ValidateArgs_trigger(argc, argv);

        //
        // Create the socket, and attempt to connect to the server
        //
        sClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

        if ( sClient == -1 )
        {
            fprintf( stderr, "socket() failed: %s\n", strerror( errno ) );
            return 1;
        }

        int opt = 1;
        setsockopt ( sClient, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof( opt ) );

        server.sin_family      = AF_INET;
        server.sin_port        = htons(iPort);
        server.sin_addr.s_addr = inet_addr(szServer);

        //
        // If the supplied server address wasn't in the form
        // "aaa.bbb.ccc.ddd" it's a hostname, so try to resolve it
        //
        if ( server.sin_addr.s_addr == INADDR_NONE )
        {
            host = gethostbyname(szServer);
            if ( host == NULL )
            {
                fprintf( stderr, "Unable to resolve server: %s\n", szServer );
                return 1;
            }
            memcpy( &server.sin_addr, host->h_addr_list[0], host->h_length );
        }
        // wait for connection
        bool bConnected = false;

        while ( !bConnected )
        {
            if (connect( sClient, (struct sockaddr *)&server, sizeof( server ) ) == -1 )
            {
                fprintf( stderr, "connect() failed: %s\n", strerror( errno ) );
                sleep( 1 );
            }
            else
                bConnected = true;
        }

        fprintf( stderr, "connected!\n" );

        unsigned int  bytesInBuffer = 0;
        size_t        bufferSize    = sizeof( RDB_MSG_HDR_t );
        unsigned int  count         = 0;
        unsigned char *pData        = ( unsigned char* ) calloc( 1, bufferSize );

        // Send and receive data - forever!
        //
        for(;;)
        {
            bool bMsgComplete = false;

            // CAREFUL: we are not reading the network, so the test will fail after a while!!!

            // do some other stuff before returning to network reading
            usleep( 500000 );

            int val = kbhit();
            if ( val == 97 ) {
                sSendTrigger = true;
            }
            else if ( val == 'c') {
                sSendTrigger = false;
            }
            else if ( val == 'i') {
                sSendTrigger = false;
                sendTrigger( sClient, 0.0, 0 );
            }
            if ( sSendTrigger )
                sendTrigger( sClient, 0.0, 0 );
        }
        ::close(sClient);

        return 0;
    }
    std::string m_server;
    int m_port;
    int m_sensor_port;
    bool m_triggers;
    int m_frames_to_read;
    bool m_write_ts_gt;
    boost::filesystem::path m_ts_gt_out_dir;

}




