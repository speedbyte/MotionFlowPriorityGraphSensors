//#include "ImageUtils.h"
#include <boost/bind.hpp>
#include "vires/RDBHandler.hh"

#include <sys/ipc.h>
#include <sys/shm.h>

#include <boost/filesystem.hpp>

#include <iostream>

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


/**
* information about usage of the software
* this method will exit the program
*/
void usage()
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
                    usage();
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
    std::cout << "start my program";
    std::string m_server;
    int m_port;
    int m_sensor_port;
    bool m_triggers;
    int m_frames_to_read;
    bool m_write_ts_gt;
    boost::filesystem::path m_ts_gt_out_dir;

}




