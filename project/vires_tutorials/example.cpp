/**
this is a rough example how to compose a message including an object state and send it
to the taskControl (TC) via network. Note: the example is not complete and basic
networking stuff must be realized by the user
*/
bool
DynamicsPlugin::sendFeedback( )
{
    // for the moment: no feedback if playerID is invalid
    if ( mOwnPlayerId < 0 )
        return false;

    // instantiate an RDB handler object
    Framework::RDBHandler myHandler;

    // compose the start of the frame
    myHandler.addPackage( mSimTime, mFrameNo, RDB_PKG_ID_START_OF_FRAME, 1, 0, 0 );

    // add the object state of the own vehicle
    RDB_OBJECT_STATE_t* objState = ( RDB_OBJECT_STATE_t* ) myHandler.addPackage( mSimTime, mFrameNo, RDB_PKG_ID_OBJECT_STATE, 1, true );    // extended package

    // fill immediately since memory might have been re-allocated and pointers might become
    // invalid by additional addPackage commands

    if ( !objState )
        return false;

    // fill the message data from a given source state (srcState) or explicitly member by member
    memcpy( objState, srcState, RDBpkgId2size( RDB_PKG_ID_OBJECT_STATE, true ) );

    // add the vehicle systems of the own player
    RDB_VEHICLE_SYSTEMS_t* tgtSys = ( RDB_VEHICLE_SYSTEMS_t* ) myHandler.addPackage( mSimTime, mFrameNo, RDB_PKG_ID_VEHICLE_SYSTEMS, 1 );  // basic package

    if ( !tgtSys )
        return false;

    // fill the message data from a given source (srcSys) or explicitly member by member
    memcpy( tgtSys, srcSys, RDBpkgId2size( RDB_PKG_ID_VEHICLE_SYSTEMS, false ) );


    // fill any other date (e.g. wheels)


    // now add the end of the frame
    myHandler.addPackage( mSimTime, mFrameNo, RDB_PKG_ID_END_OF_FRAME, 1, 0, 0 );

    // and finally send the whole stuff via network
    if ( myHandler.getMsg() )
        ModulePlugin::sendFeedback( myHandler.getMsg(), myHandler.getMsgTotalSize() );

    return true;
}



/* ------------------------------------------------------------------------------------------------------------------------------------- */

/**
very rough example of using the RDBHandler
- in this example, two images are written into two different buffers of the
  image generator's shared memory
- opening the shared memory is the user's task
(c)2012 VIRES Simulationstechnologie GmbH
*/

int
VigLightMapTest::dualBufferTest( SensorIface* sensorData )
{
    Framework::RDBHandler myHandler;

    unsigned int imgWidth  = 1024;
    unsigned int imgHeight = 512;
    unsigned int imgSize   = imgWidth * imgHeight * 4 * sizeof( char );

    // write the stuff to shared memory
    // get a pointer to the header of the shared memory
    if ( !myHandler.configureShm( mIgShm->getStart(), 2 ) )        //allocate two buffers
        return 0;

    // do this for two buffers
    for ( int bufferId = 0; bufferId < 2; bufferId++ )
    {
        // free the current message queue and provide room for the next one
        myHandler.initMsg();

        // compose the message which is to be located within the shared memory
        RDB_IMAGE_t* imageData = ( RDB_IMAGE_t* ) myHandler.addPackage( sensorData->mSimTime, sensorData->mFrameNo, RDB_PKG_ID_LIGHT_MAP, 1,false, imgSize );

        if ( !imageData )
            return 0;

        imageData->id          = bufferId + 1;
        imageData->width       = imgWidth;
        imageData->height      = imgHeight;
        imageData->pixelSize   = 32;
        imageData->pixelFormat = RDB_PIX_FORMAT_RGBA_24;
        imageData->imgSize     = imgSize;
        imageData->color[0]    = 255;
        imageData->color[1]    = 255;
        imageData->color[2]    = bufferId * 255;
        imageData->color[3]    = 255;

        float* dataPtr = ( float* ) ( ( ( char* ) imageData ) + sizeof( RDB_IMAGE_t ) );

        for ( unsigned int j = 0; j < imgHeight; j++ )
        {
            for ( unsigned int i = 0; i < imgWidth; i++ )
            {
                *dataPtr = 0.5 * ( sin( 0.01 * sensorData->mFrameNo + ( bufferId * 0.5 * M_PI ) ) + 1.0 ) * ( 10.0f * i + 10.0f * j );
                dataPtr++;
            }
        }

        // tell the buffers about their IDs and flags
        myHandler.setShmBufferId( bufferId, bufferId );
        myHandler.setShmBufferFlags( bufferId, RDB_SHM_BUFFER_FLAG_IG );

        // map the local message data to the target location
        myHandler.mapMsgToShm( bufferId );

        fprintf( stderr, "VigLightMapTest::dualBufferTest: wrote image to SHM, frameNo = %ld\n",
                 sensorData->mFrameNo );
        fprintf( stderr, "VigLightMapTest::dualBufferTest: shmHdr:    headerSize = %d, dataSize = %d, noBuffers = %d\n",
                 myHandler.getShmHdr()->headerSize,myHandler.getShmHdr()->dataSize, myHandler.getShmHdr()->noBuffers );
        fprintf( stderr, "VigLightMapTest::dualBufferTest: bufferInfo: thisSize = %d, bufferSize = %d, id = %d, flags = 0x%x, offset = %d\n",
                 myHandler.getShmBufferInfo( bufferId )->thisSize, myHandler.getShmBufferInfo( bufferId )->bufferSize, myHandler.getShmBufferInfo( bufferId )->id,
                 myHandler.getShmBufferInfo( bufferId )->flags, myHandler.getShmBufferInfo( bufferId )->offset );
        fprintf( stderr, "VigLightMapTest::dualBufferTest: msgHdr:    headerSize = %d, dataSize = %d, frameNo = %d, magicNo = %d, simTime = %.3lf\n",
                 myHandler.getMsgHdr()->headerSize, myHandler.getMsgHdr()->dataSize, myHandler.getMsgHdr()->frameNo,
                 myHandler.getMsgHdr()->magicNo, myHandler.getMsgHdr()->simTime );
    }
    return 1;
}


int writeTriggerToShm()
{
    if ( !mShmPtr_writer )
        return 0;

    // get access to the administration information of the first RDB buffer in SHM
    RDB_SHM_BUFFER_INFO_t* info = mRdbHandler.shmBufferGetInfo( 0 );

    if ( !info )
        return 0;

    // is the buffer ready for write?
    if ( info->flags )      // is the buffer accessible (flags == 0)?
        return 0;

    // clear the buffer before writing to it (otherwise messages will accumulate)
    if ( !mRdbHandler.shmBufferClear( 0, true ) )   // true = clearing will be forced; not recommended!
        return 0;

    fprintf( stderr, "writeTriggerToShm: sending single trigger\n" );

    // increase the frame number
    mFrameNo++;

    // create a message containing the sync information
    RDB_SYNC_t* syncData = ( RDB_SYNC_t* ) mRdbHandler.addPackage( mFrameNo * mFrameTime, mFrameNo, RDB_PKG_ID_SYNC );

    if ( !syncData )
        return 0;

    syncData->mask    = 0x0;
    syncData->cmdMask = RDB_SYNC_CMD_RENDER_SINGLE_FRAME;

    // set some information concerning the RDB buffer itself
    info->id    = 1;
    info->flags = RDB_SHM_BUFFER_FLAG_IG;

    // now copy the symc message to the first RDB buffer in SHM
    mRdbHandler.mapMsgToShm( 0 );

    return 1;
}


else if ( strcmp(argv[1], "write") == 0 ) {
ValidateArgs(argc, argv);

// first: open the shared memory (try to attach without creating a new segment)

fprintf( stderr, "attaching to shared memory....\n" );

while ( !mShmPtr_writer )
{
openShm();
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
