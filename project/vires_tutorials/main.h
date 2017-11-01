//
// Created by veikas on 01.11.17.
//

#ifndef VIRES_TUTORIAL_MAIN_H
#define VIRES_TUTORIAL_MAIN_H




#define DEFAULT_PORT        48190   /* for image port it should be 48192 */
#define DEFAULT_BUFFER      204800

#define DEFAULT_RX_PORT     48190   /* for image port it should be 48192 */
#define DEFAULT_TX_PORT     48191

class MyRDBHandler : Framework::RDBHandler {

public:
    void parseStartOfFrame(const double &simTime, const unsigned int &simFrame);

    void parseEndOfFrame( const double & simTime, const unsigned int & simFrame );

    void parseEntry( RDB_OBJECT_CFG_t *data, const double & simTime, const unsigned int &
    simFrame, const unsigned short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem );

    void parseEntry( RDB_OBJECT_STATE_t *data, const double & simTime, const unsigned int & simFrame, const unsigned
    short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem );

    void parseEntry( RDB_IMAGE_t *data, const double & simTime, const unsigned int & simFrame, const unsigned
    short & pkgId, const unsigned short & flags, const unsigned int & elemId, const unsigned int & totalElem );

    void parseMessage( RDB_MSG_t* msg );

};

#endif //VIRES_TUTORIAL_MAIN_H
