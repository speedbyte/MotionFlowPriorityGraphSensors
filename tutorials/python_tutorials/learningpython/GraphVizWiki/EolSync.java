/*
 * Schema model
 * UML User Guide p. 112
 */

/**
 * @opt operations
 * @opt attributes
 * @opt types
 * @hidden
 */
class UMLOptions {}

/* Define some types we use */
/** @hidden */
class Array {}
/** @hidden */
class Number {}
/** @hidden */
class Name {}
/**
 * @note this class receives the 
 * payload from the Payloadprotocol 
 * class and divides it into segments 
 * according to the ISOTP protocol
 */
class IsoTpProtocol {
        Array Payload;
        void getCanMessages() {}
        void getFlowControlMessage() {}
        void convertCanToEolFrame() {}

}

/**
 * @extends IsoTpProtocol
 * @note This class receives the payload 
 * from the CreatePayload class and 
 * interprets the raw value into 
 * human understandable form
 */
class PayloadProtocol {
		void setServiceId() {}
        void setRestPayload() {}
        void getPayload() {}
        void getResponseId() {}
        void getTestStatus() {}
}

/**
 * @extends PayloadProtocol
 * @note This class receives a Payload 
 * from the application and forwards 
 * it to lower layers.
 */



class CreatePayload {
        void createPayloadAndSegment() {}
        void createFlowControlMessage() {}
}


/**
 * @extends CreatePayload
 * @note Application program to issue 
 * series of Can commands to Sync module 
 * automatically for EMI Tests
 */
class EolApp {
    Name name;
    Number courseID;
    void SendReceiveMain() {}
}