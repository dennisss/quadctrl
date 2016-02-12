#include "onboard.h"

#include <XBee.h>



XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();




void xbee_setup(){
	Serial2.begin(115200);

	xbee.begin(Serial2);
}



bool xbee_ready(){


	xbee.readPacket();
	if(xbee.getResponse().isAvailable()){ // got something

		return true;
	}
	else if(xbee.getResponse().isError()){
		//nss.print("Error reading packet.  Error code: ");
		//nss.println(xbee.getResponse().getErrorCode());
	}


	return false;
}



void xbee_send(packet *pkt, unsigned int datalen){

	// SH + SL Address of receiving XBee
	XBeeAddress64 addr64 = XBeeAddress64(0, 0); // Send to coordinator

	ZBTxRequest zbTx = ZBTxRequest(addr64, (uint8_t *)pkt, (uint8_t)(sizeof(uint32_t) + datalen));
	ZBTxStatusResponse txStatus = ZBTxStatusResponse();

	xbee.send(zbTx);
}



void xbee_process(){


	if(xbee.getResponse().getApiId() == ZB_RX_RESPONSE){
		// got a zb rx packet

		// now fill our zb rx class
		xbee.getResponse().getZBRxResponse(rx);


		network_process((packet *)rx.getData(), rx.getDataLength());




		if(rx.getOption() == ZB_PACKET_ACKNOWLEDGED){
			// the sender got an ACK
			//flashLed(statusLed, 10, 10);
		}
		else{
			// we got it (obviously) but sender didn't get an ACK
			//flashLed(errorLed, 2, 20);
		}
		// set dataLed PWM to value of the first byte in the data
		//analogWrite(dataLed, rx.getData(0));
	}

	/*
		else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
			xbee.getResponse().getModemStatusResponse(msr);
			// the local XBee sends this response on certain events, like association/dissociation

			if (msr.getStatus() == ASSOCIATED) {
				// yay this is great.  flash led
				flashLed(statusLed, 10, 10);
			} else if (msr.getStatus() == DISASSOCIATED) {
				// this is awful.. flash led to show our discontent
				flashLed(errorLed, 10, 10);
			} else {
				// another status
				flashLed(statusLed, 5, 10);
			}
		} else {
			// not something we were expecting
			flashLed(errorLed, 1, 25);
		}
		*/

}
