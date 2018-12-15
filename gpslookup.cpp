#include "Arduino.h"
#include "gpslookup.h"
Gpslookup::Gpslookup(){}
unsigned char Gpslookup::charToHex(unsigned char c) {
  if(c > 47 && c < 58) {
    return (c - 48);
  } else if(c > 64 && c < 71) {
    return (c - 55);
  }
  return 0;
}
bool Gpslookup::gpsdata(char data){	
  if (data != -1){
	if (data == '$' && !dataReady) { // start of sentence, data in buffer was processed
		rxWriteIndex = 0;
		bufferEnabled = 1;
		xorcalcEnabled = 0;
		rxValueIndex=0;
		xorcalc = 0;
		checksum=0;
	}
//	satelites=0;
	if (bufferEnabled) {
		if (data == ','){
			if (rxValueIndex==0){
				for (i=0; i<6; i++){
					if(rxBuffer[i]!=starting[i])bufferEnabled=0;
				}
			}
			if (rxValueIndex==1){
				GPShour = (rxBuffer[1]-48)+((rxBuffer[0]-48)*10);
        GPSmin = (rxBuffer[3]-48)+((rxBuffer[2]-48)*10);
        GPSsec = (rxBuffer[5]-48)+((rxBuffer[4]-48)*10);
			}
      if (rxValueIndex==2){       
        if (rxBuffer[0]==65){
          Status=1;
        }else{
          Status=0;
        }
      }
			if (rxValueIndex==3){
				for (i=0; i<8; i++)lat[i]=rxBuffer[i];
				if (rxWriteIndex<4)bufferEnabled=0;
			}
			if (rxValueIndex==4){
				i=rxBuffer[0];
				if (i==83)North=1;
				if (i==78)North=0;
			}
			if (rxValueIndex==5){
				for (i=0; i<9; i++)lon[i]=rxBuffer[i];
				if (rxWriteIndex<4)bufferEnabled=0;
			}
			if (rxValueIndex==6){
				i=rxBuffer[0];
				if (i==69) East=0;
				if (i==87) East=1;
			}
			if (rxValueIndex==7){
        cog = ((rxBuffer[0]-48)*1000)+((rxBuffer[1]-48)*100)+((rxBuffer[2]-48)*10)+(rxBuffer[4]-48);
			}
      if (rxValueIndex==8){
        sog = ((rxBuffer[0]-48)*1000)+((rxBuffer[1]-48)*100)+((rxBuffer[2]-48)*10)+(rxBuffer[4]-48);
      }
      if (rxValueIndex==9){
        GPSday = (rxBuffer[1]-48)+((rxBuffer[0]-48)*10);
        GPSmonth = (rxBuffer[3]-48)+((rxBuffer[2]-48)*10);
        GPSyear = (rxBuffer[5]-48)+((rxBuffer[4]-48)*10);
      }
			for (i=0; i<=rxWriteIndex;i++){
				rxBuffer[i]=0;
			}
			rxWriteIndex = 0;
			rxValueIndex++;
		}else {
			rxBuffer[rxWriteIndex++] = data;
		}
		if (xorcalcEnabled ==0 && data != '$') {
			if (data == '*') {
				xorcalcEnabled=1;
				checksumpos=1;
				checksum=0;
			}else{
				xorcalc ^= data;
			}
		}else{
			if(xorcalcEnabled==1){
				if (checksumpos){
					checksum = charToHex(data) << 4;
					checksumpos=0;
				}else{
					checksum += charToHex(data);
					xorcalcEnabled=0;
					bufferEnabled=0;
					/**** Got GPS Position and Check it OK to process ***/
					if ((xorcalc == checksum)) {
						if(Status==1){
						  for (i=0; i<9;i++){
						    	//PutChar(rxBuffer[i]);
								rxBuffer[i]=0;
							}
							/*
              latitude=((lat[0]-48)*6000000)+((lat[1]-48)*600000)+((lat[2]-48)*10000)+((lat[3]-48)*1000)+((lat[5]-48)*100)+((lat[6]-48)*10)+(lat[7]-48);
              if (North)latitude=-latitude;
							longitude=((lon[0]-48)*60000000)+((lon[1]-48)*6000000)+((lon[2]-48)*600000)+((lon[3]-48)*10000)+((lon[4]-48)*1000)+((lon[6]-48)*100)+((lon[7]-48)*10)+(lon[8]-48);
              if (East)longitude=-longitude;
              //utc= GPSsec + GPSmin*60 + GPShour*3600 + tm_yday*86400+(GPSyear-70)*31536000+((GPSyear-69)/4)*86400 - ((GPSyear-1)/100)*86400 + ((GPSyear+299)/400)*86400;
							*/
							return(true);
						}
				   }
					}
				}

		}
	}
  return(false);
}

}
