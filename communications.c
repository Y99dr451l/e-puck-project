#include "ch.h"
#include "hal.h"
#include <main.h>
#include "communications.h"

void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size) {
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(float) * size);
}

void SendInt8ToComputer(BaseSequentialStream* out, int8_t* data, uint16_t size) {
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(int8_t) * size);
}

void SendInt16ToComputer(BaseSequentialStream* out, int16_t* data, uint16_t size) {
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(int16_t) * size);
}

uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in, int16_t* data, uint16_t size) {
	volatile uint8_t c1, c2;
	volatile uint16_t temp_size = 0;
	uint8_t state = 0;
	while(state != 5) {
        c1 = chSequentialStreamGet(in);
        switch(state) {
        	case 0:
        		if(c1 == 'S') state = 1;
        		else state = 0;
        	case 1:
        		if(c1 == 'T') state = 2;
        		else if(c1 == 'S') state = 1;
        		else state = 0;
        	case 2:
        		if(c1 == 'A') state = 3;
        		else if(c1 == 'S') state = 1;
        		else state = 0;
        	case 3:
        		if(c1 == 'R') state = 4;
        		else if(c1 == 'S') state = 1;
        		else state = 0;
        	case 4:
        		if(c1 == 'T') state = 5;
        		else if(c1 == 'S') state = 1;
        		else state = 0;
        }
	}
	c1 = chSequentialStreamGet(in);
	c2 = chSequentialStreamGet(in);
	temp_size = (int16_t)((c1 | c2<<8));
	if((temp_size/2) == size)
		for(uint16_t i = 0; i < (temp_size/2); i++) {
			c1 = chSequentialStreamGet(in);
			c2 = chSequentialStreamGet(in);
			data[i] = (int16_t)((c1 | c2 << 8));
		}
	return temp_size/2;
}

uint16_t ReceiveFloatFromComputer(BaseSequentialStream* in, float* data, uint16_t size) {
	volatile uint8_t c1, c2;
	volatile uint16_t temp_size = 0;
	uint8_t state = 0;
	while(state != 5){
        c1 = chSequentialStreamGet(in);
        switch(state){
        	case 0:
        		if(c1 == 'S') state = 1;
        		else state = 0;
        	case 1:
        		if(c1 == 'T') state = 2;
        		else if(c1 == 'S') state = 1;
        		else state = 0;
        	case 2:
        		if(c1 == 'A') state = 3;
        		else if(c1 == 'S') state = 1;
        		else state = 0;
        	case 3:
        		if(c1 == 'R') state = 4;
        		else if(c1 == 'S') state = 1;
        		else state = 0;
        	case 4:
        		if(c1 == 'T') state = 5;
        		else if(c1 == 'S') state = 1;
        		else state = 0;
        }
	}
	c1 = chSequentialStreamGet(in);
	c2 = chSequentialStreamGet(in);
	temp_size = (int16_t)((c1 | c2 << 8));
	if((temp_size/4) == size) {
		uint8_t buffer[4] = {0, 0, 0, 0};
		for(uint16_t i = 0; i < temp_size/4; i++) {
			for(int j = 0; j < 4; j++) buffer[j] = chSequentialStreamGet(in);
//			for(int j = 3; j >= 0; j--) buffer[j] = chSequentialStreamGet(in);
			data[i] = *(float *)&buffer;
		}
	}
	return temp_size/4;
}
