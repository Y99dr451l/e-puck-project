#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H


void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size);

void SendInt8ToComputer(BaseSequentialStream* out, int8_t* data, uint16_t size);

void SendInt16ToComputer(BaseSequentialStream* out, int16_t* data, uint16_t size);

uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in, int16_t* data, uint16_t size);

uint16_t ReceiveFloatFromComputer(BaseSequentialStream* in, float* data, uint16_t size);

#endif /* COMMUNICATIONS_H */
