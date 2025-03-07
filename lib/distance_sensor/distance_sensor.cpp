#include "distance_sensor.h"
#ifdef ARDUINO
#include <Arduino.h>
#include <wiring_private.h>
#endif

DistanceSensor::DistanceSensor(uint8_t SensorPin, long SensorModel) {
    _sensorPin = SensorPin;
    _model = SensorModel;

    pinMode(_sensorPin, INPUT);
}

void DistanceSensor::sort(int a[], int size) {
    for(int i = 0; i < size-1; i++) {
		bool swapped = false;
		for(int j = 0; j < size-i-1; j++) {
			if(a[j] > a[j+1]) {
				float temp = a[j];
				a[j] = a[j+1];
				a[j+1] = temp;
				swapped = true;
			}
		}
		if (!swapped) break;
	}
}

int DistanceSensor::getDistance() {
    int ir_val[SAMPLES] = {};
	int distanceCM;
	int median;

	for (int i=0; i<SAMPLES; i++){
		ir_val[i] = analogRead(_sensorPin);
	}

	sort(ir_val, SAMPLES);
	median = ir_val[SAMPLES/2];

	if (_model == 1080) {
		if (median > 1000) distanceCM = 0;
		else if (median < 1000 && median > 150)
			distanceCM = 29.988 * pow(map(median, 0, 1023, 0, 5000)/1000.0, -1.173);
		else 
			distanceCM = 80;
    }
	else if (_model == 20150) {
		if (median > 1000 ) distanceCM = 0; 
		else if (median < 1000 && median > 150)
			distanceCM = 60.374 * pow(map(median, 0, 1023, 0, 5000)/1000.0, -1.16);
		else 
			distanceCM = 150;
    }
    else if (_model == 430) {
        if (median > 1000) distanceCM = 0;
        else if (median < 1000 && median > 150)
            distanceCM = 12.08 * pow(map(median, 0, 1023, 0, 5000)/1000.0, -1.058);
        else 
            distanceCM = 30;
    }
	else 
		return -1;
	return distanceCM;
}