#ifndef __func_h__
#define __func_h__

float myMap(float value, float fromMinValue, float fromMaxValue, float toMinValue, float toMaxValue){
    if (fromMinValue > fromMaxValue){
        float temp = fromMaxValue;
        fromMaxValue = fromMinValue;
        fromMinValue = temp;
        if (value < fromMinValue){ return toMaxValue; }
        if (value > fromMaxValue){ return toMinValue; }
        return (fromMaxValue - value) / (fromMaxValue - fromMinValue) * (toMaxValue - toMinValue) + toMinValue;
    } else {
        if (value < fromMinValue){ return toMinValue; }
        if (value > fromMaxValue){ return toMaxValue; }
        return (value - fromMinValue) / (fromMaxValue - fromMinValue) * (toMaxValue - toMinValue) + toMinValue;
    }
}

int minMax(int value, int minValue, int maxValue){
	if (value > maxValue) { value = maxValue; }
	else if (value < minValue) { value = minValue; }
	return value;
}

#endif