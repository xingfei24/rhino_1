#ifndef DATA_COLLECTOR_H
#define DATA_COLLECTOR_H

#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "MpuData.h"

class DataCollector
{
public:
  DataCollector();
  void begin();
  void collectData(const MpuData &mpuData, const float *legAngles, int numAngles);
  void saveToSD(const char *filename);
  void loadFromSD(const char *filename);

  float *getTrainingData() { return trainingData; }
  float *getTargetData() { return targetData; }
  int getDataCount() { return dataCount; }

private:
  static const int MAX_SAMPLES = 1000;
  static const int FEATURES_PER_SAMPLE = 14; // 6 MPU + 8 leg angles

  float *trainingData;
  float *targetData;
  int dataCount;

  void normalizeData();
};

#endif