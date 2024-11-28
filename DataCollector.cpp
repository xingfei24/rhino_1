#include "DataCollector.h"

DataCollector::DataCollector()
{
    trainingData = new float[MAX_SAMPLES * FEATURES_PER_SAMPLE];
    targetData = new float[MAX_SAMPLES * 8]; // 8 target angles for 4 legs
    dataCount = 0;
}

void DataCollector::begin()
{
    if (!SD.begin())
    {
        Serial.println("SD Card initialization failed!");
        return;
    }
}

void DataCollector::collectData(const MpuData &mpuData, const float *legAngles, int numAngles)
{
    if (dataCount >= MAX_SAMPLES)
        return;

    int idx = dataCount * FEATURES_PER_SAMPLE;

    // Store MPU data
    trainingData[idx++] = mpuData.accel_x;
    trainingData[idx++] = mpuData.accel_y;
    trainingData[idx++] = mpuData.accel_z;
    trainingData[idx++] = mpuData.gyro_x;
    trainingData[idx++] = mpuData.gyro_y;
    trainingData[idx++] = mpuData.gyro_z;

    // Store leg angles
    for (int i = 0; i < numAngles; i++)
    {
        trainingData[idx++] = legAngles[i];
    }

    // Store target data (ideal angles for stable walking)
    int targetIdx = dataCount * 8;
    for (int i = 0; i < 8; i++)
    {
        targetData[targetIdx + i] = legAngles[i]; // Initially use current angles as targets
    }

    dataCount++;
}

void DataCollector::saveToSD(const char *filename)
{
    File file = SD.open(filename, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }

    file.write((uint8_t *)&dataCount, sizeof(dataCount));
    file.write((uint8_t *)trainingData, dataCount * FEATURES_PER_SAMPLE * sizeof(float));
    file.write((uint8_t *)targetData, dataCount * 8 * sizeof(float));

    file.close();
}

void DataCollector::loadFromSD(const char *filename)
{
    File file = SD.open(filename);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return;
    }

    file.read((uint8_t *)&dataCount, sizeof(dataCount));
    file.read((uint8_t *)trainingData, dataCount * FEATURES_PER_SAMPLE * sizeof(float));
    file.read((uint8_t *)targetData, dataCount * 8 * sizeof(float));

    file.close();
}

void DataCollector::normalizeData()
{
    // Calculate mean and standard deviation for each feature
    float *means = new float[FEATURES_PER_SAMPLE];
    float *stdDevs = new float[FEATURES_PER_SAMPLE];

    // Initialize arrays
    for (int i = 0; i < FEATURES_PER_SAMPLE; i++)
    {
        means[i] = 0;
        stdDevs[i] = 0;
    }

    // Calculate means
    for (int i = 0; i < dataCount; i++)
    {
        for (int j = 0; j < FEATURES_PER_SAMPLE; j++)
        {
            means[j] += trainingData[i * FEATURES_PER_SAMPLE + j];
        }
    }

    for (int i = 0; i < FEATURES_PER_SAMPLE; i++)
    {
        means[i] /= dataCount;
    }

    // Calculate standard deviations
    for (int i = 0; i < dataCount; i++)
    {
        for (int j = 0; j < FEATURES_PER_SAMPLE; j++)
        {
            float diff = trainingData[i * FEATURES_PER_SAMPLE + j] - means[j];
            stdDevs[j] += diff * diff;
        }
    }

    for (int i = 0; i < FEATURES_PER_SAMPLE; i++)
    {
        stdDevs[i] = sqrt(stdDevs[i] / dataCount);
        if (stdDevs[i] < 1e-7)
            stdDevs[i] = 1; // Prevent division by zero
    }

    // Normalize the data
    for (int i = 0; i < dataCount; i++)
    {
        for (int j = 0; j < FEATURES_PER_SAMPLE; j++)
        {
            trainingData[i * FEATURES_PER_SAMPLE + j] =
                (trainingData[i * FEATURES_PER_SAMPLE + j] - means[j]) / stdDevs[j];
        }
    }

    delete[] means;
    delete[] stdDevs;
}