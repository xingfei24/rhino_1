#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H

#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

class NeuralNetwork
{
public:
    NeuralNetwork(int inputSize, int hiddenSize, int outputSize);
    ~NeuralNetwork();

    void forward(float *input, float *output);
    void backward(float *target, float learningRate);
    void save(const char *filename);
    void load(const char *filename);
    void train(float *input, float *target, float learningRate);

private:
    int inputSize;
    int hiddenSize;
    int outputSize;

    float *inputLayer;
    float *hiddenLayer;
    float *outputLayer;

    float *weightsIH; // Input to Hidden weights
    float *weightsHO; // Hidden to Output weights

    float *biasH; // Hidden layer bias
    float *biasO; // Output layer bias

    void initializeWeights();
    float sigmoid(float x);
    float sigmoidDerivative(float x);
};

#endif