#include "NeuralNetwork.h"

NeuralNetwork::NeuralNetwork(int inputSize, int hiddenSize, int outputSize)
    : inputSize(inputSize), hiddenSize(hiddenSize), outputSize(outputSize)
{

    inputLayer = new float[inputSize];
    hiddenLayer = new float[hiddenSize];
    outputLayer = new float[outputSize];

    weightsIH = new float[inputSize * hiddenSize];
    weightsHO = new float[hiddenSize * outputSize];

    biasH = new float[hiddenSize];
    biasO = new float[outputSize];

    initializeWeights();
}

NeuralNetwork::~NeuralNetwork()
{
    delete[] inputLayer;
    delete[] hiddenLayer;
    delete[] outputLayer;
    delete[] weightsIH;
    delete[] weightsHO;
    delete[] biasH;
    delete[] biasO;
}

void NeuralNetwork::initializeWeights()
{
    // Xavier initialization
    float weightRange = sqrt(6.0 / (inputSize + hiddenSize));

    for (int i = 0; i < inputSize * hiddenSize; i++)
    {
        weightsIH[i] = (random(1000) / 1000.0f) * 2.0f * weightRange - weightRange;
    }

    weightRange = sqrt(6.0 / (hiddenSize + outputSize));

    for (int i = 0; i < hiddenSize * outputSize; i++)
    {
        weightsHO[i] = (random(1000) / 1000.0f) * 2.0f * weightRange - weightRange;
    }

    for (int i = 0; i < hiddenSize; i++)
    {
        biasH[i] = 0.0f;
    }

    for (int i = 0; i < outputSize; i++)
    {
        biasO[i] = 0.0f;
    }
}

float NeuralNetwork::sigmoid(float x)
{
    return 1.0f / (1.0f + exp(-x));
}

float NeuralNetwork::sigmoidDerivative(float x)
{
    float sx = sigmoid(x);
    return sx * (1 - sx);
}

void NeuralNetwork::forward(float *input, float *output)
{
    // Copy input
    memcpy(inputLayer, input, inputSize * sizeof(float));

    // Hidden layer
    for (int i = 0; i < hiddenSize; i++)
    {
        float sum = biasH[i];
        for (int j = 0; j < inputSize; j++)
        {
            sum += inputLayer[j] * weightsIH[j * hiddenSize + i];
        }
        hiddenLayer[i] = sigmoid(sum);
    }

    // Output layer
    for (int i = 0; i < outputSize; i++)
    {
        float sum = biasO[i];
        for (int j = 0; j < hiddenSize; j++)
        {
            sum += hiddenLayer[j] * weightsHO[j * outputSize + i];
        }
        outputLayer[i] = sigmoid(sum);
    }

    // Copy output
    memcpy(output, outputLayer, outputSize * sizeof(float));
}

void NeuralNetwork::backward(float *target, float learningRate)
{
    // Output layer error
    float *outputError = new float[outputSize];
    for (int i = 0; i < outputSize; i++)
    {
        outputError[i] = (target[i] - outputLayer[i]) * sigmoidDerivative(outputLayer[i]);
    }

    // Hidden layer error
    float *hiddenError = new float[hiddenSize];
    for (int i = 0; i < hiddenSize; i++)
    {
        float sum = 0.0f;
        for (int j = 0; j < outputSize; j++)
        {
            sum += outputError[j] * weightsHO[i * outputSize + j];
        }
        hiddenError[i] = sum * sigmoidDerivative(hiddenLayer[i]);
    }

    // Update weights and biases
    // Hidden to Output
    for (int i = 0; i < hiddenSize; i++)
    {
        for (int j = 0; j < outputSize; j++)
        {
            weightsHO[i * outputSize + j] += learningRate * outputError[j] * hiddenLayer[i];
        }
    }

    // Input to Hidden
    for (int i = 0; i < inputSize; i++)
    {
        for (int j = 0; j < hiddenSize; j++)
        {
            weightsIH[i * hiddenSize + j] += learningRate * hiddenError[j] * inputLayer[i];
        }
    }

    // Update biases
    for (int i = 0; i < outputSize; i++)
    {
        biasO[i] += learningRate * outputError[i];
    }

    for (int i = 0; i < hiddenSize; i++)
    {
        biasH[i] += learningRate * hiddenError[i];
    }

    delete[] outputError;
    delete[] hiddenError;
}

void NeuralNetwork::train(float *input, float *target, float learningRate)
{
    forward(input, outputLayer);
    backward(target, learningRate);
}

void NeuralNetwork::save(const char *filename)
{
    File file = SD.open(filename, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }

    // Save network architecture
    file.write((uint8_t *)&inputSize, sizeof(inputSize));
    file.write((uint8_t *)&hiddenSize, sizeof(hiddenSize));
    file.write((uint8_t *)&outputSize, sizeof(outputSize));

    // Save weights and biases
    file.write((uint8_t *)weightsIH, inputSize * hiddenSize * sizeof(float));
    file.write((uint8_t *)weightsHO, hiddenSize * outputSize * sizeof(float));
    file.write((uint8_t *)biasH, hiddenSize * sizeof(float));
    file.write((uint8_t *)biasO, outputSize * sizeof(float));

    file.close();
}

void NeuralNetwork::load(const char *filename)
{
    File file = SD.open(filename);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return;
    }

    // Load network architecture
    int loadedInputSize, loadedHiddenSize, loadedOutputSize;
    file.read((uint8_t *)&loadedInputSize, sizeof(loadedInputSize));
    file.read((uint8_t *)&loadedHiddenSize, sizeof(loadedHiddenSize));
    file.read((uint8_t *)&loadedOutputSize, sizeof(loadedOutputSize));

    if (loadedInputSize != inputSize || loadedHiddenSize != hiddenSize ||
        loadedOutputSize != outputSize)
    {
        Serial.println("Network architecture mismatch");
        file.close();
        return;
    }

    // Load weights and biases
    file.read((uint8_t *)weightsIH, inputSize * hiddenSize * sizeof(float));
    file.read((uint8_t *)weightsHO, hiddenSize * outputSize * sizeof(float));
    file.read((uint8_t *)biasH, hiddenSize * sizeof(float));
    file.read((uint8_t *)biasO, outputSize * sizeof(float));

    file.close();
}