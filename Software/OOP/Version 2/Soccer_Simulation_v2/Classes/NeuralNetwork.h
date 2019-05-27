///////////////////////////////////////////////////////////
//  NeuralNetwork.h
//  Implementation of the Class Robot
//  Created on:      09-jun-2018 09:28:38
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

#include <fstream>
#include <vector>
#include <iostream>
#include <cstdlib>//rand
#include <cassert>//assert
#include <cmath>//tanh

using namespace std;

struct Connection{
	float weight;
	float deltaWeight;
};
class Neuron;
typedef vector<Neuron> Layer;

//------------ NeuralNetwork ------------//
class NeuralNetwork{
public:
	NeuralNetwork(const vector<unsigned int> &topology);// EX: {5,11,11,5} neurons per layer

	void feedForward(const vector<float> &inputVals);
	void backPropagation(const vector<float> &targetVals);
	void getResults(vector<float> &resultVals) const;
	void debug(const vector<float> &inputVals,const vector<float> &targetVals);
	double getError() const { return recentAvarageError; }

	void saveData();//Creaate this class
	void runData();//Creaate this class

private:
	vector<Layer> n_layers;// all layers together ==> n_layers[layerNum][neuronNum]
	unsigned long numIterations;// times that this neuralNetwork was run until now
	double error;
	double recentAvarageError;
	double recentAvarageSmoothingFactor;
};

//------------ Neuron ------------//
class Neuron{
public:
	Neuron(unsigned int numOutputs, unsigned int _myIndex);
	void setOutputVal(float val) { outputVal = val; }
	float getOutputVal()const { return outputVal; }
	void getWeights(vector<float> &_weights,const Layer &nextLayer);
	void setWeights(float _weights,int num);
	void feedForward(const Layer &prevLayer);
	void calcOutputGradients(float targetVal);
	void calcHiddenGradients(const Layer &nextLayer);
	void updateInputWeights(Layer &prevLayer);


private:
	static float eta;// [0.0..1.0] overall net training rate
	static float alpha;// [0.0..n] multiplier of last weight change (momentum)
	static float transferFunction(float x);
	static float transferFunctionDerivative(float x);

	static float randomWeight();
	float sumDOW(const Layer &nextLayer)const;

	float outputVal;
	float gradient;
	vector<Connection> outputWeights;// each output have a weight
	unsigned int myIndex;
};

#endif // NEURALNETWORK_H