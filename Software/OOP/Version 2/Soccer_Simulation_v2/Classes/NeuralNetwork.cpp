///////////////////////////////////////////////////////////
//  NeuralNetwork.cpp
//  Implementation of the Class Robot
//  Created on:      09-jun-2018 09:39:31
//  Original author: Breno Queiroz (based in David Miller example)
///////////////////////////////////////////////////////////

#include "NeuralNetwork.h"

//------------ NeuralNetwork ------------//
NeuralNetwork::NeuralNetwork(const vector<unsigned int> &topology){
	unsigned int numLayers = topology.size();// number of layers

	for (unsigned int layerNum = 0; layerNum < numLayers; layerNum++){// run each layer
		n_layers.push_back(Layer());// create this layer
		unsigned int numOutputs = layerNum == topology.size() - 1 ? 0 : topology[layerNum + 1];//calculate the number of output for each neuron


		for (unsigned int neuronNum = 0; neuronNum <= topology[layerNum]; neuronNum++){// for each layer, create the neurons + create one neuron bias(<= signal)
			n_layers.back().push_back(Neuron(numOutputs, neuronNum));// create the neurons in the last layer created
		}
		// force the bias node's output value to 1.0. It's the last neuron created above
		n_layers.back().back().setOutputVal(1.0);
	}
}

void NeuralNetwork::debug(const vector<float> &inputVals,const vector<float>&targetVals){
	vector<float> result;
	getResults(result);

	cout << "\nPass " << numIterations;
	cout << "\nInput: ";
	for (unsigned int i = 0; i < inputVals.size(); i++){
		cout << inputVals[i]<<" ";
	}
	cout << "\nTarget: ";
	for (unsigned int i = 0; i < targetVals.size(); i++){
		cout << targetVals[i] << " ";
	}
	cout << "\nOutputs: ";
	for (unsigned int i = 0; i < result.size(); i++){
		cout << result[i] << " ";
	}
	cout << "\nNet average error: " << recentAvarageError<<endl;
}

void NeuralNetwork::getResults(vector<float> &resultVals) const{
	resultVals.clear();

	for (unsigned int n = 0; n < n_layers.back().size() - 1; n++){
		resultVals.push_back(n_layers.back()[n].getOutputVal());
	}
}

void NeuralNetwork::feedForward(const vector<float> &inputVals){
	// return a error if the number of inputs and neurons are different
	assert(inputVals.size() == n_layers[0].size() - 1 && " the inputValues size needs to be the same of the first layer - bias");
	for (unsigned int i = 0; i < inputVals.size(); i++){// run each input/neuron
		n_layers[0][i].setOutputVal(inputVals[i]);// put the inputVals on each input neuron (layer 0)
	}
	// foward propagation
	for (unsigned int layerNum = 1; layerNum < n_layers.size(); layerNum++){// run each layer
		Layer &prevLayer = n_layers[layerNum - 1];// create a reference for the previous layer
		for (unsigned int n = 0; n < n_layers[layerNum].size()-1; n++){// run each neuron in the layer
			n_layers[layerNum][n].feedForward(prevLayer);// run feedFoward of each neuron
		}
	}
}

void NeuralNetwork::backPropagation(const vector<float> &targetVals){
	// calculate overall net error (RMS of output neuron errors)
	Layer &outputLayer = n_layers.back();// reference for the output layer
	error = 0.0;
	for (unsigned int n = 0; n < outputLayer.size()-1; n++){// run each neuron in the output layer
		double delta = targetVals[n] - outputLayer[n].getOutputVal();
		error += delta*delta;// sum error squared
	}
	//error *= 0.5;
	error /= outputLayer.size() - 1;// get average error squared
	error = sqrt(error);// RMS

	//implement a recent avarage measurement
	recentAvarageError = (recentAvarageError*recentAvarageSmoothingFactor + error) /
		(recentAvarageSmoothingFactor + 1.0);
	//recentAvarageError = error;

	// calculate output layer gradients
	for (unsigned int n = 0; n < outputLayer.size()-1; n++){
		outputLayer[n].calcOutputGradients(targetVals[n]);
	}

	// calculate gradients on hidden layers
	for (unsigned int layerNum = n_layers.size()-2; layerNum > 0; layerNum--){
		Layer &hiddenLayer = n_layers[layerNum];// reference to this hidden layer
		Layer &nextLayer= n_layers[layerNum+1];// reference to next layer

		for (unsigned int n = 0; n < hiddenLayer.size(); n++){// run each neuron in the hidden layer
			hiddenLayer[n].calcHiddenGradients(nextLayer);
		}
	}

	// for all layers from outputs to first hidden layer, update connection weights
	for (unsigned int layerNum = n_layers.size()-1; layerNum > 0; layerNum--){// run each layer from the last
		Layer &layer = n_layers[layerNum];//actual layer by reference
		Layer &prevLayer = n_layers[layerNum - 1];//previsous layer by reference
		for (unsigned int n = 0; n < layer.size() - 1; n++){// run each neuron in the layer
			layer[n].updateInputWeights(prevLayer);// update the weight of the previsous layer
		}
	}
	numIterations++;//update the number of learning itineations (for debug)
}

void NeuralNetwork::saveData(){
	ofstream myfile;
	int cont = 0;
	myfile.open("NeuralData.txt");
	for (unsigned int layerNum = 0; layerNum < n_layers.size()-1; layerNum++){// run each layer
		for (unsigned int n = 0; n < n_layers[layerNum].size() - 1; n++){// run each neuron in the layer
			Layer &nextLayer = n_layers[layerNum+1];
			vector<float>weightsNeuron;
			n_layers[layerNum][n].getWeights(weightsNeuron, nextLayer);//put all connection weights of this neuron in this vector
			for (int i = 0; i < nextLayer.size() - 1; i++)
			{
				myfile << weightsNeuron[i]<<" ";
				cont++;
			}
		}
	}
	//myfile << " " << cont << " " << " humans will die\n";
	myfile.close();
}

void NeuralNetwork::runData(){
	//ofstream myfile;
	//myfile.open("NeuralData.txt");
	float readedWeight = 0;
	ifstream myfile;
	 myfile.open("NeuralData.txt");
	 if (myfile.is_open()) {
			for (unsigned int layerNum = 0; layerNum < n_layers.size() - 1; layerNum++){// run each layer
				for (unsigned int n = 0; n < n_layers[layerNum].size() - 1; n++){// run each neuron in the layer
					Layer &nextLayer = n_layers[layerNum + 1];
					//vector<float>weightsNeuron;
					//n_layers[layerNum][n].getWeights(weightsNeuron, nextLayer);//put all connection weights of this neuron in this vector
					for (int i = 0; i < nextLayer.size() - 1; i++)
					{
						myfile >> readedWeight;
						n_layers[layerNum][n].setWeights(readedWeight, i);
					}
				}
			}
		myfile.close();
	}
}

//------------ Neuron ------------//
Neuron::Neuron(unsigned int numOutputs,unsigned int _myIndex)
	:myIndex(_myIndex), outputVal(0)
{
	for (unsigned int c = 0; c < numOutputs; c++){
		outputWeights.push_back(Connection());// create one connection for each output (weight+deltaWeight)
		outputWeights.back().weight = randomWeight();// random weight
	}
}

float Neuron::eta = 0.15;// overall net learning rate [0.0..1.0]
float Neuron::alpha = 0.5;// momentum, multiplier of last deltaWeight [0.0..n]

void Neuron::updateInputWeights(Layer &prevLayer){
	// the weights to be updated are in the connection container
	// in the neurons in the preceding layer

	//eta: 0 = slow learner // 0.2 = medium learner // 1 = reckless learner
	//alpha: 0 = no momentum // 0.5 = moderate momentum

	for (unsigned int n = 0; n < prevLayer.size(); n++){// run each neuron in the previous layer
		Neuron &neuron = prevLayer[n];
		float oldDeltaWeight = neuron.outputWeights[myIndex].deltaWeight;

		float newDeltaWeight =
			// individual input, magnified by the gradient and train rate:
			eta* neuron.getOutputVal()*gradient
			//also add momentum = a fraction of the previous delta weight
			+ alpha * oldDeltaWeight;

		neuron.outputWeights[myIndex].deltaWeight = newDeltaWeight;
		neuron.outputWeights[myIndex].weight += newDeltaWeight;
	}
}

void  Neuron::getWeights(vector<float> &_weights, const Layer &nextLayer){
	_weights.clear();
	for (int i = 0; i < nextLayer.size() - 1; i++){
		_weights.push_back(outputWeights[i].weight);
	}
}
void Neuron::setWeights(float _weights, int num){
	outputWeights[num].weight = _weights;
}

float Neuron::sumDOW(const Layer &nextLayer) const{
	float sum = 0.0;

	// sum our contributions of the errors at the nodes feeded

	for (unsigned int n = 0; n < nextLayer.size()-1; n++){
		sum += outputWeights[n].weight * nextLayer[n].gradient;
	}
	return sum;
}

void Neuron::calcHiddenGradients(const Layer &nextLayer){
	float dow = sumDOW(nextLayer);// simular to "calcOutputGradients" but have no target (need to calculate)
	gradient = dow*Neuron::transferFunctionDerivative(outputVal);
}

void Neuron::calcOutputGradients(float targetVal){
	float delta = targetVal - outputVal;// error to the target difference
	gradient = delta*Neuron::transferFunctionDerivative(outputVal);
}

void Neuron::feedForward(const Layer &prevLayer){
	float sum = 0.0;

	//sum the previous layer's outputs (which are the inputs)
	//include the bias node from the previous layer
	for (unsigned int n = 0; n < prevLayer.size(); n++){// run each neuron in the previous layer
		sum += prevLayer[n].getOutputVal() *
			prevLayer[n].outputWeights[myIndex].weight;//sum= SUM(Output*Weight)
	}
	outputVal = Neuron::transferFunction(sum);//pass the sum into a function ==> output= f(SUM(Output*Weight))
}

float Neuron::transferFunction(float x){
	//used to calculate the output value
	//can use tanh or sigmoid (I'm using tanh)
	return tanh(x);
}

float Neuron::transferFunctionDerivative(float x){
	//tanh derivative
	return 1.0 - tanh(x) * tanh(x);
}

float Neuron::randomWeight(){
	return (2*rand() / float(RAND_MAX))-1;
}
