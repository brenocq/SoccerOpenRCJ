///////////////////////////////////////////////////////////
//  ExternalCommunication.cpp
//  Implementation of the Class ExternalCommunication
//  Created on:      21-mar-2018 21:46:57
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#include "ExternalCommunication.h"


ExternalCommunication::ExternalCommunication(int _robotN1, int _robotN2)
	:robotN1(_robotN1), robotN2(_robotN2){
}

ExternalCommunication::~ExternalCommunication(){}

void  ExternalCommunication::write(const char byte, int robotN){
	if (robotN == robotN1)
		Communication::write(byte, 2);
	else
		Communication::write(byte, 1); 
}
char  ExternalCommunication::read(int robotN) const{
	if (robotN == robotN1)
		return Communication::read(1);
	else
		return Communication::read(2);
	return 0;
}
int ExternalCommunication::available(int robotN)const{
	if (robotN == robotN1)
		return Communication::available(1);
	else
		return Communication::available(2);
}