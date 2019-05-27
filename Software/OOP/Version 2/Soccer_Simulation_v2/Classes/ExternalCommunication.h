///////////////////////////////////////////////////////////
//  ExternalCommunication.h
//  Implementation of the Class ExternalCommunication
//  Created on:      21-mar-2018 21:46:56
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef EXTERNALCOMMUNICATION_H
#define EXTERNALCOMMUNICATION_H

#include "Communication.h"
#include "Robot.h"

class ExternalCommunication : public Communication
{
public:
	ExternalCommunication(int _robotN1,int _robotN2);
	virtual ~ExternalCommunication();

	void virtual write(const char byte, int robotN);
	char virtual read(int robotN) const;
	int available(int robotN)const;
private:
	const int robotN1;//robot number 1
	const int robotN2;//robot number 2
};
#endif // EXTERNALCOMMUNICATION_H
