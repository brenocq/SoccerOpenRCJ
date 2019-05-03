///////////////////////////////////////////////////////////
//  InternalCommunication.h
//  Implementation of the Class InternalCommunication
//  Created on:      21-mar-2018 21:46:57
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef INTERNALCOMMUNICATION_H
#define INTERNALCOMMUNICATION_H

#include "Robot.h"
#include "Communication.h"

class InternalCommunication : public Communication
{

public:
	virtual ~InternalCommunication();
	Robot *m_Robot;

	InternalCommunication();

};
#endif // INTERNALCOMMUNICATION_H
