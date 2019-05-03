///////////////////////////////////////////////////////////
//  FieldMatrix.cpp
//  Implementation of the Class FieldMatrix
//  Created on:      21-mar-2018 21:46:57
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#include "FieldMatrix.h"


FieldMatrix::FieldMatrix(){
	fieldMatrix = new BitMatrix(widthField, lengthField);

	cleanField(false);
}

FieldMatrix::~FieldMatrix(){
	delete fieldMatrix;
}


//Read
int FieldMatrix::pointRead(int x0, int y0, int val)
{
	bool value = val;
	if (fieldMatrix->read(x0, y0) == true)
		return 1;
	else
		return 0;
}

//Write
void FieldMatrix::pointWrite(int x0, int y0, int val)
{
	fieldMatrix->write(x0, y0, val);
}

void FieldMatrix::cleanField(int val)
{
	for (int i = 0; i < lengthField; i++)
	{
		for (int j = 0; j < widthField; j += 8)
		{
			fieldMatrix->writeByte(j, i, val);
		}
	}
}