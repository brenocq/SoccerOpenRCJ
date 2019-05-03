///////////////////////////////////////////////////////////
//  FieldMatrix.h
//  Implementation of the Class FieldMatrix
//  Created on:      21-mar-2018 21:46:57
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef FIELDMATRIX_H
#define FIELDMATRIX_H

#include "Field.h"
#include "BitMatrix.h"

class FieldMatrix : public Field
{

public:
	FieldMatrix();
	virtual ~FieldMatrix();
	
	//Read
	virtual int pointRead(int x0, int y0, int val);
	//Write
	virtual void pointWrite(int x0, int y0, int val);

	void cleanField(int val);

	void getField();
	void getFieldByte();
	
private:
	BitMatrix *fieldMatrix;
};
#endif // FIELDMATRIX_H
