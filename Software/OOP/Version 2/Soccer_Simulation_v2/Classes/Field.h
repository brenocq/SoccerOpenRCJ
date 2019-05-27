///////////////////////////////////////////////////////////
//  Field.h
//  Implementation of the Class Field
//  Created on:      21-mar-2018 21:46:57
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef FIELD_H
#define FIELD_H

#include "BitMatrix.h"

class Field
{

public:
	Field();
	virtual ~Field();
	
	//Read
	virtual int pointRead(int x0, int y0, int val)=0;
	int lineRead(int x0, int y0, int x1, int y1, int val);
	int bigLineRead(int x0, int y0, int x1, int y1, int thickness, int val);
	int circleRead(int x0, int y0, int radius, int thickness, int val);
	int fillCircleRead(int x0, int y0, int radius, int val);
	int fillQuadrilateralRead(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3, int val);
	int fillTriangleRead(int x1, int y1, int x2, int y2, int x3, int y3, int val);
	//Write
	virtual void pointWrite(int x0, int y0, int val) = 0;
	void lineWrite(int x0, int y0, int x1, int y1, int val);
	void bigLineWrite(int x0, int y0, int x1, int y1, int thickness, int val);
	void circleWrite(int x0, int y0, int radius, int thickness, int val);
	void fillCircleWrite(int x0, int y0, int radius, int val);
	void arcThreePoints(float x1, float y1, float x2, float y2, float x3, float y3, int val);
	void fillQuadrilateralWrite(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3, int val);
	void fillCenterSquareWrite(int x, int y, int l, int h, int val);
	void fillTriangleWrite(int x1, int y1, int x2, int y2, int x3, int y3, int val);
	

	//functions
	float distTwoPoints(int x0, int  y0, int  x1, int y1);
	float angleTwoPoints(int x0, int  y0, int  x1, int y1);
	void cleanField(int val);
	
	//Get property
	int getWidthField() const;
	int getLengthField() const;

protected:
	const int widthField;
	const int lengthField;
};
#endif // FIELD_H
