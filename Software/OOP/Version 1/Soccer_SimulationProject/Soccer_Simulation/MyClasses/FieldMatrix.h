// Library (FieldMatrix) robô de futebol 2018 CIC Robotics
// 16/01/2018
// Desenvolvedor: Breno Cunha Queiroz
#ifndef FIELDMATRIX_H
#define FIELDMATRIX_H

class Ultrasonic;
class BitMatrix;

class FieldMatrix
{
public:
	FieldMatrix(Ultrasonic &);
	~FieldMatrix();

	void lineWrite(int x0, int y0, int x1, int y1, bool value);
	int lineRead(int x0, int y0, int x1, int y1, bool value) const;

	int bigLineRead(int x0, int y0, int x1, int y1, int ratio, int lineDistance, bool value) const;

	void circleWrite(int x0, int y0, int radius, bool value, int thickness);
	int circleRead(int x0, int y0, int radius, bool value, int thickness) const;

	void fillCircleWrite(int x0, int y0, int radius, bool value);
	int fillCircleRead(int x0, int y0, int radius, bool value) const;

	int distTwoPoints(int x0, int y0, int x1, int y1) const;
	int angleTwoPoints(double x0, double  y0, double  x1, double y1) const;

	bool getField(int x, int y);
	bool getFieldByte(int x, int y);
	//-----ACTIONS-----//
	void delAlonePoints();//delete alone points in the field
	void drawSonarPoints();//draw detected points by the sonars

	void cleanField();
	void cleaningLine(int x0, int y0, int x1, int y1);

	BitMatrix *fieldMatrix;
private:
	const int widthField;
	const int lengthField;
	const int robotRadio;

	int robotX;
	int robotY;

	//Objects
	Ultrasonic &ultrasonic;
	
};

#endif // FIELDMATRIX_H