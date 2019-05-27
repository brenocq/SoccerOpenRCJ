// Library (FieldMatrix) robô de futebol 2018 CIC Robotics
// 03/02/2018
// Desenvolvedor: Breno Cunha Queiroz

#include "FieldMatrix.h"
#include "Ultrasonic.h"
#include "BitMatrix.h"

#define _USE_MATH_DEFINES
#include <math.h>//Pi,cos,sin,atan

#if defined(ARDUINO) && ARDUINO>=100
#include "Arduino.h"//arduino version >= 1.0.0
#elif defined(ARDUINO) && ARDUINO<100
#include "WProgram.h"//arduino old version
#else

#include <cstdint>//int16_t values
#include <string>//string
#include <iostream>//cout,cin
using std::cout;
using std::cin;
using std::endl;
#endif

#define swap(a, b) { int16_t t = a; a = b; b = t; }

FieldMatrix::FieldMatrix(Ultrasonic &robotUltrasonic)
	:widthField(182),
	lengthField(243),
	robotRadio(11),
	ultrasonic(robotUltrasonic)
{
	fieldMatrix = new BitMatrix(widthField, lengthField);

	for (int i = 0; i < lengthField; i++)
	{
		for (int j = 0; j < widthField; j +=8)
		{
			fieldMatrix->writeByte(j, i, false);
		}
	}
}

FieldMatrix::~FieldMatrix()
{
	delete fieldMatrix;
}

void FieldMatrix::lineWrite(int x0, int y0, int x1, int y1, bool value)
{
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	}
	else {
		ystep = -1;
	}

	for (; x0 <= x1; x0++) {
		if (steep) {
			if (y0 < 0)
				y0 = 0;
			if (x0 < 0)
				x0 = 0;
			if (((y0) >= 0 && (y0) < widthField) && ((x0) >= 0 && (x0) < lengthField))
				fieldMatrix->write(y0, x0, value);
		}
		else {
			if (y0 < 0)
				y0 = 0;
			if (x0 < 0)
				x0 = 0;
			if (((x0) >= 0 && (x0) < widthField) && ((y0) >= 0 && (y0) < lengthField))
				fieldMatrix->write(x0, y0, value);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}

int FieldMatrix::lineRead(int x0, int y0, int x1, int y1, bool value) const
{
	//returns the number of points in the line that have the value
	int pointsSum = 0;

	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	}
	else {
		ystep = -1;
	}

	for (; x0 <= x1; x0++) {
		if (steep) {
			if (y0 < 0)
				y0 = 0;
			if (x0 < 0)
				x0 = 0;
			if (((y0) >= 0 && (y0) < widthField) && ((x0) >= 0 && (x0) < lengthField))
				if (fieldMatrix->read(y0, x0) == value)
					pointsSum++;
		}
		else {
			if (y0 < 0)
				y0 = 0;
			if (x0 < 0)
				x0 = 0;
			if (((x0) >= 0 && (x0) < widthField) && ((y0) >= 0 && (y0) < lengthField))
				if (fieldMatrix->read(x0, y0) == value)
					pointsSum++;
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
	return pointsSum;
}

int FieldMatrix::bigLineRead(int x0, int y0, int x1, int y1, int ratio, int lineDistance, bool value) const
{
	int lineAngle = angleTwoPoints(x0, y0, x1, y1);
	int numberLines = 0;
	int readSum = 0;
	//-----temp variables-----//
	float trigonometricAngle = 0;
	//-----left point 0-----//
	int pointXLeft0 = 0;
	int pointYLeft0 = 0;

	trigonometricAngle = ((lineAngle + 90) + 90)*M_PI / 180;//angle in radians
	pointXLeft0 = x0 + int(ratio*cos(trigonometricAngle));
	pointYLeft0 = y0 + int(ratio*sin(trigonometricAngle));

	//drawFillCircle(pointXLeft0, lengthField - pointYLeft0, 3, 4);
	//-----right point 0-----//
	int pointXRight0 = 0;
	int pointYRight0 = 0;

	trigonometricAngle = ((lineAngle - 90) + 90)*M_PI / 180;//angle in radians
	pointXRight0 = x0 + int(ratio*cos(trigonometricAngle));
	pointYRight0 = y0 + int(ratio*sin(trigonometricAngle));

	//drawFillCircle(pointXRight0, lengthField - pointYRight0, 3, 4);
	//-----point point 1-----//
	int pointXLeft1 = 0;
	int pointYLeft1 = 0;

	trigonometricAngle = ((lineAngle - 90) + 90)*M_PI / 180;//angle in radians
	pointXLeft1 = x1 + int(ratio*cos(trigonometricAngle));
	pointYLeft1 = y1 + int(ratio*sin(trigonometricAngle));

	//drawFillCircle(pointXLeft1, lengthField - pointYLeft1, 3, 4);
	//-----right point goal-----//
	int pointXRight1 = 0;
	int pointYRight1 = 0;

	trigonometricAngle = ((lineAngle + 90) + 90)*M_PI / 180;//angle in radians
	pointXRight1 = x1 + int(ratio*cos(trigonometricAngle));
	pointYRight1 = y1 + int(ratio*sin(trigonometricAngle));

	//drawFillCircle(pointXRight1, lengthField - pointYRight1, 3, 4);
	//-----middle line-----//
	int lineX = x0 - x1;//X=Xend-Xstart
	int lineY = y0 - y1;//Y=Yend-Ystart
	float lineHip = sqrt(pow(lineX, 2) + pow(lineY, 2));//diagonal robot->goal
	float newLineHip = 0;
	float relation = 0;

	//-----read line-----//
	while (lineDistance * numberLines <= lineHip)
	{
		newLineHip = lineDistance * numberLines;
		relation = (newLineHip / lineHip);//relation between diagonal (robot->goal) and (robot->circle)

		int newLineX = relation*lineX;
		int newLineY = relation*lineY;

		int leftPointX = pointXLeft0 - newLineX;
		int leftPointY = (pointYLeft0 - newLineY);
		int rightPointX = pointXRight0 - newLineX;
		int rightPointY = (pointYRight0 - newLineY);
		//drawLine(leftPointX, leftPointY, rightPointX, rightPointY, 5);
		readSum += lineRead(leftPointX, leftPointY, rightPointX, rightPointY, 1);
		numberLines++;
	}
	return readSum;
}

void FieldMatrix::circleWrite(int x0, int y0, int radius, bool value, int thickness = 1)
{
	int x = radius - 1;
	int y = 0;
	int dx = 1;
	int dy = 1;
	int err = dx - (radius << 1);

	while (x >= y)
	{
		if (((x0 + x) >= 0 && (x0 + x) < widthField) && ((y0 + y) >= 0 && (y0 + y) < lengthField))
			fieldMatrix->write(x0 + x, y0 + y, value);

		if (((x0 + y) >= 0 && (x0 + y) < widthField) && ((y0 + x) >= 0 && (y0 + x) < lengthField))
			fieldMatrix->write(x0 + y, y0 + x, value);

		if (((x0 - y) >= 0 && (x0 - y) < widthField) && ((y0 + x) >= 0 && (y0 + x) < lengthField))
			fieldMatrix->write(x0 - y, y0 + x, value);

		if (((x0 - x) >= 0 && (x0 - x) < widthField) && ((y0 + y) >= 0 && (y0 + y) < lengthField))
			fieldMatrix->write(x0 - x, y0 + y, value);

		if (((x0 - x) >= 0 && (x0 - x) < widthField) && ((y0 - y) >= 0 && (y0 - y) < lengthField))
			fieldMatrix->write(x0 - x, y0 - y, value);

		if (((x0 - y) >= 0 && (x0 - y) < widthField) && ((y0 - x) >= 0 && (y0 - x) < lengthField))
			fieldMatrix->write(x0 - y, y0 - x, value);

		if (((x0 + y) >= 0 && (x0 + y) < widthField) && ((y0 - x) >= 0 && (y0 - x) < lengthField))
			fieldMatrix->write(x0 + y, y0 - x, value);

		if (((x0 + x) >= 0 && (x0 + x) < widthField) && ((y0 - y) >= 0 && (y0 - y) < lengthField))
			fieldMatrix->write(x0 + x, y0 - y, value);

		if (err <= 0)
		{
			y++;
			err += dy;
			dy += 2;
		}
		if (err > 0)
		{
			x--;
			dx += 2;
			err += dx - (radius << 1);
		}
	}
	if (thickness > 1)
		circleWrite(x0, y0, radius - 1, value, thickness - 1);
}

int FieldMatrix::circleRead(int x0, int y0, int radius, bool value, int thickness = 1) const
{
	//returns the number of points in the circle that have the value
	int x = radius - 1;
	int y = 0;
	int dx = 1;
	int dy = 1;
	int err = dx - (radius << 1);

	int pointsSum = 0;

	while (x >= y)
	{
		if (((x0 + x) >= 0 && (x0 + x) < widthField) && ((y0 + y) >= 0 && (y0 + y) < lengthField))
			if (fieldMatrix->read(x0 + x, y0 + y) == value)
				pointsSum++;

		if (((x0 + y) >= 0 && (x0 + y) < widthField) && ((y0 + x) >= 0 && (y0 + x) < lengthField))
			if (fieldMatrix->read(x0 + y, y0 + x) == value)
				pointsSum++;

		if (((x0 - y) >= 0 && (x0 - y) < widthField) && ((y0 + x) >= 0 && (y0 + x) < lengthField))
			if (fieldMatrix->read(x0 - y, y0 + x) == value)
				pointsSum++;

		if (((x0 - x) >= 0 && (x0 - x) < widthField) && ((y0 + y) >= 0 && (y0 + y) < lengthField))
			if (fieldMatrix->read(x0 - x, y0 + y) == value)
				pointsSum++;

		if (((x0 - x) >= 0 && (x0 - x) < widthField) && ((y0 - y) >= 0 && (y0 - y) < lengthField))
			if (fieldMatrix->read(x0 - x, y0 - y) == value)
				pointsSum++;

		if (((x0 - y) >= 0 && (x0 - y) < widthField) && ((y0 - x) >= 0 && (y0 - x) < lengthField))
			if (fieldMatrix->read(x0 - y, y0 - x) == value)
				pointsSum++;

		if (((x0 + y) >= 0 && (x0 + y) < widthField) && ((y0 - x) >= 0 && (y0 - x) < lengthField))
			if (fieldMatrix->read(x0 + y, y0 - x) == value)
				pointsSum++;

		if (((x0 + x) >= 0 && (x0 + x) < widthField) && ((y0 - y) >= 0 && (y0 - y) < lengthField))
			if (fieldMatrix->read(x0 + x, y0 - y) == value)
				pointsSum++;

		if (err <= 0)
		{
			y++;
			err += dy;
			dy += 2;
		}
		if (err > 0)
		{
			x--;
			dx += 2;
			err += dx - (radius << 1);
		}
	}
	if (thickness > 1)
		pointsSum += circleRead(x0, y0, radius - 1, value, thickness - 1);
	return pointsSum;
}

void  FieldMatrix::fillCircleWrite(int x0, int y0, int radius, bool value)
{

	lineWrite((x0), (y0 - radius), (x0), (y0 - radius) + (2 * radius + 1) - 1, value);


	int16_t f = 1 - radius;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * radius;
	int16_t x = 0;
	int16_t y = radius;

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		if (3 & 0x1) {
			lineWrite((x0 + x), (y0 - y), (x0 + x), (y0 - y) + (2 * y + 1 + 0) - 1, value);
			lineWrite((x0 + y), (y0 - x), (x0 + y), (y0 - x) + (2 * x + 1 + 0) - 1, value);
		}
		if (3 & 0x2) {
			lineWrite((x0 - x), (y0 - y), (x0 - x), (y0 - y) + (2 * y + 1 + 0) - 1, value);
			lineWrite((x0 - y), (y0 - x), (x0 - y), (y0 - x) + (2 * x + 1 + 0) - 1, value);
		}
	}
}

int FieldMatrix::fillCircleRead(int x0, int y0, int radius, bool value) const
{
	//returns the number of points in the fill circle that have the value
	int pointsSum = 0;

	pointsSum += lineRead((x0), (y0 - radius), (x0), (y0 - radius) + (2 * radius + 1) - 1, value);


	int16_t f = 1 - radius;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * radius;
	int16_t x = 0;
	int16_t y = radius;

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		if (3 & 0x1) {
			//drawFastVLine(x0 + x, y0 - y, 2 * y + 1 + 0, color);
			pointsSum += lineRead((x0 + x), (y0 - y), (x0 + x), (y0 - y) + (2 * y + 1 + 0) - 1, value);
			//drawFastVLine(x0 + y, y0 - x, 2 * x + 1 + 0, color);
			pointsSum += lineRead((x0 + y), (y0 - x), (x0 + y), (y0 - x) + (2 * x + 1 + 0) - 1, value);
		}
		if (3 & 0x2) {
			//drawFastVLine(x0 - x, y0 - y, 2 * y + 1 + 0, color);
			pointsSum += lineRead((x0 - x), (y0 - y), (x0 - x), (y0 - y) + (2 * y + 1 + 0) - 1, value);
			//drawFastVLine(x0 - y, y0 - x, 2 * x + 1 + 0, color);
			pointsSum += lineRead((x0 - y), (y0 - x), (x0 - y), (y0 - x) + (2 * x + 1 + 0) - 1, value);
		}
	}
	return pointsSum;
}

int FieldMatrix::distTwoPoints(int x0, int y0, int x1, int y1) const
{
	int distanceX = (x0 - x1);
	int distanceY = (y0 - y1);
	int hip = pow(distanceX, 2) + pow(distanceY, 2);

	return sqrt(hip);
}

int  FieldMatrix::angleTwoPoints(double x0, double  y0, double  x1, double y1) const
{
	int angleValue = 360 - (int(atan2(x0 - x1, y0 - y1) * 180 / M_PI)) + 180;

	while (angleValue > 360)
		angleValue -= 360;
	while (angleValue < 0)
		angleValue += 360;
	return angleValue;
}

void FieldMatrix::delAlonePoints()
{
	int pointsSum = 0;
	for (int i = 0; i < lengthField; i += 10)
	{
		for (int j = 0; j < widthField; j += 10)
		{
			pointsSum += fillCircleRead(j, i, 7, 1);
			if (pointsSum>0 && pointsSum <= 2)
			{
				fillCircleWrite(j, i, 7, false);
			}
			pointsSum = 0;
		}
	}
}

void FieldMatrix::drawSonarPoints()
{
	robotX=ultrasonic.getRobotX();
	robotY=ultrasonic.getRobotY();

	cleanField();

	int XPoint;
	int YPoint;

	ultrasonic.updatePointX();
	ultrasonic.updatePointY();

	if (ultrasonic.getIsXknowed() == true && ultrasonic.getIsYknowed() == true)
	{
		for (int i = 0; i < 12; i++)
		{
			XPoint = robotX + ultrasonic.getPointX(i);
			YPoint = robotY + ultrasonic.getPointY(i);
			if (XPoint < 0)
				XPoint = 0;
			if (XPoint >= widthField)
				XPoint = widthField - 1;
			if (YPoint < 0)
				YPoint = 0;
			if (YPoint >= lengthField)
				YPoint = lengthField - 1;

			fieldMatrix->write(XPoint, YPoint, true);
		}
	}
}

void FieldMatrix::cleanField()
{

	if (ultrasonic.getIsXknowed() == true && ultrasonic.getIsYknowed() == true)
	{
		for (int i = 0; i < 12; i++)
		{
			int lineX = (robotX)-(robotX + ultrasonic.getPointX(i));//X=Xend-Xstart
			int lineY = (robotY)-(robotY + ultrasonic.getPointY(i));//Y=Yend-Ystart
			float lineHip = sqrt(pow(lineX, 2) + pow(lineY, 2));

			float newLineHip = lineHip - 5;

			float relation = (newLineHip / lineHip);

			int newLineX = relation*lineX;
			int newLineY = relation*lineY;

			cleaningLine(robotX, robotY, int(robotX - newLineX), int(robotY - newLineY));

		}
	}
}

void FieldMatrix::cleaningLine(int x0, int y0, int x1, int y1)
{
	int numberSquares = 1;

	int lineX = x0 - x1;//X=Xend-Xstart
	int lineY = y0 - y1;//Y=Yend-Ystart
	float lineHip = sqrt(pow(lineX, 2) + pow(lineY, 2));
	float newLineHip = 0;
	float relation = 0;

	int contWhile = 0;
	while (((5 + 2 * numberSquares <= lineHip) && (lineHip > 20)) && (contWhile < 1000))
	{
		newLineHip = 5 + 2 * numberSquares;
		relation = (newLineHip / lineHip);

		int newLineX = relation*lineX;
		int newLineY = relation*lineY;

		fillCircleWrite(int(robotX - newLineX), int(robotY - newLineY), 3, false);

		contWhile++;
		numberSquares++;
	}
}

bool FieldMatrix::getField(int x, int y)
{
	return fieldMatrix->read(x, y);
}

bool FieldMatrix::getFieldByte(int x, int y)
{
	return fieldMatrix->readByte(x, y);
}