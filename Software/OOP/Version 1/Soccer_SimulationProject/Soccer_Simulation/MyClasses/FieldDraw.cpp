// Library (FieldDraw) robô de futebol 2018 CIC Robotics
// 03/02/2018
// Desenvolvedor: Breno Cunha Queiroz

#include "FieldDraw.h"
#include "DataBase.h"
#include "Compass.h"
#include "Ultrasonic.h"
#include "FieldMatrix.h"

#define _USE_MATH_DEFINES
#include <math.h>//Pi,cos,sin,atan

#include <cstdint>//int16_t values
#include <algorithm>//swap function
using std::swap;
#include <string>//string
#include <iostream>//cout,cin
using std::cout;
using std::cin;
using std::endl;
#include <windows.h>//to see pixels
#include <cstdlib>//to clear the screen

FieldDraw::FieldDraw(DataBase &robotDataBase, Compass &robotCompass, Ultrasonic &robotUltrasonic, FieldMatrix &ReadingMatrix)
	:widthField(182),
	lengthField(243),
	robotRadio(11),
	seeSonarLines(false),
	seeRobot(false),
	dataBase(robotDataBase),
	compass(robotCompass),
	ultrasonic(robotUltrasonic),
	reading(ReadingMatrix)
{
	fieldMatrix = new Color*[widthField];
	for (int i = 0; i < widthField; ++i)
		fieldMatrix[i] = new Color[lengthField];

	for (int i = 0; i < lengthField; i++)
	{
		for (int j = 0; j < widthField; j++)
		{
			fieldMatrix[j][i] = WHITE;
		}
	}
}

FieldDraw::~FieldDraw()
{
	for (int i = 0; i < widthField; ++i) {
		delete[] fieldMatrix[i];
	}
	delete[] fieldMatrix;
}

void FieldDraw::line(int x0, int y0, int x1, int y1, Color color)
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
				fieldMatrix[y0][x0] = color;
		}
		else {
			if (y0 < 0)
				y0 = 0;
			if (x0 < 0)
				x0 = 0;
			if (((x0) >= 0 && (x0) < widthField) && ((y0) >= 0 && (y0) < lengthField))
				fieldMatrix[x0][y0] = color;
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}

void FieldDraw::bigLine(int x0, int y0, int x1, int y1, int ratio, int lineDistance, Color color)
{
	int lineAngle = angleTwoPoints(x0, y0, x1, y1);
	int numberLines = 0;
	//-----temp variables-----//
	float trigonometricAngle = 0;
	//-----left point 0-----//
	int pointXLeft0 = 0;
	int pointYLeft0 = 0;

	trigonometricAngle = ((lineAngle + 90) + 90)*M_PI / 180;//angle in radians
	pointXLeft0 = x0 + int(ratio*cos(trigonometricAngle));
	pointYLeft0 = y0 + int(ratio*sin(trigonometricAngle));

	//-----right point 0-----//
	int pointXRight0 = 0;
	int pointYRight0 = 0;

	trigonometricAngle = ((lineAngle - 90) + 90)*M_PI / 180;//angle in radians
	pointXRight0 = x0 + int(ratio*cos(trigonometricAngle));
	pointYRight0 = y0 + int(ratio*sin(trigonometricAngle));

	//-----point point 1-----//
	int pointXLeft1 = 0;
	int pointYLeft1 = 0;

	trigonometricAngle = ((lineAngle - 90) + 90)*M_PI / 180;//angle in radians
	pointXLeft1 = x1 + int(ratio*cos(trigonometricAngle));
	pointYLeft1 = y1 + int(ratio*sin(trigonometricAngle));

	//-----right point goal-----//
	int pointXRight1 = 0;
	int pointYRight1 = 0;

	trigonometricAngle = ((lineAngle + 90) + 90)*M_PI / 180;//angle in radians
	pointXRight1 = x1 + int(ratio*cos(trigonometricAngle));
	pointYRight1 = y1 + int(ratio*sin(trigonometricAngle));

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
		line(leftPointX, leftPointY, rightPointX, rightPointY,color);
		numberLines++;
	}
}

void FieldDraw::circle(int x0, int y0, int radius, Color color, int thickness = 1)
{
	int x = radius - 1;
	int y = 0;
	int dx = 1;
	int dy = 1;
	int err = dx - (radius << 1);

	while (x >= y)
	{
		if (((x0 + x) >= 0 && (x0 + x) < widthField) && ((y0 + y) >= 0 && (y0 + y) < lengthField))
			fieldMatrix[x0 + x][y0 + y] = color;

		if (((x0 + y) >= 0 && (x0 + y) < widthField) && ((y0 + x) >= 0 && (y0 + x) < lengthField))
			fieldMatrix[x0 + y][y0 + x] = color;

		if (((x0 - y) >= 0 && (x0 - y) < widthField) && ((y0 + x) >= 0 && (y0 + x) < lengthField))
			fieldMatrix[x0 - y][y0 + x] = color;

		if (((x0 - x) >= 0 && (x0 - x) < widthField) && ((y0 + y) >= 0 && (y0 + y) < lengthField))
			fieldMatrix[x0 - x][y0 + y] = color;

		if (((x0 - x) >= 0 && (x0 - x) < widthField) && ((y0 - y) >= 0 && (y0 - y) < lengthField))
			fieldMatrix[x0 - x][y0 - y] = color;

		if (((x0 - y) >= 0 && (x0 - y) < widthField) && ((y0 - x) >= 0 && (y0 - x) < lengthField))
			fieldMatrix[x0 - y][y0 - x] = color;

		if (((x0 + y) >= 0 && (x0 + y) < widthField) && ((y0 - x) >= 0 && (y0 - x) < lengthField))
			fieldMatrix[x0 + y][y0 - x] = color;

		if (((x0 + x) >= 0 && (x0 + x) < widthField) && ((y0 - y) >= 0 && (y0 - y) < lengthField))
			fieldMatrix[x0 + x][y0 - y] = color;

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
		circle(x0, y0, radius - 1, color, thickness - 1);
}

void FieldDraw::fillCircle(int x0, int y0, int radius, Color color)
{
	line((x0), (y0 - radius), (x0), (y0 - radius) + (2 * radius + 1) - 1, color);


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
			line((x0 + x), (y0 - y), (x0 + x), (y0 - y) + (2 * y + 1 + 0) - 1, color);
			line((x0 + y), (y0 - x), (x0 + y), (y0 - x) + (2 * x + 1 + 0) - 1, color);
		}
		if (3 & 0x2) {
			line((x0 - x), (y0 - y), (x0 - x), (y0 - y) + (2 * y + 1 + 0) - 1, color);
			line((x0 - y), (y0 - x), (x0 - y), (y0 - x) + (2 * x + 1 + 0) - 1, color);
		}
	}
}

void FieldDraw::seeField()
{
	//Console
	HWND myconsole = GetConsoleWindow();
	HDC mydc = GetDC(myconsole);

	COLORREF white = RGB(255, 255, 255);
	COLORREF green = RGB(0, 255, 0);
	COLORREF red = RGB(255, 0, 0);
	COLORREF blue = RGB(0, 0, 255);
	COLORREF magenta = RGB(255, 0, 255);
	COLORREF black = RGB(0, 0, 0);
	COLORREF cyan = RGB(0, 255, 255);
	COLORREF yellow = RGB(255, 255, 0);

	for (int i = 0; i < lengthField; i++)//Y
	{
		for (int j = 0; j < widthField; j+=8)//X
		{
			if (reading.getFieldByte(j, i) == true)
			{
				for (int k = j; k < j + 8; k++)
				{
					switch (reading.getField(k, i))
					{
					case false:
						break;
					case true:
						SetPixel(mydc, k, lengthField - i, red);
						break;
					default:
						break;
					}
				}
			}
		}
	}

	if (seeSonarLines == true)
		sonarLines();

	int robotX = ultrasonic.getRobotX();
	int robotY = ultrasonic.getRobotY();

	if ((seeRobot == true) && (dataBase.getIsRobotXKnowed()) && (dataBase.getIsRobotYKnowed()))
		fillCircle(robotX, robotY, robotRadio, BLACK);

	if ((seeBall == true) && (dataBase.getIsBallKnowed()))
		fillCircle(dataBase.getBallX(), dataBase.getBallY(), dataBase.getBallDiameter(), MAGENTA);

	for (int i = 0; i < lengthField; i++)//Y
	{
		for (int j = 0; j < widthField; j++)//X
		{
			switch (fieldMatrix[j][i])
			{
			case EMPTY:
				break;
			case WHITE:
				SetPixel(mydc, j, lengthField - i, white);
				fieldMatrix[j][i] = EMPTY;
				break;
			case GREEN:
				SetPixel(mydc, j, lengthField - i, green);
				fieldMatrix[j][i] = WHITE;
				break;
			case RED:
				SetPixel(mydc, j, lengthField - i, red);
				fieldMatrix[j][i] = WHITE;
				break;
			case BLUE:
				SetPixel(mydc, j, lengthField - i, blue);
				fieldMatrix[j][i] = WHITE;
				break;
			case MAGENTA:
				SetPixel(mydc, j, lengthField - i, magenta);
				fieldMatrix[j][i] = WHITE;
				break;
			case BLACK:
				SetPixel(mydc, j, lengthField - i, black);
				fieldMatrix[j][i] = WHITE;
				break;
			case CYAN:
				SetPixel(mydc, j, lengthField - i, cyan);
				fieldMatrix[j][i] = WHITE;
				break;
			case YELLOW:
				SetPixel(mydc, j, lengthField - i, yellow);
				fieldMatrix[j][i] = WHITE;
				break;
			}
		}
	}
}

void FieldDraw::setSeeSonarLines(bool b)
{
	seeSonarLines = b;
}

void FieldDraw::setSeeRobot(bool b)
{
	seeRobot = b;
}

void FieldDraw::setSeeBall(bool b)
{
	seeBall = b;
}

void FieldDraw::sonarLines()
{
	int XPoint;
	int YPoint;
	ultrasonic.updatePointX();
	ultrasonic.updatePointY();
	int robotX = ultrasonic.getRobotX();
	int robotY = ultrasonic.getRobotY();
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
		//fieldMatrix[XPoint][YPoint] = 1;
		fillCircle(XPoint, YPoint, 2, GREEN);

		line(robotX, robotY, XPoint, YPoint, GREEN);
	}
}

int  FieldDraw::angleTwoPoints(double x0, double  y0, double  x1, double y1) const
{
	int angleValue = 360 - (int(atan2(x0 - x1, y0 - y1) * 180 / M_PI)) + 180;

	while (angleValue > 360)
		angleValue -= 360;
	while (angleValue < 0)
		angleValue += 360;
	return angleValue;
}