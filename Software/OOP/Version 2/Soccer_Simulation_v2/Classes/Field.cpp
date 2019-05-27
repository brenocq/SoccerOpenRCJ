///////////////////////////////////////////////////////////
//  Field.cpp
//  Implementation of the Class Field
//  Created on:      21-mar-2018 21:46:57
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#include "Field.h"

#define _USE_MATH_DEFINES
#include <math.h>//Pi,cos,sin,atan

#include <cstdint>//int16_t values
#include <algorithm>//swap function
using std::swap;

#include <iostream>//deletar
using namespace std;

Field::Field()
	:widthField(182), lengthField(243)
{

}

Field::~Field(){

}

//Read
int Field::lineRead(int x0, int y0, int x1, int y1, int val){
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
				if (pointRead(y0, x0, val) == 1)
					pointsSum++;
		}
		else {
			if (y0 < 0)
				y0 = 0;
			if (x0 < 0)
				x0 = 0;
			if (((x0) >= 0 && (x0) < widthField) && ((y0) >= 0 && (y0) < lengthField))
				if (pointRead(x0, y0, val) == 1)
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
int Field::bigLineRead(int x0, int y0, int x1, int y1, int thickness, int val){
	int lineAngle = angleTwoPoints(x0, y0, x1, y1);
	//-----temp variables-----//
	float trigonometricAngle = 0;
	//-----left point 0-----//
	int pointXLeft0 = 0;
	int pointYLeft0 = 0;

	trigonometricAngle = ((lineAngle + 90) + 90)*M_PI / 180;//angle in radians
	pointXLeft0 = x0 + int(thickness*cos(trigonometricAngle));
	pointYLeft0 = y0 + int(thickness*sin(trigonometricAngle));

	//-----right point 0-----//
	int pointXRight0 = 0;
	int pointYRight0 = 0;

	trigonometricAngle = ((lineAngle - 90) + 90)*M_PI / 180;//angle in radians
	pointXRight0 = x0 + int(thickness*cos(trigonometricAngle));
	pointYRight0 = y0 + int(thickness*sin(trigonometricAngle));

	//-----left point 1-----//
	int pointXLeft1 = 0;
	int pointYLeft1 = 0;

	trigonometricAngle = ((lineAngle - 90) + 90)*M_PI / 180;//angle in radians
	pointXLeft1 = x1 + int(thickness*cos(trigonometricAngle));
	pointYLeft1 = y1 + int(thickness*sin(trigonometricAngle));

	//-----right point 1-----//
	int pointXRight1 = 0;
	int pointYRight1 = 0;

	trigonometricAngle = ((lineAngle + 90) + 90)*M_PI / 180;//angle in radians
	pointXRight1 = x1 + int(thickness*cos(trigonometricAngle));
	pointYRight1 = y1 + int(thickness*sin(trigonometricAngle));

	return fillQuadrilateralRead(	pointXLeft0, pointYLeft0, pointXRight0, pointYRight0,
									pointXLeft1, pointYLeft1, pointXRight1, pointYRight1, val);
}
int Field::circleRead(int x0, int y0, int radius, int thickness, int val){
	//returns the number of points in the circle that have the value
	int pointsSum = 0;

	int x = radius - 1;
	int y = 0;
	int dx = 1;
	int dy = 1;
	int err = dx - (radius << 1);

	while (x >= y)
	{
		if (((x0 + x) >= 0 && (x0 + x) < widthField) && ((y0 + y) >= 0 && (y0 + y) < lengthField))
		if (pointRead(x0 + x, y0 + y, val) == 1)
			pointsSum++;

		if (((x0 + y) >= 0 && (x0 + y) < widthField) && ((y0 + x) >= 0 && (y0 + x) < lengthField))
		if (pointRead(x0 + y, y0 + x, val) == 1)
			pointsSum++;


		if (((x0 - y) >= 0 && (x0 - y) < widthField) && ((y0 + x) >= 0 && (y0 + x) < lengthField))
		if (pointRead(x0 - y, y0 + x, val) == 1)
			pointsSum++;

		if (((x0 - x) >= 0 && (x0 - x) < widthField) && ((y0 + y) >= 0 && (y0 + y) < lengthField))
		if (pointRead(x0 - x, y0 + y, val) == 1)
			pointsSum++;

		if (((x0 - x) >= 0 && (x0 - x) < widthField) && ((y0 - y) >= 0 && (y0 - y) < lengthField))
		if (pointRead(x0 - x, y0 - y, val) == 1)
			pointsSum++;

		if (((x0 - y) >= 0 && (x0 - y) < widthField) && ((y0 - x) >= 0 && (y0 - x) < lengthField))
		if (pointRead(x0 - y, y0 - x, val) == 1)
			pointsSum++;

		if (((x0 + y) >= 0 && (x0 + y) < widthField) && ((y0 - x) >= 0 && (y0 - x) < lengthField))
		if (pointRead(x0 + y, y0 - x, val) == 1)
			pointsSum++;

		if (((x0 + x) >= 0 && (x0 + x) < widthField) && ((y0 - y) >= 0 && (y0 - y) < lengthField))
		if (pointRead(x0 + x, y0 - y, val) == 1)
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
		pointsSum += circleRead(x0, y0, radius - 1, thickness - 1,val);

	return pointsSum;
}
int Field::fillCircleRead(int x0, int y0, int radius, int val){
	//returns the number of points in the fill circle that have the value
	int pointsSum = 0;

	pointsSum += lineRead((x0), (y0 - radius), (x0), (y0 - radius) + (2 * radius + 1) - 1, val);


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
			pointsSum += lineRead((x0 + x), (y0 - y), (x0 + x), (y0 - y) + (2 * y + 1 + 0) - 1, val);
			//drawFastVLine(x0 + y, y0 - x, 2 * x + 1 + 0, color);
			pointsSum += lineRead((x0 + y), (y0 - x), (x0 + y), (y0 - x) + (2 * x + 1 + 0) - 1, val);
		}
		if (3 & 0x2) {
			//drawFastVLine(x0 - x, y0 - y, 2 * y + 1 + 0, color);
			pointsSum += lineRead((x0 - x), (y0 - y), (x0 - x), (y0 - y) + (2 * y + 1 + 0) - 1, val);
			//drawFastVLine(x0 - y, y0 - x, 2 * x + 1 + 0, color);
			pointsSum += lineRead((x0 - y), (y0 - x), (x0 - y), (y0 - x) + (2 * x + 1 + 0) - 1, val);
		}
	}
	return pointsSum;
}
int Field::fillQuadrilateralRead(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3, int val){
	//returns the number of points in the quadrilateral that have the value
	int pointsSum = 0;
	pointsSum += fillTriangleRead(x0, y0, x2, y2, x1, y1, val);
	pointsSum += fillTriangleRead(x0, y0, x2, y2, x3, y3, val);
	return pointsSum;
}
int Field::fillTriangleRead(int x1, int y1, int x2, int y2, int x3, int y3, int val){
	//returns the number of points in the triangle that have the value
	int pointsSum = 0;

	uint8_t t1x, t2x, y, minx, maxx, t1xp, t2xp;
	bool changed1 = false;
	bool changed2 = false;
	int8_t signx1, signx2, dx1, dy1, dx2, dy2;
	uint8_t e1, e2;
	// Sort vertices
	if (y1>y2) { swap(y1, y2); swap(x1, x2); }
	if (y1>y3) { swap(y1, y3); swap(x1, x3); }
	if (y2>y3) { swap(y2, y3); swap(x2, x3); }

	t1x = t2x = x1; y = y1;   // Starting points

	dx1 = (int8_t)(x2 - x1); if (dx1<0) { dx1 = -dx1; signx1 = -1; }
	else signx1 = 1;
	dy1 = (int8_t)(y2 - y1);

	dx2 = (int8_t)(x3 - x1); if (dx2<0) { dx2 = -dx2; signx2 = -1; }
	else signx2 = 1;
	dy2 = (int8_t)(y3 - y1);

	if (dy1 > dx1) {   // swap values
		swap(dx1, dy1);
		changed1 = true;
	}
	if (dy2 > dx2) {   // swap values
		swap(dy2, dx2);
		changed2 = true;
	}

	e2 = (uint8_t)(dx2 >> 1);
	// Flat top, just process the second half
	if (y1 == y2) goto next;
	e1 = (uint8_t)(dx1 >> 1);

	for (uint8_t i = 0; i < dx1;) {
		t1xp = 0; t2xp = 0;
		if (t1x<t2x) { minx = t1x; maxx = t2x; }
		else		{ minx = t2x; maxx = t1x; }
		// process first line until y value is about to change
		while (i<dx1) {
			i++;
			e1 += dy1;
			while (e1 >= dx1) {
				e1 -= dx1;
				if (changed1) t1xp = signx1;//t1x += signx1;
				else          goto next1;
			}
			if (changed1) break;
			else t1x += signx1;
		}
		// Move line
	next1:
		// process second line until y value is about to change
		while (1) {
			e2 += dy2;
			while (e2 >= dx2) {
				e2 -= dx2;
				if (changed2) t2xp = signx2;//t2x += signx2;
				else          goto next2;
			}
			if (changed2)     break;
			else              t2x += signx2;
		}
	next2:
		if (minx>t1x) minx = t1x; if (minx>t2x) minx = t2x;
		if (maxx<t1x) maxx = t1x; if (maxx<t2x) maxx = t2x;
		pointsSum += lineRead(minx, y, maxx, y, val);// Read line from min to max points found on the y
		// Now increase y
		if (!changed1) t1x += signx1;
		t1x += t1xp;
		if (!changed2) t2x += signx2;
		t2x += t2xp;
		y += 1;
		if (y == y2) break;

	}
next:
	// Second half
	dx1 = (int8_t)(x3 - x2); if (dx1<0) { dx1 = -dx1; signx1 = -1; }
	else signx1 = 1;
	dy1 = (int8_t)(y3 - y2);
	t1x = x2;

	if (dy1 > dx1) {   // swap values
		swap(dy1, dx1);
		changed1 = true;
	}
	else changed1 = false;

	e1 = (uint8_t)(dx1 >> 1);

	for (uint8_t i = 0; i <= dx1; i++) {
		t1xp = 0; t2xp = 0;
		if (t1x<t2x) { minx = t1x; maxx = t2x; }
		else		{ minx = t2x; maxx = t1x; }
		// process first line until y value is about to change
		while (i<dx1) {
			e1 += dy1;
			while (e1 >= dx1) {
				e1 -= dx1;
				if (changed1) { t1xp = signx1; break; }//t1x += signx1;
				else          goto next3;
			}
			if (changed1) break;
			else   	   	  t1x += signx1;
			if (i<dx1) i++;
		}
	next3:
		// process second line until y value is about to change
		while (t2x != x3) {
			e2 += dy2;
			while (e2 >= dx2) {
				e2 -= dx2;
				if (changed2) t2xp = signx2;
				else          goto next4;
			}
			if (changed2)     break;
			else              t2x += signx2;
		}
	next4:

		if (minx>t1x) minx = t1x; if (minx>t2x) minx = t2x;
		if (maxx<t1x) maxx = t1x; if (maxx<t2x) maxx = t2x;
		pointsSum += lineRead(minx, y, maxx, y, val);// Read line from min to max points found on the y
		// Now increase y
		if (!changed1) t1x += signx1;
		t1x += t1xp;
		if (!changed2) t2x += signx2;
		t2x += t2xp;
		y += 1;
		if (y>y3) return pointsSum;
	}
}
//Write
void Field::lineWrite(int x0, int y0, int x1, int y1, int val){
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
			pointWrite(y0, x0, val);
		}
		else {
			if (y0 < 0)
				y0 = 0;
			if (x0 < 0)
				x0 = 0;
			if (((x0) >= 0 && (x0) < widthField) && ((y0) >= 0 && (y0) < lengthField))
				pointWrite(x0, y0, val);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}
void Field::bigLineWrite(int x0, int y0, int x1, int y1, int thickness, int val){
	int lineAngle = angleTwoPoints(x0, y0, x1, y1);
	//-----temp variables-----//
	float trigonometricAngle = 0;
	//-----left point 0-----//
	int pointXLeft0 = 0;
	int pointYLeft0 = 0;

	trigonometricAngle = ((lineAngle + 90) + 90)*M_PI / 180;//angle in radians
	pointXLeft0 = x0 + int(thickness*cos(trigonometricAngle));
	pointYLeft0 = y0 + int(thickness*sin(trigonometricAngle));

	//-----right point 0-----//
	int pointXRight0 = 0;
	int pointYRight0 = 0;

	trigonometricAngle = ((lineAngle - 90) + 90)*M_PI / 180;//angle in radians
	pointXRight0 = x0 + int(thickness*cos(trigonometricAngle));
	pointYRight0 = y0 + int(thickness*sin(trigonometricAngle));

	//-----left point 1-----//
	int pointXLeft1 = 0;
	int pointYLeft1 = 0;

	trigonometricAngle = ((lineAngle - 90) + 90)*M_PI / 180;//angle in radians
	pointXLeft1 = x1 + int(thickness*cos(trigonometricAngle));
	pointYLeft1 = y1 + int(thickness*sin(trigonometricAngle));

	//-----right point 1-----//
	int pointXRight1 = 0;
	int pointYRight1 = 0;

	trigonometricAngle = ((lineAngle + 90) + 90)*M_PI / 180;//angle in radians
	pointXRight1 = x1 + int(thickness*cos(trigonometricAngle));
	pointYRight1 = y1 + int(thickness*sin(trigonometricAngle));

	fillQuadrilateralWrite(	pointXLeft0, pointYLeft0, pointXRight0, pointYRight0,
							pointXLeft1, pointYLeft1, pointXRight1, pointYRight1, val);
}
void Field::circleWrite(int x0, int y0, int radius, int thickness, int val){
	int x = radius - 1;
	int y = 0;
	int dx = 1;
	int dy = 1;
	int err = dx - (radius << 1);

	while (x >= y)
	{
		if (((x0 + x) >= 0 && (x0 + x) < widthField) && ((y0 + y) >= 0 && (y0 + y) < lengthField))
		pointWrite(x0 + x, y0 + y, val);

		if (((x0 + y) >= 0 && (x0 + y) < widthField) && ((y0 + x) >= 0 && (y0 + x) < lengthField))
		pointWrite(x0 + y, y0 + x, val);

		if (((x0 - y) >= 0 && (x0 - y) < widthField) && ((y0 + x) >= 0 && (y0 + x) < lengthField))
		pointWrite(x0 - y, y0 + x, val);

		if (((x0 - x) >= 0 && (x0 - x) < widthField) && ((y0 + y) >= 0 && (y0 + y) < lengthField))
		pointWrite(x0 - x, y0 + y, val);

		if (((x0 - x) >= 0 && (x0 - x) < widthField) && ((y0 - y) >= 0 && (y0 - y) < lengthField))
		pointWrite(x0 - x, y0 - y, val);

		if (((x0 - y) >= 0 && (x0 - y) < widthField) && ((y0 - x) >= 0 && (y0 - x) < lengthField))
		pointWrite(x0 - y, y0 - x, val);

		if (((x0 + y) >= 0 && (x0 + y) < widthField) && ((y0 - x) >= 0 && (y0 - x) < lengthField))
		pointWrite(x0 + y, y0 - x, val);

		if (((x0 + x) >= 0 && (x0 + x) < widthField) && ((y0 - y) >= 0 && (y0 - y) < lengthField))
		pointWrite(x0 + x, y0 - y, val);

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
		circleWrite(x0, y0, radius - 1, thickness - 1, val);
}
void Field::fillCircleWrite(int x0, int y0, int radius, int val){
	lineWrite((x0), (y0 - radius), (x0), (y0 - radius) + (2 * radius + 1) - 1, val);


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
			lineWrite((x0 + x), (y0 - y), (x0 + x), (y0 - y) + (2 * y + 1 + 0) - 1, val);
			lineWrite((x0 + y), (y0 - x), (x0 + y), (y0 - x) + (2 * x + 1 + 0) - 1, val);
		}
		if (3 & 0x2) {
			lineWrite((x0 - x), (y0 - y), (x0 - x), (y0 - y) + (2 * y + 1 + 0) - 1, val);
			lineWrite((x0 - y), (y0 - x), (x0 - y), (y0 - x) + (2 * x + 1 + 0) - 1, val);
		}
	}
}
void Field::arcThreePoints(float x1, float y1, float x2, float y2, float x3, float y3, int val){
	//help from http://www.ucancode.net/faq/C-Line-Intersection-2D-drawing.htm
	float xc;
	float yc;

	//points one and two (slope-intercept) y=ax+b
	float x1m = (x1+x2)/2;
	float y1m = (y1+y2)/2;
	float A1s = -(x2 - x1)/(y2 - y1);
	float B1s = y1m - A1s*x1m;
	//points one and two (standard form) Ax+By=C
	float A1 = A1s;
	float B1 = -1;
	float C1 = -B1s;

	//points two and three (slope-intercept) y=ax+b
	float x2m = (x3 + x2) / 2;
	float y2m = (y3 + y2) / 2;
	float A2s = -(x2 - x3) / (y2 - y3);
	float B2s = y2m - A2s*x2m;
	//points  two and three (standard form) Ax+By=C
	float A2 = A2s;
	float B2 = -1;
	float C2 = -B2s;

	float det = A1*B2 - A2*B1;
	if (det == 0){
		//Lines are parallel
	}
	else{
		xc = (B2*C1 - B1*C2) / det;
		yc = (A1*C2 - A2*C1) / det;

		circleWrite(x1, y1, 3, 2, val);
		circleWrite(x2, y2, 3, 2, val);
		circleWrite(x3, y3, 3, 2, val);

		float radius = distTwoPoints(xc, yc, x1, y1);

		float angle1 = atan2(y1 - yc, x1-xc ) * 180 / M_PI;//atan2(y,x)
		float angle2 = atan2(y2 - yc, x2 - xc) * 180 / M_PI;//atan2(y,x)
		float angle3 = atan2(y3 - yc, x3- xc) * 180 / M_PI;//atan2(y,x)

		float angleNow = angle3;
		float angleRate = 3;

		while (angleNow < angle1){
			//cout << "now:" << angleNow << endl;

			float pointX = xc + radius*cos(angleNow / 180 * 3.1415);
			float pointY = yc + radius*sin(angleNow / 180 * 3.1415);
			//cout << angleNow << "<" << angle1 << "pX:" << pointX << "pY:" << pointY << endl;
			fillCircleWrite(pointX, pointY, 5, val);
			angleNow += angleRate;
		}

		//lineWrite(x1, y1, x2, y2, val);
		//lineWrite(x3, y3, x2, y2, val);
		//lineWrite(x1m, y1m, x, y, val);
		//lineWrite(x2m, y2m, x, y, val);
		circleWrite(xc, yc, radius, 1, val);
	}
}
void Field::fillQuadrilateralWrite(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3, int val){
	fillTriangleWrite(x0, y0, x2, y2, x1, y1, val);
	fillTriangleWrite(x0, y0, x2, y2, x3, y3, val);
}
void Field::fillCenterSquareWrite(int x, int y, int l, int h, int val){
	int halfLenght = l/2;
	int halfHeight = h/2;

	fillTriangleWrite(x - halfLenght, y - halfHeight, x + halfLenght, y - halfHeight, x + halfLenght, y + halfHeight, val);
	fillTriangleWrite(x - halfLenght, y - halfHeight, x - halfLenght, y + halfHeight, x + halfLenght, y + halfHeight, val);
}
void Field::fillTriangleWrite(int x1, int y1, int x2, int y2, int x3, int y3, int val){
	uint8_t t1x, t2x, y, minx, maxx, t1xp, t2xp;
	bool changed1 = false;
	bool changed2 = false;
	int8_t signx1, signx2, dx1, dy1, dx2, dy2;
	uint8_t e1, e2;
	// Sort vertices
	if (y1>y2) { swap(y1, y2); swap(x1, x2); }
	if (y1>y3) { swap(y1, y3); swap(x1, x3); }
	if (y2>y3) { swap(y2, y3); swap(x2, x3); }

	t1x = t2x = x1; y = y1;   // Starting points

	dx1 = (int8_t)(x2 - x1); if (dx1<0) { dx1 = -dx1; signx1 = -1; }
	else signx1 = 1;
	dy1 = (int8_t)(y2 - y1);

	dx2 = (int8_t)(x3 - x1); if (dx2<0) { dx2 = -dx2; signx2 = -1; }
	else signx2 = 1;
	dy2 = (int8_t)(y3 - y1);

	if (dy1 > dx1) {   // swap values
		swap(dx1, dy1);
		changed1 = true;
	}
	if (dy2 > dx2) {   // swap values
		swap(dy2, dx2);
		changed2 = true;
	}

	e2 = (uint8_t)(dx2 >> 1);
	// Flat top, just process the second half
	if (y1 == y2) goto next;
	e1 = (uint8_t)(dx1 >> 1);

	for (uint8_t i = 0; i < dx1;) {
		t1xp = 0; t2xp = 0;
		if (t1x<t2x) { minx = t1x; maxx = t2x; }
		else		{ minx = t2x; maxx = t1x; }
		// process first line until y value is about to change
		while (i<dx1) {
			i++;
			e1 += dy1;
			while (e1 >= dx1) {
				e1 -= dx1;
				if (changed1) t1xp = signx1;//t1x += signx1;
				else          goto next1;
			}
			if (changed1) break;
			else t1x += signx1;
		}
		// Move line
	next1:
		// process second line until y value is about to change
		while (1) {
			e2 += dy2;
			while (e2 >= dx2) {
				e2 -= dx2;
				if (changed2) t2xp = signx2;//t2x += signx2;
				else          goto next2;
			}
			if (changed2)     break;
			else              t2x += signx2;
		}
	next2:
		if (minx>t1x) minx = t1x; if (minx>t2x) minx = t2x;
		if (maxx<t1x) maxx = t1x; if (maxx<t2x) maxx = t2x;
		lineWrite(minx, y, maxx, y, val);    // Draw line from min to max points found on the y
		// Now increase y
		if (!changed1) t1x += signx1;
		t1x += t1xp;
		if (!changed2) t2x += signx2;
		t2x += t2xp;
		y += 1;
		if (y == y2) break;

	}
next:
	// Second half
	dx1 = (int8_t)(x3 - x2); if (dx1<0) { dx1 = -dx1; signx1 = -1; }
	else signx1 = 1;
	dy1 = (int8_t)(y3 - y2);
	t1x = x2;

	if (dy1 > dx1) {   // swap values
		swap(dy1, dx1);
		changed1 = true;
	}
	else changed1 = false;

	e1 = (uint8_t)(dx1 >> 1);

	for (uint8_t i = 0; i <= dx1; i++) {
		t1xp = 0; t2xp = 0;
		if (t1x<t2x) { minx = t1x; maxx = t2x; }
		else		{ minx = t2x; maxx = t1x; }
		// process first line until y value is about to change
		while (i<dx1) {
			e1 += dy1;
			while (e1 >= dx1) {
				e1 -= dx1;
				if (changed1) { t1xp = signx1; break; }//t1x += signx1;
				else          goto next3;
			}
			if (changed1) break;
			else   	   	  t1x += signx1;
			if (i<dx1) i++;
		}
	next3:
		// process second line until y value is about to change
		while (t2x != x3) {
			e2 += dy2;
			while (e2 >= dx2) {
				e2 -= dx2;
				if (changed2) t2xp = signx2;
				else          goto next4;
			}
			if (changed2)     break;
			else              t2x += signx2;
		}
	next4:

		if (minx>t1x) minx = t1x; if (minx>t2x) minx = t2x;
		if (maxx<t1x) maxx = t1x; if (maxx<t2x) maxx = t2x;
		lineWrite(minx, y, maxx, y, val);    // Draw line from min to max points found on the y
		// Now increase y
		if (!changed1) t1x += signx1;
		t1x += t1xp;
		if (!changed2) t2x += signx2;
		t2x += t2xp;
		y += 1;
		if (y>y3) return;
	}
}
void Field::cleanField(int val){
	for (int i = 0; i < lengthField; i++)
	{
		for (int j = 0; j < widthField; j++)
		{
			pointWrite(j, i, val);
		}
	}
}


//functions
float Field::distTwoPoints(int x0, int  y0, int  x1, int y1){
	int distanceX = (x0 - x1);
	int distanceY = (y0 - y1);
	int hip = pow(distanceX, 2) + pow(distanceY, 2);

	return sqrt(hip);
}
float Field::angleTwoPoints(int x0, int  y0, int  x1, int y1){
	int angleValue = 360 - (int(atan2(x0 - x1, y0 - y1) * 180 / M_PI)) + 180;

	while (angleValue > 360)
		angleValue -= 360;
	while (angleValue < 0)
		angleValue += 360;
	return angleValue;
}

//Get property
int Field::getWidthField() const
{
	return widthField;
}
int Field::getLengthField() const
{
	return lengthField;
}