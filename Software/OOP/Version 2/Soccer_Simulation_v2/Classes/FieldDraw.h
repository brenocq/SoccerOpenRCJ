///////////////////////////////////////////////////////////
//  FieldDraw.h
//  Implementation of the Class FieldDraw
//  Created on:      21-mar-2018 21:46:57
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////

#ifndef FIELDDRAW_H
#define FIELDDRAW_H

#include "Field.h"

#include <vector>
using std::vector;
#include <windows.h>//to see pixels

class Robot;

class FieldDraw : public Field
{
public:
	enum Color { EMPTY, WHITE, BLACK, RED, GREEN, BLUE, MAGENTA, PURPLE, CYAN, YELLOW, ORANGE };

	FieldDraw(Robot* _robot1, Robot* _robot2, Robot* _robot3, Robot* _robot4);
	
	virtual ~FieldDraw();

	//----- Robots -----//

	//----- Read -----//
	virtual int pointRead(int x0, int y0, int val);
	//----- Write -----//
	virtual void pointWrite(int x0, int y0, int val);

	//----- Configure Draw -----//
	void setSeeRobot(bool b, int robotNumber, Color color);
	void setSeeSonarLines(bool b, int robotNumber, Color color);
	void setSeeSonarPoints(bool b, int robotNumber, Color color);
	void setSeeReading(bool b, int robotNumber, Color color);
	void setSeeBall(bool b, int robotNumber, Color color);
	void setSeeAdversaries(bool b, int robotNumber, Color color);
	void setSeeFriend(bool b, int robotNumber, Color color);
	void setSeePath(bool b, int robotNumber, Color color);
	void setSeeFieldLine(bool b, Color color);

	//draw
	void promptSetPixel(int x,int y,COLORREF color);
	void print();

private:
	Color  **fieldMatrix;//Field
	//Objects
	Robot &robot1;
	Robot &robot2;
	Robot &robot3;
	Robot &robot4;
	vector <Robot*> robots;
	//configure draw
	Color seeRobot[5];//[1]=robot1 [2]=robot2 [3]=robot3 [4]=robot4
	Color seeSonarLines[5];
	Color seeSonarPoints[5];
	Color seeReading[5];//obstacles
	Color seeBall[5];//relative by robot
	Color seeAdversaries[5];//relative by robot
	Color seeFriend[5];
	Color seePath[5];
	Color seeFieldLine;
	//print objects
	HWND myconsole;
	HDC mydc;
	//variables
	bool firstPrint;

};
#endif // FIELDDRAW_H
