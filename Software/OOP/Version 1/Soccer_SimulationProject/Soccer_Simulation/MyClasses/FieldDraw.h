// Library (FieldDraw) robô de futebol 2018 CIC Robotics
// 16/01/2018
// Desenvolvedor: Breno Cunha Queiroz

class Compass;
class Ultrasonic;
class FieldMatrix;
class DataBase;

#ifndef FIELDDRAW_H
#define FIELDDRAW_H

class FieldDraw
{
public:
	enum Color {EMPTY,WHITE, BLACK, RED, GREEN, BLUE, MAGENTA, CYAN, YELLOW};

	FieldDraw(DataBase &,Compass &, Ultrasonic &, FieldMatrix &);
	~FieldDraw();

	void line(int x0, int y0, int x1, int y1, Color color);
	void bigLine(int x0, int y0, int x1, int y1, int ratio, int lineDistance, Color color);
	void circle(int x0, int y0, int radius, Color color, int thickness);
	void fillCircle(int x0, int y0, int radius, Color color);

	void seeField();
	void setSeeSonarLines(bool b);
	void setSeeRobot(bool b);
	void setSeeBall(bool b);

	void sonarLines();
	
private:
	int angleTwoPoints(double x0, double  y0, double  x1, double y1) const;
	const int widthField;
	const int lengthField;
	const int robotRadio;

	bool seeSonarLines = false;
	bool seeRobot = false;
	bool seeBall = false;


	Color  **fieldMatrix;//Field
	//Objects
	Compass &compass;
	Ultrasonic &ultrasonic;
	FieldMatrix &reading;
	DataBase &dataBase;
};

#endif // FIELDDRAW_H