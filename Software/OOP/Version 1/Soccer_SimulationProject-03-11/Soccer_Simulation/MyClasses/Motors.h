// Library (Motors) robô de futebol 2018 CIC Robotics
// 16/01/2018
// Desenvolvedor: Breno Cunha Queiroz

#ifndef MOTORS_H
#define MOTORS_H

class Compass;

class Motors
{
public:
	enum MotorInitials { FR, BR, BL, FL };
	Motors(Compass &);
	void setMaxPower(int maximumPower);//											(maximumPower)
	void pin(MotorInitials motorInitials, int pos, int neg, int pwm);//			("FR"/"FL"/"BR"/"BL", pin+, pin-, pinPWM)

	void motor(MotorInitials motorInitials, float motorPower);//control one motor with real initial
	void relativeMotor(MotorInitials motorInitials, int motorPower);//control one motor with realive initial (motors change)
	void simpleMove(int moveAngle, int frontRobotAngle);//							(moveAngle, frontRobotAngle)

	//Simulation
	void initializeSimulation(int ID);//initialize the simulation motors and ID

private:
	float PIDController(int realRobotAngle);//										(realRobotAngle)
	bool isNear(float firstNumber, float secondNumber, float discrepancy);

	int maxPower;
	float Kp;//used in PIDController

	int motorFR[3];//pins [+,-,pwm]
	int motorBR[3];//pins [+,-,pwm]
	int motorBL[3];//pins [+,-,pwm]
	int motorFL[3];//pins [+,-,pwm]
	//Simulation
	int clientIDSimulation;
	int MotorFRSimulation;
	int MotorFLSimulation;
	int MotorBRSimulation;
	int MotorBLSimulation;
	//Objects
	Compass &compass;

};

#endif // MOTORS_H