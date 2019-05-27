// Library (Compass) robô de futebol 2018 CIC Robotics
// 16/01/2018
// Desenvolvedor: Breno Cunha Queiroz
#ifndef COMPASS_H
#define COMPASS_H

class Compass
{
public:
	Compass();
	bool getIsRealWorld();//return isRealWorld
	float value();//return the compassValue
	void update();//update the compassValue
	//Simulation
	void initializeSimulation(int ID);//initialize the simulation ultrasonics and ID

private:
	float compassValue;
	//Simulation
	int clientIDSimulation;
	int compassSimulation;
};

#endif //COMPASS_H