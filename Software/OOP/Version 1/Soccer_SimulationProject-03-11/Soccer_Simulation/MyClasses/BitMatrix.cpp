// Library (BitMatrix) robô de futebol 2018 CIC Robotics
// 11/02/2018
// Desenvolvedor: Breno Cunha Queiroz

#include "BitMatrix.h"

#if defined(ARDUINO) && ARDUINO>=100
#include "Arduino.h"//arduino version >= 1.0.0
#elif defined(ARDUINO) && ARDUINO<100
#include "WProgram.h"//arduino old version
#else
#include <cstdint>
#include <iostream>
using namespace std;
#endif

BitMatrix::BitMatrix(int col, int row)
	:byteRows(row),
	byteColumns( (col % 8 == 0) ? (col / 8) : (col / 8 + 1) )
{

	byteMatrix = new uint8_t*[byteRows];//create rows

	for (int i = 0; i < byteRows; ++i)//create columns
		byteMatrix[i] = new uint8_t[byteColumns];

	for (int i = 0; i < byteColumns; i++)//set bytes to 0
	{
		for (int j = 0; j < byteRows; j++)
		{
			byteMatrix[j][i] = 0;
		}
	}
}

BitMatrix::~BitMatrix()
{
	for (int i = 0; i < byteRows; ++i)//delete coloumns
		delete[] byteMatrix[i];

	delete[] byteMatrix;//delete rows
}

//write one bit in matrix
void BitMatrix::write(int col, int row, bool b)//used in bitMatrix(x,y)=bool
{
	uint16_t bytecol = col / 8;
	int SHIFT = col % 8;
	uint8_t mask = 128;//1000 0000

	mask = mask >> SHIFT;

	uint8_t result = byteMatrix[row][bytecol] & mask;

	if (result == 0 && b == true)
	{
		byteMatrix[row][bytecol] |= mask;
	}
	else if (result > 0 && b == false)
	{
		byteMatrix[row][bytecol] ^= mask;
	}
}

//read one bit in matrix
bool BitMatrix::read(int col, int row) const
{
	int bytecol = col / 8;
	int SHIFT = col % 8;
	uint8_t byte = byteMatrix[row][bytecol];
	uint8_t mask = 128;//10000000

	uint8_t result = byte & (mask >> SHIFT);
	if (result == 0)
		return false;
	else
		return true;
}

//write one byte in matrix
void BitMatrix::writeByte(int col, int row, bool value)
{
	int bytecol = col / 8;
	if (value == false)
		byteMatrix[row][bytecol] = 0;
	else
		byteMatrix[row][bytecol] = 255;
}

//read one byte in matrix
bool BitMatrix::readByte(int col, int row) const
{
	int bytecol = col / 8;
	if (byteMatrix[row][bytecol] == 0)
		return false;
	else
		return true;
}

//it works only in simulation
void  BitMatrix::print() const
{
#if !defined(ARDUINO)
	for (int j = 0; j < byteRows; j++)
	{
		for (int i = 0; i < byteColumns*8; i++)
		{
			if ((i % 8 == 0))
				cout << " ";
			read(i, j) == true ? cout << 1 : cout << 0;
		}
		cout << endl;
	}
	cout << endl;
#endif
}

