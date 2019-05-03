// Library (BitMatrix) robô de futebol 2018 CIC Robotics
// 11/02/2018
// Desenvolvedor: Breno Cunha Queiroz
#ifndef BITMATRIX_H
#define BITMATRIX_H

#if defined(ARDUINO) && ARDUINO>=100
#include "Arduino.h"//arduino version >= 1.0.0
#elif defined(ARDUINO) && ARDUINO<100
#include "WProgram.h"//arduino old version
#else
 #include <cstdint>
#endif

class BitMatrix
{
public:
	BitMatrix(int row, int col);
	~BitMatrix();

	//----------------------------BIT
	void write(int col, int row,bool value);//true->1 false->0
	bool read(int col, int row) const;//return (bit==1?true:false)

	//----------------------------BYTE
	void writeByte(int col, int row, bool value);//true->255 false->0
	bool readByte(int col, int row) const;//return (byte!=0?true:false)

	void print() const;//can't be used in Arduino (yet)

private:
	uint8_t **byteMatrix;//looks like byteMatrix[row][col]
	const int byteRows;
	const int byteColumns;
};

#endif // BITMATRIX_H
