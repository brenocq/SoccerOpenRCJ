///////////////////////////////////////////////////////////
//  Communication.h
//  Implementation of the Class Communication
//  Created on:      21-mar-2018 21:46:56
//  Original author: Breno Queiroz
///////////////////////////////////////////////////////////


#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "listnode.h" // definição da classe ListNode
//#include "List.h" // designed by Deitel


class Communication
{

public:
	Communication();
	virtual ~Communication();

	void write(const char byte, int channel);
	char read(int channel) const;
	int available(int channel)const;

private:
	List< char > *channel1;//the robot two write in the one
	List< char > *channel2;//the robot one write in the two
};

#include "listnode.h" // definição da classe ListNode

template< typename NODETYPE >
class List
{
public:
	List(); // construtor
	~List(); // destrutor
	void insertAtFront(const NODETYPE &);
	void insertAtBack(const NODETYPE &);
	bool removeFromFront(NODETYPE &);
	bool removeFromBack(NODETYPE &);
	int size() const;
	bool isEmpty() const;
	void print() const;
private:
	ListNode< NODETYPE > *firstPtr; // ponteiro para o primeiro nó
	ListNode< NODETYPE > *lastPtr; // ponteiro para o último nó  

	// função utilitária para alocar novo nó
	ListNode< NODETYPE > *getNewNode(const NODETYPE &);
}; // fim da classe List


#endif//COMMUNICATION_H