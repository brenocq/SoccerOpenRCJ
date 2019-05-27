#include "Classes/List.h" // designed by Deitel


// construtor padrão
template< typename NODETYPE >
List< NODETYPE >::List()
	: firstPtr(0), lastPtr(0)
{
	// corpo vazio
} // fim do construtor List

// destrutor
template< typename NODETYPE >
List< NODETYPE >::~List()
{
	if (!isEmpty()) // List não está vazia
	{
		cout << "Destroying nodes ...\n";

		ListNode< NODETYPE > *currentPtr = firstPtr;
		ListNode< NODETYPE > *tempPtr;

		while (currentPtr != 0) // exclui nós restantes
		{
			tempPtr = currentPtr;
			cout << tempPtr->data << '\n';
			currentPtr = currentPtr->nextPtr;
			delete tempPtr;
		} // fim do while
	} // fim do if

	cout << "All nodes destroyed\n\n";
} // fim do destrutor List

// insere nó na frente da lista
template< typename NODETYPE >
void List< NODETYPE >::insertAtFront(const NODETYPE &value)
{
	ListNode< NODETYPE > *newPtr = getNewNode(value); // novo nó

	if (isEmpty()) // List está vazia
		firstPtr = lastPtr = newPtr; // nova lista tem apenas um nó
	else // List não está vazia
	{
		newPtr->nextPtr = firstPtr; // aponta novo nó para o primeiro nó anterior 
		firstPtr = newPtr; // aponta firstPtr para o novo nó
	} // fim de else
} // fim da função insertAtFront

// insere nó no fim da lista
template< typename NODETYPE >
void List< NODETYPE >::insertAtBack(const NODETYPE &value)
{
	ListNode< NODETYPE > *newPtr = getNewNode(value); // novo nó

	if (isEmpty()) // List está vazia
		firstPtr = lastPtr = newPtr; // nova lista tem apenas um nó
	else // List não está vazia
	{
		lastPtr->nextPtr = newPtr; // atualiza o último nó anterior
		lastPtr = newPtr; // novo último nó
	} // fim de else
} // fim da função insertAtBack

// exclui nó da frente da lista
template< typename NODETYPE >
bool List< NODETYPE >::removeFromFront(NODETYPE &value)
{
	if (isEmpty()) // List está vazia
		return false; // exclui malsucedido
	else
	{
		ListNode< NODETYPE > *tempPtr = firstPtr; // armazena tempPtr para excluir

		if (firstPtr == lastPtr)
			firstPtr = lastPtr = 0; // nenhum nó permanece depois da exclusão
		else
			firstPtr = firstPtr->nextPtr; // aponta para segundo nó anterior

		value = tempPtr->data; // retorna os dados sendo removidos
		delete tempPtr; // reivindica nó frontal anterior
		return true; // exclusão bem-sucedido
	} // fim de else
} // fim da função removeFromFront

// exclui nó do fim da lista
template< typename NODETYPE >
bool List< NODETYPE >::removeFromBack(NODETYPE &value)
{
	if (isEmpty()) // List está vazia
		return false; // exclui malsucedido
	else
	{
		ListNode< NODETYPE > *tempPtr = lastPtr; // armazena tempPtr para excluir

		if (firstPtr == lastPtr) // List tem um elemento
			firstPtr = lastPtr = 0; // nenhum nó permanece depois da exclusão
		else
		{
			ListNode< NODETYPE > *currentPtr = firstPtr;

			// localiza do segundo ao último elemento
			while (currentPtr->nextPtr != lastPtr)
				currentPtr = currentPtr->nextPtr; // move para próximo nó

			lastPtr = currentPtr; // remove último nó
			currentPtr->nextPtr = 0; // esse é agora o último nó
		} // fim de else

		value = tempPtr->data; // retorna valor do último nó antigo
		delete tempPtr; // reivindica o primeiro último nó
		return true; // exclusão bem-sucedido
	} // fim de else
} // fim da função removeFromBack

// List está vazia?
template< typename NODETYPE >
bool List< NODETYPE >::isEmpty() const
{
	return firstPtr == 0;
} // fim da função isEmpty

// retorna ponteiro para nó recentemente alocado
template< typename NODETYPE >
ListNode< NODETYPE > *List< NODETYPE >::getNewNode(
	const NODETYPE &value)
{
	return new ListNode< NODETYPE >(value);
} // fim da função getNewNode

// exibe o conteúdo de List
template< typename NODETYPE >
void List< NODETYPE >::print() const
{
	if (isEmpty()) // List está vazia
	{
		cout << "The list is empty\n\n";
		return;
	} // fim do if

	ListNode< NODETYPE > *currentPtr = firstPtr;

	cout << "The list is: ";

	while (currentPtr != 0) // obtém dados de elemento
	{
		cout << currentPtr->data << ' ';
		currentPtr = currentPtr->nextPtr;
	} // fim do while

	cout << "\n\n";
} // fim da função print