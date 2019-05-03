// Fig. 21.3: Listnode.h
// definição da template de classe ListNode.
#ifndef LISTNODE_H
#define LISTNODE_H

// declaração antecipada da classe List necessária para anunciar essa classe
// List existe, portanto, pode ser utilizada na declaração friend na linha 13
template< typename NODETYPE > class List;                                   

template< typename NODETYPE> 
class ListNode 
{
   friend class List< NODETYPE >; // torna List uma amiga (friend)

public:
   ListNode( const NODETYPE & ); // construtor
   NODETYPE getData() const; // retorna dados no nó
private:
   NODETYPE data; // dados
   ListNode< NODETYPE > *nextPtr; // próximo nó na lista
}; // fim da classe ListNode

// construtor
template< typename NODETYPE> 
ListNode< NODETYPE >::ListNode( const NODETYPE &info )
   : data( info ), nextPtr( 0 )
{ 
   // corpo vazio
} // fim do construtor ListNode

// retorna cópia de dados no nó
template< typename NODETYPE >
NODETYPE ListNode< NODETYPE >::getData() const 
{ 
   return data; 
} // fim da função getData

#endif


/**************************************************************************
 * (C) Copyright 1992-2005 Deitel & Associates, Inc. e                    *
 * Pearson Education, Inc. Todos os direitos reservados                   *
 *                                                                        *
 * NOTA DE ISENÇÃO DE RESPONSABILIDADES: Os autores e o editor deste      *
 * livro empregaram seus melhores esforços na preparação do livro. Esses  *
 * esforços incluem o desenvolvimento, pesquisa e teste das teorias e     *
 * programas para determinar sua eficácia. Os autores e o editor não      *
 * oferecem nenhum tipo de garantia, explícita ou implicitamente, com     *
 * referência a esses programas ou à documentação contida nesses livros.  *
 * Os autores e o editor não serão responsáveis por quaisquer danos,      *
 * acidentais ou conseqüentes, relacionados com ou provenientes do        *
 * fornecimento, desempenho ou utilização desses programas.               *
 **************************************************************************/
