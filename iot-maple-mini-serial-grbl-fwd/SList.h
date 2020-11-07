#ifndef _SEARCHATHING_ARDUINO_UTILS_SLIST_H
#define _SEARCHATHING_ARDUINO_UTILS_SLIST_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

namespace SearchAThing
{

	namespace Arduino
	{
		
		// Templated simple linked-list node element.
		// Store templated element `T' into a node object that allow to
		// follow using a `next' pointer to the next one in the list.		
		// The given template `T' must support default constructor for
		// internal purpose.
		template<class T>
		class SListNode
		{
		public:
			// Constructor stores a copy of given element into internal data.
			SListNode<T>(const T& _data) { data = _data; }

			// Element data.
			T data;

			// Pointer to the next node in simple linked list.
			SListNode<T> *next = NULL;
		};

		// Templated simple linked-list.
		// Store templated element `T' into a simple linked list.
		// The given template `T' must support default constructor for
		// internal purpose.
		template<class T>
		class SList
		{
			uint16_t size = 0;
			SListNode<T> *first = NULL;
			SListNode<T> *last = NULL;

		public:
			// Default constructor.
			SList()
			{
			}

			// Copy constructor.
			SList(const SList& other)
			{
				*this = other;
			}

			// Assign operator. Creates a copy of the given `other' list.
			SList& operator = (const SList& other)
			{
				Clear();

				auto node = other.first;

				while (node)
				{
					Add(node->data);
					node = node->next;
				}

				return *this;
			}

			// Destructor. Deallocates memory used for nodes and thus calls
			// destructor of stored templated objects.
			~SList()
			{
				Clear();
			}

			// Current list size.
			uint16_t Size() const { return size; }

			// Adds given templated object `data' to the list.
			// If DEBUG and DEBUG_ASSERT are enabled a report about out-of-
			// memory will be reported if unable to allocate more nodes.
			T& Add(const T& data)
			{
				auto node = new SListNode<T>(data);
				if (node == NULL)
				{
#if defined DEBUG && defined DEBUG_ASSERT	
					DPrint(F("* Fatal: SList alloc of node out of memory"));
#endif
				}
				if (first == NULL)
					first = last = node;
				else
				{
					last->next = node;
					last = node;
				}
				++size;

				return node->data;
			}

			// Clear the list destroying each nodes thus calling the
			// destructor of contained templated data objects.
			void Clear()
			{
				if (first == NULL) return;

				SListNode<T> *node = first;
				while (node != NULL)
				{
					SListNode<T> *tmp = node->next;
					delete node;
					node = tmp;
					--size;
				}
				first = last = NULL;
			}

			// Remove the node by idx ( 0 is the first ).
			// It does nothing if invalid index out of bounds.
			void Remove(uint16_t idx)
			{
				if (idx >= size) return; // arg exception

				if (idx == 0) // delete first
				{
					SListNode<T> *tmp = first->next;
					delete first;
					first = tmp;
				}
				else
				{
					SListNode<T> *before = GetNode(idx - 1);
					if (idx == size - 1) // delete last
					{
						delete last;
						before->next = NULL;
						last = before;
					}
					else
					{
						SListNode<T> *after = before->next->next;
						delete before->next;
						before->next = after;
					}
				}

				--size;
			}

			// Retrieve a reference of the template object at the given `idx'
			// in the node list. Note: don't use `auto' pointer of the
			// returned object will be copied instead of referenced.
			T& Get(int idx) const
			{
				return GetNode(idx)->data;
			}

			// Retrieve a pointer to the node at the given `idx'
			// ( 0 is start ). If an invalid index was given returns NULL.
			SListNode<T> *GetNode(int idx) const
			{
				if (idx == size - 1) return last;

				if (idx >= size) return NULL;

				SListNode<T> *res = first;
				while (idx--) { res = res->next; }

				return res;
			}

		};

	}

}

#endif