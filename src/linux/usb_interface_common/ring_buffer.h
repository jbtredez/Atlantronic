#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stddef.h>

class RingBuffer
{
	public:
		RingBuffer(int size);
		~RingBuffer();

		void clear();
		int push(unsigned char* data, unsigned int n);
		int pop(unsigned char* data, unsigned int n);
		int copy(unsigned char* data, unsigned int n);

		unsigned char* m_buffer;
		unsigned char* m_buffer_end;
		unsigned char* m_head;
		unsigned char* m_tail;
		unsigned int m_size;
		unsigned int m_count;

};

#endif
