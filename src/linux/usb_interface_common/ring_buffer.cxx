#include "ring_buffer.h"
#include <string.h>

RingBuffer::RingBuffer(int size)
{
	m_size = size;
	m_buffer = new unsigned char[m_size];
	m_buffer_end = m_buffer + m_size;
	m_head = m_buffer;
	m_tail = m_buffer;
	m_count = 0;
}

RingBuffer::~RingBuffer()
{
	delete [ ] m_buffer;
}

void RingBuffer::clear()
{
	m_head = m_buffer;
	m_tail = m_buffer;
	m_count = 0;
}

int RingBuffer::push(unsigned char* data, unsigned int n)
{
	if( m_count + n > m_size)
	{
		return -1;
	}

	unsigned int nMax = m_buffer_end - m_head;
	if( n < nMax )
	{
		nMax = n;
	}
	memcpy(m_head, data, nMax);

	if( n > nMax )
	{
		// on cycle sur le buffer
		// copie du reste
		memcpy(m_buffer, data + nMax, n - nMax);
		m_head = m_buffer + n - nMax;
	}
	else
	{
		m_head = m_head + nMax;
		if(m_head == m_buffer_end)
		{
			m_head = m_buffer;
		}
	}

	m_count += n;
	return 0;
}

int RingBuffer::pop(unsigned char* data, unsigned int n)
{
	if( m_count < n)
	{
		return -1;
	}

	unsigned int nMax = m_buffer_end - m_tail;
	if(n < nMax)
	{
		nMax = n;
	}

	memcpy(data, m_tail, nMax);

	if( n > nMax )
	{
		// on cycle sur le buffer
		// copie du reste
		memcpy(data + nMax, m_buffer, n - nMax);
		m_tail = m_buffer + n - nMax;
	}
	else
	{
		m_tail = m_tail + nMax;
		if(m_tail == m_buffer_end)
		{
			m_tail = m_buffer;
		}
	}

	m_count -= n;

	return 0;
}

int RingBuffer::copy(unsigned char* data, unsigned int n)
{
	if( m_count < n)
	{
		return -1;
	}

	unsigned int nMax = m_buffer_end - m_tail;
	if(n < nMax)
	{
		nMax = n;
	}

	memcpy(data, m_tail, nMax);

	if( n > nMax )
	{
		// on cycle sur le buffer
		// copie du reste
		memcpy(data + nMax, m_buffer, n - nMax);
	}

	return 0;
}
