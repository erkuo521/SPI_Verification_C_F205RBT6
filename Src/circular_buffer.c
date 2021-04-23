

#include "sensor.h"


#if 0


typedef struct{
    uint8_t * buffer;
    uint16_t head; //reach tail max and set to zero then
    uint8_t tail_max; //always max count in openimu test
    uint8_t real_buf_count; //count of the buffer
    bool full; //if the circular is full or not
}circular_buf_t;


typedef struct{
    acc_raw_t imu_accdata;
	  gyro_raw_t imu_gyrodata;
}imu_size_t;
		


// circular buffer size(bufLen) init according to RAM size of EVB 
circular_buf_t * cb_init(int bufLen)
{
	if(bufLen==0)
	{
	  return NULL;
	}
	circular_buf_t* cbBuffer=malloc(sizeof(circular_buf_t));
	assert(cbBuffer);
	memset(cbBuffer,0,sizeof(circular_buf_t));
	cbBuffer->buflen=bufLen;
	cbBuffer->buf=(char *)malloc(bufLen);
	memset(cbBuffer->buf,0,bufLen);
	return cbBuffer;

}


void circular_buf_insert(circular_buf_t *cb_in_p, uint8_t data_in)
{
	cb_in_p->

	
}

void circular_buf_read()
{
	
	
	
	
}


void circular_buf_reset(cbuf_handle_t cbuf)
{
    assert(cbuf);

    cbuf->head = 0;
    cbuf->tail = 0;
    cbuf->full = false;
}


void circular_buf_free(cbuf_handle_t cbuf)
{
    assert(cbuf);
    free(cbuf);
}


void circular_buf_empty()
{
	
	
}







void circular_buf_full()
{
	
	
	
}

#endif
