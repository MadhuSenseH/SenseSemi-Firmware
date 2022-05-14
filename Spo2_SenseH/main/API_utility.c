/*
 * API_utility.c
 *
 *  Created on: 01-Aug-2019
 *      Author: chitra
 */

#include <stdbool.h>
#include "API_utility.h"

void MemSet(void *buffer, uint8_t value, uint32_t size)
{
	uint32_t count = 0;
	unsigned char *temp = buffer;

	if(buffer != NULL && size > 0)
	{
		for(count = 0; count < size; count++)
		{
			*temp++ = value;
		}
	}

}

void MemCpy(void *dest, void *src, uint32_t size)
{
   uint32_t len   = 0x00;
   uint8_t  *ptr_dst;
   uint8_t  *ptr_src;

   ptr_dst = dest;
   ptr_src = src;

   // Copy contents of src[] to dest[]
   for (len = 0; len < size; len++)
   {
	   *ptr_dst ++ = *ptr_src ++;
   }
}

char* StrCat(char* destination, const char* source)
{
	int i, j;

		// move to the end of destination string
		for (i = 0; destination[i] != '\0'; i++);

		// i now points to terminating null character in destination

		// Appends characters of source to the destination string
		for (j = 0; source[j] != '\0'; j++)
		{
			destination[i + j] = source[j];
		}

		// null terminate destination string
		destination[i + j] = '\0';

		// destination is returned by standard strcat()
		return destination;

}

void IntergerToString(char str[], uint32_t num)
{
	uint32_t i, rem, len = 0, n;

    n = num;
    for (;n != 0;len++) // number of digits counting
    {
       n /= 10;
    }

    for (i = 0; i < len; i++)  // loop converts the number to string
    {
        rem = num % 10;
        num = num / 10;
        str[len - (i + 1)] = rem + '0';
    }
    str[len] = '\0';
}

void FloatToString(float value,char* dst_str)
{
    float   temp       = 0x00;;
	uint8_t int_part   = 0x00;;
	uint8_t float_part = 0x00;;
	uint8_t rem        = 0xFF;
	uint8_t que        = 0xFF;
	uint8_t index      = 0x00;

	uint8_t num_of_digitis_int_part   = 0x00;
	//uint8_t num_of_digitis_float_part = 0x00;
	char    int_part_str[10];
	char    float_part_str[10];


   int_part   = (uint8_t) value;
   temp       = value - int_part;
   float_part = temp*100;

   que = int_part/10;
   if(int_part)
   {
	   num_of_digitis_int_part++;

	   for(;(que !=0);)
	   {
		   que = que/10;
		   num_of_digitis_int_part++;
	   }
   }


		que   = int_part;
		rem   = int_part;
		MemSet(int_part_str,'\0',sizeof(int_part_str));
		for(index=num_of_digitis_int_part;index>0;index--)
	    {
			rem = que % 10;
			que = que / 10;
			int_part_str[index-1] = rem + '0';
	    }

		que   = float_part;
		rem   = float_part;
		MemSet(float_part_str,'\0',sizeof(float_part_str));
		for(index=2;index>0;index--)
		{
			rem = que % 10;
			que = que / 10;

			if(que<10)
			{
				float_part_str[index-1] = rem + '0';
				index --;
				float_part_str[index-1] = que + '0';
			}
			else
			{
				rem = que % 10;
				que = que / 10;
				float_part_str[index-1] = rem + '0';
			}
		}

    MemCpy(dst_str,int_part_str,num_of_digitis_int_part);

     dst_str += num_of_digitis_int_part;
    *dst_str ++  = '.';
    *dst_str ++  = float_part_str[0];
    *dst_str ++  = float_part_str[1];
    *dst_str ++  = '\0';

}

uint16_t Get_strlen(const char* source)
{
    uint16_t str_len = 0x00;

    for(str_len=0 ;str_len>=0; )
    {
        if((*source ++) != '\0')
        {
        	str_len ++;
        }

        else
        {
        	break;
        }
    }

    return str_len;
}

uint32_t Length_padding_multiple_of_four(uint32_t len)
{

	 if( (len % 4 ) > 0 ) // condition to check the number byte should be in multiples of 4
		    {
		    	if((len % 4) == 1)
				{
		    		len += 3;
				}

		    	else if((len % 4) == 2)
				{
		    		len += 2;
				}

		    	else
		    	{
		    		len += 1;
		    	}

		    }

	 return len;
}

/* void Hex_to_Float(uint8_t hex_val[], float float_val)
 * \brief	converts hex value to float
 *			Provides upto 6 pt precision
 *
 */
void Hex_to_Float(uint8_t hex_val[], float* float_val)
{
	uint32_t num_to_conv = 0;

	num_to_conv = ((hex_val[3] << 24)|(hex_val[2]<<16) |(hex_val[1] << 8) | (hex_val[0]));

	*float_val = *((float*)&num_to_conv);
}
