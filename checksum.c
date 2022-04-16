#include <stdio.h>

int main()
{

    unsigned char sum = 0x00;
    unsigned char array[] = {0x49 , 0xd9 , 0xaf , 0x37 , 0x01 , 0x14 };
    int i=0;
    for(i=0;i<6;i++)
    {
        sum = sum + array[i];
    }
    printf("%x\n",255-sum);

    return 0;
}
// /*
// Check sum calculation
// The check sum is the sum of all 32 data bytes within the current data block, truncated to 8-bits and
// complemented. Example code to calculate the check sum follows:
// pData = a pointer to the data that was changed in the data block.
// nLength = length of the data block.

// */
// unsigned char checksum(unsigned char *pData, unsigned char nLength)
// {
// unsigned char nSum = 0x00;
// unsigned char n;
// for (n = 0; n <nLength; n++)
// nSum += pData[n];
// nSum = 0xFF - nSum;
// return nSum;
// }