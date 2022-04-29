#include "battery.h"
#include "I2C.h"


// for apalis imx8 
char *i2c_bus = "/dev/apalis-i2c1";

//for colibri imx6uul
//char *i2c_bus = "/dev/i2c-0";

FILE *fptr;



int write_to_file()
{
        while(1)
        { 
                //opening the file
                fptr = fopen("/tmp/battery_parameters","w");

                //check if file opened successfully
                if(fptr == NULL)
                {
                        printf("ERROR!");
                        return -1;
                }

               float internal_temp = (float)internal_temperature()*0.1-273.15;
                //writing internal temp to file
                if(fprintf(fptr,"Temperature : %0.2f C\n",internal_temp )<0)
                {   
                        printf("error writing temperature to file \n");     
                }

                uint16_t volt = voltage();
                //writing internal temp to file
                if(fprintf(fptr,"Voltage : %d mV\n",volt )<0)
                {   
                        printf("error writing voltage to file \n");     
                }

                uint16_t current_value = average_current();
                //writing current to file
                if(fprintf(fptr,"Average Current : %d mA\n",current_value )<0)
                {   
                        printf("error writing current to file \n");     
                }

                fclose(fptr);
                sleep(2);
        }
        return 0;
}

int main()
{
        // start the i2c bus
        if(i2c_init(i2c_bus)<0)
        {
                return -1;
        }

        //gauge_parameters();
        write_to_file();

        // close the i2c bus
        i2c_close(i2c_bus);
        return 0;
}



