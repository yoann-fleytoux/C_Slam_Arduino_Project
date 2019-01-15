/** @file */
//AUTOR: Fernando Pais
//MAIL:  ferpais2508@gmail.com
//DATA: 6/6/2016
// VERSÃO 6.4.0
//
//Alterações: Problema de compatibilidade entre encoder e infravermelho resolvido
//            Odometria actualizada automaticamente
//            Valor da bateria verificado na inicialização
//            Motores movem-se de 0 a 1000 para melhor difrenciação
//

//#include "mbed.h"
//#include "init.h"
//#define _USE_MATH_DEFINES
# define M_PI           3.14159265358979323846  /* pi */
#include <math.h>
#include <string.h>
#include "VCNL40x0.h"
#include "nRF24L01P.h"

void Odometria();
void OdometriaKalman(float* R_cm, float* L_cm);

// classes adicionais
nRF24L01P my_nrf24l01p(PTD2, PTD3, PTD1, PTC13, PTC12, PTA13);
VCNL40x0 VCNL40x0_Device (PTC9, PTC8, VCNL40x0_ADDRESS);
Timeout timeout;

Serial pc(PTE0,PTE1);
I2C i2c(PTC9,PTC8);
I2C i2c1(PTC11,PTC10);

// Variables needed by the lib
unsigned int  ProxiValue=0;
short int prev_R=0;
short int prev_L=0;
long int total_R=0;
long int total_L=0;
long int ticks2d=0;
long int ticks2e=0;
float X=20;
float Y=20;
float AtractX = 0;
float AtractY = 0;
float theta=0;
int sensor_left=0;
int sensor_front=0;
int sensor_right=0;
short int flag=0;
int IRobot=0;
int JRobot=0;

//SAIDAS DIGITAIS (normal)
DigitalOut  q_pha_mot_rig       (PTE4,0);     //Phase Motor Right
DigitalOut  q_sleep_mot_rig     (PTE3,0);     //Nano Sleep Motor Right
DigitalOut  q_pha_mot_lef       (PTA17,0);    //Phase Motor Left
DigitalOut  q_sleep_mot_lef     (PTB11,0);    //Nano Sleep Motor Left
DigitalOut  q_pow_ena_i2c_p     (PTE2,0);     //Power Enable i2c FET P (0- enable 1-disable)
DigitalOut  q_pow_ena_mic_p     (PTA14,0);    //Power enable Micro FET P (0- enable 1-disable)
DigitalOut  q_pow_as5600_n      (PTC6,1);     //AS5600 Power MOSFET N (1- enable 0-disable)
DigitalOut  q_pow_as5600_p      (PTC5,0);     //AS5600 Power MOSFET P (0- enable 1-disable)
DigitalOut  q_pow_spi           (PTC4,0);     //SPI Power MOSFET P (0- enable 1-disable)
DigitalOut  q_ena_mppt          (PTC0,0);     //Enable MPPT Control (0- enable 1-disable)
DigitalOut  q_boost_ps          (PTC7,1);     //Boost Power Save (1- enable 0-disable)
DigitalOut  q_tca9548_reset     (PTC3,1);     //Reset TCA9548 (1- enable 0-disable)
DigitalOut  power_36khz         (PTD0,0);     //Power enable pic12f - 36khz (0- enable 1-disable)


// ********************************************************************
// ********************************************************************
//DEFINIÇÃO DE ENTRADAS E SAIDAS DO ROBOT
//ENTRADAS DIGITAIS (normal input)
DigitalIn   i_enc_dir_rig       (PTB8);     //Encoder Right Direction
DigitalIn   i_enc_dir_lef       (PTB9);     //Encoder Left Direction
DigitalIn   i_micro_sd_det      (PTC16);    //MICRO SD Card Detect
DigitalIn   i_mppt_fail         (PTE5);     //Fail MPPT Signal
DigitalIn   i_usb_volt          (PTB10);    //USB Voltage detect
DigitalIn   i_sup_cap_est       (PTB19);    //Supercap State Charger
DigitalIn   i_li_ion_est        (PTB18);    //Li-ion State Charger


// ********************************************************************
//ENTRADAS DIGITAIS (external interrupt)
InterruptIn i_int_mpu9250       (PTA15);    //Interrupt MPU9250
InterruptIn i_int_isl29125      (PTA16);    //Interrupt ISL29125 Color S.
InterruptIn i_mic_f_l           (PTD7);     //Interrupt Comp Micro F L
InterruptIn i_mic_f_r           (PTD6);     //Interrupt Comp Micro F R
InterruptIn i_mic_r_c           (PTD5);     //Interrupt Comp Micro R C


// ********************************************************************
//ENTRADAS ANALOGICAS
AnalogIn    a_enc_rig           (PTC2);     //Encoder Left Output_AS_MR
AnalogIn    a_enc_lef           (PTC1);     //Encoder Right Output_AS_ML
AnalogIn    a_mic_f_l           (PTB0);     //Analog microphone F L
AnalogIn    a_mic_f_r           (PTB1);     //Analog microphone F R
AnalogIn    a_mic_r_c           (PTB2);     //Analog microphone R C
AnalogIn    a_temp_bat          (PTB3);     //Temperature Battery


// ********************************************************************

//PWM OR DIGITAL OUTPUT NORMAL
//DigitalOut    q_led_whi         (PTE29);    //Led white pwm
DigitalOut    q_led_red_fro     (PTA4);     //Led Red Front
DigitalOut    q_led_gre_fro     (PTA5);     //Led Green Front
DigitalOut    q_led_blu_fro     (PTA12);    //Led Blue Front
DigitalOut    q_led_red_rea     (PTD4);     //Led Red Rear
DigitalOut    q_led_gre_rea     (PTA1);     //Led Green Rear
DigitalOut    q_led_blu_rea     (PTA2);     //Led Blue Rear


//SAIDAS DIGITAIS (pwm)
PwmOut      pwm_mot_rig         (PTE20);    //PWM Enable Motor Right
PwmOut      pwm_mot_lef         (PTE31);    //PWM Enable Motor Left
PwmOut      pwm_buzzer          (PTE21);    //Buzzer PWM
PwmOut      pwm_led_whi         (PTE29);    //Led white pwm

// ********************************************************************
//SAIDAS ANALOGICAS
AnalogOut   dac_comp_mic        (PTE30);        //Dac_Comparator MIC


/* Powers up all the VCNL4020. */
void init_Infrared()
{
    VCNL40x0_Device.SetCurrent (20);     // Set current to 200mA
}

/**
 * Selects the wich infrared to comunicate.
 *
 * @param ch - Infrared to read (1..5)
 */
void tca9548_select_ch(char ch)
{
    char ch_f[1];
    char addr=0xE0;

    if(ch==0)
        ch_f[0]=1;

    if(ch>=1)
        ch_f[0]=1<<ch;

    i2c.start();
    i2c.write(addr,ch_f,1);
    i2c.stop();
}


/**
 * Get ADC value of the chosen infrared.
 *
 * @param ch - Infrared to read (1..5)
 *
 * Note: for the values of ch it reads (0-right, ... ,4-left, 5-back)
 */
long int read_Infrared(char ch) // 0-direita 4-esquerda 5-tras
{
    tca9548_select_ch(ch);
    VCNL40x0_Device.ReadProxiOnDemand (&ProxiValue);    // read prox value on demand

    return ProxiValue;
}

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////     MOTOR       ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

// Calculo do Duty tem de ser revisto, o motor aguenta 6 V e o max definido aqui ronda os 4.2 V
// consultar pag 39 e 95

/**
 * Sets speed and direction of the left motor.
 *
 * @param Dir - Direction of movement, 0 for back, or 1 for fron
 * @param Speed - Percentage of speed of the motor (1..100)
 *
 * Note: Because of differences in the motors they need to be calibrated, test the robot going front and back
 *  at different speeds and see if it makes a straigth line
 */
void leftMotor(short int Dir,short int Speed)
{
    float Duty;

    if(Dir==1) {
        q_pha_mot_lef=0;            //Andar em frente
        if(Speed>1000)                   //limite de segurança
            Speed=1000;
        if(Speed>0) {
            Duty=Speed*.082 +35;         // 35 = minimo para o motor rodar
            q_sleep_mot_lef=1;          //Nano Sleep Motor Left
            pwm_mot_lef.pulsewidth_us(Duty*5);
        } else {
            q_sleep_mot_lef=0;
        }
    }
    if(Dir==0) {
        q_pha_mot_lef=1;            //Andar para tras

        if(Speed>1000)                   //limite de segurança
            Speed=1000;
        if(Speed>0) {
            Duty=Speed*.082 +35;         // 35 = minimo para o motor rodar
            q_sleep_mot_lef=1;          //Nano Sleep Motor Left
            pwm_mot_lef.pulsewidth_us(Duty*5);
        } else {
            q_sleep_mot_lef=0;
        }
    }
}


/**
 * Sets speed and direction of the right motor.
 *
 * @param Dir - Direction of movement, 0 for back, or 1 for fron
 * @param Speed - Percentage of speed of the motor (1..100)
 *
 * Note: Because of differences in the motors they need to be calibrated, test the robot going front and back
 *  at different speeds and see if it makes a straigth line
 */
void rightMotor(short int Dir,short int Speed)
{
    float Duty;

    if(Dir==1) {
        q_pha_mot_rig=0;            //Andar em frente

        if(Speed>1000)                   //limite de segurança
            Speed=1000;
        if(Speed>0) {
            Duty=Speed*.082 +35;         // 35 = minimo para o motor rodar
            q_sleep_mot_rig=1;          //Nano Sleep Motor Right
            pwm_mot_rig.pulsewidth_us(Duty*5);
        } else {
            q_sleep_mot_rig=0;
        }
    }
    if(Dir==0) {
        q_pha_mot_rig=1;            //Andar para tras


        if(Speed>1000)                   //limite de segurança
            Speed=1000;
        if(Speed>0) {
            Duty=Speed*.082 +35;         // 35 = minimo para o motor rodar
            q_sleep_mot_rig=1;          //Nano Sleep Motor Right
            pwm_mot_rig.pulsewidth_us(Duty*5);
        } else {
            q_sleep_mot_rig=0;
        }
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////     ENCODER     ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Reads Position of left magnetic encoder.
 *
 * @return The absolute position of the left wheel encoder (0..4095)
 */
long int read_L_encoder()
{

    char data_r_2[5];

    i2c.start();
    i2c.write(0x6C);
    i2c.write(0x0C);
    i2c.read(0x6D,data_r_2,4,0);
    i2c.stop();

    short int val1=data_r_2[0];
    short int val2=data_r_2[1];
    val1=(val1&0xf)*256;
    long int final=(val2+val1);

    return  final;
}


/**
 * Reads Position of right magnetic encoder.
 *
 * @return The absolute position of the right wheel encoder (0..4095)
 */
long int read_R_encoder()
{

    char data_r_2[5];

    i2c1.start();
    i2c1.write(0x6C);
    i2c1.write(0x0C);
    i2c1.read(0x6D,data_r_2,4,0);
    i2c1.stop();

    short int val1=data_r_2[0];
    short int val2=data_r_2[1];
    val1=(val1&0xf)*256;
    long int final=(val2+val1);

    return  final;
}


/**
 * Calculates and returns the value of the  right "incremental" encoder.
 *
 * @return The value of "tics" of the right encoder since it was initialized
 */
long int incremental_R_encoder()
{
    short int next_R=read_R_encoder(); // Reads curent value of the encoder
    short int dif=next_R-prev_R;       // Calculates the diference from last reading

    if(dif>3000) {                     // Going back and pass zero
        total_R=total_R-4096+dif;
    }
    if(dif<3000&&dif>0) {              // Going front
        total_R=total_R+dif;
    }
    if(dif<-3000) {                    // Going front and pass zero
        total_R=total_R+4096+dif;
    }
    if(dif>-3000&&dif<0) {             // going back
        total_R=total_R+dif;
    }
    prev_R=next_R;                     // Sets last reading for next iteration

    return  total_R;
}


/**
 * Calculates and returns the value of the  left "incremental" encoder.
 *
 * @return The value of "tics" of the left encoder since it was initialized
 */
long int incremental_L_encoder()
{
    short int next_L=read_L_encoder(); // Reads curent value of the encoder
    short int dif=-next_L+prev_L;      // Calculates the diference from last reading

    if(dif>3000) {                     // Going back and pass zero
        total_L=total_L-4096+dif;
    }
    if(dif<3000&&dif>0) {              // Going front
        total_L=total_L+dif;
    }
    if(dif<-3000) {                    // Going front and pass zero
        total_L=total_L+4096+dif;
    }
    if(dif>-3000&&dif<0) {             // going back
        total_L=total_L+dif;
    }
    prev_L=next_L;                     // Sets last reading for next iteration

    return  total_L;
}


/**
 * Calculate the value of both encoder "incremental" every 10 ms.
 */
void timer_event()  //10ms interrupt
{
    timeout.attach(&timer_event,0.01);
    if(flag==0) {
        incremental_R_encoder();
        incremental_L_encoder();
    }
    Odometria();

    return;
}


/**
 * Set the initial position for the "incremental" enconder and "starts" them.
 */
void initEncoders()
{
    prev_R=read_R_encoder();
    prev_L=read_L_encoder();
    timeout.attach(&timer_event,0.01);
}


/**
 * Returns to the user the value of the right "incremental" encoder.
 *
 * @return The value of "tics" of the right encoder since it was initialized
 */
long int R_encoder()
{
    wait(0.0001);

    return total_R;
}

/**
 * Returns to the user the value of the right "incremental" encoder.
 *
 * @return The value of "tics" of the right encoder since it was initialized
 */
long int L_encoder()
{
    wait(0.0001);

    return total_L;
}


///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////     BATTERY     ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Reads adc of the battery.
 *
 * @param addr - Address to read
 * @return The voltage of the batery
 */
long int read16_mcp3424(char addr)
{
    char data[4];
    i2c1.start();
    i2c1.read(addr,data,3);
    i2c1.stop();

    return(((data[0]&127)*256)+data[1]);
}

/**
 * Reads adc of the battery.
 *
 * @param n_bits - Resolution of measure
 * @param ch - Chose value to read, if voltage or current of solar or batery
 * @param gain -
 * @param addr - Address to write to
 */
void write_mcp3424(int n_bits, int  ch, int gain, char  addr)  //chanel 1-4    write -> 0xD0
{

    int chanel_end=(ch-1)<<5; //shift left
    char n_bits_end=0;

    if(n_bits==12) {
        n_bits_end=0;
    } else if(n_bits==14) {
        n_bits_end=1;
    } else if(n_bits==16) {
        n_bits_end=2;
    } else {
        n_bits_end=3;
    }
    n_bits_end=n_bits_end<<2; //shift left

    char data[1];
    data[0]= (char)chanel_end | (char)n_bits_end | (char)(gain-1) | 128;
    i2c1.start();
    i2c1.write(addr,data,1);
    i2c1.stop();
}


/**
 * Reads adc of the battery.
 *
 * @return The voltage of the batery
 */
float value_of_Batery()
{
    float   R1=75000.0;
    float   R2=39200.0;
    float   R3=178000.0;
    float   Gain=1.0;
    write_mcp3424(16,3,1,0xd8);
    float cha3_v2=read16_mcp3424(0xd9); //read  voltage
    float Vin_v_battery=(((cha3_v2*2.048)/32767))/Gain;
    float Vin_b_v_battery=(-((-Vin_v_battery)*(R1*R2 + R1*R3 + R2*R3))/(R1*R2));
    Vin_b_v_battery=(Vin_b_v_battery-0.0)*1.00268;

    return Vin_b_v_battery;
}

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////     RF COMUNICATION     ///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////


/**
 * Initializes the NRF24 module for comunication.
 *
 * Note: if the module is broken or badly connected this init will cause the code to stop,
 *  if all these messages don't appear thats the case
 */
void config_init_nrf()
{
    my_nrf24l01p.powerUp(); // powers module
    my_nrf24l01p.setRfFrequency (2400); // channel 0 (2400-0 ... 2516-116)
    my_nrf24l01p.setTransferSize(10);   // number of bytes to be transfer
    my_nrf24l01p.setCrcWidth(8);
    my_nrf24l01p.enableAutoAcknowledge(NRF24L01P_PIPE_P0); // pipe where data transfer occurs (0..6)
    pc.printf( "nRF24L01+ Frequency    : %d MHz\r\n",  my_nrf24l01p.getRfFrequency() );
    pc.printf( "nRF24L01+ Data Rate    : %d kbps\r\n", my_nrf24l01p.getAirDataRate() );
    pc.printf( "nRF24L01+ TX Address   : 0x%010llX\r\n", my_nrf24l01p.getTxAddress() );
    pc.printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", my_nrf24l01p.getRxAddress() );
    pc.printf( "nRF24L01+ CrC Width    : %d CrC\r\n", my_nrf24l01p.getCrcWidth() );
    pc.printf( "nRF24L01+ TransferSize : %d Paket Size\r\n", my_nrf24l01p.getTransferSize () );
    my_nrf24l01p.setReceiveMode();
    my_nrf24l01p.enable();
    pc.printf( "Setup complete, Starting While loop\r\n");
}


/**
 * Receives a number from the Arduino.
 *
 * @return The value send by the arduino
 */
double receiveValue(void)
{
    char temp[4];
    double Val;
    bool ok=0;
    my_nrf24l01p.setTransferSize(4);
    my_nrf24l01p.setReceiveMode();
    my_nrf24l01p.enable();
    do {
        if(my_nrf24l01p.readable(NRF24L01P_PIPE_P0)) {
            ok = my_nrf24l01p.read( NRF24L01P_PIPE_P0,temp, 4);
        }
    } while(ok==0);

    //transformation of temp to convert to original value
    if(temp[0]==0) // if first elemente is 0 then its negative
        Val = double(-(int(temp[1])+int(temp[2])*255+int(temp[3])*255*255));
    else // else its positive
        Val = double(int(temp[1])+int(temp[2])*255+int(temp[3])*255*255);

    return Val;
}


/**
 * Sends a number to the Arduino
 *
 * @param Value - number to be sent to the Arduino
 */
void sendValue(long int Value)
{
    bool ok=0;  // comunication sucess, o if failed 1 if sucessfull
    // double math=Value/65025; // temporary variable save results
    int zero=1;  // 1 byte, ( - ) if 0 ( + ) if 1
    int one=0;   // 2 byte (0..255), multiplied by 1
    int two=0;   // 3 byte (0..255), multiplied by 255
    int three=0; // 4 byte (0..255), multiplied by 255*255

//transformation of Value to send correctly through pipe
    if (Value<0) {
        zero=0;
        Value=abs(Value);
    }
    //  Value=abs(Value);

    double math=Value/65025; // temporary variable save results

    if(math<1) {
        math=Value/255;
        if(math<1) {
            two=0;
            one=Value;
        } else {
            two=(int)math;
            one=Value % 255;
        }
    } else {
        three=(int)math;
        math=Value/255;
        if(math<1) {
            two=0;
            one=Value;
        } else {
            two=(int)math;
            one=Value % 255;
        }
    }
    char temp[4]= {(int)zero,(int)one,(int)two,(int)three};

    // Apagar depois de testar mais vezes
    // pc.printf("1 inidice...%i...\r", temp[0]);
    // pc.printf("2 inidice...%i...\r", temp[1]);
    // pc.printf("3 inidice...%i...\r", temp[2]);
    // pc.printf("4 inidice...%i...\r", temp[3]);

    my_nrf24l01p.setTransferSize(4);
    my_nrf24l01p.setTransmitMode();
    my_nrf24l01p.enable();
    do {
        ok = my_nrf24l01p.write( NRF24L01P_PIPE_P0,temp, 4);
        if(ok==1)
            pc.printf("Done.....%i...\r", Value);
        else {
            pc.printf("Failed\r");
            wait(1);
        }
    } while(ok==0);
}

/**
 *  Sends matrix to arduino.
 *
 * @param matrix - Matrix of numbers to send [0..255]
 * @param row - Number of rows
 * @param column - Number of columns
 */
void sendMatrix(int (*matrix)[18],int row , int column)
{
    short ok=0;
    short int i =0;
    short int j=0;
    short int byte=0;
    int members=column*row;
    char message[32]= {0};
    pc.printf("J ...%d... \r",members);

    my_nrf24l01p.setTransferSize(32);
    my_nrf24l01p.setTransmitMode();
    my_nrf24l01p.enable();

    do {
        int* point = matrix[j];

        do {
            message[byte]= point[i];
            if (byte==31 || (i+1)*(j+1)==members) {

                do {
                    ok = my_nrf24l01p.write( NRF24L01P_PIPE_P0,message, 32);
                    if(ok==0)
                        wait(1);

                } while(ok==0);

                byte=-1;
            }

            byte++;
            i++;

        } while(i<column);

        i=0;
        j++;
    } while(j<row);

}

///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////      Sonar     ////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
//      Commands of operation with ultrasonic module

//    WRITE OPTION:
//        ENABLE DC DC CONVERTER          - 0x0C;
//        DISABLE DC DC CONVERTER         - 0x0B;
//        START MEASURE LEFT SENSOR       - 0x0A;
//        START MEASURE FRONT SENSOR      - 0x09;
//        START MEASURE RIGHT SENSOR      - 0x08;
//        SENSORS ALWAYS MEASURE ON       - 0x07;
//        SENSORS ALWAYS MEASURE OFF      - 0x06;

// READ OPTION:
//        GET MEASURE OF LEFT SENSOR          - 0x05;
//        GET MEASURE OF FRONT SENSOR         - 0x04;
//        GET MEASURE OF IGHT SENSOR          - 0x03;
//        GET STATUS SENSORS ALWAYS MEASURE   - 0x02;
//        GET STATUS DC DC CONVERTER          - 0x01;

void enable_dc_dc_boost()
{
    char data[1];
    data[0]= 0x0C;
    wait_ms(1);
    i2c1.start();
    i2c1.write(0x30,data,1);
    i2c1.stop();
    i2c1.start();
    i2c1.write(0x30,data,1);
    i2c1.stop();
}


void disable_dc_dc_boost()
{
    char data[1];
    data[0]= 0x0B;
    wait_ms(1);
    i2c1.start();
    i2c1.write(0x30,data,1);
    i2c1.stop();
}


void start_read_left_sensor()
{
    char data[1];
    data[0]= 0x0A;
    wait_ms(1);
    i2c1.start();
    i2c1.write(0x30,data,1);
    i2c1.stop();
}


void start_read_front_sensor()
{
    char data[1];
    data[0]= 0x09;
    wait_ms(1);
    i2c1.start();
    i2c1.write(0x30,data,1);
    i2c1.stop();
}


void start_read_right_sensor()
{
    char data[1];
    data[0]= 0x08;
    wait_ms(1);
    i2c1.start();
    i2c1.write(0x30,data,1);
    i2c1.stop();
}


void measure_always_on()  // left, front, right
{
    char data[1];
    data[0]= 0x07;
    wait_ms(1);
    i2c1.start();
    i2c1.write(0x30,data,1);
    i2c1.stop();
}


void measure_always_off()
{
    char data[1];
    data[0]= 0x06;
    wait_ms(1);
    i2c1.start();
    i2c1.write(0x30,data,1);
    i2c1.stop();
}

/**
 * Returns left sensor value
 */
static unsigned int get_distance_left_sensor()
{

    static char data_r[3];
    static unsigned int aux;
    flag=1;

    data_r[0]= 0x05;
    wait_ms(1);
    i2c1.start();
    i2c1.write(0x30,data_r,1);
    i2c1.stop();
    wait_ms(10);
    i2c1.start();
    i2c1.read(0x31,data_r,2,0);
    i2c1.stop();

    aux=(data_r[0]*256)+data_r[1];
    flag=0;
    return aux;
    // sensor_left=aux;
    // pc.printf("\nDistance Left Sensor: %u mm",aux); //0 - 2500mm

}


/**
 * Returns front sensor value
 */
static unsigned int get_distance_front_sensor()
{

    static char data_r[3];
    static unsigned int aux;
    flag=1;
    data_r[0]= 0x04;
    wait_ms(1);
    i2c1.start();
    i2c1.write(0x30,data_r,1);
    i2c1.stop();
    wait_ms(10);
    i2c1.start();
    i2c1.read(0x31,data_r,2,0);
    i2c1.stop();

    aux=(data_r[0]*256)+data_r[1];
    flag=0;
    return aux;
    // sensor_front=aux;
    // pc.printf("\nDistance Front Sensor: %u mm",aux); //0 - 2500mm

}


/**
 * Returns right sensor value
 */
static unsigned int get_distance_right_sensor()
{

    static char data_r[3];
    static unsigned int aux;
    flag=1;

    data_r[0]= 0x03;
    wait_ms(1);
    i2c1.start();
    i2c1.write(0x30,data_r,1);
    i2c1.stop();
    wait_ms(10);
    i2c1.start();
    i2c1.read(0x31,data_r,2,0);
    i2c1.stop();

    aux=(data_r[0]*256)+data_r[1];
    flag=0;
    return aux;
    // sensor_right=aux;
    // pc.printf("\nDistance Right Sensor: %u \r",aux); //0 - 2500mm

}


void get_status_always_measure()
{

    static char data_r[3];
    static unsigned int aux;

    data_r[0]= 0x02;
    wait_ms(1);
    i2c1.start();
    i2c1.write(0x30,data_r,1);
    i2c1.stop();
    wait_ms(10);
    i2c1.start();
    i2c1.read(0x31,data_r,2,0);
    i2c1.stop();

    aux=data_r[0];
    pc.printf("\nStatus of read always on/off: %u ",aux); //0 (off) - 1 (on)

}


void get_status_dcdc_converter()
{

    static char data_r[3];
    static unsigned int aux;

    data_r[0]= 0x01;
    wait_ms(1);
    i2c1.start();
    i2c1.write(0x30,data_r,1);
    i2c1.stop();
    wait_ms(10);
    i2c1.start();
    i2c1.read(0x31,data_r,2,0);
    i2c1.stop();

    aux=data_r[0];
    pc.printf("\nStatus of DC/DC Converter: %u ",aux); //0 (off) - 1 (on)

}


///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////      MISC.      ////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////


/**
 * Initializes the necessary robot pins
 */
void init_robot_pins()
{

    //SAIDAS DIGITAIS (normal)
    //q_pha_mot_rig=0;            //Phase Motor Right
    //q_sleep_mot_rig=0;          //Nano Sleep Motor Right
    //q_pha_mot_lef=0;            //Phase Motor Left
    //q_sleep_mot_lef=0;          //Nano Sleep Motor Left
    //q_pow_ena_i2c_p=0;          //Power Enable i2c FET P
    //q_pow_ena_mic_p=0;          //Power enable Micro FET P
    //q_pow_as5600_n=1;           //AS5600 Power MOSFET N
    //q_pow_as5600_p=0;           //AS5600 Power MOSFET P
    //q_pow_spi=0;                //SPI Power MOSFET P
    //q_ena_mppt=0;               //Enable MPPT Control
    //q_boost_ps=1;               //Boost Power Save
    //q_tca9548_reset=1;          //Reset TCA9548

    //SAIDAS DIGITAIS (normal)
    q_pha_mot_rig=0;            //Phase Motor Right
    q_sleep_mot_rig=0;          //Nano Sleep Motor Right
    q_pha_mot_lef=0;            //Phase Motor Left
    q_sleep_mot_lef=0;          //Nano Sleep Motor Left

    q_pow_ena_i2c_p=0;         //Power Enable i2c FET P
    q_pow_ena_mic_p=0;          //Power enable Micro FET P
    q_pow_as5600_p=0;           //AS5600 Power MOSFET P
    // q_pow_spi=0;                //SPI Power MOSFET P
    q_pow_as5600_n=1;           //AS5600 Power MOSFET N


    q_ena_mppt=0;               //Enable MPPT Control
    q_boost_ps=1;               //Boost Power Save
    q_tca9548_reset=1;          //Reset TCA9548

    //Leds caso seja saida digital:
    q_led_red_fro=1;          //Led Red Front (led off)
    q_led_gre_fro=1;          //Led Green Front (led off)
    q_led_blu_fro=1;          //Led Blue Front (led off)
    q_led_red_rea=1;          //Led Red Rear (led off)
    q_led_gre_rea=1;          //Led Green Rear (led off)
    q_led_blu_rea=1;          //Led Blue Rear (led off)r


//********************************************************************
    //SAIDAS DIGITAIS (pwm)
    //PWM Enable Motor Right
    pwm_mot_rig.period_us(500);
    pwm_mot_rig.pulsewidth_us(0);

    //PWM Enable Motor Left
    pwm_mot_lef.period_us(500);
    pwm_mot_lef.pulsewidth_us(0);

    //Buzzer PWM
    pwm_buzzer.period_us(500);
    pwm_buzzer.pulsewidth_us(0);

    //LED white
    pwm_led_whi.period_us(500);
    pwm_led_whi.pulsewidth_us(0);

}


/**
 * Initializes all the pins and all the modules necessary
 */
void initRobot(void)
{
    init_robot_pins();
    enable_dc_dc_boost();
    init_Infrared();
    initEncoders();
    config_init_nrf();
    enable_dc_dc_boost();
    wait_ms(100); //wait for read wait(>=150ms);
    measure_always_on();
    float value = value_of_Batery();
    pc.printf("Initialization Successful \n\r");
    pc.printf("Battery level: %f \n\r",value);
    if(value < 3.0) {
        pc.printf(" WARNING: BATTERY NEEDS CHARGING ");
    }

    // float level = value_of_Batery();
    // sendValue(int(level*100));

}


////////////////////////////////////////////////////

/**
 * Updates the position and orientation of the robot based on the data from the encoders
 *
 * Note: Needs to be calibrated for each robot, in this case the radius of the whells is 3.55
 * and the distance between them is 7.4
 */
void Odometria()
{
    long int ticks1d=R_encoder();
    long int ticks1e=L_encoder();
    //pc.printf("\r\nticks1d:%f",ticks1d);
    //pc.printf("\r\nticks1e:%f",ticks1d);
    long int D_ticks=ticks1d - ticks2d;
    long int E_ticks=ticks1e - ticks2e;
    //pc.printf("\r\nD_ticks:%f",D_ticks);
    //pc.printf("\r\nE_ticks:%f",E_ticks);
    ticks2d=ticks1d;
    ticks2e=ticks1e;

    float D_cm= (float)D_ticks*((3.25*3.1415)/4096);
    float L_cm= (float)E_ticks*((3.25*3.1415)/4096);
    //pc.printf("\r\nD_cm:%f",D_cm);
    //pc.printf("\r\nL_cm:%f",L_cm);
    float CM=(D_cm + L_cm)/2;

    //pc.printf("\r\nCM:%f",CM);
    theta +=(D_cm - L_cm)/7.18;
    theta = atan2(sin(theta), cos(theta));

    // meter entre 0
    X += CM*cos(theta);
    Y += CM*sin(theta);

}

void OdometriaKalman(float* R_cm, float* L_cm)
{
    long int ticks1d=R_encoder();
    long int ticks1e=L_encoder();

    long int D_ticks=ticks1d - ticks2d;
    long int E_ticks=ticks1e - ticks2e;

    ticks2d=ticks1d;
    ticks2e=ticks1e;

    float D_cm= (float)D_ticks*((3.25*3.1415)/4096);
    float E_cm= (float)E_ticks*((3.25*3.1415)/4096);

    float CM=(D_cm + E_cm)/2;

    theta +=(D_cm - E_cm)/7.18;

    theta = atan2(sin(theta), cos(theta));

    // meter entre 0

    X += CM*cos(theta);
    Y += CM*sin(theta);

    *R_cm= D_cm;
    *L_cm= E_cm;
}