//Bibliotecas gerais:

//Relativas ao BMP280
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <math.h>

//Relativas ao MPU6050
//Pinagem do Toradex

//Declarar portas

//Relativa ao MPU6050
int read_word_2c(int addr)
{
int val;
val = wiringToradexI2CReadReg8(fd, addr);
val = val << 8;
val += wiringToradexI2CReadReg8(fd, addr+1);
if (val >= 0x8000)
val = -(65536 - val);
 
return val;
}



//Declaração de variáveis
double pressao = 0;
double altitude = 0;
int fd;
double aceleracao_X, aceleracao_Y, aceleracao_Z;
double giro_X, giro_Y, giro_Z;
double aceleracao_X_escalar, aceleracao_Y_escalar, aceleracao_Z_escalar;
double gyroX_scaled, gyroY_scaled, gyroZ_scaled;
int indice = 40;

//Funcoes
//MPU6050
double dist(double a, double b)
{
return sqrt((a*a) + (b*b));
}
 
double Inclinacao_YZ(double x, double y, double z)
{
double radians;
radians = atan2(x, dist(y, z));
return (radians * (180.0 / M_PI));
}
 
double Inclinacao_XZ(double x, double y, double z)
{
double radians;
radians = atan2(y, dist(x, z));
return (radians * (180.0 / M_PI));
}

//Controle
  float ax = 0.1; // ângulo de trajetõria desejado em torno do eixo x do foguete(rad)
  float ay = 0.1; // ângulo de trajetõria desejado em torno do eixo y do foguete(rad)
  float a2 = ay*cos(.5236)+ax*cos(2.0944); // transpond Ângulos para vetores perpendiculares a aleta 2
  float a3 =ay*cos(2.618)+ax*cos(4.1888); // transpond Ângulos para vetores perpendiculares a aleta 3

  typedef struct {

    
    float Kp = 1;// constante proporcional, definida experimentalmente.
    float Ki = 1 ;// constante integrativa, definida experimentalmente.
    float Kd = 1;// constante deriviativa


    float tau = 10; //constante para filtragem de frequencias altas no termo derivativo .

    //limites para saída dos sensores 
    float limMin = 256;
    float limMax= -256;
    
    //limites do integrador 
    float limMinInt = 50;
    float limMaxInt = 50;

    
    float T = 0.2;// tempo entre amostras
    // fatores utilizados para o calulo do controle
    float erroanterior1;
    float erroanterior2;
    float erroanterior3;
    float inte1;
    float inte2;
    float inte3;
    float deri1;
    float deri2;
    float deri3;		
    float out1;
    float out2;
    float out3;
    float angxanterior;
    float angyanterior;
    float ang2anterior;
    float ang3anterior;

  } PID;


 void iniciar(PID*pid){
    pid->inte1 = 0.0f;
    pid->inte2 = 0.0f;
    pid->inte3 = 0.0f;
    pid->erroanterior1  = 0.0f;
    pid->erroanterior2  = 0.0f;
    pid->erroanterior2  = 0.0f;
    pid->deri1  = 0.0f;
    pid->deri2  = 0.0f;
    pid->deri3  = 0.0f;
    pid->angxanterior = 0.0f;
    pid->angyanterior = 0.0f;
    pid->ang2anterior = 0.0f;
    pid->ang3anterior = 0.0f;

    pid->out1 = 0.0f;
    pid->out2 = 0.0f;
    pid->out3 = 0.0f;}
    
  float controleservo1(PID*pid, ax, axatual){
    
    float erro1 = ax - axatual; // calculando o erro


      float prop = pid->Kp * erro1; // calculando o termo proporcional 

      pid->inte1= pid->inte1 + 0.5f * pid->Ki * pid->T * (erro1 + pid->erroanterior1); //calculando o termo integral


      if (pid->inte1> pid->limMaxInt) { // limitando o termo integral

          pid->inte1 = pid->limMaxInt;

      } else if (pid->inte1 < pid->limMinInt) {

          pid->inte1 = pid->limMinInt;

      }



      
      pid->deri1 = -(2.0f * pid->Kd * (axatual - pid->angxanterior)	// calculando termo diferencial com filtro passa baixa 
                          + (2.0f * pid->tau - pid->T) * pid->deri1)
                          / (2.0f * pid->tau + pid->T);



      pid->out1 = prop + pid->inte1+ pid->deri1; // calculo da saída para o servo

      if (pid->out1 > pid->limMax) {

          pid->out1 = pid->limMax;

      } else if (pid->ou1t < pid->limMin) {

          pid->out1 = pid->limMin;

      }

    
      pid->erroanterior1      = erro1;
      pid->angxanterior = axatual;

      return pid->out1;

    }
    float controleservo2(PID*pid, a2, a2atual){
    
      float erro2 = a2 - a2atual; // calculando o erro
    
    
      float prop2 = pid->Kp * erro2; // calculando o termo proporcional 
    
      pid->inte2= pid->inte2 + 0.5f * pid->Ki * pid->T * (erro2 + pid->erroanterior2); //calculando o termo integral
    
    
      if (pid->inte2> pid->limMaxInt) { // limitando o termo integral
    
        pid->inte2 = pid->limMaxInt;
    
      } else if (pid->inte2 < pid->limMinInt) {
    
        pid->inte2 = pid->limMinInt;
    
      }
    
    
      pid->deri2 = -(2.0f * pid->Kd * (a2atual - pid->ang2anterior)	// calculando termo diferencial com filtro passa baixa 
                + (2.0f * pid->tau - pid->T) * pid->deri2)
                / (2.0f * pid->tau + pid->T);
    
    
    
      pid->out2 = prop2 + pid->inte2+ pid->deri2; // calculo da saída para o servo
    
      if (pid->out2 > pid->limMax) {
    
        pid->out2 = pid->limMax;
    
      } else if (pid->out2 < pid->limMin) {
    
        pid->out2 = pid->limMin;
    
      }
    
      
      pid->erroanterior2      = erro2;
      pid->ang2anterior = a2atual;
    
      return pid->out2;
    
    
      
      }

      float controleservo3(PID*pid, a3, a3atual){
    
        float erro3 = a3 - a3atual; // calculando o erro
      
      
        float prop3 = pid->Kp * erro3; // calculando o termo proporcional 
      
        pid->inte3= pid->inte3 + 0.5f * pid->Ki * pid->T * (erro3 + pid->erroanterior3); //calculando o termo integral
      
      
        if (pid->inte3> pid->limMaxInt) { // limitando o termo integral
      
          pid->inte3 = pid->limMaxInt;
      
        } else if (pid->inte3 < pid->limMinInt) {
      
          pid->inte3 = pid->limMinInt;
      
        }
      
          
        pid->deri3 = -(2.0f * pid->Kd * (a3atual - pid->ang3anterior)	// calculando termo diferencial com filtro passa baixa 
                  + (2.0f * pid->tau - pid->T) * pid->deri3)
                  / (2.0f * pid->tau + pid->T);
      
      
      
        pid->out3 = prop3 + pid->inte3+ pid->deri3; // calculo da saída para o servo
      
        if (pid->out3 > pid->limMax) {
      
          pid->out3 = pid->limMax;
      
        } else if (pid->out3 < pid->limMin) {
      
          pid->out3 = pid->limMin;
      
        }
      
        
        pid->erroanterior3      = erro3;
        pid->ang3anterior = a3atual;
      
        return pid->out3;
      
      
        
        }


int main() { 
	
while (1)
{

  //BMP280
  //Lê 24 Bytes de dados do endereço 0x88 -> checar se é essa porta
	char reg[1] = {0x88};
	write(arquivo, reg, 1);
	char data[24] = {0};
	if(read(arquivo, data, 24) != 24)
	{
		printf("Erro: Erro de entrada ou saida, 24 bytes\n");
		exit(1);
	}
	// Converte os dados obtidos pelo BMP280 em pressão
  //Coeficientes de pressão
	int dig_P1 = data[7] * 256 + data[6];
	int dig_P2  = data[9] * 256 + data[8];
	if(dig_P2 > 32767)
	{
		dig_P2 -= 65536;
	}
	int dig_P3 = data[11]* 256 + data[10];
	if(dig_P3 > 32767)
	{
		dig_P3 -= 65536;
	}
	int dig_P4 = data[13]* 256 + data[12];
	if(dig_P4 > 32767)
	{
		dig_P4 -= 65536;
	}
	int dig_P5 = data[15]* 256 + data[14];
	if(dig_P5 > 32767)
	{
		dig_P5 -= 65536;
	}
	int dig_P6 = data[17]* 256 + data[16];
	if(dig_P6 > 32767)
	{
		dig_P6 -= 65536;
	}
	int dig_P7 = data[19]* 256 + data[18];
	if(dig_P7 > 32767)
	{
		dig_P7 -= 65536;
	}
	int dig_P8 = data[21]* 256 + data[20];
	if(dig_P8 > 32767)
	{
		dig_P8 -= 65536;
	}
	int dig_P9 = data[23]* 256 + data[22];
	if(dig_P9 > 32767)
	{
		dig_P9 -= 65536;
	}
 
	// Seleciona o registrador controlador de medidas (0xF4)
	// Modo normal, temperatura e pressão over sampling rate = 1(0x27)
	char config[2] = {0};
	config[0] = 0xF4;
	config[1] = 0x27;
	write(arquivo, config, 2);
 
	// Seleciona a configuração de registrador (0xF5)
	config[0] = 0xF5;
	config[1] = 0xA0;
	write(arquivo, config, 2);

	// Lê 8 bytes de dados do registrador (0xF7)
	// pressão msb1, pressão msb, pressão lsb
	reg[0] = 0xF7;
	write(arquivo, reg, 1);
	if(read(arquivo), data, 8) != 8)
	{
		printf("Erro: falha na entrada ou saida, 8 bytes\n");
		exit(1);
	}
 
	// Convert pressão para 19 bits
	long adc_p = (((long)data[0] * 65536) + ((long)data[1] * 256) + (long)(data[2] & 0xF0)) / 16;
	 
	// Calcula a pressao
	var1 = ((double)t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
	var1 = (((double) dig_P3) * var1 * var1 / 524288.0 + ((double) dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
	double p = 1048576.0 - (double)adc_p;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double) dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double) dig_P8) / 32768.0;
	pressao = (p + (var1 + var2 + ((double)dig_P7)) / 16.0) / 100;

  //Altitude sai em pés e é convertida para metros
  altitude = ((10^(log(pressao/101325.0)/5.2558797)-1/(-6.8755856*10^-6))*0,3048;

  //MPU6050
  
  //Define a pinagem do MPU6050
  fd = wiringToradexI2CSetup (0x68);
  wiringToradexI2CWriteReg8 (fd,0x6B,0x00);
  printf("Porta MPU6050 escolhida: 0x6B=%X\n", wiringToradexI2CReadReg8 (fd,0x6B));
  
  aceleracao_X = read_word_2c(0x3B);
  aceleracao_Y = read_word_2c(0x3D);
  aceleracao_Z = read_word_2c(0x3F);
  
  aceleracao_X_escalar = aceleracao_X / 16384.0;
  aceleracao_Y_escalar = aceleracao_Y/ 16384.0;
  aceleracao_Z_escalar = aceleracao_Z / 16384.0;

  printf("Inclinacao em XZ: %f\n", Inclinacao_XZ(aceleracao_X_escalar, aceleracao_Y_escalar, aceleracao_Z_escalar));
  printf("Inclinacao em YZ: %f\n", Inclinacao_YZ(aceleracao_X_escalar, aceleracao_Y_escalar, aceleracao_Z_escalar));
  

  double axatual = Inclinacao_XZ(aceleracao_X_escalar, aceleracao_Y_escalar, aceleracao_Z_escalar);

  double ayatual = Inclinacao_YZ(aceleracao_X_escalar, aceleracao_Y_escalar, aceleracao_Z_escalar);


  float a2atual =  ayatual*cos(.5236)+axatual*cos(2.0944);
  float a3atual = ayatual*cos(2.618)+axatual*cos(4.1888);

  controleservo1(PID*pid, ax, axatual);
  controleservo2(PID*pid, a2, a2atual);
  controleservo3(PID*pid, a3, a3atual);
  
  printf("Aceleracao escalar X: %f\n", aceleracao_X_escalar);
  printf("Aceleracao escalar Y: %f\n", aceleracao_Y_escalar);
  printf("Aceleracao escalar Z: %f\n", aceleracao_Z_escalar);
   
	// Imprime na tela pressao e altitude
	printf("Pressao : %.3f hPa \n", pressao);
  printf("Altitude : %.3f m \n", altitude);

  FILE *pont_arq; // cria variável ponteiro para o arquivo
  
  //abrindo o arquivo com tipo de abertura w
  pont_arq = fopen("Dados.txt", "a");
  
  //Armazenando no SD
  fprintf(pont_arq, "%s", aceleracao_X_escalar);
  fprintf(pont_arq, "%s", aceleracao_Y_escalar);
  fprintf(pont_arq, "%s", aceleracao_Z_escalar);
  fprintf(pont_arq, "%s", pressao);
  fprintf(pont_arq, "%s", altitude);
  fprintf(pont_arq, "%s", axatual);
  fprintf(pont_arq, "%s", ayatual);

  //Fechando o arquivo
  fclose(pont_arq);

  sleep(0.2);

  return 0;

}
}
