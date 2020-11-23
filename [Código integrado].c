//Bibliotecas gerais:

//Relativas ao BMP280
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <math.h>

//Relativas ao MPU6050
#include <wiringPiI2C.h>
#include <wiringPi.h> //-> Descobrir a pinagem do Toradex
 




//Declarar portas

//Relativa ao MPU6050
int read_word_2c(int addr)
{
int val;
val = wiringPiI2CReadReg8(fd, addr);
val = val << 8;
val += wiringPiI2CReadReg8(fd, addr+1);
if (val >= 0x8000)
val = -(65536 - val);
 
return val;
}


//Inicialização de sensores




//Declaração de variáveis
double pressao = 0;
double altitude = 0;
int fd;
int aceleracao_X, aceleracao_Y, aceleracao_Z;
int giro_X, giro_Y, giro_Z;
double aceleracao_X_escalar, aceleracao_Y_escalar, aceleracao_Z_escalar;
double gyroX_scaled, gyroY_scaled, gyroZ_scaled;

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
return -(radians * (180.0 / M_PI));
}
 
double Inclinacao_XZ(double x, double y, double z)
{
double radians;
radians = atan2(y, dist(x, z));
return (radians * (180.0 / M_PI));
}

int main() { 
	
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
  fd = wiringPiI2CSetup (0x68);
  wiringPiI2CWriteReg8 (fd,0x6B,0x00);
  printf("Porta MPU6050 escolhida: 0x6B=%X\n",wiringPiI2CReadReg8 (fd,0x6B));
  
  while(1) {
  aceleracao_X = read_word_2c(0x3B);
  aceleracao_Y = read_word_2c(0x3D);
  aceleracao_Z = read_word_2c(0x3F);
  
  aceleracao_X_escalar = aceleracao_X / 16384.0;
  aceleracao_Y_escalar = aceleracao_Y/ 16384.0;
  aceleracao_Z_escalar = aceleracao_Z / 16384.0;
  
  printf("Aceleracao escalar X: %f\n", aceleracao_X_escalar);
  printf("Aceleracao escalar Y: %f\n", aceleracao_Y_escalar);
  printf("Aceleracao escalar Z: %f\n", aceleracao_Z_escalar);
  
  printf("Inclinacao em XZ: %f\n", Inclinacao_XZ(aceleracao_X_escalar, aceleracao_Y_escalar, aceleracao_Z_escalar));
  printf("Inclinacao em XY: %f\n", Inclinacao_YZ(Inclinacao_XZ(aceleracao_X_escalar, aceleracao_Y_escalar, aceleracao_Z_escalar));
  }

//Lembrar de trocar as variaveis da aceleracao e da rotacao





	// Imprime na tela pressao e altitude
	printf("Pressao : %.3f hPa \n", pressao);
  printf("Altitude : %.3f m \n", altitude);




  return 0;

}
