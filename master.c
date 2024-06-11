#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdint.h>  /* Include for uint16_t */
#define TR 1 // Deley entre as requisições - 1s
#define TS 20000 // Tempo entre a requisição e a resposta - 20ms




int main() {
  // Configurando Porta Serial
  printf("Configurando porta Serial:\n");
  char *portname = "/dev/ttyUSB0";
  // char *portname = "/dev/ttyACM0";
  int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
      perror("Erro ao abrir a porta serial");
      return 1;
  }
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd, &tty) != 0) {
      perror("Erro ao obter os atributos da porta serial");
      return 1;
  }
  cfsetospeed(&tty, B9600);
  cfsetispeed(&tty, B9600);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag |= CREAD | CLOCAL;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_oflag &= ~OPOST;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10;
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      perror("Erro ao configurar a porta serial");
      return 1;
  }


  // Transmissão da porta serial em MODBUS
  int bytes_written = 0;
  int bytes_read = 0;
  unsigned char response[10];  // unsigned char para valores de 0 a 255


  // Comandos MODBUS para cada medida
  // estrutura: ID_SLAVE, CMD_Leitura, Med_H, Med_L, SE_L, SE_H, CRC_L, CRC_H
  unsigned char temp[] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xD5, 0xCA};
  unsigned char umid[] = {0x01, 0x03, 0x00, 0x05, 0x00, 0x01, 0x94, 0x0B};
  unsigned char pres[] = {0x01, 0x03, 0x00, 0x63, 0x00, 0x01, 0x74, 0x14};

  // Variáveis para armazenar os valores lidos
  float tempVal;
  float umidVal;
  float presVal;
  float *pTempVal = &tempVal;
  float *pUmidVal = &umidVal;
  float *pPresVal = &presVal;


  while (1) {
      // Deley de 1 segundo
      sleep(TR);

      // Leitura da temperatura;
      printf("Temp: ");
      bytes_written = write(fd, temp, sizeof(temp));
      usleep(TS);
      bytes_read = read(fd, response, 8);
      if (bytes_read > 4) {
          uint16_t tempRaw = (response[3] << 8) | response[4];  // Combina os dois bytes
          *pTempVal = tempRaw / 10.0f;
          printf("%.1f\n", *pTempVal);
      } 
      else {
          printf("Sem resposta!\n");
      }


      // Leitura da Umidade;
      printf("Umid: ");
      bytes_written = write(fd, umid, sizeof(umid));
      usleep(TS);
      bytes_read = read(fd, response, 8);
      if (bytes_read > 4) {
          uint16_t umidRaw = (response[3] << 8) | response[4];  // Combina os dois bytes
          *pUmidVal = umidRaw / 100.0f;
          printf("%.2f\n", *pUmidVal);
      } 
      else {
          printf("Sem resposta!\n");
      }
      


      // Leitura da pressão;
      printf("Pres: ");
      bytes_written = write(fd, pres, sizeof(pres));
      usleep(TS);
      bytes_read = read(fd, response, 8);
      if (bytes_read > 4) {
          uint16_t presRaw = (response[3] << 8) | response[4];  // Combina os dois bytes
          *pPresVal = presRaw / 100.0f;
          printf("%.2f\n", *pPresVal);
      } 
      else {
          printf("Sem resposta!\n");
      }

  }
  close(fd);
  printf("Terminado\n");
  return 0;
}