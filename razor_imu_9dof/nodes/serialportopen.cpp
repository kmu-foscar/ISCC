#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>

#define MODEMDEVICE "/dev/ttyUSB0" //UART1 사용
#define BAUDRATE B115200 //보레이트 9600 설정

int main( int argc, char **argv )
{
int fd;
int res;
unsigned char buf[255];
unsigned char buf2[6] = "HELLO";
struct termios oldtio, newtio; //터미널 구조체
int txemptystate;

fd = open( MODEMDEVICE, O_RDWR | O_NOCTTY ); //UART1 디바이스 파일 오픈
if( fd < 0 )
{
perror( MODEMDEVICE );
exit( -1 );
}
printf( "\n%s open!!\n",MODEMDEVICE ); //정상적으로 오픈되었을때 출력

tcgetattr(fd, &oldtio); //현재 설정을 oldtio에 저장

memset( &newtio, 0x0, sizeof( newtio ) );
newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD; //baud 통신 속도, CS8 (8bit, No Parity, 1 Stop Bit)설정
newtio.c_iflag = IGNPAR; //Parity 오류가 있는 문자 무시
newtio.c_oflag = 0; //출력처리 설정 0이면 아무것도 안함
newtio.c_lflag = 0; //Local Mode 설정, ICANON이면 널 문자가 들어올때까지 수신
newtio.c_cc[VTIME] = 0; //time-out 값으로 사용, time-out 값은 TIME*0.1초
newtio.c_cc[VMIN] = 1; //read가 리턴되기 위한 최소한의 문자 개수

tcflush( fd, TCIFLUSH ); //설정을 초기화
tcsetattr( fd, TCSANOW, &newtio ); //newtio 터미널 구조체의 속정으로 설정을 적용
fcntl( fd, F_SETFL, FNDELAY );

write(fd, buf2, sizeof(buf2)); //송신 테스트, HELLO를 전송, nDVR 리눅스 커널 /linux-2.4.26/drivers/char/tty_io.c 파일의 tty_write 함수

while( 1 )
{
if( (res = read( fd, buf, sizeof(buf))) > 0 ) //입력을 기다림, 문자의 갯수가 리텀됨, nDVR 리눅스 커널 /linux-2.4.26/drivers/char/tty_io.c 파일의 tty_read 함수
{
 printf( "read size = %d\n",res );
printf("============================================================================\n");
for(int i = 0 ; i < 256 ; ++i)
{
  if(48 <= (int)buf[i] && (int)buf[i] <= 57)
    printf("%d : %d\n",i, buf[i]-48);
  else if((int)buf[i] == 42)
    printf("%d : %c\n",i, '*');
  else if((int)buf[i] == 46)
    printf("%d : %c\n",i, '.');
  else if((int)buf[i] == 45)
    printf("%d : %c\n",i , '-');
  else if((int)buf[i] == 44)
    printf("%d : %c\n",i , ',');
}
printf("\n");

write( fd, buf, sizeof(buf) ); //입력을 받으면 HELLO를 전송하고 다시 read함수에서 기다림
}
else
{
//printf("-");
}
}
tcsetattr( fd, TCSANOW, &oldtio ); //이전 상태로 되돌린다.
close( fd );
}
