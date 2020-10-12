#include "link_list.h"
#include "data_global.h"

#define LEN_ENV 20
#define LEN_RFID 4

extern int dev_uart_fd;
extern linklist linkHead;
extern pthread_cond_t cond_analysis;
extern pthread_mutex_t mutex_linklist;

//串口初始化函数
void serial_init(int fd)
{
    /*
    *   tcgetattr()用于获取与终端相关的参数
    *   termions有四个成员变量
    *   tcflag_t c_iflags;
    *   tcflag_t c_oflag;
    *   tcflag_t c_cflag;
    *   tcflag_t c_lflag;
    *   cc_t c_cc[NCCS];
    */
    struct termios options;
    tcgetattr(fd, &options);
    options.c_cflags |= (CLOCAL | CREAD);   //设置忽略调制解调器线路状态使用接收器
    options.c_cflags &= ~CSIZE;             //关闭字符长度设置
    options.c_cflags &= ~CRTSCTS;           //关闭RTS/CTS流控制
    options.c_cflags |= CS8;                //设置字符长度为CS8
    options.c_cflags &= ~CSTOPB;            //停止位设置为2
    options.c_iflags |= IGNPAR;             //打开忽略奇偶校验位
    options.c_iflags &= ~(ICRNL | IXON);    //将输入回车转换为换行和进行流控制
    options.c_oflag = 0;
    options.c_lflag = 0;

    cfsetispeed(&options, B115200); //设置输入波特率
    cfsetospeed(&options, B115200)；//设置输出波特率
    tcsetattr(fd, TCSANOW, &options); //设置中短程参数的函数
}

void *pthread_transfer(void *arg)
{
    int i = 0, len;
    char flag = 0, check;
    link_datatype buf;

    linkHead = CreateEmptyLinklist();
    if((dev_uart_fd = open(DEV_ZIGBEE, O_RDWR)) < 0)
    {
        perror("open");
        exit(-1);
    }

    serial_init(dev_uart_fd);
    printf("pthread_transfer is ok\n");

    while(1)
    {
        memset(&buf, 0, sizeof(link_datatype));
        read(dev_uart_fd, &check, 1);
        if(check == 'c')
        {
            sendMsgQueue(MSG_M0, MSG_CONNECT_SUCCESS);
        }
        if(check == 'c')
        {
            check = 0;
            read(dev_uart_fd, &check, 1);
            if(check == ':')
            {
                check = 0;
                buf.msg_type = 'e';
                usleep(1);
                if ((len = read(dev_uart_fd, buf.text, LEN_ENV)) != LEN_ENV)
                {
                    for (i = len; i < LEN_ENV; i++)
					{
						read (dev_uart_fd, buf.text+i, 1);
					}
                }
                flag = 1;
            }
            else if (check == 'r')
			{
				buf.msg_type = 'r';
			    usleep(1);
				if ((len = read (dev_uart_fd, buf.text, LEN_RFID)) != LEN_RFID)
				{
					for (i = len; i < LEN_RFID; i++)
					{
					    read (dev_uart_fd, buf.text+i, 1);
					}
				}
					flag = 1;
			}
        }
        if (1 == flag)
		{
			pthread_mutex_lock (&mutex_linklist);
			if ((InsertLinknode (buf)) == -1)
			{
				pthread_mutex_unlock (&mutex_linklist);
				printf ("NONMEM\n");
			}
			pthread_mutex_unlock (&mutex_linklist);
			flag = 0;
			pthread_cond_signal (&cond_analysis);
		}
    }
    return 0;
}
