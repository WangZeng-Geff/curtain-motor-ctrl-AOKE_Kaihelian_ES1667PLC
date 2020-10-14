#include "include.h"
#include "sec.h"
#include "uart.h"
#include "task.h"
#include "repeater.h"
#include "softtimer.h"
#include "protocol.h"

#include "esuart.h"
#include "sec_printf.h"
#include "sec_tick.h"

static void byte_delay(void)
{
	delay10ms(1);
}

//处理电机串口传送报文并执行
static void uart1_check(struct task *t)
{
    uint32_t len = 0, dlen, ret = 0;
    uint8_t buf[0x200];

    len = esuart_peak_data(UART_CHN_1, buf, sizeof(buf));
    while(len) 
    {
    	byte_delay();
    	dlen = esuart_peak_data(UART_CHN_1, buf, sizeof(buf));
    	if(len == dlen)
    		break;
    	else
    		len = dlen;
    }
    if(len)
    {
        //工装测试本地通信所用
        pr_info("[PLC-LOCALCAST]:");
        print_debug_array(KERN_INFO, buf, len);
        ret = adaptor_on_uart1_received(buf, len);
        if (ret > 0){
            esuart_read_data(UART_CHN_DM, buf, ret);
            return ;
        } else if(len >= sizeof(buf)){
            esuart_read_data(UART_CHN_DM, buf, len);
            return ;
        }
        //电机控制串口处理
		if(get_uart_smart_frame(buf, len))								//careful of this function in different protocol !!!
		{
			do_send_report(buf, len);
		}
   		esuart_read_data(UART_CHN_1, buf, len);
    }
		
}
struct task uart1_check_task = 
{
	.flags   = TF_ALWAYS_ALIVE,
    .do_task = uart1_check,
};

//---------------------------------------------------------------------------------------
//发送报文到电机串口
static void dev_control(struct task *t)
{	
	struct UART_FRAME frame;
	static uint8_t cnt = 0;
	if(report.uart_busy)
	{
		if(++cnt >= 4)
		{
			cnt = 0;
			report.uart_busy = 0;
		}
		return;
	}
	if (peek_uart_order(&frame))
	{
        
		esuart_send_data(UART_CHN_DM, (uint8_t *)&frame, frame.data_len + 4);
		report.uart_busy = 1;
	}
	else
	{
		return;
	}
}

struct task dev_control_task = 
{
    .do_task = dev_control,
    .pri = &dev_control_task,
};

int sec_main(int argc, char *argv[])
{
	START_INIT();

    task_add(&soft_timer_task);
    task_add(&uart1_check_task);
    task_add(&dev_control_task);
    system_init();
	while(1)
	{
        task_handle();
	}
}
//---------------------------------------------------------------------------------------
