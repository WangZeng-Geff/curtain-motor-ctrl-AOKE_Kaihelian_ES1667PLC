#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "include.h"
#include "sdk.h"
#include "flash.h"
#include "sdk_svc.h"
#include "comfunc.h"
#include "sec_printf.h"
#include "protocol.h"
#include "update.h"
#include "timer.h"
#include "repeater.h"
#include "gpio.h"
#include "sec_tick.h"
#include "dev_show.h"
#include "task.h"
#include "softtimer.h"
#include "gpio.h"
#include "esuart.h"

struct DEV_DATA dev_data;
struct DEV_STATE dev_state;
struct DEV_STATE rx_uart;
struct Report report;



//---------------------------------测试工装功能------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//设置aid
static int on_set_aid(uint8_t *data, int len)
{
    if (len != AID_LEN) return -1;
	
    if (!memcmp(repeater.aid, data, sizeof(repeater.aid))) 
    {
    	data[0] = CMD_NAK;
        return 1;
    }
    memcpy(repeater.aid, data, sizeof(repeater.aid));
    data[0] = CMD_ACK;
    return 1;
}
static int on_get_aid(uint8_t *data, int len)
{
    if (is_all_xx(repeater.aid, 0, sizeof(repeater.aid))) 
    {
        data[0] = CMD_NAK;
        return 1;
    }

    data[0] = CMD_ACK_AID;
    memcpy(&data[1], repeater.aid, sizeof(repeater.aid));
    return 1 + sizeof(repeater.aid);
}//---------------------------------------------------------------------------------------
static int on_get_eid(uint8_t *data, int len)
{
    sdk_svc_mac_t mac;
    sdk_err_t err = sdk_svc_get_mac(&mac);
    if (err != SDK_ERR_OK)
    {
    	pr_info("[LOCALDID-GETEID]got svc mac failed, err[%d].\n", err);
        return -1;
    }

    data[0] = CMD_ACK_EID;
    memcpy(&data[1], repeater.mac, sizeof(mac.mac));
    memset(&data[1+sizeof(repeater.mac)], 0, 2);
    return 9;
}

//---------------------------------------------------------------------------------------
static const struct LocalOps ops[] = 
{
    {"SET AID", CMD_SET_AID, on_set_aid},
    {"GET AID", CMD_GET_AID, on_get_aid},
    {"GET EID", CMD_GET_EID, on_get_eid},
};

static const struct LocalOps *get_ops(int cmd)
{
    int i;

    for (i = 0; i < array_size(ops); ++i)
    {
        if (ops[i].cmd == cmd) 
        {
            return &ops[i];
        }
    }
    return NULL;
}
static int do_local_frame(struct SmartFrame *pframe)
{
    const struct LocalOps *ops = get_ops(pframe->data[0]);
    if (!ops)
    {
        pr_info("[PROGRESS-LOCALFRAM]not find cmd, cmd[%d].\n", pframe->data[0]);
        return -1;
    }

    pr_info("[PROGRESS-LOCALFRAM]Local Frame[%s].\n", ops->name); 
    print_debug_array(KERN_INFO, pframe, frame_len(pframe));
    int ret = ops->handle(&pframe->data[1], pframe->len-1);
    if (ret > 0)
    {
        ret = code_local_frame(&pframe->data[1], ret, pframe, 0x100);//组织一条本息通信报文 数据域位从pframe->data[1]开始长度为ret输出到pframe内最大长度0x100
        esuart_send_data(UART_CHN_DM, (uint8_t*)pframe, ret);
    }
    return 0;
}

//---------------------------------------------------------------------------------------
int smart_frame_handle(struct SmartFrame *frame)
{
    if (is_all_xx(frame->said, 0x00, AID_LEN) && is_all_xx(frame->taid, 0x00, AID_LEN))
    {
        return do_local_frame(frame);
    }
		return 0;
}

int adaptor_on_uart1_received(const uint8_t *data, int len)
{
    struct SmartFrame *frame = get_smart_frame(data, len);
    if (frame)
    {
        len = ((const uint8_t *)frame - data) + frame_len(frame);
        smart_frame_handle(frame);
        return len;
    }

    return 0;
}







//---------------------------------------------------------------------------------------
//------------------------------电力线通信报文--------------------------------------------
//---------------------------------------------------------------------------------------
static void add_last_aid(uint8_t *aid, uint8_t *mac)
{
	if(!memcmp(repeater.gid, aid, AID_LEN))
		return;
	int i;
	for(i = 0; i < MAX_CTRL_NUM; i++)
	{
		if(!memcmp(ram_data.ctrl_meg[i].aid, aid, AID_LEN))
			return;		
	}
	for(i = 0; i < MAX_CTRL_NUM-1; i++)
	{
		memcpy(ram_data.ctrl_meg[i].aid, ram_data.ctrl_meg[i+1].aid, AID_LEN);
		memcpy(ram_data.ctrl_meg[i].mac, ram_data.ctrl_meg[i+1].mac, MAC_LEN);
	}
	memcpy(ram_data.ctrl_meg[i].aid, aid, AID_LEN);
	memcpy(ram_data.ctrl_meg[i].mac, mac, MAC_LEN);
}
//---------------------------------------------------------------------------------------
struct SmartFrame *get_smart_frame(const uint8_t *in, int len)
{
    int i = 0;

 start_lbl:
    while (i < len)
    {
        if (STC == in[i])
            break;
        i++;
    }
    if (len - i < SMART_FRAME_HEAD)
        return NULL;
    struct SmartFrame *pframe = (struct SmartFrame *)&in[i];
    int dlen = pframe->len;

    if (i + SMART_FRAME_HEAD + dlen + 1 > len)
    {
        //i++;
        //goto start_lbl;
		return NULL;
    }

    if (pframe->data[dlen] != checksum(pframe, dlen + SMART_FRAME_HEAD))
    {
        i++;
        goto start_lbl;
    }
    pframe = (struct SmartFrame *)&in[i];
    return pframe;
}
//---------------------------------------------------------------------------------------
int code_frame(const uint8_t *src, const uint8_t *dest, int seq, int cmd, 
    const uint8_t *data, int len, void *out, int maxlen)
{
    const uint8_t addr[AID_LEN] = {0x00, 0x00, 0x00, 0x00};

    static uint8_t _seq = 0;
    struct SmartFrame *pframe = (struct SmartFrame *)out;

    pframe->stc = STC;
    if (!src) src = addr;
    if (!dest) dest = addr;
    memcpy(pframe->said, src, AID_LEN);
    memcpy(pframe->taid, dest, AID_LEN);
    pframe->seq = seq < 0 ? (_seq++ & 0x7F) : seq;

    pframe->data[0] = cmd;
    memcpy(&pframe->data[1], data, len);
    pframe->len = len+1;
    pframe->data[pframe->len] = checksum(pframe, SMART_FRAME_HEAD + pframe->len);
    return SMART_FRAME_HEAD + pframe->len + 1;
}

int code_local_frame(const uint8_t *in, int len, void *out, int maxlen)
{
    return code_frame(NULL, NULL, 0, in[0], &in[1], len-1, out, maxlen);
}

int code_ret_frame(struct SmartFrame *pframe, int len)
{
    memcpy(pframe->taid, pframe->said, AID_LEN); 
    memcpy(pframe->said, repeater.aid, AID_LEN); 
    pframe->seq |= 0x80;
    pframe->len = len;
    pframe->data[len] = checksum(pframe, pframe->len + SMART_FRAME_HEAD);
    return pframe->len + SMART_FRAME_HEAD + 1;
}

int code_body(uint16_t did, int err, const void *data, int len, void *out, int maxlen)
{
    struct Body *body = (struct Body *)out;

    put_le_val(did, body->did, sizeof(body->did));
    body->ctrl = get_bits(len, 0, 6);
    if (err) body->ctrl |= 0x80;
    memcpy(body->data, data, len);

    return sizeof(struct Body) + len;
}
//---------------------------------------------------------------------------------------
int is_my_smart_frame(const struct SmartFrame *frame)
{
    if (is_all_xx(repeater.aid, 0x00, AID_LEN))     //本地无ID直接退出
    {
        pr_info("I do not have aid, drop it!\n");
        return 0;
    }

    if (!is_all_xx(frame->taid, 0xFF, AID_LEN) && memcmp(frame->taid, repeater.aid, AID_LEN)) //非广播  并且  目标ID与本地ID不同
    {
        return 0;
    }
    return 1;
}
//---------------------------------------------------------------------------------------
static int is_lowpw_frame(struct SmartFrame *pframe)
{
    int i;
    const uint16_t low_did[] = {0x0004, 0x0603, 0x0604, 0x0605, 0x0606, 0x0607, 0x0608, 0x060A, 0x060C};

    struct AppFrame *app = (struct AppFrame *)pframe->data;

	if (CMD_SHOW == app->cmd)
	{
		return 1;
	}
	else
	{
		uint16_t did = get_le_val(((struct Body *)app->data)->did, DID_LEN);

		for (i = 0; i < array_size(low_did); i++)
		{
			if (did == low_did[i])
				return 1;
		}
	}

	return 0;
}
//---------------------------------------------------------------------------------------
int do_send_frame(uint8_t *mac, struct SmartFrame *frame)
{
    int key_mod = is_lowpw_frame(frame) ? SDK_KEY_INIT : SDK_KEY_USER;
    if (is_all_xx(frame->taid, 0xFF, AID_LEN)) 
    {
        return do_plc_broadcast(key_mod, frame, frame_len(frame));
    }
    
    return do_plc_unicast(mac, key_mod, frame, frame_len(frame));
}
//---------------------------------------------------------------------------------------
static void reboot_tmr_handle(ULONG arg)
{
    sdk_svc_reset();
}
//---------------------------------------------------------------------------------------
static int do_reboot(const uint8_t *src, struct Body *body)
{
    uint32_t wait = 2000; //default 2s
    if (get_bits(body->ctrl, 0, 6) == 2) 
        wait = get_le_val(body->data, 2); 

    estimer_t reboot_tmr = timer_create(wait/10, 0, TMR_RUNNING, TMR_TRIGGER, 
        reboot_tmr_handle, NULL, "reboot_tmr");
    if (!reboot_tmr)
    {
        pr_info("creat bps timer failed!\n");
        return -OTHER_ERR;
    }
    return NO_ERR;
}
//---------------------------------------------------------------------------------------
static int do_get_dev_ver(const uint8_t *src, struct Body *body)
{
    const char *soft_ver = get_dev_ver();
    strcpy((char *)body->data, soft_ver); 
    return strlen(soft_ver);
}
//---------------------------------------------------------------------------------------
static int do_get_plc_ver(const uint8_t *src, struct Body *body)
{
    const char *soft_ver = get_plc_ver();
    strcpy((char *)body->data, soft_ver); 
    return strlen(soft_ver);
}
//---------------------------------------------------------------------------------------
static int do_get_dev_type(const uint8_t *src, struct Body *body)
{
	return get_dev_type(body->data);
}
//---------------------------------------------------------------------------------------
static int do_get_comm_ver(const uint8_t *src, struct Body *body)
{
    strcpy((char *)body->data, comm_protocol_ver); 
    return strlen(comm_protocol_ver);
}
//---------------------------------------------------------------------------------------
static int do_get_plc_type(const uint8_t *src, struct Body *body)
{
    strcpy((char *)body->data, repeater_plc_type); 
    return strlen(repeater_plc_type);
}
//---------------------------------------------------------------------------------------
static void do_notify_link(const uint8_t *dst, int unlink)
{
    uint8_t reg[AID_LEN+SID_LEN];
    memcpy(&reg[0], repeater.aid, sizeof(repeater.aid));
    memcpy(&reg[AID_LEN], repeater.sid, sizeof(repeater.sid)); 

    uint8_t body[0x100];
    int len = code_body(0x0603, 0, reg, sizeof(reg), body, sizeof(body));
    if (unlink) ((struct Body *)body)->ctrl |= 0x80;

    uint8_t tmp[0x100];
    len = code_frame(repeater.aid, repeater.gid, -1, CMD_SET, body, len, tmp, sizeof(tmp));

    do_plc_unicast(dst, SDK_KEY_INIT, tmp, len);
}
//---------------------------------------------------------------------------------------
static int do_reg(const uint8_t *src, struct Body *body)
{
	struct deviceCfgInfo cfg;
    if (get_bits(body->ctrl, 0, 6) != sizeof(struct RegData)) 
        return -LEN_ERR; 

    struct RegData *reg = (struct RegData *)body->data; 
    if (is_all_xx(repeater.aid, 0x00, AID_LEN) || memcmp(repeater.aid, reg->aid, AID_LEN))
        return -OTHER_ERR;
    int unlink = 1;
    if(!memcmp(repeater.passwd, reg->pw, PW_LEN))
    {
    	memcpy(repeater.panid, reg->panid, PANID_LEN);
    	memcpy(repeater.gid, reg->gid, AID_LEN);
    	memcpy(repeater.sid, reg->sid, SID_LEN);
    	memcpy(repeater.gmac, src, MAC_LEN);
    	do_set_psk();
    	LED_ON(LED_NET);
    	cfg.magic = DEVICE_CFG_MAGIC;
    	memcpy(cfg.gid, repeater.gid, AID_LEN);
    	memcpy(cfg.gmac, repeater.gmac, MAC_LEN);
    	memcpy(cfg.panid, repeater.panid, PANID_LEN);
    	memcpy(cfg.sid, repeater.sid, SID_LEN);
    	flash_user_info_write(DEVICE_CFG_ADDR, (uint8_t*)&cfg, sizeof(cfg));
    	unlink = 0;
    }
    do_notify_link(src, unlink);
    return -NO_RETURN;
}
//---------------------------------------------------------------------------------------
static int do_notify_aid(const uint8_t *src, struct Body *body)
{
    return -NO_RETURN;
}
//---------------------------------------------------------------------------------------
static int do_get_aid(const uint8_t *src, struct Body *body)
{
    if (is_all_xx(repeater.aid, 0, AID_LEN) || memcmp(body->data, repeater.aid, AID_LEN)) 
        return -NO_RETURN;

    memcpy(body->data, repeater.aid, AID_LEN);
    return AID_LEN;
}
//---------------------------------------------------------------------------------------
static int do_get_ext_ver(const uint8_t *src, struct Body *body)
{
    strcpy((char *)body->data, repeater_ext_ver); 
    reverse(body->data, strlen(repeater_ext_ver));
    return strlen(repeater_ext_ver);
}
//---------------------------------------------------------------------------------------
static int do_get_dev_key(const uint8_t *src, struct Body *body)
{
	memcpy(body->data, repeater.dkey, DKEY_LEN);
	return DKEY_LEN;
}
//---------------------------------------------------------------------------------------
static int do_get_dev_sn(const uint8_t *src, struct Body *body)
{
	memcpy(body->data, repeater.sn, SN_LEN);
	return SN_LEN;
}
//---------------------------------------------------------------------------------------
static int do_get_pwd(const uint8_t *src, struct Body *body)
{
	memcpy(body->data, repeater.passwd, PW_LEN);
	return PW_LEN;
}
//---------------------------------------------------------------------------------------
static void dev_show_timer_handle(struct soft_timer *st)
{
	static uint8_t i = 0;
	uint8_t data_close[5] = {0x9A, 0x0A, 0x01, 0xDD, 0x4C};
	uint8_t data_open[5]= {0x9A, 0x0A, 0x01, 0xEE, 0x7F};
	uint8_t data_stop[5] = {0x9A, 0x0A, 0x01, 0xCC, 0x5D};

	st->expires = sec_jiffies + SEC_HZ*3;
	i++;
	if(i == 1)
	{
//        pr_info("uart: send data--------------------from dev_show_timer_handle1!");
		esuart_send_data(UART_CHN_DM, data_open, 5);
		report.uart_busy = 1;
		soft_timer_add(st);
	}	
	else if(i == 2)
	{
//        pr_info("uart: send data--------------------from dev_show_timer_handle2!");
		esuart_send_data(UART_CHN_DM, data_close, 5);
		report.uart_busy = 1;
		soft_timer_add(st);
		
	}
	else if(i > 2)
	{
//        pr_info("uart: send data--------------------from dev_show_timer_handle3!");
		esuart_send_data(UART_CHN_DM, data_stop, 5);
		report.uart_busy = 1;
		i = 0;
	}
}
struct soft_timer dev_show_timer =
{
    .timer_cb = dev_show_timer_handle,
};
static int do_set_dev_show(const uint8_t *src, struct Body *body)
{
	soft_timer_del(&dev_show_timer);
	dev_show_timer.expires = sec_jiffies;
	soft_timer_add(&dev_show_timer);
	return NO_ERR;
}
//---------------------------------------------------------------------------------------
static int do_set_silent_time(const uint8_t *src, struct Body *body)
{
   if (body->ctrl != 0x02)
   {
	   return -DATA_ERR;
   }
   ram_data.silent_time = sec_jiffies + get_le_val(body->data, TIME_LEN) * SEC_HZ;

   return NO_ERR;
}
//---------------------------------------------------------------------------------------
static int do_get_dev_location(const uint8_t *src, struct Body *body)
{
	body->data[0] = dev_data.location;
	return(1);
}
static int do_set_dev_location(const uint8_t *src, struct Body *body)
{
	uint8_t data1[3] = {0x00, 0x64, 0x00};
	uint8_t data2[3] = {0x02, 0x64, 0x00};
    if(body->ctrl != 0x01 || body->data[0] > 1)
    {
        return -DATA_ERR;
    }
	dev_data.location = body->data[0];
	if(dev_data.location == INIT_DIR)
	{
		code_uart_frame(0x11, data1, 3);
	}
	else
	{
		code_uart_frame(0x11, data2, 3);
	}
    flash_user_info_write(AIR_DATA_ADDR, (uint8_t*)&dev_data, sizeof(dev_data));
    return NO_ERR;
}
//---------------------------------------------------------------------------------------
uint8_t code_uart_frame(uint8_t cmd, uint8_t *data, uint8_t  data_len)
{
	struct UART_FRAME frame;
	frame.start = START_BYTE;
	frame.cmd	= cmd;
	frame.data_len = data_len;
	memcpy(&frame.data[0], data, data_len);
	frame.data[data_len] = XorCheack((uint8_t *)&frame, data_len + 3);
	insert_uart_order(&frame);
	return sizeof(frame);
}
//---------------------------------------------------------------------------------------
static int do_dev_control_compatible(const uint8_t *src, struct Body *body)	//DID 0A01 1.stop 2.open 3.close 兼容旧设备，适配LCD等设备
{
	uint8_t ctrl_state = 0;
	uint8_t rx_val = 0;

	if(body->ctrl != 0x04)
	{
		return (-DATA_ERR);
	}
	rx_val = (body->data[1] & 0x0F);
	if(rx_val < 0x01 || rx_val > 0x03)
	{
		return (-DATA_ERR);
	} 
	if(rx_val == STOP && dev_state.motor_state == STOP)
	{
//        pr_info("Set Flage's Type--------------------------:NO_ERR");
		return (NO_ERR);
	}
	
    dev_state.ctrl_flag = NO_CTRL_FLAG;									//identify controlor
	if(memcmp(ram_data.last_ctrl_aid, repeater.gid, AID_LEN) == 0)
	{
		dev_state.ctrl_flag = GW_CTRL_FLAG;
	}
	else
	{
		dev_state.ctrl_flag = SUB_CTRL_FLAG;
		memcpy(ram_data.last_ctrl_mac, src, MAC_LEN);
	}
	
	switch(rx_val)
	{
		case 1: ctrl_state = 0xCC; dev_state.rx_plc_motor_stop = FLAGON; break;								//stop
		case 2: ctrl_state = 0x64; dev_state.target_open_percent = 0x64; break;								//open
		case 3: ctrl_state = 0x00; dev_state.target_open_percent = 0x00; break;								//close
	}
	if(ctrl_state == 0xCC)
		code_uart_frame(0x0A, &ctrl_state, 1);
	else
		code_uart_frame(0x0D, &ctrl_state, 1);
	
	return (NO_ERR);
}
//---------------------------------------------------------------------------------------
static int do_dev_control(const uint8_t *src, struct Body *body)	//DID 0A04   1.stop 2.open 3.close
{
	uint8_t ctrl_state = 0;
	if(body->ctrl != 0x01)
	{
		return (-DATA_ERR);
	}
	if(body->data[0] < 0x01 || body->data[0] > 0x03)
	{
		return (-DATA_ERR);
	} 
	if(body->data[0] == STOP && dev_state.motor_state == STOP)
	{
		return (NO_ERR);
	}
  dev_state.ctrl_flag = NO_CTRL_FLAG;									//identify controlor
	if(memcmp(ram_data.last_ctrl_aid, repeater.gid, AID_LEN) == 0)
	{
		dev_state.ctrl_flag = GW_CTRL_FLAG;
	}
	else
	{
		dev_state.ctrl_flag = SUB_CTRL_FLAG;
		memcpy(ram_data.last_ctrl_mac, src, MAC_LEN);
	}
	
	switch(body->data[0])
	{
		case 1: ctrl_state = 0xCC; dev_state.rx_plc_motor_stop = FLAGON; break;								//stop
		case 2: ctrl_state = 0x64; dev_state.target_open_percent = 0x64; break;								//open
		case 3: ctrl_state = 0x00; dev_state.target_open_percent = 0x00; break;								//close
	}

//    pr_info("Set Flage's Type:%d--------------------------!\n",body->data[0]);
	if(ctrl_state == 0xCC)
		code_uart_frame(0x0A, &ctrl_state, 1);
    else
		code_uart_frame(0x0D, &ctrl_state, 1);
	
	return (NO_ERR);
}
//---------------------------------------------------------------------------------------
static int do_get_open_percent(const uint8_t *src, struct Body *body)
{
	body->data[0] = dev_state.target_open_percent;
	return (1);
}
static int do_set_open_percent(const uint8_t *src, struct Body *body)
{

	if(body->ctrl != 0x01)
	{
		return (-DATA_ERR);
	}
	if(body->data[0] > 100)
	{
		return (-DATA_ERR);
	}

	dev_state.ctrl_flag = NO_CTRL_FLAG;								//identify controlor
	if(memcmp(ram_data.last_ctrl_aid, repeater.gid, AID_LEN) == 0)
	{
		dev_state.ctrl_flag = GW_CTRL_FLAG;
	}
	else
	{
		dev_state.ctrl_flag = SUB_CTRL_FLAG;
		memcpy(ram_data.last_ctrl_mac, src, MAC_LEN);
	}
//    if (body->data[0] == dev_state.target_open_percent) {
//        return (NO_ERR);
//    }

	dev_state.target_open_percent = body->data[0];
	code_uart_frame(0x0D, &body->data[0], 1);
	return (NO_ERR);
}
//---------------------------------------------------------------------------------------
enum
{
    HCTR_LOWPW = 1 << 0,
};

struct func_ops
{
    int did;
    const char *tip;
    uint32_t ctrl;
    int (*read)  (const uint8_t *src, struct Body *body);
    int (*write) (const uint8_t *src, struct Body *body);
    int (*read_resp)  (const uint8_t *src, struct Body *body);
    int (*write_resp) (const uint8_t *src, struct Body *body);
};
const struct func_ops func_items[] = 
{
    {0x0601, "Reboot",     0, NULL, do_reboot, NULL, NULL},
    {0x0603, "Register",   0, NULL, do_reg, NULL, NULL},
    {0x0604, "Notify Aid", 0, NULL, do_notify_aid, NULL, NULL},
    {0x0605, "Get Aid",    0, do_get_aid,	   NULL, NULL, NULL},
    {0x0609, "GetPLCType", 0, do_get_plc_type, NULL, NULL, NULL},
    {0x060A, "GetPLCVer",  0, do_get_plc_ver,  NULL, NULL, NULL},
    {0x0001, "DevType",	   0, do_get_dev_type, NULL, NULL, NULL},
    {0x0002, "ComProVer",  0, do_get_comm_ver, NULL, NULL, NULL},
    {0x0003, "DevVersion", 0, do_get_dev_ver,  NULL, NULL, NULL},
    {0x0004, "GetExtVer",  0, do_get_ext_ver,  NULL, NULL, NULL},
    {0x0005, "DevKey", 	   0, do_get_dev_key,  NULL, NULL, NULL},
    {0x0007, "DevSn",      0, do_get_dev_sn,   NULL, NULL, NULL},
	{0x0009, "SetDevShow", 0, NULL, do_set_dev_show, NULL, NULL},
    {0x000A, "GetPwd", 	   0, do_get_pwd,	   NULL, NULL, NULL},
    {0x000B, "SetSilTime", 0, NULL, do_set_silent_time, NULL, NULL},
	
	{0x0A08, "DevLocation", 0, do_get_dev_location, do_set_dev_location, NULL, NULL}, 			//运动方向 即电机安装位置在左or在右
	{0x0A01, "DevContrl",	0, NULL, 		  do_dev_control_compatible ,NULL, NULL},			//开合帘开/关/停
    {0x0A03, "OpenPercent", 0, do_get_open_percent, do_set_open_percent, NULL, NULL}, 			//开合度设置/读取
    {0x0A04, "DevContrl",	0, NULL, 				do_dev_control,		 NULL, NULL},			//开合帘开/关/停
};

//根据did获取将要执行的步骤并执行
static const struct func_ops *get_option(int did)
{
    int i;

    const struct func_ops *funcs = func_items;
    int funcs_nr = array_size(func_items);

    for (i = 0; i < funcs_nr; i++)
    {
        if (funcs[i].did == did)
            return &funcs[i];
    }
    return NULL;
}
//---------------------------------------------------------------------------------------
static int form_error_body(void *out, uint8_t * did, int err)
{
    struct Body *body = (struct Body *)out;

    memcpy(body->did, did, sizeof(body->did));
    body->ctrl = 0x82;
    put_le_val(err, body->data, 2);

    return offset_of(struct Body, data)+get_bits(body->ctrl, 0, 6);
}
//---------------------------------------------------------------------------------------
//单独控制指令的解析与执行     电力线接收数据参数  指令类型   数据长度      通信数据     数据长度        地址
static int do_cmd(const sdk_evt_rx_plc_t *info, int cmd, int resp, uint8_t *data, int len, uint8_t *said)
{
    int outidx = 0, inidx = 0;

    uint8_t tmp[BUF_LEN];
    
    memcpy(tmp, data, len);

    while (len >= FBD_FRAME_HEAD)
    {
        struct Body *body = (struct Body *)&tmp[inidx];
        struct Body *outbd = (struct Body *)&data[outidx];

        int dlen = get_bits(body->ctrl, 0, 6);
        if (len < FBD_FRAME_HEAD + dlen)
        {
            pr_info("Smart frame, body length error!\n");
            outidx += form_error_body(outbd, body->did, LEN_ERR);
            break;
        }
        inidx += FBD_FRAME_HEAD + dlen;
        len -= FBD_FRAME_HEAD + dlen;

        const struct func_ops *op = get_option(get_le_val(body->did, sizeof(body->did)));
        int (*handler)(const uint8_t *src, struct Body *body) = NULL;
        if (cmd == CMD_GET)
        {
            handler = op ? (resp ? op->read_resp : op->read) : NULL;
        }
        else
        {
            handler = op ? (resp ? op->write_resp : op->write) : NULL;
        }

        if (handler)
        {
			memcpy(ram_data.last_ctrl_aid, said, AID_LEN);
			add_last_aid(said, (uint8_t*)info->src);
            pr_info("Do cmd[%s]...\n", op->tip);

            memcpy(outbd, body, sizeof(struct Body) + dlen);
            int ret = handler(info->src, outbd); 
            if (ret < 0)
            {
                if (ret == -NO_RETURN) continue;
                form_error_body(outbd, body->did, -ret);
            }
            else if (ret > 0)
            {
                memcpy(outbd->did, body->did, sizeof(body->did));
                outbd->ctrl = ret;
            }
            else
            {
                body->ctrl = 0;
				if((op->did == 0x0A01 || op->did == 0x0A03 || op->did == 0x0A04) && (handler == op->write))
				{
					report.broadcast_ctrl = 0;
					report.broadcast_my_seq = 0;
				}
            }
        }
        else if(!resp)
        {
        	form_error_body(outbd, body->did, DID_ERR);
        }
        else
        {
        	return -1;
        }
        outidx += FBD_FRAME_HEAD + get_bits(outbd->ctrl, 0, 6);
    }
    return outidx;
}
//---------------------------------------------------------------------------------------
int resend_check(const uint8_t *src, const struct SmartFrame *pframe)
{
    static uint8_t last_seq, last_src[6], last_sum;
    static uint32_t last_tm;

    uint32_t cur_seq = get_bits(pframe->seq, 0, 6);     //当前帧序号获取
    uint32_t cur_tm = sec_jiffies;                      //当前时间
    uint8_t cur_sum = pframe->data[pframe->len];        //当前校验和
    if (!memcmp(src, last_src, sizeof(last_src))        //还是那个一个MAC地址与当下相同
        && cur_seq == last_seq                          //上一个帧序号和当下一致
        && cur_sum == last_sum                          //上一个校验和和当先一致
        && time_after(last_tm+SEC_HZ/5, cur_tm))        //当前时间与上次接收时间小于标准间隔
    {
        pr_info("Receivd dummy frame, drop it!, "
                              "last_tm[%d], cur_tm[%d]\n", last_tm, cur_tm);
        return -1;
    }
    memcpy(last_src, src, sizeof(last_src));
    last_seq = cur_seq;
    last_sum = cur_sum;
    last_tm = cur_tm;
    return 0;
}
//---------------------------------------------------------------------------------------
/*
 data[0]: bit[0~5]: data length
          bit[6~7]: group type
 group_type: 0: bit
             1: 1byte  group id
             2: 2bytes group id
*/
int is_gid_equal(const uint8_t * data, const uint8_t * sid)
{
    int dlen = get_bits(data[0], 0, 5);
    int group_type = get_bits(data[0], 6, 7);
    int _sid = get_le_val(sid, 2);
	uint16_t j = 0;
	uint8_t sid_mod_8 = (_sid%8) ? (_sid%8):8;
	
    data++;
    if (group_type == 0) /* bit type */
    {
        _sid--;
        if (dlen < get_bits(_sid, 3, 7) + 1)
		{
			for(j = 0; j < dlen; j++)
			{
				report.broadcast_cnt += get_1byte_bit1_number(data[j],8);
			}
            return (0);
		}
        if (tst_bit(data[get_bits(_sid, 3, 7)], _sid & 0x07))
		{
			for(j=0; j<get_bits(_sid, 3, 7); j++)
			{
				report.broadcast_cnt += get_1byte_bit1_number(data[j],8);
			}
			report.broadcast_cnt += get_1byte_bit1_number(data[get_bits(_sid, 3, 7)], sid_mod_8);
			report.broadcast_my_seq = report.broadcast_cnt;
            return (1);
		}
		else
		{
			for(j=0; j<dlen; j++)
			{
				report.broadcast_cnt += get_1byte_bit1_number(data[j],8);
			}
			return (0);
		}
    }
    else			    /* bytes type */
    {
        int i;
        int gid_unit_len = group_type == 1 ? 1 : 2;

        for (i = 0; i < dlen; i += gid_unit_len)
        {
			report.broadcast_cnt++;
            int _gid = get_le_val(data + i, gid_unit_len);
            if ((_gid == 0) || (_sid == _gid))
			{
				report.broadcast_my_seq = report.broadcast_cnt;
                return (1);
			}
        }
    }

    return 0;
}
//---------------------------------------------------------------------------------------
//广播控制指令的解析与执行
int do_group_cmd(const sdk_evt_rx_plc_t *info, int resp, uint8_t *data, int len, uint8_t *said)
{
	int inidx = 0, gid_len;
	report.broadcast_cnt = 0;
    gid_len = get_bits(data[inidx], 0, 5) + 1;

    while (len >= FBD_FRAME_HEAD + gid_len)
    {
        struct Body *body = (struct Body *) &data[inidx + gid_len];
        int body_len = gid_len + FBD_FRAME_HEAD + get_bits(body->ctrl, 0, 6);

        if (len < body_len)
            break;
		
        if (is_gid_equal(&data[inidx], repeater.sid))
        {
            const struct func_ops *op = get_option(get_le_val(body->did, sizeof(body->did)));
            int (*handler) (const uint8_t * src, struct Body * body) = NULL;
            handler = op ? (resp ? op->write_resp : op->write) : NULL;
            if (handler)
            {
				memcpy(ram_data.last_ctrl_aid, said, AID_LEN);   
            	//add_last_aid(said, (uint8_t*)info->src);		//情景模式不认为是订阅者
                pr_info("Do cmd[%s]...\n", op->tip);
                int ret = handler(info->src, body);
				if((op->did == 0x0A01 || op->did == 0x0A03 || op->did == 0x0A04) && (NO_ERR == ret))
				{		
					report.broadcast_ctrl = 1;
				}
            }
        }
        inidx += body_len;
        len -= body_len;
        gid_len = get_bits(data[inidx], 0, 5) + 1;
    }

    return 0;
}
//---------------------------------------------------------------------------------------
static int rec_gw_ack(struct SmartFrame *pframe)
{
	if(pframe->seq & 0x80)
	{
		if((CMD_RELI_REPORT == (pframe->data[0])) && (0x00 == memcmp(repeater.gid, pframe->said, AID_LEN)))
	    {
			report.report_to_gw = FINISH;
	    	memset(report.rpt_7E_buf, 0x00, sizeof(report.rpt_7E_buf));
	    	return 1;
	    }
		else
			return 0;
	}
	else
		return 0;
}
//---------------------------------------------------------------------------------------
static int plc_frame_hook(const sdk_evt_rx_plc_t *info, struct SmartFrame *pframe)
{
    int ret = -1;
    struct AppFrame *app = (struct AppFrame *)pframe->data;     //获取报文内容

    int len = pframe->len - sizeof(struct AppFrame);            //获取报文总长度
    if(rec_gw_ack(pframe))//过滤网关主动上报（需要网关回复）上报的错误帧报文
    	return 0;
    switch (app->cmd)     //根据报文类别选择不同执行步骤
    {
    case CMD_GET://查询
    case CMD_SET://设置
        if (is_all_xx(pframe->taid, 0xff, AID_LEN) && app->cmd == CMD_SET)//如果发送者的目的地址taid全部是0xff并且指令模式为设置指令则进入组操作模式
            ret = do_group_cmd(info, tst_bit(pframe->seq, 7), app->data, len, pframe->said);
        else//进入单项操作模式
            ret = do_cmd(info, app->cmd, tst_bit(pframe->seq, 7), app->data, len, pframe->said); 
        break;
    case CMD_DEV_UPDATE:
    case CMD_NET_UPDATE://远程升级
        {
            if (tst_bit(pframe->seq, 7)) 
                return -1;
            ret = do_update(app->data, len);
        }
        break;
    case CMD_SHOW://搜索设备
    {
    	ret = search_frame_opt(info, pframe);
    }
    	break;
    default:
        break;
    }
    if (ret > 0 && !is_all_xx(pframe->taid, 0xFF, sizeof(pframe->taid))) //报文应答机制
    {
        ret += sizeof(struct AppFrame);
        code_ret_frame(pframe, ret);
        return do_send_frame((uint8_t *)info->src, pframe);
    }
    return ret;
}
//---------------------------------------------------------------------------------------
int do_plc_frame(const sdk_evt_rx_plc_t *info, struct SmartFrame *pframe)
{
    if (info->key_mod == SDK_KEY_INIT && !is_lowpw_frame(pframe))//若报文无强密码密码过滤掉需要强密码才能执行的报文
    {
        pr_info("Error: this frame need strong password\n");
        return 0;
    }

    return plc_frame_hook(info, pframe);
}
//---------------------------------------------------------------------------------------
static void state_report(struct soft_timer *st)
{
	if(report.init_flag)
	{
		report.report_to_sub = UNFINISH;
		report.report_to_gw = FINISH;
		report.init_flag = 0;
	}
	static uint8_t i = 0;
	st->expires = sec_jiffies + SEC_HZ;
	if(report.report_to_sub == UNFINISH)
	{
		while(i < MAX_CTRL_NUM)
		{
			if(is_all_xx(ram_data.ctrl_meg[i].aid, 0x00, AID_LEN) || memcmp(ram_data.ctrl_meg[i].aid, repeater.gid, AID_LEN) == 0)
			{
				i++;
				continue;
			}
			if(report.body[3] != dev_state.target_open_percent)								//receive new value
			{
				report.report_to_sub = FINISH;
				return;
			}
			report.len_7E_buf = code_frame(repeater.aid, ram_data.ctrl_meg[i].aid, -1, CMD_NRELI_REPORT, report.body, report.body_len, report.rpt_7E_buf, sizeof(report.rpt_7E_buf));
			do_send_frame(ram_data.ctrl_meg[i].mac, (struct SmartFrame *)report.rpt_7E_buf);
			i++;
			break;
		}
		if(report.report_to_sub == UNFINISH)
		{
			soft_timer_add(st);
		}
		if(MAX_CTRL_NUM == i)
		{
			i = 0;
			report.report_to_sub = FINISH;
			report.body_len += code_body(0xC01A, 0, report.ctrl_aid, AID_LEN, report.body + report.body_len, sizeof(report.body));
		}
	}
	else if(report.report_to_sub == FINISH)
	{
		//if(uart_list.count > 0 && report.first_read == 0)
		//	return;
		if(report.body[3] != dev_state.target_open_percent)
		{
			return;
		}
		if(!is_all_xx(repeater.gid, 0x00, AID_LEN))
		{
			report.report_to_gw = UNFINISH;
			report.len_7E_buf = code_frame(repeater.aid, repeater.gid, -1, CMD_RELI_REPORT , report.body,report.body_len, report.rpt_7E_buf, sizeof(report.rpt_7E_buf));
			do_send_frame(repeater.gmac, (struct SmartFrame *)report.rpt_7E_buf);
		}
	}
}

struct soft_timer start_state_report =
{
	.timer_cb = state_report,
};
//---------------------------------------------------------------------------------------
static void identify_controllor()
{
	if(dev_state.ctrl_flag == GW_CTRL_FLAG && !rx_uart.motor_state && dev_state.rx_plc_motor_stop)
		memcpy(report.ctrl_aid, repeater.gid, AID_LEN);
	else if(dev_state.ctrl_flag == GW_CTRL_FLAG && !rx_uart.motor_state && !dev_state.rx_plc_motor_stop
			&& ((dev_state.target_open_percent-2) <= dev_state.realtime_open_percent
			&&  (dev_state.target_open_percent+2) >= dev_state.realtime_open_percent))
	{
		dev_state.realtime_open_percent = dev_state.target_open_percent;
		memcpy(report.ctrl_aid, repeater.gid, AID_LEN);
	}
	
	else if(dev_state.ctrl_flag == SUB_CTRL_FLAG && !rx_uart.motor_state && dev_state.rx_plc_motor_stop)	//receive  stop CMD
		memcpy(report.ctrl_aid, ram_data.last_ctrl_aid, AID_LEN);
	else if(dev_state.ctrl_flag == SUB_CTRL_FLAG && !rx_uart.motor_state && !dev_state.rx_plc_motor_stop	//normal procedure
			&& ((dev_state.target_open_percent-2) <= dev_state.realtime_open_percent
			&&  (dev_state.target_open_percent+2) >= dev_state.realtime_open_percent))
	{
		dev_state.realtime_open_percent = dev_state.target_open_percent;
		memcpy(report.ctrl_aid, ram_data.last_ctrl_aid, AID_LEN);
	}
	else
	{
		memset(report.ctrl_aid, 0x00, AID_LEN);
		if(rx_uart.motor_state == 0x01)
			dev_state.target_open_percent = 0x00;
		else if(rx_uart.motor_state == 0x02)
			dev_state.target_open_percent = 0x64;
	}
}
//---------------------------------------------------------------------------------------
void do_send_report(uint8_t *data, uint32_t len)
{
	uint8_t old_state = 0xFF;
	static uint8_t first_read = 1;

//    pr_info("do_send_report->len %d --------------------------:\n", len);
    print_debug_array(KERN_INFO, data, len);
	
	if(ACK_FRAME_LEN == len)								//receive ack frame
	{
		struct UART_FRAME frame;
		get_uart_order(&frame);
		return;
	}
	if(0x0A == len)
		return;	

	old_state = dev_state.realtime_open_percent;
	if(STATE_FRAME_LEN == len && 0xA1 == data[1])									//receive valid data frame
	{
		if(uart_list.count > 0 && first_read == 0)
		{
//            pr_info("report Exit ERROR---------------------\n");
			return;
		}
		
		if(first_read)																//used for power on report
		{
			struct UART_FRAME frame;
			get_uart_order(&frame);
			first_read = 0;
//            pr_info("first_read is %d---------------------\n", first_read);
		}
		rx_uart.motor_state = data[3] & 0x03;	
//        pr_info("Get Flage's state----do_send_report-------:motor_state \n");
		dev_state.motor_state = (data[3] & 0x03) + 1;			//protocol matching
		if(rx_uart.motor_state != 0x00)
		{
			if(dev_state.ctrl_flag == NO_CTRL_FLAG)
			{
				if(rx_uart.motor_state == 0x01)
					dev_state.target_open_percent = 0x00;
				else if(rx_uart.motor_state == 0x02)
					dev_state.target_open_percent = 0x64;
			}
			return;
		}
		dev_state.realtime_open_percent = data[4];				//Curtain opening degree
	}
	else
	{
		return;
	}
	
	identify_controllor();
	if(old_state == INIT_DATA)												//power on report， 0xFF
	{
		memcpy(report.ctrl_aid, repeater.aid, AID_LEN);
		dev_state.target_open_percent = 0xFF;
		dev_state.realtime_open_percent = 0xFF;
	}
	
	if(rx_uart.motor_state == 0)
	{
		dev_state.ctrl_flag = NO_CTRL_FLAG;
		if(dev_state.rx_plc_motor_stop == FLAGON)
		{
			dev_state.target_open_percent = data[4];
			dev_state.rx_plc_motor_stop = FLAGOFF;
		}
	}
	
	report.body_len  = code_body(0x0A03, 0, &dev_state.target_open_percent, 1, report.body, sizeof(report.body));
	report.body_len += code_body(0x0A05, 0, &dev_state.realtime_open_percent, 1, report.body + report.body_len, sizeof(report.body));
	report.init_flag = 1;
	
	soft_timer_del(&start_state_report);
	if(report.broadcast_ctrl == 1)
	{
		report.broadcast_ctrl = 0;
		start_state_report.expires = sec_jiffies + ((5+(report.broadcast_my_seq*2))*SEC_HZ);
		report.broadcast_my_seq = 0;
	}
	else
		start_state_report.expires = sec_jiffies + 5 * SEC_HZ;
	soft_timer_add(&start_state_report);	
}
//---------------------------------------------------------------------------------------
static int uart_resend_check(const uint8_t *data, uint32_t len)
{
    static uint32_t last_tm;
	static uint8_t rx_data[15] = {0};
    uint32_t cur_tm = sec_jiffies; 
	
    if (!memcmp(data, rx_data, len) && time_after(last_tm+SEC_HZ/2, cur_tm)) 
    {
//        pr_info("Receivd dummy uart frame, drop it!\n");
        return -1;
    }
    memcpy(rx_data, data, len);
    last_tm = cur_tm;
    return 0;
}
uint8_t get_uart_smart_frame(uint8_t *data, uint32_t len)
{
	report.uart_busy = 0;
	if(len != ACK_FRAME_LEN && len != STATE_FRAME_LEN && len != 0x0A)
		return 0;
	if(data[0] != START_BYTE)
		return 0;
	if(data[len-1] != XorCheack(data, len - 1))
		return 0;
	if(uart_resend_check(data, len) < 0)
		return 0;
	return 1;
}
//---------------------------------------------------------------------------------------
struct UART_LIST uart_list;
uint8_t insert_uart_order(struct UART_FRAME *frame)
{
    if (uart_list.count > 0 ) 
    {
        if (!memcmp(&uart_list.frame[uart_list.count-1], frame, 5))
            return (0);
    }
    if (uart_list.count < MAX_UART_LIST)
    {
    	memcpy(&uart_list.frame[uart_list.count], frame, sizeof(struct UART_FRAME));
//        pr_info("insert_uart_order %d --------------------------:\n", uart_list.count);
//        print_debug_array(KERN_INFO, frame, sizeof(struct UART_FRAME));

        uart_list.count++;

        return (1);
    }
    return (0);
}

uint8_t get_uart_order(struct UART_FRAME *frame)
{
    if(uart_list.count > 0)
    {
        memcpy(frame, &uart_list.frame[0], uart_list.frame[0].data_len + 4);
        memmove(&uart_list.frame[0], &uart_list.frame[1], uart_list.count*sizeof(struct UART_FRAME));

//        pr_info("get_uart_order %d --------------------------:\n", uart_list.count);
//        print_debug_array(KERN_INFO, frame, sizeof(struct UART_FRAME));


        uart_list.count--;

        return (1);
    }
    return (0);
}

uint8_t peek_uart_order(struct UART_FRAME *frame)
{
    if(uart_list.count > 0)
    {
        
        memcpy(frame, &uart_list.frame[0], sizeof(struct UART_FRAME));

//        pr_info("peek_uart_order %d-------------------!\n", uart_list.count);
//        print_debug_array(KERN_INFO, frame, sizeof(struct UART_FRAME));


        return (1);
    }
    return (0);
}

