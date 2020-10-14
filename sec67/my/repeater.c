#include "include.h"
#include "sdk_svc.h"
#include "sdk_evt.h"
#include "timer.h"
#include "gpio.h"
#include "uart.h"
#include "comfunc.h"
#include "sec_printf.h"
#include "cmd.h"
#include "update.h"
#include "protocol.h"
#include "repeater.h"
#include "sec_tick.h"
#include "esuart.h"
#include "softtimer.h"
#include "gpio.h"
#include "task.h"

struct RAM_DATA ram_data;
//-------------------------------------------------------------------------------------------------
const char *repeater_plc_type = PLC_MATERIAL_CODE;
const char *comm_protocol_ver = COMM_PROTOCOL_VER;
const char *repeater_ext_ver = REPEATER_VERSION_EXT;

const uint8_t plc_dev_type[8] = {0xFF, 0xFF, 0x32, 0x00, 0x04, 0x00, 0x00, 0x01};
const uint8_t repeater_dev_type[8] = {0xFF, 0xFF, 0x04, 0x00, 0x01, 0x00, 0x05, 0x00};
//-------------------------------------------------------------------------------------------------
struct Repeater repeater;
//-------------------------------------------------------------------------------------------------
int get_dev_type(uint8_t *type)
{
    memcpy(type, repeater_dev_type, sizeof(repeater_dev_type));
    return sizeof(repeater_dev_type);
}
int get_plc_type(uint8_t *type)
{
    memcpy(type, plc_dev_type, sizeof(plc_dev_type));
    return sizeof(plc_dev_type);
}
int get_dev_type_by_layer(enum LAYER_TYPE layer, uint8_t *dev_type)
{
    switch (layer) 
    {
    case LAYER_PLC:
        return get_plc_type(dev_type);
    case LAYER_REPEATER:
        return get_dev_type(dev_type);
    default:
        return -1;
    }
}
enum LAYER_TYPE get_layer_by_dev_type(const uint8_t *dev_type) 
{
    if (!memcmp(repeater_dev_type, dev_type, sizeof(repeater_dev_type))) 
    {
        return LAYER_REPEATER;
    }
    else if (!memcmp(plc_dev_type, dev_type, sizeof(plc_dev_type))) 
    {
        return LAYER_PLC;
    }
    return LAYER_NR;
}
//-------------------------------------------------------------------------------------------------
const char *get_dev_ver(void)
{
    static char dev_ver[0x40];

    snprintf(dev_ver, sizeof(dev_ver), "ESACT-CM-1/12-PL-C21(v%01x.%01x)-20%06x",
             get_bits(REPEATER_VERSION, 8, 15), get_bits(REPEATER_VERSION, 0, 7), 
             get_bits(REPEATER_DATE, 0, 23));
    return dev_ver;
}
const char *get_plc_ver(void)
{
    sdk_err_t err;
    sdk_svc_ver_t ver;
    static char plc_ver[0x40];

    if ((err = sdk_svc_get_ver(&ver)) != SDK_ERR_OK) 
    {
        pr_err("sdk_svc_get_ver failed, err[%d]\n", err);
        return NULL;
    }

    snprintf(plc_ver, sizeof(plc_ver), "SSC1667-PLC(v%01x.%x)-20%06x", 
             get_bits(ver.ver, 8, 15), get_bits(ver.ver, 0, 7), 
             get_bits(ver.date, 0, 23));
    return plc_ver;
}
const char *get_soft_ver_by_layer(enum LAYER_TYPE layer)
{
    switch (layer) 
    {
    case LAYER_PLC:
        return get_plc_ver();
    case LAYER_REPEATER:
        return get_dev_ver();
    default:
        return NULL;
    }
}
//-------------------------------------------------------------------------------------------------
void repeater_show(void)
{
    sdk_svc_mac_t mac;
    sdk_svc_get_mac(&mac);
    memcpy(repeater.mac, mac.mac, MAC_LEN);
    esprintf("SID:[%02X %02X]\n", repeater.sid[0], repeater.sid[1]);
    
    esprintf("PANID:[%02X %02X]\n", repeater.panid[0], repeater.panid[1]);
    
    esprintf("AID:[%02X %02X %02X %02X]\n", repeater.aid[0], repeater.aid[1], repeater.aid[2], repeater.aid[3]);
   
    esprintf("MAC:[%02X:%02X:%02X:%02X:%02X:%02X]\n",
        mac.mac[0], mac.mac[1], mac.mac[2], mac.mac[3], mac.mac[4], mac.mac[5]);
 
    esprintf("DKEY:[%02X %02X %02X %02X %02X %02X %02X %02X]\n",
        repeater.dkey[0], repeater.dkey[1], repeater.dkey[2], repeater.dkey[3], 
        repeater.dkey[4], repeater.dkey[5], repeater.dkey[6], repeater.dkey[7]);
     
    esprintf("SN:[%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X]\n",
        repeater.sn[0], repeater.sn[1], repeater.sn[2], repeater.sn[3], repeater.sn[4], repeater.sn[5], 
        repeater.sn[6], repeater.sn[7], repeater.sn[8], repeater.sn[9], repeater.sn[10], repeater.sn[11]);

    esprintf("APP:[Ver:%s]\n", get_dev_ver()); 
    sdk_svc_ver_t plc;
    sdk_svc_get_ver(&plc);
    esprintf("PLC:[Manu:%04X,Chn:%02X,Mod:%02X,Config:%04X,Ver:%04X,Data:%08X]\n",
        plc.manu, plc.chl, plc.mod, plc.config, plc.ver, plc.date);
}
//-------------------------------------------------------------------------------------------------
void dev_info_init(void)
{
	struct deviceInfo info;
	memcpy(&info, (uint8_t*)(DEVICE_INFO_ADDR), sizeof(info));
    if (info.magic != DEVICE_INFO_MAGIC)
    {
        pr_emerg("devinfo invalid, need to be written!!!\n");
        return;
    }
	memcpy(repeater.dkey, info.dkey, DKEY_LEN);
	memcpy(repeater.sn, info.sn, SN_LEN);
	memcpy(repeater.aid, info.aid, AID_LEN);
	memcpy(repeater.passwd, info.passwd, PW_LEN);
	repeater.baud = info.baud;
	repeater.mod = info.mod;
    uart_config(UART_CHN_DM, repeater.baud, repeater.mod);
}
//-------------------------------------------------------------------------------------------------
static void dev_cfg_init(void)
{
	struct deviceCfgInfo cfg;
    memcpy(&cfg, (uint8_t*)DEVICE_CFG_ADDR, sizeof(cfg));
    if (cfg.magic == DEVICE_CFG_MAGIC)
    {
    	memcpy(repeater.panid, cfg.panid, PANID_LEN);
    	memcpy(repeater.gid, cfg.gid, AID_LEN);
    	memcpy(repeater.gmac, cfg.gmac, MAC_LEN);
    	memcpy(repeater.sid, cfg.sid, SID_LEN);
    }
	do_set_psk();
}
//-----------------------------------------------------------------------------------------------
static void dev_data_init()
{
    memcpy(&dev_data, (uint8_t*)AIR_DATA_ADDR, sizeof(dev_data));
    if (dev_data.magic != AIR_DATA_MAGIC)
    {
    	memset(&dev_data, 0, sizeof(dev_data));
    	dev_data.magic = AIR_DATA_MAGIC;
    	dev_data.location = INIT_DIR;
    	flash_user_info_write(AIR_DATA_ADDR, (uint8_t*)&dev_data, sizeof(dev_data));
    }
	dev_state.realtime_open_percent = INIT_DATA;
	dev_state.target_open_percent	= INIT_DATA;
	rx_uart.motor_state = INIT_DATA;
}
//-------------------------------------------------------------------------------------------------
void read_state_timer(struct soft_timer *st)
{
	uint8_t data1[1] ={0x00};
	//uint8_t data3[5] = {0x9A, 0XA1, 0X01, 0x00, 0x3A};
	st->expires = sec_jiffies + SEC_HZ * 10;
	
//	code_uart_frame(0xA1, data1, 1);

#if 0
	if(uart_list.count || dev_state.motor_state == OPENING || dev_state.motor_state == CLOSEING || report.uart_busy)
		return;
	else if(first_config == 0)
	{
		esuart_send_data(UART_CHN_DM, data3, 5);
		report.uart_busy = 1;
	}
#endif
}

struct soft_timer state_sync_timer =
{
    .timer_cb = read_state_timer,
	//.opt =  TIMER_ALWAYS_ALIVE,
};
static int state_sync(void)
{
	uint8_t rand; 
	
	rand = get_le_val(repeater.sid, SID_LEN)%100;
	state_sync_timer.expires = sec_jiffies + SEC_HZ*(60-6+rand); 
	//state_sync_timer.expires = sec_jiffies + SEC_HZ*10; 					//used for debug, for save time
    soft_timer_add(&state_sync_timer);
    return 0;
}
//-------------------------------------------------------------------------------------------------
static void task_50ms(struct soft_timer *st)
{
	static uint32_t report_time = 0;
	static uint8_t i = 0;
	extern struct task dev_control_task;
	
	st->expires = sec_jiffies + SEC_HZ/20;
	
	if((++i)%5== 0)
	{
		task_start(&dev_control_task);
	}
	
	if(report.report_to_gw == FINISH)
		report_time = 0;
	else if (report.report_to_gw == UNFINISH)
	{
		report_time++;
		if (report_time == 200)           										//second report
		{
			report.report_to_gw = UNFINISH;
			do_send_frame(repeater.gmac, (struct SmartFrame *)report.rpt_7E_buf); 
		}
		if (report_time == (200+2000))		 									//third report
		{
			report.report_to_gw = UNFINISH;
			do_send_frame(repeater.gmac, (struct SmartFrame *)report.rpt_7E_buf);
			memset(report.rpt_7E_buf,0x00, sizeof(report.rpt_7E_buf)); 
			report_time  = 0;
			report.report_to_gw = FINISH;
		}
	}
}
struct soft_timer task_50ms_timer =
{
    .timer_cb = task_50ms,
    .opt      = TIMER_ALWAYS_ALIVE,
};
static void task_50ms_init(void)
{
	report.report_to_gw = FINISH;
	task_50ms_timer.expires = sec_jiffies + SEC_HZ/20;
	soft_timer_add(&task_50ms_timer);
}
//-------------------------------------------------------------------------------------------------
void system_init(void)
{
    memset(&repeater, 0, sizeof(repeater));

    esuart_init();
    sec_cmd_init();
    dev_info_init();
    dev_cfg_init();
    update_init();
    sec_tick_init();
    task_50ms_init();
    repeater_show();
	dev_data_init();
    state_sync();	
}
//-------------------------------------------------------------------------------------------------
void repeater_reset(void)
{
    flash_user_info_erase(USER_ADDR, USER_LEN);
	sdk_svc_reset();
}
//-------------------------------------------------------------------------------------------------
int do_plc_broadcast(int key_mod, const void *data, int len)
{
    sdk_svc_brd_t info;

    pr_info("PLC[Broadcast]:\n");
    print_debug_array(KERN_INFO, data, len);

    info.src_mod = SDK_MOD_6364;
    info.dst_mod = SDK_MOD_6364;
    info.prio = SDK_PRIO_LOW;
    info.key_mod = (sdk_key_mod_t)key_mod;
    info.radius = 0; /* max radius */
    info.multi = SDK_MULTI_DIS;
    if (is_all_xx(repeater.panid, 0x00, sizeof(repeater.panid))) 
    {
        info.radius = 1;
    }
    sdk_err_t err = sdk_svc_brodcast(&info, (uint8_t *)data, len);
    if (err != SDK_ERR_OK)
    {
    	pr_info("sdk_svc_brodcast failed, err[%d]!\n", err);
    }
    
    return 0;
}
int do_plc_unicast(const uint8_t *dst, int key_mod, const void *data, int len)
{
    sdk_svc_uni_t info;

	char str[0x40];
	pr_info("PLC[Unicast to %s]:\n", arr2str(dst, 6, str, sizeof(str)));
    print_debug_array(KERN_INFO, data, len);

    info.src_mod = SDK_MOD_6364;
    info.dst_mod = SDK_MOD_6364;
    info.prio = SDK_PRIO_LOW;
    info.key_mod = (sdk_key_mod_t)key_mod;
    memcpy(info.dst, dst, sizeof(info.dst));
    info.multi = SDK_MULTI_DIS;
    
    sdk_err_t err = sdk_svc_unicast(&info, (uint8_t *)data, len);
    if (err != SDK_ERR_OK)
    {
        pr_info("sdk_svc_unicast failed, err[%d]!\n", err);
        return -1;
    }
    return 0;
}
int do_set_psk(void)
{
    sdk_err_t err;
    sdk_svc_psk_t info;

    memset(&info, 0, sizeof(info));
    info.len = 8;
    if (is_all_xx(repeater.panid, 0, sizeof(repeater.panid))) 
    {
        strcpy((char*)info.psk, "eastsoft");
    }
    else
    {
        memcpy(info.psk, repeater.panid, sizeof(repeater.panid));
    }

    err = sdk_svc_set_psk(&info);
    if (err != SDK_ERR_OK)
    {
		pr_info("got svc version failed, err[%d]!\n", err);
        return -1;
    }
    return 0;
}
int do_psk_resp(const uint8_t *dst, int state)
{
    sdk_svc_rmtpsk_resp_t info;

    info.src_mod = SDK_MOD_6364;
    info.dst_mod = SDK_MOD_6364;
    info.prio = SDK_PRIO_LOW;
    memcpy(info.dst, dst, sizeof(info.dst));
    info.state = (sdk_rmtpsk_state_t)state;
    info.multi = SDK_MULTI_DIS;

    /* deny networking */
    sdk_err_t err = sdk_svc_rmtpsk_resp(&info);
    if (err != SDK_ERR_OK)
    {
		pr_info("sdk_svc_rmtpsk_resp failed, err[%d]!\n", err);
        return -1;
    }
    pr_info("sdk_svc_rmtpsk_resp, state[%d]!\n", state);
    return 0;
}
//-------------------------------------------------------------------------------------------------
static int do_devprop_req(const uint8_t *src, int taskid, int state,
    const void *data, int len)
{
    sdk_err_t err;
    sdk_svc_devprop_resp_t info;

    info.src_mod = SDK_MOD_6364;
    info.dst_mod = SDK_MOD_6364;
    info.state = (sdk_sch_state_t)state;
    memcpy(info.src, src, sizeof(info.src));
    info.taskid = taskid;
    info.multi = SDK_MULTI_DIS;

    err = sdk_svc_devprop_resp(&info, (uint8_t *)data, len);
    if (err != SDK_ERR_OK)
    {
		pr_info("got svc version failed, err[%d]!\n", err);
        return -1;
    }
    pr_info("respond devprop frame[%s], att:\n", state == SDK_SCH_PARTICIPATE ? "PART" : "ABS");
    print_debug_array(KERN_INFO, data, len);
    return 0;
}
//-------------------------------------------------------------------------------------------------
void repeater_on_evt_devprop_req(const sdk_evt_devprop_req_t *req, const uint8_t *att, int len)
{
	pr_info("got devprop frame, att:\n");
    print_debug_array(KERN_INFO, att, len);

    int state = SDK_SCH_ABSENT;

    switch (len)
    {
    case 0:
        state = SDK_SCH_PARTICIPATE;
        break;
    case SID_LEN:   // att: PANID. Gateway first poweron
        if (!memcmp(att, repeater.panid, sizeof(repeater.panid)))
            state = SDK_SCH_PARTICIPATE;
        break;
    case AID_LEN:
        if (!is_all_xx(repeater.aid, 0x00, AID_LEN) && !memcmp(att, repeater.aid, AID_LEN))
            state = SDK_SCH_PARTICIPATE;
        break;
    default:
        break;
    }
    do_devprop_req(req->src, req->taskid, state, repeater.aid, sizeof(repeater.aid));
}

//-------------------------------------------------------------------------------------------------
//解析报文模式
//	Input		:	req - 请求参数
//					att - 请求中的属性数据
//					len - 属性数据长度
void repeater_on_evt_rx_plc(const sdk_evt_rx_plc_t *info, const uint8_t *data, int len)
{
	char str[0x40];
	pr_info("PLC[RX], mac[%s]:\n",
        arr2str(info->src, sizeof(info->src), str, sizeof(str)));
    print_debug_array(KERN_INFO, data, len);

	sdk_evt_rx_plc_t info_tmp;              //电力线接受数据参数
	memcpy(&info_tmp, info, sizeof(sdk_evt_rx_plc_t));
	
	uint8_t buf[300];                       //电力线接受报文内容报文长度
	len = len > sizeof(buf) ? sizeof(buf) : len;
	memcpy(buf, data, len);

    struct SmartFrame *frame = get_smart_frame(buf, len);   //获取一帧有效的电力线报文

    if (frame && is_my_smart_frame(frame))//如果接受报文不为空并且本地存在ID接收报文的目的接受者是本机
    {
        if (resend_check(info_tmp.src, frame) < 0)      //过滤重复报文帧
            return;
        do_plc_frame(&info_tmp, frame);//传入info_tmp：电力线接受参数  ； frame：一帧完整的电力线报文
    }
}
