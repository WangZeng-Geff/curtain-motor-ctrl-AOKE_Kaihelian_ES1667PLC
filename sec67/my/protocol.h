#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include <stdint.h>
#include "sdk_evt.h"

//---------------------------------------------------------------------------------------
#define STC             0x7E
#define DID_LEN         0x02
#define SID_LEN         0x02
#define AID_LEN         0x04
#define PANID_LEN		0x02
#define BUF_LEN			0xFF
#define EID_LEN			0x08
#define PW_LEN          0x02
#define PSK_LEN 		0x08
#define SN_LEN          0x0C
#define DKEY_LEN        0x08
#define MAC_LEN			0x06

//---------------------------------------------------------------------------------------
#define CMD_SET_AID     0x01
#define CMD_GET_AID     0x03
#define CMD_ACK_AID     0x13
#define CMD_DEL_AID     0x04
#define CMD_REQ_AID     0x05
#define CMD_GET_SID     0x06
#define CMD_ACK_SID     0x16
#define CMD_GET_EID     0x07
#define CMD_ACK_EID     0x17
#define CMD_SET_BPS     0x08
#define CMD_SET_REG     0x09
#define CMD_UNLINK      0x0A
#define CMD_REGINFOR    0x0B
#define CMD_SET_PANID   0x0C
#define CMD_GET_GWAID   0x0D
#define CMD_GET_VER     0x0E
#define CMD_ACK_VER     0x1E
#define CMD_GET_PANID   0x0F
#define CMD_ACK_PANID   0x1F
#define CMD_TST_PLC     0x20
#define CMD_CHG_TONE    0x21

#define CMD_ACK         0x00
#define CMD_NAK         0xFF

#define CMD_SET         0x07
#define CMD_GET        	0x02
#define CMD_SHOW		0x03
#define CMD_DEV_UPDATE      0x05 /*APP UPDATE 0x05, NETWORK UPDATE 0x06*/
#define CMD_NET_UPDATE      0x06
#define CMD_RELI_REPORT     0x01
#define CMD_NRELI_REPORT    0x00
//---------------------------------------------------------------------------------------
#define NO_CTRL_FLAG		0x00
#define GW_CTRL_FLAG        0x01
#define SUB_CTRL_FLAG		0x02
#define MAX_CTRL_NUM		0x03
//---------------------------------------------------------------------------------------
#define NO_ERR        0x00
#define OTHER_ERR     0x0F
#define LEN_ERR       0x01
#define BUFFER_ERR    0x02
#define DATA_ERR      0x03
#define DID_ERR       0x04
#define DEV_BUSY      0x05
#define NO_RETURN     0x10
//---------------------------------------------------------------------------------------
#define UNFINISH      0x00
#define FINISH    	  0x01
#define FLAGOFF		  0x00
#define FLAGON		  0x01
#define STOP		  0x01
#define OPENING		  0x02
#define CLOSEING	  0x03
//---------------------------------------------------------------------------------------
#define START_BYTE			0x9A
#define READ_WORKING_STATE  0xA1

#define INIT_DIR	0x00
#define REVE_DIR	0x01

#define INIT_DATA		0xEE
#define ACK_FRAME_LEN	0x05
#define STATE_FRAME_LEN	0x07		
#define MAX_UART_LIST	128

#pragma pack(1)


struct SmartFrame
{
    uint8_t stc;
    uint8_t said[AID_LEN];
    uint8_t taid[AID_LEN];
    uint8_t seq;
    uint8_t len;
    uint8_t data[0];
};
#define SMART_FRAME_HEAD sizeof(struct SmartFrame)

struct AppFrame
{
    uint8_t cmd;
    uint8_t data[0];
};
struct GroupFrame
{
    uint8_t len : 6;
    uint8_t type: 2;
    uint8_t data[0];
};

struct Body
{
    uint8_t did[2];
    uint8_t ctrl;
    uint8_t data[0];
};
#define FBD_FRAME_HEAD  sizeof(struct Body)

struct RegData
{
    uint8_t aid[AID_LEN];
    uint8_t panid[SID_LEN];
    uint8_t pw[PW_LEN];
    uint8_t gid[AID_LEN];
    uint8_t sid[SID_LEN];
};

struct Report
{
	uint8_t rpt_7E_buf[0xC8];
	uint8_t len_7E_buf;
	uint8_t report_to_sub;
	uint8_t report_to_gw;
	//uint8_t rpting;
	uint8_t ctrl_aid[AID_LEN];
	uint8_t body[0x20];
	uint8_t body_len;
	uint8_t uart_busy :1;
	//uint8_t ctrl_busy :1;
	//uint8_t first_read;
	uint8_t init_flag :1;
	
	uint8_t broadcast_ctrl;
	uint8_t broadcast_cnt;
	uint8_t broadcast_my_seq;
};
//-----------------------------------------------
struct DEV_STATE
{
	uint8_t realtime_open_percent;
	uint8_t target_open_percent;
	uint8_t rx_plc_motor_stop;
	uint8_t motor_state;
	uint8_t ctrl_flag;				//控制者：网关或其他订阅者
};

struct UART_FRAME
{
	uint8_t start;
	uint8_t cmd;
	uint8_t data_len;
	uint8_t data[7];
};
struct UART_LIST
{
	uint8_t count;
	struct UART_FRAME frame[MAX_UART_LIST];
};
#pragma pack()

extern struct DEV_STATE dev_state;
extern struct DEV_STATE rx_uart;
extern struct Report report;
extern struct UART_LIST uart_list;

#define frame_len(frame) (SMART_FRAME_HEAD + frame->len + 1)

struct SmartFrame *get_smart_frame(const uint8_t *in, int len);
int is_my_smart_frame(const struct SmartFrame *frame);
int code_body(uint16_t did, int err, const void *data, int len, void *out, int maxlen);
int code_frame(const uint8_t *src, const uint8_t *dest, int seq, int cmd, 
    const uint8_t *data, int len, void *out, int maxlen);
int code_local_frame(const uint8_t *in, int len, void *out, int maxlen);
void do_evt_tst_resp(const sdk_evt_tst_resp_t *resp);

int code_ret_frame(struct SmartFrame *pframe, int len);
int smart_frame_handle(struct SmartFrame *frame);
int adaptor_on_uart1_received(const uint8_t *data, int len);
int do_plc_frame(const sdk_evt_rx_plc_t *info, struct SmartFrame *pframe);
int resend_check(const uint8_t *src, const struct SmartFrame *pframe);
int do_send_frame(uint8_t *mac, struct SmartFrame *frame);
int is_gid_equal(const uint8_t * data, const uint8_t * sid);
void do_send_report(uint8_t *data, uint32_t len);

uint8_t insert_uart_order(struct UART_FRAME *frame);
uint8_t get_uart_order(struct UART_FRAME *frame);
uint8_t peek_uart_order(struct UART_FRAME *frame);
uint8_t code_uart_frame(uint8_t cmd, uint8_t *data, uint8_t data_len);
uint8_t get_uart_smart_frame(uint8_t *data, uint32_t len);

#endif
