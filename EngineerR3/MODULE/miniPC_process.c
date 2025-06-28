#include "miniPC_process.h"
#include "string.h"
#include "robot_def.h"
#include "daemon.h"

static Nac_Instance *nac_instance; // ���ں��Ӿ�ͨ�ŵĴ���ʵ��
static uint8_t *vis_recv_buff __attribute__((unused));
static Daemon_Instance *nac_daemon_instance;
// ȫ�ֱ�����
extern uint16_t CRC_INIT;
/**
 * @brief �����Ӿ����������
 *
 * @param recv
 * @param rx_buff
 */
static void RecvProcess(Nac_Recv_s *recv, uint8_t *rx_buff)
{
	recv->basket_angle     = (rx_buff[2]<<8)|rx_buff[3];
	recv->basket_distance  = (rx_buff[4]<<8)|rx_buff[5];
	recv->robo_angle       = (rx_buff[6]<<8)|rx_buff[7];
	recv->robo_distance    = (rx_buff[8]<<8)|rx_buff[9];

}

/**
 * @brief �ص�������ȷ��֡ͷ�����ڽ����Ӿ�����
 *
 */
static void DecodeNac()
{
    DaemonReload(nac_daemon_instance); // ι��


    if (nac_instance->usart->recv_buff[0] == nac_instance->recv_data->header1 && nac_instance->usart->recv_buff[1] == nac_instance->recv_data->header2 &&nac_instance->usart->recv_buff[10] == nac_instance->recv_data->tail) {
        // ��ȡ�Ӿ�����
        RecvProcess(nac_instance->recv_data, nac_instance->usart->recv_buff);
    }

}


/**
 * @brief ���߻ص�����,����daemon.c�б�daemon task����
 * @attention ����HAL����������,���ڿ���DMA����֮��ͬʱ�����и��ʳ���__HAL_LOCK()���µ�����,ʹ���޷�
 *            ��������ж�.ͨ��daemon�ж����ݸ���,���µ��÷������������Խ��������.
 *
 * @param id vision_usart_instance�ĵ�ַ,�˴�û��.
 */
static void VisionOfflineCallback(void *id)
{

    USARTServiceInit(nac_instance->usart);

}



/**
 * @brief �������ݴ�����
 *
 * @param send ����������
 * @param tx_buff ���ͻ�����
 *
 */
static void SendProcess(Nac_Send_s *send, uint8_t *tx_buff)
{
    /* ����֡ͷ��Ŀ����ɫ���Ƿ����õ����� */
}

/**
 * @brief ���ͺ���
 *
 * @param send ����������
 *
 */
void NacSend()
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    SendProcess(nac_instance->send_data, send_buff);
    USARTSend(nac_instance->usart, send_buff, VISION_SEND_SIZE, USART_TRANSFER_BLOCKING);
}

/**
 * @brief ����ע��һ���Ӿ��������ݽṹ��,����һ���Ӿ��������ݽṹ��ָ��
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Nac_Recv_s *NacRecvRegister(Nac_Recv_Init_Config_s *recv_config)
{
    Nac_Recv_s *recv_data = (Nac_Recv_s *)malloc(sizeof(Nac_Recv_s));
    memset(recv_data, 0, sizeof(Nac_Recv_s));

    recv_data->header1 = recv_config->header1;
		recv_data->header2 = recv_config->header2;
		recv_data->tail = recv_config->tail;

    return recv_data;
}

/**
 * @brief ����ע��һ���Ӿ��������ݽṹ��,����һ���Ӿ��������ݽṹ��ָ��
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Nac_Send_s *NacSendRegister(Nac_Send_Init_Config_s *send_config)
{
    Nac_Send_s *send_data = (Nac_Send_s *)malloc(sizeof(Nac_Send_s));
    memset(send_data, 0, sizeof(Nac_Send_s));

    send_data->header   = send_config->header;
    send_data->tail     = send_config->tail;
    return send_data;
}

Nac_Recv_s *NacInit(UART_HandleTypeDef *nac_usart_handle)
{
    nac_instance = (Nac_Instance *)malloc(sizeof(Nac_Instance));
    memset(nac_instance, 0, sizeof(Nac_Instance));
	
		USART_Init_Config_s conf;
    conf.module_callback = DecodeNac;
    conf.recv_buff_size  = VISION_RECV_SIZE;
    conf.usart_handle    = nac_usart_handle;

    nac_instance->usart                = USARTRegister(&conf);
    Nac_Recv_Init_Config_s recv_config = {
        .header1 = NAC_RECV_HEADER1,
			  .header2 = NAC_RECV_HEADER2,
			  .tail    = NAC_RECV_TAIL,
    };

    nac_instance->recv_data            = NacRecvRegister(&recv_config);
    Nac_Send_Init_Config_s send_config = {
        .header        = NAC_SEND_HEADER,
        .tail          = NAC_SEND_TAIL,
    };
    nac_instance->send_data = NacSendRegister(&send_config);
    // Ϊmaster processע��daemon,�����ж��Ӿ�ͨ���Ƿ�����
    Daemon_Init_Config_s daemon_conf = {
        .callback     = VisionOfflineCallback, // ����ʱ���õĻص�����,���������ڽ���
        .owner_id     = NULL,
        .reload_count = 5, // 50ms
    };
    nac_daemon_instance = DaemonRegister(&daemon_conf);

    return nac_instance->recv_data;
}
