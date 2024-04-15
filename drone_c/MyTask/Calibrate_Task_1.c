#include "Calibrate_task.h"
#include "cmsis_os.h"
#include "bmi088driver.h"

//cali_sensor_t       cali_sensor_gyro;
static imu_cali_t   gyro_cali;       //gyro cali data������У׼����
static uint32_t *cali_sensor_buf = (uint32_t *)&gyro_cali;//gyro_caliУ׼���ݵ�ַ
char calibrate_done_flag=0;

/**
  * @brief          �������豸У׼����
  * @param[in][out] cali:ָ��ָ������������,��cmdΪCALI_FUNC_CMD_INIT, ����������,CALI_FUNC_CMD_ON,���������
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: �����ʼ��
                    CALI_FUNC_CMD_ON: ����ʼУ׼
  * @retval         0:У׼����û�����
                    1:У׼�����Ѿ����
  */
static uint16_t count_time = 0;
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
{
    imu_cali_t *local_cali_t = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        INS_set_cali_gyro(local_cali_t->scale, local_cali_t->offset);    
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
				INS_cali_gyro(local_cali_t->scale, local_cali_t->offset, &count_time);

        if (count_time > GYRO_CALIBRATE_TIME)
        {
            count_time = 0;
//            cali_buzzer_off();
						RC_restart(SBUS_RX_BUF_NUM);
            return 1;
        }
        else
        {
						RC_unable();
//            imu_start_buzzer();    
            return 0;
        }
    }

    return 0;//�������е�����
}


extern TaskHandle_t Calibrate_task_handle;//������
extern bmi088_real_data_t bmi088_real_data;//��ȡ�¶�
void Calibrate_task(void const * argument)
{
	
	cali_gyro_hook(cali_sensor_buf,CALI_FUNC_CMD_INIT);
	
	//�¶ȴﵽ44�Ȳſ�ʼ�궨
	while(bmi088_real_data.temp<44.0f)
	{
		osDelay(100);
	}
	
	while(1)
	{
		if(cali_gyro_hook(cali_sensor_buf,CALI_FUNC_CMD_ON))
		{
			calibrate_done_flag=1;
			vTaskDelete(Calibrate_task_handle);
		}
		
	osDelay(CALIBRATE_CONTROL_TIME);
		
	}
}

