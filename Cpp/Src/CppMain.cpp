// //
// // Created by 57500 on 2026/2/27.
// //
#include "CppMain.h"

#include <cstdio>
#include <cstring>

#include "can.h"
#include "MotorControl.h"
#include "Forward_Kinematics.h"
#include "Inverse_Kinematics.h"
#include "RobotState.h"
#include "Trajectory_Planning.h"
#include "usart.h"
#include "main.h"
#include "NewBee.h"

// 启用DWT
void Init_DWT(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// //测时专用
// DWT->CYCCNT = 0;  // 清零计数器
// uint32_t start_cycles = DWT->CYCCNT;
//
// //->测时内容
//
//
// uint32_t end_cycles = DWT->CYCCNT;
// uint32_t total_cycles = end_cycles - start_cycles;
//
// // 转换为时间（单位：微秒）
// float time_us = (float)total_cycles * 1000000.0f / SystemCoreClock;
// // 或者毫秒
// float time_ms = (float)total_cycles * 1000.0f / SystemCoreClock;
//
// // 输出结果
// char buffer[64];
// sprintf(buffer, "执行时间: %.3f ms (周期数: %lu)\r\n", time_ms, total_cycles);
// HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);


static NewBee nb;

void CppMain(void)
{
    Init_DWT();

    while (1)
    {
        if (Tim_Flag==1)
        {
            if (nb.Get_Current_Step()>0)
            {
                nb.UpDate();
            }
            Tim_Flag=0;
        }

        if (uart1.parse_flag==1)
        {
            uart1.parse_flag=0;
            if (uart1.rx_buffer[0]=='0'&&uart1.rx_buffer[1]=='\0')
            {
                nb.Emergency_Stop();
                UART_SendFloat_sprintf(&huart1,1,1);
            }
            else if (nb.Get_Current_Step()==0)
            {
                char *token;
                int count = 0;
                float temp[6];

                // 以此逗号 "," 为分隔符获取第一个数据
                token = strtok(uart1.rx_buffer, ",");

                while(token != NULL && count < 6)
                {
                    //将字符串转为浮点数
                    temp[count] = atof(token);
                    count++;

                    //以此逗号 "," 为分隔符获取下一个数据
                    token = strtok(NULL, ",");
                }

                if (count == 6)
                {
                    Coordinates_Pose target;
                    target.x=temp[0];
                    target.y=temp[1];
                    target.z=temp[2];
                    target.alpha=temp[3];
                    target.beta=temp[4];
                    target.gamma=temp[5];

                    nb.UpDate_Current_Angle_Rad();
                    nb.UpDate_Current_CP();
                    nb.UpDate_Target_CP(target);
                    nb.UpDate_S_Curve_Profile();
                }
            }
        }
    }
}
