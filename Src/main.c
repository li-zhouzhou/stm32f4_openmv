/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#undef __GNUC__
#ifdef __GNUC__
   #define PUTCHAR_PROTOTYPE int _io_putchar(int ch)
#else
	 #define PUTCHAR_PROTOTYPE int fputc(int ch,FILE *f)
#endif
	 
PUTCHAR_PROTOTYPE
	 {
		 HAL_UART_Transmit(&huart2,(uint8_t *)&ch,1,0xffff);
		 return ch;
	 }


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void measure(void);
void trans(int16_t data);	 
void measure6(void);
void trans6(int16_t data);	 
void MG996R_Set_Angle(int16_t angle);	 
void MG996R_Set_Angle2(int16_t angle2);
	 
int a=-80,b=-80;
int shape,size,model=1;
u32 value;
int keystatues1=0,keystatues2=0;
int M=0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
 
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

//oled参数设置
	float num = 13.14;
	char num_temp_buffer[16];		
  int i; 
	int degreex=0,degreey=0;
				
  OLED_Init();
	OLED_CLS();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

		HAL_TIM_Base_Start_IT(&htim4);
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
		 if(HAL_UART_Receive_IT(&huart6,(u8 *)&i,1) != HAL_OK)   Error_Handler();
    /* 开启接收错误中断 */
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);
	
			HAL_Delay(500);

		
//KEY1控制任务一、二 *****************************************8		
if(HAL_GPIO_ReadPin(key1_GPIO_Port,key1_Pin)==0){
			HAL_Delay(100);
			
			if(HAL_GPIO_ReadPin(key1_GPIO_Port,key1_Pin)==0){
	   	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	     HAL_Delay(500);
				keystatues1+=1;
				HAL_UART_ErrorCallback(&huart6);
				HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,RESET);
				OLED_CLS();
				shape=0;
				
			}
					
		}
		
		if(keystatues1==1){
            M=0;   //云台不转动
			  HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET);//关闭激光标记中心
				if(shape ==1){
					OLED_ShowStr(0, 0,"Shape:circle  ", 2);//显示字符串
					HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,SET);
				}
				else if(shape ==2){
					OLED_ShowStr(0, 0,"Shape:rectangle ", 2);//显示字符串
					HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,SET);
				}
				else if(shape ==3){
					OLED_ShowStr(0, 0,"Shape:triangle ", 2);//显示字符串
					HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,SET);
				}	
				
				OLED_ShowStr(0, 3, "Size:34.8cm", 2);//显示字符串
				OLED_ShowStr(0, 6, "Distance:", 2);//显示字符串		
				
				OLED_ShowNum(70,7,value,5,20);
   
	
 // HAL_GPIO_WritePin(beep_GPIO_Port, beep_Pin, GPIO_PIN_RESET);
		
	}else if(keystatues1==2){
		
		
		
		 M=0;   //云台不转动
			  HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET);//关闭激光标记中心
				if(shape ==1){
					OLED_ShowStr(0, 0,"Shape:circle  ", 2);//显示字符串
					HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,SET);
				}
				else if(shape ==2){
					OLED_ShowStr(0, 0,"Shape:rectangle ", 2);//显示字符串
					HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,SET);
				}
				else if(shape ==3){
					OLED_ShowStr(0, 0,"Shape:triangle ", 2);//显示字符串
					HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,SET);
				}	
				
				OLED_ShowStr(0, 3, "Size:36.2cm", 2);//显示字符串
				OLED_ShowStr(0, 6, "Distance:", 2);//显示字符串		
				
				OLED_ShowNum(70,7,value,5,20);
	
		
	}else if(keystatues1==3){		
		keystatues1=0;		
	}
	
//KEY2控制任务三、四*************************************8
if(HAL_GPIO_ReadPin(key2_GPIO_Port,key2_Pin)==0){
			HAL_Delay(100);
			
			if(HAL_GPIO_ReadPin(key2_GPIO_Port,key2_Pin)==0){
	   	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	     HAL_Delay(500);
				keystatues2+=1;	
	HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,RESET);			
     OLED_CLS();	
      shape = 0;
      model = 0;				
			}					
		}
		
		if(keystatues2==1){
			M=1;     //云台转动
    HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_SET);//开启激光标记中心
			
		if(shape ==1){
			OLED_ShowStr(0, 0,"Shape:circle  ", 2);//显示字符串
			HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,SET);
		}
		else if(shape ==2){
			OLED_ShowStr(0, 0,"Shape:rectangle ", 2);//显示字符串
			HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,SET);
		}
		else if(shape ==3){
			OLED_ShowStr(0, 0,"Shape:triangle ", 2);//显示字符串
			HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,SET);
		}	
		
	  OLED_ShowStr(0, 3, "Size:33.7cm", 2);//显示字符串
	  OLED_ShowStr(0, 6, "Distance:", 2);//显示字符串		
		
    OLED_ShowNum(70,7,value,5,20);
	
	}
	else if(keystatues2==2)
	{  //任务四：识别球的种类
            M=1;   //云台不转动
			  HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_SET);//开启激光标记中心
		
		   	OLED_ShowStr(0, 0,"Model:basketball  ", 2);//篮球
					HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,SET);
				if(model ==1){
				
				}
				else if(model ==2){
					OLED_ShowStr(0, 0,"Model:football ", 2);//足球
					HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,SET);
				}
				else if(model ==3){
					OLED_ShowStr(0, 0,"Model:volleyball ", 2);//排球
					HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,SET);
				}					
				OLED_ShowStr(0, 6, "Distance:", 2);//显示字符串					
				OLED_ShowNum(70,7,value,5,20);
	
	}	
		else if(	keystatues2==3)
		{	
			HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET);		
			for (degreex = -40; degreex <= 40; degreex += 1)
			{
				MG996R_Set_Angle2(degreex);
				HAL_Delay(10);
			}
			
			for (degreex = 40; degreex >= -40; degreex -= 1)
			{
				MG996R_Set_Angle2(degreex);
				HAL_Delay(10);
			}
			
			for (degreey = -30; degreey <= 30; degreey += 1)
			{
				MG996R_Set_Angle(degreey);
				HAL_Delay(10);
			}
			
			for (degreey = 30; degreey >= -30; degreey -= 1)
			{
				MG996R_Set_Angle(degreey);
				HAL_Delay(10);
			}
		
			OLED_ShowStr(0, 0,"thank you", 2);//篮球	
		}
	
		else if(	keystatues2==4)
		{
			keystatues2=0;
		}		
  }
	OLED_CLS();
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint8_t my_re_buf1[2000];            //openmv
uint8_t my_re_buf2[2000];            //ttl
uint8_t my_re_buf6[2000];            //radar
uint16_t pt_w1=0,pt_w2=0,pt_r1=0,pt_r2=0;
uint16_t pt_w6=0,pt_r6=0;


/**
  * @brief  UART error callback.
  * @param  huart UART handle.
  * @retval None
  */
 void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint8_t i = 0;

    if(__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE) != RESET) 
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(huart,(u8 *)&i,1);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)     //opmv-receive
{
	if(huart==&huart1)
	{	
		HAL_UART_Receive_IT(&huart1,&my_re_buf1[++pt_w1],1);  	
		trans(my_re_buf1[pt_w1]);	
//		measure();	
	}
	
	if(huart==&huart6)               //radar
	{	
		HAL_UART_Receive_IT(&huart6,&my_re_buf6[++pt_w6],1);		
		trans6(my_re_buf6[pt_w6]);	
	}	
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim4)
	{
		while(pt_r1<pt_w1 )
		{
			while(pt_r1<pt_w1){
				
				pt_r1++;

				//HAL_UART_Transmit(&huart2,&my_re_buf1[pt_r1++],1,1000);				//opmv-stm32-ttl
	      
				//printf("%d",my_re_buf1);		
			}				
		}
		
		if(pt_r1>=pt_w1)
		{
			pt_w1=pt_r1=0;
			HAL_UART_AbortReceive_IT(&huart1);
	  	HAL_UART_Receive_IT(&huart1,my_re_buf1,1);
			
			//test 2
		}
		
		
//		while(pt_r2<pt_w2 )
//		{
//	
//			HAL_UART_Transmit(&huart1,&my_re_buf2[pt_r2++],1,1000);					//将PC发给板子的数据转发到wifi模块			
//			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);    //LED
//	
//		}
//		if(pt_r2>=pt_w2)
//		{
//			pt_w2=pt_r2=0;
//			HAL_UART_AbortReceive_IT(&huart2);	
//			HAL_UART_Receive_IT(&huart2,my_re_buf2,1);
//		}

		while(pt_r6<pt_w6 )
		{
			while(pt_r6<pt_w6){
				
				pt_r6++;

			
//				HAL_UART_Transmit(&huart2,&my_re_buf6[pt_r6++],1,1000);				//opmv-stm32-ttl
	      
//		   printf("%d",my_re_buf1);
		   
		
			}				
		}
		
		if(pt_r6>=pt_w6)
		{
			pt_w6=pt_r6=0;
			HAL_UART_AbortReceive_IT(&huart6);
	  	HAL_UART_Receive_IT(&huart6,my_re_buf6,1);
			
			//test 2
		}
			
	}
}

void MG996R_Set_Angle(int16_t angle)
{
	uint16_t CCR_value = 0;
	
	if ((-90 <= angle) && (angle <= 90))
	{
		CCR_value = (10. / 9) * angle + 150;		// ?? to CCR?
	}
	else
	{
		CCR_value = 150;
	}
	
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, CCR_value);
}


void MG996R_Set_Angle2(int16_t angle2)
{
	uint16_t CCR_value = 0;
	
	if ((-90 <= angle2) && (angle2 <= 90))
	{
		CCR_value = (10. / 9) * angle2 + 150;		// ?? to CCR?
	}
	else
	{
		CCR_value = 150;
	}
	
	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, CCR_value);
}

int opmvdata[8],i,m=0,servo_angle=0,servo_angle2=0;
int value1=0,value2=0,value3=0;

int model1=0,model2=0,model3=0;
unsigned char radardata[16];

void measure(void)
{
	  int x,y;
		 
	  x=opmvdata[3];//坐标
	  y=opmvdata[4];
	 	
if(opmvdata[5]==0x01){
 	model1++;
//	shape=1;//圆形
}

else if(opmvdata[5]==0x02){
	
	model2++;
//	shape=2;//正方形
}
else if (opmvdata[5] == 0x03){
	
	model3++;
//	shape=3;   //三角形
}


if(model1>1){

	model=1;//篮球
	model1=0;
	model2=0;
	model3=0;
	
}
else if(model2>1){

	model=2;//足球
	model1=0;
	model2=0;
	model3=0;
	
}

else if(model3>1){

	model=3;//排球
	model1=0;
	model2=0;
	model3=0;
	
}
	

if(opmvdata[2]==0xb1){
 	value1++;
//	shape=1;//圆形
}
else if(opmvdata[2]==0xb2){
	
	value2++;
//	shape=2;//正方形
}
else if (opmvdata[2] == 0xb7){
	
	value3++;
//	shape=3;   //三角形
}

if(value1>50){

	shape=1;//圆形
	value1=0;
	value2=0;
	value3=0;
	
	
}
else if(value2>50){

	shape=2;
	value1=0;
	value2=0;
	value3=0;
	
}

else if(value3>50){

	shape=3;
	value1=0;
	value2=0;
	value3=0;
	
}

	//servo begin
  if(m>5&&M==1){
		
			  if(x<75&&x>0){
				
				if(servo_angle2<90){
									
	     	servo_angle2+=1;
	     	MG996R_Set_Angle2(servo_angle2); 	//SERVO
					
				}
		
      	}else if(x>75&&x<200){
					
					
					if(servo_angle2>-90){
									
	     	servo_angle2-=1 ;
	     	MG996R_Set_Angle2(servo_angle2); 	//SERVO
					 
				}
			}
					
				
				//y servo        #############
				
				
				if(y<67&&y>0){
				
				if(servo_angle<90){
									
	     	servo_angle-=1;
	     	MG996R_Set_Angle(servo_angle); 	//SERVO
					
				}
		
      	}else if(y>67&&y<100){
					
					
					if(servo_angle>-90){
									
	     	servo_angle+=1 ;
	     	MG996R_Set_Angle(servo_angle); 	//SERVO
					 
				} 
			}
	
	m=0;
		
	}
	m++;
	
	//servo end	
}

void measure6(void)
{
	  float distance;
	  static int i;
	  value = (radardata[9]-0x30)*(16*16*16) + (radardata[10]-0x30)*(16*16) + (radardata[11]-0x30)*16 + (radardata[12]-0x30);	
     if(value>4000){
			 value-=1900;
		 }
}




void trans(int16_t data){
	
	static int state = 0;	
	if(state==0&&data==0xb3)
	{
		state=1;
		opmvdata[0]=data;
	}
	
	else if(state==1&&data==0xb4)
	{
		state=2;
		opmvdata[1]=data;
	}
	else if(state==2)
	{
		state=3;
		opmvdata[2]=data;//形状：b1-->圆
	}
	else if(state==3)
	{
		state = 4;
		opmvdata[3]=data;
	}
	else if(state==4)
	{
        state = 5;
        opmvdata[4]=data;
	}
	else if(state==5)
	{
        state = 6;
        opmvdata[5]=data;
	}
	else if(state==6)
	{
        state = 7;
        opmvdata[6]=data;
	}

	else if(state==7)		//检测是否接受到结束标志
	{
        if(data == 0xb6)
        {
            state = 0;
            opmvdata[7]=data;
            measure();
				
        }
        else if(data != 0xb6)
        {
            state = 0;
            for(i=0;i<8;i++)
            {
                opmvdata[i]=0x00;
            }           
        }
	} 
	else
	{
			      state = 0;
            for(i=0;i<8;i++)
            {
                opmvdata[i]=0x00;
            }
	}
}
	

//radar

void trans6(int16_t data){
	
	static int state = 0;
	
	if(state==0&&data==0x7E)
	{
		state=1;
		radardata[0]=data;

	}
	else if(state==1&&data==0x30)
	{
		state=2;
		radardata[1]=data;
	}
	else if(state==2)
	{
		state=3;
		radardata[2]=data;//形状：b1-->圆
	}
	else if(state==3)
	{
		state = 4;
		radardata[3]=data;
	}
	else if(state==4)
	{
        state = 5;
        radardata[4]=data;
	}
	else if(state==5)
	{
        state = 6;
        radardata[5]=data;
	}
	else if(state==6)
	{
        state = 7;
        radardata[6]=data;
	}
	
	
	else if(state==7)
	{
        state = 8;
        radardata[7]=data;
	}
	
	
	else if(state==8)
	{
        state = 9;
        radardata[8]=data;
	}
	
	else if(state==9)
	{
        state = 10;
        radardata[9]=data;
	}
	
	else if(state==10)
	{
        state = 11;
        radardata[10]=data;
	}
	
	else if(state==11)
	{
        state = 12;
        radardata[11]=data;
	}

	else if(state==12)		//检测是否接受到结束标志
	{
       
            state = 0;
            radardata[12]=data;
            measure6();
				
        }
       
	else
	{
			      state = 0;
            for(i=0;i<12;i++)
            {
                radardata[i]=0x00;
            }
	}
}








/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
