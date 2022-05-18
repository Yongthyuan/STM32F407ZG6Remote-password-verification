#include <stdio.h>
#include "main.h"
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);

/*int fputc(int ch, FILE* f) {
	uint8_t temp[1] = {ch};
	if (HAL_UART_Transmit(&huart1, temp, 1, 0xffff) != HAL_OK)
		Error_Handler();
	return 0;
}*/
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}

//ң��������״̬
//[7]:�յ����������־
//[6]:�õ���һ��������������Ϣ
//[5]:����
//[4]:����������Ƿ��Ѿ�������						   
//[3:0]:�����ʱ��
u8 	RmtSta=0;	  	  
u16 Dval;		//�½���ʱ��������ֵ
u32 RmtRec=0;	//������յ�������	    
u8  RmtCnt=0;	//�������µĴ���

//��ʱ�����£�������жϻص�����
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance==TIM1){
 		if(RmtSta&0x80)//�ϴ������ݱ����յ���
		{
			RmtSta&=~0X10;						//ȡ���������Ѿ���������
			if((RmtSta&0X0F)==0X00){
			//printf("����Ѿ����һ�ΰ�\n");
			RmtSta|=1<<6;}//����Ѿ����һ�ΰ����ļ�ֵ��Ϣ�ɼ�
			if((RmtSta&0X0F)<14)RmtSta++;
			else
			{
				RmtSta&=~(1<<7);//���������ʶ
				RmtSta&=0XF0;	//��ռ�����	
			}						 	   	
		}	
 
 }
}

//��ʱ�����벶���жϻص�����
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//�����жϷ���ʱִ��
{
 if(htim->Instance==TIM1)
{
	//�������������
 	if(BIT_ADDR((GPIOA_BASE+16), 8))//�����ز���
	{
		
		TIM_RESET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1);   //һ��Ҫ�����ԭ�������ã���
        TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);//CC1P=1 ����Ϊ�½��ز���
        __HAL_TIM_SET_COUNTER(&htim1,0);  //��ն�ʱ��ֵ   	  
		  	RmtSta|=0X10;					//����������Ѿ�������
	}else //�½��ز���
	{
        Dval=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);//��ȡCCR1Ҳ������CC1IF��־λ
        TIM_RESET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1);   //һ��Ҫ�����ԭ�������ã���
        TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING);//����TIM1ͨ��1�����ز���
		if(RmtSta&0X10)					//���һ�θߵ�ƽ���� 
		{
			if(RmtSta&0X80)//���յ���������
			{
				if(Dval>300&&Dval<800)			//560Ϊ��׼ֵ,560us
				{
					RmtRec<<=1;	//����һλ.
					RmtRec|=0;	//���յ�0	   
				}else if(Dval>1400&&Dval<1800)	//1680Ϊ��׼ֵ,1680us
				{
					RmtRec<<=1;	//����һλ.
					RmtRec|=1;	//���յ�1
				}else if(Dval>2200&&Dval<2600)	//�õ�������ֵ���ӵ���Ϣ 2500Ϊ��׼ֵ2.5ms
				{
					RmtCnt++; 		//������������1��
					RmtSta&=0XF0;	//��ռ�ʱ��		
				}
			}else if(Dval>4200&&Dval<4700)		//4500Ϊ��׼ֵ4.5ms
				{
					RmtSta|=1<<7;	//��ǳɹ����յ���������
					RmtCnt=0;		//�����������������
				}						 
			}
		RmtSta&=~(1<<4);  //�������ز���λ
		}				 		     	    
	}
}

//����������
//����ֵ:
//0,û���κΰ�������
//����,���µİ�����ֵ.
u8 Remote_Scan(void)
{        
	u8 sta=0;       
  u8 t1,t2;  
	if(RmtSta&(1<<6))//�õ�һ��������������Ϣ��
	{ 
	    t1=RmtRec>>24;			//�õ���ַ��
	    t2=(RmtRec>>16)&0xff;	//�õ���ַ���� 
 	    if((t1==(u8)~t2))//����ң��ʶ����(ID)����ַ 
	    { 
	        t1=RmtRec>>8;
	        t2=RmtRec; 	
	        if(t1==(u8)~t2)sta=t1;//��ֵ��ȷ	 
		}   
		if((sta==0)||((RmtSta&0X80)==0))//�������ݴ���/ң���Ѿ�û�а�����
		{
		 	RmtSta&=~(1<<6);//������յ���Ч������ʶ
			RmtCnt=0;		//�����������������
		}
	}  
    return sta;
}

int main(void){
	
	u8 key;
	u8 t=0;	
	u8 *str;
	u8 sum=0;//
	u8 keys[5]="1234";
	u8 nkeys[5]="0000";
	int check=0,time0=0,time1=0,key_time=4;
  
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
	__HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
	
  while (1){
		key=Remote_Scan();	
		if(key)
		{	 
			//sum+=20;
			if(!sum){
				switch(key)
			{
				case 0:str="ERROR";break;			   
				case 162:str="POWER";break;	    
				case 98:str="UP";break;	    
				case 2:str="PLAY";break;		 
				case 226:str="ALIENTEK";break;		  
				case 194:str="RIGHT";break;	   
				case 34:str="LEFT";break;		  
				case 224:str="VOL-";break;		  
				case 168:str="DOWN";break;		   
				case 144:str="VOL+";break;		    
				case 104:str="1";nkeys[time0++]=(uint8_t)'1';break;		  
				case 152:str="2";nkeys[time0++]=(uint8_t)'2';break;	   
				case 176:str="3";nkeys[time0++]=(uint8_t)'3';break;	    
				case 48:str="4";nkeys[time0++]=(uint8_t)'4';break;		    
				case 24:str="5";nkeys[time0++]=(uint8_t)'5';break;		    
				case 122:str="6";nkeys[time0++]=(uint8_t)'6';break;		  
				case 16:str="7";nkeys[time0++]=(uint8_t)'7';break;			   					
				case 56:str="8";nkeys[time0++]=(uint8_t)'8';break;	 
				case 90:str="9";nkeys[time0++]=(uint8_t)'9';break;
				case 66:str="0";nkeys[time0++]=(uint8_t)'0';break;
				case 82:str="DELETE";break;		 
			}
			
			printf("%s ",str);
			nkeys[4]=0;
			keys[4]=0;
			printf("%s",nkeys);
			//printf(" ");
			printf("%s",keys);
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,0);
			
			
			if(time0==4){
				time0=0;
				for(int k=0;k<4;k++){
					if(keys[k]==nkeys[k]){
						check++;
					}
				}
				
				if(check==4){
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,1);
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,0);
					printf("\n������ȷ!\nPOWER���˳�ϵͳ��PLAY���޸�����");
					while(1){
					key=Remote_Scan();
						
					if(key==162){str="POWER";break;}
					if(key==2){str="PLAY";break;}
					}
					//�˳�ϵͳ�������ֻ�ǽ���������̵�Ϩ��
			if(str=="POWER"){
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,0);
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,1);
				printf("�ɹ��˳�");
				break;
			}else if(str=="PLAY"){
				printf("��ʼ�޸�����");
				while(1){
					key=Remote_Scan();
						switch(key)
						{
									    
								case 104:str="1";keys[time1++]=(uint8_t)'1';key_time--;break;		  
								case 152:str="2";keys[time1++]=(uint8_t)'2';key_time--;break;	   
								case 176:str="3";keys[time1++]=(uint8_t)'3';key_time--;break;	    
								case 48:str="4";keys[time1++]=(uint8_t)'4';key_time--;break;		    
								case 24:str="5";keys[time1++]=(uint8_t)'5';key_time--;break;		    
								case 122:str="6";keys[time1++]=(uint8_t)'6';key_time--;break;		  
								case 16:str="7";keys[time1++]=(uint8_t)'7';key_time--;break;			   					
								case 56:str="8";keys[time1++]=(uint8_t)'8';key_time--;break;	 
								case 90:str="9";keys[time1++]=(uint8_t)'9';key_time--;break;
								case 66:str="0";keys[time1++]=(uint8_t)'0';key_time--;break;
							default :str="ERROR";break;
								//case 82:str="DELETE";break;		 
						}
						HAL_Delay(1000);
						if(str=="ERROR"){
							continue;
						}
						printf("%s ",str);
						
						if(key_time==0){
							
							key_time=4;
							printf("%s",keys);
							break;}
				}
				time1=0;
				time0=0;
			}
				}
				check=0;
			}
				//HAL_Delay(60);
			}
			
		}
		HAL_Delay(1000);
		
	}
}

//��ʱnus
//nusΪҪ��ʱ��us��.	
//nus:0~190887435(���ֵ��2^32/fac_us@fac_us=22.5)	 


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
	
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pins : PF9 PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
		printf("error_handler!");
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
