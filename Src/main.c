/* USER CODE BEGIN Header */


#include "main.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*MPU6050 register addresses */

#define MPU6050_REG_POWER               0x6B
#define MPU6050_REG_ACCEL_CONFIG        0x1C
#define MPU6050_REG_GYRO_CONFIG         0x1B

/*These are the addresses of mpu6050 from which you will fetch accelerometer x,y,z high and low values */
#define MPU6050_REG_ACC_X_HIGH          0x3B
#define MPU6050_REG_ACC_X_LOW           0x3C
#define MPU6050_REG_ACC_Y_HIGH          0x3D
#define MPU6050_REG_ACC_Y_LOW           0x3E
#define MPU6050_REG_ACC_Z_HIGH          0x3F
#define MPU6050_REG_ACC_Z_LOW           0x40

/*These are the addresses of mpu6050 from which you will fetch gyro x,y,z high and low values */
#define MPU6050_REG_GYRO_X_HIGH          0x43
#define MPU6050_REG_GYRO_X_LOW           0x44
#define MPU6050_REG_GYRO_Y_HIGH          0x45
#define MPU6050_REG_GYRO_Y_LOW           0x46
#define MPU6050_REG_GYRO_Z_HIGH          0x47
#define MPU6050_REG_GYRO_Z_LOW           0x48

/*
 * Different full scale ranges for acc and gyro
 * refer table 6.2 and 6.3 in the document MPU-6000 and MPU-6050 Product Specification Revision 3.4
 *
 */
#define ACC_FS_SENSITIVITY_0					      16384
#define ACC_FS_SENSITIVITY_1		            8192
#define ACC_FS_SENSITIVITY_2		            4096
#define ACC_FS_SENSITIVITY_3		            2048

#define GYR_FS_SENSITIVITY_0					 131
#define GYR_FS_SENSITIVITY_1					 65.5
#define GYR_FS_SENSITIVITY_2					 32.8
#define GYR_FS_SENSITIVITY_3				 	 16.4

/* This is the I2C slave address of mpu6050 sensor */
#define MPU6050_SLAVE_ADDR 0x68

int mpu6050_write(uint8_t addr, uint8_t data);
int mpu6050_read(uint8_t base_addr, uint8_t *pBuffer,uint32_t len);
void mpu6050_init();
void mpu6050_read_acc(short int *pBuffer);
void mpu6050_read_gyro(short *pBuffer);
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
I2C_HandleTypeDef hi2c1;


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  SystemClock_Config();//32 MHZ

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();


   mpu6050_init();
	 short acc_value[3],gyro_value[3];
   double accx,accy,accz,gyrox,gyroy,gyroz;
   uint8_t count=0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  if(count < 100)
  {
    /* USER CODE END WHILE */
        mpu6050_read_acc(acc_value);
        mpu6050_read_gyro(gyro_value);

        /*Convert acc raw values in to 'g' values*/
        accx = (double) acc_value[0]/ACC_FS_SENSITIVITY_0;
        accy = (double) acc_value[1]/ACC_FS_SENSITIVITY_0;
        accz = (double) acc_value[2]/ACC_FS_SENSITIVITY_0;

        /* Convert gyro raw values in to  "?/s" (deg/seconds) */
        
        gyrox = (double) gyro_value[0]/GYR_FS_SENSITIVITY_0;
        gyroy = (double) gyro_value[1]/GYR_FS_SENSITIVITY_0;
        gyroz = (double) gyro_value[2]/GYR_FS_SENSITIVITY_0;
		    /*
		     
		      Application Specific code Lines
		       ........................
		       ........................
		
		
		         */
		    count++;
		    HAL_Delay(10); //Delay of 10 ms
 
    /* USER CODE BEGIN 3 */
  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

}
int mpu6050_write(uint8_t addr, uint8_t data)
{
  int ret;
    
  
  ret= HAL_I2C_Master_Transmit(&hi2c1,addr,&data,1,HAL_MAX_DELAY);

  
  if (ret <= 0)
  {
      perror("write failed\n");
      return -1;
  }
  return 0;
}

/*read "len" many bytes from "addr" of the sensor in to the adresss indicated by "pBuffer" */
int mpu6050_read(uint8_t base_addr, uint8_t *pBuffer,uint32_t len)
{
  int ret;
  
  ret= HAL_I2C_Master_Receive(&hi2c1,base_addr,pBuffer,len,HAL_MAX_DELAY);
  if (ret <= 0)
  {
      perror("write address failed\n");
      return -1;
  }

 
  return 0;
}


/* by default mpu6050 will in sleep mode, so disable its sleep mode and also configure 
 * the full scale ranges for gyro and acc
 */
void mpu6050_init()
{
    // 1. disable sleep mode
    mpu6050_write(MPU6050_REG_POWER, 0x00);
    HAL_Delay(500);

    // Adjust full scale values for gyro and acc
    mpu6050_write(MPU6050_REG_ACCEL_CONFIG, 0x00);
    HAL_Delay(500);
    mpu6050_write(MPU6050_REG_GYRO_CONFIG, 0x00);
    HAL_Delay(500);
}

/* read accelerometer values of x,y,z in to the buffer "pBuffer" */
void mpu6050_read_acc(short int *pBuffer)
{
    //each axis value is of 2byte, so we need a buffer of 6bytes. 
    uint8_t acc_buffer[6];
    
    //start reading from the base address of accelerometer values i.e MPU6050_REG_ACC_X_HIGH
    mpu6050_read(MPU6050_REG_ACC_X_HIGH,acc_buffer,6);
    
    /* pBuffer[0]= acc x axis value , pBuffer[1]= acc y axis value , pBuffer[2]= acc z axis value  */
    pBuffer[0] = (int) ( (acc_buffer[0] << 8) |  acc_buffer[1] );
    pBuffer[1] = (int) ( (acc_buffer[2] << 8) |  acc_buffer[3] );
    pBuffer[2] = (int) ( (acc_buffer[4] << 8) |  acc_buffer[5] );

}

/* read gyro values of x,y,z in to the buffer "pBuffer" */
void mpu6050_read_gyro(short *pBuffer)
{
    uint8_t gyro_buffer[6];
    
    //start reading from the base address of gyro values i.e MPU6050_REG_GYRO_X_HIGH
    mpu6050_read(MPU6050_REG_GYRO_X_HIGH,gyro_buffer,6);

    pBuffer[0] =  ( (gyro_buffer[0] << 8) +  gyro_buffer[1] );
    pBuffer[1] =  ( (gyro_buffer[2] << 8) +  gyro_buffer[3] );
    pBuffer[2] =  ( (gyro_buffer[4] << 8) +  gyro_buffer[5] );

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
