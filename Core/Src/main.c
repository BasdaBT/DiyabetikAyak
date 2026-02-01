/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Alfa Projesi - Gram Kalibrasyonlu Diyabetik Risk Analizi
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "MLX90640_API.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define MLX_ADDR 0x33

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart3;
extern ADC_HandleTypeDef hadc1;
//bluetooth kontrolleri
uint8_t rcmd;
char cmd[32];
uint8_t idxcmd;
volatile uint8_t st_rst = 0;
volatile uint8_t st_basla = 0;
volatile uint8_t st_devam = 0;
volatile uint8_t state = 0;
//Ölçüm değişkenleri
uint8_t prs = 0;
// Termal Değişkenler
static uint16_t eeMLX90640[832];
static uint16_t frameData[834];
static float mlx90640To[768];
paramsMLX90640 mlx90640;

// Analiz Saklama
static float foot1_temp[768], foot2_temp[768];
static uint32_t foot1_press[10], foot2_press[10];

uint16_t fsrValues[10];
char msg[512];
int measurementStep = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t MLX90640_TestConnection(void);
void doReset(void);
void SmartAnalysis(float *data, float *maxOut, int *idxOut);
void CalculateRisk(float maxT1, float maxT2, uint32_t *p1, uint32_t *p2);
uint32_t ADC_to_Grams(uint32_t adcVal);
/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3, &rcmd, 1);
  idxcmd = 0;
  HAL_UART_Transmit(&huart3, (uint8_t*)"Jarvis: Sistem Kalibre Edildi. Komutunuz bekleniyor...\r\n", 48, 100);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)fsrValues, 10);

  if (MLX90640_TestConnection()) {
      MLX90640_DumpEE(MLX_ADDR, eeMLX90640);
      MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
      MLX90640_SetRefreshRate(MLX_ADDR, 0x02);
  }
  /* USER CODE END 2 */

  while (1)
  {
	  if(st_rst){
		  doReset();
		  continue;
	  }
   if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == GPIO_PIN_RESET){
	   HAL_Delay(250);
	   if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){
		   if(state == 0){
			   st_basla = 1;
		   }
		   else
		   {
			   st_devam = 1;
		   }
		   while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);
	       }
      }

   if (state == 0) {
       if (st_basla) {
           st_basla = 0;

           char *m = "ilk ölçüm başlatılıyor\r\n";
           HAL_UART_Transmit(&huart3, (uint8_t*)m, strlen(m), 100);

           uint32_t startTemp = HAL_GetTick();
           while (HAL_GetTick() - startTemp < 6000) {
               if (st_rst) { doReset(); goto LOOP_CONTINUE; }

               if (MLX90640_GetFrameData(MLX_ADDR, frameData) >= 0) {
                   float ta = MLX90640_GetTa(frameData, &mlx90640);
                   MLX90640_CalculateTo(frameData, &mlx90640, 0.95, ta - 8.0, mlx90640To);
                   memcpy(foot1_temp, mlx90640To, sizeof(mlx90640To));
               }

               HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
               HAL_Delay(200);
           }

           char *m1 = "Basınc kaydı yapılıyor...\r\n";
           HAL_UART_Transmit(&huart3, (uint8_t*)m1, strlen(m1), 100);

           uint32_t Psum[10] = {0}, Pcount = 0;
           uint32_t startPres = HAL_GetTick();
           while (HAL_GetTick() - startPres < 6000) {
               if (st_rst) { doReset(); goto LOOP_CONTINUE; }

               for (int i = 0; i < 10; i++) Psum[i] += fsrValues[i];
               Pcount++;
               HAL_Delay(100);
           }

           for (int i = 0; i < 10; i++) foot1_press[i] = Psum[i] / Pcount;

           state = 1;
           char *m2 = "Birinci bitti, 'devam' bekleniyor...\r\n";
           HAL_UART_Transmit(&huart3, (uint8_t*)m2, strlen(m2), 100);
           HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
       }
   }
   else if (state == 1) {
       if (st_devam) {
           st_devam = 0;

           char *m3 = "İkinci ölçüm başlatılıyor...\r\n";
           HAL_UART_Transmit(&huart3, (uint8_t*)m3, strlen(m3), 100);

           uint32_t startTemp = HAL_GetTick();
           while (HAL_GetTick() - startTemp < 6000) {
               if (st_rst) { doReset(); goto LOOP_CONTINUE; }

               if (MLX90640_GetFrameData(MLX_ADDR, frameData) >= 0) {
                   float ta = MLX90640_GetTa(frameData, &mlx90640);
                   MLX90640_CalculateTo(frameData, &mlx90640, 0.95, ta - 8.0, mlx90640To);
                   memcpy(foot2_temp, mlx90640To, sizeof(mlx90640To));
               }

               HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
               HAL_Delay(200);
           }

           char *m4 = "Basınç kaydı yapılıyor\r\n";
           HAL_UART_Transmit(&huart3, (uint8_t*)m4, strlen(m4), 100);

           uint32_t Psum[10] = {0}, Pcount = 0;
           uint32_t startPres = HAL_GetTick();
           while (HAL_GetTick() - startPres < 6000) {
               if (st_rst) { doReset(); goto LOOP_CONTINUE; }

               for (int i = 0; i < 10; i++) Psum[i] += fsrValues[i];
               Pcount++;
               HAL_Delay(100);
           }

           for (int i = 0; i < 10; i++) foot2_press[i] = Psum[i] / Pcount;

           float maxT1, maxT2; int idx1, idx2;
           SmartAnalysis(foot1_temp, &maxT1, &idx1);
           SmartAnalysis(foot2_temp, &maxT2, &idx2);
           CalculateRisk(maxT1, maxT2, foot1_press, foot2_press);

           state = 0;
           char *mf = "Ölçüm başarıyla tamamlandı.\r\n";
           HAL_UART_Transmit(&huart3, (uint8_t*)mf, strlen(mf), 100);
       }
   }

   LOOP_CONTINUE:
   ;}}



/* --- DESTEK FONKSIYONLARI --- */
void doReset(void){
	st_rst = 0;
	st_basla = 0;
	st_devam=0;

	state = 0;
	measurementStep = 0;

	memset(foot1_temp,0,sizeof(foot1_temp) );
	memset(foot2_temp,0,sizeof(foot2_temp));
	memset(foot1_press,0,sizeof(foot1_press));
	memset(foot2_press,0,sizeof(foot2_press));

	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

	char*msg = "Sıfırlama işlemi başarılı...\r\n";
	HAL_UART_Transmit(&huart3, (uint8_t*)msg, (uint16_t)strlen(msg),100);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart->Instance == USART3){
		if(rcmd  == '\r' || rcmd == '\n'){
			cmd[idxcmd] = '\0';
			if(idxcmd > 0){
				if(strcmp(cmd, "basla") == 0){
					st_basla = 1;
				}else if(strcmp(cmd, "devam") == 0){
					st_devam = 1;
				}else if(strcmp(cmd, "rst") == 0){
					st_rst = 1;
				}
				else
				{
					char*msg = "Gecersiz komut\r\n";
				    HAL_UART_Transmit(&huart3, (uint8_t*)msg, (uint16_t)strlen(msg), 100);

				}
			}
			idxcmd = 0;

		}
		else{
			if(idxcmd < sizeof(cmd) - 1){
				cmd[idxcmd++] = (char)rcmd;
			}else{
				idxcmd = 0;
			}

		   }
		HAL_UART_Receive_IT(&huart3, &rcmd,1);
	    }

}

uint32_t ADC_to_Grams(uint32_t adcVal) {
    if (adcVal < 50) return 0; // Gürültü filtresi

    float voltage = (float)adcVal * (3.3f / 4095.0f);
    // Vout 3.3V'a çok yakınsa (doyum), hesaplama hatasını önlemek için sınırla
    if (voltage >= 3.25f) voltage = 3.25f;

    // R_fsr = ((Vcc * R_pd) / Vout) - R_pd  (R_pd = 10k kabul edildi)
    float fsrRes = ((3.3f * 10000.0f) / voltage) - 10000.0f;
    float conductance = 1.0f / fsrRes;

    // Senin verine göre: 2.2kg (2200gr) -> 3749 ADC değeri için çarpan
    // Bu değer yaklaşık bir tahmindir, hassas terazi ile kalibre edilebilir.
    uint32_t grams = (uint32_t)(conductance * 950000.0f);
    return grams;
}

void CalculateRisk(float maxT1, float maxT2, uint32_t *p1, uint32_t *p2) {
    float riskScore = 0;
    float tempDiff = fabs(maxT1 - maxT2);
    if (tempDiff >= 2.2) riskScore += 60;
    else if (tempDiff >= 1.5) riskScore += 30;
    else riskScore += (tempDiff * 10);

    float totalAsymmetry = 0; int active = 0;
    for(int i=0; i<10; i++) {
        uint32_t g1 = ADC_to_Grams(p1[i]);
        uint32_t g2 = ADC_to_Grams(p2[i]);
        if (g1 > 100 || g2 > 100) {
            float diffP = fabs((float)g1 - (float)g2);
            float maxP = (g1 > g2) ? g1 : g2;
            float ratio = (diffP / maxP);
            totalAsymmetry += ratio; active++;
            if (ratio > 0.25) {
                sprintf(msg, "! ASIMETRI: Sensor [%d] (%%%.1f fark)\r\n", i, ratio * 100);
                HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
            }
        }
    }
    if (active > 0) {
        float avgAsymmetry = (totalAsymmetry / active) * 100;
        if (avgAsymmetry > 25) riskScore += 40;
        else riskScore += (avgAsymmetry * 1.6);
    }
    sprintf(msg, "\r\nSicaklik Farki: %.2f C\r\nGENEL RISK: %%%.1f\r\n", tempDiff, riskScore);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
}

void SmartAnalysis(float *data, float *maxOut, int *idxOut) {
    float sum = 0.0, count = 0.0, sqDiffSum = 0.0;
    *maxOut = 0; *idxOut = -1;
    for (int i = 0; i < 768; i++) {
        if (data[i] >= 33.0 && data[i] <= 40.0) { sum += data[i]; count++; }
    }
    if (count == 0) return;
    float mean = sum / count;
    for (int i = 0; i < 768; i++) {
        if (data[i] >= 33.0 && data[i] <= 40.0) { sqDiffSum += pow(data[i] - mean, 2); }
    }
    float stdDev = sqrt(sqDiffSum / count);
    float currentMax = -100.0;
    for (int i = 0; i < 768; i++) {
        if (data[i] >= 33.0 && data[i] <= 40.0 && data[i] < (mean + 2 * stdDev)) {
            if (data[i] > currentMax) { currentMax = data[i]; *idxOut = i; }
        }
    }
    *maxOut = currentMax;
}

static uint8_t MLX90640_TestConnection(void) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, MLX_ADDR << 1, 3, 200) == HAL_OK) return 1;
    return 0;
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void Error_Handler(void) {
  while (1) { HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); HAL_Delay(100); }
}
