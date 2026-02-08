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

// UART RX - komut altyapısı
uint8_t rcmd;
volatile uint8_t cmd_ready = 0;
char cmd_line[32];
volatile uint8_t cmd_len = 0;

// Durum flagleri
volatile uint8_t st_rst   = 0;
volatile uint8_t st_basla = 0;
volatile uint8_t st_devam = 0;
volatile uint8_t state    = 0;

// Ölçüm değişkenleri
uint8_t prs = 0;

// Termal Değişkenler
static uint16_t eeMLX90640[832];
static uint16_t frameData[834];
static float mlx90640To[768];
paramsMLX90640 mlx90640;

// Analiz Saklama
static float foot1_temp[768], foot2_temp[768];
static uint32_t foot1_press[10], foot2_press[10];

static const uint8_t MAP[10] = {9,2,5,8,1,4,7,3,0,6};
uint16_t fsrValues[10];
char msg[512];
char msgg[512];
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

static void ProcessCommandLine(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Komut işleyici: main loop'tan çağrılır */
static void ProcessCommandLine(void)
{
    if (!cmd_ready) return;
    cmd_ready = 0;

    // Sondaki boşluk/tab temizliği (BT app bazen boşluk ekliyor)
    for (int i = (int)strlen(cmd_line) - 1; i >= 0; i--) {
        if (cmd_line[i] == ' ' || cmd_line[i] == '\t') cmd_line[i] = '\0';
        else break;
    }

    if (strcmp(cmd_line, "rst") == 0) {
        st_rst = 1;
        HAL_UART_Transmit(&huart3, (uint8_t*)"Resetleme işlemi başarılı, 'basla' komutu bekleniyor. \r\n", 9, 100);
    }
    else if (strcmp(cmd_line, "basla") == 0) {
        if (state == 0) {
            st_basla = 1;
            HAL_UART_Transmit(&huart3, (uint8_t*)"OK: basla\r\n", 11, 100);
        } else {
            HAL_UART_Transmit(&huart3, (uint8_t*)"ERR: state=1, devam bekleniyor\r\n", 33, 100);
        }
    }
    else if (strcmp(cmd_line, "devam") == 0) {
        if (state == 1) {
            st_devam = 1;
            HAL_UART_Transmit(&huart3, (uint8_t*)"OK: devam\r\n", 11, 100);
        } else {
            HAL_UART_Transmit(&huart3, (uint8_t*)"ERR: state=0, basla bekleniyor\r\n", 33, 100);
        }
    }
    else {
        HAL_UART_Transmit(&huart3, (uint8_t*)"ERR: gecersiz komut\r\n", 22, 100);
    }
}
/* USER CODE END 0 */

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

  char *hello = "Jarvis: Sistem Kalibre Edildi. Komutunuz bekleniyor...\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)hello, strlen(hello), 100);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)fsrValues, 10);

  if (MLX90640_TestConnection()) {
      MLX90640_DumpEE(MLX_ADDR, eeMLX90640);
      MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
      MLX90640_SetRefreshRate(MLX_ADDR, 0x02);
  } else {
      char *e = "MLX90640 baglanti yok!\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)e, strlen(e), 100);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */

    // Komutları sürekli işle (ISR sadece topluyor)
    ProcessCommandLine();

    if (st_rst) {
        doReset();
        continue;
    }

    // Buton kontrolün (senin mantık)
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
        HAL_Delay(250);
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
            if (state == 0) st_basla = 1;
            else st_devam = 1;

            while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) { /* bekle */ }
        }
    }

    if (state == 0) {
        if (st_basla) {
            st_basla = 0;

            char *m = "ilk ölçüm başlatılıyor\r\n";
            HAL_UART_Transmit(&huart3, (uint8_t*)m, strlen(m), 100);

            uint32_t startTemp = HAL_GetTick();
            while (HAL_GetTick() - startTemp < 6000) {
                ProcessCommandLine();
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
                ProcessCommandLine();
                if (st_rst) { doReset(); goto LOOP_CONTINUE; }

                for (int i = 0; i < 10; i++) Psum[i] += fsrValues[MAP[i]];
                Pcount++;
                HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
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
                ProcessCommandLine();
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
                ProcessCommandLine();
                if (st_rst) { doReset(); goto LOOP_CONTINUE; }

                for (int i = 0; i < 10; i++) Psum[i] += fsrValues[MAP[i]];
                Pcount++;
                HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
                HAL_Delay(100);
            }
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
            for (int i = 0; i < 10; i++) foot2_press[i] = Psum[i] / Pcount;

            float maxT1, maxT2; int idx1, idx2;
            SmartAnalysis(foot1_temp, &maxT1, &idx1);
            SmartAnalysis(foot2_temp, &maxT2, &idx2);
            CalculateRisk(maxT1, maxT2, foot1_press, foot2_press);

            state = 0;
            char *mf = "Ölçüm başarıyla tamamlandı. Yeniden başlatmak için 'basla' komutu bekleniyor.\r\n";
            HAL_UART_Transmit(&huart3, (uint8_t*)mf, strlen(mf), 100);
        }
    }

LOOP_CONTINUE:
    ;
    /* USER CODE END WHILE */
  }
  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* --- DESTEK FONKSIYONLARI --- */
void doReset(void)
{
    st_rst = 0;
    st_basla = 0;
    st_devam = 0;

    state = 0;
    measurementStep = 0;

    memset(foot1_temp,  0, sizeof(foot1_temp));
    memset(foot2_temp,  0, sizeof(foot2_temp));
    memset(foot1_press, 0, sizeof(foot1_press));
    memset(foot2_press, 0, sizeof(foot2_press));

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

    char *m = "Sıfırlama işlemi başarılı...\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)m, (uint16_t)strlen(m), 100);
}

/* ISR: sadece toplar */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        char c = (char)rcmd;

        if (c == '\r' || c == '\n')
        {
            if (cmd_len > 0)
            {
                cmd_line[cmd_len] = '\0';
                cmd_ready = 1;
                cmd_len = 0;
            }
        }
        else
        {
            if (cmd_len < sizeof(cmd_line) - 1) cmd_line[cmd_len++] = c;
            else cmd_len = 0;
        }

        HAL_UART_Receive_IT(&huart3, &rcmd, 1);
    }
}

/* UART hata olursa RX’i toparla */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        HAL_UART_Receive_IT(&huart3, &rcmd, 1);
    }
}

uint32_t ADC_to_Grams(uint32_t adcVal)
{
    if (adcVal < 50) return 0;

    float voltage = (float)adcVal * (3.3f / 4095.0f);
    if (voltage >= 3.25f) voltage = 3.25f;

    float fsrRes = ((3.3f * 10000.0f) / voltage) - 10000.0f;
    float conductance = 1.0f / fsrRes;

    uint32_t grams = (uint32_t)(conductance * 950000.0f);
    return grams;
}

void CalculateRisk(float maxT1, float maxT2, uint32_t *p1, uint32_t *p2)
{
    float riskScore = 0;
    float tempDiff = fabs(maxT1 - maxT2);

    if (tempDiff >= 2.2f) riskScore += 60;
    else if (tempDiff >= 1.5f) riskScore += 30;
    else riskScore += (tempDiff * 10);

    float totalAsymmetry = 0; int active = 0;

    for (int i = 0; i < 10; i++)
    {
        uint32_t g1 = ADC_to_Grams(p1[i]);
        uint32_t g2 = ADC_to_Grams(p2[i]);
        sprintf(msgg,"ilk ayak [%d] sensör [%u] | ikinci ayak [%d] senör [%u] \r\n",i,(unsigned)p1[i],i,(unsigned)p2[i]);
        HAL_UART_Transmit(&huart3, (uint8_t*)msgg, strlen(msgg),100);

        if (g1 > 100 || g2 > 100)
        {
            float diffP = fabs((float)g1 - (float)g2);
            float maxP  = (g1 > g2) ? (float)g1 : (float)g2;
            float ratio = (maxP > 0.0f) ? (diffP / maxP) : 0.0f;

            totalAsymmetry += ratio;
            active++;

            if (ratio > 0.25f)
            {
                sprintf(msg, "! ASIMETRI: Sensor [%d] (%%%.1f fark)\r\n", i, ratio * 100.0f);
                HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
            }

        }

    }

    if (active > 0)
    {
        float avgAsymmetry = (totalAsymmetry / active) * 100.0f;
        if (avgAsymmetry > 25.0f) riskScore += 40;
        else riskScore += (avgAsymmetry * 1.6f);
    }

    sprintf(msg, "\r\nSicaklik Farki: %.2f C\r\nGENEL RISK: %%%.1f\r\n", tempDiff, riskScore);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
}

void SmartAnalysis(float *data, float *maxOut, int *idxOut)
{
    float sum = 0.0f, count = 0.0f, sqDiffSum = 0.0f;
    *maxOut = 0.0f; *idxOut = -1;

    for (int i = 0; i < 768; i++) {
        if (data[i] >= 33.0f && data[i] <= 40.0f) { sum += data[i]; count++; }
    }
    if (count == 0.0f) return;

    float mean = sum / count;

    for (int i = 0; i < 768; i++) {
        if (data[i] >= 33.0f && data[i] <= 40.0f) {
            float d = data[i] - mean;
            sqDiffSum += d * d;
        }
    }

    float stdDev = sqrtf(sqDiffSum / count);

    float currentMax = -100.0f;
    for (int i = 0; i < 768; i++) {
        if (data[i] >= 33.0f && data[i] <= 40.0f && data[i] < (mean + 2.0f * stdDev)) {
            if (data[i] > currentMax) { currentMax = data[i]; *idxOut = i; }
        }
    }
    *maxOut = currentMax;
}

static uint8_t MLX90640_TestConnection(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, MLX_ADDR << 1, 3, 200) == HAL_OK) return 1;
    return 0;
}

/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
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

void Error_Handler(void)
{
  while (1) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
    HAL_Delay(100);
  }
}
