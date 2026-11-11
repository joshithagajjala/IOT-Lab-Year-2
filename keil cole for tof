/*
 * main.c
 * STM32F410RB + VL53L1X -> JSON over USART1 -> ESP8266 AT firmware -> MQTT
 *
 * USART1: ESP8266/Wemos D1 Mini (115200 baud)
 * USART2: PuTTY debug (115200 baud)
 * I2C1:   VL53L1X
 *
 * IMPORTANT:
 * 1. The Wemos must have ESP8266 AT firmware installed.
 * 2. Set WIFI_SSID and WIFI_PASSWORD below.
 * 3. This uses the public EMQX broker shown in the MQTTX screenshot.
 * 4. Keep the CubeMX-generated GPIO/UART MSP configuration in
 *    stm32f4xx_hal_msp.c. USART1 TX/RX pins must be configured there.
 */

#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include "VL53L1X_api.h"
#include "vl53l1_platform.h"

/* Peripheral handles */
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart1;   /* Wemos / ESP8266 */
UART_HandleTypeDef huart2;   /* PuTTY */

/* CubeMX initialization functions */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* ---------------- USER SETTINGS ---------------- */

/* Replace these with the Wi-Fi credentials used by your network. */
#define WIFI_SSID              "SNUC"
#define WIFI_PASSWORD          "snu12345"

/* EMQX public broker settings from your MQTTX screenshot. */
#define MQTT_HOST              "broker.emqx.io"
#define MQTT_PORT              1883
#define MQTT_CLIENT_ID         "stm32_vl53l1x"
#define MQTT_TOPIC             "stm32/tof/data"

#define TOF_ADDR               0x52
#define TOF_TIMING_BUDGET_MS   50
#define TOF_PERIOD_MS          100
#define DATA_TIMEOUT_MS        3000
#define AVG_SAMPLES            5
#define DEBUG_PRINT            1
#define STATUS_OK              0

#define UART_SEND_INTERVAL_MS  500

#define ESP_UART_TIMEOUT       3000
#define MQTT_PACKET_MAX        300

static const uint16_t dev = TOF_ADDR;

static uint16_t avgBuf[AVG_SAMPLES];
static uint8_t avgCount = 0;
static uint8_t avgIndex = 0;

static uint8_t mqttConnected = 0;

/* ---------------- DEBUG PRINTING ---------------- */

static void Uart_Printf(UART_HandleTypeDef *h, const char *fmt, ...)
{
    char buf[220];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n > 0)
    {
        if (n >= (int)sizeof(buf))
            n = (int)sizeof(buf) - 1;

        HAL_UART_Transmit(h, (uint8_t *)buf, (uint16_t)n, 500);
    }
}

#if DEBUG_PRINT
#define LOG(...) Uart_Printf(&huart2, __VA_ARGS__)
#else
#define LOG(...) ((void)0)
#endif

/* ---------------- VL53L1X I2C PLATFORM FUNCTIONS ---------------- */

int8_t VL53L1_WriteMulti(uint16_t d, uint16_t index,
                         uint8_t *pdata, uint32_t count)
{
    return (HAL_I2C_Mem_Write(&hi2c1, d, index, I2C_MEMADD_SIZE_16BIT,
                              pdata, (uint16_t)count, 100) == HAL_OK) ? 0 : -1;
}

int8_t VL53L1_ReadMulti(uint16_t d, uint16_t index,
                        uint8_t *pdata, uint32_t count)
{
    return (HAL_I2C_Mem_Read(&hi2c1, d, index, I2C_MEMADD_SIZE_16BIT,
                             pdata, (uint16_t)count, 100) == HAL_OK) ? 0 : -1;
}

int8_t VL53L1_WrByte(uint16_t d, uint16_t index, uint8_t data)
{
    return VL53L1_WriteMulti(d, index, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t d, uint16_t index, uint16_t data)
{
    uint8_t b[2] = { (uint8_t)(data >> 8), (uint8_t)data };
    return VL53L1_WriteMulti(d, index, b, 2);
}

int8_t VL53L1_WrDWord(uint16_t d, uint16_t index, uint32_t data)
{
    uint8_t b[4] = {
        (uint8_t)(data >> 24), (uint8_t)(data >> 16),
        (uint8_t)(data >> 8), (uint8_t)data
    };
    return VL53L1_WriteMulti(d, index, b, 4);
}

int8_t VL53L1_RdByte(uint16_t d, uint16_t index, uint8_t *pdata)
{
    return VL53L1_ReadMulti(d, index, pdata, 1);
}

int8_t VL53L1_RdWord(uint16_t d, uint16_t index, uint16_t *pdata)
{
    uint8_t b[2];
    if (VL53L1_ReadMulti(d, index, b, 2) != 0)
        return -1;

    *pdata = ((uint16_t)b[0] << 8) | b[1];
    return 0;
}

int8_t VL53L1_RdDWord(uint16_t d, uint16_t index, uint32_t *pdata)
{
    uint8_t b[4];
    if (VL53L1_ReadMulti(d, index, b, 4) != 0)
        return -1;

    *pdata = ((uint32_t)b[0] << 24) |
             ((uint32_t)b[1] << 16) |
             ((uint32_t)b[2] << 8) |
             (uint32_t)b[3];
    return 0;
}

int8_t VL53L1_WaitMs(uint16_t d, int32_t wait_ms)
{
    (void)d;
    if (wait_ms > 0)
        HAL_Delay((uint32_t)wait_ms);
    return 0;
}

/* ---------------- ESP8266 AT COMMAND HELPERS ---------------- */

/*
 * Wait until a response contains the requested text.
 * Bytes received from the ESP8266 are consumed here.
 */
static int ESP_WaitFor(const char *expected, uint32_t timeout_ms)
{
    char window[80] = {0};
    size_t used = 0;
    uint8_t ch;
    uint32_t start = HAL_GetTick();
    size_t expectedLen = strlen(expected);

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (HAL_UART_Receive(&huart1, &ch, 1, 20) == HAL_OK)
        {
            if (used < sizeof(window) - 1)
            {
                window[used++] = (char)ch;
                window[used] = '\0';
            }
            else
            {
                memmove(window, window + 1, sizeof(window) - 2);
                window[sizeof(window) - 2] = (char)ch;
                window[sizeof(window) - 1] = '\0';
                used = sizeof(window) - 1;
            }

            if (expectedLen <= used &&
                strstr(window, expected) != NULL)
                return 1;
        }
    }

    return 0;
}

static void ESP_FlushRx(void)
{
    uint8_t ch;
    while (HAL_UART_Receive(&huart1, &ch, 1, 2) == HAL_OK)
    {
        /* Discard old response bytes before a new AT command. */
    }
}

static int ESP_SendCommand(const char *command,
                           const char *expected,
                           uint32_t timeout_ms)
{
    ESP_FlushRx();

    if (HAL_UART_Transmit(&huart1, (uint8_t *)command,
                          (uint16_t)strlen(command), 1000) != HAL_OK)
        return 0;

    return ESP_WaitFor(expected, timeout_ms);
}

/* Send one binary MQTT packet using ESP8266 AT+CIPSEND. */
static int ESP_SendTCP(const uint8_t *data, uint16_t length)
{
    char command[32];

    snprintf(command, sizeof(command), "AT+CIPSEND=%u\r\n", length);

    ESP_FlushRx();

    if (HAL_UART_Transmit(&huart1, (uint8_t *)command,
                          (uint16_t)strlen(command), 1000) != HAL_OK)
        return 0;

    if (!ESP_WaitFor(">", 3000))
        return 0;

    if (HAL_UART_Transmit(&huart1, (uint8_t *)data, length, 2000) != HAL_OK)
        return 0;

    if (!ESP_WaitFor("SEND OK", 5000))
        return 0;

    return 1;
}

/* ---------------- MQTT PACKET BUILDING ---------------- */

/* MQTT Remaining Length uses a variable-length encoding. */
static uint8_t MQTT_EncodeRemainingLength(uint32_t value, uint8_t *out)
{
    uint8_t count = 0;

    do
    {
        uint8_t encoded = (uint8_t)(value % 128U);
        value /= 128U;

        if (value > 0U)
            encoded |= 0x80U;

        out[count++] = encoded;
    }
    while (value > 0U && count < 4U);

    return count;
}

/* Build and send an MQTT 3.1.1 CONNECT packet (no username/password). */
static int MQTT_Connect(void)
{
    uint8_t packet[100];
    uint8_t pos = 0;
    uint8_t remainingBytes[4];
    uint8_t remainingCount;
    uint16_t clientLen = (uint16_t)strlen(MQTT_CLIENT_ID);
    uint32_t remainingLength = 10U + 2U + clientLen;

    packet[pos++] = 0x10; /* CONNECT fixed header */

    remainingCount = MQTT_EncodeRemainingLength(remainingLength,
                                                remainingBytes);
    for (uint8_t i = 0; i < remainingCount; i++)
        packet[pos++] = remainingBytes[i];

    /* Variable header: protocol name MQTT, level 4, clean session. */
    packet[pos++] = 0x00;
    packet[pos++] = 0x04;
    packet[pos++] = 'M';
    packet[pos++] = 'Q';
    packet[pos++] = 'T';
    packet[pos++] = 'T';
    packet[pos++] = 0x04;
    packet[pos++] = 0x02; /* Clean Session */
    packet[pos++] = 0x00;
    packet[pos++] = 0x3C; /* Keep Alive: 60 seconds */

    /* Payload: Client ID */
    packet[pos++] = (uint8_t)(clientLen >> 8);
    packet[pos++] = (uint8_t)clientLen;

    memcpy(&packet[pos], MQTT_CLIENT_ID, clientLen);
    pos = (uint8_t)(pos + clientLen);

    if (!ESP_SendTCP(packet, pos))
        return 0;

    /* Allow the broker's CONNACK to arrive before publishing. */
    HAL_Delay(300);
    return 1;
}

/* Build and send a QoS 0 MQTT PUBLISH packet. */
static int MQTT_Publish(const char *topic, const char *payload)
{
    uint8_t packet[MQTT_PACKET_MAX];
    uint8_t remainingBytes[4];
    uint8_t remainingCount;
    uint16_t pos = 0;
    uint16_t topicLen = (uint16_t)strlen(topic);
    uint16_t payloadLen = (uint16_t)strlen(payload);
    uint32_t remainingLength = 2U + topicLen + payloadLen;

    if (remainingLength > 268U)
        return 0;

    packet[pos++] = 0x30; /* PUBLISH, QoS 0, not retained */

    remainingCount = MQTT_EncodeRemainingLength(remainingLength,
                                                remainingBytes);
    for (uint8_t i = 0; i < remainingCount; i++)
        packet[pos++] = remainingBytes[i];

    packet[pos++] = (uint8_t)(topicLen >> 8);
    packet[pos++] = (uint8_t)topicLen;

    memcpy(&packet[pos], topic, topicLen);
    pos = (uint16_t)(pos + topicLen);

    memcpy(&packet[pos], payload, payloadLen);
    pos = (uint16_t)(pos + payloadLen);

    return ESP_SendTCP(packet, pos);
}

/* ---------------- WIFI + MQTT CONNECTION ---------------- */

static int ESP_ConnectMQTT(void)
{
    char command[180];

    mqttConnected = 0;

    LOG("Connecting to ESP8266...\r\n");

    if (!ESP_SendCommand("AT\r\n", "OK", 2000))
    {
        LOG("ESP8266 did not respond to AT.\r\n");
        return 0;
    }

    if (!ESP_SendCommand("ATE0\r\n", "OK", 2000))
    {
        LOG("Could not disable AT echo.\r\n");
        return 0;
    }

    if (!ESP_SendCommand("AT+CWMODE=1\r\n", "OK", 3000))
    {
        LOG("Could not set Wi-Fi station mode.\r\n");
        return 0;
    }

    snprintf(command, sizeof(command),
             "AT+CWJAP=\"%s\",\"%s\"\r\n",
             WIFI_SSID, WIFI_PASSWORD);

    LOG("Connecting to Wi-Fi...\r\n");
    if (!ESP_SendCommand(command, "OK", 20000))
    {
        LOG("Wi-Fi connection failed. Check SSID/password.\r\n");
        return 0;
    }

    if (!ESP_SendCommand("AT+CIPMUX=0\r\n", "OK", 3000))
    {
        LOG("Could not set single connection mode.\r\n");
        return 0;
    }

    /* Close any previous TCP session before reconnecting. */
    (void)ESP_SendCommand("AT+CIPCLOSE\r\n", "OK", 1500);
    HAL_Delay(100);

    snprintf(command, sizeof(command),
             "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n",
             MQTT_HOST, MQTT_PORT);

    LOG("Connecting to MQTT broker...\r\n");
    if (!ESP_SendCommand(command, "OK", 10000))
    {
        LOG("TCP connection to MQTT broker failed.\r\n");
        return 0;
    }

    if (!MQTT_Connect())
    {
        LOG("MQTT CONNECT packet failed.\r\n");
        return 0;
    }

    mqttConnected = 1;
    LOG("MQTT connection sequence sent.\r\n");
    return 1;
}

/* ---------------- DISTANCE ZONE ANALYSIS ---------------- */

static void AnalyseDistance(uint16_t distance,
                            const char **zone,
                            const char **suggestion)
{
    if (distance <= 300U)
    {
        *zone = "STOP";
        *suggestion = "Stop and reverse a little";
    }
    else if (distance <= 500U)
    {
        *zone = "TURN_LEFT";
        *suggestion = "Turn left, then re-check the distance";
    }
    else if (distance <= 1000U)
    {
        *zone = "SLOW";
        *suggestion = "Slow down";
    }
    else
    {
        *zone = "CLEAR";
        *suggestion = "Path clear, full speed";
    }
}

/* ---------------- SENSOR INITIALIZATION ---------------- */

static int Tof_Init(void)
{
    uint8_t state = 0;
    uint16_t id = 0;
    uint32_t t0 = HAL_GetTick();

    if (HAL_I2C_IsDeviceReady(&hi2c1, dev, 3, 100) != HAL_OK)
    {
        LOG("ERROR: VL53L1X not detected\r\n");
        return -1;
    }

    do
    {
        if (VL53L1X_BootState(dev, &state) != 0)
            return -2;

        if (state)
            break;

        HAL_Delay(2);
    }
    while ((HAL_GetTick() - t0) < 3000U);

    if (!state)
    {
        LOG("ERROR: sensor boot timeout\r\n");
        return -3;
    }

    if (VL53L1X_GetSensorId(dev, &id) != 0)
        return -4;

    LOG("Sensor ID: 0x%04X\r\n", id);

    if (VL53L1X_SensorInit(dev) != 0)
        return -5;

    if (VL53L1X_SetDistanceMode(dev, 2) != 0)
        return -6;

    if (VL53L1X_SetTimingBudgetInMs(dev, TOF_TIMING_BUDGET_MS) != 0)
        return -7;

    if (VL53L1X_SetInterMeasurementInMs(dev, TOF_PERIOD_MS) != 0)
        return -8;

    if (VL53L1X_StartRanging(dev) != 0)
        return -9;

    return 0;
}

static void Tof_InitRetry(void)
{
    int status;

    while ((status = Tof_Init()) != 0)
    {
        LOG("Sensor init failed (%d), retrying...\r\n", status);
        HAL_Delay(500);
    }

    LOG("VL53L1X ranging started.\r\n");
}

/* ---------------- MOVING AVERAGE ---------------- */

static uint16_t Avg_Add(uint16_t value)
{
    uint32_t sum = 0;

    avgBuf[avgIndex] = value;
    avgIndex = (uint8_t)((avgIndex + 1U) % AVG_SAMPLES);

    if (avgCount < AVG_SAMPLES)
        avgCount++;

    for (uint8_t i = 0; i < avgCount; i++)
        sum += avgBuf[i];

    return (uint16_t)(sum / avgCount);
}

/* ---------------- MAIN ---------------- */


int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();

    HAL_Delay(1000);

    LOG("\r\nVL53L1X -> STM32 -> Wemos -> MQTT\r\n");

    Tof_InitRetry();

    uint32_t lastData = HAL_GetTick();
		uint32_t lastUartSend = HAL_GetTick();
	
    while (1)
    {
        uint8_t ready = 0;
        uint8_t rangeStatus = 0;
        uint16_t distance = 0;

        if (VL53L1X_CheckForDataReady(dev, &ready) != 0)
        {
            LOG("I2C error; restarting sensor\r\n");
            VL53L1X_StopRanging(dev);
            Tof_InitRetry();
            lastData = HAL_GetTick();
            continue;
        }

        if (!ready)
        {
            if ((HAL_GetTick() - lastData) > DATA_TIMEOUT_MS)
            {
                LOG("Sensor timeout; restarting\r\n");
                VL53L1X_StopRanging(dev);
                Tof_InitRetry();
                lastData = HAL_GetTick();
            }

            HAL_Delay(2);
            continue;
        }

        lastData = HAL_GetTick();

        int statusRead =
            VL53L1X_GetRangeStatus(dev, &rangeStatus);

        int distanceRead =
            VL53L1X_GetDistance(dev, &distance);

        VL53L1X_ClearInterrupt(dev);

        if (statusRead != 0 || distanceRead != 0)
        {
            LOG("Sensor read failed\r\n");
            continue;
        }

        
if (rangeStatus == STATUS_OK)
{
    // Update moving average every valid sensor reading
    uint16_t smooth = Avg_Add(distance);

    // Send JSON only once every 500 ms
    if ((HAL_GetTick() - lastUartSend) >=
        UART_SEND_INTERVAL_MS)
    {
        lastUartSend = HAL_GetTick();

        const char *zone;
        const char *suggestion;
        char json[220];

        AnalyseDistance(smooth, &zone, &suggestion);

        snprintf(json, sizeof(json),
            "{\"distance\":%u,\"raw\":%u,"
            "\"zone\":\"%s\","
            "\"suggestion\":\"%s\"}\n",
            smooth, distance, zone, suggestion);

        HAL_StatusTypeDef uartStatus =
            HAL_UART_Transmit(
                &huart1,
                (uint8_t *)json,
                strlen(json),
                1000
            );

        if (uartStatus == HAL_OK)
        {
            LOG("Distance: %u mm | Zone: %s\r\n",
                smooth, zone);

            LOG("JSON sent: %s", json);
        }
        else
        {
            LOG("UART transmission failed\r\n");
        }
    }
}
else
{
    LOG("Invalid range status: %u\r\n",
        rangeStatus);
}

    }
}


/* ---------------- CLOCK CONFIGURATION ---------------- */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 84;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

/* ---------------- I2C1 INITIALIZATION ---------------- */

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
        Error_Handler();
}

/* ---------------- USART1: WEMOS ---------------- */

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
        Error_Handler();
}

/* ---------------- USART2: PUTTY ---------------- */

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
        Error_Handler();
}

/* ---------------- GPIO CLOCKS ---------------- */

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* ---------------- ERROR HANDLER ---------------- */

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
