#include <config/appconfig.h>
#include <stm32l4xx_hal.h>
#include <stm-hal/hal-uart.hpp>
#include <serial_reception/serial_reception.hpp>

static constexpr uint16_t SERIAL_RX_BUFFER_SIZE = 512;

struct UartConfig
{
    USART_TypeDef* uart;
    uint32_t speed;
    DMA_Channel_TypeDef* tx_channel;
    uint8_t tx_request;
    DMA_Channel_TypeDef* rx_channel;
    uint8_t rx_request;
};

static constexpr UartConfig s_uart_config[UART_TYPE_TOTAL] =
{
    { LPUART1, 921600, DMA1_Channel2, DMA_REQUEST_LPUART1_TX, DMA1_Channel3, DMA_REQUEST_LPUART1_RX },    // UART_TYPE_DEBUG_SERIAL
}

struct UartData
{
    UART_HandleTypeDef uart;
    DMA_HandleTypeDef tx_dma;
    DMA_HandleTypeDef rx_dma;
    SerialReception<SERIAL_RX_BUFFER_SIZE> rx;
    FinishCb tx_end_cb;
    void* param;
};

static UartData s_uart_data[UART_TYPE_TOTAL];

void hal_uart_init_default()
{
    __HAL_RCC_LPUART1_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
}

void hal_uart_init(const uint8_t type, FinishCb finish_tx_cb, void* param)
{
    assert(type >= 0 && type < UART_TYPE_TOTAL);

    const UartConfig* config = &s_uart_config[type];

    // Safe finish transmite callback function and parameter
    s_uart_data[type].tx_end_cb = finish_tx_cb;
    s_uart_data[type].param = param,

    // Configure UART
    USART_TypeDef* uart = s_uart_data[type].uart;

    uart->Instance = config->uart;
    uart->Init.BaudRate = config->speed;
    uart->Init.WordLength = UART_WORDLENGTH_8B;
    uart->Init.StopBits = UART_STOPBITS_1;
    uart->Init.Parity = UART_PARITY_NONE;
    uart->Init.Mode = UART_MODE_TX_RX;
    uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart->Init.OverSampling = UART_OVERSAMPLING_16;
    uart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
    uart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    assert(HAL_UART_Init(uart) == HAL_OK &&
        HAL_UARTEx_SetTxFifoThreshold(uart, UART_TXFIFO_THRESHOLD_1_8) == HAL_OK &&
        HAL_UARTEx_SetRxFifoThreshold(uart, UART_RXFIFO_THRESHOLD_1_8) == HAL_OK &&
        HAL_UARTEx_DisableFifoMode(uart) == HAL_OK);

    // Configure TX DMA
    if (config->tx_channel != nullptr)
    {
        DMA_HandleTypeDef* tx_dma = &s_uart_data[type].tx_dma;

        tx_dma->Instance = config->tx_channel;
        tx_dma->Init.Request = config->tx_request;
        tx_dma->Init.Direction = DMA_MEMORY_TO_PERIPH;
        tx_dma->Init.PeriphInc = DMA_PINC_DISABLE;
        tx_dma->Init.MemInc = DMA_MINC_ENABLE;
        tx_dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        tx_dma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        tx_dma->Init.Mode = DMA_NORMAL;
        tx_dma->Init.Priority = DMA_PRIORITY_LOW;
        assert(HAL_DMA_Init(tx_dma) == HAL_OK);

        __HAL_LINKDMA(uart, hdmatx, &tx_dma);
    }

    // Configure RX DMA
    if (config->rx_channel != nullptr)
    {
        DMA_HandleTypeDef* rx_dma = &s_uart_data[type].rx_dma;

        rx_dma->Instance = config->rx_channel;
        rx_dma->Init.Request = config->rx_request;;
        rx_dma->Init.Direction = DMA_PERIPH_TO_MEMORY;
        rx_dma->Init.PeriphInc = DMA_PINC_DISABLE;
        rx_dma->Init.MemInc = DMA_MINC_ENABLE;
        rx_dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        rx_dma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        rx_dma->Init.Mode = DMA_CIRCULAR;
        rx_dma->Init.Priority = DMA_PRIORITY_LOW;
        assert(HAL_DMA_Init(rx_dma) == HAL_OK);

        __HAL_LINKDMA(uart, hdmarx, &rx_dma);
    }

    // TODO: INIT RX
}

void hal_uart_write(const UartTypes type, const uint8_t* data, uint32_t size)
{
    assert(type >= 0 && type < UART_TYPE_TOTAL);
    USART_TypeDef* uart = s_uart_data[type].uart;

    HAL_UART_Transmit_DMA(uart, data, size);
}

uint32_t hal_uart_read(const UartTypes type, uint8_t* data, uint32_t size)
{
    assert(type >= 0 && type < UART_TYPE_TOTAL);
    return s_uart_data[type].rx.read(data, size);
}

extern "C" {
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    for (int i = 0; i < UART_TYPE_TOTAL; i++)
    {
        UartData* data = &s_uart_data[i];

        if (huart->Instance == data->uart)
        {
            if (data->tx_end_cb != nullptr)
            {
                data->tx_end_cb(data->param);
            }
        }
        return;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    for (int i = 0; i < UART_TYPE_TOTAL; i++)
    {
        UartData* data = &s_uart_data[i];

        if (huart->Instance == data->uart)
        {
            data->rx.error();
        }
        return;
    }
}

}