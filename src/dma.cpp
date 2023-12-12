#include "dma.hpp"
#include "stm32_def.h"

// So close to not hardcoding :(
static constexpr size_t DMA_LIDAR_BUFFER_SIZE = 47;
static volatile uint8_t dma_lidar_buffer[DMA_LIDAR_BUFFER_SIZE] = {0};

void DMASerialRx::MX_DMA_Init() {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    /* DMAMUX1_OVR_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMAMUX1_OVR_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMAMUX1_OVR_IRQn);
}

void DMASerialRx::MX_UART_Init(uint32_t baud) {
    raw_serial.Init.BaudRate = baud;
    raw_serial.Init.WordLength = UART_WORDLENGTH_8B;
    raw_serial.Init.StopBits = UART_STOPBITS_1;
    raw_serial.Init.Parity = UART_PARITY_NONE;
    raw_serial.Init.Mode = UART_MODE_RX;
    raw_serial.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    raw_serial.Init.OverSampling = UART_OVERSAMPLING_16;
    raw_serial.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    raw_serial.Init.ClockPrescaler = UART_PRESCALER_DIV1;
// TODO below two may not be needed
    raw_serial.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
    raw_serial.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
    if (HAL_UART_Init(&raw_serial) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&raw_serial, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&raw_serial, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&raw_serial) != HAL_OK) {
        Error_Handler();
    }
}

void DMASerialRx::init(uint32_t baud, std::function<void(uint8_t *)> rx_callback) {
    MX_DMA_Init();
    MX_UART_Init(baud);
    cb = std::move(rx_callback);
}

void DMASerialRx::begin() {
    // Call first DMA receive to start the train
    HAL_UART_Receive_DMA(&raw_serial, const_cast<uint8_t *>(dma_lidar_buffer), 1);
}

DMASerialRx::DMASerialRx(USART_TypeDef *uart) {
    raw_serial.Instance = uart;
}

extern "C" {
// Called on each DMA finish
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // Call user callback
    dmaSerialRx5.cb(const_cast<uint8_t *>(dma_lidar_buffer));

    // Recurse
    HAL_UART_Receive_DMA(huart, const_cast<uint8_t *>(dma_lidar_buffer), DMA_LIDAR_BUFFER_SIZE);
}

// Called on DMA error
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    HAL_UART_Receive_DMA(huart, const_cast<uint8_t *>(dma_lidar_buffer), DMA_LIDAR_BUFFER_SIZE);
}
}