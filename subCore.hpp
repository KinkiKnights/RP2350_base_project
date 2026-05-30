#ifndef SUBCORE_HPP
#define SUBCORE_HPP

#include <stdint.h>
#include <stdio.h>
#include <atomic>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "include/mcp25625/mcp25625.hpp"

/*==============================================
CANリングバッファの定義
===============================================*/
struct CanMsg {
    uint32_t id;
    uint8_t  DLC;
    uint8_t  data[8];
};

const static uint8_t MAX_MSG_BUFFER = 100;

class CanBuffer{
private:
    can_frame rxMsg[MAX_MSG_BUFFER];
    can_frame txMsg[MAX_MSG_BUFFER];
    std::atomic<int16_t> writeIndex;
    std::atomic<int16_t> readIndex;
public:
    CanBuffer(){
        writeIndex.store(0);
        readIndex.store(0);
    }
    /**
     * @brief メッセージを取得する
     * @param msg メッセージを格納する変数のポインタ
     * @return メッセージが取得できたか
     */
    bool getMsg(can_frame* msg){
        int16_t index = readIndex.load();
        if (writeIndex.load() == index){
            return false;
        }
        *msg = rxMsg[index];
        readIndex.store((index + 1) % MAX_MSG_BUFFER);

        msg->can_id = rxMsg[index].can_id;
        msg->can_dlc = rxMsg[index].can_dlc;
        for (uint8_t i = 0; i < msg->can_dlc; i++){
            msg->data[i] = rxMsg[index].data[i];
        }
        return true;
    }

    /** 
     * @brief メッセージを追加する
     * @param msg メッセージを格納する変数のポインタ
     * @return メッセージが追加できたか
     */
    bool putMsg(can_frame* msg){
        if ((writeIndex.load() + 1) % MAX_MSG_BUFFER == readIndex.load()){
            return false;
        }
        int16_t index = writeIndex.load();
        rxMsg[index] = *msg;   
        writeIndex.store((index + 1) % MAX_MSG_BUFFER);
        
        txMsg[index].can_id = msg->can_id;
        txMsg[index].can_dlc = msg->can_dlc;
        for (uint8_t i = 0; i < msg->can_dlc; i++){
            txMsg[index].data[i] = msg->data[i];
        }
        return true;

    }
};

CanBuffer canBufferTx;
CanBuffer canBufferRx;

// ==== can関連ピンの定義 ====
#define SPI_PORT spi1
#define PIN_MISO 28
#define PIN_CS   29
#define PIN_SCK  30
#define PIN_MOSI 31
#define PIN_RST  32

MCP25625 can0(SPI_PORT, PIN_CS, PIN_MOSI, PIN_MISO, PIN_SCK);

uint32_t count_test_run = 0;

/*==============================================
サブコアのメインループ
===============================================*/
bool is_subcore_running = false;

void subCoreLoop(void) {

    is_subcore_running = true;
    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_put(PIN_RST, 1);
    sleep_ms(200);
    can_frame msg;

    if (can0.reset() != MCP25625::ERROR::ERROR_OK) {is_subcore_running = false; printf("CAN reset failed\n");return;}
    if (can0.setBitrate(CAN_1000KBPS, MCP_16MHZ) != MCP25625::ERROR::ERROR_OK) {is_subcore_running = false; printf("CAN bitrate failed\n"); return;}
    if (can0.setFilterMask(MCP25625::MASK0, false, 0x0) != MCP25625::ERROR::ERROR_OK) {is_subcore_running = false; printf("CAN filter mask failed\n"); return;}
    if (can0.setFilter(MCP25625::RXF0, false, 0x0) != MCP25625::ERROR::ERROR_OK) {is_subcore_running = false; printf("CAN filter failed\n"); return;}
    if (can0.setNormalMode() != MCP25625::ERROR::ERROR_OK) {is_subcore_running = false; printf("CAN Normal mode failed\n"); return;}
    // if (can0.setNormalMode() != MCP25625::ERROR::ERROR_OK) {is_subcore_running = false; return;}
        
    while (true) {
        count_test_run++;
        // エラークリア
        if (can0.checkError()){
            uint16_t eflg = can0.getErrorFlags();
            can0.clearRXnOVR();
            can0.clearMERR();
            can0.clearERRIF();
        }

        MCP25625::ERROR err = can0.readMessage(&msg);
        while (err != MCP25625::ERROR_NOMSG){
            can0.clearInterrupts();
            canBufferRx.putMsg(&msg);
            err = can0.readMessage(&msg);
        }
        
        while (canBufferTx.getMsg(&msg)){
            can0.sendMessage(&msg);
            printf("Send CAN");
        }
    }
}

/*==============================================
サブコアの起動(メインコアで呼び出し)
===============================================*/
void initSubCore(void) {
    multicore_launch_core1(subCoreLoop);
}

#endif // SUBCORE_HPP