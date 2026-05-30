#include <stdio.h>
#include "pico/stdlib.h"

// テストモード: 正転 → 逆転 → 停止 を duty 0.2 で順繰り（コメントを外して有効化）
#define test
#ifdef test
#define TEST_PHASE_MS  2000u
#endif
#include "pico/multicore.h"
#include "pico/stdio_uart.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "subCore.hpp"

// UART1: GPIO20=TX, GPIO21=RX（PinAssign.md）
#define STDIO_UART_TX_PIN 20
#define STDIO_UART_RX_PIN 21
#define STDIO_UART_BAUD   115200

// ピン定義（PinAssign.md）
#define BUTTON1_PIN  23
#define BUTTON2_PIN  24
#define LED_RUN_PIN  27
#define LED1_PIN     25
#define LED2_PIN     26

#define MOTOR1_PWM_A_PIN  14
#define MOTOR1_PWM_B_PIN  15
#define MOTOR2_PWM_A_PIN  16
#define MOTOR2_PWM_B_PIN  17

// 0x010 の 8 バイト中、この基板が使うモーター番号（0～7）
#define M1  0
#define M2  1

// PWM 周期: 約 1kHz
// f_pwm = 125MHz / (clkdiv * (wrap + 1))
// clkdiv = 64, wrap = 1952 → f_pwm ≒ 1000.06 Hz
#define PWM_WRAP  1952u
#define BUTTON_DUTY  (255 * 2 / 10)   // 約 20% Duty

static uint slice_m1;  // GPIO14,15
static uint slice_m2;  // GPIO16,17

/** 1台分の PWM 出力（slice 1つ = 1モーターの A/B）
 *  duty_x は「ON時間の長さ」として解釈し、出力極性が反転している場合に対応するため
 *  実際のレベルは (PWM_WRAP - duty_x) を設定する。
 */
static void set_motor_duty(uint slice, uint16_t duty_a, uint16_t duty_b) {
    pwm_set_chan_level(slice, PWM_CHAN_A, PWM_WRAP - duty_a * PWM_WRAP / 255);
    pwm_set_chan_level(slice, PWM_CHAN_B, PWM_WRAP - duty_b * PWM_WRAP / 255);
}

static void motor_pwm_init(void) {
    gpio_set_function(MOTOR1_PWM_A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR1_PWM_B_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR2_PWM_A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR2_PWM_B_PIN, GPIO_FUNC_PWM);

    slice_m1 = pwm_gpio_to_slice_num(MOTOR1_PWM_A_PIN);
    slice_m2 = pwm_gpio_to_slice_num(MOTOR2_PWM_A_PIN);

    pwm_config c = pwm_get_default_config();
    pwm_config_set_wrap(&c, PWM_WRAP);
    pwm_config_set_clkdiv_int_frac(&c, 64, 0);
    pwm_init(slice_m1, &c, true);
    pwm_init(slice_m2, &c, true);

    set_motor_duty(slice_m1, 0, 0);
    set_motor_duty(slice_m2, 0, 0);
}

/** 1台分の値(0,1,128,255)を指定 slice の A/B PWM に反映
 *  0 は「無視」ではなく、安全側に倒して Duty=0 (停止) とする。
 */
static void apply_cmd(uint slice, uint8_t val) {
    if (val == 0) {
        set_motor_duty(slice, 0, 0);
        return;
    }
    if (val > 120 && val < 136) {
        set_motor_duty(slice, 255, 255);
        return;
    }
    if (val == 128) {
        set_motor_duty(slice, 0, 0);
        return;
    }
    if (val == 255) {
        set_motor_duty(slice, 255, 0);
        return;
    }
    if (val == 1) {
        set_motor_duty(slice, 0, 255);
        return;
    }
    if (val < 128) {
        set_motor_duty(slice, 0, (128 - val) * 2);
        return;
    }
    if (val > 128) {
        set_motor_duty(slice, (val - 128) * 2, 0);
        return;
    }
}

static void gpio_init_buttons_leds(void) {
    gpio_init(BUTTON1_PIN);
    gpio_set_dir(BUTTON1_PIN, GPIO_IN);
    gpio_pull_up(BUTTON1_PIN);
    gpio_init(BUTTON2_PIN);
    gpio_set_dir(BUTTON2_PIN, GPIO_IN);
    gpio_pull_up(BUTTON2_PIN);

    gpio_init(LED_RUN_PIN);
    gpio_set_dir(LED_RUN_PIN, GPIO_OUT);
    gpio_init(LED1_PIN);
    gpio_set_dir(LED1_PIN, GPIO_OUT);
    gpio_init(LED2_PIN);
    gpio_set_dir(LED2_PIN, GPIO_OUT);
    gpio_put(LED_RUN_PIN, 0);
    gpio_put(LED1_PIN, 0);
    gpio_put(LED2_PIN, 0);
}

int main() {

    stdio_init_all();
    initSubCore();

    gpio_init_buttons_leds();
    motor_pwm_init();

#ifdef test
    printf("Core2026Motor TEST mode (fwd/rev/stop @ duty 0.2, %u ms/phase)\n\n", TEST_PHASE_MS);
#else
    printf("Core2026Interface2 Ready (2 motors)\n\n");
#endif

    can_frame msg;
    absolute_time_t last_can_sec = get_absolute_time();
    absolute_time_t last_led_run = get_absolute_time();
    absolute_time_t last_0x010_rx = get_absolute_time();
    const uint32_t led_run_period_ms = 200;
    bool led_run_on = false;
    bool led1_on = false;
    bool led2_on = false;
    uint8_t motor1_cmd = 128;  // 停止
    uint8_t motor2_cmd = 128;

#ifdef test
    int test_phase = 0;
    absolute_time_t last_test_change = get_absolute_time();
#endif

    while (true) {
        absolute_time_t now = get_absolute_time();

#ifdef test
        // 正転(0.2) → 逆転(0.2) → 停止 を順繰り
        if (absolute_time_diff_us(last_test_change, now) >= (int64_t)TEST_PHASE_MS * 1000) {
            last_test_change = now;
            test_phase = (test_phase + 1) % 3;
        }
        switch (test_phase) {
            case 0:  // 正転
                set_motor_duty(slice_m1, BUTTON_DUTY, 0);
                set_motor_duty(slice_m2, BUTTON_DUTY, 0);
                break;
            case 1:  // 逆転
                set_motor_duty(slice_m1, 0, BUTTON_DUTY);
                set_motor_duty(slice_m2, 0, BUTTON_DUTY);
                break;
            default:  // 停止
                set_motor_duty(slice_m1, 0, 0);
                set_motor_duty(slice_m2, 0, 0);
                break;
        }
#else
        // ボタン: Button1 → 両モーター Duty 0.2 正転, Button2 → 逆転（押していないときは CAN 指令を反映）
        bool btn1 = gpio_get(BUTTON1_PIN);
        bool btn2 = gpio_get(BUTTON2_PIN);
        if (btn1) {
            set_motor_duty(slice_m1, BUTTON_DUTY, 0);
            set_motor_duty(slice_m2, BUTTON_DUTY, 0);
        } else if (btn2) {
            set_motor_duty(slice_m1, 0, BUTTON_DUTY);
            set_motor_duty(slice_m2, 0, BUTTON_DUTY);
        } else {
            apply_cmd(slice_m1, motor1_cmd);
            apply_cmd(slice_m2, motor2_cmd);
            printf("motor1_cmd: %d, motor2_cmd: %d\n", motor1_cmd, motor2_cmd);
        }
#endif

        // LED_RUN: 200ms 周期で明滅
        if (absolute_time_diff_us(last_led_run, now) >= (int64_t)led_run_period_ms * 1000) {
            last_led_run = now;
            led_run_on = !led_run_on;
            gpio_put(LED_RUN_PIN, led_run_on ? 1 : 0);
        }

        // CAN 受信処理
        if (canBufferRx.getMsg(&msg)) {
            led1_on = !led1_on;
            gpio_put(LED1_PIN, led1_on ? 1 : 0);

            if (msg.can_id == 0x010 && msg.can_dlc >= M2+1 && msg.can_dlc >= M1+1) {
                last_0x010_rx = now;
                led2_on = !led2_on;
                gpio_put(LED2_PIN, led2_on ? 1 : 0);

                motor1_cmd = msg.data[M1];
                motor2_cmd = msg.data[M2];
            }
        }

        // 2秒以上 0x010 を受信していない → 強制的に Duty 0
        if (absolute_time_diff_us(last_0x010_rx, now) >= 2000000) {
            motor1_cmd = 128;
            motor2_cmd = 128;
        }

        // 1秒ごと: 0x220 で [0x02+M1] を送信（生存情報）
        if (absolute_time_diff_us(last_can_sec, now) >= 1000000) {
            last_can_sec = now;
            msg.can_id = 0x220;
            msg.can_dlc = 1;
            msg.data[0] = (uint8_t)(0x02 + M1);
            canBufferTx.putMsg(&msg);
        }

        sleep_ms(5);
    }
    return 0;
}
