/**
 * InterCoreQueue.hpp
 * RP2350 マルチコア間で「10ワード×最大20スロット以上」を安全にやり取りするキュー
 *
 * 安全のポイント:
 * 1) 送信側: スロットへ全ワードを書き終えてから __dsb() を入れ、その後にのみ
 *    write_idx を更新する（release  semantics）。→ 受信側が write_idx を見て「書いた」と
 *    判断するため、書き込み途中のスロットを参照しない。
 * 2) 受信側: write_idx を acquire で読み、読める件数を確認してから __dsb() の後でスロットを読む。
 * 3) リングは SPSC（片方のコアが書く／もう片方が読む）に限定し、インデックス競合を防ぐ。
 *
 * 方式: リングバッファのみ。ハードウェア FIFO は 8 エントリのため 20 件スタックに不向きなので
 *       使わず、インデックス＋メモリバリアで完全に制御。最大 ICQ_RING_SIZE 件までスタック可能。
 */

#ifndef INTER_CORE_QUEUE_HPP
#define INTER_CORE_QUEUE_HPP

#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/sync.h"

#if defined(__cplusplus) && __cplusplus >= 201103L
#include <atomic>
#define ICQ_USE_ATOMICS 1
#else
#include <stddef.h>
#define ICQ_USE_ATOMICS 0
#endif

// 1メッセージ = 10ワード（要変更時はここだけ変更）
#define ICQ_WORDS_PER_MSG  10
// リングのスロット数（20程度スタック想定）
#define ICQ_RING_SIZE      32

// 1スロットのデータ（10ワード）
struct InterCoreMsg {
    uint32_t words[ICQ_WORDS_PER_MSG];
};

#if ICQ_USE_ATOMICS
typedef std::atomic<uint32_t> icq_index_t;
#else
typedef volatile uint32_t icq_index_t;
#endif

// ========== Core0 → Core1 用キュー ==========
static InterCoreMsg  icq_ring_0to1[ICQ_RING_SIZE];
static icq_index_t   icq_write_0to1;  // Core0 のみ更新
static icq_index_t   icq_read_0to1;   // Core1 のみ更新

// Core0: 1件送信。満杯ならブロック。
static inline void inter_core_push_0to1(const uint32_t* words) {
    uint32_t w;
#if ICQ_USE_ATOMICS
    w = icq_write_0to1.load(std::memory_order_relaxed);
    while ((w - icq_read_0to1.load(std::memory_order_acquire)) >= ICQ_RING_SIZE) {
        tight_loop_contents();
        w = icq_write_0to1.load(std::memory_order_relaxed);
    }
#else
    while ((icq_write_0to1 - icq_read_0to1) >= ICQ_RING_SIZE) {
        tight_loop_contents();
    }
    w = icq_write_0to1;
#endif
    uint32_t slot = w % ICQ_RING_SIZE;
    for (int i = 0; i < ICQ_WORDS_PER_MSG; i++) {
        icq_ring_0to1[slot].words[i] = words[i];
    }
    __dsb();  // スロットへの全書き込みを他コアに見えるようにしてから write_idx を進める
#if ICQ_USE_ATOMICS
    icq_write_0to1.store(w + 1, std::memory_order_release);
#else
    icq_write_0to1 = w + 1;
#endif
}

// Core1: 1件受信。データが来るまでブロック（スピン）。
static inline void inter_core_pop_0to1(uint32_t* words) {
#if ICQ_USE_ATOMICS
    while (icq_read_0to1.load(std::memory_order_relaxed) >=
           icq_write_0to1.load(std::memory_order_acquire)) {
        tight_loop_contents();
    }
    uint32_t r = icq_read_0to1.load(std::memory_order_relaxed);
#else
    while (icq_read_0to1 >= icq_write_0to1) {
        tight_loop_contents();
    }
    __dsb();  // write_idx の読み取りを確定させてからスロットを読む
    uint32_t r = icq_read_0to1;
#endif
    uint32_t slot = r % ICQ_RING_SIZE;
    for (int i = 0; i < ICQ_WORDS_PER_MSG; i++) {
        words[i] = icq_ring_0to1[slot].words[i];
    }
#if ICQ_USE_ATOMICS
    __dsb();
    icq_read_0to1.store(r + 1, std::memory_order_release);
#else
    __dsb();
    icq_read_0to1 = r + 1;
#endif
}

// Core1 から見た「読める件数」（ノンブロック判定用）
static inline uint32_t inter_core_available_0to1(void) {
#if ICQ_USE_ATOMICS
    return icq_write_0to1.load(std::memory_order_acquire) -
           icq_read_0to1.load(std::memory_order_relaxed);
#else
    return icq_write_0to1 - icq_read_0to1;
#endif
}

// ========== Core1 → Core0 用キュー ==========
static InterCoreMsg  icq_ring_1to0[ICQ_RING_SIZE];
static icq_index_t   icq_write_1to0;  // Core1 のみ更新
static icq_index_t   icq_read_1to0;   // Core0 のみ更新

// Core1: 1件送信。満杯ならブロック。
static inline void inter_core_push_1to0(const uint32_t* words) {
    uint32_t w;
#if ICQ_USE_ATOMICS
    w = icq_write_1to0.load(std::memory_order_relaxed);
    while ((w - icq_read_1to0.load(std::memory_order_acquire)) >= ICQ_RING_SIZE) {
        tight_loop_contents();
        w = icq_write_1to0.load(std::memory_order_relaxed);
    }
#else
    while ((icq_write_1to0 - icq_read_1to0) >= ICQ_RING_SIZE) {
        tight_loop_contents();
    }
    w = icq_write_1to0;
#endif
    uint32_t slot = w % ICQ_RING_SIZE;
    for (int i = 0; i < ICQ_WORDS_PER_MSG; i++) {
        icq_ring_1to0[slot].words[i] = words[i];
    }
    __dsb();
#if ICQ_USE_ATOMICS
    icq_write_1to0.store(w + 1, std::memory_order_release);
#else
    icq_write_1to0 = w + 1;
#endif
}

// Core0: 1件受信。ブロック。
static inline void inter_core_pop_1to0(uint32_t* words) {
#if ICQ_USE_ATOMICS
    while (icq_read_1to0.load(std::memory_order_relaxed) >=
           icq_write_1to0.load(std::memory_order_acquire)) {
        tight_loop_contents();
    }
    uint32_t r = icq_read_1to0.load(std::memory_order_relaxed);
#else
    while (icq_read_1to0 >= icq_write_1to0) {
        tight_loop_contents();
    }
    __dsb();
    uint32_t r = icq_read_1to0;
#endif
    uint32_t slot = r % ICQ_RING_SIZE;
    for (int i = 0; i < ICQ_WORDS_PER_MSG; i++) {
        words[i] = icq_ring_1to0[slot].words[i];
    }
#if ICQ_USE_ATOMICS
    __dsb();
    icq_read_1to0.store(r + 1, std::memory_order_release);
#else
    __dsb();
    icq_read_1to0 = r + 1;
#endif
}

// Core0 から見た「読める件数」
static inline uint32_t inter_core_available_1to0(void) {
#if ICQ_USE_ATOMICS
    return icq_write_1to0.load(std::memory_order_acquire) -
           icq_read_1to0.load(std::memory_order_relaxed);
#else
    return icq_write_1to0 - icq_read_1to0;
#endif
}

// ========== 構造体で渡すヘルパー ==========
static inline void inter_core_push_0to1_msg(const InterCoreMsg* msg) {
    inter_core_push_0to1(msg->words);
}
static inline void inter_core_pop_0to1_msg(InterCoreMsg* msg) {
    inter_core_pop_0to1(msg->words);
}
static inline void inter_core_push_1to0_msg(const InterCoreMsg* msg) {
    inter_core_push_1to0(msg->words);
}
static inline void inter_core_pop_1to0_msg(InterCoreMsg* msg) {
    inter_core_pop_1to0(msg->words);
}

// キュー使用前に一度だけ呼ぶ（Core0 で core1 起動前に推奨）
static inline void inter_core_queue_init(void) {
#if ICQ_USE_ATOMICS
    icq_write_0to1.store(0);
    icq_read_0to1.store(0);
    icq_write_1to0.store(0);
    icq_read_1to0.store(0);
#else
    icq_write_0to1 = 0;
    icq_read_0to1  = 0;
    icq_write_1to0 = 0;
    icq_read_1to0  = 0;
#endif
}

/*
 * 使用例:
 *   Core0 → Core1:  Core0 で inter_core_push_0to1(words); Core1 で inter_core_pop_0to1(words);
 *   Core1 → Core0:  Core1 で inter_core_push_1to0(words); Core0 で inter_core_pop_1to0(words);
 *   ノンブロック:   if (inter_core_available_0to1() > 0) { inter_core_pop_0to1(words); }
 *   構造体で:      InterCoreMsg msg; ...; inter_core_push_0to1_msg(&msg);
 */

#endif // INTER_CORE_QUEUE_HPP
