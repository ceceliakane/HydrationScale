#pragma once
#include <Arduino.h>

// Capacitive touch pad (RC delay + LM339 comparator), from capTouch.cpp / CP4.
//   D2 -> 2 Mohm -> pad node -> comparator (+);  Vref -> comparator (-)
//   comparator output -> 10k pull-up -> D3
#define CAPTOUCH_DEBUG 1     // 1 = print a live td/threshold line every second + press/release events

void  capTouchBegin();        // measures the untouched baseline (keep hands OFF the pad)
void  capTouchRecalibrate();  // same, callable later (main does this when you send 'c')
bool  readTouch();            // call every loop: runs 10 cycles (~33 ms), returns debounced PRESSED state
float capTouchDelayUs();      // last averaged delay td (us), for logging/debug
