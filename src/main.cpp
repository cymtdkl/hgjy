/*
 * BH40V200 双电机滑轨控制器
 * ==========================
 * 硬件: STM32F103C8T6 (Blue Pill) + Arduino 框架
 * 驱动器: BH40V200 极驱V2.0 x2
 * 电机: 57BL75S10-230 (极对数2) x2
 * 屏幕: 0.96" OLED SSD1306 (I2C)
 *
 * 控制逻辑:
 *   - 长按前进 -> 双电机正转慢速 -> 屏幕显示"前进"
 *   - 长按后退 -> 双电机反转慢速 -> 屏幕显示"后退"
 *   - 松手 -> 双电机停止 -> 屏幕显示"停止"
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// ====================================================================
// 引脚定义 (STM32F103C8T6)
// ====================================================================
// --- 驱动器1 (滑轨一侧) ---
#define PIN_VSP1   PA0    // PWM 调速  -> BH40V200 #1 VSP  (TIM2_CH1)
#define PIN_DIR1   PB0    // 方向控制  -> BH40V200 #1 DIR  (HIGH=正转, LOW=反转)
#define PIN_BRK1   PB1    // 刹车控制  -> BH40V200 #1 BRK  (HIGH=运行, LOW=刹车)

// --- 驱动器2 (滑轨另一侧) ---
#define PIN_VSP2   PA1    // PWM 调速  -> BH40V200 #2 VSP  (TIM2_CH2)
#define PIN_DIR2   PB12   // 方向控制  -> BH40V200 #2 DIR
#define PIN_BRK2   PB13   // 刹车控制  -> BH40V200 #2 BRK

// --- 按键 (按下=接GND, 内部上拉) ---
#define PIN_BTN_FWD  PB10   // 前进按钮 (按下 LOW)
#define PIN_BTN_REV  PB11   // 后退按钮 (按下 LOW)

// --- OLED (I2C) ---
// SCL -> PB6 (I2C1_SCL)
// SDA -> PB7 (I2C1_SDA)

// ====================================================================
// 可调参数
// ====================================================================
#define PWM_SLOW_SPEED  35     // 慢速 PWM 占空比 (0-255, 约14%)
// 如果觉得太快, 把这个值改小(如20); 太慢就改大(如50)
#define LOOP_DELAY_MS   50     // 主循环间隔 (ms)

// ====================================================================
// 全局对象
// ====================================================================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ====================================================================
// 电机状态
// ====================================================================
enum MotorState {
  STATE_STOP,
  STATE_FORWARD,
  STATE_REVERSE
};

static MotorState g_state = STATE_STOP;

// ====================================================================
// PWM 初始化
// ====================================================================
void initPWM() {
  analogWrite(PIN_VSP1, 0);
  analogWrite(PIN_VSP2, 0);
}

// ====================================================================
// 电机控制函数
// ====================================================================
void setMotorsForward() {
  digitalWrite(PIN_DIR1, HIGH);   // 正转
  digitalWrite(PIN_DIR2, HIGH);
  digitalWrite(PIN_BRK1, HIGH);   // 释放刹车
  digitalWrite(PIN_BRK2, HIGH);
  analogWrite(PIN_VSP1, PWM_SLOW_SPEED);   // 慢速 PWM
  analogWrite(PIN_VSP2, PWM_SLOW_SPEED);
}

void setMotorsReverse() {
  digitalWrite(PIN_DIR1, LOW);    // 反转
  digitalWrite(PIN_DIR2, LOW);
  digitalWrite(PIN_BRK1, HIGH);   // 释放刹车
  digitalWrite(PIN_BRK2, HIGH);
  analogWrite(PIN_VSP1, PWM_SLOW_SPEED);
  analogWrite(PIN_VSP2, PWM_SLOW_SPEED);
}

void stopMotors() {
  analogWrite(PIN_VSP1, 0);       // PWM 归零 -> 滑轨自然停止
  analogWrite(PIN_VSP2, 0);
  // BRK 保持 HIGH (不主动刹车, 避免急停冲击)
}

// ====================================================================
// 屏幕故障恢复 (电机噪声可能干扰 I2C 总线)
// ====================================================================
void recoverDisplay() {
  // 重启 I2C 总线
  Wire.end();
  delay(5);
  Wire.begin();
  Wire.setClock(100000);   // 100kHz 标准模式, 比默认更快恢复
  // 重新初始化 OLED
  u8g2.begin();
  u8g2.setPowerSave(0);
}

// ====================================================================
// 屏幕显示 (带 I2C 抗干扰)
// ====================================================================
void updateDisplay() {
  // 每次刷新前重启 I2C, 防止上次被噪声打断的传输卡死总线
  recoverDisplay();

  u8g2.clearBuffer();

  // 边框
  u8g2.drawFrame(0, 0, 128, 64);
  u8g2.drawLine(0, 20, 128, 20);
  u8g2.drawLine(0, 48, 128, 48);

  // 标题
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.drawStr(8, 14, "Dual-Motor Slide");

  // 状态文字 (中间大号显示)
  u8g2.setFont(u8g2_font_10x20_tf);
  switch (g_state) {
    case STATE_FORWARD:
      u8g2.drawStr(20, 40, ">> FORWARD >>");
      break;
    case STATE_REVERSE:
      u8g2.drawStr(20, 40, "<< REVERSE <<");
      break;
    case STATE_STOP:
      u8g2.drawStr(34, 40, "STOPPED");
      break;
  }

  // 底部: 电机图标
  u8g2.setFont(u8g2_font_6x10_tf);
  if (g_state == STATE_FORWARD) {
    u8g2.drawStr(10, 60, "M1:>>  M2:>>");
  } else if (g_state == STATE_REVERSE) {
    u8g2.drawStr(10, 60, "M1:<<  M2:<<");
  } else {
    u8g2.drawStr(18, 60, "M1:--  M2:--");
  }

  u8g2.sendBuffer();
}

// ====================================================================
// 初始化
// ====================================================================
void setup() {
  // PWM
  pinMode(PIN_VSP1, OUTPUT);
  pinMode(PIN_VSP2, OUTPUT);
  initPWM();

  // 方向 (默认正转)
  pinMode(PIN_DIR1, OUTPUT);
  pinMode(PIN_DIR2, OUTPUT);
  digitalWrite(PIN_DIR1, HIGH);
  digitalWrite(PIN_DIR2, HIGH);

  // 刹车 (默认释放)
  pinMode(PIN_BRK1, OUTPUT);
  pinMode(PIN_BRK2, OUTPUT);
  digitalWrite(PIN_BRK1, HIGH);
  digitalWrite(PIN_BRK2, HIGH);

  // 按键
  pinMode(PIN_BTN_FWD, INPUT_PULLUP);
  pinMode(PIN_BTN_REV, INPUT_PULLUP);

  // OLED (I2C 100kHz 标准模式, 提高抗干扰)
  Wire.begin();
  Wire.setClock(100000);
  u8g2.begin();

  // 开机画面
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x13_tf);
  u8g2.drawStr(12, 24, "Slide Control");
  u8g2.drawStr(12, 42, "System Ready");
  u8g2.sendBuffer();
  delay(1500);

  stopMotors();
  g_state = STATE_STOP;
  updateDisplay();
}

// ====================================================================
// 主循环
// ====================================================================
void loop() {
  // 读按键 (LOW = 按下)
  bool fwdPressed = (digitalRead(PIN_BTN_FWD) == LOW);
  bool revPressed = (digitalRead(PIN_BTN_REV) == LOW);

  // 判定状态: 同时按或都不按 -> 停止
  MotorState newState;
  if (fwdPressed && !revPressed) {
    newState = STATE_FORWARD;
  } else if (revPressed && !fwdPressed) {
    newState = STATE_REVERSE;
  } else {
    newState = STATE_STOP;
  }

  // 状态变化时执行动作 + 刷新屏幕
  if (newState != g_state) {
    g_state = newState;
    switch (g_state) {
      case STATE_FORWARD: setMotorsForward(); break;
      case STATE_REVERSE: setMotorsReverse(); break;
      case STATE_STOP:    stopMotors();        break;
    }
    delay(20);        // 等电源稳定后再刷屏幕
    updateDisplay();
  }

  delay(LOOP_DELAY_MS);
}
