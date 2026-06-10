/*
 * BH40V200 双电机滑轨控制器
 * ==========================
 * 硬件: STM32F103C8T6 (Blue Pill) + Arduino 框架
 * 驱动器: BH40V200 极驱V2.0 x2
 * 电机: 57BL75S10-230 (极对数2) x2
 * 编码器: E6B2-CWZ6C (1000P/R) x1
 * 屏幕: 0.96" OLED SSD1306 (I2C)
 *
 * 控制逻辑:
 *   - 长按前进 -> 双电机正转慢速 -> 滑轨前进
 *   - 长按后退 -> 双电机反转慢速 -> 滑轨后退
 *   - 松手 -> 双电机停止
 *   - 短按功能键 -> 保存当前位置
 *   - 长按功能键 -> 自动返回保存位置
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

// --- 编码器1 E6B2-CWZ6C (TIM3 编码器模式) -> 电机1 ---
#define PIN_ENC1_A  PA6    // A相 (黑线) -> TIM3_CH1
#define PIN_ENC1_B  PA7    // B相 (白线) -> TIM3_CH2

// --- 编码器2 E6B2-CWZ6C (TIM1 编码器模式) -> 电机2 ---
#define PIN_ENC2_A  PA8    // A相 (黑线) -> TIM1_CH1
#define PIN_ENC2_B  PA9    // B相 (白线) -> TIM1_CH2
// Z相 (橙线) 可选, 暂不使用

// --- 蜂鸣器 ---
#define PIN_BUZZER  PA2     // 有源蜂鸣器 (高电平响)

// --- 功能按键 ---
#define PIN_BTN_RECORD  PB14   // 记录位置 (按下即记忆当前位置)
#define PIN_BTN_RESET   PB15   // 复位 (按下即自动返回保存位置)

// ====================================================================
// 可调参数
// ====================================================================
#define PWM_SLOW_SPEED  35     // 慢速 PWM 占空比 (0-255, 约14%)
#define RETURN_SPEED    30     // 自动返回时的 PWM 占空比
#define ENC_TOLERANCE   5      // 定位容差 (编码器计数)
#define LOOP_DELAY_MS   20     // 主循环间隔 (ms) (缩短以提升定位精度)

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

// 前向声明 (供 handleFunctionButton 调用)
void updateDisplay();

// ====================================================================
// 编码器全局变量
// ====================================================================
// --- 编码器1 (TIM3, 电机1) ---
static volatile int32_t g_enc1Pos = 0;
static uint16_t g_lastEnc1Cnt = 0;

// --- 编码器2 (TIM1, 电机2) ---
static volatile int32_t g_enc2Pos = 0;
static uint16_t g_lastEnc2Cnt = 0;

// --- 共享 ---
static int32_t g_savedPos = 0;              // 保存的目标位置 (用编码器1做主定位)
static bool g_posSaved = false;             // 是否保存过位置
static bool g_isReturning = false;          // 是否正在自动返回

// ====================================================================
// 编码器1 初始化 (TIM3, 引脚 PA6/PA7)
// ====================================================================
void initEncoder1() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  // PA6, PA7 = 浮空输入模式 (CNF=01, MODE=00)
  // 外部已有 2.2kΩ 上拉电阻到 3.3V
  GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6 | GPIO_CRL_CNF7 | GPIO_CRL_MODE7);
  GPIOA->CRL |= GPIO_CRL_CNF6_0 | GPIO_CRL_CNF7_0;   // CNF=01 (浮空输入)

  TIM3->CR1 = 0;  TIM3->SMCR = 0;

  TIM3->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;          // 编码器模式3
  TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;     // TI1, TI2
  TIM3->CCMR1 &= ~(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F | TIM_CCMR1_IC1PSC | TIM_CCMR1_IC2PSC);
  TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
  TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;           // 使能捕获通道
  TIM3->ARR = 0xFFFF;  TIM3->CNT = 0;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->CR1 = TIM_CR1_CEN;

  g_lastEnc1Cnt = 0;  g_enc1Pos = 0;
}

// ====================================================================
// 编码器2 初始化 (TIM1, 引脚 PA8/PA9)
// ====================================================================
void initEncoder2() {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;                     // TIM1 在 APB2 上

  // PA8, PA9 = 浮空输入模式 (CNF=01, MODE=00)
  // 外部已有 2.2kΩ 上拉电阻到 3.3V
  GPIOA->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8 | GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
  GPIOA->CRH |= GPIO_CRH_CNF8_0 | GPIO_CRH_CNF9_0;   // CNF=01 (浮空输入)

  TIM1->CR1 = 0;  TIM1->SMCR = 0;

  TIM1->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;          // 编码器模式3
  TIM1->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;     // TI1, TI2

  // TIM1 的 CCMR1 位域与 TIM3 相同
  TIM1->CCMR1 &= ~(TIM_CCMR1_IC1F | TIM_CCMR1_IC2F | TIM_CCMR1_IC1PSC | TIM_CCMR1_IC2PSC);
  TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
  TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;           // 使能捕获通道
  TIM1->ARR = 0xFFFF;  TIM1->CNT = 0;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->CR1 = TIM_CR1_CEN;

  g_lastEnc2Cnt = 0;  g_enc2Pos = 0;
}

// ====================================================================
// 更新编码器累加位置 (每次主循环调用)
// ====================================================================
void updateEncoders() {
  // 编码器1
  uint16_t cnt1 = TIM3->CNT;
  int16_t delta1 = (int16_t)(cnt1 - g_lastEnc1Cnt);
  g_enc1Pos += delta1;
  g_lastEnc1Cnt = cnt1;

  // 编码器2
  uint16_t cnt2 = TIM1->CNT;
  int16_t delta2 = (int16_t)(cnt2 - g_lastEnc2Cnt);
  g_enc2Pos += delta2;
  g_lastEnc2Cnt = cnt2;
}

int32_t getEnc1Pos() { return g_enc1Pos; }
int32_t getEnc2Pos() { return g_enc2Pos; }

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
  g_isReturning = false;           // 取消自动返回状态
  // BRK 保持 HIGH (不主动刹车, 避免急停冲击)
}

// ====================================================================
// 蜂鸣器: 响 n 下 (有源蜂鸣器, 高电平驱动)
// ====================================================================
void beep(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(120);
    digitalWrite(PIN_BUZZER, LOW);
    if (i < times - 1) delay(150);
  }
}

// ====================================================================
// 带速度参数的电机控制 (供自动返回使用)
// ====================================================================
void setMotorsForwardAtSpeed(uint8_t speed) {
  digitalWrite(PIN_DIR1, HIGH);
  digitalWrite(PIN_DIR2, HIGH);
  digitalWrite(PIN_BRK1, HIGH);
  digitalWrite(PIN_BRK2, HIGH);
  analogWrite(PIN_VSP1, speed);
  analogWrite(PIN_VSP2, speed);
}

void setMotorsReverseAtSpeed(uint8_t speed) {
  digitalWrite(PIN_DIR1, LOW);
  digitalWrite(PIN_DIR2, LOW);
  digitalWrite(PIN_BRK1, HIGH);
  digitalWrite(PIN_BRK2, HIGH);
  analogWrite(PIN_VSP1, speed);
  analogWrite(PIN_VSP2, speed);
}

// ====================================================================
// 自动返回保存位置 (位置闭环控制)
// ====================================================================
void doAutoReturn() {
  int32_t currentPos = getEnc1Pos();
  int32_t error = g_savedPos - currentPos;
  int32_t absErr = error > 0 ? error : -error;

  // 到达目标位置 (容差范围内)
  if (absErr <= ENC_TOLERANCE) {
    stopMotors();
    g_isReturning = false;
    updateDisplay();
    beep(2);             // 到位 → 哔哔两声
    return;
  }

  g_isReturning = true;

  // 用较小的速度匀速靠近目标
  uint8_t speed = RETURN_SPEED;
  if (absErr < 50) speed = 20;   // 接近目标时减速

  if (error > 0) {
    setMotorsForwardAtSpeed(speed);
  } else {
    setMotorsReverseAtSpeed(speed);
  }
}

// ====================================================================
// 按键响应: 记录键 (按下沿触发, 保存位置)
// ====================================================================
void handleRecordButton() {
  static bool lastRecord = HIGH;
  bool recordNow = digitalRead(PIN_BTN_RECORD);
  if (recordNow == LOW && lastRecord == HIGH) {
    g_savedPos = getEnc1Pos();
    g_posSaved = true;
    g_isReturning = false;
    stopMotors();
    delay(20);
    updateDisplay();
    beep(1);           // 记录成功 → 哔一声
  }
  lastRecord = recordNow;
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

  if (g_isReturning) {
    // 自动返回中
    u8g2.drawStr(6, 40, "RETURNING..");
  } else if (g_state == STATE_FORWARD) {
    u8g2.drawStr(20, 40, ">> FORWARD >>");
  } else if (g_state == STATE_REVERSE) {
    u8g2.drawStr(20, 40, "<< REVERSE <<");
  } else if (g_state == STATE_STOP) {
    // 停止时: 显示刚才是否保存过位置
    if (g_posSaved) {
      int32_t dist = getEnc1Pos() - g_savedPos;
      char buf[17];
      snprintf(buf, sizeof(buf), "SAVED +%5ld", (long)dist);
      u8g2.drawStr(10, 40, buf);
    } else {
      u8g2.drawStr(34, 40, "STOPPED");
    }
  }

  // 底部: 双编码器计数
  u8g2.setFont(u8g2_font_6x10_tf);
  char buf[20];
  snprintf(buf, sizeof(buf), "E1:%5ld", (long)getEnc1Pos());
  u8g2.drawStr(2, 56, buf);
  snprintf(buf, sizeof(buf), "E2:%5ld", (long)getEnc2Pos());
  u8g2.drawStr(2, 63, buf);

  if (g_posSaved) {
    u8g2.drawStr(72, 56, "MEM:Y");
  } else {
    u8g2.drawStr(72, 56, "MEM:-");
  }
  if (g_isReturning) {
    u8g2.drawStr(72, 63, "RETURN");
  } else if (g_state == STATE_FORWARD) {
    u8g2.drawStr(72, 63, ">>FWD");
  } else if (g_state == STATE_REVERSE) {
    u8g2.drawStr(72, 63, "<<REV");
  } else {
    u8g2.drawStr(72, 63, "STOP");
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
  pinMode(PIN_BTN_RECORD, INPUT_PULLUP);
  pinMode(PIN_BTN_RESET, INPUT_PULLUP);

  // 蜂鸣器
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  // 编码器1 (TIM3, PA6/PA7) + 编码器2 (TIM1, PA8/PA9)
  initEncoder1();
  initEncoder2();

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
  beep(1);             // 开机确认
  delay(1500);

  stopMotors();
  g_state = STATE_STOP;
  updateDisplay();
}

// ====================================================================
// 主循环
// ====================================================================
void loop() {
  // 更新编码器位置 (必须在每次循环开头)
  updateEncoders();

  // --- 记录按钮 (任何时候按下都保存当前位置) ---
  handleRecordButton();

  // --- 复位按钮 (按住 = 自动返回, 松开 = 停止) ---
  bool resetHeld = (digitalRead(PIN_BTN_RESET) == LOW);

  // 按下的瞬间启动自动返回
  if (resetHeld && g_posSaved && !g_isReturning) {
    g_isReturning = true;
  }

  if (g_isReturning) {
    if (resetHeld) {
      doAutoReturn();             // 继续向目标移动
    } else {
      stopMotors();               // 松手即停
      g_state = STATE_STOP;
      g_isReturning = false;
    }
    updateDisplay();
    delay(LOOP_DELAY_MS);
    return;
  }

  // --- 正常模式: 前进/后退控制 ---
  bool fwdPressed = (digitalRead(PIN_BTN_FWD) == LOW);
  bool revPressed = (digitalRead(PIN_BTN_REV) == LOW);

  MotorState newState;
  if (fwdPressed && !revPressed) {
    newState = STATE_FORWARD;
  } else if (revPressed && !fwdPressed) {
    newState = STATE_REVERSE;
  } else {
    newState = STATE_STOP;
  }

  if (newState != g_state) {
    g_state = newState;
    switch (g_state) {
      case STATE_FORWARD: setMotorsForward(); break;
      case STATE_REVERSE: setMotorsReverse(); break;
      case STATE_STOP:    stopMotors();        break;
    }
    delay(20);
    updateDisplay();
  }

  delay(LOOP_DELAY_MS);
}
