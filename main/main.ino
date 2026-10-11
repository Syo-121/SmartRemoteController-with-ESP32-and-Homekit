/*
SmartRemoteController
X : @_syo_syo_syo_
GitHub : https://github.com/Syo-121/SmartRemoteController-with-ESP32-and-Homekit
*/

//ライブラリ
#include <Arduino.h>
// HomeKit
#include <HomeSpan.h>
// 赤外線通信関連
#include <IRremoteESP8266.h> 
#include <IRsend.h> 
#include <ir_Panasonic.h>
#include <ir_Mitsubishi.h>

// 公開しない設定（ペアリングコード，APIトークン）は secrets.h に書く（secrets.h.example を参照）
#if __has_include("secrets.h")
#include "secrets.h"
#endif

// GPIO定義
const uint16_t kIrLedPin = 1;      // D0: 赤外線LED
const int kForceApButtonPin = 2;   // D1: APモード起動用ボタン
const int kControlPin = 3;         // D2: HomeSpan標準のコントロールボタン（ファクトリーリセット用）
const int kStatusLedPin = 21;      // Onboard LED: ステータス表示

// パナソニックと三菱の新旧モデルに対応
enum AcProtocol { PROTO_PANASONIC, PROTO_MITSUBISHI };

struct AcModelDef {
  const char* name;
  AcProtocol protocol;
  int modelType;
};

// モデル定義(よく使う順に上から）
const AcModelDef AC_MODELS[] = {
  { "Mitsubishi Modern (144)",  PROTO_MITSUBISHI, 0 }, // 10%
  { "Mitsubishi Old (112)",     PROTO_MITSUBISHI, 0 }, // 20%
  { "Panasonic Standard (RKR)", PROTO_PANASONIC,  kPanasonicRkr },
  { "Panasonic (JKE)",          PROTO_PANASONIC,  kPanasonicJke },
  { "Panasonic (LKE)",          PROTO_PANASONIC,  kPanasonicLke },
  { "Mitsubishi (136)",         PROTO_MITSUBISHI, 0 },
  { "Panasonic (NKE)",          PROTO_PANASONIC,  kPanasonicNke }
};
const int NUM_MODELS = sizeof(AC_MODELS) / sizeof(AC_MODELS[0]);

IRsend irSend(kIrLedPin);
IRPanasonicAc acPanasonic(kIrLedPin);
IRMitsubishiAC acMitsubishi(kIrLedPin); 

// パナソニック照明(HK9488等)用 家製協フォーマット 40bit
// [カスタマーコード16bit 0x344A][パリティ4bit][コマンド12bit][パリティ8bit]
const uint16_t kPanaLightCustomer = 0x344A;
const uint8_t  kPanaLightParity1  = 0x9;   // カスタマーコード4bitごとのXOR
const uint16_t kPanaLightBits     = 40;

// コマンド(12bit)：[チャンネル][CH1, CH2, CH3]
enum PanaLightCmd { LIGHT_ON, LIGHT_OFF, LIGHT_BRIGHTER, LIGHT_DARKER, LIGHT_FULL, LIGHT_NIGHT, LIGHT_SLEEP30 };
const uint16_t PANA_LIGHT_CMDS[][3] = {
  { 0x0B4, 0x0AC, 0x0BC },  // 点灯（普段）
  { 0x0F4, 0x0EC, 0x0FC },  // 消灯
  { 0x054, 0x04C, 0x05C },  // 明るい
  { 0x0D4, 0x0CC, 0x0DC },  // 暗い
  { 0x034, 0x02C, 0x03C },  // 全灯
  { 0x074, 0x06C, 0x07C },  // 常夜灯
  { 0xC85, 0xC55, 0xCCD },  // おやすみ30
};
const int kPanaLightChannel = 1;  // リモコンのチャンネル(1〜3)

// 照明のHomeKit上の見せ方
// 1：1つのアクセサリにまとめる（タイル1つ，長押しで各ボタン）
// 0：ボタンごとに別々のアクセサリにする（タイル6つ）
#define LIGHT_GROUPED 0

// 40bitフレームを組み立てる
uint64_t buildPanaLightFrame(uint16_t cmd) {
  uint8_t hi = (kPanaLightParity1 << 4) | ((cmd >> 8) & 0x0F);
  uint8_t lo = cmd & 0xFF;
  uint8_t parity2 = hi ^ lo;
  return ((uint64_t)kPanaLightCustomer << 24) | ((uint64_t)hi << 16) | ((uint64_t)lo << 8) | parity2;
}

// 家製協フォーマットのタイミング（T=425us，搬送波38kHz）
// ※ライブラリのsendPanasonic64()は搬送波36.7kHzなので照明用に自前で送る
const uint16_t kAehaT    = 425;
const uint16_t kAehaFreq = 38000;
const uint32_t kAehaGap  = 75000;  // フレーム間隔(us)
const uint8_t  kAehaDuty = 33;

void sendPanaLightCode(uint16_t code, uint16_t repeat) {
  uint64_t frame = buildPanaLightFrame(code);
  irSend.sendGeneric(8 * kAehaT, 4 * kAehaT,   // リーダー
                     kAehaT, 3 * kAehaT,       // 1
                     kAehaT, kAehaT,           // 0
                     kAehaT, kAehaGap,         // トレーラー
                     frame, kPanaLightBits, kAehaFreq, true, repeat, kAehaDuty);
  Serial.printf("[Light] frame=0x%010llX repeat=%d\n", frame, repeat);
}

void sendPanaLight(PanaLightCmd cmd) {
  uint16_t code = PANA_LIGHT_CMDS[cmd][kPanaLightChannel - 1];
  // 点灯・消灯・全灯・常夜灯は何度送っても同じ結果なので念のため1回リピート
  // 明るい・暗い・おやすみ30は2回効いてしまうのでリピートしない
  bool idempotent = (cmd == LIGHT_ON || cmd == LIGHT_OFF || cmd == LIGHT_FULL || cmd == LIGHT_NIGHT);
  sendPanaLightCode(code, idempotent ? 1 : 0);
}

// シリアルモニタ用テストコマンド
// @L <ボタン番号0-6> <チャンネル1-3>
//   ボタン番号：0点灯 1消灯 2明るい 3暗い 4全灯 5常夜灯 6おやすみ30
//   例）@L 4 1 → CH1の全灯を送信
void lightTestCommand(const char *buf) {
  int cmd = -1, ch = kPanaLightChannel;
  sscanf(buf + 1, "%d %d", &cmd, &ch);
  if (cmd < 0 || cmd > LIGHT_SLEEP30 || ch < 1 || ch > 3) {
    Serial.println("usage: @L <button 0-6> <ch 1-3>");
    return;
  }
  sendPanaLightCode(PANA_LIGHT_CMDS[cmd][ch - 1], 0);
}

// @B：38kHzの搬送波を5秒間出し続ける（スマホカメラでLEDの発光を確認する用）
void irBlinkTestCommand(const char *buf) {
  Serial.println("[IR] 5秒間連続発光します");
  irSend.enableIROut(kAehaFreq, kAehaDuty);
  for (int i = 0; i < 100; i++) {
    irSend.mark(50000);  // 50ms × 100回 = 5秒
    yield();
  }
  irSend.space(0);
  Serial.println("[IR] 終了");
}

// Homekit用クラス定義
// モデル選択用疑似照明
struct ModelSelector : Service::LightBulb {
  SpanCharacteristic *power;
  SpanCharacteristic *level;
  ModelSelector() : Service::LightBulb() {
    power = new Characteristic::On(true, true);
    level = new Characteristic::Brightness(10, true); 
    level->setRange(10, NUM_MODELS * 10, 10);
  }
  int getModelIndex() {
    int val = level->getVal();
    if (val < 10) val = 10;
    if (val > NUM_MODELS * 10) val = NUM_MODELS * 10;
    return (val / 10) - 1;
  }
  boolean update() override { return true; }
};

// エアコン本体
struct SmartAC : Service::HeaterCooler {
  SpanCharacteristic *active;
  SpanCharacteristic *currentTemp;
  SpanCharacteristic *targetState;
  SpanCharacteristic *currentState;
  SpanCharacteristic *coolingThresh;
  SpanCharacteristic *heatingThresh;
  ModelSelector *selector;

  SmartAC(ModelSelector *sel) : Service::HeaterCooler() {
    selector = sel;
    active = new Characteristic::Active(0);
    currentTemp = new Characteristic::CurrentTemperature(24);
    targetState = new Characteristic::TargetHeaterCoolerState(0);
    currentState = new Characteristic::CurrentHeaterCoolerState(0);
    coolingThresh = new Characteristic::CoolingThresholdTemperature(26);
    heatingThresh = new Characteristic::HeatingThresholdTemperature(20);
    
    // 範囲設定
    coolingThresh->setRange(16, 30, 1);
    heatingThresh->setRange(16, 30, 1);
  }

  boolean update() override {
    apply(active->getNewVal(), targetState->getNewVal(), coolingThresh->getNewVal(), heatingThresh->getNewVal());
    return true;
  }

  // HTTP APIなどHomeKit以外から操作する時用（HomeKit側の表示も更新する）
  void setState(int pwr, int mode, float coolTemp, float heatTemp) {
    active->setVal(pwr);
    targetState->setVal(mode);
    coolingThresh->setVal(coolTemp);
    heatingThresh->setVal(heatTemp);
    apply(pwr, mode, coolTemp, heatTemp);
  }

  // 赤外線を送信して現在状態を更新する
  void apply(int pwr, int mode, float coolTemp, float heatTemp) {
    int modelIdx = selector->getModelIndex();
    const AcModelDef* model = &AC_MODELS[modelIdx];

    float targetTemp = (mode == 1) ? heatTemp : coolTemp;
    currentTemp->setVal(targetTemp);

    sendIR(model, pwr, mode, targetTemp);

    if (pwr == 0) currentState->setVal(0);
    else {
      if (mode == 1) currentState->setVal(2);
      else if (mode == 2) currentState->setVal(3);
      else currentState->setVal(1);
    }
  }

  // 赤外線送信用の関数
  void sendIR(const AcModelDef* model, int power, int mode, float temp) {
    if (model->protocol == PROTO_PANASONIC) {
      acPanasonic.setModel(static_cast<panasonic_ac_remote_model_t>(model->modelType));
      if (power == 0) acPanasonic.off();
      else {
        acPanasonic.on();
        acPanasonic.setTemp((uint8_t)temp);
        if (mode == 1) acPanasonic.setMode(kPanasonicAcHeat);
        else if (mode == 2) acPanasonic.setMode(kPanasonicAcCool);
        else acPanasonic.setMode(kPanasonicAcAuto);
        acPanasonic.setFan(kPanasonicAcFanAuto);
        acPanasonic.setSwingVertical(kPanasonicAcSwingVAuto);
      }
      acPanasonic.send();
    } else if (model->protocol == PROTO_MITSUBISHI) {
      if (power == 0) acMitsubishi.off();
      else {
        acMitsubishi.on();
        acMitsubishi.setTemp((uint8_t)temp);
        if (mode == 1) acMitsubishi.setMode(kMitsubishiAcHeat);
        else if (mode == 2) acMitsubishi.setMode(kMitsubishiAcCool);
        else acMitsubishi.setMode(kMitsubishiAcAuto);
        acMitsubishi.setFan(kMitsubishiAcFanAuto);
        acMitsubishi.setVane(kMitsubishiAcVaneAuto);
      }
      acMitsubishi.send();
    }
  }
};

// 照明(パナソニック HK9488)
// ON → 点灯（普段），OFF → 消灯
struct SmartLight : Service::LightBulb {
  SpanCharacteristic *power;

  SmartLight() : Service::LightBulb() {
    power = new Characteristic::On(false);
  }

  boolean update() override {
    sendPanaLight(power->getNewVal() ? LIGHT_ON : LIGHT_OFF);
    return true;
  }

  // HTTP APIなどHomeKit以外から操作する時用（HomeKit側の表示も更新する）
  void setPower(bool on) {
    power->setVal(on);
    sendPanaLight(on ? LIGHT_ON : LIGHT_OFF);
  }
};

// HTTP APIから操作するためのインスタンス
ModelSelector *gSelector = nullptr;
SmartAC *gAC = nullptr;
SmartLight *gLight = nullptr;

// 全灯・常夜灯を送った時は照明をON表示にそろえる
void sendPanaLightButton(PanaLightCmd cmd) {
  sendPanaLight(cmd);
  if ((cmd == LIGHT_FULL || cmd == LIGHT_NIGHT) && gLight) gLight->power->setVal(true);
}

// 照明リモコンのボタン（全灯・常夜灯など）
// ONにすると信号を1回送り，すぐに自動でOFFに戻る（疑似プッシュスイッチ）
struct LightButton : Service::Switch {
  SpanCharacteristic *power;
  PanaLightCmd cmd;
  const uint32_t kResetMs = 0;  // 0：信号を送った直後のloopでOFFに戻す

  LightButton(PanaLightCmd c, const char *name) : Service::Switch() {
    cmd = c;
    power = new Characteristic::On(false);
    new Characteristic::ConfiguredName(name);  // まとめた時のボタン名
  }

  boolean update() override {
    if (power->getNewVal()) sendPanaLightButton(cmd);
    return true;
  }

  void loop() override {
    if (power->getVal() && power->timeVal() > kResetMs) power->setVal(false);
  }
};

struct LightButtonDef {
  PanaLightCmd cmd;
  const char *name;
};
const LightButtonDef LIGHT_BUTTONS[] = {
  { LIGHT_FULL,     "全灯" },
  { LIGHT_NIGHT,    "常夜灯" },
  { LIGHT_BRIGHTER, "明るい" },
  { LIGHT_DARKER,   "暗い" },
  { LIGHT_SLEEP30,  "おやすみ30" },
};

void setup() {
  Serial.begin(115200);

  // ステータスLEDの設定（今回はオンボードLEDにしました）
  homeSpan.setStatusPin(kStatusLedPin);
  
  // ファクトリーLED
  homeSpan.setControlPin(kControlPin);
  
  // ペアリングコード
  // ペアリングコードの設定(8桁，ゾロ目等推測されやすいものはiOSに弾かれるらしいです)
  // 未設定の場合はHomeSpanの初期値(466-37-726)になる
#ifdef SMARTRC_PAIRING_CODE
  homeSpan.setPairingCode(SMARTRC_PAIRING_CODE);
#endif
  
  // APモードの設定
  homeSpan.setApSSID("SmartRC-Setup"); // APモード用SSID

  // APモード用ボタン
  pinMode(kForceApButtonPin, INPUT_PULLUP); 

  // IR初期化
  irSend.begin();
  acPanasonic.begin();
  acMitsubishi.begin();

  // HTTP API（Wi-Fi接続後に起動）
  homeSpan.setWifiCallback(startHttpApi);

  // HomeSpan開始（ホスト名：smartrc.local）
  homeSpan.setHostNameSuffix("");
  homeSpan.begin(Category::Bridges, "Smart RC Bridge", "smartrc");

  // 照明テスト用シリアルコマンド
  new SpanUserCommand('L', "<button> <ch> - 照明の赤外線テスト送信", lightTestCommand);
  new SpanUserCommand('B', "- 赤外線LEDを5秒間連続点灯（カメラでの確認用）", irBlinkTestCommand);

  new SpanAccessory();  
    new Service::AccessoryInformation();
    new Characteristic::Identify();
    new Characteristic::Name("RC Bridge");
    new Characteristic::Model("v1.0");

  new SpanAccessory();
    new Service::AccessoryInformation();
    new Characteristic::Identify();
    new Characteristic::Name("AC Model Selector");
    gSelector = new ModelSelector();

  new SpanAccessory();
    new Service::AccessoryInformation();
    new Characteristic::Identify();
    new Characteristic::Name("Main AC");
    gAC = new SmartAC(gSelector);

  new SpanAccessory();
    new Service::AccessoryInformation();
    new Characteristic::Identify();
    new Characteristic::Name("Main Light");
    gLight = new SmartLight();
#if LIGHT_GROUPED
    // 同じアクセサリにボタンを追加
    for (const LightButtonDef &b : LIGHT_BUTTONS) new LightButton(b.cmd, b.name);
#else
  // ボタンごとに別のアクセサリを作る
  for (const LightButtonDef &b : LIGHT_BUTTONS) {
    new SpanAccessory();
      new Service::AccessoryInformation();
      new Characteristic::Identify();
      new Characteristic::Name(b.name);
      new LightButton(b.cmd, b.name);
  }
#endif

  // APモード判定
  if (digitalRead(kForceApButtonPin) == LOW) {
    Serial.println("AP Mode Start!");
    
    // APモードへの移行を知らせる
    for(int i=0; i<3; i++) {
        digitalWrite(kStatusLedPin, LOW); delay(200);
        digitalWrite(kStatusLedPin, HIGH); delay(200);
    }
    
    // "A"を押したことにしてアクセスポイントモードへ
    homeSpan.processSerialCommand("A");
  }
}

void loop() {
  homeSpan.poll();
  handleHttpApi();
}