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
    int pwr = active->getNewVal();
    int mode = targetState->getNewVal();
    float coolTemp = coolingThresh->getNewVal();
    float heatTemp = heatingThresh->getNewVal();
    
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
    return true;
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

void setup() {
  Serial.begin(115200);

  // ステータスLEDの設定（今回はオンボードLEDにしました）
  homeSpan.setStatusPin(kStatusLedPin);
  
  // ファクトリーLED
  homeSpan.setControlPin(kControlPin);
  
  // ペアリングコード
  homeSpan.setPairingCode("00000000"); // ペアリングコードの設定(8桁，ゾロ目等推測されやすいものはiOSに弾かれるらしいです)
  
  // APモードの設定
  homeSpan.setApSSID("SmartRC-Setup"); // APモード用SSID

  // APモード用ボタン
  pinMode(kForceApButtonPin, INPUT_PULLUP); 

  // IR初期化
  irSend.begin();
  acPanasonic.begin();
  acMitsubishi.begin();

  // HomeSpan開始
  homeSpan.begin(Category::Bridges, "Smart RC Bridge");

  new SpanAccessory();  
    new Service::AccessoryInformation();
    new Characteristic::Identify();
    new Characteristic::Name("RC Bridge");
    new Characteristic::Model("v1.0");

  new SpanAccessory();
    new Service::AccessoryInformation();
    new Characteristic::Identify();
    new Characteristic::Name("AC Model Selector");
    ModelSelector *selector = new ModelSelector(); 

  new SpanAccessory();
    new Service::AccessoryInformation();
    new Characteristic::Identify();
    new Characteristic::Name("Main AC");
    new SmartAC(selector); 

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
}