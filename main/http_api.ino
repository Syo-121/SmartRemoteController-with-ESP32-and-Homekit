/*
HTTP API（パソコンやAIから操作する用）
ポート8080（80番はHomeSpanが使用）

認証：全リクエストに "Authorization: Bearer <トークン>" ヘッダが必要
      トークンは secrets.h の SMARTRC_API_TOKEN で設定（secrets.h.example を参照）

GET  /status                          現在の状態（最後に送った指令）を返す
POST /ac            power=on|off  mode=auto|heat|cool  temp=16〜30  model=0〜6
                    （指定した項目だけ変更．指定しなかった項目は現在の値のまま）
POST /light         power=on|off      （点灯（普段）／消灯）
POST /light/button  button=full|night|brighter|darker|sleep30

例）curl -X POST -H "Authorization: Bearer xxx" "http://smartrc.local:8080/ac?power=on&mode=cool&temp=26"
*/

#include <WebServer.h>

// secrets.h は main.ino で読み込み済み
#ifndef SMARTRC_API_TOKEN
#define SMARTRC_API_TOKEN ""  // 未設定の場合はAPIを起動しない
#endif

const uint16_t kHttpApiPort = 8080;
WebServer httpApi(kHttpApiPort);
bool httpApiStarted = false;

const char *AC_MODE_NAMES[] = { "auto", "heat", "cool" };  // TargetHeaterCoolerStateの0,1,2

struct LightButtonApiDef {
  const char *name;
  PanaLightCmd cmd;
};
const LightButtonApiDef LIGHT_BUTTON_API[] = {
  { "full",     LIGHT_FULL },
  { "night",    LIGHT_NIGHT },
  { "brighter", LIGHT_BRIGHTER },
  { "darker",   LIGHT_DARKER },
  { "sleep30",  LIGHT_SLEEP30 },
};

// ---------- 共通処理 ----------

void sendJson(int code, const String &body) {
  httpApi.send(code, "application/json; charset=utf-8", body);
}

void sendError(int code, const String &msg) {
  sendJson(code, "{\"error\":\"" + msg + "\"}");
}

bool checkAuth() {
  String expected = String("Bearer ") + SMARTRC_API_TOKEN;
  if (httpApi.header("Authorization") == expected) return true;
  sendError(401, "unauthorized");
  return false;
}

// "on"/"off" を解釈する．不正な値ならfalseを返す
bool parseOnOff(const String &s, bool &out) {
  if (s == "on" || s == "1" || s == "true") { out = true; return true; }
  if (s == "off" || s == "0" || s == "false") { out = false; return true; }
  return false;
}

String statusJson() {
  int pwr = gAC->active->getVal();
  int mode = gAC->targetState->getVal();
  int coolTemp = gAC->coolingThresh->getVal();
  int heatTemp = gAC->heatingThresh->getVal();
  int modelIdx = gSelector->getModelIndex();

  String s = "{";
  s += "\"ac\":{";
  s += "\"power\":\"" + String(pwr ? "on" : "off") + "\",";
  s += "\"mode\":\"" + String(AC_MODE_NAMES[mode]) + "\",";
  s += "\"temp\":" + String(mode == 1 ? heatTemp : coolTemp) + ",";
  s += "\"cool_temp\":" + String(coolTemp) + ",";
  s += "\"heat_temp\":" + String(heatTemp) + ",";
  s += "\"model\":{\"index\":" + String(modelIdx) + ",\"name\":\"" + AC_MODELS[modelIdx].name + "\"}";
  s += "},";
  s += "\"light\":{\"power\":\"" + String(gLight->power->getVal() ? "on" : "off") + "\"},";
  s += "\"ac_models\":[";
  for (int i = 0; i < NUM_MODELS; i++) {
    if (i) s += ",";
    s += "{\"index\":" + String(i) + ",\"name\":\"" + AC_MODELS[i].name + "\"}";
  }
  s += "],";
  s += "\"light_buttons\":[";
  for (size_t i = 0; i < sizeof(LIGHT_BUTTON_API) / sizeof(LIGHT_BUTTON_API[0]); i++) {
    if (i) s += ",";
    s += "\"" + String(LIGHT_BUTTON_API[i].name) + "\"";
  }
  s += "],";
  s += "\"note\":\"state is the last command sent, not measured\"";
  s += "}";
  return s;
}

// ---------- ハンドラ ----------

void handleStatus() {
  if (!checkAuth()) return;
  sendJson(200, statusJson());
}

void handleAc() {
  if (!checkAuth()) return;

  int pwr = gAC->active->getVal();
  int mode = gAC->targetState->getVal();
  float coolTemp = gAC->coolingThresh->getVal();
  float heatTemp = gAC->heatingThresh->getVal();
  int modelIdx = gSelector->getModelIndex();

  if (httpApi.hasArg("power")) {
    bool on;
    if (!parseOnOff(httpApi.arg("power"), on)) return sendError(400, "power must be on or off");
    pwr = on ? 1 : 0;
  }
  if (httpApi.hasArg("mode")) {
    String m = httpApi.arg("mode");
    int found = -1;
    for (int i = 0; i < 3; i++) if (m == AC_MODE_NAMES[i]) found = i;
    if (found < 0) return sendError(400, "mode must be auto, heat or cool");
    mode = found;
  }
  if (httpApi.hasArg("temp")) {
    int t = httpApi.arg("temp").toInt();
    if (t < 16 || t > 30) return sendError(400, "temp must be 16-30");
    // 暖房なら暖房用，それ以外は冷房用の設定温度を変更（SmartAC::applyと同じ判定）
    if (mode == 1) heatTemp = t;
    else coolTemp = t;
  }
  if (httpApi.hasArg("model")) {
    String ms = httpApi.arg("model");
    int idx = ms.toInt();
    if (ms.length() == 0 || !isDigit(ms[0]) || idx < 0 || idx >= NUM_MODELS)
      return sendError(400, "model must be 0-" + String(NUM_MODELS - 1));
    modelIdx = idx;
  }

  gSelector->level->setVal((modelIdx + 1) * 10);
  gAC->setState(pwr, mode, coolTemp, heatTemp);
  sendJson(200, statusJson());
}

void handleLight() {
  if (!checkAuth()) return;
  bool on;
  if (!httpApi.hasArg("power") || !parseOnOff(httpApi.arg("power"), on))
    return sendError(400, "power must be on or off");
  gLight->setPower(on);
  sendJson(200, statusJson());
}

void handleLightButton() {
  if (!checkAuth()) return;
  String b = httpApi.arg("button");
  for (const LightButtonApiDef &d : LIGHT_BUTTON_API) {
    if (b == d.name) {
      sendPanaLightButton(d.cmd);
      sendJson(200, statusJson());
      return;
    }
  }
  sendError(400, "button must be full, night, brighter, darker or sleep30");
}

// ---------- 起動・ループ ----------

// Wi-Fi接続時にHomeSpanから呼ばれる
void startHttpApi() {
  if (httpApiStarted) return;
  if (strlen(SMARTRC_API_TOKEN) == 0) {
    Serial.println("[API] SMARTRC_API_TOKEN が未設定のためHTTP APIは無効です（secrets.h を作成してください）");
    return;
  }

  const char *headers[] = { "Authorization" };
  httpApi.collectHeaders(headers, 1);

  httpApi.on("/status", HTTP_GET, handleStatus);
  httpApi.on("/ac", HTTP_POST, handleAc);
  httpApi.on("/light", HTTP_POST, handleLight);
  httpApi.on("/light/button", HTTP_POST, handleLightButton);
  httpApi.onNotFound([]() { sendError(404, "not found"); });

  httpApi.begin();
  httpApiStarted = true;
  Serial.printf("[API] HTTP API started: http://smartrc.local:%d/ (%s)\n", kHttpApiPort, WiFi.localIP().toString().c_str());
}

void handleHttpApi() {
  if (httpApiStarted) httpApi.handleClient();
}
