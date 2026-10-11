# SmartRC MCP サーバー

ESP32 の HTTP API（`main/http_api.ino`）を MCP ツールとして公開し，Claude などの AI からエアコン・照明を操作できるようにします．

```
AI（Claude Code / Claude Desktop など）
   │ MCP (stdio)
   ▼
smartrc_mcp.py（パソコン）
   │ HTTP :8080（Bearer トークン認証）
   ▼
ESP32 ── 赤外線 ──▶ エアコン・照明
```

## ツール

| ツール | 内容 |
|---|---|
| `get_status` | 現在の状態（最後に送った指令），機種一覧，照明ボタン一覧 |
| `set_ac` | エアコンの運転/停止・モード（auto/heat/cool）・温度（16〜30）・機種．指定した項目だけ変更 |
| `set_light` | 照明の点灯（普段）/消灯 |
| `press_light_button` | 照明ボタン（full / night / brighter / darker / sleep30）を1回押す |

## セットアップ

### 1. ESP32 側

1. `main/secrets.h.example` を `main/secrets.h` にコピーし，`SMARTRC_API_TOKEN` にランダムな文字列を設定
   （`python -c "import secrets;print(secrets.token_urlsafe(24))"` で生成できます）
2. 書き込み後，シリアルモニタに `[API] HTTP API started: http://smartrc.local:8080/ (192.168.x.x)` と出れば起動成功
3. 動作確認
   ```
   curl -H "Authorization: Bearer <トークン>" http://smartrc.local:8080/status
   ```

### 2. パソコン側

```
cd mcp
python -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
```

### 3. AI に登録

環境変数

- `SMARTRC_TOKEN`：`secrets.h` と同じトークン（必須）
- `SMARTRC_URL`：ESP32 の URL（既定 `http://smartrc.local:8080`．tailnet 経由など `.local` が引けない場合は `http://<IPアドレス>:8080`）

**Claude Code**

```
claude mcp add smartrc -e SMARTRC_TOKEN=<トークン> -- <このフォルダ>\.venv\Scripts\python.exe <このフォルダ>\smartrc_mcp.py
```

**Claude Desktop**（`claude_desktop_config.json`）

```json
{
  "mcpServers": {
    "smartrc": {
      "command": "<このフォルダ>\\.venv\\Scripts\\python.exe",
      "args": ["<このフォルダ>\\smartrc_mcp.py"],
      "env": { "SMARTRC_TOKEN": "<トークン>" }
    }
  }
}
```

## 注意

- 赤外線は一方通行なので，状態は「最後に送った指令」です．純正リモコンで操作すると実機とずれます．
- トークンを知っている人は誰でも操作できます．`secrets.h` やトークンを公開しないでください．
