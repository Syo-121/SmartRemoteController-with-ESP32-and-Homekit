"""SmartRemoteController 用 MCP サーバー

ESP32 の HTTP API (main/http_api.ino) を呼び出して，AI からエアコン・照明を操作する．

環境変数:
  SMARTRC_URL    ESP32 の URL（既定: http://smartrc.local:8080）
  SMARTRC_TOKEN  main/secrets.h の SMARTRC_API_TOKEN と同じ値（必須）
"""

import os
from typing import Literal, Optional

import httpx
from mcp.server.mcpserver import MCPServer

BASE_URL = os.environ.get("SMARTRC_URL", "http://smartrc.local:8080").rstrip("/")
TOKEN = os.environ.get("SMARTRC_TOKEN", "")
TIMEOUT = 10.0

mcp = MCPServer("smartrc")


def _request(method: str, path: str, params: Optional[dict] = None) -> dict:
    if not TOKEN:
        return {"error": "SMARTRC_TOKEN is not set on the MCP server"}
    try:
        r = httpx.request(
            method,
            f"{BASE_URL}{path}",
            params={k: v for k, v in (params or {}).items() if v is not None},
            headers={"Authorization": f"Bearer {TOKEN}"},
            timeout=TIMEOUT,
        )
    except httpx.HTTPError as e:
        return {"error": f"cannot reach SmartRC at {BASE_URL}: {e}"}
    try:
        return r.json()
    except ValueError:
        return {"error": f"HTTP {r.status_code}: {r.text}"}


@mcp.tool()
def get_status() -> dict:
    """エアコンと照明の現在の状態を取得する．

    赤外線リモコンなので実機の状態は測れず，最後に送った指令の内容を返す
    （純正リモコンで操作した場合はずれる）．
    利用できるエアコンの機種一覧 (ac_models) と照明ボタン一覧 (light_buttons) も含む．
    """
    return _request("GET", "/status")


@mcp.tool()
def set_ac(
    power: Optional[Literal["on", "off"]] = None,
    mode: Optional[Literal["auto", "heat", "cool"]] = None,
    temperature: Optional[int] = None,
    model: Optional[int] = None,
) -> dict:
    """エアコンを操作する．指定した項目だけ変更し，残りは現在の値のまま送信する．

    Args:
        power: "on" で運転，"off" で停止
        mode: "auto"（自動），"heat"（暖房），"cool"（冷房）
        temperature: 設定温度（16〜30℃）．暖房時は暖房の，それ以外は冷房の設定温度になる
        model: エアコンの機種番号（get_status の ac_models を参照）．通常は変更不要
    """
    if temperature is not None and not 16 <= temperature <= 30:
        return {"error": "temperature must be 16-30"}
    return _request(
        "POST",
        "/ac",
        {"power": power, "mode": mode, "temp": temperature, "model": model},
    )


@mcp.tool()
def set_light(power: Literal["on", "off"]) -> dict:
    """照明を点灯（普段の明るさ）または消灯する．

    Args:
        power: "on" で点灯（普段），"off" で消灯
    """
    return _request("POST", "/light", {"power": power})


@mcp.tool()
def press_light_button(
    button: Literal["full", "night", "brighter", "darker", "sleep30"],
) -> dict:
    """照明リモコンのボタンを1回押す．

    Args:
        button:
            "full"     全灯（最大の明るさ）
            "night"    常夜灯
            "brighter" 1段階明るくする（押すたびに効くので必要な回数だけ呼ぶ）
            "darker"   1段階暗くする（同上）
            "sleep30"  おやすみ30（30分後に消灯）
    """
    return _request("POST", "/light/button", {"button": button})


if __name__ == "__main__":
    mcp.run()
