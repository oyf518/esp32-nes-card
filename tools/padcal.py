#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手柄校准程序(ESP32 NES 模拟器配套)
================================
用法: python3 padcal.py  然后浏览器打开 http://127.0.0.1:8788

网页上高亮一个键 -> 手柄上按它 -> 自动识别该键在 HID 报文里的位置
全部按完后"保存到板子",布局写进板子 NVS,以后开机自动生效(无需重新烧录)。

串口协议(与固件 nes/main/ble_pad.c 的 cal_task 对应):
  发 CAL ON              -> 板子回 CALREADY,此后每条 HID 报文回一行 EVT <id> <len> <hex>
  发 CAL GET             -> 回 CALLAY <21字节hex> CONN=<0/1>
  发 CAL SAVE <21字节hex> -> 写 NVS 并生效,回 CALOK / CALERR
  发 CAL FORGET          -> 清空本端全部 bond,回 CALOK
  连接状态变化            -> 板子主动输出 CALCONN <0/1>
"""
import json
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import serial

SERIAL_PORT = os.environ.get("PADCAL_PORT", "/dev/cu.usbmodem13301")
BAUD = 115200
HTTP_PORT = int(os.environ.get("PADCAL_HTTP", "8788"))
NES_BIT = {"A": 0x01, "B": 0x02, "SELECT": 0x04, "START": 0x08}
# 校准顺序:NES 功能名 + 中文提示 + 网页高亮元素 id
KEYS = [
    ("SELECT", "选择键(SELECT)"),
    ("START", "开始键(START)"),
    ("A", "A 键(你想当 NES 的 A)"),
    ("B", "B 键(你想当 NES 的 B)"),
    ("UP", "十字键 上"),
    ("DOWN", "十字键 下"),
    ("LEFT", "十字键 左"),
    ("RIGHT", "十字键 右"),
]
DIRS = {"UP", "DOWN", "LEFT", "RIGHT"}


class Bridge:
    """串口读写 + 校准状态机,全局单例"""

    def __init__(self):
        self.ser = None
        self.wlock = threading.Lock()
        self.ready = False       # 板子回过 CALREADY
        self.callay = None       # CAL GET 的回执
        self.conn = None         # None=未知 0/1=断开/已连
        self.evts = []           # [(time, bytes)] 最近 64 条
        self.save_status = None  # None/ok/err
        self.forget_at = 0
        self.forget_status = None
        # 校准状态机
        self.st = {
            "phase": "idle",     # idle / waiting / done
            "sub": "quiet",      # quiet / press / release(每个键的三段式)
            "idx": 0,
            "key2loc": {},       # key -> ('hat',byte,val) | ('btn',byte,bit)
            "loc2key": {},
            "conflict": None,    # str
            "arm_t": 0.0,
            "baseline": None,    # bytes,按下前的空闲报文
        }

    # ---------- 串口 ----------
    def open_port(self):
        try:
            # 关键:open 之前就设好 dtr/rts=False,避免 pyserial 默认拉高
            # 触发 USB-Serial-JTAG 的复位时序,把板子打进 DOWNLOAD 等待态
            s = serial.Serial()
            s.port = SERIAL_PORT
            s.baudrate = BAUD
            s.timeout = 0.15
            s.dtr = False
            s.rts = False
            s.open()
            self.ser = s
            print("串口已打开:", SERIAL_PORT)
        except Exception as e:
            self.ser = None
            print("串口未就绪:", e)

    def send(self, line):
        if os.environ.get("PADCAL_DEBUG"):
            print("[TX]", line[:60])
        with self.wlock:
            if not self.ser:
                self.open_port()
            if self.ser:
                try:
                    self.ser.write((line + "\n").encode())
                    return True
                except Exception:
                    self.ser = None
        return False

    def keepalive(self):
        # 无条件周期发 CAL ON:板子重启后校准模式清零,靠它自动恢复 EVT 流
        while True:
            self.send("CAL ON")
            time.sleep(2)

    def reader(self):
        buf = b""
        while True:
            if not self.ser:
                time.sleep(1)
                self.open_port()
                continue
            try:
                chunk = self.ser.read(512)
            except Exception:
                time.sleep(1)
                self.open_port()
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    self.handle_line(line.decode("utf-8", "replace").strip())
                except Exception as e:
                    print("行处理异常:", e)

    def handle_line(self, line):
        if line and os.environ.get("PADCAL_DEBUG"):
            print("[RX]", line[:120])
        if line.startswith("CALREADY"):
            self.ready = True
            print("校准通道就绪")
        elif line.startswith("CALCONN"):
            try:
                self.conn = int(line.split("=")[-1].strip())
            except ValueError:
                pass
        elif line.startswith("CALOK"):
            if time.time() - self.forget_at < 5:
                self.forget_status = "ok"
            else:
                self.save_status = "ok"
        elif line.startswith("CALLAY"):
            self.callay = line
        elif line.startswith("CALERR"):
            self.save_status = "err"
        elif line.startswith("EVT"):
            m = re.match(r"EVT (\d+) (\d+) ([0-9A-Fa-f]+)\s*$", line)
            if m:
                data = bytes.fromhex(m.group(3))
                self.evts.append((time.time(), data))
                self.evts = self.evts[-64:]

    # ---------- 校准状态机 ----------
    def start(self):
        self.st.update(phase="waiting", sub="quiet", idx=0, key2loc={}, loc2key={},
                       conflict=None, arm_t=time.time(),
                       baseline=self._quiet_baseline())

    def reset(self):
        self.st.update(phase="idle", sub="quiet", idx=0, key2loc={}, loc2key={},
                       conflict=None)

    def _quiet_baseline(self):
        """最近一次 EVT 报文作为'空闲态'基线(按下前的样子)"""
        return self.evts[-1][1] if self.evts else None

    def poll(self):
        """每 150ms 调一次。每个键三段式:
        quiet(安静0.6s,记基线) -> press(等与基线不同的报文=按下,过滤释放沿)
        -> release(安静0.6s) -> 下一个键"""
        st = self.st
        if st["phase"] != "waiting":
            return
        if not self.evts:
            return
        now = time.time()
        last_t = self.evts[-1][0]
        sub = st.get("sub", "quiet")

        if sub == "quiet":
            if now - last_t >= 0.6:
                st["baseline"] = self.evts[-1][1]
                st["sub"] = "press"
            return

        if sub == "press":
            news = [e for e in self.evts if e[0] > st["arm_t"]]
            if not news:
                return
            if now - last_t < 0.25:
                return          # 报文还在连发(按着/抖动),等停
            press = news[0][1]
            base = st["baseline"]
            if base is None or len(base) != len(press):
                st["baseline"] = press
                st["arm_t"] = time.time()
                return
            diffs = [(i, base[i], press[i]) for i in range(len(press))
                     if base[i] != press[i]]
            if not diffs:
                st["baseline"] = press
                st["arm_t"] = time.time()
                return
            # 全部变化都回中性值 = 这是"松开"沿,不当按下,刷新基线继续等
            if all(n == 0x00 or n == 0x0F for (_, _, n) in diffs):
                st["baseline"] = press
                st["arm_t"] = time.time()
                return
            key, label = KEYS[st["idx"]]
            loc = self._decode(key, diffs)
            if loc is None:
                st["log_msg"] = "%s 变化不明显,再按一次" % label
            else:
                other = st["loc2key"].get(loc)
                if other and other != key:
                    st["conflict"] = ("注意:%s 与 %s 在手柄上是同一个位(%s)!"
                                      "保存后该位映射成后按的键" % (label, other, loc_str(loc)))
                st["key2loc"][key] = loc
                st["loc2key"][loc] = key
            st["sub"] = "release"
            st["arm_t"] = time.time()
            return

        if sub == "release":
            if now - last_t >= 0.6:
                st["idx"] += 1
                if st["idx"] >= len(KEYS):
                    st["phase"] = "done"
                else:
                    st["sub"] = "quiet"

    @staticmethod
    def _decode(key, diffs):
        """按下引起的变化 -> 位置。方向键优先按 hat 识别,其余按单 bit 位。"""
        if not diffs:
            return None
        # 变化位数最少的一处最可信
        i, old, new = min(diffs, key=lambda d: bin(d[1] ^ d[2]).count("1"))
        if key in DIRS and new <= 0x07:
            return ("hat", i, new)
        x = old ^ new
        if x and (x & (x - 1)) == 0:            # 单 bit 变化
            return ("btn", i, new.bit_length() - 1)
        return ("btn", i, new.bit_length() - 1)

    # ---------- 布局生成 / 保存 ----------
    def build_blob_hex(self):
        st = self.st
        hats = [loc for loc in st["loc2key"] if loc[0] == "hat"]
        bits = [loc for loc in st["loc2key"] if loc[0] in ("btn", "btn?")]
        blob = bytearray(21)
        if hats:
            blob[0] = 1                       # DPAD_HAT
            blob[1] = hats[0][1]
        blob[4] = min((l[1] for l in bits), default=2)   # btn_byte
        for key, loc in st["key2loc"].items():
            if loc[0] in ("btn", "btn?"):
                idx = (loc[1] - blob[4]) * 8 + loc[2]
                if 0 <= idx < 16:
                    blob[5 + idx] = NES_BIT.get(key, 0)
        return blob.hex().upper()

    def save(self):
        self.save_status = None
        self.send("CAL SAVE " + self.build_blob_hex())
        for _ in range(20):
            if self.save_status:
                return self.save_status
            time.sleep(0.15)
        return "timeout"

    def ping(self):
        self.callay = None
        self.send('CAL GET')
        for _ in range(20):
            if self.callay:
                return 'ok'
            time.sleep(0.15)
        return 'timeout'

    def forget(self):
        self.forget_status = None
        self.forget_at = time.time()
        self.send("CAL FORGET")
        for _ in range(20):
            if self.forget_status:
                return self.forget_status
            time.sleep(0.15)
        return "timeout"

    def state_json(self):
        st = self.st
        idx = st["idx"]
        cur = KEYS[idx] if st["phase"] == "waiting" and idx < len(KEYS) else None
        return {
            "ready": self.ready,
            "conn": self.conn,
            "phase": st["phase"],
            "sub": st.get("sub", ""),
            "idx": idx,
            "current": {"name": cur[0], "label": cur[1]} if cur else None,
            "captured": {k: loc_str(v) for k, v in st["key2loc"].items()},
            "conflict": st["conflict"],
            "blob": self.build_blob_hex(),
            "save_status": self.save_status,
            "forget_status": self.forget_status,
            "last_raw": self.evts[-1][1].hex().upper() if self.evts else "",
        }


def loc_str(loc):
    if loc[0] == "hat":
        return "HAT@byte%d(值%d)" % (loc[1], loc[2])
    if loc[0] == "btn":
        return "byte%d bit%d" % (loc[1], loc[2])
    return "未知(%s)" % (loc,)


BR = Bridge()
threading.Thread(target=BR.reader, daemon=True).start()
threading.Thread(target=BR.keepalive, daemon=True).start()

PAGE = """<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8">
<title>NES 手柄校准</title>
<style>
 body{font-family:-apple-system,system-ui,sans-serif;background:#1b1e24;color:#e8e8e8;
      display:flex;flex-direction:column;align-items:center;margin:0;padding:24px;}
 h1{font-size:20px;font-weight:600;}
 #pad{background:#2a2e37;border-radius:20px;padding:18px 28px;box-shadow:0 8px 30px #0008;}
 .lit{fill:#ff5252 !important;stroke:#fff;animation:p 0.7s infinite alternate;}
 @keyframes p{from{opacity:1}to{opacity:0.35}}
 .donek{fill:#4caf50 !important;}
 #msg{min-height:28px;font-size:18px;margin:14px 0;}
 #cur{color:#ff8a80;font-weight:700;}
 table{border-collapse:collapse;margin-top:8px;font-size:14px;}
 td,th{border:1px solid #444;padding:4px 10px;}
 th{color:#9fa8b2;font-weight:500;}
 button{background:#3a7bd5;color:#fff;border:0;border-radius:8px;padding:10px 18px;
        font-size:15px;margin:6px;cursor:pointer;}
 button.gray{background:#555;} button.red{background:#c0392b;}
 button:disabled{opacity:0.4;cursor:default;}
 #conf{color:#ffb74d;margin-top:8px;max-width:560px;text-align:center;}
 #row{display:flex;gap:10px;align-items:center;margin-top:14px;flex-wrap:wrap;justify-content:center;}
 #hint{color:#9fa8b2;font-size:13px;margin-top:14px;text-align:center;line-height:1.7}
</style></head><body>
<h1>NES 手柄校准</h1>
<div id="pad">
<svg width="420" height="200" viewBox="0 0 420 200">
 <ellipse cx="210" cy="100" rx="205" ry="95" fill="#333945"/>
 <!-- 十字键 -->
 <g id="g-dpad">
  <rect id="k-UP"    x="70" y="38"  width="30" height="34" rx="5" fill="#555c68"/>
  <rect id="k-DOWN"  x="70" y="128" width="30" height="34" rx="5" fill="#555c68"/>
  <rect id="k-LEFT"  x="26" y="85"  width="34" height="30" rx="5" fill="#555c68"/>
  <rect id="k-RIGHT" x="110" y="85" width="34" height="30" rx="5" fill="#555c68"/>
  <rect x="70" y="72" width="30" height="56" fill="#3d434e"/>
  <rect x="60" y="85" width="50" height="30" fill="#3d434e"/>
 </g>
 <!-- 选择/开始 -->
 <g id="g-mid">
  <rect id="k-SELECT" x="176" y="96" width="26" height="12" rx="6" fill="#555c68"/>
  <rect id="k-START"  x="218" y="96" width="26" height="12" rx="6" fill="#555c68"/>
  <text x="189" y="126" fill="#9aa3ad" font-size="11" text-anchor="middle">选择</text>
  <text x="231" y="126" fill="#9aa3ad" font-size="11" text-anchor="middle">开始</text>
 </g>
 <!-- A/B -->
 <g id="g-ab">
  <circle id="k-B" cx="322" cy="112" r="21" fill="#555c68"/>
  <circle id="k-A" cx="368" cy="86"  r="21" fill="#555c68"/>
  <text x="322" y="117" fill="#cfd6dd" font-size="14" text-anchor="middle">B</text>
  <text x="368" y="91"  fill="#cfd6dd" font-size="14" text-anchor="middle">A</text>
 </g>
</svg>
</div>
<div id="msg"></div>
<div id="conf"></div>
<div id="row">
 <button id="b-start" onclick="cmd('start')">开始校准</button>
 <button id="b-save"  onclick="cmd('save')" disabled>保存到板子(NVS)</button>
 <button id="b-reset" onclick="cmd('reset')" class="gray">重新来</button>
 <button id="b-forget" onclick="cmd('forget')" class="red">清除板子配对(重配手柄用)</button>
</div>
<table id="tbl"><tr><th>NES 键</th><th>手柄上的位置</th></tr></table>
<div id="hint">
 保存后布局存在板子 NVS 里,重启仍有效,不用再烧录固件。<br>
 手柄连不上时:点"清除板子配对",再让手柄进配对模式(多数手柄是长按 Home/功能键至灯快闪)。
</div>
<script>
const ORDER = ["SELECT","START","A","B","UP","DOWN","LEFT","RIGHT"];
const CN = {SELECT:"选择",START:"开始",A:"A",B:"B",UP:"上",DOWN:"下",LEFT:"左",RIGHT:"右"};
async function poll(){
  const s = await (await fetch("/state")).json();
  document.querySelectorAll(".lit").forEach(e=>e.classList.remove("lit"));
  document.querySelectorAll(".donek").forEach(e=>e.classList.remove("donek"));
  for(const k of ORDER) if(s.captured[k]) document.getElementById("k-"+k).classList.add("donek");
  let msg;
  if(!s.ready) msg = "⌛ 正在与板子建立校准通道…(确认没有别的程序占用串口)";
  else if(s.phase==="idle") msg = "点【开始校准】,按提示逐个按键";
  else if(s.phase==="waiting") {
    if(s.sub==="quiet") msg = "请先松开所有按键…";
    else if(s.sub==="release") msg = "✓ 已识别,请松开 <span id='cur'>"+s.current.label+"</span>,稍等…";
    else msg = "请在手柄上<b>按住约半秒</b>再松开:<span id='cur'>"+s.current.label+"</span>";
  }
  else msg = "✅ 校准完成!核对下表后点【保存到板子】";
  document.getElementById("msg").innerHTML = msg;
  document.getElementById("conf").textContent = s.conflict || "";
  document.getElementById("b-save").disabled = !(s.phase==="done");
  document.getElementById("b-start").disabled = s.phase==="waiting";
  const t = document.getElementById("tbl");
  t.innerHTML = "<tr><th>NES 键</th><th>手柄上的位置</th></tr>" +
    ORDER.filter(k=>s.captured[k]).map(k=>"<tr><td>"+CN[k]+"</td><td>"+s.captured[k]+"</td></tr>").join("") +
    "<tr><td>布局数据</td><td>"+(s.blob||"")+"</td></tr>";
}
async function cmd(do_what){
  const r = await (await fetch("/cmd?do="+do_what)).json();
  if(do_what==="forget"){
    alert(r.result==="ok" ? "板子里的配对已清除。现在:①把手柄关机 ②按住 Home/功能键开机进配对模式(灯快闪) ③等板子自动连上" : "指令超时,看板子日志");
  }
  if(do_what==="save"){
    alert(r.result==="ok" ? "✅ 已写入板子 NVS 并立即生效!去游戏里试试吧" : "保存失败("+r.result+"),重试一次");
  }
  poll();
}
setInterval(poll, 250); poll();
</script>
</body></html>"""


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, ctype, body):
        data = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            self._send(200, "text/html; charset=utf-8", PAGE)
        elif u.path == "/state":
            BR.poll()
            self._send(200, "application/json", json.dumps(BR.state_json(), ensure_ascii=False))
        elif u.path == "/cmd":
            do = parse_qs(u.query).get("do", [""])[0]
            if do == "start":
                BR.start()
                res = "ok"
            elif do == "reset":
                BR.reset()
                res = "ok"
            elif do == "save":
                res = BR.save()
            elif do == "ping":
                res = BR.ping()
            elif do == "forget":
                res = BR.forget()
            else:
                res = "badcmd"
            self._send(200, "application/json", json.dumps({"result": res}, ensure_ascii=False))
        else:
            self._send(404, "text/plain", "404")

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    print("手柄校准程序: http://127.0.0.1:%d  (串口 %s)" % (HTTP_PORT, SERIAL_PORT))
    ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler).serve_forever()
