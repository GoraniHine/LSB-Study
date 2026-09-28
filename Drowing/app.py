"""
CHOCO.DRAW backend
라즈베리파이 <-USB-C-> BTT Octopus (Marlin) 직렬 통신 + G코드 생성/스트리밍 서버

실행:
    pip install -r requirements.txt
    python app.py
    -> http://<라즈베리파이 IP>:5000  (크로미움 키오스크로 이 주소를 띄우면 됨)
"""

import time
import threading
import os
import math
import queue

from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO
import serial
import serial.tools.list_ports

# ============================================================
#  프린터 설정 (필요할 때 여기만 고치면 됨)
# ============================================================

PRINTER = {
    "bed_x": 200,
    "bed_y": 240,
    "bed_z": 130,
    "first_layer_height": 0.8,
    "line_width": 1.2,
    "walls": 1,
    "flow_pct": 160,          # 100 -> 160: 압출량 1.6배로
    "e_calibration": 0.064,     # 레퍼런스 G코드에서 역산한 실측값(~0.0636)에 맞춤
    "extrude_speed": 12,        # mm/s
    "travel_speed": 30,         # mm/s
    "nozzle_temp": 34,
    "bed_temp": 34,
    "z_hop": 2,                 # mm
    "baud": 250000,
    "retract_mm": 600,          # 300 -> 600
    "retract_speed": 250,       # mm/s
    # 인쇄 영역: 여백 없이 배드 전체(기계 X 0~200, 기계 Y 0~240) 그대로 사용.
    # 파킹(G0 X-65)은 이 범위 밖(음수 X)에서 이뤄지므로 X쪽에 별도 여백을 둘 필요 없음.
    # 화면에는 배드를 가로로 눕혀서 보여준다(화면 가로=기계 Y, 화면 세로=기계 X) -
    # 실제 회전/매핑은 to_machine_pt()에서 처리하고 index.html의 printW/printH도
    # 이에 맞춰 x/y를 바꿔서 계산한다.
    "print_x_min": 0.0,
    "print_x_max": 200.0,
    "print_y_min": 0.0,
    "print_y_max": 240.0,
    # 고정 포트를 알고 있으면 여기에 적어두세요 (예: "/dev/serial/by-id/usb-...")
    # 비워두면(""), 연결된 포트 중 첫 번째를 자동으로 찾아 연결합니다.
    "serial_port": "",
}

# ============================================================
#  화면 좌표 -> 기계 좌표 매핑
#
#  [중요/수정] 예전엔 "화면을 가로로 눕혀서 보여준다"는 이유로 화면 가로<->세로를
#  기계 X<->Y에 맞바꿔서(축 스왑) 매핑했었다. 그런데 이 방식은 구조적으로 문제가
#  있었다: 좌우 대칭인 도형(스페이드 등)의 중심점은 항상 "화면 가로 중앙"에 있는데,
#  축 스왑 매핑에서는 이 점이 반드시 "기계 Y(앞뒤) 중앙"으로만 가게 되어 있어서,
#  아무리 회전/대칭 조합을 바꿔도 도형이 절대 앞/뒤 방향을 가질 수 없고 좌/우로만
#  나오는 근본적인 한계가 있었다(실측으로 확인됨).
#
#  그래서 축 스왑을 완전히 없애고, 화면을 그대로(세로형) 기계 좌표에 매핑한다:
#    화면 가로(screen x) -> 기계 X (좌우)
#    화면 세로(screen y) -> 기계 Y (앞뒤)
#  (index.html의 printW/printH도 이에 맞춰 세로형으로 같이 바꿨다.)
#
#  이제 남은 문제는 "좌우가 뒤집혔는지", "앞뒤가 뒤집혔는지" 둘 뿐이라, 아래 두
#  값만 True/False로 바꿔가며 테스트하면 된다.
#    FLIP_X: 그림에서 왼쪽이었던 게 실제로는 오른쪽에 찍히면 True
#    FLIP_Y: 그림에서 위쪽(화면 상단)이었던 게 실제로는 반대 방향(정면 쪽)에 찍히면 True
# ============================================================
FLIP_X = False
FLIP_Y = True

# ---- 시작/끝 G코드: 여기에 직접 내장하세요 -------------------
START_GCODE = """
; ---- START GCODE ----
G21 ; mm 단위
G90 ; 절대좌표
M82 ; 압출 절대좌표 (레퍼런스 G코드와 동일한 방식)
M104 S{nozzle_temp} ; 노즐 예열 시작
M140 S{bed_temp} ; 배드 예열 시작
M109 S{nozzle_temp} ; 노즐이 실제로 목표 온도에 도달할 때까지 대기
M190 S{bed_temp} ; 배드가 실제로 목표 온도에 도달할 때까지 대기
G91
G0 Z5 F500
G90
G28
G28 X
T0
G0 X-65 F2000
G92 E0
G1 E60 F200
""".strip()

END_GCODE = """
; ---- END GCODE ----
T0
G91
G0 Z10 F1000
G90
G28 XY ; XY 홈으로 복귀 (진짜 원위치로 감), Z가 이미 10mm 들려있어서 그림을 스치지 않음
G1 E{unretract_e:.4f} F{f_retract} ; 마지막 리트랙션 복원 (다음 프린트를 위한 드리프트 방지)
M84
""".strip()
# ---------------------------------------------------------------


app = Flask(__name__)
app.config["SECRET_KEY"] = "choco-draw"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ============================================================
#  시리얼 연결 관리
# ============================================================

class PrinterLink:
    def __init__(self):
        self.ser = None
        self.connected = False
        self.reader_thread = None
        self.poll_thread = None
        self.stop_flag = False
        self.connect_lock = threading.Lock()  # connect()가 동시에 두 번 실행되는 것 방지
        self.connecting = False              # connect() 진행 중인지 (auto_connect_loop 경쟁 방지용)
        self.write_lock = threading.Lock()   # 쓰기(write)만 보호 - 읽기는 아래 큐로 단일화
        self.rx_queue = queue.Queue()        # _read_loop(단 하나의 스레드)만 여기 채움
        self.last_temp = {"nozzle": 0, "bed": 0, "nozzle_target": 0, "bed_target": 0}
        self.printing = False
        self.cancel_flag = False

    def list_ports(self):
        ports = serial.tools.list_ports.comports()
        return [{"device": p.device, "desc": p.description} for p in ports]

    def connect(self, port, baud=None):
        baud = baud or PRINTER["baud"]
        with self.connect_lock:
            self.connecting = True
            try:
                if self.connected:
                    print(f"[CONNECT] 이미 연결돼 있어서 기존 연결부터 정리하고 재연결: {port}")
                    self._disconnect_locked()
                    time.sleep(0.3)
                try:
                    self.ser = serial.Serial(port, baud, timeout=2)
                    time.sleep(2)  # Marlin 리셋 대기
                    self.ser.reset_input_buffer()
                    with self.rx_queue.mutex:
                        self.rx_queue.queue.clear()
                    self.stop_flag = False
                    self.connected = True
                    self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
                    self.reader_thread.start()
                    self.poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
                    self.poll_thread.start()
                    print(f"[CONNECT] 성공: {port}")
                    return True, "connected"
                except Exception as e:
                    print(f"[CONNECT] 실패: {port} -> {e}")
                    return False, str(e)
            finally:
                self.connecting = False

    def disconnect(self):
        with self.connect_lock:
            self._disconnect_locked()

    def _disconnect_locked(self):
        self.stop_flag = True
        self.connected = False
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=3)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _read_loop(self):
        """시리얼 포트에서 실제로 readline()을 호출하는 곳은 여기 단 한 군데뿐.

        [수정] M105 온도 폴링 응답("ok T:.. /.. B:.. /..")은 다른 명령(특히
        M109/M190/G코드 각 줄)의 'ok' ack와 절대 섞이면 안 되므로, 온도 응답인
        줄은 rx_queue(ack용)에 넣지 않고 temp_update/serial_line emit에만 사용한다.
        예전 코드는 이걸 구분하지 않아서, 프린트 시작 전 쌓여있던 M105 응답을
        M109/M190의 'ok'로 잘못 소비해버려 예열이 끝나기도 전에 다음 동작으로
        진행해버리는 문제가 있었다.
        """
        while not self.stop_flag and self.ser:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if line:
                    print(f"[RECV] {line}")
                    is_temp_report = "T:" in line and "B:" in line
                    self._parse_line(line)
                    if not is_temp_report:
                        self.rx_queue.put(line)
            except Exception:
                break
        if not self.stop_flag:
            self.connected = False
            socketio.emit("connect_result", {"ok": False, "msg": "연결 끊김 - 재연결 시도 중"})

    def _parse_line(self, line):
        # 예: "ok T:34.1 /34.0 B:34.0 /34.0"
        if "T:" in line and "B:" in line:
            try:
                t = float(line.split("T:")[1].split()[0].split("/")[0])
                tt = float(line.split("T:")[1].split()[0].split("/")[1])
                b = float(line.split("B:")[1].split()[0].split("/")[0])
                bt = float(line.split("B:")[1].split()[0].split("/")[1])
                self.last_temp = {"nozzle": t, "nozzle_target": tt, "bed": b, "bed_target": bt}
                socketio.emit("temp_update", self.last_temp)
            except Exception:
                pass
        socketio.emit("serial_line", {"line": line})

    def _poll_loop(self):
        """온도 폴링 전용 스레드. connect()마다 딱 하나만 생성됨(중복 방지)."""
        while self.connected and not self.stop_flag:
            if not self.printing:  # 프린트 중엔 M105가 끼어들어 응답 순서가 꼬이는 걸 방지
                self.send_line("M105", wait_ok=False)
            time.sleep(2)

    def send_line(self, line, wait_ok=True, timeout=15):
        if not self.ser or not self.connected:
            print(f"[SEND 실패] 연결 안 됨: {line}")
            return False
        with self.write_lock:
            print(f"[SEND] {line}")
            self.ser.write((line + "\n").encode())
        if not wait_ok:
            return True

        # 응답은 _read_loop(단일 리더 스레드)가 큐에 채워줌 - 여기선 큐에서 꺼내기만 함
        # (온도 폴링 응답은 _read_loop에서 이미 걸러져서 이 큐에 안 들어오므로,
        #  여기서 꺼내는 건 항상 우리가 보낸 명령들에 대한 실제 ack다)
        start = time.time()
        while time.time() - start < timeout:
            remaining = timeout - (time.time() - start)
            try:
                resp = self.rx_queue.get(timeout=max(0.1, remaining))
            except queue.Empty:
                break
            if resp.startswith("ok"):
                return True
            if resp.startswith("Error") or resp.startswith("!!"):
                return False
        print(f"[TIMEOUT] {timeout}초 동안 'ok' 응답 없음: {line}")
        return False

    def stream_gcode(self, gcode_text):
        self.printing = True
        self.cancel_flag = False
        lines = [l for l in gcode_text.splitlines() if l.strip() and not l.strip().startswith(";")]
        total = len(lines)
        print(f"[PRINT START] 총 {total}줄")
        for i, line in enumerate(lines):
            if self.cancel_flag:
                print("[PRINT CANCELLED]")
                socketio.emit("print_status", {"state": "cancelled"})
                break
            # M109(노즐 대기)/M190(배드 대기)는 온도 도달까지 오래 걸릴 수 있어 타임아웃을 길게 줌
            cmd = line.strip().split(";")[0].strip().upper()
            if cmd.startswith("M109") or cmd.startswith("M190"):
                line_timeout = 3600  # 최대 1시간까지 대기
                socketio.emit("print_status", {"state": "heating", "line": line})
            else:
                line_timeout = 30  # 홈 이동/긴 프라이밍 압출처럼 15초 넘게 걸리는 정상 동작 감안
            ok = self.send_line(line, wait_ok=True, timeout=line_timeout)
            pct = round((i + 1) / total * 100, 1)
            socketio.emit("print_progress", {"percent": pct, "line": line, "ok": ok})
            if not ok:
                print(f"[PRINT WARNING] {i+1}번째 줄 실패했지만 계속 진행: {line}")
        self.printing = False
        if not self.cancel_flag:
            print("[PRINT DONE]")
            socketio.emit("print_status", {"state": "done"})

    def cancel_print(self):
        self.cancel_flag = True


link = PrinterLink()


def auto_connect_loop():
    """USB가 꽂혀 있으면 서버 시작과 동시에, 그리고 연결이 끊길 때마다 자동으로 재연결을 시도."""
    while True:
        if not link.connected and not link.connecting:
            target = None
            preferred = PRINTER.get("serial_port") or ""
            if preferred and os.path.exists(preferred):
                target = preferred
            if not target:
                ports = link.list_ports()
                # /dev/ttyS0 같은 라즈베리파이 내장 UART는 프린터가 아니므로 후보에서 제외
                ports = [p for p in ports if "ttyS" not in p["device"] and "ttyAMA" not in p["device"]]
                if ports:
                    target = ports[0]["device"]
            if target:
                ok, msg = link.connect(target)
                socketio.emit("connect_result", {
                    "ok": ok,
                    "msg": f"자동 연결됨 ({target})" if ok else f"자동 연결 실패: {msg}",
                })
        time.sleep(3)


threading.Thread(target=auto_connect_loop, daemon=True).start()

# ============================================================
#  G코드 생성 (프론트에서 그린 stroke 좌표 -> Marlin G코드)
# ============================================================

def dist(a, b):
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


def catmull_rom_point(p0, p1, p2, p3, t):
    """Catmull-Rom 스플라인 보간. p1->p2 사이를 t(0~1)로 부드럽게 지나가는 점을 계산."""
    t2 = t * t
    t3 = t2 * t
    x = 0.5 * (
        (2 * p1["x"])
        + (-p0["x"] + p2["x"]) * t
        + (2 * p0["x"] - 5 * p1["x"] + 4 * p2["x"] - p3["x"]) * t2
        + (-p0["x"] + 3 * p1["x"] - 3 * p2["x"] + p3["x"]) * t3
    )
    y = 0.5 * (
        (2 * p1["y"])
        + (-p0["y"] + p2["y"]) * t
        + (2 * p0["y"] - 5 * p1["y"] + 4 * p2["y"] - p3["y"]) * t2
        + (-p0["y"] + 3 * p1["y"] - 3 * p2["y"] + p3["y"]) * t3
    )
    return {"x": x, "y": y}


def smooth_polyline(pts, samples_per_segment=6):
    """직선으로 이어진 폴리라인 점들을 Catmull-Rom 스플라인으로 재보간해서
    부드러운 곡선 점열로 바꿔준다. 원래 점들을 그대로 지나가면서 사이사이를
    곡선으로 채워주기 때문에, 폰트 윤곽선(하트/스페이드/다이아/클로버)처럼
    원래 각진 폴리곤으로 근사된 도형을 매끄럽게 만드는 데 적합하다.

    - 점이 3개 미만이면(직선 툴로 그은 2점짜리 획 등) 스무딩 없이 그대로 반환.
    - 첫점과 끝점이 거의 같으면(템플릿 채우기로 찍은 닫힌 도형) 이어붙임 구간까지
      포함해서 부드럽게 처리.
    """
    n = len(pts)
    if n < 3:
        return pts

    closed = dist(pts[0], pts[-1]) < 0.01
    work = pts[:-1] if closed else pts
    m = len(work)
    if m < 3:
        return pts

    result = []
    count = m if closed else m - 1
    for i in range(count):
        if closed:
            p0 = work[(i - 1) % m]
            p1 = work[i % m]
            p2 = work[(i + 1) % m]
            p3 = work[(i + 2) % m]
        else:
            p0 = work[max(i - 1, 0)]
            p1 = work[i]
            p2 = work[min(i + 1, m - 1)]
            p3 = work[min(i + 2, m - 1)]
        for s in range(samples_per_segment):
            t = s / samples_per_segment
            result.append(catmull_rom_point(p0, p1, p2, p3, t))

    result.append(result[0] if closed else work[-1])
    return result


def to_machine_pt(pt, x_min, x_max, y_min, y_max):
    """프론트엔드가 보내는 좌표는 '화면 기준(mm), 0부터 시작'이고, 이제 화면은
    회전 없이 그대로(세로형) 기계 좌표에 대응한다: 화면 x -> 기계 X, 화면 y -> 기계 Y.
    FLIP_X/FLIP_Y로 필요할 때만 좌우/앞뒤를 뒤집는다.
    """
    sx, sy = pt["x"], pt["y"]
    if FLIP_X:
        sx = (x_max - x_min) - sx
    if FLIP_Y:
        sy = (y_max - y_min) - sy
    return {"x": sx + x_min, "y": sy + y_min}


def generate_gcode(strokes, overrides=None):
    p = dict(PRINTER)
    if overrides:
        p.update(overrides)

    first_layer = p["first_layer_height"]
    flow = p["flow_pct"]
    walls = max(1, int(p["walls"]))
    e_cal = p["e_calibration"]
    ex_speed = p["extrude_speed"]
    travel_speed = p["travel_speed"]
    z_hop = p["z_hop"]
    retract_mm = p.get("retract_mm", 1.0)
    f_retract = int(p.get("retract_speed", 20) * 60)

    f_extrude = int(ex_speed * 60)
    f_travel = int(travel_speed * 60)

    lines = []
    lines.append("; ==== CHOCO.DRAW generated G-code ====")
    lines.append(START_GCODE.format(nozzle_temp=p["nozzle_temp"], bed_temp=p["bed_temp"]))
    lines.append("")
    lines.append("G92 E0 ; 그리기 시작 전 압출 좌표 기준점 리셋 (절대좌표로 누적 계산)")
    lines.append(f"G1 Z{first_layer + z_hop:.3f} F{f_travel}")

    e_pos = 0.0  # 절대(absolute) 압출 좌표 누적값 - 레퍼런스 G코드와 동일한 방식
    have_extruded = False  # 맨 처음 이동 전에는 리트랙션 불필요

    # 모든 획을 먼저 기계 좌표로 변환 -> 곡선으로 스무딩 -> 왼쪽(작은 X)부터 그리도록 정렬/방향 조정
    converted = []
    for s in strokes:
        pts = [
            to_machine_pt(pt, p["print_x_min"], p["print_x_max"], p["print_y_min"], p["print_y_max"])
            for pt in s["points"]
        ]
        if len(pts) < 2:
            continue
        pts = smooth_polyline(pts, samples_per_segment=6)  # 각진 폴리라인 -> 부드러운 곡선으로 재보간
        if pts[0]["x"] > pts[-1]["x"]:  # 획 안에서도 왼쪽 끝에서 시작하도록 방향 뒤집기
            pts = list(reversed(pts))
        converted.append(pts)
    converted.sort(key=lambda pts: min(pt["x"] for pt in pts))  # 획들끼리도 왼쪽에 있는 것부터

    for idx, pts in enumerate(converted):
        lines.append(f"; stroke {idx+1} ({len(pts)} pts)")
        for _ in range(walls):
            # 이미 직전 획 끝에서 리트랙션 상태로 들어와 있으니, 이동 전에 또 뺄 필요 없음
            lines.append(f"G0 X{pts[0]['x']:.3f} Y{pts[0]['y']:.3f} Z{first_layer+z_hop:.3f} F{f_travel}")
            lines.append(f"G1 Z{first_layer:.3f} F{f_travel}")
            if have_extruded:
                e_pos += retract_mm
                lines.append(f"G1 E{e_pos:.4f} F{f_retract} ; un-retract (그리기 시작 전)")
            for i in range(1, len(pts)):
                d = dist(pts[i - 1], pts[i])
                e_pos += d * e_cal * (flow / 100)
                lines.append(f"G1 X{pts[i]['x']:.3f} Y{pts[i]['y']:.3f} E{e_pos:.4f} F{f_extrude}")
            have_extruded = True
            e_pos -= retract_mm
            lines.append(f"G1 E{e_pos:.4f} F{f_retract} ; retract (다 그리고 들어올리기 전)")
            lines.append(f"G1 Z{first_layer+z_hop:.3f} F{f_travel}")

    lines.append("")
    # [수정] 마지막 획이 끝나면 retract_mm 만큼 리트랙션된 채로 프린트가 끝나는데,
    # 이걸 되돌려줄 다음 획이 없어서 프린트 1번이 끝날 때마다 압출기(플런저)가
    # "un-retract 없이 retract만" 순손실로 누적된다. G92 E0는 논리 좌표만 리셋할 뿐
    # 실제 플런저 물리 위치는 그대로라서, 이 손실이 프린트를 거듭할수록 계속 쌓여
    # 다음 프린트에서 시작 프라이밍(G1 E60)만으로는 부족해져 압출이 잘 안 나오게 된다.
    # -> END_GCODE 자체에 마지막 retract를 다시 un-retract 하는 라인을 넣어서,
    #    프린트가 "E 중립" 상태로 끝나도록 만들어 프린트 간 드리프트를 없앤다.
    unretract_e = e_pos + retract_mm if have_extruded else e_pos
    lines.append(END_GCODE.format(
        park_z=min(p["bed_z"], first_layer + 20),
        unretract_e=unretract_e,
        f_retract=f_retract,
    ))
    lines.append(f"; final absolute E: {unretract_e:.3f} mm")
    return "\n".join(lines)


# ============================================================
#  라우트 / 소켓 이벤트
# ============================================================

@app.route("/")
def index():
    return render_template("index.html", printer=PRINTER)


@app.route("/api/ports")
def api_ports():
    return jsonify(link.list_ports())


@socketio.on("connect")
def on_client_connect():
    # 새 브라우저 탭/창이 열릴 때, 이미 자동 연결돼 있으면 현재 상태를 바로 알려줌
    socketio.emit("connect_result", {
        "ok": link.connected,
        "msg": "연결됨" if link.connected else "연결 대기 중",
    })
    if link.connected:
        socketio.emit("temp_update", link.last_temp)


@socketio.on("connect_printer")
def on_connect_printer(data):
    ok, msg = link.connect(data.get("port"), data.get("baud"))
    socketio.emit("connect_result", {"ok": ok, "msg": msg})


@socketio.on("disconnect_printer")
def on_disconnect_printer():
    link.disconnect()
    socketio.emit("connect_result", {"ok": False, "msg": "disconnected"})


@socketio.on("start_print")
def on_start_print(data):
    strokes = data.get("strokes", [])
    overrides = data.get("overrides", {})
    gcode = generate_gcode(strokes, overrides)
    socketio.emit("gcode_preview", {"gcode": gcode})
    threading.Thread(target=link.stream_gcode, args=(gcode,), daemon=True).start()


@socketio.on("cancel_print")
def on_cancel_print():
    link.cancel_print()


@socketio.on("preview_gcode")
def on_preview_gcode(data):
    strokes = data.get("strokes", [])
    overrides = data.get("overrides", {})
    gcode = generate_gcode(strokes, overrides)
    socketio.emit("gcode_preview", {"gcode": gcode})


if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000, debug=False)