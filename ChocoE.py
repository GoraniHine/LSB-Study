"""
라즈베리파이 + 3D프린터 초콜릿 붓기 제어 스크립트 (키보드 조작 버전)

키 조작 (영상 창을 클릭해서 선택한 상태에서, 영문 입력 상태로)
  G : 압출 시작 (시작 위치로 이동 후 원 그리며 압출)
  X : 압출 중이면 정지 후 홈으로 / 대기 중이면 홈으로
      홈에 있을 때 한 번 더 누르면 시작 위치(Z 10cm)로 복귀 → 다시 G 가능
  Q : 프로그램 종료 (히터 끄기)

자동 정지: 초콜릿 비율이 50% 초과 상태로 연속 N프레임이면 압출 정지
압출 종료 시: 리트랙션(되감기) → 다음 G 시작 때 되감은 만큼 다시 밀어넣음
온도: 대기 중 33도, 압출 중 37도 (압출 끝나면 다시 33도)
냉각 팬: 대기/실행 상관없이 노즐 온도 40도 이상이면 GPIO23 팬 ON
"""

import math
import re
import threading
import time
import os

import cv2
import serial
from ultralytics import YOLO

try:
    from gpiozero import OutputDevice
except ImportError:
    OutputDevice = None

# ===================== 설정 =====================
SERIAL_PORT = '/dev/ttyACM0'      # 안 잡히면 ls /dev/ttyACM* /dev/ttyUSB* 로 확인
BAUD_RATE = 115200                # 프린터에 따라 250000 일 수 있음

# 이 스크립트와 같은 폴더에 있는 best.pt
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'best.pt')
CAMERA_INDEX = 0
IMG_SIZE = 640                    # 라즈베리파이가 느리면 320 등으로 낮추기

CENTER_X = 110.0                  # mm
CENTER_Y = 110.0                  # mm
RADIUS = 10.0                     # mm (중앙에서 1cm)
Z_HEIGHT = 100.0                  # mm (10cm)
IDLE_TEMP = 33                    # 대기 중 노즐 온도 (도)
EXTRUDE_TEMP = 37                 # 압출 중 노즐 온도 (도)
HEAT_TIMEOUT = 600                # 가열 최대 대기 시간(초)

FEEDRATE = 275                    # mm/min (원 그리는 속도)
TRAVEL_FEEDRATE = 3000            # mm/min (이동 속도)
SEGMENTS = 36                     # 원을 몇 개의 직선으로 나눌지
EXTRUDE_PER_MM = 4.75             # 이동 1mm당 압출량(mm). 한 바퀴(62.8mm)에 약 600
PRIME_LENGTH = 60.0               # 첫 압출 전에 한 번 짜내는 양(mm), 슬라이서 E60과 동일. 0이면 안 함
PRIME_SPEED = 200                 # 짜내는 속도(mm/min), 슬라이서 F200과 동일
RETRACT_LENGTH = 5.0              # 압출 종료 시 리트랙션 길이(mm). 0이면 안 함
RETRACT_SPEED = 1200              # 리트랙션 속도(mm/min)
LIFT_AFTER_STOP = 10.0            # 정지 후 노즐을 올리는 높이(mm)

STOP_PERCENT = 50.0               # 이 비율을 넘으면 정지
CONFIRM_FRAMES = 5                # 연속 몇 프레임 넘어야 정지할지
MAX_RUN_SECONDS = 600             # 안전장치: 한 번 압출의 최대 시간(초)

TURN_OFF_HEATER_AT_EXIT = True    # 프로그램 종료 시 히터 끄기

FAN_GPIO = 23                     # BCM 번호 GPIO23 (물리 핀 16번)
FAN_ON_TEMP = 40.0                # 이 온도 이상이면 팬 ON
FAN_OFF_TEMP = 39.5               # 이 온도 미만으로 내려가면 팬 OFF (깜빡임 방지)
FAN_ACTIVE_HIGH = True            # 모듈이 LOW에서 켜지는 타입이면 False
TEMP_POLL_SEC = 2.0               # 온도 확인 주기(초)
# ===============================================


class Printer:
    """시리얼로 G코드를 한 줄씩 보내고 'ok'를 기다리는 클래스.
    응답에 온도(T:xx.x)가 있으면 last_temp를 갱신하고 on_temp 콜백을 부름"""

    TEMP_RE = re.compile(r'T:\s*(-?\d+(?:\.\d+)?)')

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        self.ser.reset_input_buffer()
        self.lock = threading.Lock()
        self.last_temp = None
        self.on_temp = None

    def _parse_temp(self, line):
        m = self.TEMP_RE.search(line)
        if m:
            self.last_temp = float(m.group(1))
            if self.on_temp:
                self.on_temp(self.last_temp)

    def _send_locked(self, cmd, timeout):
        self.ser.write((cmd + '\n').encode())
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                continue
            self._parse_temp(line)            # M109 대기 중 출력되는 온도도 반영
            if line.startswith('ok'):
                return
            if 'error' in line.lower():
                print(f"[프린터 에러] {cmd} -> {line}")
            elif line.startswith('echo:busy'):
                deadline = time.time() + timeout
        raise TimeoutError(f"프린터 응답 없음: {cmd}")

    def send(self, cmd, timeout=30):
        with self.lock:
            self._send_locked(cmd, timeout)

    def poll_temp(self):
        """다른 명령이 오래 잡고 있으면(M109 등) 건너뜀. 그때는 그 명령의 출력에서 온도를 읽음"""
        if self.lock.acquire(timeout=1.0):
            try:
                self._send_locked('M105', 5)
            finally:
                self.lock.release()

    def close(self):
        self.ser.close()


class FanController:
    """라즈베리파이 GPIO로 냉각 팬 ON/OFF (히스테리시스 적용)"""

    def __init__(self, pin):
        self.on = False
        self.dev = None
        if OutputDevice is None:
            print("gpiozero가 없어 팬 제어를 사용할 수 없습니다. (sudo apt install python3-gpiozero)")
            return
        try:
            self.dev = OutputDevice(pin, active_high=FAN_ACTIVE_HIGH, initial_value=False)
            print(f"팬 제어 준비 (GPIO{pin})")
        except Exception as ex:
            print(f"GPIO{pin} 초기화 실패, 팬 제어 없이 진행: {ex}")

    def update(self, temp):
        if temp >= FAN_ON_TEMP and not self.on:
            self._set(True)
            print(f"온도 {temp:.1f}도 - 팬 ON")
        elif temp < FAN_OFF_TEMP and self.on:
            self._set(False)
            print(f"온도 {temp:.1f}도 - 팬 OFF")

    def _set(self, value):
        self.on = value
        if self.dev is not None:
            self.dev.on() if value else self.dev.off()

    def close(self):
        if self.dev is not None:
            self.dev.off()
            self.dev.close()


class TempMonitor:
    """TEMP_POLL_SEC마다 M105로 온도를 읽는 스레드 (대기/실행 상관없이 동작)"""

    def __init__(self, printer):
        self.printer = printer
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            try:
                self.printer.poll_temp()
            except Exception as ex:
                print(f"[온도 읽기 오류] {ex}")
            time.sleep(TEMP_POLL_SEC)

    def stop(self):
        self.running = False
        self.thread.join(timeout=TEMP_POLL_SEC + 7)


class Controller:
    """
    프린터 동작을 백그라운드 스레드(job)로 실행해서 영상이 멈추지 않게 함.
    mode : 'SETUP' / 'IDLE' / 'RUNNING' / 'BUSY'
    at   : 'READY'(시작 위치) / 'HOME' / 'LIFTED'(압출 후 올라간 위치) / 'UNKNOWN'
    """

    def __init__(self, printer):
        self.printer = printer
        self.mode = 'IDLE'
        self.at = 'UNKNOWN'
        self.job = None
        self.stop_event = threading.Event()
        self.home_after_stop = False
        self.run_start = None
        self.laps = 0
        self.retracted = False
        self.primed = False
        self.target_temp = None

    # ---------- 내부 동작 ----------
    def _start_job(self, fn, mode):
        self.mode = mode

        def wrapper():
            try:
                fn()
            except Exception as ex:
                print(f"[동작 오류] {ex}")
                self.at = 'UNKNOWN'
            finally:
                self.mode = 'IDLE'
                print(f"대기 중 ({self.status_text()})")

        self.job = threading.Thread(target=wrapper, daemon=True)
        self.job.start()

    def is_busy(self):
        return self.job is not None and self.job.is_alive()

    def _go_ready(self):
        p = self.printer
        print("시작 위치로 이동 중...")
        p.send('G90')
        p.send('M83')                        # Marlin은 G90이 압출도 절대값으로 바꾸므로 다시 상대값으로
        p.send(f'G1 Z{Z_HEIGHT:.1f} F600', timeout=120)
        p.send(f'G1 X{CENTER_X + RADIUS:.3f} Y{CENTER_Y:.3f} F{TRAVEL_FEEDRATE}')
        p.send('M400', timeout=120)
        self.at = 'READY'

    def _home(self):
        print("홈으로 이동 중...")
        self.printer.send('G28', timeout=180)
        self.printer.send('M400', timeout=120)
        self.at = 'HOME'

    def _setup(self):
        p = self.printer
        print("프린터 초기 설정 중...")
        p.send('T0')                         # 압출기 선택 (슬라이서 G코드와 동일)
        p.send('G21')
        p.send('G90')
        p.send('M83')                        # 압출 상대값
        p.send('M302 S0')                    # 저온 압출 허용
        self._home()
        self._go_ready()
        self._heat_to(IDLE_TEMP)
        print("준비 완료 - G: 압출 시작 / X: 홈")

    def _heat_to(self, temp, cancel_event=None):
        """M104로 목표 설정 후 도달할 때까지 대기. 이미 더 뜨거우면 기다리지 않음.
        M109와 달리 대기 중에도 X로 취소 가능"""
        self.target_temp = temp
        self.printer.send(f'M104 S{temp}')
        print(f"{temp}도 가열 대기 중...")
        t0 = time.time()
        while True:
            cur = self.printer.last_temp
            if cur is not None and cur >= temp - 0.5:
                print(f"{temp}도 도달 ({cur:.1f}도)")
                return True
            if cancel_event is not None and cancel_event.is_set():
                return False
            if time.time() - t0 > HEAT_TIMEOUT:
                print("가열 시간 초과. 그대로 진행합니다.")
                return True
            time.sleep(0.5)

    def _stop_sequence(self):
        p = self.printer
        p.send('M410')                       # 버퍼에 남은 이동 즉시 취소
        p.send('M400', timeout=120)
        if EXTRUDE_PER_MM > 0 and RETRACT_LENGTH > 0 and not self.retracted:
            print(f"리트랙션 {RETRACT_LENGTH}mm")
            p.send(f'G1 E-{RETRACT_LENGTH:.2f} F{RETRACT_SPEED}')
            p.send('M400', timeout=60)
            self.retracted = True
        if LIFT_AFTER_STOP > 0:
            p.send('G91')
            p.send(f'G1 Z{LIFT_AFTER_STOP:.1f} F600')
            p.send('G90')
            p.send('M83')
        p.send('M400', timeout=120)
        self.target_temp = IDLE_TEMP
        p.send(f'M104 S{IDLE_TEMP}')           # 압출 끝 → 대기 온도로 내림
        print(f"대기 온도 {IDLE_TEMP}도로 설정")
        self.at = 'LIFTED'

    def _extrude(self):
        if self.at != 'READY':
            self._go_ready()
        if self.stop_event.is_set():         # 이동 중 정지 요청이 온 경우
            if self.home_after_stop:
                self._home()
            return

        if not self._heat_to(EXTRUDE_TEMP, self.stop_event):   # 압출 온도 37도로
            self.target_temp = IDLE_TEMP
            self.printer.send(f'M104 S{IDLE_TEMP}')
            if self.home_after_stop:
                self._home()
            return

        self.printer.send('M83')             # 압출 상대값 확실히
        if self.retracted:                   # 지난번에 되감은 만큼 다시 밀어넣기
            print(f"언리트랙션 {RETRACT_LENGTH}mm")
            self.printer.send(f'G1 E{RETRACT_LENGTH:.2f} F{RETRACT_SPEED}')
            self.retracted = False

        if EXTRUDE_PER_MM > 0 and PRIME_LENGTH > 0 and not self.primed:
            print(f"초기 짜내기 {PRIME_LENGTH}mm (약 {PRIME_LENGTH / PRIME_SPEED * 60:.0f}초)")
            self.printer.send(f'G1 E{PRIME_LENGTH:.2f} F{PRIME_SPEED}')
            self.printer.send('M400', timeout=PRIME_LENGTH / PRIME_SPEED * 60 + 60)
            self.primed = True

        print(f"압출 시작 (1mm당 E{EXTRUDE_PER_MM})")
        seg_len = 2 * math.pi * RADIUS / SEGMENTS
        e = seg_len * EXTRUDE_PER_MM
        self.laps = 0
        self.run_start = time.time()
        self.at = 'UNKNOWN'
        try:
            while not self.stop_event.is_set():
                for k in range(1, SEGMENTS + 1):
                    if self.stop_event.is_set():
                        break
                    a = 2 * math.pi * k / SEGMENTS
                    x = CENTER_X + RADIUS * math.cos(a)
                    y = CENTER_Y + RADIUS * math.sin(a)
                    cmd = f'G1 X{x:.3f} Y{y:.3f}'
                    if e > 0:
                        cmd += f' E{e:.4f}'
                    cmd += f' F{FEEDRATE}'
                    self.printer.send(cmd)
                else:
                    self.laps += 1
        finally:
            self.run_start = None
            print(f"압출 정지 (총 {self.laps}바퀴)")
            self._stop_sequence()
            if self.home_after_stop:
                self._home()

    # ---------- 외부에서 부르는 동작 ----------
    def setup(self):
        self._start_job(self._setup, 'SETUP')

    def press_g(self):
        if self.is_busy():
            print("다른 동작 중이라 G를 무시합니다.")
            return False
        self.stop_event.clear()
        self.home_after_stop = False
        self._start_job(self._extrude, 'RUNNING')
        return True

    def press_x(self):
        if self.mode == 'RUNNING':
            print("압출 정지 후 홈으로 이동합니다.")
            self.home_after_stop = True
            self.stop_event.set()
        elif self.is_busy():
            print("다른 동작 중이라 X를 무시합니다.")
        elif self.at == 'HOME':
            self._start_job(self._go_ready, 'BUSY')
        else:
            self._start_job(self._home, 'BUSY')

    def auto_stop(self, reason):
        if self.mode == 'RUNNING' and not self.stop_event.is_set():
            print(reason + " 압출을 정지합니다.")
            self.home_after_stop = False
            self.stop_event.set()

    def shutdown(self):
        if self.mode == 'RUNNING':
            self.home_after_stop = False
            self.stop_event.set()
        if self.job is not None:
            self.job.join(timeout=120)
        if TURN_OFF_HEATER_AT_EXIT:
            try:
                self.printer.send('M104 S0')
            except Exception as ex:
                print(f"히터 끄기 실패: {ex}")

    def status_text(self):
        if self.mode == 'SETUP':
            return f"SETUP (homing / heating {IDLE_TEMP}C)..."
        if self.mode == 'RUNNING':
            if self.stop_event.is_set():
                return "STOPPING..."
            return f"RUNNING {self.target_temp}C  lap {self.laps}  [X: stop+home]"
        if self.mode == 'BUSY':
            return "MOVING..."
        if self.at == 'HOME':
            return "HOME  [X: to start pos]"
        return "READY  [G: start / X: home]"


def measure_chocolate_level(results, class_names):
    """반환: (percentage 또는 None, 비커 감지 여부)"""
    if results.boxes is None or len(results.boxes) == 0:
        return None, False

    boxes = results.boxes.xyxy.cpu().numpy()
    classes = results.boxes.cls.cpu().numpy().astype(int)
    confs = results.boxes.conf.cpu().numpy()

    beaker_idx = None
    for i, c in enumerate(classes):
        if class_names.get(c, str(c)) == 'beaker':
            if beaker_idx is None or confs[i] > confs[beaker_idx]:
                beaker_idx = i
    if beaker_idx is None:
        return None, False

    _, beaker_top, _, beaker_bottom = boxes[beaker_idx]
    beaker_height = beaker_bottom - beaker_top
    if beaker_height <= 0:
        return None, True

    choco_ys = []
    if results.masks is not None:
        masks = results.masks.xy
        for i, c in enumerate(classes):
            if class_names.get(c, str(c)) == 'chocolate' and i < len(masks):
                polygon = masks[i]
                if len(polygon) > 0:
                    choco_ys.extend(polygon[:, 1].tolist())

    if not choco_ys:
        return 0.0, True

    filled = beaker_bottom - min(choco_ys)
    filled = max(0.0, min(filled, beaker_height))
    return float(filled / beaker_height * 100), True


def draw(frame, results, class_names, percentage, count, status, temp, fan_on):
    out = frame.copy()
    if results.boxes is not None:
        boxes = results.boxes.xyxy.cpu().numpy()
        classes = results.boxes.cls.cpu().numpy().astype(int)
        confs = results.boxes.conf.cpu().numpy()
        for i, c in enumerate(classes):
            name = class_names.get(c, str(c))
            x1, y1, x2, y2 = map(int, boxes[i][:4])
            color = (255, 0, 0) if name == 'beaker' else (0, 0, 255)
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            cv2.putText(out, f"{name} {confs[i]:.2f}", (x1, max(y1 - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    level = "--" if percentage is None else f"{percentage:.1f}%"
    cv2.putText(out, f"Chocolate Level: {level}", (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
    cv2.putText(out, f"Over {STOP_PERCENT:.0f}%: {count}/{CONFIRM_FRAMES}", (30, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(out, status, (30, 130),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    t_text = "--" if temp is None else f"{temp:.1f}C"
    cv2.putText(out, f"Nozzle: {t_text}  Fan: {'ON' if fan_on else 'OFF'}", (30, 170),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255) if fan_on else (200, 200, 200), 2)
    cv2.putText(out, "G:start  X:home/ready  Q:quit", (30, out.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return out


class CameraStream:
    """OpenCV로 카메라를 계속 찍고 가장 최신 프레임만 보관"""

    def __init__(self, index):
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.frame = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        if not self.cap.isOpened():
            return False
        self.running = True
        self.thread.start()
        return True

    def _loop(self):
        while self.running:
            ok, frame = self.cap.read()
            if ok:
                with self.lock:
                    self.frame = frame
            else:
                time.sleep(0.01)

    def read(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1)
        self.cap.release()


def main():
    if not os.path.exists(MODEL_PATH):
        print(f"모델 파일을 찾을 수 없습니다: {MODEL_PATH}")
        return

    model = YOLO(MODEL_PATH)
    class_names = model.names
    print(f"모델 클래스: {class_names}")

    camera = CameraStream(CAMERA_INDEX)
    if not camera.start():
        print("카메라를 열 수 없습니다.")
        return

    t0 = time.time()
    while camera.read() is None and time.time() - t0 < 5:
        time.sleep(0.05)

    try:
        printer = Printer(SERIAL_PORT, BAUD_RATE)
    except serial.SerialException as ex:
        print(f"프린터에 연결할 수 없습니다: {ex}")
        print("USB 연결과 SERIAL_PORT 설정을 확인하세요. (ls /dev/ttyACM* /dev/ttyUSB*)")
        camera.stop()
        return

    fan = FanController(FAN_GPIO)
    printer.on_temp = fan.update
    monitor = TempMonitor(printer)

    ctrl = Controller(printer)
    ctrl.setup()
    over_count = 0

    try:
        while True:
            frame = camera.read()
            if frame is None:
                time.sleep(0.01)
                continue

            results = model(frame, imgsz=IMG_SIZE, verbose=False)[0]
            percentage, beaker_found = measure_chocolate_level(results, class_names)

            # 압출 중일 때만 자동 정지 판단
            if ctrl.mode == 'RUNNING' and not ctrl.stop_event.is_set():
                if percentage is not None:
                    over_count = over_count + 1 if percentage > STOP_PERCENT else 0
                print(f"비율: {'--' if percentage is None else f'{percentage:.1f}%'} | "
                      f"비커 감지: {beaker_found} | 초과 연속: {over_count}")

                if over_count >= CONFIRM_FRAMES:
                    ctrl.auto_stop(f"초콜릿이 {STOP_PERCENT}%를 넘었습니다.")
                elif ctrl.run_start and time.time() - ctrl.run_start > MAX_RUN_SECONDS:
                    ctrl.auto_stop("최대 압출 시간 초과.")

            view = draw(frame, results, class_names, percentage, over_count,
                        ctrl.status_text(), printer.last_temp, fan.on)
            cv2.imshow("Chocolate Level", view)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('g'), ord('G')):
                if ctrl.press_g():
                    over_count = 0
            elif key in (ord('x'), ord('X')):
                ctrl.press_x()
            elif key in (ord('q'), ord('Q')):
                print("종료합니다.")
                break

    except KeyboardInterrupt:
        print("Ctrl+C 종료")
    finally:
        ctrl.shutdown()
        monitor.stop()
        fan.close()
        printer.close()
        camera.stop()
        cv2.destroyAllWindows()
        print("종료")


if __name__ == '__main__':
    main()
