"""
CHOCO.DRAW 앱 런처
- Flask+SocketIO 서버를 백그라운드 스레드로 띄우고
- pywebview로 주소창/탭 없는 네이티브 창을 열어서 "진짜 앱"처럼 보이게 함
실행:
    python run_app.py
라즈베리파이 OS(Bookworm 기준) 사전 설치:
    sudo apt install python3-gi python3-gi-cairo gir1.2-gtk-3.0 gir1.2-webkit2-4.1
    pip install -r requirements.txt
"""
import threading
import time
import webview
from app import app, socketio  # 기존 Flask 앱 재사용

HOST = "127.0.0.1"
PORT = 5000

# [수정] display_hdmi_rotate 등으로 화면을 돌린 뒤에는 실제 물리 해상도를
# 코드가 잘못 짐작하는 경우가 있어, 필요하면 여기 강제로 값을 지정할 수 있게
# 만들어둠. None으로 두면 webview.screens에서 자동으로 조회한다.
# (자동 조회가 틀리게 나오면 터미널에서 `xrandr`로 실제 해상도 확인 후
#  아래 두 값을 직접 채워 넣으면 됨. 예: FORCE_WIDTH = 800, FORCE_HEIGHT = 1280)
FORCE_WIDTH = None
FORCE_HEIGHT = None


def run_server():
    socketio.run(app, host=HOST, port=PORT, debug=True, use_reloader=False,
                 allow_unsafe_werkzeug=True)


def get_screen_size():
    """실제 모니터 해상도를 조회. FORCE_WIDTH/HEIGHT가 지정돼 있으면 그 값을
    그대로 쓰고, 아니면 webview.screens에서 첫 번째 모니터 크기를 읽어온다.
    둘 다 실패하면 안전한 기본값(1280x800)으로 대체."""
    if FORCE_WIDTH and FORCE_HEIGHT:
        return FORCE_WIDTH, FORCE_HEIGHT
    try:
        screens = webview.screens
        if screens:
            scr = screens[0]
            print(f"[SCREEN] 감지된 해상도: {scr.width}x{scr.height}")
            return scr.width, scr.height
    except Exception as e:
        print(f"[SCREEN] 해상도 조회 실패: {e}")
    print("[SCREEN] 기본값(1280x800) 사용 - 실제와 다르면 FORCE_WIDTH/FORCE_HEIGHT를 직접 지정하세요")
    return 1280, 800


def main():
    t = threading.Thread(target=run_server, daemon=True)
    t.start()
    time.sleep(1.5)  # 서버 뜰 때까지 잠깐 대기

    width, height = get_screen_size()

    # [수정] fullscreen=True만으로는 창관리자(WM)가 없는 kiosk 환경이나
    # 화면 회전 직후에 실제 화면 크기와 어긋나는 경우가 있어서, 대신
    # 조회한 실제 해상도로 frameless 창을 (0,0)에 정확히 맞춰서 띄운다.
    # 이러면 WM의 "전체화면" 처리에 의존하지 않고 항상 화면 전체를 덮는다.
    window = webview.create_window(
        "CHOCO.DRAW",
        f"http://{HOST}:{PORT}",
        width=width,
        height=height,
        x=0,
        y=0,
        frameless=True,      # 타이틀바/테두리 완전 제거
        easy_drag=False,
        confirm_close=False,
        text_select=False,
    )

    def on_loaded():
        # 일부 GTK 백엔드/버전 조합에서 frameless+정확한 크기 지정만으로도
        # 화면을 다 못 덮는 경우가 있어, 콘텐츠 로드 후 fullscreen을 한 번 더
        # 명시적으로 토글해서 확실히 맞춘다. (이미 꽉 차 있으면 아무 변화 없음)
        try:
            window.toggle_fullscreen()
        except Exception as e:
            print(f"[FULLSCREEN] toggle_fullscreen 실패(무시하고 진행): {e}")

    window.events.loaded += on_loaded

    webview.start(debug=False)  # 창 닫히면 프로세스 종료


if __name__ == "__main__":
    main()