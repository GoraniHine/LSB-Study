"""
YOLO 학습용 사진 촬영
  스페이스바 : 사진 1장 저장
  Q         : 종료
영상 창을 클릭해서 선택한 상태 + 영문 입력 상태에서 누르세요.
"""

import os
import time
import cv2

CAMERA_INDEX = 0
TARGET_COUNT = 40
SAVE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dataset_images')


def next_index(folder):
    """이미 저장된 사진이 있으면 이어서 번호 매기기"""
    nums = []
    for f in os.listdir(folder):
        name, ext = os.path.splitext(f)
        if ext.lower() == '.jpg' and name.startswith('img_') and name[4:].isdigit():
            nums.append(int(name[4:]))
    return max(nums) + 1 if nums else 1


def main():
    os.makedirs(SAVE_DIR, exist_ok=True)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    idx = next_index(SAVE_DIR)
    saved = 0
    flash_until = 0
    print(f"저장 위치: {SAVE_DIR}")
    print(f"스페이스바: 촬영 / Q: 종료 (목표 {TARGET_COUNT}장)")

    while saved < TARGET_COUNT:
        ok, frame = cap.read()
        if not ok:
            print("프레임을 읽지 못했습니다.")
            time.sleep(0.1)
            continue

        view = frame.copy()
        cv2.putText(view, f"{saved}/{TARGET_COUNT}  SPACE: capture  Q: quit",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        if time.time() < flash_until:
            cv2.putText(view, "SAVED", (20, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        cv2.imshow("Capture", view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            path = os.path.join(SAVE_DIR, f"img_{idx:03d}.jpg")
            cv2.imwrite(path, frame)          # 글자 없는 원본 프레임 저장
            print(f"[{saved + 1}/{TARGET_COUNT}] 저장: {os.path.basename(path)}")
            idx += 1
            saved += 1
            flash_until = time.time() + 0.5
        elif key in (ord('q'), ord('Q')):
            break

    print(f"총 {saved}장 저장 완료")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()