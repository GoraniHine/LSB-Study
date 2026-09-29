import cv2
import glob
import os
from ultralytics import YOLO

def test_single_images():
    model_path = 'runs/segment/train/weights/best.pt'
    if not os.path.exists(model_path):
        print(f"모델 파일을 찾을 수 없습니다: {model_path}")
        return
        
    model = YOLO(model_path)
    class_names = model.names
    print(f"모델 클래스 매핑 정보: {class_names}")

    test_image_paths = glob.glob('chocolate_dataset/val/images/*.jpg')
    if not test_image_paths:
        test_image_paths = glob.glob('chocolate_dataset/train/images/*.jpg')
    
    if not test_image_paths:
        print("테스트할 이미지를 찾지 못했습니다. 이미지 경로를 확인해주세요.")
        return

    print(f"총 {len(test_image_paths)}장의 이미지를 테스트합니다. 종료하려면 'q'를 누르세요.")

    for img_path in test_image_paths:
        image = cv2.imread(img_path)
        if image is None:
            continue

        results = model(image, verbose=False)[0]
        
        beaker_box = None
        chocolate_mask_y_coords = []
        percentage = 0.0

        if results.boxes is not None:
            boxes = results.boxes.xyxy.cpu().numpy()
            classes = results.boxes.cls.cpu().numpy()
            confs = results.boxes.conf.cpu().numpy()
            
            # 1. 박스 정보에서 비커 찾기
            for i, cls_id in enumerate(classes):
                cls_id = int(cls_id)
                current_class_name = class_names.get(cls_id, str(cls_id))
                
                if current_class_name == 'beaker':
                    x1, y1, x2, y2 = boxes[i]
                    beaker_box = (y1, y2)
                    break

        # 2. 마스크 정보에서 초콜릿 Y 좌표 찾기
        if results.masks is not None:
            masks = results.masks.xy
            classes = results.boxes.cls.cpu().numpy()
            
            for i, cls_id in enumerate(classes):
                cls_id = int(cls_id)
                current_class_name = class_names.get(cls_id, str(cls_id))
                
                if current_class_name == 'chocolate' and i < len(masks):
                    polygon = masks[i]
                    if len(polygon) > 0:
                        y_coords = polygon[:, 1]
                        chocolate_mask_y_coords.extend(y_coords)

        # 3. 비율 계산 로직
        if beaker_box is not None and len(chocolate_mask_y_coords) > 0:
            beaker_top, beaker_bottom = beaker_box
            beaker_height = beaker_bottom - beaker_top

            choco_top = min(chocolate_mask_y_coords)
            filled_height = beaker_bottom - choco_top

            if filled_height < 0: filled_height = 0
            if filled_height > beaker_height: filled_height = beaker_height

            if beaker_height > 0:
                percentage = (filled_height / beaker_height) * 100

        # 4. 시각화 (OpenCV로 깔끔하게 박스만 그리기)
        annotated_frame = image.copy()

        if results.boxes is not None:
            boxes = results.boxes.xyxy.cpu().numpy()
            classes = results.boxes.cls.cpu().numpy()
            confs = results.boxes.conf.cpu().numpy()

            for i, cls_id in enumerate(classes):
                cls_id = int(cls_id)
                c_name = class_names.get(cls_id, str(cls_id))
                x1, y1, x2, y2 = map(int, boxes[i][:4])
                conf = confs[i]

                if c_name == 'beaker':
                    color = (255, 0, 0)   # 비커: 파란색 (Blue)
                else:
                    color = (0, 0, 255)   # 초콜릿: 빨간색 (Red)

                # 박스 및 글자 출력
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(annotated_frame, f"{c_name} {conf:.2f}", (x1, max(y1 - 10, 20)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 상단에 초콜릿 비율 텍스트 표시
        text = f"Chocolate Level: {percentage:.1f}%"
        cv2.putText(annotated_frame, text, (30, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

        print(f"[{os.path.basename(img_path)}] - 측정 비율: {percentage:.1f}% | 비커 감지: {beaker_box is not None} | 초콜릿 점 개수: {len(chocolate_mask_y_coords)}")

        cv2.imshow("Image Chocolate Measurement", annotated_frame)
        
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    test_single_images()
