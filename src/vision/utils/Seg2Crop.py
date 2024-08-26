import cv2
import numpy as np

def extract_objects_from_image(rgb_image, segmask):
    # 고유 라벨 값을 추출 (배경을 제외한)
    unique_labels = np.unique(segmask)
    unique_labels = unique_labels[unique_labels != 0]  # 배경 라벨(0)을 제외

    cropped_images = []
    
    for label in unique_labels:
        # 현재 라벨에 해당하는 마스크 생성
        mask = np.where(segmask == label, 255, 0).astype(np.uint8)
        
        # 객체의 바운딩 박스 추출
        offset = 10
        x, y, w, h = cv2.boundingRect(mask)
        print(x, y, w, h)
        # RGB 이미지에서 객체 영역 크롭
        x0 = max(0, x-offset)
        y0 = max(0, y-offset)
        x1 = min(x+w+offset, 640)
        y1 = min(y+h+offset, 480)
        cropped_img = rgb_image[y0:y1, x0:x1]
        cropped_img = add_padding(cropped_img, [100, 100])
        cropped_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2RGB)
        cropped_images.append(([x, y], cropped_img))
        print(cropped_img.shape)

    return cropped_images

def add_padding(image, target_size):
    h, w, _ = image.shape
    target_h, target_w = target_size
    scale = min(target_w / w, target_h / h)
    new_w = int(w * scale)
    new_h = int(h * scale)
    resized_image = cv2.resize(image, (new_w, new_h))
    
    pad_w = (target_w - new_w) // 2
    pad_h = (target_h - new_h) // 2
    
    padded_image = cv2.copyMakeBorder(
        resized_image, 
        pad_h, target_h - new_h - pad_h, 
        pad_w, target_w - new_w - pad_w, 
        cv2.BORDER_CONSTANT, 
        value=[0, 0, 0]
    )
    return padded_image


# 예제 RGB 이미지와 segmask
img_file = '/home/choiyj/catkin_ws/src/soomac/src/vision/uois/example_images/test_image_19.npy'
d = np.load(img_file, allow_pickle=True, encoding='bytes').item()
rgb_image = d['rgb']
segmask = cv2.imread('/home/choiyj/catkin_ws/src/soomac/src/vision/uois/SegMask_6.png', cv2.IMREAD_GRAYSCALE)

# 객체 크롭
cropped_images = extract_objects_from_image(rgb_image, segmask)
# print(cropped_images)
# 결과 확인
for idx, img in enumerate(cropped_images):
    cv2.imwrite(f'/home/choiyj/catkin_ws/src/soomac/src/vision/a/cropped/cropped_object_{idx}.png', img[1])
