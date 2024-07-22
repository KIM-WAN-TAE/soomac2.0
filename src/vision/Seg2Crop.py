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
        x, y, w, h = cv2.boundingRect(mask)
        print(x, y, w, h)
        # RGB 이미지에서 객체 영역 크롭
        cropped_img = rgb_image[y:y+h, x:x+w]
        cropped_images.append((label, cropped_img))
        print(cropped_img.shape)

    return cropped_images

# 예제 RGB 이미지와 segmask
img_file = '/home/choiyj/catkin_ws/src/soomac/src/vision/uois/example_images/test_image_1.npy'
d = np.load(img_file, allow_pickle=True, encoding='bytes').item()
rgb_image = d['rgb']
segmask = cv2.imread('/home/choiyj/catkin_ws/src/soomac/src/vision/uois/SegMask_6.png', cv2.IMREAD_GRAYSCALE)

# 객체 크롭
cropped_images = extract_objects_from_image(rgb_image, segmask)
# print(cropped_images)
# 결과 확인
for idx, img in enumerate(cropped_images):
    cv2.imwrite(f'cropped_object_{idx}.png', img[1])
