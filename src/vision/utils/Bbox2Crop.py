import os
import cv2
import glob

# 이미지와 라벨 경로 설정
image_path = '/home/choiyoonji/Downloads/e/train/images'  # 이미지가 있는 폴더 경로
label_path = '/home/choiyoonji/Downloads/e/train/labels'  # 라벨이 있는 폴더 경로
output_path = '/home/choiyoonji/catkin_ws/src/soomac/src/vision/siamese_network/data'  # 크롭된 이미지가 저장될 폴더 경로


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


# 출력 폴더가 없으면 생성
if not os.path.exists(output_path):
    os.makedirs(output_path)

# 라벨 파일들을 불러오기
label_files = glob.glob(os.path.join(label_path, '*.txt'))

for label_file in label_files:
    # 각 라벨 파일에 대해 이미지 이름 얻기
    with open(label_file, 'r') as file:
        lines = file.readlines()
    
    # 이미지 파일 경로 생성
    image_file = os.path.join(image_path, os.path.basename(label_file).replace('.txt', '.jpg'))
    
    # 이미지 로드
    image = cv2.imread(image_file)
    h, w, _ = image.shape
    
    for i, line in enumerate(lines):
        # 라벨 파일의 각 줄에 대해 처리
        data = line.strip().split()
        label = data[0]
        x_center, y_center, width, height = map(float, data[1:])
        
        # 크롭 영역 계산
        x1 = int((x_center - width / 2) * w)
        y1 = int((y_center - height / 2) * h)
        x2 = int((x_center + width / 2) * w)
        y2 = int((y_center + height / 2) * h)
        
        offset = 10
        x1 = max(0, x1-offset)
        y1 = max(0, y1-offset)
        x2 = min(x2+offset, 640)
        y2 = min(y2+offset, 480)
        print(x1, y1, x2, y2)

        # 객체 크롭
        cropped_image = image[y1:y2, x1:x2]
        if cropped_image.shape[0] <= 0 or cropped_image.shape[1] <= 0:
            print("size 0")
            continue
        cropped_image = add_padding(cropped_image, [100, 100])
        # cropped_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2RGB)
        
        # 라벨별 폴더 생성
        label_folder = os.path.join(output_path, label)
        if not os.path.exists(label_folder):
            os.makedirs(label_folder)
        
        # 크롭된 이미지 저장
        crop_filename = os.path.join(label_folder, f'{os.path.basename(label_file).replace(".txt", "")}_{i}.jpg')
        cv2.imwrite(crop_filename, cropped_image)
