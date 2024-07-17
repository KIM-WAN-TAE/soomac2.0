import numpy as np

k = np.load("/home/choiyj/catkin_ws/src/soomac/src/vision/uois/example_images/OCID_image_0.npy", allow_pickle=True)
print(k.shape)
print(k.item().get('label'))