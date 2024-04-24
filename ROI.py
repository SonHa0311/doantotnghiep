import cv2


def apply_roi(img, roi):
    # resize ROI to match the original image size
    roi = cv2.resize(src=roi, dsize=(img.shape[1], img.shape[0]))

    assert img.shape[:2] == roi.shape[:2]

    # scale ROI to [0, 1] => binary mask
    thresh, roi = cv2.threshold(roi, thresh=128, maxval=1, type=cv2.THRESH_BINARY)

    # apply ROI on the original image
    #new_img = cv2.bitwise_and(img, roi, mask= None)
    new_img= img*roi

    return new_img


#if __name__ == "__main__":
    #assert len(sys.argv) == 3, '[USAGE] $ python opencv_ROI.py input_image roi_image'
   #input_image_path, roi_image_path = sys.argv[1], sys.argv[2]
'''input_image_path = cv2.VideoCapture("cavani.mp4")
while True:
    ret, img = input_image_path.read()
    roi_image_path = "ROI.jpg"
    #assert os.path.isfile(input_image_path), 'Image not found @ %s' % input_image_path
    #assert os.path.isfile(roi_image_path), 'ROI not found @ %s' % roi_image_path

    #img = cv2.imread(input_image_path)  # shape: (588, 586, 3)
    roi = cv2.imread(roi_image_path)  # shape: (300, 300, 3)

    new_img = apply_roi(img, roi)

    #cv2.imwrite('new_%s' % os.path.basename(input_image_path), new_img)
    cv2.imshow("z",new_img)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
cv2.destroyAllWindows()'''