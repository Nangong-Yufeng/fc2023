import cv2

def imshow(*args, **kargs):
    cv2.imshow(*args, **kargs)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def imshow_dict(dict:dict):
    for name, img in dict.items():
        imshow(name, img)
        cv2.waitKey(0)
    cv2.destroyAllWindows()