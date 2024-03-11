import requests
import cv2
import json


def predict_image(image):
    img_str = cv2.imencode(".png", image)[1].tobytes()
    # img = open(f"datasets/train/images/{number}.png", "rb")
    files = {"image": img_str}
    r = requests.post("https://regolonet.mgteam.one/predict", files=files, timeout=3)
    if r.ok:
        data = json.loads(r.text)
        return True, data
    return False, None


if __name__ == "__main__":
    number = int(input("Numero immagine: "))
    image = cv2.imread(f"datasets/train/images/{number}.png")
    success, data = predict_image(image)
    print(success)
    print(data)
