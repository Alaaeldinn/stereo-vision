import requests
import cv2
import numpy as np

class ESP32CameraManager:
    def __init__(self, url_left: str, url_right: str):
        self.URL_left = url_left
        self.URL_right = url_right
        self.AWB = True
        self.cnt = 1

    def set_resolution(self, url: str, index: int = 1, verbose: bool = False):
        try:
            if verbose:
                resolutions = "10: UXGA(1600x1200)\n9: SXGA(1280x1024)\n8: XGA(1024x768)\n7: SVGA(800x600)\n6: VGA(640x480)\n5: CIF(400x296)\n4: QVGA(320x240)\n3: HQVGA(240x176)\n0: QQVGA(160x120)"
                print("Available resolutions:\n{}".format(resolutions))
            if index in [10, 9, 8, 7, 6, 5, 4, 3, 0]:
                requests.get(url + "/control?var=framesize&val={}".format(index))
            else:
                print("Wrong index")
        except Exception as e:
            print(f"SET_RESOLUTION: Something went wrong - {str(e)}")

    def set_quality(self, url: str, value: int = 1, verbose: bool = False):
        try:
            if 10 <= value <= 63:
                requests.get(url + "/control?var=quality&val={}".format(value))
            else:
                print("Quality value should be between 10 and 63")
        except Exception as e:
            print(f"SET_QUALITY: Something went wrong - {str(e)}")

    def set_awb(self, url: str):
        try:
            self.AWB = not self.AWB
            requests.get(url + "/control?var=awb&val={}".format(1 if self.AWB else 0))
        except Exception as e:
            print(f"SET_AWB: Something went wrong - {str(e)}")
        return self.AWB

    def get_image(self, url: str):
        try:
            response = requests.get(url + "/capture")
            if response.status_code == 200:
                image_array = np.frombuffer(response.content, dtype=np.uint8)
                image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                return image
            else:
                print(f"Failed to get image from {url}. Status code: {response.status_code}")
                return None
        except Exception as e:
            print(f"GET_IMAGE: Something went wrong - {str(e)}")
            return None

    def get_stereo_images(self):
        left_image = self.get_image(self.URL_left)
        right_image = self.get_image(self.URL_right)
        return left_image, right_image

    def configure_cameras(self, resolution_index: int = 6, quality: int = 10):
        for url in [self.URL_left, self.URL_right]:
            self.set_resolution(url, resolution_index)
            self.set_quality(url, quality)
            self.set_awb(url)

# Example usage:
if __name__ == "__main__":
    URL_left = "http://192.168.1.121"
    URL_right = "http://192.168.1.129"
    
    camera_manager = ESP32CameraManager(URL_left, URL_right)
    
    # Configure both cameras
    camera_manager.configure_cameras(resolution_index=6, quality=10)

    # Get stereo images
    left_img, right_img = camera_manager.get_stereo_images()
    