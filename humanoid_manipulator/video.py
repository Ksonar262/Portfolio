import cv2
import urllib.request
import numpy as np


def retrieve_camera_image(url):
    """
    Retrieve the camera image from the specified URL.

    Args:
        url (str): The URL to retrieve the camera image from.

    Returns:
        numpy.ndarray: The retrieved camera image as a NumPy array.
    """
    try:
        # Retrieve the image from the URL
        resp = urllib.request.urlopen(url)
        img = np.asarray(bytearray(resp.read()), dtype=np.uint8)
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)

        return img

    except Exception as e:
        print("Error retrieving camera image:", e)
        return None


def display_camera_image(img):
    """
    Display the camera image.

    Args:
        img (numpy.ndarray): The camera image as a NumPy array.
    """
    cv2.imshow("EZ-B Camera Image", img)


if __name__ == '__main__':
    # URL of the EZ-B camera image 
    url = "http://192.168.0.11/CameraImage.jpg?c=Camera&Password=admin"

    while True:
        try:
            # Retrieve the camera image
            img = retrieve_camera_image(url)

            if img is not None:
                # Display the camera image
                display_camera_image(img)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except KeyboardInterrupt:
            break
        except Exception as e:
            print("Error:", e)
            continue

    cv2.destroyAllWindows()
