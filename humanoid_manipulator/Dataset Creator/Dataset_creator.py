import os
import requests
import keyboard
import time

# Constants
URL = "http://192.168.0.11/CameraImage.jpg?password=admin&c=Camera"
DATASET_DIR = "Dataset"

def ensure_dataset_directory(directory):
    """
    Ensure the Dataset directory exists. If not, create it.

    Args:
    - directory (str): The directory path to ensure its existence.

    Returns:
    - None
    """
    if not os.path.exists(directory):
        os.makedirs(directory)

def save_image_from_url(url, save_dir):
    """
    Fetch image from the provided URL and save it in the given directory.

    Args:
    - url (str): The URL from where to fetch the image.
    - save_dir (str): The directory where the image should be saved.

    Returns:
    - None
    """
    try:
        response = requests.get(url, stream=True)
        response.raise_for_status()

        # Create a timestamp for unique filename
        timestamp = int(time.time())
        filepath = os.path.join(save_dir, f"{timestamp}.jpg")

        with open(filepath, 'wb') as file:
            for chunk in response.iter_content(chunk_size=8192):
                file.write(chunk)

        print(f"Image saved to {filepath}")

    except requests.ConnectionError:
        print("Error: Unable to connect to the camera's HTTP server. Check if the camera is online and accessible.")
    except requests.RequestException as e:
        print(f"Error fetching image: {e}")

def main():
    """
    Main execution loop. Listens for key presses.

    When the spacebar is pressed, the script fetches an image from the EZ-Robot camera's local HTTP server and saves it to the 'Dataset' directory.
    To exit the script, press the 'esc' key.

    Returns:
    - None
    """
    print("Press the spacebar to fetch an image or press 'esc' to exit...")

    while True:
        if keyboard.is_pressed('space'):
            save_image_from_url(URL, DATASET_DIR)
            # Delay to avoid multiple captures on a single press
            time.sleep(0.5)
        elif keyboard.is_pressed('esc'):
            print("Exiting...")
            break

if __name__ == "__main__":
    # Ensure the Dataset directory exists
    ensure_dataset_directory(DATASET_DIR)
    main()
