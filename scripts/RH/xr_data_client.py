import requests
import time
import json

BASE_URL = "http://127.0.0.1:5000"

def fetch_data(endpoint: str):
    """Fetches data from a given endpoint and returns it as a dictionary."""
    url = f"{BASE_URL}/{endpoint}"
    try:
        response = requests.get(url, timeout=5)
        response.raise_for_status()  # Raise an exception for HTTP errors (4xx or 5xx)
        return response.json()
    except requests.exceptions.Timeout:
        print(f"Timeout when fetching from {url}")
    except requests.exceptions.ConnectionError:
        print(f"Connection error. Is the server running at {BASE_URL}?")
    except requests.exceptions.RequestException as e:
        print(f"Request failed for {url}: {e}")
    except json.JSONDecodeError:
        print(f"Failed to decode JSON from response for {url}")
    return None

def main():
    print(f"Starting XR Data Client. Connecting to {BASE_URL}")
    while True:
        print("\n--- Fetching Data ---")
        
        # Fetch Headset Data
        head_data = fetch_data("head")
        if head_data:
            print(f"Headset Data: {head_data}")
        else:
            print("Headset Data: Not available")

        # Fetch Left Controller Data
        left_data = fetch_data("left")
        if left_data:
            print(f"Left Controller Data: {left_data}")
        else:
            print("Left Controller Data: Not available")

        # Fetch Right Controller Data
        right_data = fetch_data("right")
        if right_data:
            print(f"Right Controller Data: {right_data}")
        else:
            print("Right Controller Data: Not available")
        
        time.sleep(1) # Wait for 1 second before next fetch

if __name__ == "__main__":
    main()