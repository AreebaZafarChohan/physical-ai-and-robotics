import requests
import json

# Test the login endpoint with sample data
login_data = {
    "email": "test@example.com",
    "password": "testpassword"
}

try:
    response = requests.post("http://localhost:9000/auth/login", json=login_data)
    print(f"Status Code: {response.status_code}")
    print(f"Response: {response.text}")
    print(f"Headers: {dict(response.headers)}")
except Exception as e:
    print(f"Error making request: {e}")