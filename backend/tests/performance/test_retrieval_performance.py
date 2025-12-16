from locust import HttpUser, task, between
import random

class RetrievalUser(HttpUser):
    wait_time = between(1, 2.5)  # Users wait between 1 and 2.5 seconds between tasks

    # Host should be configured when running Locust, e.g., locust -H http://localhost:8000

    @task
    def retrieve_chunks(self):
        queries = [
            "What is ROS 2?",
            "How does Gazebo simulate physics?",
            "Explain NVIDIA Isaac Sim features.",
            "Cognitive planning in LLMs for robotics.",
            "What are the ethical implications of AI in robotics?",
            "How to integrate URDF models?",
            "Sensor simulation techniques in digital twins.",
            "Path planning algorithms for autonomous navigation.",
            "Voice-to-action systems in robotics.",
            "Reinforcement learning applications in robotics."
        ]
        selected_query = random.choice(queries)
        
        headers = {'Content-Type': 'application/json'}
        json_data = {
            "query": selected_query,
            "top_k": random.randint(1, 10)
        }
        self.client.post("/retrieve", headers=headers, json=json_data)

