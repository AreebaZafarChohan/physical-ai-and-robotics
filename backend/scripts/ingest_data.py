import os
import httpx # A good async HTTP client for Python
import asyncio

# Assuming the backend API will be running at http://localhost:8000
BACKEND_API_URL = os.getenv("BACKEND_API_URL", "http://localhost:8000")

async def ingest_document(chapter: str, source_link: str, paragraph_id: str, text_content: str):
    """
    Ingests a single document chunk into the vector database via the backend API.
    """
    embed_url = f"{BACKEND_API_URL}/embed"
    payload = {
        "text": text_content,
        "chapter": chapter,
        "source_link": source_link,
        "paragraph_id": paragraph_id,
    }
    async with httpx.AsyncClient() as client:
        try:
            response = await client.post(embed_url, json=payload, timeout=60.0)
            response.raise_for_status() # Raise an exception for HTTP errors
            print(f"Successfully ingested {paragraph_id}: {response.json()}")
            return response.json()
        except httpx.HTTPStatusError as e:
            print(f"HTTP error during ingestion of {paragraph_id}: {e}")
        except httpx.RequestError as e:
            print(f"Network error during ingestion of {paragraph_id}: {e}")
        return None

async def batch_ingest_data(data_source_path: str):
    """
    Reads data from a source (e.g., a directory of Markdown files),
    chunks it, and calls ingest_document for each chunk.
    This is a placeholder for the actual data processing logic.
    """
    print(f"Starting batch ingestion from: {data_source_path}")

    # --- Placeholder for actual data loading and chunking logic ---
    # In a real scenario, you'd read files, parse Markdown, chunk text,
    # extract metadata (chapter, source_link, paragraph_id), etc.
    sample_data = [
        {"chapter": "Intro to ROS2", "source_link": "docs/01-module-ros/01-intro-to-ros2.md", "paragraph_id": "ros2-p1", "text_content": "ROS 2 is a flexible framework for writing robot software."},
        {"chapter": "Intro to ROS2", "source_link": "docs/01-module-ros/01-intro-to-ros2.md", "paragraph_id": "ros2-p2", "text_content": "It consists of a set of tools, libraries, and conventions."},
    ]
    # --- End Placeholder ---

    tasks = []
    for item in sample_data:
        tasks.append(ingest_document(
            chapter=item["chapter"],
            source_link=item["source_link"],
            paragraph_id=item["paragraph_id"],
            text_content=item["text_content"]
        ))
    
    await asyncio.gather(*tasks)
    print("Batch ingestion process completed.")

if __name__ == "__main__":
    # Example usage:
    # Set BACKEND_API_URL in your environment or directly here for testing
    # os.environ["BACKEND_API_URL"] = "http://localhost:8000"
    asyncio.run(batch_ingest_data("/path/to/your/book/content"))
