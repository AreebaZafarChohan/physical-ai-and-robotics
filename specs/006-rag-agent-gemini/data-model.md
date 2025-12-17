# Data Model for RAG Agent with Gemini Backend

## Entities

### RAG Agent
- **Description**: The core orchestrator responsible for processing user queries, invoking the retrieval tool, and generating grounded responses using the Gemini model.
- **Attributes**:
    - `query`: (Input) User's natural language question (string).
    - `response`: (Output) Agent's generated answer (string), or "I don't know" if no content found, or a generic error message if retrieval fails.
    - `retrieved_content`: (Internal) List of text chunks retrieved from Qdrant (list of strings).
    - `model_name`: (Configuration) Identifier for the Gemini model used (string, e.g., "gemini-pro").
    - `max_tokens`: (Configuration) Maximum tokens for model response (integer).
    - `temperature`: (Configuration) Model's creativity setting (float).

### Retrieval Tool
- **Description**: A component that interacts with the Qdrant vector database to find relevant information.
- **Attributes**:
    - `query_embedding`: (Input) Vector representation of the user's query (list of floats).
    - `qdrant_collection_name`: (Configuration) Name of the Qdrant collection to query (string).
    - `top_k`: (Configuration) Number of top relevant results to retrieve (integer).
    - `search_results`: (Output) List of relevant text chunks from Qdrant (list of strings).
    - `cohere_api_key`: (Configuration) API key for Cohere embeddings (string).

### Gemini Model (via OpenAI-compatible API)
- **Description**: The large language model used for generating responses.
- **Attributes**:
    - `input_prompt`: (Input) Constructed prompt including user query and retrieved context (string).
    - `model_output`: (Output) Raw text response from the Gemini model (string).
    - `api_key`: (Configuration) Gemini API key (string).
    - `base_url`: (Configuration) URL of the OpenAI-compatible Gemini API endpoint (string).

### FastAPI Endpoint
- **Description**: The external interface for interacting with the RAG Agent.
- **Attributes**:
    - `user_query`: (Input) JSON payload containing the user's question (string).
    - `agent_response`: (Output) JSON payload containing the agent's answer (string).
    - `status_code`: (Output) HTTP status code (integer).
    - `error_message`: (Output, optional) Error message in case of failure (string).