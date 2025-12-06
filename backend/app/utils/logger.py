import logging
import os

# Configure basic logging
logging.basicConfig(
    level=os.getenv("LOG_LEVEL", "INFO").upper(),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler()
    ]
)

def get_logger(name: str):
    """
    Returns a logger instance with the given name.
    """
    return logging.getLogger(name)

# Example usage:
# logger = get_logger(__name__)
# logger.info("This is an info message.")
# logger.error("This is an error message.")
