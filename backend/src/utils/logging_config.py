import logging
import sys

def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        handlers=[logging.StreamHandler(sys.stdout)]
    )
    # Set higher logging level for httpx to avoid verbose output
    logging.getLogger("httpx").setLevel(logging.WARNING)

if __name__ == '__main__':
    setup_logging()
    logger = logging.getLogger(__name__)
    logger.info("Logging configured and tested.")
    logger.warning("This is a warning message.")
    logger.error("This is an error message.")