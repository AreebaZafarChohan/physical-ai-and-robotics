import logging
import sys
import os # Added
from loguru import logger

class InterceptHandler(logging.Handler):
    def emit(self, record):
        # Get corresponding Loguru level if it exists
        try:
            level = logger.level(record.levelname).name
        except ValueError:
            level = record.levelno

        # Find caller from where originated the logged message
        frame, depth = logging.currentframe(), 2
        while frame.f_code.co_filename == logging.__file__:
            frame = frame.f_back
            depth += 1

        logger.opt(depth=depth, exception=record.exc_info).log(level, record.getMessage())

def setup_logging():
    # Remove default handlers
    logger.remove()
    # Add our custom handler
    logger.add(
        sys.stderr,
        level=os.getenv("LOG_LEVEL", "INFO"), # Use LOG_LEVEL from env, default to INFO
        format="{time} {level} {message}",
        colorize=True,
        backtrace=True,
        diagnose=True,
    )
    # Intercept everything at the root logger
    logging.basicConfig(handlers=[InterceptHandler()], level=0, force=True)

# Call setup_logging when this module is imported
setup_logging()
