import logging
import sys
import traceback
import os

class Logger:
    """
    A logger class that logs to a file and also logs uncaught exceptions.
    """
    def __init__(self, name, log_file, level=logging.INFO, mode='w'):
        """Initialize the logger with a specific name and log file."""
        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)

        # get the folder from the log_file path
        log_file_folder = os.path.dirname(log_file)
        if log_file_folder and not os.path.exists(log_file_folder):
            os.makedirs(log_file_folder, exist_ok=True)

        # Create a file handler
        file_handler = logging.FileHandler(log_file, mode=mode)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)

        # Add the file handler to the logger
        self.logger.addHandler(file_handler)

        # Set up exception hook to log uncaught exceptions
        sys.excepthook = self.log_uncaught_exceptions

    def info(self, message):
        self.logger.info(message)

    def debug(self, message):
        self.logger.debug(message)

    def warning(self, message):
        self.logger.warning(message)

    def error(self, message):
        self.logger.error(message)

    def log_uncaught_exceptions(self, exc_type, exc_value, exc_traceback):
        """Log uncaught exceptions."""
        if issubclass(exc_type, KeyboardInterrupt):
            # Call the default excepthook for KeyboardInterrupt
            sys.__excepthook__(exc_type, exc_value, exc_traceback)
            return

        self.logger.error("Uncaught exception", exc_info=(exc_type, exc_value, exc_traceback))

# Example usage
# logger_instance = Logger('my_logger', 'central_log.log')
# logger_instance.logger.info('This is an info message')
