import logging
import os
import sys


class Logger:
    """
    A logger class that logs to a file and also logs uncaught exceptions.
    """

    def __init__(self, name, log_file, level=logging.INFO, mode='w'):
        """
        Initialize the logger with a specific name and log file.

        :param name: Name of the logger.
        :param log_file: Path to the log file.
        :param level: Logging level (default is logging.INFO).
        :param mode: File mode for the log file (default is 'w').
        """
        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)

        # Get the folder from the log_file path
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
        """
        Log an info level message.

        :param message: The message to log.
        """
        self.logger.info(message)

    def debug(self, message):
        """
        Log a debug level message.

        :param message: The message to log.
        """
        self.logger.debug(message)

    def warning(self, message):
        """
        Log a warning level message.

        :param message: The message to log.
        """
        self.logger.warning(message)

    def error(self, message):
        """
        Log an error level message.

        :param message: The message to log.
        """
        self.logger.error(message)

    def log_uncaught_exceptions(self, exc_type, exc_value, exc_traceback):
        """
        Log uncaught exceptions.

        :param exc_type: Exception type.
        :param exc_value: Exception value.
        :param exc_traceback: Exception traceback.
        """
        if issubclass(exc_type, KeyboardInterrupt):
            # Call the default excepthook for KeyboardInterrupt
            sys.__excepthook__(exc_type, exc_value, exc_traceback)
            return

        self.logger.error("Uncaught exception", exc_info=(exc_type, exc_value, exc_traceback))

# Example usage
# logger_instance = Logger('my_logger', 'central_log.log')
# logger_instance.logger.info('This is an info message')
