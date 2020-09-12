import logging
import os
from datetime import datetime


def get_logger(name, level=None, filename=None):
    if not os.path.exists('errorLogs'):
        os.makedirs('errorLogs')
    logger = logging.getLogger(__name__)
    if filename == None:
        filename = "errorLogs/logFile_" + datetime.now().strftime('%d%m%Y') + ".log"
    if level == None:
        level = logging.DEBUG
    logFormatter = logging.Formatter("%(asctime)s [%(levelname)s] %(process)s (%(threadName)-9s) %(name)s: %(message)s")
    logger = logging.getLogger(name)
    fileHandler = logging.FileHandler(filename)
    consoleHandler = logging.StreamHandler()
    fileHandler.setFormatter(logFormatter)
    logger.addHandler(fileHandler)
    consoleHandler.setFormatter(logFormatter)
    logger.addHandler(consoleHandler)
    logger.setLevel(level)
    return logger
