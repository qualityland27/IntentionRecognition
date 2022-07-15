import logging
from colorlog import ColoredFormatter


class CustomFormatter():
    def __init__(self):
        pass

    def customFormatter(self):
        LOG_LEVEL = logging.DEBUG
        LOGFORMAT = "  %(log_color)s%(levelname)-8s%(reset)s | %(log_color)s%(message)s%(reset)s"

        logging.root.setLevel(LOG_LEVEL)
        formatter = ColoredFormatter(LOGFORMAT)
        stream = logging.StreamHandler()
        stream.setLevel(LOG_LEVEL)
        stream.setFormatter(formatter)
        log = logging.getLogger('intention_recognition')
        log.setLevel(LOG_LEVEL)
        log.addHandler(stream)

        # log.debug("A quirky message only developers care about")
        # log.info("Curious users might want to know this")
        # log.warn("Something is wrong and any user should be informed")
        # log.error("Serious stuff, this is red for a reason")
        # log.critical("OH NO everything is on fire")

        return log