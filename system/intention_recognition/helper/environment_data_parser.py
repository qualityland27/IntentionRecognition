import os
from dotenv import load_dotenv


class EnvironmentDataParser:
    """Environment Data Parser.

      The Environment Data Parser lads all Variables from .env file and stores its
      values within the object.

      """
    def __init__(self):
        load_dotenv()
        # Image Variables
        self.WIDTH = int(os.getenv('WIDTH'))
        self.HEIGHT = int(os.getenv('HEIGHT'))
        self.IS_FLIPPED = str(os.getenv('IS_FLIPPED'))
        self.VISUALIZER = os.getenv('VISUALIZER', 'False').lower() in ('true', '1', 't')

        # Machine Learning Variables
        self.ML_MODEL_NAME = os.getenv('ML_MODEL_NAME')
        self.CLASS_NAMES_ARR = os.getenv('CLASS_NAMES_ARR').split(' ')
        self.ADDITIONAL_FEATURES = os.getenv('ADDITIONAL_FEATURES').split(' ')
        self.SELECTED_FEATURES = os.getenv('SELECTED_FEATURES').split(' ')
        self.PATH_TO_BAG_FILES = os.getenv('PATH_TO_BAG_FILES')
        #self.PATH_TO_BAG_FILES_FOR_EVALUATION = os.getenv('PATH_TO_BAG_FILES_FOR_EVALUATION')
        self.CSV_FILE_NAME = os.getenv('CSV_FILE_NAME')
        self.ROLLING_WINDOW_SIZE = int(os.getenv('ROLLING_WINDOW_SIZE'))

        # Object detection
        self.DETECT_OBJECTS = os.getenv('DETECT_OBJECTS', 'True').lower() in ('true', '1', 't')


        # ROS Variables
        self.MASTER_IP = os.getenv('MASTER_IP')
        self.NODE_NAME = os.getenv('NODE_NAME')
        self.TOPIC_NAME = os.getenv('TOPIC_NAME')
        self.NO_ROLLING_FEATURES = os.getenv('NO_ROLLING_FEATURES').split(' ')

        self.TOPIC_INTENTION_PUBLISHER = os.getenv('TOPIC_INTENTION_PUBLISHER')
        self.TOPIC_WRIST_PUBLISHER = os.getenv('TOPIC_WRIST_PUBLISHER')
        self.TOPIC_OBJECT_PUBLISHER = os.getenv('TOPIC_OBJECT_PUBLISHER')
        self.TOPIC_RECOVERY_GESTURE_PUBLISHER = os.getenv('TOPIC_RECOVERY_GESTURE_PUBLISHER')
