import logging


class NoDrakeDifferentialIKFilter(logging.Filter):
    def filter(self, record):
        return not record.getMessage().startswith("Differential IK")
