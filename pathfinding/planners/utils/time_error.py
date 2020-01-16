class OutOfTimeError(Exception):
    def __init__(self, msg=None):
        self.value = msg

    def __str__(self):
        return repr(self.value)
