import io
import os

class FileBase:
    TEXT = 0
    BINARY = 1

    def __init__(self):
        self.filefp_ = None
        self.filetype_ = self.TEXT
        self.columns_ = 0

    def close(self):
        if self.filefp_ and not self.filefp_.closed:
            self.filefp_.close()
            self.filefp_ = None

    def isOpen(self):
        return self.filefp_ is not None and not self.filefp_.closed

    def isEof(self):
        if self.filefp_ and not self.filefp_.closed:
            return self.filefp_.tell() == os.fstat(self.filefp_.fileno()).st_size
        return True

    def fstream(self):
        return self.filefp_

    def columns(self):
        return self.columns_

# Example usage:
# if __name__ == "__main__":
#     fb = FileBase()
#     # Open a file (example.txt in read mode)
#     fb.file = open('dataset/truth.nav', 'r')
#     print(fb.isOpen())  # Check if file is open
#     print(fb.isEof())   # Check if EOF is reached
#     fb.close()           # Close the file
