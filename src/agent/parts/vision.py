import os, random
from interaction.face.detector import AgeGenderDetector


class Vision():
    def __init__(self):
        self.scanner = AgeGenderDetector()
        self.dir_path = "src/interaction/face/images"

    def scan(self):
        image = v.getImage
        ppl = self.scanner.get_age_predictions(image)
        return ppl

    def getImage(self):
        image = random.choice(os.listdir(self.dir_path))


if __name__ == "__main__":
    v = Vision()
    v.scanner.display_img("test", v.getImage)