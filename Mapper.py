class ServoMapper:
    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def base(self, deg):
        return self.clamp(deg, 0, 180)

    def shoulder(self, deg):
        return self.clamp(deg, 0, 180)

    def elbow(self, deg):
        return self.clamp(180 + deg, 50, 180)

    def wrist_pitch(self, deg):
        return self.clamp(90 - deg, 0, 130)

    def wrist_roll(self, deg):
        return self.clamp(deg, 0, 180)
