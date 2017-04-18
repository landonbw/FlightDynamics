import numpy as np

def compliment(angle):
    r = -np.sign(angle) * (abs(angle) - 2 * np.pi)
    return r

if __name__ == "__main__":
    while True:
        ang = float(input("give me an angle"))
        print(compliment(ang))
        print(2*np.pi)
        print('\n\n\n')